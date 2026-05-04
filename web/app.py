#!/usr/bin/env python3
"""
Web control panel for NovaROS drone.
Proxies API calls to FastAPI backend with server-side timestamps.
"""

import time
import requests as http
from flask import Flask, render_template, request, jsonify, Response

app = Flask(__name__)
app.config["TEMPLATES_AUTO_RELOAD"] = True
app.jinja_env.auto_reload = True
API = "http://localhost:8080"
VISION = "http://localhost:8081"


@app.route("/")
def index():
    return render_template("index.html")


def _stream_proxy(upstream_url: str):
    """Shared streaming proxy for MJPEG feeds.

    chunk_size=None: yield bytes as they arrive from the upstream socket,
    no batching. direct_passthrough: bypass Flask's WSGI output buffer so
    each chunk hits the browser the moment we yield it. Together these
    keep the stream latency equal to one network hop + one frame time."""
    try:
        r = http.get(upstream_url, stream=True, timeout=30)
        resp = Response(
            r.iter_content(chunk_size=None),
            content_type=r.headers.get("Content-Type",
                                       "multipart/x-mixed-replace; boundary=frame"),
            direct_passthrough=True,
        )
        return resp
    except http.exceptions.RequestException as e:
        return {"error": str(e)}, 502


def _snapshot_proxy(upstream_url: str):
    try:
        r = http.get(upstream_url, timeout=5)
        return Response(r.content, content_type=r.headers.get("Content-Type", "image/jpeg")), r.status_code
    except http.exceptions.RequestException as e:
        return {"error": str(e)}, 502


@app.route("/api/camera/stream")
def camera_stream():
    """Legacy alias — streams the landing camera."""
    return _stream_proxy(f"{API}/camera/stream")


@app.route("/api/camera/snapshot")
def camera_snapshot():
    """Legacy alias — snapshot of the landing camera."""
    return _snapshot_proxy(f"{API}/camera/snapshot")


@app.route("/api/camera/<name>/stream")
def camera_named_stream(name):
    """MJPEG streaming proxy for a named camera (landing / tracking)."""
    return _stream_proxy(f"{API}/camera/{name}/stream")


@app.route("/api/camera/<name>/snapshot")
def camera_named_snapshot(name):
    """Single JPEG for a named camera."""
    return _snapshot_proxy(f"{API}/camera/{name}/snapshot")


# --- Vision (object detector) proxy -----------------------------------------
# detector.py runs in vision-detect container on :8081 with its own embedded
# HTML at /. We expose it under /vision so the browser stays on web-control's
# port and CORS / cookies stay simple. The embedded PAGE uses absolute paths
# (/frame /state /lock); we rewrite them to /vision/... when serving root.

@app.route("/vision")
@app.route("/vision/")
def vision_root():
    try:
        r = http.get(f"{VISION}/", timeout=5)
        html = r.text.replace('"/frame"', '"/vision/frame"') \
                     .replace("'/frame'", "'/vision/frame'") \
                     .replace('"/state"', '"/vision/state"') \
                     .replace("'/state'", "'/vision/state'") \
                     .replace('"/lock"', '"/vision/lock"') \
                     .replace("'/lock'", "'/vision/lock'")
        return Response(html, content_type=r.headers.get("Content-Type", "text/html"))
    except http.exceptions.RequestException as e:
        return f"Vision unavailable: {e}", 502


@app.route("/vision/frame")
def vision_frame():
    """Streaming-friendly proxy for the latest annotated JPEG frame."""
    try:
        r = http.get(f"{VISION}/frame", timeout=5)
        return Response(r.content, content_type=r.headers.get("Content-Type", "image/jpeg")), r.status_code
    except http.exceptions.RequestException as e:
        return {"error": str(e)}, 502


@app.route("/vision/state")
def vision_state():
    try:
        r = http.get(f"{VISION}/state", timeout=5)
        return Response(r.content, content_type=r.headers.get("Content-Type", "application/json")), r.status_code
    except http.exceptions.RequestException as e:
        return {"error": str(e)}, 502


# --- VTX broadcast (source-switching layer for the analog VTX feed) ----
# Generic /api/<path> proxy below handles /api/vtx/source GET+POST cleanly,
# but the MJPEG stream needs the streaming-passthrough proxy and the
# snapshot needs the binary-passthrough proxy — same pattern as cameras.

@app.route("/api/vtx/snapshot")
def vtx_snapshot():
    return _snapshot_proxy(f"{API}/vtx/snapshot")


@app.route("/api/vtx/stream")
def vtx_stream():
    return _stream_proxy(f"{API}/vtx/stream")


@app.route("/vision/lock", methods=["POST"])
def vision_lock():
    try:
        r = http.post(
            f"{VISION}/lock",
            data=request.get_data(),
            headers={"Content-Type": "application/json"},
            timeout=5,
        )
        return Response(r.content, content_type=r.headers.get("Content-Type", "application/json")), r.status_code
    except http.exceptions.RequestException as e:
        return {"error": str(e)}, 502


@app.route("/api/<path:path>", methods=["GET", "POST", "DELETE"])
def proxy(path):
    url = f"{API}/{path}"
    try:
        if request.method == "GET":
            r = http.get(url, timeout=5)
        elif request.method == "POST":
            data = request.get_json(silent=True)
            # Inject server timestamp — eliminates client clock skew
            if path == "control/command" and data:
                data["timestamp"] = time.time()
            # Calibrations can block up to ~20s on the FC
            post_timeout = 30 if path.startswith("calibration/") else 5
            r = http.post(url, json=data, timeout=post_timeout)
        elif request.method == "DELETE":
            r = http.delete(url, timeout=5)
        return r.json(), r.status_code
    except http.exceptions.RequestException as e:
        return {"error": str(e)}, 502


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
