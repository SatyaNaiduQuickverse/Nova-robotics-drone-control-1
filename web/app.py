#!/usr/bin/env python3
"""
Web control panel for NovaROS drone.
Proxies API calls to FastAPI backend with server-side timestamps.
"""

import time
import requests as http
from flask import Flask, render_template, request, jsonify

app = Flask(__name__)
API = "http://localhost:8080"


@app.route("/")
def index():
    return render_template("index.html")


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
            r = http.post(url, json=data, timeout=5)
        elif request.method == "DELETE":
            r = http.delete(url, timeout=5)
        return r.json(), r.status_code
    except http.exceptions.RequestException as e:
        return {"error": str(e)}, 502


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
