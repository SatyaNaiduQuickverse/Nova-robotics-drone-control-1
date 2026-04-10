#!/usr/bin/env python3
"""
Camera + audio test server for RPi 5.
Single muxed WebM stream (VP8 + Opus) for synced audio/video.
Open http://<pi-ip>:9090 in your browser.
Ctrl+C to stop.
"""

import subprocess
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler

VIDEO_DEVICE = "/dev/video0"
AUDIO_DEVICE = "hw:2,0"
PORT = 9090

HTML = """<!DOCTYPE html>
<html>
<head>
  <title>Camera Test</title>
  <style>
    body { background:#111; color:#eee; font-family:monospace; text-align:center; margin:2rem; }
    video { border:2px solid #444; border-radius:8px; max-width:90vw; }
    .ok { color:#0f0; }
    .dim { color:#888; font-size:0.8em; }
    button { padding:0.5rem 1.5rem; font-size:1rem; cursor:pointer; margin:1rem;
             background:#333; color:#eee; border:1px solid #666; border-radius:4px; }
  </style>
</head>
<body>
  <h1>NovaROS Camera Test</h1>
  <p class="ok">Live video + audio (synced)</p>
  <video id="v" src="/stream" autoplay muted controls></video>
  <br>
  <button onclick="document.getElementById('v').muted=false;this.hidden=true;">Unmute Audio</button>
  <p class="dim">640x480 @ 15fps VP8 + Opus | WebM</p>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        if self.path == "/":
            self._serve_html()
        elif self.path == "/stream":
            self._serve_stream()
        else:
            self.send_error(404)

    def _serve_html(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/html")
        self.end_headers()
        self.wfile.write(HTML.encode())

    def _serve_stream(self):
        """Single muxed WebM stream — VP8 video + Opus audio, synced."""
        proc = subprocess.Popen(
            [
                "ffmpeg",
                "-fflags", "nobuffer",
                "-f", "v4l2",
                "-video_size", "640x480",
                "-framerate", "15",
                "-i", VIDEO_DEVICE,
                "-f", "alsa",
                "-channels", "2",
                "-sample_rate", "48000",
                "-i", AUDIO_DEVICE,
                "-c:v", "libvpx",
                "-quality", "realtime",
                "-speed", "8",
                "-b:v", "500k",
                "-g", "30",
                "-c:a", "libopus",
                "-b:a", "48k",
                "-ac", "1",
                "-f", "webm",
                "pipe:1",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )
        self.send_response(200)
        self.send_header("Content-Type", "video/webm")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        try:
            while True:
                chunk = proc.stdout.read(4096)
                if not chunk:
                    break
                self.wfile.write(chunk)
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            proc.kill()
            proc.wait()


class ThreadedHTTPServer(HTTPServer):
    def process_request(self, request, client_address):
        t = threading.Thread(target=self.finish_request, args=(request, client_address), daemon=True)
        t.start()


def main():
    server = ThreadedHTTPServer(("0.0.0.0", PORT), Handler)
    print(f"Camera test at http://0.0.0.0:{PORT}")
    print("  Stream: 640x480 @ 15fps VP8 + Opus (muxed WebM)")
    print("Ctrl+C to stop")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        print("\nStopped.")


if __name__ == "__main__":
    main()
