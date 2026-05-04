#!/usr/bin/env python3
"""
Camera capture module.
Supports multiple named cameras (e.g. "landing", "tracking").

Two source types:
  - v4l2_mjpeg: local USB camera, ffmpeg reads native MJPEG from /dev/video*
  - tcp_mjpeg:  MJPEG stream over TCP (e.g. pi-cam sidecar running rpicam-vid
                in listen mode on 127.0.0.1:8090). Used for the Pi CSI cam.

Stable device paths via /dev/v4l/by-id/ — survive USB re-enumeration.
"""

import socket
import subprocess
import threading
import time
import os
from collections import deque
from typing import Optional


# Stable by-id path for the USB front camera (survives replug). Pi Cam
# Module 3 / CSI path retired — only USB camera is connected, used as
# the front (tracking) camera. Lenovo FHD UVC webcam — true 30 fps at
# 1280x720 MJPEG (unlike the Jieli AV-capture chip which capped at ~16).
FRONT_DEVICE = "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._Lenovo_FHD_Webcam_Audio_SN0001-video-index0"

# VRX loopback capture — MacroSilicon MS210x USB AV-grabber receiving
# the 5.8 GHz analog feed from the on-drone GEPRC VTX. YUYV-only chip
# (no MJPEG support), so this entry uses the v4l2_yuyv source kind which
# transcodes via ffmpeg before serving as JPEG.
# Standard: PAL (matches Pi sdtv_mode=2 — proven config from earlier
# working setup). NTSC alternative: 720x480 @ 30 fps with sdtv_mode=0.
VRX_DEVICE = "/dev/v4l/by-id/usb-MACROSILICON_USB_Video_20200909-video-index0"

# Per-camera config:
#   tracking      — USB Lenovo FHD webcam (front-facing). MJPEG native.
#                   Consumed by the dashboard tracking tile and by
#                   vision-detect. The frame this serves is what the
#                   VTX renderer also reads → goes out J7 composite.
#   vrx_loopback  — MacroSilicon USB AV-capture receiving the 5.8 GHz
#                   feed back from the VTX (loopback validation).
#                   YUYV-only, transcoded to MJPEG via ffmpeg.
CAMERAS = {
    "tracking": {
        "source": "v4l2_mjpeg",
        "device": FRONT_DEVICE,
        "width": 1280,
        "height": 720,
        "fps": 30,
    },
    # vrx_loopback DISABLED 2026-05-01 to recover ~30% CPU (YUYV→MJPEG ffmpeg
    # transcode was the cost). Loopback path was already validated end-to-end
    # (std=39.3 lock confirmed). Re-enable by uncommenting below + restart drone.
    # "vrx_loopback": {
    #     "source": "v4l2_yuyv",
    #     "device": VRX_DEVICE,
    #     "width": 720,
    #     "height": 576,
    #     "fps": 25,
    # },
}


class Camera:
    """Single camera instance — owns an ffmpeg subprocess and a frame buffer."""

    def __init__(self, name: str, source: str = "v4l2_mjpeg",
                 device: Optional[str] = None, url: Optional[str] = None,
                 width: int = 640, height: int = 480, fps: int = 30):
        self.name = name
        self.source = source
        self.device = device
        self.url = url
        self.width = width
        self.height = height
        self.fps = fps

        self._frame: Optional[bytes] = None
        self._lock = threading.Lock()
        self._proc: Optional[subprocess.Popen] = None
        self._running = False
        self._frame_count = 0
        self._start_time = 0.0
        # Rolling window of recent frame timestamps for live fps reporting.
        # 60 entries × ~33-66 ms/frame → 2-4 second window — recent enough
        # to react to live changes, long enough to smooth USB jitter.
        self._frame_ts: deque[float] = deque(maxlen=60)

    def start(self):
        if self._running:
            return
        # v4l2 sources need the /dev node present at start time. TCP sources
        # don't — the pi-cam sidecar may still be booting, so the capture
        # loop retries on connect failure.
        if self.source == "v4l2_mjpeg" and not os.path.exists(self.device or ""):
            return

        self._frame_count = 0
        self._start_time = time.monotonic()
        self._running = True
        threading.Thread(target=self._capture_loop, daemon=True).start()

    def stop(self):
        self._running = False
        if self._proc:
            self._proc.kill()
            self._proc.wait()
            self._proc = None
        with self._lock:
            self._frame = None

    def _capture_loop(self):
        # Outer loop: reconnect if the source dies (pi-cam sidecar restarted,
        # USB camera transient unplug, ffmpeg exit). 1 s backoff.
        while self._running:
            if self.source == "tcp_mjpeg":
                self._run_tcp()
            elif self.source == "v4l2_yuyv":
                # YUYV-only devices (cheap USB AV-capture chips like
                # MacroSilicon MS210x). v4l2-ctl can't produce MJPEG
                # from these — we transcode via ffmpeg.
                self._run_ffmpeg_yuyv()
            else:
                # v4l2_mjpeg (default) — kernel-direct via v4l2-ctl,
                # zero re-encode tax. Used by UVC cameras that natively
                # produce MJPEG (most modern webcams).
                self._run_ffmpeg()
            if self._running:
                time.sleep(1.0)

    def _consume_mjpeg(self, read_fn) -> None:
        """Pull bytes via read_fn(n) and emit JPEG frames by SOI/EOI scan.
        read_fn can be a socket.recv, a subprocess.stdout.read, or any
        blocking reader returning b'' on EOF."""
        buf = b""
        try:
            while self._running:
                chunk = read_fn(65536)
                if not chunk:
                    break
                buf += chunk
                while True:
                    soi = buf.find(b'\xff\xd8')
                    if soi == -1:
                        buf = b""
                        break
                    eoi = buf.find(b'\xff\xd9', soi + 2)
                    if eoi == -1:
                        buf = buf[soi:]
                        break
                    frame_data = buf[soi:eoi + 2]
                    buf = buf[eoi + 2:]
                    with self._lock:
                        self._frame = frame_data
                        self._frame_count += 1
                        self._frame_ts.append(time.monotonic())
        except Exception:
            pass

    def _run_tcp(self):
        """Direct TCP socket path. Skips ffmpeg entirely — the remote
        (rpicam-vid) already outputs raw MJPEG, so putting ffmpeg in the
        middle only adds demuxer buffering (100s of ms of latency)."""
        host_port = (self.url or "").replace("tcp://", "")
        if ":" not in host_port:
            return
        host, port_s = host_port.rsplit(":", 1)
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # TCP_NODELAY: disable Nagle — we want every JPEG boundary
            # delivered immediately, not coalesced into 200 ms batches.
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(5.0)
            sock.connect((host, int(port_s)))
            sock.settimeout(None)
            self._consume_mjpeg(sock.recv)
        except Exception:
            pass
        finally:
            try:
                if sock is not None:
                    sock.close()
            except Exception:
                pass

    def _run_ffmpeg(self):
        """v4l2 path — v4l2-ctl streams native MJPEG off /dev/video* via
        kernel mmap directly to stdout. Replaces the old ffmpeg-based
        path because ffmpeg adds ~7 fps of overhead even with `-c:v copy`
        (measured: 16.4 fps via ffmpeg vs 23.5 fps via v4l2-ctl on the
        same hardware). The bytestream is raw concatenated JPEG frames,
        which `_consume_mjpeg` already parses by SOI/EOI scan.

        Why v4l2-ctl instead of pyv4l2 / cv2 / direct ioctl: v4l2-ctl is
        a tiny C tool already present (apt v4l-utils, ~MB), works against
        any UVC device without per-camera quirks, and `--stream-mmap=4
        --stream-to=-` is the simplest pipe-to-stdout incantation that
        delivers the raw kernel buffers untouched."""
        cmd = [
            "v4l2-ctl",
            "--device", self.device,
            "--set-fmt-video",
                f"width={self.width},height={self.height},pixelformat=MJPG",
            "--set-parm", str(self.fps),
            "--stream-mmap=4",     # 4 driver buffers — matches what we tested
            "--stream-count=0",    # 0 = stream until killed
            "--stream-to=-",       # raw MJPEG bytes to stdout
        ]
        try:
            self._proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0,
            )
        except Exception:
            return
        try:
            self._consume_mjpeg(self._proc.stdout.read)
        finally:
            try:
                if self._proc:
                    self._proc.kill()
                    self._proc.wait(timeout=1.0)
            except Exception:
                pass
            self._proc = None

    def _run_ffmpeg_yuyv(self):
        """YUYV → MJPEG transcode path for cameras that don't produce
        MJPEG natively (e.g. MacroSilicon MS210x cheap AV-capture chips).
        ffmpeg reads YUYV from V4L2 and re-encodes to MJPEG on the way
        out so the existing _consume_mjpeg SOI/EOI scanner can parse it.

        Trade-off: ffmpeg's YUYV→JPEG encode adds ~5-15 ms per frame
        and ~10-20% CPU at 720x576@25 — acceptable for the VRX-loopback
        use case where we're capturing for verification, not for the
        critical FPV path. Quality 5 (q:v=5) is visibly clean, ~80 KB
        per frame at 720x576."""
        cmd = [
            "ffmpeg", "-hide_banner", "-loglevel", "error",
            "-f", "v4l2",
            "-input_format", "yuyv422",
            "-video_size", f"{self.width}x{self.height}",
            "-framerate", str(self.fps),
            "-i", self.device,
            "-c:v", "mjpeg",
            "-q:v", "5",
            "-f", "mjpeg",
            "pipe:1",
        ]
        try:
            self._proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0,
            )
        except Exception:
            return
        try:
            self._consume_mjpeg(self._proc.stdout.read)
        finally:
            try:
                if self._proc:
                    self._proc.kill()
                    self._proc.wait(timeout=1.0)
            except Exception:
                pass
            self._proc = None

    def get_frame(self) -> Optional[bytes]:
        with self._lock:
            return self._frame

    def get_status(self) -> dict:
        elapsed = time.monotonic() - self._start_time if self._running else 0
        # Live fps from the rolling-window timestamp buffer. Computed from
        # the span between the oldest and newest timestamp so it reflects
        # the CURRENT capture rate, not lifetime average (which would hide
        # a sudden drop or never recover from a slow startup).
        with self._lock:
            ts = list(self._frame_ts)
        if len(ts) >= 2:
            span = ts[-1] - ts[0]
            fps_live = round((len(ts) - 1) / span, 1) if span > 0 else 0
        else:
            fps_live = 0
        return {
            "name": self.name,
            "active": self._running,
            "has_frame": self._frame is not None,
            "frame_count": self._frame_count,
            "fps": fps_live,                                                # live rolling window
            "fps_lifetime": round(self._frame_count / elapsed, 1) if elapsed > 1 else 0,
            "source": self.source,
            "device": self.device,
            "url": self.url,
            "width": self.width,
            "height": self.height,
            "target_fps": self.fps,
        }


# --- Registry ---

_registry: dict[str, Camera] = {}


def start():
    """Start all configured cameras. Safe if some devices are missing."""
    for name, cfg in CAMERAS.items():
        if name not in _registry:
            _registry[name] = Camera(name, **cfg)
        _registry[name].start()


def stop():
    """Stop all cameras."""
    for cam in _registry.values():
        cam.stop()


def get(name: str) -> Optional[Camera]:
    return _registry.get(name)


def get_frame(name: str = "landing") -> Optional[bytes]:
    """Backwards-compat: default to landing cam (used by precision_land.py)."""
    cam = _registry.get(name)
    return cam.get_frame() if cam else None


def get_status(name: str = "landing") -> Optional[dict]:
    cam = _registry.get(name)
    return cam.get_status() if cam else None


def list_cameras() -> dict:
    """Status of all configured cameras."""
    return {name: cam.get_status() for name, cam in _registry.items()}


# Backwards-compat module-level constants — referenced by precision_land.py.
# Now reflect the tracking (front USB) camera since landing slot is gone.
# When a downward landing cam is wired, restore a "landing" entry above
# and point these constants back to it.
WIDTH = CAMERAS["tracking"]["width"]
HEIGHT = CAMERAS["tracking"]["height"]
VIDEO_DEVICE = CAMERAS["tracking"].get("device")
FPS = CAMERAS["tracking"]["fps"]
