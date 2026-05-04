#!/usr/bin/env python3
"""VTX renderer — pulls JPEGs from drone-control's /camera/tracking/snapshot,
blits to a pygame fullscreen window on the SD-resolution composite display
(Pi 5 J7 → analog VTX → 5.8 GHz).

Why pygame/Wayland instead of /dev/fb0:
    On Pi 5 with vc4-kms-v3d, /dev/fb0 is a fbdev compatibility shim. Writes
    succeed (file content changes) but nothing actively scans those pixels
    out to the composite plane unless a DRM client owns the CRTC. Verified
    diagnostically: writing solid white/green/red to fb0 produced NO change
    at the VRX (constant blue no-signal). Pygame via Wayland (labwc) drives
    the composite plane through KMS properly — same approach used in earlier
    proven runs.

Environment requirements (pygame talks to the running labwc Wayland session):
    XDG_RUNTIME_DIR=/run/user/$UID
    WAYLAND_DISPLAY=wayland-0
    SDL_VIDEODRIVER=wayland

These are set explicitly inside this script if not already present, so it
works under SSH/cron/systemd where the desktop session env isn't inherited.

Run:
    python3 scripts/vtx_renderer.py            # default 25 fps
    python3 scripts/vtx_renderer.py --fps 30   # bump for NTSC capture rate
    python3 scripts/vtx_renderer.py --src http://localhost:8080/camera/<name>/snapshot

Stop: Ctrl-C — pygame quits cleanly.
"""

from __future__ import annotations

import argparse
import logging
import os
import signal
import sys
import time
import urllib.error
import urllib.request
from collections import deque

# --- Pygame env must be set BEFORE the import. -------------------------
# Without these, pygame defaults to kmsdrm which fails because labwc is
# already holding the DRM card.
os.environ.setdefault("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")
os.environ.setdefault("WAYLAND_DISPLAY", "wayland-0")
os.environ.setdefault("SDL_VIDEODRIVER", "wayland")

import cv2
import numpy as np
import pygame  # noqa: E402  (intentional import-after-env-set)


# --- Constants ---------------------------------------------------------

DEFAULT_SRC  = "http://localhost:8080/vtx/snapshot"
DEFAULT_FPS  = 25.0
HTTP_TIMEOUT = 1.0
ERR_BACKOFF  = 0.05

LOG_FMT = "%(asctime)s.%(msecs)03d %(levelname)-7s %(message)s"
LOG_DATEFMT = "%H:%M:%S"

# An "SD" display is anything narrower than HD-720p. The Pi 5 composite
# display is the only sub-1000-wide display target in our setup, so this
# heuristic identifies it unambiguously.
SD_WIDTH_THRESHOLD = 1000


# --- Display setup -----------------------------------------------------

def find_sd_display() -> tuple[int, tuple[int, int]]:
    """Return (display_index, (w,h)) of the first SD-resolution display.
    Raises RuntimeError if none found."""
    sizes = pygame.display.get_desktop_sizes()
    for i, s in enumerate(sizes):
        if s[0] < SD_WIDTH_THRESHOLD:
            return i, s
    raise RuntimeError(
        f"no SD-resolution display found (got {sizes}); is the composite "
        f"output enabled in /boot/firmware/config.txt + cmdline.txt?"
    )


# --- Frame fetch + decode ---------------------------------------------

def fetch_jpeg(url: str) -> bytes | None:
    try:
        with urllib.request.urlopen(url, timeout=HTTP_TIMEOUT) as r:
            return r.read()
    except (urllib.error.URLError, OSError, TimeoutError):
        return None


def jpeg_to_pygame_surface(jpg: bytes,
                           dst_size: tuple[int, int]) -> pygame.Surface | None:
    """JPEG bytes → cv2.imdecode (BGR) → resize → BGR→RGB → pygame Surface.
    pygame.image.frombuffer expects 'RGB' (not BGR), so we cv2 the swap.
    Returns None on decode failure."""
    arr = np.frombuffer(jpg, dtype=np.uint8)
    bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if bgr is None:
        return None
    if (bgr.shape[1], bgr.shape[0]) != dst_size:
        bgr = cv2.resize(bgr, dst_size, interpolation=cv2.INTER_LINEAR)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    # frombuffer expects bytes; numpy arrays satisfy the buffer protocol.
    return pygame.image.frombuffer(rgb.tobytes(), dst_size, "RGB")


# --- Main loop ---------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--src", default=DEFAULT_SRC, help="JPEG snapshot URL")
    ap.add_argument("--fps", type=float, default=DEFAULT_FPS,
                    help=f"target render rate (default {DEFAULT_FPS})")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    logging.basicConfig(
        format=LOG_FMT, datefmt=LOG_DATEFMT,
        level=logging.DEBUG if args.verbose else logging.INFO,
    )
    log = logging.getLogger("vtx_renderer")

    log.info("pygame env: XDG_RUNTIME_DIR=%s WAYLAND_DISPLAY=%s "
             "SDL_VIDEODRIVER=%s",
             os.environ.get("XDG_RUNTIME_DIR"),
             os.environ.get("WAYLAND_DISPLAY"),
             os.environ.get("SDL_VIDEODRIVER"))

    pygame.display.init()
    log.info("pygame driver: %s", pygame.display.get_driver())

    try:
        sd_idx, sd_size = find_sd_display()
    except RuntimeError as e:
        log.error(str(e))
        return 1

    log.info("SD display: index=%d  size=%dx%d  → composite-out",
             sd_idx, sd_size[0], sd_size[1])

    screen = pygame.display.set_mode(
        sd_size, pygame.FULLSCREEN, display=sd_idx,
    )
    # Start with a blank dark frame — better than whatever labwc had before.
    screen.fill((0, 0, 0))
    pygame.display.flip()

    log.info("source %s @ target %.1f fps", args.src, args.fps)

    # Clean shutdown.
    stopping = False
    def _stop(signum, _frame):
        nonlocal stopping
        log.info("signal %d — stopping", signum)
        stopping = True
    signal.signal(signal.SIGINT,  _stop)
    signal.signal(signal.SIGTERM, _stop)

    period = 1.0 / max(args.fps, 1.0)
    next_t = time.monotonic()

    ts_window: deque[float] = deque(maxlen=60)
    err_count = 0
    last_log = time.monotonic()

    try:
        while not stopping:
            t0 = time.monotonic()

            # Drain pygame events (otherwise compositor may consider the
            # window unresponsive).
            for _ in pygame.event.get():
                pass

            jpg = fetch_jpeg(args.src)
            if jpg is None:
                err_count += 1
                time.sleep(ERR_BACKOFF)
                continue

            surf = jpeg_to_pygame_surface(jpg, sd_size)
            if surf is None:
                err_count += 1
                time.sleep(ERR_BACKOFF)
                continue

            screen.blit(surf, (0, 0))
            pygame.display.flip()
            ts_window.append(t0)

            now = time.monotonic()
            if now - last_log >= 5.0:
                last_log = now
                if len(ts_window) >= 2:
                    span = ts_window[-1] - ts_window[0]
                    live_fps = (len(ts_window) - 1) / span if span > 0 else 0
                else:
                    live_fps = 0
                log.info("rendered=%-3.1f fps  errors=%d  jpg=%d B",
                         live_fps, err_count, len(jpg))

            # Pace to target fps.
            next_t += period
            sleep_for = next_t - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_t = time.monotonic()

    finally:
        log.info("blanking + quitting pygame")
        try:
            screen.fill((0, 0, 0))
            pygame.display.flip()
        except Exception:
            pass
        pygame.quit()

    return 0


if __name__ == "__main__":
    sys.exit(main())
