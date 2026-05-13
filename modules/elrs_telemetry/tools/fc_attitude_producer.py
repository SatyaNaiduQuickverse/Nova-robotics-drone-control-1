#!/usr/bin/env python3
"""FC-sourced CRSF 0x1E ATTITUDE producer @ 10 Hz.

Reads /telemetry/orientation (roll, pitch, yaw in radians) and forwards
as CRSF 0x1E. Push rate fixed at 10 Hz — matches the downlink substrate
ceiling at rf_mode=23 measured during H4 step 4 (host 30 Hz collapsed
to ~10 Hz on wire). No benefit pushing faster; saves CPU + OTA budget
for telemetry types that aren't already saturated.

Frame layout (CRSF 0x1E, BE):
    [0xC8] [0x08] [0x1E] [pitch_rad×10000 i16]
                         [roll_rad×10000  i16]
                         [yaw_rad×10000   i16]
                         [CRC]
"""
from __future__ import annotations

import datetime
import json
import math
import os
import struct
import sys
import time
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fc_client import FcClient
from producer_safety import (
    production_setup, release_pidfile, install_clean_shutdown_handlers,
    ProducerSafetyError,
)

URL = "http://localhost:5003/telemetry/raw"
PUSH_HZ = 10.0
PUSH_PERIOD = 1.0 / PUSH_HZ
LOG_PERIOD_S = 10.0
SCALE = 10000.0   # CRSF attitude: radians × 10000


def crc8_dvbs2(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def build_attitude_frame(pitch_raw, roll_raw, yaw_raw):
    payload = struct.pack(">hhh", pitch_raw, roll_raw, yaw_raw)
    assert len(payload) == 6
    crc = crc8_dvbs2(bytes([0x1E]) + payload)
    return bytes([0xC8, 0x08, 0x1E]) + payload + bytes([crc])


def _rad_to_raw(rad):
    """Convert radians to CRSF i16, clamped to representable range."""
    return max(-32768, min(32767, int(round(rad * SCALE))))


def encode_fc_attitude(o):
    """Convert FC orientation dict → CRSF 0x1E frame bytes (None if
    invalid). Pass-through: zeros from FC become zeros on the wire."""
    if not isinstance(o, dict):
        return None
    roll  = o.get("roll")
    pitch = o.get("pitch")
    yaw   = o.get("yaw")
    if not all(isinstance(x, (int, float)) for x in (roll, pitch, yaw)):
        return None
    # CRSF frame order: pitch / roll / yaw (note the order — pitch first).
    return build_attitude_frame(
        _rad_to_raw(pitch),
        _rad_to_raw(roll),
        _rad_to_raw(yaw),
    )


def post(frame_hex):
    body = json.dumps({"hex": frame_hex}).encode()
    req = urllib.request.Request(
        URL, data=body, method="POST",
        headers={"Content-Type": "application/json"},
    )
    try:
        urllib.request.urlopen(req, timeout=0.05).read()
    except Exception:
        pass


def utc_stamp():
    return datetime.datetime.now(datetime.timezone.utc).strftime("%H:%M:%S.%f")[:-3] + "Z"


def main():
    install_clean_shutdown_handlers()
    pidfile = production_setup("attitude")
    fc = FcClient()
    print(f"[{utc_stamp()}] fc_attitude_producer: 0x1E @ {PUSH_HZ:.0f} Hz "
          f"source={fc.base_url}/telemetry/orientation", flush=True)

    next_tick = time.monotonic()
    last_log = time.monotonic()
    sent = 0
    skipped = 0
    last_fail_logged = None
    last_attitude = None

    try:
        while True:
            o = fc.get_orientation()
            if o is None:
                skipped += 1
                if fc.last_error != last_fail_logged:
                    print(f"  [{utc_stamp()}] FC unreachable: {fc.last_error}",
                          flush=True)
                    last_fail_logged = fc.last_error
            else:
                if last_fail_logged is not None:
                    print(f"  [{utc_stamp()}] FC reachable again", flush=True)
                    last_fail_logged = None
                frame = encode_fc_attitude(o)
                if frame is not None:
                    post(frame.hex())
                    sent += 1
                    last_attitude = o
                else:
                    skipped += 1

            next_tick += PUSH_PERIOD
            sleep = next_tick - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_tick = time.monotonic()

            if time.monotonic() - last_log >= LOG_PERIOD_S:
                last_log = time.monotonic()
                if last_attitude is not None:
                    deg = lambda r: math.degrees(r)
                    p = deg(last_attitude.get("pitch", 0))
                    r = deg(last_attitude.get("roll", 0))
                    y = deg(last_attitude.get("yaw", 0))
                    extras = f"pitch={p:+6.1f}° roll={r:+6.1f}° yaw={y:+6.1f}°"
                else:
                    extras = "no attitude yet"
                print(f"  [{utc_stamp()}] sent={sent} skipped={skipped} {extras} "
                      f"fc_failures={fc.consecutive_failures}",
                      flush=True)
    except KeyboardInterrupt:
        print(f"[{utc_stamp()}] stopped — sent={sent} skipped={skipped}",
              flush=True)
    finally:
        release_pidfile(pidfile)


if __name__ == "__main__":
    try:
        main()
    except ProducerSafetyError as e:
        print(f"SAFETY: {e}", file=sys.stderr)
        sys.exit(2)
