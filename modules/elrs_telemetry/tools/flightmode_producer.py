#!/usr/bin/env python3
"""H4 step 5 — CRSF 0x21 FLIGHT_MODE producer @ 1 Hz (stubbed).

Variable-length frame test. Cycles through a list of mode strings
(short to long, alpha + underscore) so the decoder exercises:
  - dynamic LEN field handling
  - null-terminated payload boundary
  - non-alpha ASCII (underscore)
  - round-trip integrity at multiple frame sizes

Frame layout (CRSF 0x21):
    [0xC8]         sync
    [LEN]          = 1 (type) + (N+1) (payload incl trailing \\0) + 1 (crc)
                   = N + 3
    [0x21]         type = FLIGHT_MODE
    [payload]      mode_string_bytes + 0x00
    [CRC8/DVB-S2 over (type + payload)]

Example "ACRO\\0": payload=5B, LEN=7, frame=9B.
Example "FAILSAFE\\0": payload=9B, LEN=11, frame=13B.

Mode cycle: 1 s per mode, loop indefinitely.
"""
import datetime
import json
import time
import urllib.request

URL = "http://localhost:5003/telemetry/raw"
PUSH_HZ = 1.0
PUSH_PERIOD = 1.0 / PUSH_HZ

MODE_CYCLE = [
    "ACRO",
    "ANGLE",
    "STAB",
    "AUTO",
    "GUIDED",
    "RTL",
    "LAND",
    "FAILSAFE",
    "ALT_HOLD",
    "POS_HOLD",
]


def crc8_dvbs2(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def build_flightmode_frame(mode_str):
    """Pack a CRSF 0x21 FLIGHT_MODE frame for the given mode string.
    Returns the full wire frame (variable length)."""
    payload = mode_str.encode("ascii") + b"\x00"
    length = 1 + len(payload) + 1  # type + payload + crc
    crc = crc8_dvbs2(bytes([0x21]) + payload)
    return bytes([0xC8, length, 0x21]) + payload + bytes([crc])


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
    print(f"[{utc_stamp()}] flightmode_producer: 0x21 @ {PUSH_HZ:.0f} Hz to {URL}",
          flush=True)
    print(f"[{utc_stamp()}] cycling {len(MODE_CYCLE)} modes (1 s each): "
          f"{', '.join(MODE_CYCLE)}", flush=True)

    idx = 0
    next_tick = time.monotonic()
    while True:
        mode = MODE_CYCLE[idx]
        frame = build_flightmode_frame(mode)
        post(frame.hex())
        print(f"  [{utc_stamp()}] mode={mode!r:12} frame_len={len(frame)}B  "
              f"hex={frame.hex()}", flush=True)
        idx = (idx + 1) % len(MODE_CYCLE)

        next_tick += PUSH_PERIOD
        sleep = next_tick - time.monotonic()
        if sleep > 0:
            time.sleep(sleep)
        else:
            next_tick = time.monotonic()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(f"[{utc_stamp()}] stopped", flush=True)
