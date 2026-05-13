#!/usr/bin/env python3
"""H4 step 4 — CRSF 0x1E ATTITUDE producer @ 30 Hz (stubbed).

High-rate decoder validation. Pushes synthetic varying attitude values
to the daemon's /telemetry/raw endpoint. Stacks with RC stub (50 Hz),
Battery (2 Hz), GPS (5 Hz) for total ~87 Hz host push — first real
stress test of the daemon + Tx forwarding pipeline.

Frame layout (CRSF spec, BIG-ENDIAN):
    [0xC8]      sync
    [0x08]      len = type(1) + payload(6) + crc(1) = 8
    [0x1E]      type = ATTITUDE
    [payload]   6 bytes:
                  i16  pitch    radians × 10000
                  i16  roll     radians × 10000
                  i16  yaw      radians × 10000 (signed, -π..+π)
    [CRC8/DVB-S2 over (type + payload)]
    Total: 10 bytes wire

Conversion factor: deg × π/180 × 10000   (so 45° = 7854 raw)

Variance schedule:
    pitch_deg = 30 * sin(t/3)         ±30°, 18.85 s period
    roll_deg  = 45 * sin(t/2)         ±45°, 12.57 s period
    yaw_deg   = (t * 90 / 4) wrapped to -180..+180   continuous rotation
"""
import datetime
import json
import math
import struct
import time
import urllib.request

URL = "http://localhost:5003/telemetry/raw"
PUSH_HZ = 30.0
PUSH_PERIOD = 1.0 / PUSH_HZ
DEG2RAD = math.pi / 180.0


def crc8_dvbs2(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def build_attitude_frame(pitch_raw, roll_raw, yaw_raw):
    """Pack a CRSF 0x1E ATTITUDE frame. Returns 10 raw bytes."""
    payload = struct.pack(">hhh", pitch_raw, roll_raw, yaw_raw)
    assert len(payload) == 6
    crc = crc8_dvbs2(bytes([0x1E]) + payload)
    return bytes([0xC8, 0x08, 0x1E]) + payload + bytes([crc])


def deg_to_raw(deg):
    """Convert degrees to CRSF radians × 10000 (i16)."""
    return int(round(deg * DEG2RAD * 10000.0))


def wrap_180(deg):
    """Wrap an unbounded degree value into -180..+180."""
    d = deg % 360.0
    if d > 180.0:
        d -= 360.0
    return d


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
    print(f"[{utc_stamp()}] attitude_producer: 0x1E @ {PUSH_HZ:.0f} Hz to {URL}",
          flush=True)
    print(f"[{utc_stamp()}] stubbed: pitch=±30°/3s  roll=±45°/2s  yaw=continuous "
          f"22.5°/s wrapped ±180", flush=True)

    t0 = time.monotonic()
    next_tick = time.monotonic()
    count = 0
    last_log = 0.0
    while True:
        t = time.monotonic() - t0
        pitch_deg = 30.0 * math.sin(t / 3.0)
        roll_deg = 45.0 * math.sin(t / 2.0)
        yaw_deg = wrap_180(t * 90.0 / 4.0)

        frame = build_attitude_frame(
            deg_to_raw(pitch_deg),
            deg_to_raw(roll_deg),
            deg_to_raw(yaw_deg),
        )
        post(frame.hex())
        count += 1

        next_tick += PUSH_PERIOD
        sleep = next_tick - time.monotonic()
        if sleep > 0:
            time.sleep(sleep)
        else:
            next_tick = time.monotonic()

        if t - last_log >= 5.0:
            last_log = t
            print(f"  [{utc_stamp()}] t={t:5.1f}s  pitch={pitch_deg:+6.2f}°  "
                  f"roll={roll_deg:+6.2f}°  yaw={yaw_deg:+7.2f}°  "
                  f"(sent {count})", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(f"[{utc_stamp()}] stopped", flush=True)
