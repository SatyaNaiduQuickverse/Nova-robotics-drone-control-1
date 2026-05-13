#!/usr/bin/env python3
"""H4 step 2 — CRSF 0x08 BATTERY_SENSOR producer @ 2 Hz (stubbed).

Pushes synthetic varying battery values to the daemon's /telemetry/raw
endpoint so the ground-side decoder has signal to validate against.
Stubbed (not FC-sourced) keeps the test surface decoupled — pure
decoder validation, no FC pipeline dependency.

Frame layout (CRSF spec):
    [0xC8]      sync (FC address)
    [0x0A]      len = type(1) + payload(8) + crc(1)
    [0x08]      type = BATTERY_SENSOR
    [payload]   8 bytes, big-endian:
                  u16  voltage    × 0.1 V
                  u16  current    × 0.1 A
                  u24  capacity   mAh (3 bytes, BE)
                  u8   remaining  % (0..100)
    [CRC8/DVB-S2 over (type + payload)]

Variance schedule (60 s loop, matches ground's listen window):
    voltage_V       = 15.0 + 1.0 * sin(2π t / 30)      sine, ±1.0 V
    current_A       = (t mod 60) / 2.0                  ramp 0 → 30 A
    capacity_mAh   += int(current_A * 1000 / 3600 / 2) per push    accumulating
    remaining_pct   = 100 - int((t % 30.0) / 0.3)       sawtooth 100 → 0 every 30 s

OTA-rate caveat:
    Host-push at 2 Hz, but ELRS OTA telem budget at 1:128 ratio is
    ~100/128 = 0.78 Hz on this link. The Tx aggregates incoming frames
    and sends the freshest at each OTA telem slot — ground will see
    ~0.78 Hz on the wire even though we push at 2 Hz. Decoder-side
    pass criteria should account for this if it expects to match host
    push rate.
"""
import datetime
import json
import math
import struct
import time
import urllib.request

URL = "http://localhost:5003/telemetry/raw"
PUSH_HZ = 2.0
PUSH_PERIOD = 1.0 / PUSH_HZ


def crc8_dvbs2(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def build_battery_frame(voltage_dV, current_dA, capacity_mAh, remaining_pct):
    """Pack a CRSF 0x08 BATTERY_SENSOR frame. Returns 12 raw bytes."""
    # 24-bit capacity split into high byte + low u16 BE
    cap_hi = (capacity_mAh >> 16) & 0xFF
    cap_lo = capacity_mAh & 0xFFFF
    payload = struct.pack(">HHBHB",
                          voltage_dV & 0xFFFF,
                          current_dA & 0xFFFF,
                          cap_hi,
                          cap_lo,
                          remaining_pct & 0xFF)
    assert len(payload) == 8
    crc = crc8_dvbs2(bytes([0x08]) + payload)
    return bytes([0xC8, 0x0A, 0x08]) + payload + bytes([crc])


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
    print(f"[{utc_stamp()}] battery_producer: 0x08 @ {PUSH_HZ:.0f} Hz to {URL}",
          flush=True)
    print(f"[{utc_stamp()}] stubbed: voltage=sine(15±1V/30s), "
          f"current=ramp(0→30A/60s), capacity_mAh=accumulating, "
          f"remaining%=decay(100→0/60s)", flush=True)

    t0 = time.monotonic()
    capacity_mAh = 0
    next_tick = time.monotonic()
    count = 0
    last_log = 0.0
    while True:
        t = time.monotonic() - t0
        voltage_V = 15.0 + 1.0 * math.sin(2.0 * math.pi * t / 30.0)
        current_A = (t % 60.0) / 2.0
        capacity_mAh += int(current_A * 1000.0 / 3600.0 / PUSH_HZ)
        remaining_pct = 100 - int((t % 30.0) / 0.3)

        voltage_dV = int(round(voltage_V * 10.0))
        current_dA = int(round(current_A * 10.0))

        frame = build_battery_frame(
            voltage_dV, current_dA, capacity_mAh, remaining_pct,
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
            print(f"  [{utc_stamp()}] t={t:5.1f}s  V={voltage_V:5.2f}  "
                  f"I={current_A:5.2f}  mAh={capacity_mAh:5d}  "
                  f"%={remaining_pct:3d}  (sent {count})", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(f"[{utc_stamp()}] stopped", flush=True)
