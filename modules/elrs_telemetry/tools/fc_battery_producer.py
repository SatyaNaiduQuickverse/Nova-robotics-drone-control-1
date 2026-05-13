#!/usr/bin/env python3
"""FC-sourced CRSF 0x08 BATTERY_SENSOR producer @ 2 Hz.

Reads drone-control's /telemetry/battery and forwards real values as
CRSF 0x08 to elrs-telemetry. Pass-through semantics — whatever the FC
reports flows out. When battery signal pins aren't connected the FC
reports near-zero values; those propagate to ground as-is.

This is the production replacement for tools/battery_producer.py.
Producer-safety gate prevents starting alongside the stub.

Frame layout (identical to stub; verified by D2 dry-run shape match):
    [0xC8] [0x0A] [0x08] [voltage_dV BE u16] [current_dA BE u16]
                         [capacity_mAh BE u24] [remaining_pct u8] [CRC]

capacity_mAh: drone-control doesn't expose it directly. We integrate
the live current reading over producer uptime. Resets to 0 on each
producer start. When current is non-positive (no draw, or negative
regen reading) capacity stays flat — correct "no consumption" state.

FC unreachable: skip the cycle, no POST. Consumer sees no telemetry
until FC is back; that's the correct signal — don't fake stale data.
"""
from __future__ import annotations

import datetime
import json
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
PUSH_HZ = 2.0
PUSH_PERIOD = 1.0 / PUSH_HZ
LOG_PERIOD_S = 5.0


def crc8_dvbs2(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def build_battery_frame(voltage_dV, current_dA, capacity_mAh, remaining_pct):
    cap_hi = (capacity_mAh >> 16) & 0xFF
    cap_lo = capacity_mAh & 0xFFFF
    payload = struct.pack(">HHBHB",
                          voltage_dV & 0xFFFF,
                          current_dA & 0xFFFF,
                          cap_hi, cap_lo,
                          remaining_pct & 0xFF)
    assert len(payload) == 8
    crc = crc8_dvbs2(bytes([0x08]) + payload)
    return bytes([0xC8, 0x0A, 0x08]) + payload + bytes([crc])


def encode_fc_battery(b, capacity_mAh: int):
    """Convert an FC battery dict → CRSF 0x08 frame bytes (or None
    if `b` is None or any required field is missing). Pass-through:
    zeros from FC become zeros on the wire."""
    if not isinstance(b, dict):
        return None
    v = b.get("voltage")
    i = b.get("current")
    p = b.get("percentage")
    if not all(isinstance(x, (int, float)) for x in (v, i, p)):
        return None
    # CRSF voltage/current are unsigned. Clamp non-negative.
    voltage_dV = max(0, min(0xFFFF, int(round(v * 10.0))))
    current_dA = max(0, min(0xFFFF, int(round(i * 10.0))))
    # FC reports percentage as 0..1 fraction; CRSF wants 0..100 u8.
    remaining_pct = max(0, min(100, int(round(p * 100.0))))
    return build_battery_frame(voltage_dV, current_dA, capacity_mAh, remaining_pct)


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
    pidfile = production_setup("battery")
    fc = FcClient()
    print(f"[{utc_stamp()}] fc_battery_producer: 0x08 @ {PUSH_HZ:.0f} Hz "
          f"source={fc.base_url}/telemetry/battery", flush=True)

    capacity_mAh = 0
    t_last = time.monotonic()
    next_tick = time.monotonic()
    last_log = time.monotonic()
    sent = 0
    skipped = 0
    last_fail_logged = None

    try:
        while True:
            now = time.monotonic()
            dt_s = now - t_last
            t_last = now

            b = fc.get_battery()
            if b is None:
                skipped += 1
                if fc.last_error != last_fail_logged:
                    print(f"  [{utc_stamp()}] FC unreachable: {fc.last_error}",
                          flush=True)
                    last_fail_logged = fc.last_error
            else:
                if last_fail_logged is not None:
                    print(f"  [{utc_stamp()}] FC reachable again", flush=True)
                    last_fail_logged = None
                # Integrate POSITIVE current over interval → capacity_mAh.
                # Negative readings (regen / FC quirks) don't decrement.
                current_A = b.get("current")
                if isinstance(current_A, (int, float)) and current_A > 0:
                    capacity_mAh += int(current_A * 1000.0 * dt_s / 3600.0)
                frame = encode_fc_battery(b, capacity_mAh)
                if frame is not None:
                    post(frame.hex())
                    sent += 1
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
                print(f"  [{utc_stamp()}] sent={sent} skipped={skipped} "
                      f"capacity_mAh={capacity_mAh} "
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
