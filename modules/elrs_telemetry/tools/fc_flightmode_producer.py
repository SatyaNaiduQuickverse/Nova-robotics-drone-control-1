#!/usr/bin/env python3
"""FC-sourced CRSF 0x21 FLIGHT_MODE producer, on-change @ 2 Hz poll.

Polls drone-control's /telemetry/state at 2 Hz. Pushes a CRSF 0x21
frame whenever .mode changes, plus a low-rate heartbeat so a consumer
that connected mid-flight always sees a current mode within
HEARTBEAT_S seconds.

Frame layout (variable-length; identical to stub):
    [0xC8] [LEN] [0x21] [mode_str + 0x00] [CRC]
    LEN = type(1) + payload(N+1 incl null) + crc(1) = N + 3

FC unreachable: skip the cycle. No fake-mode emission.
"""
from __future__ import annotations

import datetime
import json
import os
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
POLL_HZ = 2.0
POLL_PERIOD = 1.0 / POLL_HZ
HEARTBEAT_S = 5.0          # Push current mode at least every 5 s
LOG_PERIOD_S = 10.0


def crc8_dvbs2(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def build_flightmode_frame(mode_str: str) -> bytes:
    payload = mode_str.encode("ascii", errors="ignore") + b"\x00"
    length = 1 + len(payload) + 1   # type + payload + crc
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
    install_clean_shutdown_handlers()
    pidfile = production_setup("flightmode")
    fc = FcClient()
    print(f"[{utc_stamp()}] fc_flightmode_producer: 0x21 on-change "
          f"(poll {POLL_HZ:.0f} Hz, heartbeat every {HEARTBEAT_S:.0f}s) "
          f"source={fc.base_url}/telemetry/state", flush=True)

    last_mode = None
    last_push_t = 0.0
    next_tick = time.monotonic()
    last_log = time.monotonic()
    sent = 0
    skipped = 0
    last_fail_logged = None

    try:
        while True:
            s = fc.get_state()
            if s is None:
                skipped += 1
                if fc.last_error != last_fail_logged:
                    print(f"  [{utc_stamp()}] FC unreachable: {fc.last_error}",
                          flush=True)
                    last_fail_logged = fc.last_error
            else:
                if last_fail_logged is not None:
                    print(f"  [{utc_stamp()}] FC reachable again", flush=True)
                    last_fail_logged = None
                mode = s.get("mode")
                if isinstance(mode, str) and mode:
                    now = time.monotonic()
                    on_change = (mode != last_mode)
                    heartbeat_due = (now - last_push_t) >= HEARTBEAT_S
                    if on_change or heartbeat_due:
                        post(build_flightmode_frame(mode).hex())
                        sent += 1
                        last_push_t = now
                        if on_change:
                            print(f"  [{utc_stamp()}] mode change: "
                                  f"{last_mode!r} → {mode!r}", flush=True)
                        last_mode = mode
                else:
                    skipped += 1

            next_tick += POLL_PERIOD
            sleep = next_tick - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_tick = time.monotonic()

            if time.monotonic() - last_log >= LOG_PERIOD_S:
                last_log = time.monotonic()
                print(f"  [{utc_stamp()}] sent={sent} skipped={skipped} "
                      f"current_mode={last_mode!r} "
                      f"fc_failures={fc.consecutive_failures}", flush=True)
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
