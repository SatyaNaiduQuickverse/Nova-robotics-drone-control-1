#!/usr/bin/env python3
"""Integration test for the ELRS telemetry service.

Prereqs:
    1. ESP32-C6 flashed with drone_bridge.ino, plugged into the Pi.
    2. RP4TD bound to a Ranger that's powered up on the ground side.
    3. Ground side actively transmitting (else only LINK_STATS appears).
    4. The elrs container is up:  cd modules/elrs_telemetry && docker compose up -d

Run from the host:
    python3 modules/elrs_telemetry/tests/integration_test.py
"""

import json
import sys
import time
import urllib.error
import urllib.request

BASE = "http://localhost:5003"


def get(path: str):
    with urllib.request.urlopen(BASE + path, timeout=2) as r:
        return r.status, json.loads(r.read())


def post(path: str, payload: dict):
    req = urllib.request.Request(
        BASE + path,
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=2) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, None


def fail(msg):
    print(f"\n  FAIL: {msg}")
    sys.exit(1)


def ok(msg):
    print(f"  OK   {msg}")


def main():
    print("== ELRS telemetry integration test ==\n")

    # 1. Service comes up
    try:
        code, body = get("/healthz")
    except (urllib.error.URLError, ConnectionRefusedError) as e:
        fail(f"can't reach {BASE}: {e}. Is the elrs container running?")
    if code != 200 or not body.get("ok"):
        fail(f"healthz: code={code} body={body}")
    ok(f"/healthz uptime={body['uptime_s']}s serial_open={body['serial']}")

    if not body["serial"]:
        print("  WARN: serial port not yet open. ESP32 plugged in?")

    # 2. Wait for uplink_hz to confirm the link is alive.
    #
    # Threshold rationale: with the ground bridge actively pushing 16 channels
    # at 33 Hz, the drone-side wire sees ~75 Hz. Without it (Ranger powered and
    # bound but no host driving channel data), the rate drops to ~40 Hz.
    # 30 Hz is the "link alive at all" floor; either condition passes.
    print("\n  waiting up to 8 s for uplink_hz > 30 (link-alive floor) ...")
    deadline = time.monotonic() + 8
    last_state = None
    crossed = False
    while time.monotonic() < deadline:
        code, st = get("/state")
        last_state = st
        if (st.get("uplink_hz") or 0) > 30:
            crossed = True
            break
        time.sleep(0.5)
    if not crossed:
        fail(f"uplink_hz never crossed 30 (last={json.dumps(last_state, indent=2)})")
    rate = last_state['uplink_hz']
    ground = "ground side transmitting" if rate > 60 else "ground side likely idle"
    ok(f"uplink_hz={rate} ({ground}; full-rate target ~75)")

    # 3. /channels sane shape.
    #
    # CH1–4 are stick channels and stay within standard CRSF range
    # 172..1811 (988..2012 µs). CH5–12 are switches in 6-position mode.
    # CH13–16 in ELRS Hybrid mode (rf_mode 24, our config) carry raw
    # 11-bit data and can hit 0..2047 — the API exposes raw values,
    # consumers do their own mapping.
    code, ch = get("/channels")
    if not ch["crsf"] or len(ch["crsf"]) != 16:
        fail(f"/channels missing 16 channels: {ch}")
    for i, v in enumerate(ch["crsf"]):
        if not (0 <= v <= 2047):
            fail(f"crsf channel {i+1} outside 11-bit range: {v}")
    for i, v in enumerate(ch["crsf"][:4]):
        if not (172 <= v <= 1811):
            fail(f"stick channel {i+1} outside CRSF stick range: {v}")
    if ch["age_ms"] is None or ch["age_ms"] >= 200:
        fail(f"/channels stale: age_ms={ch['age_ms']}")
    ok(f"/channels age_ms={ch['age_ms']} sticks_us={ch['us'][:4]}")

    # 4. /link
    code, lk = get("/link")
    if lk["link"]:
        l = lk["link"]
        if not (0 <= l["uplink_lq"] <= 100):
            fail(f"uplink_lq out of range: {l['uplink_lq']}")
        ok(f"/link rf_mode={l['rf_mode']} uplink_lq={l['uplink_lq']} "
           f"rssi={l['uplink_rssi_ant1']}dBm tx_pwr_idx={l['uplink_tx_power']}")
    else:
        print("  WARN: no link stats yet (downlink slow; try again in a few seconds)")

    # 5. POST flight_mode increments bytes_out
    code, before = get("/state")
    code2, body = post("/telemetry/flight_mode", {"text": "DRONE-HELLO"})
    if code2 != 202:
        fail(f"POST /telemetry/flight_mode failed: {code2} {body}")
    time.sleep(0.5)
    code, after = get("/state")
    if after["bytes_out"] <= before["bytes_out"]:
        fail(f"bytes_out did not increase ({before['bytes_out']} -> {after['bytes_out']})")
    ok(f"posted flight_mode, bytes_out {before['bytes_out']} -> {after['bytes_out']}")

    # 6. Rate limit kicks in on burst
    print("\n  bursting 20 POSTs to verify rate limit ...")
    statuses = []
    for _ in range(20):
        c, _b = post("/telemetry/flight_mode", {"text": "BURST"})
        statuses.append(c)
    accepted = sum(1 for s in statuses if s == 202)
    rate_limited = sum(1 for s in statuses if s == 429)
    if rate_limited == 0:
        fail(f"rate limit never tripped: {statuses}")
    ok(f"burst: {accepted} accepted / {rate_limited} rate-limited")

    print("\nAll integration checks passed.")


if __name__ == "__main__":
    main()
