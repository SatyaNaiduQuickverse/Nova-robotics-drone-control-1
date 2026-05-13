#!/usr/bin/env python3
"""FC-sourced CRSF 0x02 GPS producer @ 5 Hz.

Composes /telemetry/gps (lat/lon/altitude/satellites) with
/telemetry/vfr_hud (groundspeed/heading) — drone-control splits these
fields across two endpoints and the CRSF GPS frame needs both.

Pass-through semantics: zeros from FC become zeros on the wire. When
GPS pins aren't connected the FC reports fix_type=-1, lat/lon=0,
satellites=0 — these go out unchanged so the consumer sees a real
"no fix" signal rather than a fake position.

Frame layout (CRSF 0x02, all multibyte BE):
    [0xC8] [0x11] [0x02] [lat_e7 i32] [lon_e7 i32]
                         [gs_kmhx10 u16] [hdg_degx100 u16]
                         [alt_m+1000 u16] [satellites u8] [CRC]
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
PUSH_HZ = 5.0
PUSH_PERIOD = 1.0 / PUSH_HZ
LOG_PERIOD_S = 5.0
ALT_OFFSET = 1000   # CRSF GPS altitude = meters + 1000
MS_TO_KMH = 3.6     # groundspeed unit conversion


def crc8_dvbs2(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def build_gps_frame(lat_e7, lon_e7, gs_kmhx10, hdg_x100, alt_off, sats):
    payload = struct.pack(">iiHHHB",
                          int(lat_e7), int(lon_e7),
                          gs_kmhx10 & 0xFFFF,
                          hdg_x100 & 0xFFFF,
                          alt_off & 0xFFFF,
                          sats & 0xFF)
    assert len(payload) == 15
    crc = crc8_dvbs2(bytes([0x02]) + payload)
    return bytes([0xC8, 0x11, 0x02]) + payload + bytes([crc])


def encode_fc_gps(gps, vfr):
    """Compose CRSF 0x02 from FC gps + vfr_hud dicts. Returns None
    if either input is missing or doesn't have the required fields."""
    if not isinstance(gps, dict) or not isinstance(vfr, dict):
        return None
    lat   = gps.get("latitude")
    lon   = gps.get("longitude")
    alt   = gps.get("altitude")
    sats  = gps.get("satellites")
    gs_ms = vfr.get("groundspeed")        # m/s
    hdg   = vfr.get("heading")            # degrees, 0..360
    required = (lat, lon, alt, gs_ms, hdg)
    if not all(isinstance(x, (int, float)) for x in required):
        return None
    if not isinstance(sats, int):
        sats = 0

    lat_e7 = max(-2**31, min(2**31 - 1, int(round(lat * 1e7))))
    lon_e7 = max(-2**31, min(2**31 - 1, int(round(lon * 1e7))))
    # CRSF groundspeed is km/h × 10. FC vfr_hud gives m/s.
    gs_kmhx10 = max(0, min(0xFFFF, int(round(gs_ms * MS_TO_KMH * 10.0))))
    hdg_x100  = max(0, min(0xFFFF, int(round((hdg % 360.0) * 100.0))))
    # CRSF altitude is u16 with +1000 offset. Clamp to representable range
    # (-1000 .. +64535 metres). FC alt below -1000 (e.g. -17 m below sea-level
    # in odd locales) would clamp to 0 (= -1000m encoded).
    alt_off = max(0, min(0xFFFF, int(round(alt + ALT_OFFSET))))
    return build_gps_frame(lat_e7, lon_e7, gs_kmhx10, hdg_x100, alt_off, sats)


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
    pidfile = production_setup("gps")
    fc = FcClient()
    print(f"[{utc_stamp()}] fc_gps_producer: 0x02 @ {PUSH_HZ:.0f} Hz "
          f"source={fc.base_url}/telemetry/{{gps,vfr_hud}}", flush=True)

    next_tick = time.monotonic()
    last_log = time.monotonic()
    sent = 0
    skipped = 0
    last_fail_logged = None

    try:
        while True:
            # Two reads per cycle. If either fails, skip this cycle.
            gps = fc.get_gps()
            vfr = fc.get_vfr_hud()
            if gps is None or vfr is None:
                skipped += 1
                if fc.last_error != last_fail_logged:
                    print(f"  [{utc_stamp()}] FC unreachable: {fc.last_error}",
                          flush=True)
                    last_fail_logged = fc.last_error
            else:
                if last_fail_logged is not None:
                    print(f"  [{utc_stamp()}] FC reachable again", flush=True)
                    last_fail_logged = None
                frame = encode_fc_gps(gps, vfr)
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
                fix = gps.get("fix_type") if isinstance(gps, dict) else "?"
                ns = gps.get("satellites") if isinstance(gps, dict) else "?"
                print(f"  [{utc_stamp()}] sent={sent} skipped={skipped} "
                      f"fix_type={fix} sats={ns} "
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
