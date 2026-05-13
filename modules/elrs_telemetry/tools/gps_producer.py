#!/usr/bin/env python3
"""H4 step 3 — CRSF 0x02 GPS producer @ 5 Hz (stubbed).

Pushes synthetic varying GPS values to the daemon's /telemetry/raw
endpoint. Stubbed mode (no FC dependency) for decoder validation.

Frame layout (CRSF spec, all multibyte fields BIG-ENDIAN):
    [0xC8]      sync (FC address)
    [0x11]      len = type(1) + payload(15) + crc(1) = 17
    [0x02]      type = GPS
    [payload]   15 bytes:
                  i32  latitude       degrees × 1e7
                  i32  longitude      degrees × 1e7
                  u16  groundspeed    km/h × 10
                  u16  heading        degrees × 100
                  u16  altitude       meters + 1000 offset (0 m → 1000)
                  u8   satellites
    [CRC8/DVB-S2 over (type + payload)]
    Total: 19 bytes wire

Variance schedule:
    lat          = 37.4275 + 0.001 * sin(t/10)    Mountain-View wobble
    lon          = -122.1697 + 0.001 * cos(t/10)
    groundspeed  = 5 + 5 * sin(t/15)              0..10 km/h
    heading      = (4.5 * t) % 360                4.5 deg/s rotation
    altitude     = 100 + 50 * sin(t/20)           50..150 m
    satellites   = 5 + int(t/2) % 11              steps through 5..15
"""
import datetime
import json
import math
import struct
import time
import urllib.request

URL = "http://localhost:5003/telemetry/raw"
PUSH_HZ = 5.0
PUSH_PERIOD = 1.0 / PUSH_HZ
ALT_OFFSET = 1000  # CRSF altitude is encoded as meters + 1000


def crc8_dvbs2(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def build_gps_frame(lat_e7, lon_e7, gs_kmhx10, heading_degx100, alt_m_offset, sats):
    """Pack a CRSF 0x02 GPS frame. Returns 19 raw bytes."""
    payload = struct.pack(">iiHHHB",
                          lat_e7,
                          lon_e7,
                          gs_kmhx10 & 0xFFFF,
                          heading_degx100 & 0xFFFF,
                          alt_m_offset & 0xFFFF,
                          sats & 0xFF)
    assert len(payload) == 15
    crc = crc8_dvbs2(bytes([0x02]) + payload)
    return bytes([0xC8, 0x11, 0x02]) + payload + bytes([crc])


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
    print(f"[{utc_stamp()}] gps_producer: 0x02 @ {PUSH_HZ:.0f} Hz to {URL}", flush=True)
    print(f"[{utc_stamp()}] stubbed: lat=37.4275±0.001  lon=-122.1697±0.001  "
          f"gs=0..10km/h  hdg=0..360 (4.5°/s)  alt=50..150m  sats=5..15",
          flush=True)

    t0 = time.monotonic()
    next_tick = time.monotonic()
    count = 0
    last_log = 0.0
    while True:
        t = time.monotonic() - t0
        lat = 37.4275 + 0.001 * math.sin(t / 10.0)
        lon = -122.1697 + 0.001 * math.cos(t / 10.0)
        gs_kmh = 5.0 + 5.0 * math.sin(t / 15.0)
        heading = (4.5 * t) % 360.0
        altitude = 100.0 + 50.0 * math.sin(t / 20.0)
        sats = 5 + int(t / 2.0) % 11

        lat_e7 = int(round(lat * 1e7))
        lon_e7 = int(round(lon * 1e7))
        gs_x10 = int(round(gs_kmh * 10.0))
        hdg_x100 = int(round(heading * 100.0))
        alt_off = int(round(altitude + ALT_OFFSET))

        frame = build_gps_frame(lat_e7, lon_e7, gs_x10, hdg_x100, alt_off, sats)
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
            print(f"  [{utc_stamp()}] t={t:5.1f}s  lat={lat:.6f}  lon={lon:.6f}  "
                  f"gs={gs_kmh:4.1f}km/h  hdg={heading:5.1f}°  alt={altitude:5.1f}m  "
                  f"sats={sats}  (sent {count})", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(f"[{utc_stamp()}] stopped", flush=True)
