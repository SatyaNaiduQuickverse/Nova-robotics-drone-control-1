#!/usr/bin/env python3
"""Drone-side stub producer for the new downlink Tx.

Fires CRSF RC_CHANNELS_PACKED frames at 50 Hz to elrs-telemetry's
/telemetry/raw endpoint. The daemon mux-encodes them as ch=1 and writes
to UART1 (NEW Ranger Tx). Tx broadcasts at packet rate → ground-side
bound Rx LED stays solid → dn_to_pi climbs at ~1.3 kB/s.

Channel pattern is gb_v1 FS_CHANNELS[] base, with CH9 (index 8) carrying
a counter mod 2048 so the ground-side decoder can see fresh values
flowing (proves NOT cached / stale).

This is an H4 stub — production producer will source values from FC
telemetry instead of constants. For now, it just keeps the bind
visibly alive.
"""
import time
import urllib.request
import json

# gb_v1 FS_CHANNELS pattern, 16 channels in CRSF units (172..1811)
FS = [
    172,   # CH1  throttle MIN
    992,   # CH2  roll center
    992,   # CH3  pitch center
    992,   # CH4  yaw center
    992,   # CH5  arm center (=disarmed)
    992,   # CH6  force-disarm center
    172,   # CH7  mode rotary idx 0
    172,   # CH8  ops bitfield 0
    992,   # CH9  counter slot (varies per frame)
    992, 992, 992, 992, 992, 992, 992,   # CH10..16 centered
]

import os
URL = "http://localhost:5003/telemetry/raw"
HZ  = float(os.environ.get("ELRS_PRODUCER_HZ", "50"))
PERIOD = 1.0 / HZ


def crc8_dvbs2(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def build_rc_frame(channels: list[int]) -> bytes:
    """Pack 16 × 11-bit channels LSB-first into a 26-byte CRSF
    RC_CHANNELS_PACKED frame (same layout the firmware uses for failsafe
    injection in gb_v1)."""
    bits = 0
    nbits = 0
    payload = bytearray()
    for ch in channels:
        bits |= (ch & 0x7FF) << nbits
        nbits += 11
        while nbits >= 8:
            payload.append(bits & 0xFF)
            bits >>= 8
            nbits -= 8
    assert len(payload) == 22  # 16 * 11 / 8 = 22
    # Sync byte 0xC8 (FC address) + length 0x18 (= type + payload + crc = 24)
    # + type 0x16 (RC_CHANNELS_PACKED) + 22-byte payload + CRC.
    crc = crc8_dvbs2(bytes([0x16]) + bytes(payload))
    return bytes([0xC8, 0x18, 0x16]) + bytes(payload) + bytes([crc])


def post(frame_hex: str) -> None:
    body = json.dumps({"hex": frame_hex}).encode()
    req = urllib.request.Request(URL, data=body, method="POST",
                                  headers={"Content-Type": "application/json"})
    try:
        urllib.request.urlopen(req, timeout=0.05).read()
    except Exception:
        pass  # transient — keep firing


def main() -> None:
    print(f"elrs_producer: firing CRSF RC at {HZ:.0f} Hz to {URL}", flush=True)
    counter = 0
    next_tick = time.monotonic()
    last_log = 0.0
    while True:
        channels = list(FS)
        channels[8] = counter & 0x7FF   # CH9 = counter mod 2048
        frame = build_rc_frame(channels)
        post(frame.hex())
        counter += 1

        # Soft 50 Hz pacing
        next_tick += PERIOD
        sleep = next_tick - time.monotonic()
        if sleep > 0:
            time.sleep(sleep)
        else:
            next_tick = time.monotonic()  # we fell behind, resync

        # Status log every 10 s
        now = time.monotonic()
        if now - last_log >= 10.0:
            last_log = now
            print(f"  [+{int(now):>5}s] {counter} frames sent  CH9={channels[8]}",
                  flush=True)


if __name__ == "__main__":
    main()
