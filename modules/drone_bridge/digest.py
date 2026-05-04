"""32-byte CRSF telemetry digest packer.

Layout is fixed by drone-handoff PROMPT.md §2 Deliverable 2 — must NOT
be reordered without coordinated changes on the ground bridge:

    offset 0  : u8  flags  (bit 0 armed, bit 1 ekf_ok, bit 2 gps_3d_fix,
                            bit 3 home_set, bits 4-7 mode_idx)
    offset 1  : u8  battery_pct
    offset 2  : u16 voltage_mv          (little-endian)
    offset 4  : i32 latitude_e7
    offset 8  : i32 longitude_e7
    offset 12 : i16 altitude_amsl_dm    (decimeters above MSL)
    offset 14 : i16 altitude_rel_dm     (decimeters above home)
    offset 16 : u16 ground_speed_cm_s
    offset 18 : i16 heading_cdeg
    offset 20 : i16 roll_cdeg
    offset 22 : i16 pitch_cdeg
    offset 24 : u8  rssi_pct
    offset 25 : u8  link_quality_pct
    offset 26 : u16 reserved
    offset 28 : u32 monotonic_ms        (drone-side uptime, for stale detect)
"""

from __future__ import annotations

import math
import struct
import time

from . import snapshot


# --- Pack format mirrors PROMPT.md byte-for-byte. Do not reorder. ---------

DIGEST_FORMAT = "<BBHiihhHhhhBBHI"
DIGEST_SIZE   = 32
assert struct.calcsize(DIGEST_FORMAT) == DIGEST_SIZE, struct.calcsize(DIGEST_FORMAT)


# --- Mode index (4 bits, 16 slots) ----------------------------------------

MODE_OTHER = 15

MODE_INDEX = {
    "STABILIZE":    0,
    "ALT_HOLD":     1,
    "LOITER":       2,
    "AUTO":         3,
    "GUIDED":       4,
    "RTL":          5,
    "LAND":         6,
    "POSHOLD":      7,
    "BRAKE":        8,
    "THROW":        9,
    "AVOID_ADSB":  10,
    "GUIDED_NOGPS":11,
    "SMART_RTL":   12,
    "FLOWHOLD":    13,
    "FOLLOW":      14,
}


# --- Helpers --------------------------------------------------------------

def _clamp(value: float, lo: int, hi: int) -> int:
    """Round-half-to-even to int, then clamp. Avoids the 4.1*100 = 409
    truncation surprise that bites if you `int()` the multiplied value."""
    if math.isnan(value) or math.isinf(value):
        return 0
    v = int(round(value))
    return lo if v < lo else hi if v > hi else v


def rssi_dbm_to_pct(rssi_dbm: int) -> int:
    """Map ELRS RSSI (typically -50 to -130 dBm) to a 0..100 % bar.

    -120 dBm ≈ link dead → 0 %
    -50  dBm ≈ point-blank → 100 %
    rssi_dbm == 0 means "no link stats yet" → 0 % (not full bars).
    """
    if rssi_dbm == 0:
        return 0
    pct = (rssi_dbm + 120) * 100 // 70
    return max(0, min(100, pct))


def mode_to_index(mode: str) -> int:
    if not mode:
        return 0
    return MODE_INDEX.get(mode, MODE_OTHER)


# --- Pack -----------------------------------------------------------------

def pack() -> bytes:
    """Pack the current snapshot into the 32-byte digest.

    Always returns exactly DIGEST_SIZE bytes. Out-of-range values are
    clamped, not rejected — the wire format has no "invalid" representation
    for most fields, and the receiver checks `monotonic_ms` to detect
    staleness.
    """
    with snapshot.state_lock:
        s = snapshot.state

        flags = 0
        if s.armed:        flags |= 0x01
        # bit 1 ekf_ok — drone-control doesn't surface this; reserved for now
        if s.fix_type >= 3: flags |= 0x04
        if s.home_set:     flags |= 0x08
        flags |= (mode_to_index(s.mode) & 0x0F) << 4

        battery_pct = _clamp(s.battery_pct * 100, 0, 255)
        voltage_mv  = _clamp(s.voltage_v * 1000, 0, 0xFFFF)

        lat_e7 = _clamp(s.lat * 1e7, -2_147_483_648, 2_147_483_647)
        lon_e7 = _clamp(s.lon * 1e7, -2_147_483_648, 2_147_483_647)

        alt_amsl_dm = _clamp(s.alt_amsl_m * 10, -32_768, 32_767)
        alt_rel_dm  = _clamp(s.alt_rel_m  * 10, -32_768, 32_767)

        gs_cm_s    = _clamp(s.ground_speed_mps * 100, 0, 0xFFFF)
        heading_cd = _clamp(s.heading_deg * 100,    -32_768, 32_767)
        roll_cd    = _clamp(math.degrees(s.roll_rad)  * 100, -32_768, 32_767)
        pitch_cd   = _clamp(math.degrees(s.pitch_rad) * 100, -32_768, 32_767)

        rssi_pct = rssi_dbm_to_pct(s.rssi_dbm)
        lq_pct   = _clamp(s.uplink_lq, 0, 100)

        mono_ms = int(snapshot.uptime_s() * 1000) & 0xFFFFFFFF

    return struct.pack(
        DIGEST_FORMAT,
        flags, battery_pct, voltage_mv,
        lat_e7, lon_e7,
        alt_amsl_dm, alt_rel_dm,
        gs_cm_s, heading_cd, roll_cd, pitch_cd,
        rssi_pct, lq_pct,
        0,                # reserved u16
        mono_ms,
    )


def unpack(buf: bytes) -> dict:
    """Inverse of pack() — for tests and the JSON debug endpoint."""
    if len(buf) != DIGEST_SIZE:
        raise ValueError(f"digest must be {DIGEST_SIZE} bytes, got {len(buf)}")
    (flags, batt, mv, lat_e7, lon_e7, alt_amsl, alt_rel, gs, hd,
     roll, pitch, rssi, lq, _resv, mono_ms) = struct.unpack(DIGEST_FORMAT, buf)
    return {
        "flags": flags,
        "armed":     bool(flags & 0x01),
        "ekf_ok":    bool(flags & 0x02),
        "gps_3d_fix":bool(flags & 0x04),
        "home_set":  bool(flags & 0x08),
        "mode_idx":  (flags >> 4) & 0x0F,
        "battery_pct":     batt,
        "voltage_mv":      mv,
        "latitude":        lat_e7 / 1e7,
        "longitude":       lon_e7 / 1e7,
        "altitude_amsl_m": alt_amsl / 10,
        "altitude_rel_m":  alt_rel  / 10,
        "ground_speed_mps":gs / 100,
        "heading_deg":     hd / 100,
        "roll_deg":        roll / 100,
        "pitch_deg":       pitch / 100,
        "rssi_pct":        rssi,
        "link_quality_pct":lq,
        "monotonic_ms":    mono_ms,
    }
