"""Pluggable telemetry serializers.

Single canonical data model (`snapshot.Snapshot`) feeds N wire formats.
Each serializer is a thin adapter between the in-memory Snapshot and a
specific transport's expectations:

  DigestSerializer    — 32-byte CRSF-friendly bitfield, one wire frame
                        per BLE notify or per CRSF telem-uplink slot
  JsonFullSerializer  — fully-expanded JSON for HTTP debug consumers
                        (the /telemetry/digest/json endpoint and any
                        future REST-style telemetry consumer)

Why this exists: prior to this module, every transport had its own
hand-written serializer with its own copy of "how do I read battery_pct
out of the snapshot." Adding a single field meant touching every site.
With the Serializer protocol, you write one canonical Snapshot and
each transport plugs its serializer into the registry.

Adding a new serializer (e.g., for a future fiber-optic transport):

    class FiberFrameSerializer:
        wire_format = "fiber-v1"

        def serialize(self, snap: snapshot.Snapshot) -> bytes:
            # build whatever wire format the fiber link wants
            ...

        def deserialize(self, raw: bytes) -> dict:
            # for testing / round-trip verification
            ...

That's the whole contract. Register it where the transport wants it
(e.g., a new endpoint), and you've added telemetry support for the new
link without touching any other transport.

The wire-format-bytes-vs-dict split mirrors the actual usage:
  * Binary transports (BLE, CRSF, future fiber) want bytes
  * Debug/inspection consumers want JSON dicts
A single serializer can implement both, or two specialized ones can
exist side by side — the protocol allows either.
"""

from __future__ import annotations

import math
from typing import Protocol, runtime_checkable

from . import digest, snapshot


# --- Protocol ------------------------------------------------------------


@runtime_checkable
class Serializer(Protocol):
    """Common shape every serializer follows.

    Implementations either return bytes (binary wire format) or dicts
    (JSON-ish structure). Concrete classes pick one — the protocol
    accommodates both via a union return type at the call site.

    The `wire_format` attribute is a stable string identifier — useful
    for `/services` introspection and for clients that want to confirm
    they're decoding what they think they're decoding.
    """

    wire_format: str

    def serialize(self, snap: snapshot.Snapshot) -> bytes | dict:
        """Snapshot → wire format. Pure function; never mutates `snap`."""
        ...


# --- Concrete: 32-byte CRSF/BLE digest ----------------------------------


class DigestSerializer:
    """32-byte fixed-layout digest. Suitable for BLE notify frames,
    CRSF telemetry uplink slots, or anywhere a small fixed payload is
    cheaper than a JSON envelope.

    Wire format is documented in `digest.DIGEST_FORMAT` and must NOT
    be re-ordered without coordinated ground-side updates — the
    receiver assumes byte-exact layout.

    This class is a thin wrapper over `digest.pack()`/`digest.unpack()`
    so existing callers keep working; the serializer abstraction is
    additive, not a replacement.
    """

    wire_format = "digest-v1"

    def serialize(self, snap: snapshot.Snapshot) -> bytes:
        # digest.pack() reads from the global snapshot.state under its
        # own lock. To make this serializer pure (caller-provided snap),
        # we build the bytes from the explicit snapshot rather than
        # going through the global. Keeps the serializer testable and
        # deterministic — one snapshot in, one bytes out.
        return _pack_from(snap)

    def deserialize(self, buf: bytes) -> dict:
        """Inverse — for tests, round-trip verification, debug JSON."""
        return digest.unpack(buf)


def _pack_from(s: snapshot.Snapshot) -> bytes:
    """Same byte layout as digest.pack() but pulls from an explicit
    snapshot instead of the global. Keeps DigestSerializer pure.

    If the layout in digest.py changes, change this function in lock
    step. The two are intentionally parallel — if you find yourself
    wishing they shared code, push them both through the canonical
    Snapshot model and delete the global-state path.
    """
    import struct as _s

    flags = 0
    if s.armed:        flags |= 0x01
    # bit 1 ekf_ok — not currently surfaced; reserved
    if s.fix_type >= 3: flags |= 0x04
    if s.home_set:     flags |= 0x08
    flags |= (digest.mode_to_index(s.mode) & 0x0F) << 4

    battery_pct = digest._clamp(s.battery_pct * 100, 0, 255)
    voltage_mv  = digest._clamp(s.voltage_v * 1000, 0, 0xFFFF)
    lat_e7 = digest._clamp(s.lat * 1e7, -2_147_483_648, 2_147_483_647)
    lon_e7 = digest._clamp(s.lon * 1e7, -2_147_483_648, 2_147_483_647)
    alt_amsl_dm = digest._clamp(s.alt_amsl_m * 10, -32_768, 32_767)
    alt_rel_dm  = digest._clamp(s.alt_rel_m  * 10, -32_768, 32_767)
    gs_cm_s    = digest._clamp(s.ground_speed_mps * 100, 0, 0xFFFF)
    heading_cd = digest._clamp(s.heading_deg * 100, -32_768, 32_767)
    roll_cd    = digest._clamp(math.degrees(s.roll_rad)  * 100, -32_768, 32_767)
    pitch_cd   = digest._clamp(math.degrees(s.pitch_rad) * 100, -32_768, 32_767)
    rssi_pct   = digest.rssi_dbm_to_pct(s.rssi_dbm)
    lq_pct     = digest._clamp(s.uplink_lq, 0, 100)
    mono_ms    = int(snapshot.uptime_s() * 1000) & 0xFFFFFFFF

    return _s.pack(
        digest.DIGEST_FORMAT,
        flags, battery_pct, voltage_mv,
        lat_e7, lon_e7,
        alt_amsl_dm, alt_rel_dm,
        gs_cm_s, heading_cd, roll_cd, pitch_cd,
        rssi_pct, lq_pct,
        0,
        mono_ms,
    )


# --- Concrete: full JSON for debug/HTTP consumers -----------------------


class JsonFullSerializer:
    """Fully-expanded JSON shape for debug HTTP consumers and any
    REST-style telemetry path. Mirrors what main.py used to inline
    in /telemetry/digest/json.

    Returns a dict (not bytes) — the HTTP layer JSON-encodes itself.
    Any field added to `snapshot.Snapshot` should also be exposed
    here in human-readable form (degrees not radians, V not mV, etc.)
    so HTTP debugging stays useful without manual decoding.
    """

    wire_format = "json-full-v1"

    def serialize(self, snap: snapshot.Snapshot) -> dict:
        return {
            "connected":    snap.connected,
            "armed":        snap.armed,
            "mode":         snap.mode,
            "voltage":      round(snap.voltage_v, 2),
            "battery_pct":  round(snap.battery_pct * 100, 1),
            "lat":          snap.lat,
            "lon":          snap.lon,
            "alt_amsl":     round(snap.alt_amsl_m, 2),
            "alt_rel":      round(snap.alt_rel_m, 2),
            "ground_speed": round(snap.ground_speed_mps, 2),
            "heading":      round(snap.heading_deg, 2),
            "roll_deg":     round(math.degrees(snap.roll_rad), 2),
            "pitch_deg":    round(math.degrees(snap.pitch_rad), 2),
            "fix_type":     snap.fix_type,
            "home_set":     snap.home_set,
            "rssi_dbm":     snap.rssi_dbm,
            "uplink_lq":    snap.uplink_lq,
            "drone_age_ms": snapshot.age_ms(snap.drone_last_ts),
            "elrs_age_ms":  snapshot.age_ms(snap.elrs_last_ts),
        }


# --- Convenience: shared instances --------------------------------------
# Module-level singletons for the standard cases. Callers don't need to
# instantiate; they just import. (Custom subclasses can still be
# instantiated as needed.)

DIGEST = DigestSerializer()
JSON_FULL = JsonFullSerializer()
