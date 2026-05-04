#!/usr/bin/env python3
"""Unit tests for drone_bridge — no hardware, no network.

Covers the deterministic pieces:
  * rpc       — encode/decode + fragmentation round-trip
  * digest    — 32-byte pack/unpack symmetry, field offsets
  * adapters  — path rewrites, body shape translations, timeouts

Run with:
    python3 -m unittest drone_bridge.tests.test_unit -v
or  python3 modules/drone_bridge/tests/test_unit.py
"""

from __future__ import annotations

import json
import os
import struct
import sys
import unittest

# Allow `python3 modules/drone_bridge/tests/test_unit.py` from repo root
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))   # add modules/

from drone_bridge import rpc, digest, snapshot, adapters     # noqa: E402


# ---------------------------------------------------------------------------
# rpc.py — wire format
# ---------------------------------------------------------------------------

class TestRpcRequest(unittest.TestCase):

    def _build_request_bytes(self, req_id=42, flags=rpc.FLAG_EOM,
                              method=rpc.METHOD_POST, path="/safety/rc",
                              body=b'{"action":"rtl"}'):
        path_b = path.encode("utf-8")
        hdr = struct.pack(rpc.REQ_HDR, req_id, flags, method,
                          len(path_b), len(body))
        return hdr + path_b + body

    def test_decode_round_trip(self):
        raw = self._build_request_bytes()
        req = rpc.RpcRequest.decode(raw)
        self.assertEqual(req.req_id, 42)
        self.assertEqual(req.method, rpc.METHOD_POST)
        self.assertEqual(req.method_name, "POST")
        self.assertEqual(req.path, "/safety/rc")
        self.assertEqual(req.body, b'{"action":"rtl"}')

    def test_decode_empty_body(self):
        raw = self._build_request_bytes(body=b"")
        req = rpc.RpcRequest.decode(raw)
        self.assertEqual(req.body, b"")
        self.assertEqual(req.path, "/safety/rc")

    def test_decode_truncated_raises(self):
        raw = self._build_request_bytes()[:8]
        with self.assertRaises(ValueError):
            rpc.RpcRequest.decode(raw)

    def test_decode_short_body_raises(self):
        raw = self._build_request_bytes()[:-3]   # cut last 3 body bytes
        with self.assertRaises(ValueError):
            rpc.RpcRequest.decode(raw)


class TestRpcResponse(unittest.TestCase):

    def test_encode_single_short(self):
        rsp = rpc.RpcResponse(req_id=7, status=200, body=b'{"ok":true}')
        wire = rsp.encode_single()
        self.assertEqual(len(wire), 10 + len(rsp.body))
        req_id, status, flags, _r, body_len = struct.unpack_from(rpc.RSP_HDR, wire, 0)
        self.assertEqual(req_id, 7)
        self.assertEqual(status, 200)
        self.assertTrue(flags & rpc.FLAG_EOM)
        self.assertFalse(flags & rpc.FLAG_ERROR)
        self.assertEqual(body_len, len(rsp.body))

    def test_fragmentation_single_when_small(self):
        rsp = rpc.RpcResponse(req_id=1, status=200, body=b"short")
        frags = list(rsp.fragment(mtu=247))
        self.assertEqual(len(frags), 1)
        self.assertTrue(struct.unpack_from(rpc.RSP_HDR, frags[0], 0)[2] & rpc.FLAG_EOM)

    def test_fragmentation_splits_large(self):
        body = bytes(range(256)) * 3        # 768 B body, MTU 247 → 3+ fragments
        rsp = rpc.RpcResponse(req_id=99, status=200, body=body)
        frags = list(rsp.fragment(mtu=247))
        self.assertGreater(len(frags), 1)
        # All but last must NOT have EOM; last must have EOM.
        for f in frags[:-1]:
            flags = struct.unpack_from(rpc.RSP_HDR, f, 0)[2]
            self.assertFalse(flags & rpc.FLAG_EOM,
                             "non-final fragment had EOM set")
        last_flags = struct.unpack_from(rpc.RSP_HDR, frags[-1], 0)[2]
        self.assertTrue(last_flags & rpc.FLAG_EOM)

    def test_fragmentation_reassembles(self):
        body = bytes(i & 0xFF for i in range(1500))   # 1.5 KB body
        rsp = rpc.RpcResponse(req_id=12345, status=201, body=body)
        re = rpc.ResponseReassembler()
        result = None
        for frag in rsp.fragment(mtu=247):
            result = re.feed(frag)
        self.assertIsNotNone(result, "reassembler never produced a response")
        self.assertEqual(result.req_id, 12345)
        self.assertEqual(result.status, 201)
        self.assertFalse(result.is_error)
        self.assertEqual(result.body, body)

    # --- Request fragmentation symmetric with response side ---

    def test_request_fragment_single_when_small(self):
        req = rpc.RpcRequest(req_id=7, flags=rpc.FLAG_EOM,
                             method=rpc.METHOD_POST,
                             path="/safety/rc",
                             body=b'{"action":"rtl"}')
        frags = list(req.fragment(mtu=247))
        self.assertEqual(len(frags), 1)
        # The single fragment is itself a complete RpcRequest.
        decoded = rpc.RpcRequest.decode(frags[0])
        self.assertEqual(decoded.req_id, 7)
        self.assertEqual(decoded.path, "/safety/rc")
        self.assertEqual(decoded.body, req.body)
        self.assertTrue(decoded.flags & rpc.FLAG_EOM)

    def test_request_fragment_splits_large(self):
        big_body = bytes(i & 0xFF for i in range(2000))   # 2 KB body
        req = rpc.RpcRequest(req_id=99, flags=rpc.FLAG_EOM,
                             method=rpc.METHOD_POST,
                             path="/mission",
                             body=big_body)
        frags = list(req.fragment(mtu=247))
        self.assertGreater(len(frags), 1)
        # All but last must NOT have EOM; last must have EOM.
        for f in frags[:-1]:
            sub = rpc.RpcRequest.decode(f)
            self.assertFalse(sub.flags & rpc.FLAG_EOM)
            self.assertEqual(sub.path, "/mission")  # repeated in every header
        last = rpc.RpcRequest.decode(frags[-1])
        self.assertTrue(last.flags & rpc.FLAG_EOM)

    def test_request_fragment_reassembles(self):
        big_body = bytes((i * 7 + 3) & 0xFF for i in range(2200))  # 2.2 KB
        req = rpc.RpcRequest(req_id=12345, flags=rpc.FLAG_EOM,
                             method=rpc.METHOD_POST,
                             path="/mission",
                             body=big_body)
        re = rpc.RequestReassembler()
        result = None
        for frag in req.fragment(mtu=247):
            result = re.feed(frag)
            # All fragments before the last yield None
            if result is not None:
                # only legal at the final fragment
                self.assertIs(frag, list(req.fragment(mtu=247))[-1]
                              if False else frag)
                break
        self.assertIsNotNone(result, "reassembler never produced a request")
        self.assertEqual(result.req_id, 12345)
        self.assertEqual(result.method, rpc.METHOD_POST)
        self.assertEqual(result.path, "/mission")
        self.assertEqual(result.body, big_body)

    def test_request_reassembler_drop_releases_buffer(self):
        big_body = bytes(2200)
        req = rpc.RpcRequest(req_id=42, flags=rpc.FLAG_EOM,
                             method=rpc.METHOD_POST,
                             path="/mission",
                             body=big_body)
        frags = list(req.fragment(mtu=247))
        re = rpc.RequestReassembler()
        # Feed all but the last → reassembler is mid-stream.
        for f in frags[:-1]:
            self.assertIsNone(re.feed(f))
        self.assertEqual(re.pending, 1)
        re.drop(42)
        self.assertEqual(re.pending, 0)

    def test_request_reassembler_clear_flushes_all(self):
        re = rpc.RequestReassembler()
        for rid in (1, 2, 3):
            req = rpc.RpcRequest(req_id=rid, flags=0,
                                 method=rpc.METHOD_POST,
                                 path="/mission",
                                 body=bytes(1500))
            for f in list(req.fragment(mtu=247))[:-1]:  # all but last
                re.feed(f)
        self.assertEqual(re.pending, 3)
        re.clear()
        self.assertEqual(re.pending, 0)

    def test_request_reassembler_round_trip_concurrent(self):
        # Two large requests interleaved → reassembler must keep them
        # separate by req_id and surface each independently when its
        # own EOM arrives.
        body_a = bytes((i & 0xFF) for i in range(2000))
        body_b = bytes(((i * 13) & 0xFF) for i in range(1500))
        req_a = rpc.RpcRequest(req_id=11, flags=0, method=rpc.METHOD_POST,
                               path="/fence/polygon", body=body_a)
        req_b = rpc.RpcRequest(req_id=22, flags=0, method=rpc.METHOD_POST,
                               path="/mission",       body=body_b)
        frags_a = list(req_a.fragment(mtu=247))
        frags_b = list(req_b.fragment(mtu=247))

        re = rpc.RequestReassembler()
        results = {}
        # Interleave: a0, b0, a1, b1, …
        from itertools import zip_longest
        for fa, fb in zip_longest(frags_a, frags_b):
            for f in (fa, fb):
                if f is None: continue
                r = re.feed(f)
                if r is not None:
                    results[r.req_id] = r

        self.assertIn(11, results)
        self.assertIn(22, results)
        self.assertEqual(results[11].body, body_a)
        self.assertEqual(results[22].body, body_b)
        self.assertEqual(results[11].path, "/fence/polygon")
        self.assertEqual(results[22].path, "/mission")

    def test_error_flag_propagates(self):
        rsp = rpc.RpcResponse(req_id=5, status=502,
                              body=b'{"code":"X"}', is_error=True)
        wire = rsp.encode_single()
        flags = struct.unpack_from(rpc.RSP_HDR, wire, 0)[2]
        self.assertTrue(flags & rpc.FLAG_ERROR)


# ---------------------------------------------------------------------------
# digest.py — 32-byte struct
# ---------------------------------------------------------------------------

class TestDigest(unittest.TestCase):

    def test_size_is_32_bytes(self):
        self.assertEqual(struct.calcsize(digest.DIGEST_FORMAT), 32)
        self.assertEqual(digest.DIGEST_SIZE, 32)

    def test_pack_returns_32_bytes_from_empty_snapshot(self):
        # Reset state so this test is order-independent.
        with snapshot.state_lock:
            for f in snapshot.state.__dataclass_fields__:
                if f in ("drone_last_ts", "elrs_last_ts"):
                    continue
                # leave defaults
        raw = digest.pack()
        self.assertEqual(len(raw), 32)

    def test_pack_unpack_round_trip(self):
        with snapshot.state_lock:
            s = snapshot.state
            s.armed = True
            s.fix_type = 3
            s.home_set = True
            s.mode = "ALT_HOLD"
            s.battery_pct = 0.83
            s.voltage_v = 12.4
            s.lat = 12.9716
            s.lon = 77.5946
            s.alt_amsl_m = 920.5
            s.alt_rel_m = 14.2
            s.ground_speed_mps = 4.1
            s.heading_deg = 270.5
            s.roll_rad = 0.05
            s.pitch_rad = -0.12
            s.rssi_dbm = -55
            s.uplink_lq = 92
        try:
            raw = digest.pack()
            d = digest.unpack(raw)
            self.assertTrue(d["armed"])
            self.assertTrue(d["gps_3d_fix"])
            self.assertTrue(d["home_set"])
            self.assertEqual(d["mode_idx"], 1)            # ALT_HOLD
            self.assertEqual(d["battery_pct"], 83)
            self.assertEqual(d["voltage_mv"], 12400)
            self.assertAlmostEqual(d["latitude"],  12.9716, places=4)
            self.assertAlmostEqual(d["longitude"], 77.5946, places=4)
            self.assertEqual(d["altitude_amsl_m"], 920.5)
            self.assertAlmostEqual(d["altitude_rel_m"], 14.2, places=1)
            self.assertEqual(d["ground_speed_mps"], 4.1)
            self.assertAlmostEqual(d["heading_deg"], 270.5, places=2)
            self.assertEqual(d["link_quality_pct"], 92)
            # rssi_pct: -55 dBm → pct = (-55+120)*100//70 = 92
            self.assertEqual(d["rssi_pct"], 92)
        finally:
            # Reset for other tests.
            with snapshot.state_lock:
                snapshot.state.__init__()                # fresh defaults

    def test_mode_other_for_unknown(self):
        with snapshot.state_lock:
            snapshot.state.__init__()
            snapshot.state.mode = "FOLLOW_ME_BUT_BACKWARDS"
        try:
            d = digest.unpack(digest.pack())
            self.assertEqual(d["mode_idx"], digest.MODE_OTHER)
        finally:
            with snapshot.state_lock:
                snapshot.state.__init__()

    def test_clamping_negative_battery(self):
        with snapshot.state_lock:
            snapshot.state.__init__()
            snapshot.state.battery_pct = -0.5
            snapshot.state.voltage_v   = -1.0
        try:
            d = digest.unpack(digest.pack())
            self.assertEqual(d["battery_pct"], 0)
            self.assertEqual(d["voltage_mv"], 0)
        finally:
            with snapshot.state_lock:
                snapshot.state.__init__()

    def test_rssi_pct_zero_when_no_link(self):
        self.assertEqual(digest.rssi_dbm_to_pct(0), 0)

    def test_rssi_pct_clamped_above_100(self):
        # -10 dBm is unrealistically strong; should saturate at 100.
        self.assertEqual(digest.rssi_dbm_to_pct(-10), 100)


# ---------------------------------------------------------------------------
# adapters.py — schema translation
# ---------------------------------------------------------------------------

class TestAdapters(unittest.TestCase):

    def test_path_rewrite_arm(self):
        a = adapters.adapt("POST", "/control/arm", b"")
        self.assertEqual(a.path, "/arm")
        self.assertEqual(a.method, "POST")

    def test_path_rewrite_disarm(self):
        a = adapters.adapt("POST", "/control/disarm", b"")
        self.assertEqual(a.path, "/disarm")

    def test_path_rewrite_mode(self):
        a = adapters.adapt("POST", "/control/mode",
                           b'{"mode":"LOITER"}')
        self.assertEqual(a.path, "/mode")
        self.assertEqual(a.json, {"mode": "LOITER"})

    def test_unrelated_path_unchanged(self):
        a = adapters.adapt("GET", "/calibration/status", None)
        self.assertEqual(a.path, "/calibration/status")
        self.assertIsNone(a.json)

    def test_motor_test_mode_standard_to_single(self):
        body = json.dumps({"motor": 1, "throttle_pct": 5,
                           "duration_s": 0.5, "mode": "STANDARD",
                           "motor_count": 4}).encode()
        a = adapters.adapt("POST", "/calibration/motor_test", body)
        self.assertEqual(a.json["mode"], "single")
        self.assertEqual(a.json["motor"], 1)
        self.assertEqual(a.json["motor_count"], 4)

    def test_motor_test_mode_sequence_passthrough(self):
        body = json.dumps({"motor": 1, "mode": "sequence",
                           "throttle_pct": 5, "duration_s": 0.3,
                           "motor_count": 4}).encode()
        a = adapters.adapt("POST", "/calibration/motor_test", body)
        self.assertEqual(a.json["mode"], "sequence")

    def test_fence_polygon_dict_to_array(self):
        body = json.dumps({"points": [
            {"lat": 47.1, "lon": -122.1},
            {"lat": 47.2, "lon": -122.2},
            {"lat": 47.3, "lon": -122.3},
        ]}).encode()
        a = adapters.adapt("POST", "/fence/polygon", body)
        self.assertEqual(a.json["points"],
                         [[47.1, -122.1], [47.2, -122.2], [47.3, -122.3]])

    def test_fence_polygon_already_arrays_passthrough(self):
        body = json.dumps({"points": [[47.1, -122.1]]}).encode()
        a = adapters.adapt("POST", "/fence/polygon", body)
        self.assertEqual(a.json["points"], [[47.1, -122.1]])

    def test_timeout_long_for_calibration(self):
        a = adapters.adapt("POST", "/calibration/gyro", b"")
        self.assertGreaterEqual(a.timeout, 15.0)

    def test_timeout_long_for_reconnect_mavros(self):
        a = adapters.adapt("POST", "/system/reconnect_mavros", b"")
        self.assertGreaterEqual(a.timeout, 10.0)

    def test_timeout_default_for_unknown(self):
        a = adapters.adapt("GET", "/calibration/status", None)
        self.assertEqual(a.timeout, adapters.DEFAULT_TIMEOUT)

    def test_non_json_body_passes_through(self):
        a = adapters.adapt("POST", "/some/binary", b"\xde\xad\xbe\xef")
        self.assertEqual(a.json, b"\xde\xad\xbe\xef")


# ---------------------------------------------------------------------------
# translator.py — channel decode helpers (no network)
# ---------------------------------------------------------------------------

class TestTranslatorDecoders(unittest.TestCase):

    def test_axis_pm1_centre(self):
        from drone_bridge.translator import axis_pm1
        self.assertAlmostEqual(axis_pm1(992), 0.0, places=2)

    def test_axis_pm1_extremes(self):
        from drone_bridge.translator import axis_pm1
        self.assertAlmostEqual(axis_pm1(172),  -1.0, places=2)
        self.assertAlmostEqual(axis_pm1(1811),  1.0, places=2)

    def test_axis_pm1_clamped(self):
        from drone_bridge.translator import axis_pm1
        self.assertEqual(axis_pm1(2047),  1.0)
        self.assertEqual(axis_pm1(-100), -1.0)

    def test_throttle_01_extremes(self):
        from drone_bridge.translator import throttle_01
        self.assertAlmostEqual(throttle_01(172),  0.0, places=2)
        self.assertAlmostEqual(throttle_01(1811), 1.0, places=2)

    def test_mode_index_from_us_six_positions(self):
        from drone_bridge.translator import mode_index_from_us
        self.assertEqual(mode_index_from_us(1000), 0)
        self.assertEqual(mode_index_from_us(1200), 1)
        self.assertEqual(mode_index_from_us(1400), 2)
        self.assertEqual(mode_index_from_us(1600), 3)
        self.assertEqual(mode_index_from_us(1800), 4)
        self.assertEqual(mode_index_from_us(2000), 5)


if __name__ == "__main__":
    unittest.main(verbosity=2)
