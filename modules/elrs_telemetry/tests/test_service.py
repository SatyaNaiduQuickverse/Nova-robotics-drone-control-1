#!/usr/bin/env python3
"""Unit tests for the ELRS telemetry service. No hardware required.

Run from anywhere:
    python3 modules/elrs_telemetry/tests/test_service.py
"""

import os
import sys
import time
import unittest
from unittest.mock import MagicMock, patch

# Ensure we import crsf.py / service.py from the parent dir, not whatever
# happens to be on sys.path.
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))


import crsf  # noqa: E402


# --- CRSF codec round-trips ------------------------------------------------

class TestCRSFCodec(unittest.TestCase):
    def test_battery_frame_round_trip(self):
        frame = crsf.make_battery(12.5, 3.2, 1234, 75)
        parser = crsf.CRSFParser()
        out = list(parser.feed(frame))
        self.assertEqual(len(out), 1)
        addr, ftype, payload = out[0]
        self.assertEqual(ftype, crsf.FRAME_BATTERY_SENSOR)
        v = int.from_bytes(payload[0:2], 'big') / 10.0
        a = int.from_bytes(payload[2:4], 'big') / 10.0
        mah = (payload[4] << 16) | (payload[5] << 8) | payload[6]
        self.assertAlmostEqual(v, 12.5, places=1)
        self.assertAlmostEqual(a, 3.2, places=1)
        self.assertEqual(mah, 1234)
        self.assertEqual(payload[7], 75)

    def test_flight_mode_frame_round_trip(self):
        frame = crsf.make_flight_mode("ALT_HOLD")
        parser = crsf.CRSFParser()
        out = list(parser.feed(frame))
        self.assertEqual(len(out), 1)
        _, ftype, payload = out[0]
        self.assertEqual(ftype, crsf.FRAME_FLIGHT_MODE)
        self.assertEqual(payload.rstrip(b'\x00').decode(), "ALT_HOLD")

    def test_rc_channels_pack_unpack(self):
        ch_in = [172, 300, 500, 700, 900, 992, 1100, 1300,
                 1500, 1700, 1811, 992, 992, 992, 992, 992]
        unpacked = crsf.unpack_channels(crsf.pack_channels(ch_in))
        self.assertEqual(ch_in, unpacked)

    def test_us_round_trip(self):
        for us in (988, 1000, 1500, 1900, 2012):
            self.assertLessEqual(abs(us - crsf.crsf_to_us(crsf.us_to_crsf(us))), 1)

    def test_parser_skips_diagnostic_lines(self):
        diag = b"# stats uart_to_usb=100 usb_to_uart=0 uptime_s=12\n"
        battery = crsf.make_battery(11.1)
        out = list(crsf.CRSFParser().feed(diag + battery))
        self.assertEqual(len(out), 1)
        self.assertEqual(out[0][1], crsf.FRAME_BATTERY_SENSOR)

    def test_parser_skips_garbage(self):
        garbage = bytes([0xFF, 0xAA, 0x55, 0x12, 0x34])
        battery = crsf.make_battery(11.1)
        out = list(crsf.CRSFParser().feed(garbage + battery + garbage))
        self.assertEqual(len(out), 1)

    def test_parser_chunked_input(self):
        battery = crsf.make_battery(11.1)
        parser = crsf.CRSFParser()
        # Feed one byte at a time — must still yield exactly one frame.
        out = []
        for b in battery:
            out.extend(parser.feed(bytes([b])))
        self.assertEqual(len(out), 1)
        self.assertEqual(out[0][1], crsf.FRAME_BATTERY_SENSOR)

    def test_parser_rejects_bad_crc(self):
        battery = bytearray(crsf.make_battery(11.1))
        battery[-1] ^= 0xFF  # corrupt CRC
        out = list(crsf.CRSFParser().feed(bytes(battery)))
        self.assertEqual(len(out), 0)


# --- Service HTTP API (with mocked serial) ---------------------------------

class TestServiceAPI(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Patch serial.Serial before importing service so the reader thread
        # (if it ever starts) gets a mock. We never call service.main(), so
        # the thread is dormant — the patch is belt-and-suspenders.
        cls._serial_patch = patch('serial.Serial', return_value=MagicMock())
        cls._serial_patch.start()
        import service  # noqa: E402
        cls.service = service
        cls.client = service.app.test_client()
        # Pre-load a mock serial handle so writer-path tests don't drop frames.
        service._serial_handle["ser"] = MagicMock()

    @classmethod
    def tearDownClass(cls):
        cls._serial_patch.stop()

    def setUp(self):
        # Refill the rate-limit bucket between tests.
        self.service.tx_bucket.tokens = float(self.service.tx_bucket.capacity)
        # Reset link state so test order is independent.
        with self.service.state_lock:
            self.service.state.channels_crsf = None
            self.service.state.channels_us = None
            self.service.state.last_rc_ts = None
            self.service.state.link = None
            self.service.state.last_ls_ts = None

    def test_healthz(self):
        r = self.client.get("/healthz")
        self.assertEqual(r.status_code, 200)
        body = r.get_json()
        self.assertTrue(body["ok"])
        self.assertIn("uptime_s", body)

    def test_channels_initially_stale(self):
        r = self.client.get("/channels")
        self.assertEqual(r.status_code, 200)
        body = r.get_json()
        self.assertTrue(body["stale"])

    def test_channels_after_simulated_frame(self):
        # Simulate a frame arriving by calling the dispatcher directly.
        ch_in = [992] * 16
        ch_in[0] = 1500
        self.service._handle_frame(
            crsf.ADDR_FLIGHT_CTRL,
            crsf.FRAME_RC_CHANNELS_PACKED,
            crsf.pack_channels(ch_in),
        )
        r = self.client.get("/channels")
        body = r.get_json()
        self.assertFalse(body["stale"])
        self.assertEqual(body["crsf"][0], 1500)
        self.assertEqual(len(body["us"]), 16)

    def test_link_after_simulated_frame(self):
        # Construct a LINK_STATS payload with all-known values.
        payload = bytes([60, 65, 95, 5, 0, 24, 3, 70, 80, 4])
        self.service._handle_frame(
            crsf.ADDR_RX_MODULE, crsf.FRAME_LINK_STATS, payload,
        )
        r = self.client.get("/link")
        body = r.get_json()
        self.assertEqual(body["link"]["uplink_lq"], 95)
        self.assertEqual(body["link"]["rf_mode"], 24)
        self.assertEqual(body["link"]["uplink_rssi_ant1"], -60)

    def test_post_battery_enqueues_valid_frame(self):
        before = self.service.tx_queue.qsize()
        r = self.client.post("/telemetry/battery", json={
            "voltage": 12.4, "current": 1.2, "mah": 100, "percent": 80,
        })
        self.assertEqual(r.status_code, 202)
        body = r.get_json()
        self.assertGreater(body["bytes"], 0)
        # Bytes must parse back as a battery frame.
        out = list(crsf.CRSFParser().feed(bytes.fromhex(body["hex"])))
        self.assertEqual(out[0][1], crsf.FRAME_BATTERY_SENSOR)
        self.assertGreaterEqual(self.service.tx_queue.qsize(), before)

    def test_post_flight_mode_enqueues(self):
        r = self.client.post("/telemetry/flight_mode", json={"text": "TEST"})
        self.assertEqual(r.status_code, 202)
        out = list(crsf.CRSFParser().feed(bytes.fromhex(r.get_json()["hex"])))
        self.assertEqual(out[0][1], crsf.FRAME_FLIGHT_MODE)
        self.assertEqual(out[0][2].rstrip(b'\x00').decode(), "TEST")

    def test_post_battery_rejects_out_of_range(self):
        r = self.client.post("/telemetry/battery", json={"voltage": -5})
        self.assertEqual(r.status_code, 400)
        r = self.client.post("/telemetry/battery", json={"voltage": 999})
        self.assertEqual(r.status_code, 400)

    def test_post_flight_mode_rejects_garbage(self):
        r = self.client.post("/telemetry/flight_mode", json={"text": ""})
        self.assertEqual(r.status_code, 400)
        r = self.client.post("/telemetry/flight_mode", json={"text": "x" * 64})
        self.assertEqual(r.status_code, 400)
        r = self.client.post("/telemetry/flight_mode", json={"text": "weird\x01char"})
        self.assertEqual(r.status_code, 400)

    def test_post_attitude_enqueues(self):
        r = self.client.post("/telemetry/attitude",
                             json={"pitch": 0.1, "roll": -0.2, "yaw": 1.5})
        self.assertEqual(r.status_code, 202)
        out = list(crsf.CRSFParser().feed(bytes.fromhex(r.get_json()["hex"])))
        self.assertEqual(out[0][1], crsf.FRAME_ATTITUDE)

    def test_post_gps_enqueues(self):
        r = self.client.post("/telemetry/gps", json={
            "lat": 12.9716, "lon": 77.5946, "alt": 100,
            "speed": 5.5, "heading": 180.0, "satellites": 12,
        })
        self.assertEqual(r.status_code, 202)
        out = list(crsf.CRSFParser().feed(bytes.fromhex(r.get_json()["hex"])))
        self.assertEqual(out[0][1], crsf.FRAME_GPS)

    def test_post_raw_passes_through(self):
        bat = crsf.make_battery(11.0)
        r = self.client.post("/telemetry/raw", json={"hex": bat.hex()})
        self.assertEqual(r.status_code, 202)

    def test_post_raw_rejects_bad_sync(self):
        bad = bytes([0x42, 0x04, 0x08, 0x00, 0x00, 0x00])
        r = self.client.post("/telemetry/raw", json={"hex": bad.hex()})
        self.assertEqual(r.status_code, 400)

    def test_rate_limit_returns_429(self):
        # Drain bucket.
        while self.service.tx_bucket.take():
            pass
        r = self.client.post("/telemetry/flight_mode", json={"text": "X"})
        self.assertEqual(r.status_code, 429)

    def test_token_auth(self):
        # Enable auth, hit endpoint without bearer — should 401.
        original = self.service.API_TOKEN
        self.service.API_TOKEN = "secret123"
        try:
            r = self.client.post("/telemetry/flight_mode", json={"text": "X"})
            self.assertEqual(r.status_code, 401)
            r = self.client.post(
                "/telemetry/flight_mode", json={"text": "X"},
                headers={"Authorization": "Bearer secret123"},
            )
            self.assertEqual(r.status_code, 202)
        finally:
            self.service.API_TOKEN = original

    def test_state_includes_counters(self):
        r = self.client.get("/state")
        body = r.get_json()
        for key in ("uplink_hz", "link_stats_hz", "bytes_in", "bytes_out",
                    "uptime_s", "serial_connected", "tx_queue_depth"):
            self.assertIn(key, body)


if __name__ == "__main__":
    unittest.main(verbosity=2)
