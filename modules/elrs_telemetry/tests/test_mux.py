"""Unit tests for the 0xCAFE mux codec (drone-side).

Byte-locked with the v2 firmware (`firmware/drone_bridge/drone_bridge.ino`)
and the ground-pi codec. Any byte-level change here must propagate to
both other implementations.
"""
import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import mux  # noqa: E402


class TestEncode(unittest.TestCase):
    def test_simple_tx_payload(self):
        out = mux.encode(mux.CHAN_TX, b"\xC8\x18\x16")
        # header: FE CA 01 03 00; payload: C8 18 16
        self.assertEqual(out, b"\xFE\xCA\x01\x03\x00\xC8\x18\x16")

    def test_simple_rx_payload(self):
        out = mux.encode(mux.CHAN_RX, b"X")
        self.assertEqual(out, b"\xFE\xCA\x02\x01\x00X")

    def test_max_payload(self):
        body = bytes(range(256)) if False else b"x" * 256
        out = mux.encode(mux.CHAN_TX, body)
        # len-low = 0x00, len-high = 0x01  (256 LE)
        self.assertEqual(out[:5], b"\xFE\xCA\x01\x00\x01")
        self.assertEqual(len(out), 5 + 256)

    def test_reject_overlong(self):
        with self.assertRaises(mux.MuxEncodeError):
            mux.encode(mux.CHAN_TX, b"x" * 257)

    def test_reject_empty(self):
        with self.assertRaises(mux.MuxEncodeError):
            mux.encode(mux.CHAN_TX, b"")

    def test_reject_bad_channel(self):
        with self.assertRaises(mux.MuxEncodeError):
            mux.encode(0, b"x")
        with self.assertRaises(mux.MuxEncodeError):
            mux.encode(3, b"x")


class TestDecode(unittest.TestCase):
    def setUp(self):
        self.diag = []
        self.dec = mux.MuxDecoder(diag_callback=self.diag.append)

    def _consume(self, data: bytes):
        return list(self.dec.feed(data))

    def test_single_frame_round_trip(self):
        framed = mux.encode(mux.CHAN_TX, b"\xC8\x18\x16")
        out = self._consume(framed)
        self.assertEqual(out, [(mux.CHAN_TX, b"\xC8\x18\x16")])
        self.assertEqual(self.dec.frames_decoded, 1)
        self.assertEqual(self.dec.bad_sync, 0)

    def test_decode_split_across_chunks(self):
        framed = mux.encode(mux.CHAN_RX, b"hello world")
        # Feed one byte at a time
        seen = []
        for i in range(len(framed)):
            seen.extend(self.dec.feed(bytes([framed[i]])))
        self.assertEqual(seen, [(mux.CHAN_RX, b"hello world")])

    def test_back_to_back_frames(self):
        a = mux.encode(mux.CHAN_TX, b"\x14\x00\x01\x02")
        b = mux.encode(mux.CHAN_RX, b"\x16payload")
        out = self._consume(a + b)
        self.assertEqual(out, [
            (mux.CHAN_TX, b"\x14\x00\x01\x02"),
            (mux.CHAN_RX, b"\x16payload"),
        ])

    def test_diagnostic_line_routed_to_callback(self):
        stats = b"# stats up_to_pi=0 pi_to_up=0 dn_to_pi=0 bad_sync=0\n"
        out = self._consume(stats)
        self.assertEqual(out, [])
        self.assertEqual(self.diag, [
            "# stats up_to_pi=0 pi_to_up=0 dn_to_pi=0 bad_sync=0",
        ])
        self.assertEqual(self.dec.diag_lines, 1)
        self.assertEqual(self.dec.bad_sync, 0)

    def test_diag_interleaved_with_frames(self):
        a = mux.encode(mux.CHAN_RX, b"\x16chans")
        stats = b"# stats blah\n"
        b = mux.encode(mux.CHAN_RX, b"\x14linkstat")
        out = self._consume(a + stats + b)
        self.assertEqual(out, [
            (mux.CHAN_RX, b"\x16chans"),
            (mux.CHAN_RX, b"\x14linkstat"),
        ])
        self.assertEqual(self.diag, ["# stats blah"])

    def test_garbage_between_frames_resyncs(self):
        a = mux.encode(mux.CHAN_RX, b"valid1")
        garbage = b"\x01\x02\x03\xFE\x99"   # 0xFE + bad magic2 → bad_sync+1
        b = mux.encode(mux.CHAN_RX, b"valid2")
        out = self._consume(a + garbage + b)
        self.assertEqual(out, [
            (mux.CHAN_RX, b"valid1"),
            (mux.CHAN_RX, b"valid2"),
        ])
        # The \xFE\x99 sequence drops into bad_sync.
        self.assertGreaterEqual(self.dec.bad_sync, 1)

    def test_unknown_channel_drops_and_resyncs(self):
        # Hand-craft a frame with channel=3 (invalid)
        bad = b"\xFE\xCA\x03\x01\x00X"
        good = mux.encode(mux.CHAN_RX, b"after")
        out = self._consume(bad + good)
        self.assertEqual(out, [(mux.CHAN_RX, b"after")])
        self.assertEqual(self.dec.bad_sync, 1)

    def test_oversize_len_drops_and_resyncs(self):
        # len = 0x0FFF (4095) — way over MAX_PAYLOAD
        bad = b"\xFE\xCA\x01\xFF\x0F"
        good = mux.encode(mux.CHAN_TX, b"after")
        out = self._consume(bad + good)
        self.assertEqual(out, [(mux.CHAN_TX, b"after")])
        self.assertEqual(self.dec.bad_sync, 1)

    def test_double_magic_byte_stays_in_magic2(self):
        # 0xFE 0xFE 0xCA <chan=1> <len=1> <payload=X> — second 0xFE should
        # be treated as the new start, not failing.
        out = self._consume(b"\xFE\xFE\xCA\x01\x01\x00X")
        self.assertEqual(out, [(mux.CHAN_TX, b"X")])
        self.assertEqual(self.dec.bad_sync, 0)

    def test_callback_exception_does_not_break_decoder(self):
        dec = mux.MuxDecoder(diag_callback=lambda line: 1/0)
        # Diagnostic line should be silently absorbed; subsequent frame
        # should decode normally.
        text = b"# stats bad\n"
        frame = mux.encode(mux.CHAN_RX, b"after")
        seen = list(dec.feed(text + frame))
        self.assertEqual(seen, [(mux.CHAN_RX, b"after")])


if __name__ == "__main__":
    unittest.main()
