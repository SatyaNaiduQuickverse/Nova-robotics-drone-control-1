"""USB-CDC byte multiplexing for the v2 ESP firmware.

The ESP32-C6 drone-bridge firmware (v2, in `firmware/drone_bridge/`)
manages TWO UARTs and one USB-CDC link to drone-pi. Bytes from each
UART are wrapped in a 5-byte framed header before going up to the Pi,
and bytes going down from the Pi are routed by header to the correct
UART. This module is the Pi-side codec.

Wire format on USB-CDC, both directions:

    +------+------+--------+----------+----------+
    | 0xFE | 0xCA | chan u8| len_lo u8| len_hi u8|  (5-byte header)
    +------+------+--------+----------+----------+
    | payload (len bytes, raw module CRSF) ...    |
    +----------------------------------------------+

    Magic = 0xCAFE little-endian (on wire: 0xFE then 0xCA).
    chan = 1 → UART1 (NEW downlink Tx, drone → ground)
    chan = 2 → UART0 (existing uplink Rx, ground → drone)
    len    = u16 LE, max 256.

Diagnostic lines from the ESP are emitted OUTSIDE the framing, each
beginning with '#' and ending with '\\n'. The decoder routes those
to a separate callback so consumers can log them without polluting
the CRSF demux.

Byte-locked with `firmware/drone_bridge/drone_bridge.ino` (drone) and
the ground-pi's `aoa/mux.py` (verified end-to-end 2026-05-13 — see
[elrs-downlink-v2-h3-spec.md] memory).
"""
from __future__ import annotations

from collections.abc import Iterator
from typing import Optional

# Wire constants — must NOT diverge from firmware. If you change any of
# these, the firmware must change in lockstep AND both sides must be
# re-flashed/redeployed in coordinated order.
MAGIC_BYTE_1: int = 0xFE
MAGIC_BYTE_2: int = 0xCA
MAX_PAYLOAD: int  = 256
HEADER_LEN: int   = 5

# Channel ids (semantic; bytes are role-agnostic). Drone side:
CHAN_TX: int = 1   # UART1 = NEW downlink Tx (drone → ground)
CHAN_RX: int = 2   # UART0 = existing uplink Rx (ground → drone)


class MuxEncodeError(ValueError):
    """Raised when an outgoing payload can't be encoded (bad channel /
    over-length payload). These are programmer errors, not wire errors —
    caller must fix before retrying."""


def encode(channel: int, payload: bytes) -> bytes:
    """Wrap `payload` in the 5-byte mux header. Returns the full framed
    packet ready to write to USB-CDC.

    Raises MuxEncodeError if channel is not 1/2 or len(payload) > 256.
    Zero-length payloads are rejected — the firmware demuxer treats
    len=0 as a sync error.
    """
    if channel not in (CHAN_TX, CHAN_RX):
        raise MuxEncodeError(f"invalid channel {channel}; must be 1 or 2")
    n = len(payload)
    if n == 0 or n > MAX_PAYLOAD:
        raise MuxEncodeError(
            f"invalid payload length {n}; must be 1..{MAX_PAYLOAD}"
        )
    return bytes((MAGIC_BYTE_1, MAGIC_BYTE_2, channel, n & 0xFF, (n >> 8) & 0xFF)) + payload


class MuxDecoder:
    """Streaming demultiplexer. Feed it bytes as they arrive on the
    USB-CDC stream; it yields `(channel, payload)` tuples for valid
    mux frames and routes ASCII diagnostic lines (`#…\\n`) to a
    separate callback.

    Holds internal state across calls — instantiate once per serial
    connection, feed all bytes from that connection to the same
    instance. Re-instantiate on reconnect.

    Stats:
        bad_sync: number of resync events (header rejected because
                  magic2 mismatch OR len out of range OR unknown
                  channel). A few from boot or stats-line interleave
                  are expected; rising fast = real protocol error.
        diag_lines: number of complete '#…\\n' diagnostic lines seen.
        frames_decoded: number of valid mux frames yielded.

    Diagnostic-line callback signature: `fn(line: str) -> None` where
    `line` already has the trailing newline stripped. If no callback
    is given, diagnostic lines are silently discarded.
    """

    __slots__ = (
        "_state", "_chan", "_len", "_read", "_buf",
        "_diag_buf", "_diag_cb",
        "bad_sync", "diag_lines", "frames_decoded",
    )

    # States: 0=wait_magic1, 1=wait_magic2, 2=wait_chan, 3=wait_len_lo,
    # 4=wait_len_hi, 5=read_payload, 6=read_diag_line.
    _S_WAIT_MAGIC1   = 0
    _S_WAIT_MAGIC2   = 1
    _S_WAIT_CHAN     = 2
    _S_WAIT_LEN_LO   = 3
    _S_WAIT_LEN_HI   = 4
    _S_READ_PAYLOAD  = 5
    _S_READ_DIAG     = 6

    def __init__(self, diag_callback=None) -> None:
        self._state = self._S_WAIT_MAGIC1
        self._chan = 0
        self._len = 0
        self._read = 0
        self._buf = bytearray(MAX_PAYLOAD)
        self._diag_buf = bytearray()
        self._diag_cb = diag_callback
        self.bad_sync = 0
        self.diag_lines = 0
        self.frames_decoded = 0

    def feed(self, data: bytes) -> Iterator[tuple[int, bytes]]:
        """Consume bytes; yield (channel, payload) per complete frame.

        Diagnostic '#…\\n' lines are routed to `diag_callback` (if set)
        and NOT yielded — they're not mux frames. The two streams are
        interleaved on the wire and this method separates them
        statefully.
        """
        for b in data:
            yield from self._feed_byte(b)

    def _feed_byte(self, b: int) -> Iterator[tuple[int, bytes]]:
        s = self._state
        if s == self._S_WAIT_MAGIC1:
            if b == MAGIC_BYTE_1:
                self._state = self._S_WAIT_MAGIC2
            elif b == 0x23:  # '#' — start of diagnostic line
                self._state = self._S_READ_DIAG
                self._diag_buf.clear()
                self._diag_buf.append(b)
            # else: garbage, drop silently — we're hunting for a boundary
        elif s == self._S_WAIT_MAGIC2:
            if b == MAGIC_BYTE_2:
                self._state = self._S_WAIT_CHAN
            elif b == MAGIC_BYTE_1:
                # 0xFE 0xFE — stay; treat the new 0xFE as the start
                pass
            elif b == 0x23:
                self._state = self._S_READ_DIAG
                self._diag_buf.clear()
                self._diag_buf.append(b)
                self.bad_sync += 1
            else:
                self._state = self._S_WAIT_MAGIC1
                self.bad_sync += 1
        elif s == self._S_WAIT_CHAN:
            self._chan = b
            self._state = self._S_WAIT_LEN_LO
        elif s == self._S_WAIT_LEN_LO:
            self._len = b
            self._state = self._S_WAIT_LEN_HI
        elif s == self._S_WAIT_LEN_HI:
            self._len |= b << 8
            if (
                self._len == 0
                or self._len > MAX_PAYLOAD
                or self._chan not in (CHAN_TX, CHAN_RX)
            ):
                self._state = self._S_WAIT_MAGIC1
                self.bad_sync += 1
            else:
                self._read = 0
                self._state = self._S_READ_PAYLOAD
        elif s == self._S_READ_PAYLOAD:
            self._buf[self._read] = b
            self._read += 1
            if self._read >= self._len:
                yield (self._chan, bytes(self._buf[: self._len]))
                self.frames_decoded += 1
                self._state = self._S_WAIT_MAGIC1
        elif s == self._S_READ_DIAG:
            if b == 0x0A:  # '\n' — end of line
                line = self._diag_buf.decode("utf-8", errors="replace").rstrip("\r\n")
                self.diag_lines += 1
                if self._diag_cb is not None:
                    try:
                        self._diag_cb(line)
                    except Exception:
                        # Don't let consumer-side exceptions break the
                        # demuxer's progress.
                        pass
                self._diag_buf.clear()
                self._state = self._S_WAIT_MAGIC1
            else:
                # Cap diag-line length to prevent unbounded growth if
                # the trailing newline is somehow lost (e.g. desync mid-
                # diagnostic). 512 is well above the longest stats line.
                if len(self._diag_buf) < 512:
                    self._diag_buf.append(b)
                else:
                    # Drop and reset — treat as desync.
                    self._diag_buf.clear()
                    self._state = self._S_WAIT_MAGIC1
                    self.bad_sync += 1
