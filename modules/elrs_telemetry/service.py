#!/usr/bin/env python3
"""ELRS telemetry bridge service.

Reads CRSF frames from an ESP32-C6 over USB-CDC, exposes RC channels
and link statistics over REST + WebSocket, and accepts telemetry POSTs
that get framed as CRSF and pushed back upstream toward the ground.

Runs in its own Docker container — see docker-compose.yml in this dir.
"""

import json
import logging
import os
import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import serial
from flask import Flask, abort, jsonify, request
from flask_sock import Sock

import crsf
import mux
import ble_server


# --- Config ----------------------------------------------------------------

SERIAL_DEVICE = os.environ.get(
    "ELRS_SERIAL_DEVICE",
    "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_FC:01:2C:E8:BC:64-if00",
)
SERIAL_BAUD   = int(os.environ.get("ELRS_SERIAL_BAUD", "1000000"))
HTTP_BIND     = os.environ.get("ELRS_BIND", "0.0.0.0")
HTTP_PORT     = int(os.environ.get("ELRS_PORT", "5003"))
API_TOKEN     = os.environ.get("ELRS_API_TOKEN", "").strip()
TX_RATE_HZ    = float(os.environ.get("ELRS_TX_RATE_HZ", "5.0"))
TX_QUEUE_MAX  = 20
STALE_AGE_S   = 1.0

WS_CHANNELS_HZ = 30.0
WS_LINK_HZ     = 5.0


# --- Logging ---------------------------------------------------------------

logging.basicConfig(
    level=os.environ.get("ELRS_LOG_LEVEL", "INFO").upper(),
    format="%(asctime)s %(levelname)s %(name)s %(message)s",
)
log = logging.getLogger("elrs")
# Mute Flask's per-request access log — too noisy at 30 Hz WS heartbeats.
logging.getLogger("werkzeug").setLevel(logging.WARNING)


# --- State -----------------------------------------------------------------

@dataclass
class LinkState:
    channels_crsf: Optional[list] = None
    channels_us:   Optional[list] = None
    last_rc_ts:    Optional[float] = None
    rc_count:      int = 0

    link:          Optional[dict] = None
    last_ls_ts:    Optional[float] = None
    ls_count:      int = 0

    # Mux-layer (USB-CDC 0xCAFE framing) counters. Snapshotted from the
    # reader-loop's decoder instance on every chunk read so production
    # delivery measurement can poll /stats without WS-broker coalescing.
    mux_frames_decoded: int = 0
    mux_bad_sync:       int = 0
    mux_diag_lines:     int = 0
    ch1_bytes_in:       int = 0   # downlink Tx talkback bytes

    devices:       dict = field(default_factory=dict)
    bytes_in:      int = 0
    bytes_out:     int = 0
    started_at:    float = 0.0
    serial_connected: bool = False
    # Bench-test overrides: idx → (expires_at_monotonic, us, crsf_val).
    # Applied at broadcast time so /ws/channels reflects the held value
    # while real CRSF reads still write through to channels_us/channels_crsf.
    overrides:     dict = field(default_factory=dict)


state = LinkState(started_at=time.monotonic())
state_lock = threading.Lock()

# Shared serial handle, set by reader thread once open. Writer thread reads it.
# Updates are atomic in CPython (dict assignment of one key) so a lock isn't
# strictly required, but the dict-of-one indirection makes the "may be None"
# semantics obvious.
_serial_handle: dict = {"ser": None}


# --- WebSocket broker ------------------------------------------------------

class WSBroker:
    """One thread (the reader) sends to all clients. Handler threads only
    call receive(). simple_websocket Server.send() is safe under that
    single-producer pattern."""

    def __init__(self, name: str):
        self.name = name
        self.clients: list = []
        self.lock = threading.Lock()

    def add(self, ws):
        with self.lock:
            self.clients.append(ws)
        log.info("ws %s: client connected (total=%d)", self.name, len(self.clients))

    def remove(self, ws):
        with self.lock:
            try:
                self.clients.remove(ws)
            except ValueError:
                return
        log.info("ws %s: client disconnected (total=%d)", self.name, len(self.clients))

    def broadcast(self, message: str):
        # Snapshot under the lock, iterate without it. Only the reader
        # thread calls broadcast(), so no other producer contends.
        with self.lock:
            snapshot = list(self.clients)
        if not snapshot:
            return
        dead = []
        for c in snapshot:
            try:
                c.send(message)
            except Exception:
                dead.append(c)
        # Drop + close dead clients. Closing the underlying WS unblocks
        # the per-client handler's ws.receive() so it exits and the
        # central sees the disconnect — without this, a client whose
        # send failed once stays in TCP-alive limbo and silently never
        # receives another frame.
        for c in dead:
            self.remove(c)
            try:
                c.close()
            except Exception:
                pass


ch_broker = WSBroker("channels")
ls_broker = WSBroker("link")


# --- Token-bucket rate limiter --------------------------------------------

class TokenBucket:
    """Simple thread-safe token bucket. take() returns False when empty."""

    def __init__(self, rate_hz: float, capacity: int = 10):
        self.rate = rate_hz
        self.capacity = capacity
        self.tokens = float(capacity)
        self.last = time.monotonic()
        self.lock = threading.Lock()

    def take(self) -> bool:
        with self.lock:
            now = time.monotonic()
            self.tokens = min(self.capacity, self.tokens + (now - self.last) * self.rate)
            self.last = now
            if self.tokens >= 1.0:
                self.tokens -= 1.0
                return True
            return False


tx_bucket = TokenBucket(TX_RATE_HZ)
tx_queue: "queue.Queue[bytes]" = queue.Queue(maxsize=TX_QUEUE_MAX)


# --- Reader / writer threads ----------------------------------------------

# Throttle WS broadcasts to a sane rate independent of arrival rate.
_last_ch_bcast = [0.0]
_last_ls_bcast = [0.0]


def _maybe_broadcast_channels():
    now = time.monotonic()
    if now - _last_ch_bcast[0] < (1.0 / WS_CHANNELS_HZ):
        return
    _last_ch_bcast[0] = now
    with state_lock:
        if state.channels_us is None:
            return
        ch_us   = list(state.channels_us)
        ch_crsf = list(state.channels_crsf)
        # Apply any active bench-test overrides (TTL-bounded).
        for idx, (expires, us, crsf_v) in list(state.overrides.items()):
            if expires < now:
                state.overrides.pop(idx, None)
                continue
            ch_us[idx]   = us
            ch_crsf[idx] = crsf_v
        msg = json.dumps({
            "channels_crsf": ch_crsf,
            "channels_us":   ch_us,
            "ts": now,
        })
    ch_broker.broadcast(msg)


def _maybe_broadcast_link():
    now = time.monotonic()
    if now - _last_ls_bcast[0] < (1.0 / WS_LINK_HZ):
        return
    _last_ls_bcast[0] = now
    with state_lock:
        msg = json.dumps({"link": state.link, "ts": now})
    ls_broker.broadcast(msg)


def _handle_frame(addr: int, ftype: int, payload: bytes):
    now = time.monotonic()
    if ftype == crsf.FRAME_RC_CHANNELS_PACKED:
        ch = crsf.unpack_channels(payload)
        ch_us = [crsf.crsf_to_us(c) for c in ch]
        with state_lock:
            state.channels_crsf = ch
            state.channels_us = ch_us
            state.last_rc_ts = now
            state.rc_count += 1
        _maybe_broadcast_channels()
    elif ftype == crsf.FRAME_LINK_STATS:
        ls = crsf.parse_link_stats(payload)
        if ls:
            with state_lock:
                state.link = ls
                state.last_ls_ts = now
                state.ls_count += 1
            _maybe_broadcast_link()
    elif ftype == crsf.FRAME_DEVICE_INFO:
        # Payload starts with destination + origin addr; the device name is
        # a null-terminated ASCII string after that. Best-effort decode.
        try:
            name_end = payload.index(b'\x00', 2)
            name = payload[2:name_end].decode('ascii', errors='ignore')
            if name:
                with state_lock:
                    state.devices[name] = now
        except (ValueError, IndexError):
            pass


def _reader_loop():
    parser = crsf.CRSFParser()

    # v2 firmware muxes both UARTs onto USB-CDC using the 0xCAFE-framed
    # protocol (see mux.py). Demux first, then feed only the ch=2 (uplink
    # Rx) payload to the CRSF parser. ch=1 is Tx talkback — log volume
    # only for now; not routed to any consumer until Phase H4 wires a
    # telemetry consumer. Diagnostic '#…\n' lines from the ESP go to a
    # separate logger.
    decoder = mux.MuxDecoder(diag_callback=lambda line: log.info("ESP: %s", line))
    ch1_bytes_in = 0   # downlink Tx talkback (LINK_STATISTICS et al.)

    backoff = 0.5
    last_log = 0.0
    while True:
        try:
            log.info("opening serial: %s @ %d", SERIAL_DEVICE, SERIAL_BAUD)
            ser = serial.Serial(SERIAL_DEVICE, SERIAL_BAUD, timeout=0.005)
            with state_lock:
                state.serial_connected = True
            _serial_handle["ser"] = ser
            backoff = 0.5
            log.info("serial open (v2 mux protocol)")

            while True:
                chunk = ser.read(256)
                if not chunk:
                    continue
                with state_lock:
                    state.bytes_in += len(chunk)
                    # Snapshot mux-layer counters so /stats sees fresh values
                    # without exposing the decoder instance directly. Python
                    # int reads/writes are atomic under the GIL — the lock
                    # only serialises with REST handlers that read state.
                    state.mux_frames_decoded = decoder.frames_decoded
                    state.mux_bad_sync       = decoder.bad_sync
                    state.mux_diag_lines     = decoder.diag_lines
                for channel, payload in decoder.feed(chunk):
                    if channel == mux.CHAN_RX:
                        # Existing uplink path: CRSF frames from the bound
                        # ground-Tx → drone-Rx pair. Behaviour identical to
                        # the v1 firmware era — the mux layer is invisible
                        # to downstream consumers.
                        for addr, ftype, pl in parser.feed(payload):
                            _handle_frame(addr, ftype, pl)
                    elif channel == mux.CHAN_TX:
                        # Tx talkback (link stats, device-info responses).
                        # No consumer wired yet; count bytes for stats.
                        ch1_bytes_in += len(payload)
                        with state_lock:
                            state.ch1_bytes_in = ch1_bytes_in

                # Throttled summary every 10 s so the journal stays readable.
                now = time.monotonic()
                if now - last_log >= 10.0:
                    last_log = now
                    with state_lock:
                        elapsed = now - state.started_at
                        log.info(
                            "stats uplink_hz=%.1f link_hz=%.1f bytes_in=%d "
                            "bytes_out=%d devices=%d "
                            "mux=[rx_frames=%d tx_talkback_bytes=%d "
                            "diag_lines=%d bad_sync=%d]",
                            state.rc_count / elapsed if elapsed > 0 else 0,
                            state.ls_count / elapsed if elapsed > 0 else 0,
                            state.bytes_in, state.bytes_out, len(state.devices),
                            decoder.frames_decoded, ch1_bytes_in,
                            decoder.diag_lines, decoder.bad_sync,
                        )
        except (serial.SerialException, OSError) as e:
            with state_lock:
                state.serial_connected = False
            _serial_handle["ser"] = None
            log.warning("serial error: %s; retry in %.1fs", e, backoff)
            time.sleep(backoff)
            backoff = min(backoff * 2, 5.0)
        except Exception:
            log.exception("reader: unexpected error")
            time.sleep(1.0)


def _writer_loop():
    # v2 firmware demuxes by 5-byte header. All outgoing CRSF frames
    # currently target the NEW downlink Tx (ch=1) — that's the only
    # writable RF path from the drone-pi. If a future Phase H4 feature
    # needs to send to ch=2 (uplink Rx config back-channel — typically
    # for Lua param-set), it'll go through a different code path with
    # an explicit channel argument; we don't gate this here.
    while True:
        frame = tx_queue.get()
        ser = _serial_handle.get("ser")
        if ser is None:
            log.warning("tx: serial not open, dropping %d B", len(frame))
            continue
        try:
            framed = mux.encode(mux.CHAN_TX, frame)
        except mux.MuxEncodeError as e:
            log.warning("tx: mux encode failed (%s); dropping %d B",
                        e, len(frame))
            continue
        try:
            ser.write(framed)
            with state_lock:
                # Counter tracks payload bytes, not framing overhead —
                # matches the existing semantic ("bytes of CRSF sent").
                state.bytes_out += len(frame)
        except (serial.SerialException, OSError) as e:
            log.warning("tx: write failed: %s", e)
            time.sleep(0.1)


# --- HTTP / WS API ---------------------------------------------------------

app = Flask(__name__)
sock = Sock(app)


def _require_token():
    if not API_TOKEN:
        return
    auth = request.headers.get("Authorization", "")
    if not auth.startswith("Bearer ") or auth[7:] != API_TOKEN:
        abort(401)


def _age_ms(ts: Optional[float]) -> Optional[int]:
    if ts is None:
        return None
    return int((time.monotonic() - ts) * 1000)


def _stale(ts: Optional[float]) -> bool:
    return ts is None or (time.monotonic() - ts) > STALE_AGE_S


@app.route("/healthz")
def healthz():
    with state_lock:
        return jsonify(
            ok=True,
            uptime_s=round(time.monotonic() - state.started_at, 2),
            serial=state.serial_connected,
        )


@app.route("/channels")
def channels():
    with state_lock:
        return jsonify(
            crsf=state.channels_crsf,
            us=state.channels_us,
            age_ms=_age_ms(state.last_rc_ts),
            stale=_stale(state.last_rc_ts),
        )


@app.route("/link")
def link():
    with state_lock:
        return jsonify(
            link=state.link,
            age_ms=_age_ms(state.last_ls_ts),
            stale=_stale(state.last_ls_ts),
        )


@app.route("/state")
def state_endpoint():
    now = time.monotonic()
    with state_lock:
        elapsed = now - state.started_at
        return jsonify(
            channels_crsf=state.channels_crsf,
            channels_us=state.channels_us,
            channels_age_ms=_age_ms(state.last_rc_ts),
            channels_stale=_stale(state.last_rc_ts),
            link=state.link,
            link_age_ms=_age_ms(state.last_ls_ts),
            link_stale=_stale(state.last_ls_ts),
            devices={k: _age_ms(v) for k, v in state.devices.items()},
            uplink_hz=round(state.rc_count / elapsed, 2) if elapsed > 0 else 0,
            link_stats_hz=round(state.ls_count / elapsed, 2) if elapsed > 0 else 0,
            # Raw parse/mux-layer counters — monotonic, sampled at parse time.
            # Pollers compute true delivery rate via (c2-c1)/(t2-t1) without
            # WS-broker coalescing artefacts. Production-grade RF metrics.
            rc_count=state.rc_count,
            ls_count=state.ls_count,
            mux_frames_decoded=state.mux_frames_decoded,
            mux_bad_sync=state.mux_bad_sync,
            mux_diag_lines=state.mux_diag_lines,
            ch1_bytes_in=state.ch1_bytes_in,
            bytes_in=state.bytes_in,
            bytes_out=state.bytes_out,
            uptime_s=round(elapsed, 2),
            serial_connected=state.serial_connected,
            tx_queue_depth=tx_queue.qsize(),
        )


@app.route("/stats")
def stats_endpoint():
    """Lightweight production polling endpoint — raw counters only.

    Designed for periodic polling at ≥1 Hz from a flight-telemetry
    monitor or external delivery-rate computer. Excludes the heavier
    channels/link/devices payloads in /state.
    """
    now = time.monotonic()
    with state_lock:
        return jsonify(
            rc_count=state.rc_count,
            ls_count=state.ls_count,
            mux_frames_decoded=state.mux_frames_decoded,
            mux_bad_sync=state.mux_bad_sync,
            mux_diag_lines=state.mux_diag_lines,
            bytes_in=state.bytes_in,
            bytes_out=state.bytes_out,
            ch1_bytes_in=state.ch1_bytes_in,
            uptime_s=round(now - state.started_at, 2),
            serial_connected=state.serial_connected,
            ts_monotonic=round(now, 3),
        )


@app.route("/debug/inject_channel", methods=["POST"])
def debug_inject_channel():
    """TEST-ONLY: override one channel's us value. Two modes:

      hold_s == 0 (default): one-shot — overwrite state and emit one
        broadcast. The next real CRSF frame restores the natural value
        within ~20 ms. Use for edge-trigger tests.

      hold_s  > 0: install a TTL'd override applied at broadcast time
        for hold_s seconds. State.channels_* still tracks real CRSF, but
        broadcasts (and thus downstream consumers like the translator)
        see the held value. Use for sustained command tests (throttle,
        roll) where snap-back would defeat the test.

    Bench/test use only — gated by docstring, not by env. Disable in
    production by removing this route.
    """
    body = request.get_json(force=True, silent=True) or {}
    try:
        idx    = int(body.get("index", 6))
        us     = int(body.get("us", 1500))
        hold_s = float(body.get("hold_s", 0.0))
    except (TypeError, ValueError):
        return jsonify(ok=False, error="index/us/hold_s must be numeric"), 400
    if not (0 <= idx < 16):
        return jsonify(ok=False, error="index out of range"), 400
    if not (988 <= us <= 2012):
        return jsonify(ok=False, error="us out of range"), 400
    if not (0 <= hold_s <= 30.0):
        return jsonify(ok=False, error="hold_s must be 0..30"), 400
    crsf_val = int(round((us - 1500) * 8 / 5 + 992))

    if hold_s > 0:
        with state_lock:
            state.overrides[idx] = (time.monotonic() + hold_s, us, crsf_val)
        return jsonify(ok=True, mode="hold", index=idx, us=us, crsf=crsf_val,
                       hold_s=hold_s)

    with state_lock:
        if state.channels_us is None:
            return jsonify(ok=False, error="no real CRSF data yet"), 503
        state.channels_us[idx]   = us
        state.channels_crsf[idx] = crsf_val
        msg = json.dumps({
            "channels_crsf": state.channels_crsf,
            "channels_us":   state.channels_us,
            "ts": time.monotonic(),
        })
    ch_broker.broadcast(msg)
    return jsonify(ok=True, mode="oneshot", index=idx, us=us, crsf=crsf_val)


@app.route("/debug/clear_overrides", methods=["POST"])
def debug_clear_overrides():
    """TEST-ONLY: drop all active hold overrides immediately."""
    with state_lock:
        n = len(state.overrides)
        state.overrides.clear()
    return jsonify(ok=True, cleared=n)


def _ws_loop(ws, broker: WSBroker, snapshot_fn):
    """Common WS handler: register, send initial snapshot, hold open until close."""
    broker.add(ws)
    try:
        snap = snapshot_fn()
        if snap is not None:
            ws.send(snap)
        while True:
            # ws.receive(timeout=...) returns None on timeout, raises on close.
            msg = ws.receive(timeout=30)
            if msg is None:
                continue  # heartbeat tick
    except Exception:
        pass
    finally:
        broker.remove(ws)


@sock.route("/ws/channels")
def ws_channels(ws):
    def snap():
        with state_lock:
            if state.channels_crsf is None:
                return None
            return json.dumps({
                "channels_crsf": state.channels_crsf,
                "channels_us":   state.channels_us,
                "ts": time.monotonic(),
            })
    _ws_loop(ws, ch_broker, snap)


@sock.route("/ws/link")
def ws_link(ws):
    def snap():
        with state_lock:
            if state.link is None:
                return None
            return json.dumps({"link": state.link, "ts": time.monotonic()})
    _ws_loop(ws, ls_broker, snap)


# --- Telemetry POST handlers ----------------------------------------------

def _enqueue(frame: bytes):
    if not tx_bucket.take():
        abort(429, description="rate limit (ELRS_TX_RATE_HZ)")
    try:
        tx_queue.put_nowait(frame)
    except queue.Full:
        # Drop oldest to make room — bounded queue, never grows unbounded.
        try:
            tx_queue.get_nowait()
        except queue.Empty:
            pass
        try:
            tx_queue.put_nowait(frame)
        except queue.Full:
            abort(503, description="tx queue full")


def _enqueued_response(frame: bytes):
    return jsonify(enqueued=True, bytes=len(frame), hex=frame.hex()), 202


@app.route("/telemetry/battery", methods=["POST"])
def tx_battery():
    _require_token()
    data = request.get_json(force=True, silent=True) or {}
    try:
        v   = float(data.get("voltage", 0))
        a   = float(data.get("current", 0))
        mah = int(data.get("mah", 0))
        pct = int(data.get("percent", 0))
    except (TypeError, ValueError):
        abort(400, description="numeric fields required")
    if not (0 <= v <= 60):       abort(400, description="voltage 0..60")
    if not (-200 <= a <= 200):   abort(400, description="current -200..200")
    if not (0 <= mah <= 0xFFFFFF): abort(400, description="mah 0..16777215")
    if not (0 <= pct <= 100):    abort(400, description="percent 0..100")
    frame = crsf.make_battery(v, a, mah, pct)
    _enqueue(frame)
    return _enqueued_response(frame)


@app.route("/telemetry/flight_mode", methods=["POST"])
def tx_flight_mode():
    _require_token()
    data = request.get_json(force=True, silent=True) or {}
    text = str(data.get("text", "")).strip()
    if not text or len(text) > 32:
        abort(400, description="text 1..32 chars")
    if not all(0x20 <= ord(c) < 0x7F for c in text):
        abort(400, description="ascii printable only")
    frame = crsf.make_flight_mode(text)
    _enqueue(frame)
    return _enqueued_response(frame)


@app.route("/telemetry/gps", methods=["POST"])
def tx_gps():
    _require_token()
    data = request.get_json(force=True, silent=True) or {}
    try:
        # CRSF GPS frame (15 B): lat/lon int32 BE (deg * 1e7), groundspeed
        # uint16 BE (km/h * 10), heading uint16 BE (deg * 100), altitude
        # uint16 BE (m + 1000 offset), satellites uint8.
        lat        = int(float(data.get("lat", 0)) * 1e7)
        lon        = int(float(data.get("lon", 0)) * 1e7)
        speed_x10  = int(float(data.get("speed", 0)) * 10)
        head_x100  = int(float(data.get("heading", 0)) * 100)
        alt_offset = int(float(data.get("alt", 0))) + 1000
        sats       = int(data.get("satellites", 0))
    except (TypeError, ValueError):
        abort(400, description="numeric fields required")
    if not (-90  <= float(data.get("lat", 0)) <= 90):    abort(400, description="lat -90..90")
    if not (-180 <= float(data.get("lon", 0)) <= 180):   abort(400, description="lon -180..180")
    if not (0 <= speed_x10 <= 0xFFFF):                   abort(400, description="speed out of range")
    if not (0 <= head_x100 <= 0xFFFF):                   abort(400, description="heading out of range")
    if not (0 <= alt_offset <= 0xFFFF):                  abort(400, description="alt out of range")
    if not (0 <= sats <= 99):                            abort(400, description="satellites 0..99")
    payload = (
        lat.to_bytes(4, 'big', signed=True) +
        lon.to_bytes(4, 'big', signed=True) +
        speed_x10.to_bytes(2, 'big') +
        head_x100.to_bytes(2, 'big') +
        alt_offset.to_bytes(2, 'big') +
        bytes([sats & 0xFF])
    )
    frame = crsf.make_frame(crsf.ADDR_FLIGHT_CTRL, crsf.FRAME_GPS, payload)
    _enqueue(frame)
    return _enqueued_response(frame)


@app.route("/telemetry/attitude", methods=["POST"])
def tx_attitude():
    _require_token()
    data = request.get_json(force=True, silent=True) or {}
    try:
        # angles in radians, transmitted as int16 BE * 10000.
        pitch = int(float(data.get("pitch", 0)) * 10000)
        roll  = int(float(data.get("roll",  0)) * 10000)
        yaw   = int(float(data.get("yaw",   0)) * 10000)
    except (TypeError, ValueError):
        abort(400, description="numeric fields required")
    for v in (pitch, roll, yaw):
        if not (-32768 <= v <= 32767):
            abort(400, description="angle * 10000 must fit int16")
    payload = (
        pitch.to_bytes(2, 'big', signed=True) +
        roll.to_bytes(2, 'big', signed=True) +
        yaw.to_bytes(2, 'big', signed=True)
    )
    frame = crsf.make_frame(crsf.ADDR_FLIGHT_CTRL, crsf.FRAME_ATTITUDE, payload)
    _enqueue(frame)
    return _enqueued_response(frame)


@app.route("/telemetry/raw", methods=["POST"])
def tx_raw():
    """Escape hatch — accepts a complete CRSF frame as hex. Validates only
    the bare minimum (sync byte + length sanity) so callers can craft
    frame types we haven't wrapped."""
    _require_token()
    data = request.get_json(force=True, silent=True) or {}
    h = str(data.get("hex", "")).strip()
    if not h or len(h) % 2 != 0:
        abort(400, description="hex string required (even length)")
    try:
        frame = bytes.fromhex(h)
    except ValueError:
        abort(400, description="invalid hex")
    if not (4 <= len(frame) <= 64):
        abort(400, description="frame length 4..64")
    if frame[0] not in (0x00, 0xC8, 0xEA, 0xEC, 0xEE):
        abort(400, description="first byte must be a CRSF sync address")
    if frame[1] + 2 != len(frame):
        abort(400, description="length byte mismatch")
    _enqueue(frame)
    return _enqueued_response(frame)


# --- Main -----------------------------------------------------------------

def main():
    threading.Thread(target=_reader_loop, daemon=True, name="reader").start()
    threading.Thread(target=_writer_loop, daemon=True, name="writer").start()
    ble_server.start(state, log)
    log.info("starting HTTP on %s:%d (token_auth=%s, tx_rate=%.1fHz)",
             HTTP_BIND, HTTP_PORT, bool(API_TOKEN), TX_RATE_HZ)
    app.run(host=HTTP_BIND, port=HTTP_PORT, threaded=True,
            debug=False, use_reloader=False)


if __name__ == "__main__":
    main()
