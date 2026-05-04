#!/usr/bin/env python3
"""
Precision landing via QR code detection.
GPS-free: ALT_HOLD mode + virtual TX for corrections.
PID controller with low-pass filtered QR position, anti-windup.
Descent only when centered. Slower descent as altitude decreases.
"""

import io
import threading
import time
from typing import Optional

import telemetry
import camera
import virtual_tx

# --- PID gains (per axis) ---
KP = 0.35           # proportional: response strength
KI = 0.08           # integral: wind drift correction
KD = 0.15           # derivative: dampen oscillation
I_LIMIT = 0.3       # anti-windup clamp on integral term

# --- Control parameters ---
LOOP_HZ = 15.0
EMA_ALPHA = 0.4          # low-pass filter on QR position (0=smooth, 1=raw)
CENTER_THRESHOLD = 0.08  # normalized error to count as "centered"
LAND_BOX_RATIO = 0.4     # QR width / frame width → trigger LAND
SEARCH_TIMEOUT = 10.0    # seconds without QR → abort
HOVER_THROTTLE = 0.44    # mid-stick for ALT_HOLD hover
DESCENT_STEP = 0.06      # throttle reduction per descent tick (below hover)
MIN_DESCENT_STEP = 0.02  # gentler descent when close

FRAME_W = camera.WIDTH
FRAME_H = camera.HEIGHT

# States
IDLE = "idle"
SEARCHING = "searching"
DESCENDING = "descending"
COMPLETE = "complete"
ABORTED = "aborted"

_state = IDLE
_lock = threading.Lock()
_thread: Optional[threading.Thread] = None
_qr_info: dict = {}


class _PIDAxis:
    """Single-axis PID with anti-windup."""
    __slots__ = ("_integral", "_prev_error", "_prev_time")

    def __init__(self):
        self.reset()

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = 0.0

    def update(self, error: float, now: float) -> float:
        dt = now - self._prev_time if self._prev_time > 0 else 0.0
        if dt <= 0 or dt > 0.5:
            # First call or stale — just proportional
            self._prev_error = error
            self._prev_time = now
            return KP * error

        # PID terms
        p = KP * error
        self._integral += error * dt
        self._integral = max(-I_LIMIT, min(I_LIMIT, self._integral))
        i = KI * self._integral
        d = KD * (error - self._prev_error) / dt

        self._prev_error = error
        self._prev_time = now
        return p + i + d


_DEFAULTS = {
    "KP": 0.35, "KI": 0.08, "KD": 0.15, "I_LIMIT": 0.3,
    "EMA_ALPHA": 0.4, "CENTER_THRESHOLD": 0.08, "LAND_BOX_RATIO": 0.4,
    "SEARCH_TIMEOUT": 10.0, "DESCENT_STEP": 0.06, "MIN_DESCENT_STEP": 0.02,
}

_TUNABLE = {"KP", "KI", "KD", "I_LIMIT", "EMA_ALPHA", "CENTER_THRESHOLD",
            "LAND_BOX_RATIO", "SEARCH_TIMEOUT", "DESCENT_STEP", "MIN_DESCENT_STEP"}


def get_config() -> dict:
    """Get current tunable parameters with defaults."""
    return {k: {"value": globals()[k], "default": _DEFAULTS[k]} for k in _TUNABLE}


def update_config(updates: dict) -> tuple[bool, str]:
    """Update tunable parameters at runtime."""
    for key, value in updates.items():
        if key not in _TUNABLE:
            return False, f"Invalid param: {key}"
        if not isinstance(value, (int, float)):
            return False, f"Invalid value for {key}"
        globals()[key] = float(value)
    return True, "Config updated"


def start_landing() -> tuple[bool, str]:
    """Begin precision landing. Caller should set ALT_HOLD mode first."""
    global _state, _thread, _qr_info

    with _lock:
        if _state not in (IDLE, COMPLETE, ABORTED):
            return False, f"Already active: {_state}"

    if not camera.get_frame():
        return False, "No camera frames available"

    try:
        from pyzbar.pyzbar import decode  # noqa: F401
        from PIL import Image  # noqa: F401
    except ImportError as e:
        return False, f"Missing dependency: {e}"

    # Enable virtual TX if not already
    ok, msg = virtual_tx.enable()
    if not ok and "Already" not in msg:
        return False, f"Virtual TX failed: {msg}"

    _qr_info = {}
    _state = SEARCHING
    _thread = threading.Thread(target=_control_loop, daemon=True)
    _thread.start()

    return True, "Precision landing started — searching for QR"


def abort() -> tuple[bool, str]:
    """Abort landing, hover in place."""
    global _state
    with _lock:
        if _state in (IDLE, COMPLETE, ABORTED):
            return False, "Not active"
        _state = ABORTED
    # Send hover command to stop movement
    virtual_tx.send_command(
        timestamp=time.time(), throttle=HOVER_THROTTLE,
        roll=0, pitch=0, yaw=0, duration=2.0
    )
    return True, "Aborted — hovering"


def get_status() -> dict:
    """Current state + QR detection info."""
    with _lock:
        return {
            "state": _state,
            "qr_detected": bool(_qr_info),
            "qr": _qr_info.copy(),
        }


def _control_loop():
    global _state, _qr_info

    from pyzbar.pyzbar import decode
    from PIL import Image

    period = 1.0 / LOOP_HZ
    search_start = time.monotonic()

    pid_x = _PIDAxis()  # lateral (roll)
    pid_y = _PIDAxis()  # longitudinal (pitch)

    # Filtered QR position (EMA)
    filt_ex = 0.0
    filt_ey = 0.0
    has_filter = False

    while True:
        with _lock:
            if _state in (ABORTED, COMPLETE, IDLE):
                break

        frame = camera.get_frame()
        if not frame:
            time.sleep(period)
            continue

        # Detect QR
        img = Image.open(io.BytesIO(frame))
        results = decode(img)
        qr = results[0] if results else None
        now = time.monotonic()

        if qr is None:
            # No QR — hover in place, reset PID integrators
            virtual_tx.send_command(
                timestamp=time.time(), throttle=HOVER_THROTTLE,
                roll=0, pitch=0, yaw=0, duration=1.0
            )
            pid_x.reset()
            pid_y.reset()
            has_filter = False

            with _lock:
                _qr_info = {}
                if _state == DESCENDING:
                    _state = SEARCHING
                    search_start = now
                if _state == SEARCHING and (now - search_start) > SEARCH_TIMEOUT:
                    _state = ABORTED
            time.sleep(period)
            continue

        # QR found
        search_start = now
        r = qr.rect
        cx = r.left + r.width / 2
        cy = r.top + r.height / 2

        # Normalized error (-1 to 1)
        raw_ex = (cx - FRAME_W / 2) / (FRAME_W / 2)
        raw_ey = (cy - FRAME_H / 2) / (FRAME_H / 2)
        qr_ratio = r.width / FRAME_W

        # Low-pass filter (EMA)
        if has_filter:
            filt_ex = EMA_ALPHA * raw_ex + (1 - EMA_ALPHA) * filt_ex
            filt_ey = EMA_ALPHA * raw_ey + (1 - EMA_ALPHA) * filt_ey
        else:
            filt_ex = raw_ex
            filt_ey = raw_ey
            has_filter = True

        with _lock:
            _state = DESCENDING
            _qr_info = {
                "ex": round(filt_ex, 3),
                "ey": round(filt_ey, 3),
                "raw_ex": round(raw_ex, 3),
                "raw_ey": round(raw_ey, 3),
                "ratio": round(qr_ratio, 3),
                "box": [r.left, r.top, r.width, r.height],
            }

        # Close enough to ground → trigger LAND
        if qr_ratio >= LAND_BOX_RATIO:
            _switch_to_land()
            with _lock:
                _state = COMPLETE
            break

        # PID corrections
        # Image x-right → drone roll right
        # Image y-down  → drone pitch forward (nose down)
        roll_cmd = pid_x.update(filt_ex, now)
        pitch_cmd = pid_y.update(-filt_ey, now)

        # Clamp to safe range
        roll_cmd = max(-0.4, min(0.4, roll_cmd))
        pitch_cmd = max(-0.4, min(0.4, pitch_cmd))

        # Descent: only when centered, gentler when closer
        centered = abs(filt_ex) < CENTER_THRESHOLD and abs(filt_ey) < CENTER_THRESHOLD
        if centered:
            step = MIN_DESCENT_STEP if qr_ratio > 0.25 else DESCENT_STEP
            throttle = HOVER_THROTTLE - step
        else:
            throttle = HOVER_THROTTLE  # hold altitude until centered

        virtual_tx.send_command(
            timestamp=time.time(),
            throttle=throttle,
            roll=roll_cmd,
            pitch=pitch_cmd,
            yaw=0,
            duration=0.5,
        )

        time.sleep(period)

    # Ensure hover on exit
    virtual_tx.send_command(
        timestamp=time.time(), throttle=HOVER_THROTTLE,
        roll=0, pitch=0, yaw=0, duration=2.0
    )


def _switch_to_land():
    """Switch FC to LAND mode via service call."""
    from mavros_msgs.srv import SetMode
    node = telemetry.get_node()
    client = node.create_client(SetMode, "/mavros/set_mode")
    if not client.wait_for_service(timeout_sec=2.0):
        return
    req = SetMode.Request()
    req.custom_mode = "LAND"
    future = client.call_async(req)
    end = time.monotonic() + 5.0
    while not future.done() and time.monotonic() < end:
        time.sleep(0.01)
