"""
Compass calibration progress decoder.

MAVROS Humble does not publish MAG_CAL_PROGRESS / MAG_CAL_REPORT as typed
ROS topics, but it does republish every raw MAVLink packet on
/uas1/mavlink_source as mavros_msgs/Mavlink. We subscribe there, filter
for the two message ids we care about, and unpack the payload by hand.

Wire layout (field sizes, largest-first):
    MAG_CAL_PROGRESS (id 191, 27 bytes):
        float direction_x, direction_y, direction_z           (12)
        uint8 compass_id, cal_mask, cal_status, attempt,
              completion_pct                                    (5)
        uint8 completion_mask[10]                              (10)

    MAG_CAL_REPORT (id 192, 44 bytes):
        float fitness, ofs_x, ofs_y, ofs_z,
              diag_x, diag_y, diag_z,
              offdiag_x, offdiag_y, offdiag_z                  (40)
        uint8 compass_id, cal_mask, cal_status, autosaved      (4)

get_state() is thread-safe; the FastAPI layer calls it to surface progress
without touching ROS directly.
"""

import struct
import threading
import time
from typing import Optional

from mavros_msgs.msg import Mavlink
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

_MSG_ID_PROGRESS = 191
_MSG_ID_REPORT = 192

# MAG_CAL_STATUS enum values (ardupilotmega.xml)
_CAL_STATUS_NAMES = {
    0: "NOT_STARTED",
    1: "WAITING_TO_START",
    2: "RUNNING_STEP_ONE",
    3: "RUNNING_STEP_TWO",
    4: "SUCCESS",
    5: "FAILED",
    6: "BAD_ORIENTATION",
    7: "BAD_RADIUS",
}
_TERMINAL_STATUSES = {4, 5, 6, 7}

_sub = None
_lock = threading.Lock()
_state = {
    "progress": None,          # latest MAG_CAL_PROGRESS (per compass_id)
    "report": None,            # latest MAG_CAL_REPORT (per compass_id)
    "last_update_t": 0.0,
}


def _payload_bytes(msg: Mavlink) -> bytes:
    """Pack payload64 (uint64 array) back into the raw little-endian byte buffer."""
    buf = b"".join(struct.pack("<Q", x) for x in msg.payload64)
    return buf[: msg.len]


def _decode_progress(buf: bytes) -> Optional[dict]:
    if len(buf) < 17:
        return None
    dx, dy, dz = struct.unpack_from("<fff", buf, 0)
    compass_id, cal_mask, cal_status, attempt, completion_pct = struct.unpack_from(
        "<BBBBB", buf, 12
    )
    return {
        "compass_id": int(compass_id),
        "cal_status": int(cal_status),
        "cal_status_name": _CAL_STATUS_NAMES.get(cal_status, str(cal_status)),
        "completion_pct": int(completion_pct),
        "attempt": int(attempt),
        "direction": [float(dx), float(dy), float(dz)],
    }


def _decode_report(buf: bytes) -> Optional[dict]:
    if len(buf) < 44:
        return None
    (
        fitness, ofs_x, ofs_y, ofs_z,
        diag_x, diag_y, diag_z,
        offdiag_x, offdiag_y, offdiag_z,
    ) = struct.unpack_from("<10f", buf, 0)
    compass_id, _cal_mask, cal_status, autosaved = struct.unpack_from("<BBBB", buf, 40)
    return {
        "compass_id": int(compass_id),
        "cal_status": int(cal_status),
        "cal_status_name": _CAL_STATUS_NAMES.get(cal_status, str(cal_status)),
        "autosaved": bool(autosaved),
        "fitness": float(fitness),
        "offsets": [float(ofs_x), float(ofs_y), float(ofs_z)],
        "diag": [float(diag_x), float(diag_y), float(diag_z)],
        "offdiag": [float(offdiag_x), float(offdiag_y), float(offdiag_z)],
    }


def _on_mavlink(msg: Mavlink) -> None:
    mid = msg.msgid
    if mid != _MSG_ID_PROGRESS and mid != _MSG_ID_REPORT:
        return
    buf = _payload_bytes(msg)
    decoded = _decode_progress(buf) if mid == _MSG_ID_PROGRESS else _decode_report(buf)
    if decoded is None:
        return
    with _lock:
        if mid == _MSG_ID_PROGRESS:
            _state["progress"] = decoded
        else:
            _state["report"] = decoded
        _state["last_update_t"] = time.time()


def start(node) -> None:
    """Attach the subscription to the given rclpy node (shared telemetry node).

    /uas1/mavlink_source is published with BEST_EFFORT reliability, so the
    subscription must match or no messages are delivered.
    """
    global _sub
    if _sub is not None:
        return
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=50,
    )
    _sub = node.create_subscription(Mavlink, "/uas1/mavlink_source", _on_mavlink, qos)


def stop() -> None:
    global _sub
    if _sub is None:
        return
    try:
        _sub.destroy()
    except Exception:
        pass
    _sub = None


def reset() -> None:
    """Clear state at the start of a new calibration run."""
    with _lock:
        _state["progress"] = None
        _state["report"] = None
        _state["last_update_t"] = 0.0


def get_state() -> dict:
    with _lock:
        progress = _state["progress"]
        report = _state["report"]
        last_t = _state["last_update_t"]

    active = progress is not None and progress["cal_status"] not in _TERMINAL_STATUSES
    if report is not None and report["cal_status"] in _TERMINAL_STATUSES:
        active = False

    return {
        "active": active,
        "progress": progress,
        "report": report,
        "last_update_t": last_t,
    }
