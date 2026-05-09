#!/usr/bin/env python3
"""
Minimal API gateway for MAVROS drone control.
Direct MAVROS service calls, no abstractions.
Thread-safe: single lock serializes all service calls,
futures resolved by telemetry's background rclpy.spin() thread.
"""

import asyncio
import time
import threading
from fastapi import FastAPI, HTTPException
from fastapi.responses import StreamingResponse, Response
from pydantic import BaseModel
from typing import Literal, Optional

from mavros_msgs.srv import CommandBool, SetMode, CommandLong, WaypointClear, WaypointPush, StreamRate
from mavros_msgs.msg import Waypoint
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import telemetry
import mission
import virtual_tx
import camera
import precision_land
import compass_cal
import vtx_broadcast

# Mode mapping: API name -> (ArduPilot, PX4)
#
# This dict is the canonical contract for `POST /mode`: any name not
# present here is rejected at the api_gateway layer with "Invalid mode"
# before the request ever reaches MAVROS. The phone CH7 rotary, the
# Android internet-control mode picker, and the web UI all share this
# list — keep them aligned.
#
# PX4 mappings are best-effort:
#   - SMART_RTL has no PX4 equivalent → degrades to AUTO.RTL (regular RTL)
#   - AUTOTUNE has no PX4 equivalent → kept as "AUTOTUNE"; FC will reject
#     if running PX4 (acceptable — this build is ArduCopter-only)
#   - POSHOLD reuses POSCTL (same mapping as LOITER on PX4 since PX4
#     doesn't separate "GPS hold" from "GPS hold with stick override")
MODES = {
    "stabilize":  ("STABILIZE", "STABILIZED"),
    "alt_hold":   ("ALT_HOLD",  "ALTCTL"),
    "loiter":     ("LOITER",    "POSCTL"),
    "auto":       ("AUTO",      "AUTO.MISSION"),
    "guided":     ("GUIDED",    "OFFBOARD"),
    "brake":      ("BRAKE",     "AUTO.LOITER"),
    "rtl":        ("RTL",       "AUTO.RTL"),
    "land":       ("LAND",      "AUTO.LAND"),
    "poshold":    ("POSHOLD",   "POSCTL"),
    "acro":       ("ACRO",      "ACRO"),
    "smart_rtl":  ("SMART_RTL", "AUTO.RTL"),
    "autotune":   ("AUTOTUNE",  "AUTOTUNE"),
}

TELEM_CATEGORIES = [
    "state", "extended_state", "battery", "gps", "local_position",
    "orientation", "velocity", "imu", "mag", "baro", "vfr_hud",
    "rc_in", "rc_out", "home"
]

# Service clients - created once at startup (and rebuilt on FCU reconnect).
# Resolved on the telemetry node's background spin thread.
_arming_client = None
_set_mode_client = None
_command_client = None
_wp_clear_client = None
_wp_push_client = None
_param_get_client = None
_param_set_client = None
_fence_push_client = None
_fence_clear_client = None
_svc_lock = threading.Lock()


class ArmResponse(BaseModel):
    success: bool
    message: str


class ModeRequest(BaseModel):
    mode: str
    platform: str = "ardupilot"


class ModeResponse(BaseModel):
    success: bool
    message: str
    mode_sent: str


class MissionRequest(BaseModel):
    wpl: str  # QGC WPL 110 format string
    auto_arm: bool = True


class MissionResponse(BaseModel):
    success: bool
    message: str
    waypoints_uploaded: int = 0


class ControlResponse(BaseModel):
    success: bool
    message: str


class ControlCommandRequest(BaseModel):
    timestamp: float
    throttle: float   # 0.0 to 1.0
    roll: float       # -1.0 to 1.0
    pitch: float      # -1.0 to 1.0
    yaw: float        # -1.0 to 1.0
    duration: float = 1.0  # Hold time in seconds


class ControlConfigRequest(BaseModel):
    command_timeout: float = None
    failsafe_timeout: float = None
    publish_rate: float = None
    center_rate: float = None
    roll_expo: float = None
    pitch_expo: float = None
    yaw_expo: float = None
    throttle_expo: float = None
    roll_smoothing: float = None
    pitch_smoothing: float = None
    yaw_smoothing: float = None
    throttle_smoothing: float = None
    roll_deadzone: float = None
    pitch_deadzone: float = None
    yaw_deadzone: float = None
    throttle_deadzone: float = None
    roll_rate_limit: float = None
    pitch_rate_limit: float = None
    yaw_rate_limit: float = None
    throttle_rate_limit: float = None
    throttle_pwm_idle: int = None
    throttle_pwm_max: int = None


app = FastAPI(title="Drone Control API")


_rebuild_cooldown_until = 0.0  # monotonic deadline; avoid rebuild storms
_rebuild_lock = threading.Lock()
_reboot_in_progress_until = 0.0  # suppress auto-rebuild while a reboot is being handled


def _call_service(client, request, timeout: float = 5.0):
    """
    Call a ROS2 service and wait for the result.

    On timeout, auto-rebuild all MAVROS service clients and signal failure.
    rclpy caches service GIDs; when the FC reboots, MAVROS replaces its
    services with new GIDs but cached clients keep pointing at the dead
    ones — every subsequent call silently hangs. A rebuild is the only
    way to recover without restarting the container.

    A short cooldown prevents a flood of rebuilds when many calls time out
    in sequence during an actual outage.
    """
    if client is None:
        return None

    if not client.service_is_ready():
        if not client.wait_for_service(timeout_sec=2.0):
            _maybe_rebuild_clients()
            return None

    future = client.call_async(request)

    end = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() > end:
            future.cancel()
            _maybe_rebuild_clients()
            return None
        time.sleep(0.01)

    return future.result()


def _maybe_rebuild_clients() -> None:
    """
    Rebuild MAVROS service clients if not on cooldown and no FC reboot is
    currently being handled. Non-blocking. The check-and-set is locked so
    concurrent timeouts cannot all spawn rebuild threads.
    """
    global _rebuild_cooldown_until
    with _rebuild_lock:
        now = time.monotonic()
        if now < _rebuild_cooldown_until:
            return
        if now < _reboot_in_progress_until:
            # The reboot path will rebuild clients itself once MAVROS is back.
            return
        _rebuild_cooldown_until = now + 10.0

    def _rebuild():
        try:
            node = telemetry.get_node()
            with _svc_lock:
                _create_service_clients(node)
            node.get_logger().warn("MAVROS service clients rebuilt after timeout")
        except Exception as e:
            try:
                telemetry.get_node().get_logger().error(f"Rebuild failed: {e}")
            except Exception:
                pass

    threading.Thread(target=_rebuild, daemon=True).start()


def _call_arming(arm: bool) -> tuple[bool, str]:
    """Arm or disarm. Caller must hold _svc_lock."""
    telem = telemetry.get()
    if not telem["state"]["connected"]:
        return False, "Not connected to FCU"

    req = CommandBool.Request()
    req.value = arm
    result = _call_service(_arming_client, req, timeout=5.0)

    if result and result.success:
        return True, "Armed" if arm else "Disarmed"
    return False, "Arming service unavailable" if result is None else "Command rejected by FCU"


def _set_mode(mode: str, platform: str) -> tuple[bool, str, str]:
    """Set flight mode. Caller must hold _svc_lock."""
    telem = telemetry.get()
    if not telem["state"]["connected"]:
        return False, "Not connected to FCU", ""

    mode_key = mode.lower()
    if mode_key not in MODES:
        valid = ", ".join(MODES.keys())
        return False, f"Invalid mode. Valid: {valid}", ""

    idx = 0 if platform.lower() == "ardupilot" else 1
    mode_str = MODES[mode_key][idx]

    req = SetMode.Request()
    req.custom_mode = mode_str
    result = _call_service(_set_mode_client, req, timeout=5.0)

    if result and result.mode_sent:
        return True, f"Mode set to {mode_str}", mode_str
    return False, "SetMode service unavailable" if result is None else "Mode change rejected by FCU", mode_str


# MAVLink message IDs for MESSAGE_INTERVAL requests
_MSG_INTERVAL_IDS = [30, 32, 33, 74, 245]  # ATTITUDE, LOCAL_POSITION, GPS, VFR_HUD, EXTENDED_SYS_STATE


def _enable_streams():
    """Enable all MAVROS data streams. Runs in background after startup."""
    node = telemetry.get_node()
    stream_client = node.create_client(StreamRate, "/mavros/set_stream_rate")

    # Wait for MAVROS to be ready
    if not stream_client.wait_for_service(timeout_sec=30.0):
        node.get_logger().warn("Stream rate service not available")
        return

    # Enable all stream groups (0=ALL, 1-12=specific groups)
    for sid in [0, 1, 2, 3, 6, 10, 11, 12]:
        req = StreamRate.Request()
        req.stream_id = sid
        req.message_rate = 10
        req.on_off = True
        future = stream_client.call_async(req)
        end = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < end:
            time.sleep(0.01)

    # Request specific MAVLink messages via MESSAGE_INTERVAL (cmd 511)
    if not _command_client.wait_for_service(timeout_sec=5.0):
        return
    for msg_id in _MSG_INTERVAL_IDS:
        req = CommandLong.Request()
        req.command = 511  # MAV_CMD_SET_MESSAGE_INTERVAL
        req.param1 = float(msg_id)
        req.param2 = 100000.0  # 100ms interval = 10Hz
        future = _command_client.call_async(req)
        end = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < end:
            time.sleep(0.01)

    node.get_logger().info("All data streams enabled")



def _create_service_clients(node) -> None:
    """
    Create (or recreate) all MAVROS service clients on the shared telemetry node.
    Safe to call repeatedly — existing clients are destroyed first.

    Called at startup and again after an FC reboot, since MAVROS recreates its
    service endpoints with new GIDs when the FC bounces and rclpy's client
    cache otherwise holds stale handles.
    """
    global _arming_client, _set_mode_client, _command_client
    global _wp_clear_client, _wp_push_client, _param_get_client, _param_set_client
    global _fence_push_client, _fence_clear_client

    for old in (_arming_client, _set_mode_client, _command_client,
                _wp_clear_client, _wp_push_client,
                _param_get_client, _param_set_client,
                _fence_push_client, _fence_clear_client):
        if old is not None:
            try:
                node.destroy_client(old)
            except Exception:
                pass

    _arming_client = node.create_client(CommandBool, "/mavros/cmd/arming")
    _set_mode_client = node.create_client(SetMode, "/mavros/set_mode")
    _command_client = node.create_client(CommandLong, "/mavros/cmd/command")
    _wp_clear_client = node.create_client(WaypointClear, "/mavros/mission/clear")
    _wp_push_client = node.create_client(WaypointPush, "/mavros/mission/push")
    _param_get_client = node.create_client(GetParameters, "/mavros/param/get_parameters")
    _param_set_client = node.create_client(SetParameters, "/mavros/param/set_parameters")
    _fence_push_client = node.create_client(WaypointPush, "/mavros/geofence/push")
    _fence_clear_client = node.create_client(WaypointClear, "/mavros/geofence/clear")


@app.on_event("startup")
def startup():
    telemetry.start()
    virtual_tx.start()
    camera.start()

    node = telemetry.get_node()
    _create_service_clients(node)

    # Rebuild service clients automatically whenever MAVROS re-establishes
    # its FCU link — covers reboots from any source (our reboot_fc, Mission
    # Planner, power cycle, USB unplug). The callback runs on the rclpy
    # spin thread, so we defer the actual rebuild to a background thread —
    # node.create_client() is not safe from inside a subscription callback.
    def _on_reconnect():
        def _do_rebuild():
            try:
                with _svc_lock:
                    _create_service_clients(node)
                node.get_logger().info("MAVROS reconnected — service clients rebuilt")
            except Exception as e:
                node.get_logger().error(f"Reconnect rebuild failed: {e}")
        threading.Thread(target=_do_rebuild, daemon=True).start()
    telemetry.set_reconnect_callback(_on_reconnect)

    # Compass cal raw-MAVLink decoder (subscribes on the shared telemetry node)
    compass_cal.start(node)

    # Enable streams in background so startup doesn't block
    threading.Thread(target=_enable_streams, daemon=True).start()


@app.on_event("shutdown")
def shutdown():
    compass_cal.stop()
    camera.stop()
    virtual_tx.stop()
    telemetry.stop()


# --- Command Endpoints ---

@app.post("/arm", response_model=ArmResponse)
def arm():
    telem = telemetry.get()
    if telem["state"]["armed"]:
        return ArmResponse(success=True, message="Already armed")
    with _svc_lock:
        success, message = _call_arming(True)
    return ArmResponse(success=success, message=message)


@app.post("/disarm", response_model=ArmResponse)
def disarm():
    telem = telemetry.get()
    if not telem["state"]["armed"]:
        return ArmResponse(success=True, message="Already disarmed")
    with _svc_lock:
        success, message = _call_arming(False)
    return ArmResponse(success=success, message=message)


@app.post("/disarm/force", response_model=ArmResponse)
def force_disarm():
    """Force disarm - operator emergency-abort. Always passes through to MAVROS.

    Does NOT short-circuit on the telemetry pump's `armed` snapshot; that
    snapshot can lag actual FC state by hundreds of ms (10 Hz pump + ROS
    delivery), and a stale `armed=False` read would silently skip the
    force-disarm command on an actually-armed FC. ArduPilot accepts the
    command idempotently when already disarmed, so unconditional dispatch
    is safe."""
    if not telemetry.get()["state"]["connected"]:
        return ArmResponse(success=False, message="Not connected to FCU")

    with _svc_lock:
        req = CommandLong.Request()
        req.command = 400  # MAV_CMD_COMPONENT_ARM_DISARM
        req.param1 = 0.0   # 0 = disarm
        req.param2 = 21196.0  # Force magic number
        result = _call_service(_command_client, req, timeout=5.0)

    if result and result.success:
        return ArmResponse(success=True, message="Force disarmed")
    return ArmResponse(success=False, message="Command service unavailable" if result is None else "Force disarm failed")


@app.post("/mode", response_model=ModeResponse)
def set_mode(req: ModeRequest):
    with _svc_lock:
        success, message, mode_sent = _set_mode(req.mode, req.platform)
    return ModeResponse(success=success, message=message, mode_sent=mode_sent)


@app.get("/modes")
def get_modes():
    return {"modes": list(MODES.keys()), "platforms": ["ardupilot", "px4"]}


# --- Mission Endpoints ---

@app.post("/mission", response_model=MissionResponse)
def upload_and_execute_mission(req: MissionRequest):
    """Upload mission in QGC WPL 110 format, set AUTO mode, arm, and execute."""
    telem = telemetry.get()
    if not telem["state"]["connected"]:
        return MissionResponse(success=False, message="Not connected to FCU")

    # 1. Parse waypoints (pure, no lock needed)
    try:
        waypoints = mission.parse_qgc_wpl(req.wpl)
    except ValueError as e:
        return MissionResponse(success=False, message=str(e))

    if not waypoints:
        return MissionResponse(success=False, message="No waypoints parsed")

    with _svc_lock:
        # 2. Clear existing mission
        result = _call_service(_wp_clear_client, WaypointClear.Request(), timeout=10.0)
        if not result or not result.success:
            msg = "Mission clear service unavailable" if result is None else "Failed to clear mission"
            return MissionResponse(success=False, message=msg)

        # 3. Upload new mission
        push_req = WaypointPush.Request()
        push_req.start_index = 0
        push_req.waypoints = waypoints
        result = _call_service(_wp_push_client, push_req, timeout=30.0)
        if not result or not result.success:
            msg = "Mission push service unavailable" if result is None else "Failed to upload mission"
            return MissionResponse(success=False, message=msg)
        count = result.wp_transfered

        # 4. Set AUTO mode
        mode_ok, mode_msg, _ = _set_mode("auto", "ardupilot")
        if not mode_ok:
            return MissionResponse(
                success=False,
                message=f"Mission uploaded but failed to set AUTO: {mode_msg}",
                waypoints_uploaded=count
            )

        # 5. Arm if requested
        if req.auto_arm:
            arm_ok, arm_msg = _call_arming(True)
            if not arm_ok:
                return MissionResponse(
                    success=False,
                    message=f"Mission uploaded, AUTO set, but arm failed: {arm_msg}",
                    waypoints_uploaded=count
                )

    return MissionResponse(
        success=True,
        message=f"Mission started with {count} waypoints",
        waypoints_uploaded=count
    )


@app.delete("/mission", response_model=MissionResponse)
def clear_mission_endpoint():
    """Clear current mission from Pixhawk."""
    telem = telemetry.get()
    if not telem["state"]["connected"]:
        return MissionResponse(success=False, message="Not connected to FCU")

    with _svc_lock:
        result = _call_service(_wp_clear_client, WaypointClear.Request(), timeout=10.0)

    if result and result.success:
        return MissionResponse(success=True, message="Mission cleared")
    msg = "Mission clear service unavailable" if result is None else "Failed to clear mission"
    return MissionResponse(success=False, message=msg)


# --- Control Endpoints (Virtual TX) ---

# Manual flight modes that read RC throttle as a primary input. In these
# modes ArduCopter requires a continuously-published throttle channel —
# without one, motors stay clamped at PWM 1000 even when armed (the
# `/control/arm` endpoint exists precisely to set up this throttle source
# automatically). Other modes (AUTO/GUIDED/RTL/SMART_RTL/LAND/AUTOTUNE)
# use FC-internal setpoints and ignore RC override, so virtual_tx is
# harmless but unnecessary in them.
#
# POSHOLD: like LOITER but with manual stick override — needs RC.
# ACRO: full rate control, throttle stick directly drives thrust — needs RC.
MANUAL_FLIGHT_MODES = {"STABILIZE", "ALT_HOLD", "LOITER", "POSHOLD", "ACRO"}


def _ensure_throttle_source_for_arm() -> tuple[bool, str]:
    """Pre-arm setup for manual flight: enable virtual_tx and prime the FC's
    throttle reading with a neutral hold command.

    Skipped (returns ok=True) when:
      - Current mode is autonomous (FC ignores RC override anyway)
      - A real RC source is already feeding the FC (`rc_in.channels`
        non-empty AND virtual_tx not enabled — means a physical radio is
        providing the throttle channel; we must NOT override it, the
        operator is in charge)

    Returns (ok, message). On failure, /control/arm should NOT proceed
    to the FC arming service — better to fail loudly than half-arm.

    Why this exists: ArduCopter's MOT_SPIN_ARM (motor idle PWM when armed)
    only applies once the FC has a throttle channel reading. With no
    physical TX and no virtual_tx publishing, the FC has no throttle
    source and gates motor output to PWM_MIN regardless of arm state.
    Symptom: /arm succeeds, armed:true, but motors silent.
    """
    telem = telemetry.get()
    mode = telem["state"]["mode"]
    if mode not in MANUAL_FLIGHT_MODES:
        return True, f"mode={mode} is autonomous; vtx not needed"

    vtx_status = virtual_tx.get_status()
    vtx_already_enabled = bool(vtx_status.get("enabled", False))

    # Detect a real (non-vtx) RC source by checking if the FC reports
    # RC input channels while OUR virtual_tx is NOT enabled. If both
    # are present, it's our own override echoing back — not a real radio.
    real_rc_present = (
        bool(telem["rc_in"]["channels"]) and not vtx_already_enabled
    )
    if real_rc_present:
        return True, "real RC source present; not overriding"

    ok, msg = virtual_tx.enable()
    if not ok and "already" not in msg.lower():
        return False, f"virtual_tx enable failed: {msg}"

    # Prime the FC throttle reading with a neutral hold. duration=2s gives
    # the operator a safe window to start sending real stick commands
    # before failsafe-centering kicks in. Throttle 0.0 maps to throttle_pwm_idle
    # (1100 by default) — exactly the "throttle stick at idle" condition
    # ArduCopter wants to see before MOT_SPIN_ARM kicks in.
    ok, msg = virtual_tx.send_command(
        timestamp=time.time(),
        throttle=0.0, roll=0.0, pitch=0.0, yaw=0.0,
        duration=2.0,
    )
    if not ok:
        return False, f"virtual_tx prime failed: {msg}"

    # One publish tick at 50 Hz is 20 ms; 50 ms gives ~2-3 ticks of margin
    # so the FC has reliably received an RC override frame before we ask
    # it to arm. Below 20 ms is racy.
    time.sleep(0.05)
    return True, f"vtx primed (mode={mode})"


@app.post("/control/arm", response_model=ArmResponse)
def control_arm():
    """Arm for manual flight (canonical client-facing arming endpoint).

    Sets up the virtual_tx throttle source if needed, then arms the FC.
    Use this from any client that wants to fly manually:
      * Web UI ARM button
      * Android app (HTTP and BLE/ELRS paths both)
      * Any future controller

    Behavior by current flight mode:
      STABILIZE / ALT_HOLD / LOITER (manual modes):
        - If virtual_tx already enabled OR a real RC is feeding the FC:
          just arm (don't trample existing input source)
        - Otherwise: enable virtual_tx, send a neutral throttle hold,
          then arm — motors will idle at MOT_SPIN_ARM
      AUTO / GUIDED / RTL / LAND / AUTOTUNE (autonomous modes):
        - Just arm (these modes use FC-internal setpoints and ignore
          RC override)

    For raw FC arming WITHOUT virtual_tx setup, use POST /arm directly.
    That is the right choice only when you have a real RC source already
    feeding the FC and explicitly want the bare arming command.

    Failure modes:
      503 — FCU not connected, or virtual_tx setup failed
      400 — (no body — request validation N/A)
      200 with success=false — FC rejected the arming command
    """
    telem = telemetry.get()
    if not telem["state"]["connected"]:
        return ArmResponse(success=False, message="Not connected to FCU")

    if telem["state"]["armed"]:
        return ArmResponse(success=True, message="Already armed")

    ok, prep_msg = _ensure_throttle_source_for_arm()
    if not ok:
        raise HTTPException(status_code=503, detail=prep_msg)

    with _svc_lock:
        success, arm_msg = _call_arming(True)

    # Surface the prep step in the message so operators can see in logs
    # whether vtx was set up by us or pre-existing.
    if success:
        return ArmResponse(success=True, message=f"{arm_msg} ({prep_msg})")
    return ArmResponse(success=False, message=arm_msg)


@app.post("/control/disarm", response_model=ArmResponse)
def control_disarm():
    """Disarm after manual flight (canonical client-facing disarm endpoint).

    Calls the FC arming service to disarm. Does NOT disable virtual_tx —
    the operator may rearm without re-enabling, and tearing down the
    publish thread mid-session has caused arm-then-disarm cascades in
    the past (see virtual_tx.py:Bug 1).

    For full virtual_tx teardown, use POST /control/disable explicitly.
    For emergency disarm with motors spinning, use POST /disarm/force —
    the FC accepts that idempotently and bypasses the normal disarm
    rejection that fires when motors are still under load.
    """
    telem = telemetry.get()
    if not telem["state"]["connected"]:
        return ArmResponse(success=False, message="Not connected to FCU")

    if not telem["state"]["armed"]:
        return ArmResponse(success=True, message="Already disarmed")

    with _svc_lock:
        success, message = _call_arming(False)
    return ArmResponse(success=success, message=message)


@app.post("/control/enable", response_model=ControlResponse)
def enable_control():
    """Enable virtual transmitter control mode."""
    telem = telemetry.get()
    if not telem["state"]["connected"]:
        return ControlResponse(success=False, message="Not connected to FCU")

    success, message = virtual_tx.enable()
    return ControlResponse(success=success, message=message)


@app.post("/control/disable", response_model=ControlResponse)
def disable_control():
    """Disable virtual transmitter and release RC channels."""
    success, message = virtual_tx.disable()
    return ControlResponse(success=success, message=message)


@app.get("/control/status")
def get_control_status():
    """Get virtual transmitter status."""
    return virtual_tx.get_status()


@app.post("/control/command", response_model=ControlResponse)
def send_control_command(req: ControlCommandRequest):
    """Send control command to virtual transmitter."""
    success, message = virtual_tx.send_command(
        timestamp=req.timestamp,
        throttle=req.throttle,
        roll=req.roll,
        pitch=req.pitch,
        yaw=req.yaw,
        duration=req.duration
    )
    return ControlResponse(success=success, message=message)


@app.get("/control/config")
def get_control_config():
    """Get virtual transmitter configuration."""
    return virtual_tx.get_config()


@app.post("/control/config", response_model=ControlResponse)
def update_control_config(req: ControlConfigRequest):
    """Update virtual transmitter configuration."""
    updates = {k: v for k, v in req.dict().items() if v is not None}
    if not updates:
        return ControlResponse(success=False, message="No config values provided")

    success, message = virtual_tx.update_config(updates)
    return ControlResponse(success=success, message=message)


# --- Precision Landing Endpoints ---

@app.post("/land/precision", response_model=ControlResponse)
def precision_landing():
    """Start precision landing on QR code. Switches to ALT_HOLD, uses virtual TX."""
    telem = telemetry.get()
    if not telem["state"]["connected"]:
        return ControlResponse(success=False, message="Not connected to FCU")
    if not telem["state"]["armed"]:
        return ControlResponse(success=False, message="Not armed")

    # Switch to ALT_HOLD — GPS-free, FC holds altitude while we correct laterally
    with _svc_lock:
        ok, msg, _ = _set_mode("alt_hold", "ardupilot")
    if not ok:
        return ControlResponse(success=False, message=f"Failed to set ALT_HOLD: {msg}")

    success, message = precision_land.start_landing()
    return ControlResponse(success=success, message=message)


@app.post("/land/abort", response_model=ControlResponse)
def abort_landing():
    """Abort precision landing."""
    success, message = precision_land.abort()
    return ControlResponse(success=success, message=message)


@app.get("/land/status")
def landing_status():
    """Precision landing state + QR info."""
    return precision_land.get_status()


@app.get("/land/config")
def landing_config():
    """Get precision landing tunable parameters."""
    return precision_land.get_config()


@app.post("/land/config", response_model=ControlResponse)
def update_landing_config(req: dict):
    """Update precision landing parameters at runtime."""
    success, message = precision_land.update_config(req)
    return ControlResponse(success=success, message=message)


# --- Camera Endpoints ---
# Named cameras (see camera.CAMERAS): "landing" (QR precision-land) and
# "tracking" (downstream vision container — YOLO/DeepSORT).

async def _mjpeg_generator(name: str):
    """Async MJPEG generator — uses asyncio.sleep so it never blocks the
    threadpool. Open streams with disconnected cameras cost ~zero CPU.

    Rate-limited to STREAM_MAX_FPS regardless of capture rate. Browsers
    cannot back-pressure a continuous MJPEG stream — if the generator
    yields faster than the <img> element renders, frames queue in the
    TCP receive buffer and end-to-end latency grows unboundedly (we hit
    5+ seconds of lag at 24 fps capture). Capping output to ~15 fps
    keeps the buffer empty and preserves freshness: we always yield the
    LATEST captured frame at each tick, dropping intermediate ones.

    Capture loop still runs at full rate; only the streamed view is
    rate-limited. Snapshot endpoint and vision-detect pulls are
    unaffected — they have their own self-paced fetch loops."""
    STREAM_MAX_FPS = 15
    MIN_INTERVAL_S = 1.0 / STREAM_MAX_FPS
    last_seen = -1
    last_yield_mono = 0.0
    while True:
        cam = camera.get(name)
        if cam is not None:
            cur = cam._frame_count  # GIL-atomic int read; no lock needed
            if cur != last_seen:
                now = time.monotonic()
                if now - last_yield_mono >= MIN_INTERVAL_S:
                    frame = cam.get_frame()
                    if frame:
                        yield (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n\r\n"
                            + frame + b"\r\n"
                        )
                        last_seen = cur
                        last_yield_mono = now
                else:
                    # Skip — we'll catch a fresher frame on the next tick.
                    last_seen = cur
        # 4 ms poll: ~250 Hz. Worst-case staleness <= one poll interval.
        await asyncio.sleep(0.004)


@app.get("/camera/list")
def camera_list():
    """List all configured cameras and their status."""
    return camera.list_cameras()


@app.get("/camera/{name}/stream")
async def camera_named_stream(name: str):
    """MJPEG stream for a named camera. Use in an <img src=...>."""
    if camera.get(name) is None:
        raise HTTPException(status_code=404, detail=f"Unknown camera: {name}")
    return StreamingResponse(
        _mjpeg_generator(name),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@app.get("/camera/{name}/snapshot")
def camera_named_snapshot(name: str):
    """Single JPEG frame for a named camera."""
    cam = camera.get(name)
    if cam is None:
        raise HTTPException(status_code=404, detail=f"Unknown camera: {name}")
    frame = cam.get_frame()
    if not frame:
        raise HTTPException(status_code=503, detail=f"No frame available for {name}")
    return Response(content=frame, media_type="image/jpeg")


@app.get("/camera/{name}/status")
def camera_named_status(name: str):
    """Status for a named camera."""
    status = camera.get_status(name)
    if status is None:
        raise HTTPException(status_code=404, detail=f"Unknown camera: {name}")
    return status


# Backwards-compat: /camera/stream, /camera/snapshot, /camera/status
# default to the landing camera (used by precision_land.py + old vision source).

@app.get("/camera/stream")
async def camera_stream():
    return StreamingResponse(
        _mjpeg_generator("landing"),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@app.get("/camera/snapshot")
def camera_snapshot():
    frame = camera.get_frame("landing")
    if not frame:
        raise HTTPException(status_code=503, detail="No camera frame available")
    return Response(content=frame, media_type="image/jpeg")


@app.get("/camera/status")
def camera_status():
    return camera.get_status("landing") or {"active": False}


# --- VTX broadcast routing ---------------------------------------------
# Single-URL abstraction over the multiple sources that can be sent out
# the analog VTX (J7 composite-out via the host-side vtx_renderer.py).
# Renderer pulls /vtx/snapshot — never has to know which source is
# active. Source switching happens here. See vtx_broadcast.py for the
# router internals.

import urllib.request as _vtx_urllib_request
import urllib.error as _vtx_urllib_error
import io as _vtx_io
from PIL import Image as _vtx_Image, ImageDraw as _vtx_ImageDraw

# Vision-detect annotated frame URL. Drone-control becomes a /frame
# consumer when source=vision, which keeps vision-detect's on-demand
# JPEG encoding warm (see B1 optimization in detector.py).
_VTX_VISION_URL = os.environ.get(
    "VTX_VISION_URL", "http://127.0.0.1:8081/frame"
)
# Optional ground-side feed URL — leave unset for now (placeholder),
# fall back to a static "GROUND CAM OFFLINE" indicator when source=ground.
# Production wiring: set this to wherever the downward / ground-context
# stream lives (HTTP-fetchable JPEG endpoint).
_VTX_GROUND_URL = os.environ.get("VTX_GROUND_URL")  # may be None
_VTX_FETCH_TIMEOUT_S = 0.5


def _vtx_fetch_url(url: str) -> Optional[bytes]:
    """HTTP GET → JPEG bytes, or None on any failure. Fail-soft: the
    router handles None by serving last-good-frame for the source."""
    try:
        with _vtx_urllib_request.urlopen(url, timeout=_VTX_FETCH_TIMEOUT_S) as r:
            return r.read()
    except (_vtx_urllib_error.URLError, OSError, TimeoutError):
        return None


def _vtx_make_indicator_jpeg(text: str, color=(255, 0, 0)) -> bytes:
    """Generate a static 720×576 (PAL active) indicator JPEG once at
    module load. Used for offline-source fallback. PIL-based because
    drone-control container doesn't ship cv2 (only PIL via pyzbar deps).
    Color is RGB tuple — default red text on black."""
    img = _vtx_Image.new("RGB", (720, 576), color=(0, 0, 0))
    if text:
        draw = _vtx_ImageDraw.Draw(img)
        # PIL's default font is fixed bitmap ~6x11 px. Scale by drawing
        # at native size and computing center offset. textbbox available
        # since Pillow 8.0.
        try:
            l, t, r, b = draw.textbbox((0, 0), text)
            tw, th = r - l, b - t
        except AttributeError:
            tw, th = draw.textsize(text)  # legacy Pillow
        x = (720 - tw) // 2
        y = (576 - th) // 2
        draw.text((x, y), text, fill=color)
    buf = _vtx_io.BytesIO()
    img.save(buf, format="JPEG", quality=70)
    return buf.getvalue()


# Pre-generated fallback frames — created once at module load.
_VTX_BLACK_FRAME = _vtx_make_indicator_jpeg("")
_VTX_GROUND_OFFLINE_FRAME = _vtx_make_indicator_jpeg("GROUND CAM OFFLINE")


def _vtx_fetch_front() -> Optional[bytes]:
    """Source: raw front camera (drone-control's tracking slot)."""
    return camera.get_frame("tracking")


def _vtx_fetch_vision() -> Optional[bytes]:
    """Source: vision-detect's YOLO-annotated /frame (HTTP)."""
    return _vtx_fetch_url(_VTX_VISION_URL)


def _vtx_fetch_ground() -> Optional[bytes]:
    """Source: 'ground' camera if registered + producing frames; else
    optional remote ground feed at $VTX_GROUND_URL; else the OFFLINE
    static indicator. Three-tier fallback so the source name stays
    meaningful regardless of which one is wired up.

    Priority order is local-camera-first because that's the lowest-latency
    and cheapest path — no HTTP round-trip when a Jieli (or similar) is
    plugged in directly to the drone."""
    frame = camera.get_frame("ground")
    if frame:
        return frame
    if _VTX_GROUND_URL:
        jpg = _vtx_fetch_url(_VTX_GROUND_URL)
        if jpg:
            return jpg
    return _VTX_GROUND_OFFLINE_FRAME


def _vtx_fetch_black() -> Optional[bytes]:
    """Source: solid black — useful for privacy / pause without stopping
    the renderer (keeps the analog signal coherent, just shows nothing)."""
    return _VTX_BLACK_FRAME


# Register sources at module load. Order doesn't matter operationally.
vtx_broadcast.router.register("front",  _vtx_fetch_front)
vtx_broadcast.router.register("vision", _vtx_fetch_vision)
vtx_broadcast.router.register("ground", _vtx_fetch_ground)
vtx_broadcast.router.register("black",  _vtx_fetch_black)


class VtxSourceRequest(BaseModel):
    name: str


@app.get("/vtx/source")
def vtx_get_source():
    """Return current active source + list of all registered sources."""
    return {
        "current":   vtx_broadcast.router.current(),
        "available": vtx_broadcast.router.list_sources(),
    }


@app.post("/vtx/source")
def vtx_set_source(req: VtxSourceRequest):
    """Switch active VTX source. Body: {\"name\": \"front|vision|ground|black\"}.
    Switch is instant (next /vtx/snapshot returns the new source's frame)."""
    if not vtx_broadcast.router.set_source(req.name):
        raise HTTPException(
            status_code=400,
            detail=f"unknown source: {req.name}; "
                   f"available: {vtx_broadcast.router.list_sources()}",
        )
    return {"current": req.name}


@app.get("/vtx/snapshot")
def vtx_snapshot():
    """Single JPEG of whatever source is currently active. The renderer
    polls this. Returns 503 if the active source has never produced a
    valid frame (no last-good cached either)."""
    frame = vtx_broadcast.router.get_frame()
    if not frame:
        raise HTTPException(
            status_code=503,
            detail=f"no frame from current source "
                   f"({vtx_broadcast.router.current()})",
        )
    return Response(content=frame, media_type="image/jpeg")


async def _broadcast_mjpeg_generator(router: vtx_broadcast.BroadcastRouter):
    """Generic MJPEG generator over any BroadcastRouter instance.

    Used by /vtx/stream (analog VTX channel) and /inet/stream (internet
    channel) — both feed the same kind of MJPEG over HTTP, just from
    independent routers with independent active sources.

    Same 15-fps cap as the per-camera streams — browsers can't backpressure
    MJPEG, and uncapped output causes multi-second buffer-bloat in the
    consumer's TCP queue. Capture and actual transmission rates upstream
    are unaffected — only this stream view is throttled.
    """
    STREAM_MAX_FPS = 15
    MIN_INTERVAL_S = 1.0 / STREAM_MAX_FPS
    last_yield_mono = 0.0
    last_frame_id = id(None)
    while True:
        frame = router.get_frame()
        if frame is not None:
            now = time.monotonic()
            # id(frame) cheap dedup — bytes object identity flips when a
            # new fetch occurs even if pixel content matches.
            if id(frame) != last_frame_id and now - last_yield_mono >= MIN_INTERVAL_S:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n"
                    + frame + b"\r\n"
                )
                last_yield_mono = now
                last_frame_id = id(frame)
        await asyncio.sleep(0.004)


@app.get("/vtx/stream")
async def vtx_stream():
    """MJPEG stream of the active VTX-channel source — for dashboard
    preview of what's being sent out the analog VTX."""
    return StreamingResponse(
        _broadcast_mjpeg_generator(vtx_broadcast.router),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


# --- Internet broadcast channel ----------------------------------------
#
# Independent of the analog VTX channel above. Same source registry
# (front / vision / ground / black), separate active-source selection.
# This lets the operator send a different feed to the Android app over
# internet than what's going to the analog VTX simultaneously — e.g.,
# pilot flies via raw `front` on analog while observer watches `vision`
# on Android.
#
# Architectural rule: any new consumer that needs its own switchable
# feed gets its own BroadcastRouter instance + four endpoints. Do NOT
# multiplex more consumers onto a single channel — different consumers
# fighting over `set_source()` is exactly the contention this design
# avoids.

inet_router = vtx_broadcast.BroadcastRouter(default_source="vision")
inet_router.register("front",  _vtx_fetch_front)
inet_router.register("vision", _vtx_fetch_vision)
inet_router.register("ground", _vtx_fetch_ground)
inet_router.register("black",  _vtx_fetch_black)


@app.get("/inet/source")
def inet_get_source():
    """Return the internet-channel's current active source + list of all
    registered sources. Independent of /vtx/source."""
    return {
        "current":   inet_router.current(),
        "available": inet_router.list_sources(),
    }


@app.post("/inet/source")
def inet_set_source(req: VtxSourceRequest):
    """Switch the internet-channel active source. Body: {"name": "front|vision|ground|black"}.
    Switch is instant (next /inet/snapshot returns the new source's frame).
    Does NOT affect /vtx/source — analog VTX feed is independent."""
    if not inet_router.set_source(req.name):
        raise HTTPException(
            status_code=400,
            detail=f"unknown source: {req.name}; "
                   f"available: {inet_router.list_sources()}",
        )
    return {"current": req.name}


@app.get("/inet/snapshot")
def inet_snapshot():
    """Single JPEG of whatever source is currently active on the internet
    channel. Returns 503 if the active source has never produced a valid
    frame (no last-good cached either)."""
    frame = inet_router.get_frame()
    if not frame:
        raise HTTPException(
            status_code=503,
            detail=f"no frame from current internet source "
                   f"({inet_router.current()})",
        )
    return Response(content=frame, media_type="image/jpeg")


@app.get("/inet/stream")
async def inet_stream():
    """MJPEG stream of the active internet-channel source — primary
    consumer is the Android app over its data connection, also usable
    by the web dashboard's "what Android sees" tile."""
    return StreamingResponse(
        _broadcast_mjpeg_generator(inet_router),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


# --- Top-level feed inventory ------------------------------------------

@app.get("/feeds")
def feeds():
    """Operator-friendly inventory of every video feed surface available.
    Useful for clients (Android, web UI) to discover what's wired up
    without parsing the full OpenAPI spec.
    """
    cams = camera.list_cameras()
    return {
        "cameras": {
            name: {
                "active":     status.get("active", False),
                "resolution": f"{status.get('width')}x{status.get('height')}",
                "fps":        status.get("fps"),
                "raw_paths": {
                    "snapshot": f"/camera/{name}/snapshot",
                    "stream":   f"/camera/{name}/stream",
                },
            }
            for name, status in cams.items()
        },
        "channels": {
            "vtx": {
                "purpose":   "analog 5.8 GHz VTX feed (J7 composite-out)",
                "current":   vtx_broadcast.router.current(),
                "available": vtx_broadcast.router.list_sources(),
                "paths": {
                    "source":   "/vtx/source",
                    "snapshot": "/vtx/snapshot",
                    "stream":   "/vtx/stream",
                },
            },
            "inet": {
                "purpose":   "internet feed (Android app, remote observer)",
                "current":   inet_router.current(),
                "available": inet_router.list_sources(),
                "paths": {
                    "source":   "/inet/source",
                    "snapshot": "/inet/snapshot",
                    "stream":   "/inet/stream",
                },
            },
        },
    }


# --- Vision lock endpoints (drone-side handoff contract, 2026-04-30) -----
#
# Translator (drone_bridge) POSTs here on CH9 rising edge; the body carries
# normalized 0..1023 box coords as decoded from CH13-CH16. THIS endpoint
# converts norm→pixels, runs IoU×conf class resolution against the live
# vision-detect /state, then forwards to vision-detect's POST /lock.
#
# Two distinct thresholds, two distinct jobs (named, env-tunable):
#   VISION_LOCK_IOU_THRESHOLD       (rule 3: class-vs-KCF pick)
#   VISION_LOCK_IDEMPOTENCY_THRESHOLD (rule 4: skip duplicate engage)
#
# Wire format reference: see modules/drone_bridge/vision_lock.py top docstring.

import json as _json
import urllib.request as _urllib_request
import urllib.error as _urllib_error

VISION_ORIGIN = os.environ.get("VISION_ORIGIN", "http://127.0.0.1:8081")
VISION_NORM_RANGE = 1024  # phone sends 0..1023 (10-bit)

VISION_LOCK_IOU_THRESHOLD = float(
    os.environ.get("VISION_LOCK_IOU_THRESHOLD", "0.3")
)
VISION_LOCK_IDEMPOTENCY_THRESHOLD = float(
    os.environ.get("VISION_LOCK_IDEMPOTENCY_THRESHOLD", "0.5")
)


class VisionEngageRequest(BaseModel):
    """Normalized 0..1023 box coords as sent by drone_bridge translator."""
    x1: int
    y1: int
    x2: int
    y2: int


def _vision_get_state() -> Optional[dict]:
    """Fetch /vision/state from vision-detect. Returns None on failure."""
    try:
        with _urllib_request.urlopen(f"{VISION_ORIGIN}/state", timeout=1.0) as r:
            return _json.loads(r.read().decode("utf-8"))
    except (_urllib_error.URLError, OSError, ValueError):
        return None


def _vision_post_lock(body: dict) -> tuple[bool, str]:
    """POST to vision-detect's /lock. Returns (ok, message)."""
    try:
        data = _json.dumps(body).encode("utf-8")
        req = _urllib_request.Request(
            f"{VISION_ORIGIN}/lock", data=data,
            headers={"Content-Type": "application/json"}, method="POST",
        )
        with _urllib_request.urlopen(req, timeout=2.0) as r:
            return (200 <= r.status < 300), r.read().decode("utf-8", "ignore")
    except _urllib_error.HTTPError as e:
        return False, f"HTTP {e.code}"
    except (_urllib_error.URLError, OSError) as e:
        return False, str(e)


def _iou_xyxy(a: list[int], b: list[int]) -> float:
    """Standard IoU between two [x1,y1,x2,y2] boxes in any common coord space.

    Pure function. Returns 0.0 for non-overlapping, malformed, or zero-area
    boxes — never raises."""
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1 = max(ax1, bx1); iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2); iy2 = min(ay2, by2)
    iw = max(0, ix2 - ix1); ih = max(0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    a_area = max(0, ax2 - ax1) * max(0, ay2 - ay1)
    b_area = max(0, bx2 - bx1) * max(0, by2 - by1)
    union = a_area + b_area - inter
    if union <= 0:
        return 0.0
    return inter / union


def _scale_norm_to_pixels(req: VisionEngageRequest,
                          frame_w: int, frame_h: int) -> list[int]:
    """Convert normalized 0..1023 (as sent over CH13-CH16) to source-frame
    pixel coordinates. Origin top-left, Y down-positive — matches
    detector.py's frame coords.

    The phone NEVER sends raw pixels: this is the one place in the system
    where the norm→pixel mapping is realized. Frame dims come from
    /vision/state (the live source) — we never assume a fixed resolution."""
    x1 = int(round(min(req.x1, req.x2) * frame_w / VISION_NORM_RANGE))
    y1 = int(round(min(req.y1, req.y2) * frame_h / VISION_NORM_RANGE))
    x2 = int(round(max(req.x1, req.x2) * frame_w / VISION_NORM_RANGE))
    y2 = int(round(max(req.y1, req.y2) * frame_h / VISION_NORM_RANGE))
    # Clamp to frame and ensure a minimum 2-px box so vision's tracker init
    # doesn't choke on a degenerate region.
    x1 = max(0, min(frame_w - 2, x1))
    y1 = max(0, min(frame_h - 2, y1))
    x2 = max(x1 + 1, min(frame_w - 1, x2))
    y2 = max(y1 + 1, min(frame_h - 1, y2))
    return [x1, y1, x2, y2]


@app.post("/vision/engage")
def vision_engage(req: VisionEngageRequest):
    """Resolve a normalized box to a class-lock (YOLO match) or KCF lock,
    then forward to vision-detect's /lock. Rule 3 (IoU × confidence)."""
    state = _vision_get_state()
    if state is None:
        raise HTTPException(status_code=502, detail="vision /state unreachable")

    frame_w = int(state.get("frame_w") or 0)
    frame_h = int(state.get("frame_h") or 0)
    if frame_w <= 0 or frame_h <= 0:
        raise HTTPException(status_code=502,
                            detail="vision /state missing frame_w/frame_h")

    user_box_px = _scale_norm_to_pixels(req, frame_w, frame_h)

    # Rule 3: score = IoU × confidence; threshold tunable. Tie-break is
    # lowest list index (deterministic; expected to never fire).
    best_score = -1.0
    best_det = None
    best_idx = -1
    for i, det in enumerate(state.get("dets") or []):
        det_box = det.get("box")
        det_cls = det.get("cls")
        det_conf = float(det.get("conf") or 0.0)
        if not det_box or det_cls is None:
            continue
        iou = _iou_xyxy(user_box_px, det_box)
        score = iou * det_conf
        if score > best_score:
            best_score = score
            best_det = det
            best_idx = i

    # vision-detect's /lock expects {box:[x,y,w,h], cls:str|null}.
    if best_det is not None and best_score >= VISION_LOCK_IOU_THRESHOLD:
        bx1, by1, bx2, by2 = best_det["box"]
        lock_body = {
            "box": [bx1, by1, bx2 - bx1, by2 - by1],
            "cls": best_det["cls"],
        }
        mode = f"class:{best_det['cls']}"
    else:
        ux1, uy1, ux2, uy2 = user_box_px
        lock_body = {
            "box": [ux1, uy1, ux2 - ux1, uy2 - uy1],
            "cls": None,
        }
        mode = "kcf"

    ok, msg = _vision_post_lock(lock_body)
    if not ok:
        raise HTTPException(status_code=502, detail=f"vision /lock failed: {msg}")
    return {
        "ok": True,
        "mode": mode,
        "score": round(best_score, 4) if best_score >= 0 else 0.0,
        "matched_det_index": best_idx if best_det is not None
                             and best_score >= VISION_LOCK_IOU_THRESHOLD else None,
        "frame_w": frame_w,
        "frame_h": frame_h,
        "user_box_px": user_box_px,
    }


@app.post("/vision/cancel-lock")
def vision_cancel_lock():
    """Drop the active vision lock — forwards {unlock:true} to vision-detect."""
    ok, msg = _vision_post_lock({"unlock": True})
    if not ok:
        raise HTTPException(status_code=502, detail=f"vision /lock failed: {msg}")
    return {"ok": True}


@app.post("/vision/follow")
def vision_follow():
    """Trigger drone-side follow-mode autonomy on the current lock.

    HAPPY-PATH STUB: full follow autonomy (drone slews to keep target
    centered) is a separate workstream. This logs the event and returns
    200 so the translator's rising-edge dispatch is observable end-to-end.
    """
    print("[vision] /vision/follow received (stub)")
    return {"ok": True, "stub": True}


@app.post("/vision/abort")
def vision_abort():
    """Halt the current vision-driven motion.

    HAPPY-PATH STUB: the abort target (disarm? RTL? mode switch?) is a
    decision yet to be made. Logs + 200 keeps the translator dispatch
    path observable.
    """
    print("[vision] /vision/abort received (stub)")
    return {"ok": True, "stub": True}


# --- FC Parameter Endpoints ---
# _param_get_client and _param_set_client are created (and recreated on
# reconnect) by _create_service_clients(); see top of file.


@app.get("/params/{param_id}")
def get_param(param_id: str):
    """Read a single FC parameter via MAVROS ROS2 param interface."""
    req = GetParameters.Request()
    req.names = [param_id]
    with _svc_lock:
        result = _call_service(_param_get_client, req, timeout=5.0)
    if not result or not result.values:
        raise HTTPException(status_code=404, detail=f"Param {param_id} not found")
    v = result.values[0]
    # PARAMETER_NOT_SET (=0) means MAVROS returned a default-valued reply
    # because the FC doesn't have this param. Without this check the path
    # falls through and returns 200 with value=0.0 — the BLE smoke test
    # (and any well-behaved client) reads that as a real param value.
    # Batch endpoint already maps this case to None; aligning the single-GET.
    if v.type == ParameterType.PARAMETER_NOT_SET:
        raise HTTPException(status_code=404, detail=f"Param {param_id} not found")
    if v.type == ParameterType.PARAMETER_DOUBLE:
        val = v.double_value
    elif v.type == ParameterType.PARAMETER_INTEGER:
        val = float(v.integer_value)
    else:
        val = v.double_value if v.double_value != 0.0 else float(v.integer_value)
    return {"param_id": param_id, "value": val}


@app.post("/params/{param_id}")
def set_param(param_id: str, body: dict):
    """Write a single FC parameter via MAVROS ROS2 param interface."""
    if "value" not in body:
        raise HTTPException(status_code=400, detail="Missing 'value' field")
    # Read current type first
    get_req = GetParameters.Request()
    get_req.names = [param_id]
    with _svc_lock:
        cur = _call_service(_param_get_client, get_req, timeout=5.0)
    p = Parameter()
    p.name = param_id
    if cur and cur.values and cur.values[0].type == ParameterType.PARAMETER_INTEGER:
        p.value.type = ParameterType.PARAMETER_INTEGER
        p.value.integer_value = int(body["value"])
    else:
        p.value.type = ParameterType.PARAMETER_DOUBLE
        p.value.double_value = float(body["value"])
    req = SetParameters.Request()
    req.parameters = [p]
    with _svc_lock:
        result = _call_service(_param_set_client, req, timeout=5.0)
    if not result or not result.results or not result.results[0].successful:
        reason = result.results[0].reason if result and result.results else "unknown"
        raise HTTPException(status_code=500, detail=f"Failed to set {param_id}: {reason}")
    return {"param_id": param_id, "value": float(body["value"]), "success": True}


@app.post("/params/batch/get")
def get_params_batch(body: dict):
    """Read multiple FC parameters."""
    if "params" not in body:
        raise HTTPException(status_code=400, detail="Missing 'params' list")
    req = GetParameters.Request()
    req.names = body["params"]
    with _svc_lock:
        result = _call_service(_param_get_client, req, timeout=10.0)
    results = {}
    if result and result.values:
        for name, v in zip(body["params"], result.values):
            if v.type == ParameterType.PARAMETER_DOUBLE:
                results[name] = v.double_value
            elif v.type == ParameterType.PARAMETER_INTEGER:
                results[name] = float(v.integer_value)
            else:
                results[name] = None
    else:
        for name in body["params"]:
            results[name] = None
    return results


# --- Telemetry Endpoints ---

@app.get("/telemetry")
def get_all_telemetry():
    return telemetry.get()


@app.get("/telemetry/state")
def get_state():
    return telemetry.get()["state"]


@app.get("/telemetry/battery")
def get_battery():
    return telemetry.get()["battery"]


@app.get("/telemetry/gps")
def get_gps():
    return telemetry.get()["gps"]


@app.get("/telemetry/local_position")
def get_local_position():
    return telemetry.get()["local_position"]


@app.get("/telemetry/orientation")
def get_orientation():
    return telemetry.get()["orientation"]


@app.get("/telemetry/velocity")
def get_velocity():
    return telemetry.get()["velocity"]


@app.get("/telemetry/imu")
def get_imu():
    return telemetry.get()["imu"]


@app.get("/telemetry/mag")
def get_mag():
    return telemetry.get()["mag"]


@app.get("/telemetry/baro")
def get_baro():
    return telemetry.get()["baro"]


@app.get("/telemetry/vfr_hud")
def get_vfr_hud():
    return telemetry.get()["vfr_hud"]


@app.get("/telemetry/rc_in")
def get_rc_in():
    return telemetry.get()["rc_in"]


@app.get("/telemetry/rc_out")
def get_rc_out():
    return telemetry.get()["rc_out"]


@app.get("/telemetry/home")
def get_home():
    return telemetry.get()["home"]


@app.get("/telemetry/extended_state")
def get_extended_state():
    return telemetry.get()["extended_state"]


@app.get("/telemetry/messages")
def get_messages():
    return telemetry.get()["messages"]


# ─────────────────────────────────────────────────────────────
# Calibration Endpoints
# Namespace: /calibration/*
# All commands go through MAVROS /cmd/command (MAV_CMD).
# Designed for reuse: web UI, master application, CLI, scripts.
# ─────────────────────────────────────────────────────────────

MAV_CMD_DO_MOTOR_TEST = 209
MAV_CMD_PREFLIGHT_CALIBRATION = 241
MAV_CMD_DO_START_MAG_CAL = 42424
MAV_CMD_DO_ACCEPT_MAG_CAL = 42425
MAV_CMD_DO_CANCEL_MAG_CAL = 42426
MAV_CMD_ACCELCAL_VEHICLE_POS = 42429
MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246

# Accelerometer 6-position flow
ACCEL_POSITIONS = [
    (1, "LEVEL",     "Place the drone flat on a level surface, right-side up."),
    (2, "LEFT SIDE", "Roll the drone onto its left side."),
    (3, "RIGHT SIDE","Roll the drone onto its right side."),
    (4, "NOSE DOWN", "Tip the drone nose-down (pointing at the floor)."),
    (5, "NOSE UP",   "Tip the drone nose-up (pointing at the ceiling)."),
    (6, "ON BACK",   "Flip the drone upside-down, on its back."),
]

# Motor test throttle types (per MAV_CMD_DO_MOTOR_TEST spec)
MOTOR_TEST_THROTTLE_PERCENT = 0
MOTOR_TEST_THROTTLE_PWM = 1
MOTOR_TEST_THROTTLE_PILOT = 2

# Motor test ordering (ArduCopter)
MOTOR_TEST_ORDER_DEFAULT = 0   # Manual order (param5 = count starting from param1)
MOTOR_TEST_ORDER_BOARD = 1     # Board order
MOTOR_TEST_ORDER_FRAME = 2     # Frame order


class MotorTestRequest(BaseModel):
    motor: int = 1                 # 1-based motor index (starting motor for sequence)
    throttle_pct: float = 10.0     # 0-100
    duration_s: float = 2.0        # per-motor duration
    mode: Literal["single", "sequence"] = "single"
    motor_count: int = 1           # number of motors to run sequentially (sequence mode)


class CalibrationResponse(BaseModel):
    success: bool
    message: str
    result_code: Optional[int] = None


def _send_mav_command(cmd: int, p1=0.0, p2=0.0, p3=0.0, p4=0.0,
                      p5=0.0, p6=0.0, p7=0.0, timeout: float = 5.0):
    """Send a MAVLink COMMAND_LONG via MAVROS. Returns the service result or None."""
    if _command_client is None:
        return None
    req = CommandLong.Request()
    req.command = cmd
    req.param1 = float(p1)
    req.param2 = float(p2)
    req.param3 = float(p3)
    req.param4 = float(p4)
    req.param5 = float(p5)
    req.param6 = float(p6)
    req.param7 = float(p7)
    return _call_service(_command_client, req, timeout=timeout)


def _require_disarmed():
    telem = telemetry.get()
    if not telem["state"]["connected"]:
        raise HTTPException(status_code=503, detail="Not connected to FCU")
    if telem["state"]["armed"]:
        raise HTTPException(status_code=409, detail="Cannot calibrate while armed")


def _cal_response(result, ok_msg: str, fail_prefix: str) -> CalibrationResponse:
    if result is None:
        return CalibrationResponse(success=False, message="Command service unavailable")
    if result.success:
        return CalibrationResponse(success=True, message=ok_msg, result_code=int(result.result))
    return CalibrationResponse(
        success=False,
        message=f"{fail_prefix} (MAV_RESULT={int(result.result)})",
        result_code=int(result.result),
    )


@app.post("/calibration/motor_test", response_model=CalibrationResponse)
def calibration_motor_test(req: MotorTestRequest):
    """
    Spin motor(s) at a throttle percentage for a duration.

    SAFETY: Remove propellers. FC will reject if armed. Ground station is
    responsible for confirming props-off before calling.

    Modes:
      single   — spin one motor (req.motor) at req.throttle_pct for req.duration_s.
      sequence — spin motor_count motors sequentially in frame order, starting at req.motor.
    """
    if not (1 <= req.motor <= 12):
        raise HTTPException(status_code=422, detail="motor must be 1..12")
    if not (0 <= req.throttle_pct <= 100):
        raise HTTPException(status_code=422, detail="throttle_pct must be 0..100")
    if not (0.1 <= req.duration_s <= 10.0):
        raise HTTPException(status_code=422, detail="duration_s must be 0.1..10")
    if not (1 <= req.motor_count <= 12):
        raise HTTPException(status_code=422, detail="motor_count must be 1..12")

    _require_disarmed()

    if req.mode == "single":
        motor = req.motor
        count = 1
        order = MOTOR_TEST_ORDER_DEFAULT
    else:  # sequence
        motor = req.motor
        count = req.motor_count
        order = MOTOR_TEST_ORDER_FRAME

    with _svc_lock:
        result = _send_mav_command(
            MAV_CMD_DO_MOTOR_TEST,
            p1=motor,
            p2=MOTOR_TEST_THROTTLE_PERCENT,
            p3=req.throttle_pct,
            p4=req.duration_s,
            p5=count,
            p6=order,
            timeout=5.0,
        )

    ok_msg = (
        f"Motor test started: {req.mode} m{motor} @ {req.throttle_pct:.0f}% "
        f"for {req.duration_s:.1f}s" + (f" ×{count}" if count > 1 else "")
    )
    return _cal_response(result, ok_msg, "Motor test rejected")


@app.post("/calibration/motor_test/stop", response_model=CalibrationResponse)
def calibration_motor_test_stop():
    """
    Emergency stop for motor test. Sends 0% throttle and force-disarms
    as a belt-and-suspenders measure in case motors are still spinning.
    """
    with _svc_lock:
        _send_mav_command(
            MAV_CMD_DO_MOTOR_TEST,
            p1=1, p2=MOTOR_TEST_THROTTLE_PERCENT, p3=0.0, p4=0.0,
            p5=0, p6=MOTOR_TEST_ORDER_DEFAULT, timeout=2.0,
        )
        dreq = CommandLong.Request()
        dreq.command = 400  # MAV_CMD_COMPONENT_ARM_DISARM
        dreq.param1 = 0.0
        dreq.param2 = 21196.0  # force magic
        _call_service(_command_client, dreq, timeout=2.0)
    return CalibrationResponse(success=True, message="Motor test stopped")


# --- One-click preflight calibrations ---

@app.post("/calibration/gyro", response_model=CalibrationResponse)
def calibration_gyro():
    """Calibrate gyros. Drone must be still."""
    _require_disarmed()
    with _svc_lock:
        result = _send_mav_command(MAV_CMD_PREFLIGHT_CALIBRATION, p1=1, timeout=20.0)
    return _cal_response(result, "Gyro calibration complete", "Gyro calibration rejected")


@app.post("/calibration/level_horizon", response_model=CalibrationResponse)
def calibration_level_horizon():
    """
    One-click AHRS trim: sets AHRS_TRIM_X/Y from current attitude.
    Place drone on a known-level surface before calling.
    """
    _require_disarmed()
    with _svc_lock:
        result = _send_mav_command(MAV_CMD_PREFLIGHT_CALIBRATION, p5=2, timeout=20.0)
    return _cal_response(result, "Level horizon captured", "Level calibration rejected")


@app.post("/calibration/baro", response_model=CalibrationResponse)
def calibration_baro():
    """Reset barometer ground pressure (home altitude datum)."""
    _require_disarmed()
    with _svc_lock:
        result = _send_mav_command(MAV_CMD_PREFLIGHT_CALIBRATION, p3=1, timeout=10.0)
    return _cal_response(result, "Barometer zeroed", "Baro calibration rejected")


@app.get("/calibration/status")
def calibration_status():
    """Aggregate calibration-relevant state for UI gating."""
    telem = telemetry.get()
    return {
        "connected": telem["state"]["connected"],
        "armed": telem["state"]["armed"],
        "mode": telem["state"]["mode"],
        "available": {
            "motor_test": True,
            "gyro": True,
            "level_horizon": True,
            "baro": True,
            "compass": True,
            "accelerometer": True,
        },
    }


# --- Compass (onboard magnetometer) cal ---
#
# MAVROS Humble doesn't expose mag_cal progress topics, so we fire MAV_CMDs
# directly and let the frontend parse progress/result from the FCU statustext
# stream (already consumed by the FCU Console).

class CompassStartRequest(BaseModel):
    autosave: bool = True
    retry_on_fail: bool = False


@app.post("/calibration/compass/start", response_model=CalibrationResponse)
def calibration_compass_start(req: CompassStartRequest = CompassStartRequest()):
    """
    Start onboard compass calibration on all compasses.
    After starting, rotate the drone through all orientations for ~60s.
    Progress and result are reported via FCU statustext (FCU Console).
    """
    _require_disarmed()
    compass_cal.reset()
    with _svc_lock:
        result = _send_mav_command(
            MAV_CMD_DO_START_MAG_CAL,
            p1=0,                          # bitmask of compasses, 0 = all
            p2=1.0 if req.retry_on_fail else 0.0,
            p3=1.0 if req.autosave else 0.0,
            p4=0.0,                        # delay seconds
            p5=0.0,                        # autoreboot
            timeout=10.0,
        )
    return _cal_response(
        result,
        "Compass calibration started — rotate drone through all axes",
        "Compass cal start rejected",
    )


@app.post("/calibration/compass/accept", response_model=CalibrationResponse)
def calibration_compass_accept():
    """Accept current mag cal results (only needed if autosave was off)."""
    with _svc_lock:
        result = _send_mav_command(MAV_CMD_DO_ACCEPT_MAG_CAL, p1=0, timeout=5.0)
    return _cal_response(result, "Compass cal accepted", "Accept rejected")


@app.post("/calibration/compass/cancel", response_model=CalibrationResponse)
def calibration_compass_cancel():
    """Cancel an in-progress mag cal."""
    with _svc_lock:
        result = _send_mav_command(MAV_CMD_DO_CANCEL_MAG_CAL, p1=0, timeout=5.0)
    return _cal_response(result, "Compass cal cancelled", "Cancel rejected")


def _recreate_clients_after_reboot(delay_s: float = 12.0) -> None:
    """
    After the FC reboots, MAVROS recreates its service endpoints with new
    GIDs. rclpy client caches otherwise hold stale handles, making every
    subsequent service call return None ("Command service unavailable").

    We destroy and recreate all MAVROS clients once MAVROS has had time to
    reattach. While this runs, `_reboot_in_progress_until` suppresses
    `_maybe_rebuild_clients()` so we don't double-rebuild and race.
    """
    global _reboot_in_progress_until
    _reboot_in_progress_until = time.monotonic() + delay_s + 5.0
    node = telemetry.get_node()  # cache the node ref before sleeping
    time.sleep(delay_s)
    try:
        with _svc_lock:
            _create_service_clients(node)
        node.get_logger().info("Service clients recreated after FC reboot")
    except Exception as e:
        try:
            node.get_logger().error(f"Post-reboot reconnect failed: {e}")
        except Exception:
            pass
    finally:
        _reboot_in_progress_until = 0.0


@app.post("/system/reconnect_mavros", response_model=CalibrationResponse)
def system_reconnect_mavros():
    """
    Manually rebuild all MAVROS service clients. Use this if service calls
    start returning 'service unavailable' after the FC has been rebooted
    through a path that didn't trigger the auto-reconnect edge detector
    (e.g. a reboot where /mavros/state never flipped to disconnected).
    """
    try:
        node = telemetry.get_node()
        with _svc_lock:
            _create_service_clients(node)
        return CalibrationResponse(success=True, message="MAVROS service clients rebuilt")
    except Exception as e:
        return CalibrationResponse(success=False, message=f"Rebuild failed: {e}")


@app.post("/calibration/reboot_fc", response_model=CalibrationResponse)
def calibration_reboot_fc():
    """
    Reboot the flight controller autopilot. Required after compass cal so the
    new COMPASS_OFS_* values load. Refuses while armed. Automatically rebuilds
    MAVROS service clients ~12s later so subsequent calls don't hit stale
    endpoints.
    """
    _require_disarmed()
    with _svc_lock:
        result = _send_mav_command(
            MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            p1=1.0,   # 1 = reboot autopilot
            timeout=5.0,
        )
    if result is not None and result.success:
        threading.Thread(target=_recreate_clients_after_reboot, daemon=True).start()
    return _cal_response(result, "FC reboot sent", "Reboot rejected")


@app.get("/calibration/compass/progress")
def calibration_compass_progress():
    """
    Live compass cal progress, decoded from MAG_CAL_PROGRESS / MAG_CAL_REPORT
    MAVLink messages captured off /uas1/mavlink_source.

    Returns:
        active: bool — still collecting samples
        progress: latest per-compass progress (percent, cal_status, attempt, direction)
        report:   terminal result (fitness, offsets, autosaved flag) once available
    """
    return compass_cal.get_state()


# --- Accelerometer 6-position cal ---
#
# ArduCopter flow:
#   1. GCS sends MAV_CMD_PREFLIGHT_CALIBRATION param5=1
#   2. FC emits statustext "Place vehicle LEVEL and press any key"
#   3. GCS orients the drone, then sends MAV_CMD_ACCELCAL_VEHICLE_POS
#      with param1 = 1 (level). FC captures samples.
#   4. FC emits statustext for next position (LEFT, RIGHT, NOSE DOWN,
#      NOSE UP, ON BACK). GCS sends positions 2..6 in sequence.
#   5. FC emits "Calibration successful" (or failure) on completion.
#
# Position descriptions are exposed under /calibration/accel/positions so
# any client can render the same wording without hardcoding.


class AccelPositionRequest(BaseModel):
    position: int  # 1..6


@app.get("/calibration/accel/positions")
def calibration_accel_positions():
    """Return the ordered list of accel cal positions with human descriptions."""
    return [
        {"position": n, "label": label, "instruction": instr}
        for (n, label, instr) in ACCEL_POSITIONS
    ]


@app.post("/calibration/accel/start", response_model=CalibrationResponse)
def calibration_accel_start():
    """Begin accelerometer 6-position calibration. Watch FCU Console for prompts."""
    _require_disarmed()
    with _svc_lock:
        result = _send_mav_command(
            MAV_CMD_PREFLIGHT_CALIBRATION,
            p5=1.0,  # accel cal (full 6-point)
            timeout=10.0,
        )
    return _cal_response(result, "Accel cal started — place drone LEVEL", "Accel cal start rejected")


@app.post("/calibration/accel/position", response_model=CalibrationResponse)
def calibration_accel_position(req: AccelPositionRequest):
    """
    Acknowledge the current accel cal position (1..6). FC captures samples
    at this orientation and advances to the next position.
    """
    if not (1 <= req.position <= 6):
        raise HTTPException(status_code=422, detail="position must be 1..6")
    with _svc_lock:
        result = _send_mav_command(
            MAV_CMD_ACCELCAL_VEHICLE_POS,
            p1=float(req.position),
            timeout=5.0,
        )
    label = ACCEL_POSITIONS[req.position - 1][1]
    return _cal_response(result, f"Position {req.position} ({label}) captured", "Position ack rejected")


@app.post("/calibration/accel/simple", response_model=CalibrationResponse)
def calibration_accel_simple():
    """
    'Simple' accel cal — level only (param5=4 in ArduPilot).
    Faster alternative to full 6-point when drone is already roughly flat.
    """
    _require_disarmed()
    with _svc_lock:
        result = _send_mav_command(
            MAV_CMD_PREFLIGHT_CALIBRATION,
            p5=4.0,
            timeout=10.0,
        )
    return _cal_response(result, "Simple accel cal sent", "Simple accel cal rejected")


# ─────────────────────────────────────────────────────────────
# Safety (Failsafes) & Geofence
# Namespaces: /safety/*, /fence/*
#
# Operator-facing wrappers over ArduPilot's FS_*, BATT_*, FENCE_* params.
# UI sends semantic labels ("rtl", "land", "smart_rtl"); this layer maps
# them to the underlying MAV numeric enums so the frontend never has to.
# ─────────────────────────────────────────────────────────────

# Action enums (UI label -> ArduPilot value).
_RC_FS_ACTIONS = {
    "disabled":  0,
    "rtl":       1,   # Always RTL (Land if no GPS)
    "land":      3,
    "smart_rtl": 5,   # SmartRTL then Land
}

_GCS_FS_ACTIONS = {
    "disabled":  0,
    "rtl":       1,
    "smart_rtl": 4,   # SmartRTL then Land
    "land":      5,
    "brake":     7,
}

_BATT_LOW_ACTIONS = {
    "none":      0,   # warn only (no failsafe action taken)
    "rtl":       2,
    "land":      1,
    "smart_rtl": 3,
}

_BATT_CRT_ACTIONS = {
    "none":      0,   # warn only
    "land":      1,
    "smart_rtl": 3,
    "terminate": 5,   # Disarm in air — DANGEROUS
}

_FENCE_ACTIONS = {
    "report":         0,
    "rtl":            1,
    "land":           2,
    "smart_rtl":      3,
    "brake":          4,
    "smart_rtl_land": 5,
}

# FENCE_TYPE bitmask bits
_FENCE_BIT_CEILING = 1   # bit 0
_FENCE_BIT_CIRCLE  = 2   # bit 1
_FENCE_BIT_POLYGON = 4   # bit 2
_FENCE_BIT_FLOOR   = 8   # bit 3

# FS_OPTIONS bits we expose in the UI. Other bits (not in this set) are
# preserved on write so we don't stomp unrelated operator preferences.
_FSOPT_CONTINUE_AUTO_RC       = 1    # bit 0
_FSOPT_CONTINUE_AUTO_GCS      = 2    # bit 1
_FSOPT_DONT_INTERRUPT_LANDING = 8    # bit 3


def _read_params(names: list) -> dict:
    """Batch param read. Returns {name: float_value or None}."""
    req = GetParameters.Request()
    req.names = names
    with _svc_lock:
        result = _call_service(_param_get_client, req, timeout=10.0)
    out = {}
    if result and result.values:
        for name, v in zip(names, result.values):
            if v.type == ParameterType.PARAMETER_DOUBLE:
                out[name] = v.double_value
            elif v.type == ParameterType.PARAMETER_INTEGER:
                out[name] = float(v.integer_value)
            else:
                out[name] = None
    else:
        for name in names:
            out[name] = None
    return out


def _write_params(pairs: dict) -> tuple[bool, str]:
    """
    Batch param write. Does one GetParameters round-trip first to pick up
    the ROS2 parameter type for each name (INTEGER vs DOUBLE), then one
    SetParameters call with matched types. Avoids int8/int16/float
    confusion from hardcoded type tables.
    """
    if not pairs:
        return True, "No changes"
    names = list(pairs.keys())
    with _svc_lock:
        get_req = GetParameters.Request()
        get_req.names = names
        got = _call_service(_param_get_client, get_req, timeout=10.0)
        if not got or not got.values:
            return False, "Param service unavailable"

        set_req = SetParameters.Request()
        for (name, value), v in zip(pairs.items(), got.values):
            p = Parameter()
            p.name = name
            if v.type == ParameterType.PARAMETER_INTEGER:
                p.value.type = ParameterType.PARAMETER_INTEGER
                p.value.integer_value = int(value)
            else:
                p.value.type = ParameterType.PARAMETER_DOUBLE
                p.value.double_value = float(value)
            set_req.parameters.append(p)

        res = _call_service(_param_set_client, set_req, timeout=10.0)

    if not res or not res.results:
        return False, "Set service unavailable"
    fails = [f"{n}: {r.reason}" for (n, _), r in zip(pairs.items(), res.results) if not r.successful]
    if fails:
        return False, "; ".join(fails)
    return True, f"{len(pairs)} param(s) written"


def _key_from_val(enum_map: dict, val) -> str:
    """Reverse lookup for UI enum dicts — returns 'unknown' for unmapped."""
    if val is None:
        return "unknown"
    iv = int(val)
    for k, v in enum_map.items():
        if v == iv:
            return k
    return "unknown"


# --- Request models ---

class RcFailsafeRequest(BaseModel):
    action: str

class GcsFailsafeRequest(BaseModel):
    action: str
    timeout_s: float = 5.0

class BatteryFailsafeRequest(BaseModel):
    low_volt: float
    low_action: str
    crit_volt: float
    crit_action: str
    timer_s: float = 10.0

class FailsafeOptionsRequest(BaseModel):
    dont_interrupt_landing: bool = True
    continue_auto_rc: bool = False
    continue_auto_gcs: bool = False

class FenceConfigRequest(BaseModel):
    enable: bool
    autoenable: bool              # True -> FENCE_AUTOENABLE=3, False -> 0
    action: str
    margin_m: float = 2.0
    ceiling_enabled: bool = False
    ceiling_m: float = 100.0
    floor_enabled: bool = False
    floor_m: float = -5.0
    circle_enabled: bool = False
    circle_radius_m: float = 300.0
    polygon_enabled: bool = False

class FencePolygonRequest(BaseModel):
    points: list                  # [[lat, lon], ...]
    inclusion: bool = True


# ─── GET /safety/config ───────────────────────────────────────────────────

@app.get("/safety/config")
def safety_config():
    """Read current failsafe settings, decoded into UI labels."""
    params = _read_params([
        "FS_THR_ENABLE",
        "FS_GCS_ENABLE", "FS_GCS_TIMEOUT",
        "BATT_LOW_VOLT", "BATT_CRT_VOLT",
        "BATT_FS_LOW_ACT", "BATT_FS_CRT_ACT", "BATT_LOW_TIMER",
        "FS_OPTIONS",
    ])
    fso = int(params.get("FS_OPTIONS") or 0)
    return {
        "rc": {
            "action": _key_from_val(_RC_FS_ACTIONS, params.get("FS_THR_ENABLE")),
            "raw": params.get("FS_THR_ENABLE"),
        },
        "gcs": {
            "action": _key_from_val(_GCS_FS_ACTIONS, params.get("FS_GCS_ENABLE")),
            "timeout_s": params.get("FS_GCS_TIMEOUT"),
            "raw": params.get("FS_GCS_ENABLE"),
        },
        "battery": {
            "low_volt": params.get("BATT_LOW_VOLT"),
            "low_action": _key_from_val(_BATT_LOW_ACTIONS, params.get("BATT_FS_LOW_ACT")),
            "crit_volt": params.get("BATT_CRT_VOLT"),
            "crit_action": _key_from_val(_BATT_CRT_ACTIONS, params.get("BATT_FS_CRT_ACT")),
            "timer_s": params.get("BATT_LOW_TIMER"),
        },
        "options": {
            "dont_interrupt_landing": bool(fso & _FSOPT_DONT_INTERRUPT_LANDING),
            "continue_auto_rc": bool(fso & _FSOPT_CONTINUE_AUTO_RC),
            "continue_auto_gcs": bool(fso & _FSOPT_CONTINUE_AUTO_GCS),
            "raw": fso,
        },
    }


@app.post("/safety/rc")
def safety_rc(req: RcFailsafeRequest):
    """RC (throttle) failsafe — fires on RC link loss."""
    if req.action not in _RC_FS_ACTIONS:
        raise HTTPException(status_code=422,
                            detail=f"Invalid action. Options: {list(_RC_FS_ACTIONS.keys())}")
    ok, msg = _write_params({"FS_THR_ENABLE": _RC_FS_ACTIONS[req.action]})
    return {"success": ok, "message": msg}


@app.post("/safety/gcs")
def safety_gcs(req: GcsFailsafeRequest):
    """GCS (MAVLink heartbeat) failsafe."""
    if req.action not in _GCS_FS_ACTIONS:
        raise HTTPException(status_code=422,
                            detail=f"Invalid action. Options: {list(_GCS_FS_ACTIONS.keys())}")
    if req.action != "disabled" and not (1.0 <= req.timeout_s <= 30.0):
        raise HTTPException(status_code=422, detail="timeout_s must be 1..30")
    ok, msg = _write_params({
        "FS_GCS_ENABLE":  _GCS_FS_ACTIONS[req.action],
        "FS_GCS_TIMEOUT": req.timeout_s,
    })
    return {"success": ok, "message": msg}


@app.post("/safety/battery")
def safety_battery(req: BatteryFailsafeRequest):
    """Two-tier battery failsafe (low + critical voltage)."""
    if req.low_action not in _BATT_LOW_ACTIONS:
        raise HTTPException(status_code=422,
                            detail=f"Invalid low_action. Options: {list(_BATT_LOW_ACTIONS.keys())}")
    if req.crit_action not in _BATT_CRT_ACTIONS:
        raise HTTPException(status_code=422,
                            detail=f"Invalid crit_action. Options: {list(_BATT_CRT_ACTIONS.keys())}")
    if req.low_volt < 0 or req.crit_volt < 0:
        raise HTTPException(status_code=422, detail="Voltages must be >= 0")
    if req.crit_volt > 0 and req.low_volt > 0 and req.crit_volt >= req.low_volt:
        raise HTTPException(status_code=422, detail="crit_volt must be lower than low_volt")
    ok, msg = _write_params({
        "BATT_LOW_VOLT":   req.low_volt,
        "BATT_CRT_VOLT":   req.crit_volt,
        "BATT_FS_LOW_ACT": _BATT_LOW_ACTIONS[req.low_action],
        "BATT_FS_CRT_ACT": _BATT_CRT_ACTIONS[req.crit_action],
        "BATT_LOW_TIMER":  int(req.timer_s),
    })
    return {"success": ok, "message": msg}


@app.post("/safety/options")
def safety_options(req: FailsafeOptionsRequest):
    """FS_OPTIONS bitmask. Preserves bits not exposed in the UI."""
    cur = int(_read_params(["FS_OPTIONS"]).get("FS_OPTIONS") or 0)
    exposed = (_FSOPT_DONT_INTERRUPT_LANDING
               | _FSOPT_CONTINUE_AUTO_RC
               | _FSOPT_CONTINUE_AUTO_GCS)
    new = cur & ~exposed
    if req.dont_interrupt_landing: new |= _FSOPT_DONT_INTERRUPT_LANDING
    if req.continue_auto_rc:       new |= _FSOPT_CONTINUE_AUTO_RC
    if req.continue_auto_gcs:      new |= _FSOPT_CONTINUE_AUTO_GCS
    ok, msg = _write_params({"FS_OPTIONS": new})
    return {"success": ok, "message": msg, "raw": new}


@app.post("/safety/defaults")
def safety_defaults():
    """Sane starting failsafes for a drone with an RC GCS link."""
    ok, msg = _write_params({
        "FS_THR_ENABLE":   _RC_FS_ACTIONS["rtl"],
        "FS_GCS_ENABLE":   _GCS_FS_ACTIONS["rtl"],
        "FS_GCS_TIMEOUT":  5.0,
        "BATT_LOW_VOLT":   10.5,
        "BATT_CRT_VOLT":   10.0,
        "BATT_FS_LOW_ACT": _BATT_LOW_ACTIONS["rtl"],
        "BATT_FS_CRT_ACT": _BATT_CRT_ACTIONS["land"],
        "BATT_LOW_TIMER":  10,
        "FS_OPTIONS":      _FSOPT_DONT_INTERRUPT_LANDING,
    })
    return {"success": ok, "message": msg}


# ─── Geofence ─────────────────────────────────────────────────────────────

@app.get("/fence/config")
def fence_config():
    """Read current geofence config (types, bounds, action)."""
    params = _read_params([
        "FENCE_ENABLE", "FENCE_AUTOENABLE", "FENCE_ACTION", "FENCE_TYPE",
        "FENCE_MARGIN", "FENCE_ALT_MAX", "FENCE_ALT_MIN", "FENCE_RADIUS",
    ])
    ft = int(params.get("FENCE_TYPE") or 0)
    return {
        "enable": bool(int(params.get("FENCE_ENABLE") or 0)),
        "autoenable": int(params.get("FENCE_AUTOENABLE") or 0) == 3,
        "action": _key_from_val(_FENCE_ACTIONS, params.get("FENCE_ACTION")),
        "margin_m": params.get("FENCE_MARGIN"),
        "ceiling_enabled": bool(ft & _FENCE_BIT_CEILING),
        "ceiling_m": params.get("FENCE_ALT_MAX"),
        "floor_enabled": bool(ft & _FENCE_BIT_FLOOR),
        "floor_m": params.get("FENCE_ALT_MIN"),
        "circle_enabled": bool(ft & _FENCE_BIT_CIRCLE),
        "circle_radius_m": params.get("FENCE_RADIUS"),
        "polygon_enabled": bool(ft & _FENCE_BIT_POLYGON),
        "raw_type": ft,
    }


@app.post("/fence/config")
def fence_config_set(req: FenceConfigRequest):
    """Write geofence config (excluding polygon point data)."""
    if req.action not in _FENCE_ACTIONS:
        raise HTTPException(status_code=422,
                            detail=f"Invalid action. Options: {list(_FENCE_ACTIONS.keys())}")
    if req.margin_m < 0:
        raise HTTPException(status_code=422, detail="margin_m must be >= 0")
    if req.circle_enabled and req.circle_radius_m < 30:
        raise HTTPException(status_code=422, detail="circle radius must be >= 30m")
    ft = 0
    if req.ceiling_enabled: ft |= _FENCE_BIT_CEILING
    if req.floor_enabled:   ft |= _FENCE_BIT_FLOOR
    if req.circle_enabled:  ft |= _FENCE_BIT_CIRCLE
    if req.polygon_enabled: ft |= _FENCE_BIT_POLYGON
    ok, msg = _write_params({
        "FENCE_ENABLE":     1 if req.enable else 0,
        "FENCE_AUTOENABLE": 3 if req.autoenable else 0,
        "FENCE_ACTION":     _FENCE_ACTIONS[req.action],
        "FENCE_TYPE":       ft,
        "FENCE_MARGIN":     req.margin_m,
        "FENCE_ALT_MAX":    req.ceiling_m,
        "FENCE_ALT_MIN":    req.floor_m,
        "FENCE_RADIUS":     req.circle_radius_m,
    })
    return {"success": ok, "message": msg}


@app.post("/fence/defaults")
def fence_defaults():
    """Default fence: 100m ceiling + 300m radius, RTL on breach, auto-arm."""
    ok, msg = _write_params({
        "FENCE_ENABLE":     1,
        "FENCE_AUTOENABLE": 3,
        "FENCE_ACTION":     _FENCE_ACTIONS["rtl"],
        "FENCE_TYPE":       _FENCE_BIT_CEILING | _FENCE_BIT_CIRCLE,
        "FENCE_MARGIN":     2.0,
        "FENCE_ALT_MAX":    100.0,
        "FENCE_ALT_MIN":    -5.0,
        "FENCE_RADIUS":     300.0,
    })
    return {"success": ok, "message": msg}


# Polygon upload goes via MAVROS's separate geofence service (mission_type=FENCE
# on the wire). Each vertex is a Waypoint with command 5001 (inclusion) or 5002
# (exclusion), and param1 = total vertex count on every vertex.
@app.post("/fence/polygon")
def fence_polygon_set(req: FencePolygonRequest):
    """Upload a polygon fence (>=3 vertices)."""
    if not isinstance(req.points, list) or len(req.points) < 3:
        raise HTTPException(status_code=422, detail="Need at least 3 points")
    clean = []
    for i, pt in enumerate(req.points):
        if not (isinstance(pt, list) and len(pt) == 2):
            raise HTTPException(status_code=422, detail=f"Point {i}: expected [lat, lon]")
        try:
            lat, lon = float(pt[0]), float(pt[1])
        except Exception:
            raise HTTPException(status_code=422, detail=f"Point {i}: not numeric")
        if not (-90.0 <= lat <= 90.0) or not (-180.0 <= lon <= 180.0):
            raise HTTPException(status_code=422, detail=f"Point {i}: lat/lon out of range")
        clean.append((lat, lon))

    cmd = 5001 if req.inclusion else 5002   # MAV_CMD_NAV_FENCE_POLYGON_VERTEX_*
    total = len(clean)
    waypoints = []
    for lat, lon in clean:
        wp = Waypoint()
        wp.frame = 0                         # MAV_FRAME_GLOBAL
        wp.command = cmd
        wp.is_current = False
        wp.autocontinue = True
        wp.param1 = float(total)
        wp.param2 = 0.0
        wp.param3 = 0.0
        wp.param4 = 0.0
        wp.x_lat = float(lat)
        wp.y_long = float(lon)
        wp.z_alt = 0.0
        waypoints.append(wp)

    push_req = WaypointPush.Request()
    push_req.start_index = 0
    push_req.waypoints = waypoints
    with _svc_lock:
        result = _call_service(_fence_push_client, push_req, timeout=15.0)
    if not result:
        return {"success": False, "message": "Geofence push service unavailable"}
    if not result.success:
        return {"success": False, "message": "Upload rejected by FC"}
    return {"success": True, "message": f"{total} points uploaded",
            "transferred": int(result.wp_transfered)}


@app.delete("/fence/polygon")
def fence_polygon_clear():
    """Clear all polygon fence points on the FC."""
    with _svc_lock:
        result = _call_service(_fence_clear_client, WaypointClear.Request(), timeout=10.0)
    if not result:
        return {"success": False, "message": "Geofence clear service unavailable"}
    if not result.success:
        return {"success": False, "message": "Clear rejected by FC"}
    return {"success": True, "message": "Polygon cleared"}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8080)
