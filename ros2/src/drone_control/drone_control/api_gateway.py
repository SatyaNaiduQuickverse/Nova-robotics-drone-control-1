#!/usr/bin/env python3
"""
Minimal API gateway for MAVROS drone control.
Direct MAVROS service calls, no abstractions.
Thread-safe: single lock serializes all service calls,
futures resolved by telemetry's background rclpy.spin() thread.
"""

import time
import threading
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

from mavros_msgs.srv import CommandBool, SetMode, CommandLong, WaypointClear, WaypointPush

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import telemetry
import mission
import virtual_tx

# Mode mapping: API name -> (ArduPilot, PX4)
MODES = {
    "stabilize":  ("STABILIZE", "STABILIZED"),
    "loiter":     ("LOITER", "POSCTL"),
    "alt_hold":   ("ALT_HOLD", "ALTCTL"),
    "land":       ("LAND", "AUTO.LAND"),
    "rtl":        ("RTL", "AUTO.RTL"),
    "guided":     ("GUIDED", "OFFBOARD"),
    "auto":       ("AUTO", "AUTO.MISSION"),
    "brake":      ("BRAKE", "AUTO.LOITER"),
}

TELEM_CATEGORIES = [
    "state", "extended_state", "battery", "gps", "local_position",
    "orientation", "velocity", "imu", "mag", "baro", "vfr_hud",
    "rc_in", "rc_out", "home"
]

# Service clients - created once at startup, resolved by telemetry spin thread
_arming_client = None
_set_mode_client = None
_command_client = None
_wp_clear_client = None
_wp_push_client = None
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


def _call_service(client, request, timeout: float = 5.0):
    """
    Call a ROS2 service and wait for the result.
    Uses future polling — the background rclpy.spin() thread resolves the future.
    Returns the service response, or None on timeout/unavailable.
    """
    if not client.service_is_ready():
        if not client.wait_for_service(timeout_sec=2.0):
            return None

    future = client.call_async(request)

    end = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() > end:
            future.cancel()
            return None
        time.sleep(0.01)

    return future.result()


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


@app.on_event("startup")
def startup():
    global _arming_client, _set_mode_client, _command_client, _wp_clear_client, _wp_push_client

    telemetry.start()
    virtual_tx.start()

    # Create all service clients once on the telemetry node
    node = telemetry.get_node()
    _arming_client = node.create_client(CommandBool, "/mavros/cmd/arming")
    _set_mode_client = node.create_client(SetMode, "/mavros/set_mode")
    _command_client = node.create_client(CommandLong, "/mavros/cmd/command")
    _wp_clear_client = node.create_client(WaypointClear, "/mavros/mission/clear")
    _wp_push_client = node.create_client(WaypointPush, "/mavros/mission/push")


@app.on_event("shutdown")
def shutdown():
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
    """Force disarm - use with caution, bypasses safety checks."""
    telem = telemetry.get()
    if not telem["state"]["connected"]:
        return ArmResponse(success=False, message="Not connected to FCU")
    if not telem["state"]["armed"]:
        return ArmResponse(success=True, message="Already disarmed")

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


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8080)
