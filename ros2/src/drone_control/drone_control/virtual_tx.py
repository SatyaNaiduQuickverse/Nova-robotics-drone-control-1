#!/usr/bin/env python3
"""
Virtual RC Transmitter for digital drone control.
Provides timestamped commands, expo curves, smoothing, and failsafe.
"""

import time
import threading
import math
from dataclasses import dataclass, field
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from mavros_msgs.msg import OverrideRCIn


# Channel indices (ArduPilot standard)
CH_ROLL = 0
CH_PITCH = 1
CH_THROTTLE = 2
CH_YAW = 3

# PWM constants
PWM_MIN = 1000
PWM_MAX = 2000
PWM_CENTER = 1500
PWM_RELEASE = 65535  # Release channel to physical RC

# Modes where mid-stick throttle = hold altitude (failsafe should hover, not descend)
ALTITUDE_HOLD_MODES = {"ALT_HOLD", "LOITER", "POSCTL", "AUTO", "GUIDED", "AUTO.MISSION",
                       "AUTO.LOITER", "AUTO.RTL", "AUTO.LAND", "BRAKE", "OFFBOARD"}

# Failsafe throttle: 0.0 for STABILIZE (idle), ~0.44 for altitude-hold modes (mid-stick = hover)
FAILSAFE_THROTTLE_STABILIZE = 0.0
FAILSAFE_THROTTLE_ALT_HOLD = 0.44  # (1500 - 1100) / (2000 - 1100) ≈ mid-stick


@dataclass
class VirtualTXConfig:
    """All tunable parameters for the virtual transmitter."""

    # Timing
    command_timeout: float = 1.5      # Reject commands older than this (seconds)
    failsafe_timeout: float = 0.5     # Start auto-center after no input (seconds)
    publish_rate: float = 50.0        # Hz
    center_rate: float = 2.0          # Rate to return to center (per second)

    # Expo curves (0.0 = linear, 1.0 = full cubic)
    roll_expo: float = 0.3
    pitch_expo: float = 0.3
    yaw_expo: float = 0.2
    throttle_expo: float = 0.0

    # Smoothing alpha (0.0 = max smooth, 1.0 = instant)
    roll_smoothing: float = 0.7
    pitch_smoothing: float = 0.7
    yaw_smoothing: float = 0.5
    throttle_smoothing: float = 0.8

    # Deadzone (normalized 0.0-1.0)
    roll_deadzone: float = 0.05
    pitch_deadzone: float = 0.05
    yaw_deadzone: float = 0.05
    throttle_deadzone: float = 0.02

    # Rate limits (max change per second, normalized)
    roll_rate_limit: float = 3.0
    pitch_rate_limit: float = 3.0
    yaw_rate_limit: float = 2.0
    throttle_rate_limit: float = 1.5

    # Throttle range (PWM values)
    # idle = minimum when throttle input is 0 (keeps motors spinning when armed)
    throttle_pwm_idle: int = 1100
    throttle_pwm_max: int = 2000


@dataclass
class ControlInput:
    """Timestamped control input from API."""
    timestamp: float
    throttle: float  # 0.0 to 1.0
    roll: float      # -1.0 to 1.0
    pitch: float     # -1.0 to 1.0
    yaw: float       # -1.0 to 1.0
    duration: float = 1.0  # How long to hold this command (seconds)


@dataclass
class ChannelState:
    """State for a single channel with smoothing."""
    raw: float = 0.0
    smoothed: float = 0.0
    output: float = 0.0


@dataclass
class TXState:
    """Current state of the virtual transmitter."""
    enabled: bool = False
    last_command_time: float = 0.0
    hold_until: float = 0.0          # Command holds until this time
    last_accepted_ts: float = 0.0    # Timestamp of last accepted command (for ordering)
    in_failsafe: bool = False
    centering: bool = False

    roll: ChannelState = field(default_factory=ChannelState)
    pitch: ChannelState = field(default_factory=ChannelState)
    yaw: ChannelState = field(default_factory=ChannelState)
    throttle: ChannelState = field(default_factory=lambda: ChannelState(raw=0.0, smoothed=0.0, output=0.0))


def apply_deadzone(value: float, deadzone: float) -> float:
    """Apply deadzone around center (0.0)."""
    if abs(value) < deadzone:
        return 0.0
    # Rescale remaining range to 0-1
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def apply_expo(value: float, expo: float) -> float:
    """
    Apply expo curve. Standard RC formula:
    output = (1 - expo) * input + expo * input^3
    """
    return (1.0 - expo) * value + expo * (value ** 3)


def apply_smoothing(new_value: float, old_value: float, alpha: float) -> float:
    """Exponential moving average smoothing."""
    return alpha * new_value + (1.0 - alpha) * old_value


def apply_rate_limit(new_value: float, old_value: float, rate_limit: float, dt: float) -> float:
    """Limit rate of change per time step."""
    max_change = rate_limit * dt
    diff = new_value - old_value
    if abs(diff) > max_change:
        return old_value + math.copysign(max_change, diff)
    return new_value


def to_pwm_bidirectional(value: float) -> int:
    """Convert -1.0 to 1.0 → 1000 to 2000 PWM."""
    value = max(-1.0, min(1.0, value))
    return int(PWM_CENTER + value * 500)


def to_pwm_throttle(value: float, pwm_idle: int = 1100, pwm_max: int = 2000) -> int:
    """
    Convert 0.0 to 1.0 → pwm_idle to pwm_max PWM.
    0.0 = idle spin (motors spinning slowly when armed)
    1.0 = full throttle
    """
    value = max(0.0, min(1.0, value))
    return int(pwm_idle + value * (pwm_max - pwm_idle))


class VirtualTXNode(Node):
    """ROS2 node that publishes RC override commands."""

    def __init__(self, config: VirtualTXConfig, state: TXState):
        super().__init__("virtual_tx_node")
        self.config = config
        self.state = state
        self._lock = threading.Lock()
        self._last_update_time = time.time()
        self._stop_flag = False

        # MAVROS rc/override requires RELIABLE QoS
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        self.rc_pub = self.create_publisher(OverrideRCIn, "/mavros/rc/override", qos)

        self.get_logger().info(f"Virtual TX node created, publish rate: {config.publish_rate} Hz")

    def start_publish_loop(self):
        """Start the publish loop in a background thread."""
        self._stop_flag = False
        self._publish_thread = threading.Thread(target=self._run_publish_loop, daemon=True)
        self._publish_thread.start()

    def stop_publish_loop(self):
        """Stop the publish loop."""
        self._stop_flag = True

    def _run_publish_loop(self):
        """Background thread that publishes at configured rate."""
        period = 1.0 / self.config.publish_rate
        while not self._stop_flag:
            self._publish_loop()
            time.sleep(period)

    def _publish_loop(self):
        """Main control loop - processes inputs and publishes RC override."""
        now = time.time()
        dt = now - self._last_update_time
        self._last_update_time = now

        with self._lock:
            if not self.state.enabled:
                return

            # Check for failsafe: command hold expired
            if self.state.hold_until > 0 and now > self.state.hold_until:
                if not self.state.in_failsafe:
                    self.state.in_failsafe = True
                    self.state.centering = True
                    age = now - self.state.last_command_time
                    self.get_logger().warn(f"Failsafe: hold expired ({age:.1f}s since last cmd), auto-centering")

            if self.state.centering:
                self._auto_center(dt)
            else:
                self._process_channels(dt)

            # Publish RC override - always override while enabled
            msg = OverrideRCIn()
            channels = [PWM_RELEASE] * 18
            channels[CH_ROLL] = to_pwm_bidirectional(self.state.roll.output)
            channels[CH_PITCH] = to_pwm_bidirectional(self.state.pitch.output)
            channels[CH_YAW] = to_pwm_bidirectional(self.state.yaw.output)
            channels[CH_THROTTLE] = to_pwm_throttle(
                self.state.throttle.output,
                self.config.throttle_pwm_idle,
                self.config.throttle_pwm_max
            )
            msg.channels = channels
            self.rc_pub.publish(msg)

    def _process_channels(self, dt: float):
        """Process all channels through deadzone, expo, smoothing, rate limit."""
        cfg = self.config
        s = self.state

        # Roll
        val = apply_deadzone(s.roll.raw, cfg.roll_deadzone)
        val = apply_expo(val, cfg.roll_expo)
        val = apply_smoothing(val, s.roll.smoothed, cfg.roll_smoothing)
        val = apply_rate_limit(val, s.roll.output, cfg.roll_rate_limit, dt)
        s.roll.smoothed = val
        s.roll.output = val

        # Pitch
        val = apply_deadzone(s.pitch.raw, cfg.pitch_deadzone)
        val = apply_expo(val, cfg.pitch_expo)
        val = apply_smoothing(val, s.pitch.smoothed, cfg.pitch_smoothing)
        val = apply_rate_limit(val, s.pitch.output, cfg.pitch_rate_limit, dt)
        s.pitch.smoothed = val
        s.pitch.output = val

        # Yaw
        val = apply_deadzone(s.yaw.raw, cfg.yaw_deadzone)
        val = apply_expo(val, cfg.yaw_expo)
        val = apply_smoothing(val, s.yaw.smoothed, cfg.yaw_smoothing)
        val = apply_rate_limit(val, s.yaw.output, cfg.yaw_rate_limit, dt)
        s.yaw.smoothed = val
        s.yaw.output = val

        # Throttle (0-1 range, no center)
        val = apply_deadzone(s.throttle.raw, cfg.throttle_deadzone)
        val = apply_expo(val, cfg.throttle_expo)
        val = apply_smoothing(val, s.throttle.smoothed, cfg.throttle_smoothing)
        val = apply_rate_limit(val, s.throttle.output, cfg.throttle_rate_limit, dt)
        s.throttle.smoothed = val
        s.throttle.output = val

    def _auto_center(self, dt: float):
        """Gradually return all channels to center/safe values."""
        rate = self.config.center_rate * dt
        s = self.state

        # Move roll/pitch/yaw toward 0
        s.roll.output = self._move_toward(s.roll.output, 0.0, rate)
        s.pitch.output = self._move_toward(s.pitch.output, 0.0, rate)
        s.yaw.output = self._move_toward(s.yaw.output, 0.0, rate)

        # Mode-aware failsafe throttle:
        # - ALT_HOLD/LOITER/etc: mid-stick (hover in place, safest on internet loss)
        # - STABILIZE: minimum (idle, safe on ground)
        import telemetry
        mode = telemetry.get()["state"]["mode"]
        throttle_target = FAILSAFE_THROTTLE_ALT_HOLD if mode in ALTITUDE_HOLD_MODES else FAILSAFE_THROTTLE_STABILIZE
        s.throttle.output = self._move_toward(s.throttle.output, throttle_target, rate)

    def _move_toward(self, current: float, target: float, max_step: float) -> float:
        """Move current value toward target by at most max_step."""
        diff = target - current
        if abs(diff) <= max_step:
            return target
        return current + math.copysign(max_step, diff)

    def set_input(self, inp: ControlInput) -> tuple[bool, str]:
        """Set new control input. Returns (success, message)."""
        now = time.time()
        age = now - inp.timestamp

        if age > self.config.command_timeout:
            return False, f"Command too old: {age:.2f}s > {self.config.command_timeout}s"

        if age < -1.0:
            return False, f"Command from future: {age:.2f}s"

        if inp.duration <= 0.0:
            return False, f"Invalid duration: {inp.duration}s (must be > 0)"

        with self._lock:
            if not self.state.enabled:
                return False, "Virtual TX not enabled"

            # Reject out-of-order commands (internet can reorder packets)
            # Cap at now to prevent near-future timestamps from poisoning ordering
            effective_ts = min(inp.timestamp, now)
            if effective_ts < self.state.last_accepted_ts:
                return False, "Out of order: newer command already accepted"

            new_roll = max(-1.0, min(1.0, inp.roll))
            new_pitch = max(-1.0, min(1.0, inp.pitch))
            new_yaw = max(-1.0, min(1.0, inp.yaw))
            new_throttle = max(0.0, min(1.0, inp.throttle))

            # Dedup: if values match current target, just extend hold
            dz = 0.02
            is_same = (
                abs(new_roll - self.state.roll.raw) < dz and
                abs(new_pitch - self.state.pitch.raw) < dz and
                abs(new_yaw - self.state.yaw.raw) < dz and
                abs(new_throttle - self.state.throttle.raw) < dz
            )

            if not is_same:
                self.state.roll.raw = new_roll
                self.state.pitch.raw = new_pitch
                self.state.yaw.raw = new_yaw
                self.state.throttle.raw = new_throttle

            self.state.last_command_time = now
            self.state.last_accepted_ts = effective_ts
            self.state.hold_until = now + inp.duration
            self.state.in_failsafe = False
            self.state.centering = False

        return True, "Hold" if is_same else "OK"

    def enable(self) -> tuple[bool, str]:
        """Enable the virtual transmitter."""
        with self._lock:
            if self.state.enabled:
                return True, "Already enabled"

            # Reset state
            self.state.roll = ChannelState()
            self.state.pitch = ChannelState()
            self.state.yaw = ChannelState()
            self.state.throttle = ChannelState()
            self.state.last_command_time = time.time()
            self.state.hold_until = 0.0
            self.state.last_accepted_ts = 0.0
            self.state.in_failsafe = False
            self.state.centering = False
            self.state.enabled = True

            self.get_logger().info("Virtual TX enabled")
            return True, "Enabled"

    def disable(self) -> tuple[bool, str]:
        """Disable and release all channels."""
        with self._lock:
            if not self.state.enabled:
                return True, "Already disabled"

            self.state.enabled = False

            # Publish release on all channels
            msg = OverrideRCIn()
            msg.channels = [PWM_RELEASE] * 18
            self.rc_pub.publish(msg)

            self.get_logger().info("Virtual TX disabled, channels released")
            return True, "Disabled"

    def get_status(self) -> dict:
        """Get current status."""
        with self._lock:
            now = time.time()
            hold_remaining = max(0.0, self.state.hold_until - now) if self.state.hold_until > 0 else 0.0
            return {
                "enabled": self.state.enabled,
                "in_failsafe": self.state.in_failsafe,
                "centering": self.state.centering,
                "hold_remaining": round(hold_remaining, 2),
                "last_command_age": now - self.state.last_command_time if self.state.last_command_time > 0 else -1,
                "outputs": {
                    "roll": self.state.roll.output,
                    "pitch": self.state.pitch.output,
                    "yaw": self.state.yaw.output,
                    "throttle": self.state.throttle.output,
                },
                "pwm": {
                    "roll": to_pwm_bidirectional(self.state.roll.output),
                    "pitch": to_pwm_bidirectional(self.state.pitch.output),
                    "yaw": to_pwm_bidirectional(self.state.yaw.output),
                    "throttle": to_pwm_throttle(
                        self.state.throttle.output,
                        self.config.throttle_pwm_idle,
                        self.config.throttle_pwm_max
                    ),
                }
            }


# Module-level singleton
_config = VirtualTXConfig()
_state = TXState()
_node: Optional[VirtualTXNode] = None
_running = False


def start():
    """Start the virtual TX node."""
    global _node, _running
    if _running:
        return

    # Don't init rclpy - telemetry module handles that
    # Just create the node and start publish thread
    _node = VirtualTXNode(_config, _state)
    _node.start_publish_loop()
    _running = True


def stop():
    """Stop the virtual TX node."""
    global _node, _running
    if _node:
        _node.disable()
        _node.stop_publish_loop()
        _node.destroy_node()
        _node = None
    _running = False


def enable() -> tuple[bool, str]:
    """Enable virtual TX control."""
    if not _node:
        return False, "Virtual TX not started"
    return _node.enable()


def disable() -> tuple[bool, str]:
    """Disable virtual TX control."""
    if not _node:
        return False, "Virtual TX not started"
    return _node.disable()


def send_command(timestamp: float, throttle: float, roll: float, pitch: float, yaw: float, duration: float = 1.0) -> tuple[bool, str]:
    """Send a control command."""
    if not _node:
        return False, "Virtual TX not started"
    inp = ControlInput(timestamp=timestamp, throttle=throttle, roll=roll, pitch=pitch, yaw=yaw, duration=duration)
    return _node.set_input(inp)


def get_status() -> dict:
    """Get current status."""
    if not _node:
        return {"enabled": False, "error": "Virtual TX not started"}
    return _node.get_status()


def get_config() -> dict:
    """Get current configuration."""
    return {
        "command_timeout": _config.command_timeout,
        "failsafe_timeout": _config.failsafe_timeout,
        "publish_rate": _config.publish_rate,
        "center_rate": _config.center_rate,
        "roll_expo": _config.roll_expo,
        "pitch_expo": _config.pitch_expo,
        "yaw_expo": _config.yaw_expo,
        "throttle_expo": _config.throttle_expo,
        "roll_smoothing": _config.roll_smoothing,
        "pitch_smoothing": _config.pitch_smoothing,
        "yaw_smoothing": _config.yaw_smoothing,
        "throttle_smoothing": _config.throttle_smoothing,
        "roll_deadzone": _config.roll_deadzone,
        "pitch_deadzone": _config.pitch_deadzone,
        "yaw_deadzone": _config.yaw_deadzone,
        "throttle_deadzone": _config.throttle_deadzone,
        "roll_rate_limit": _config.roll_rate_limit,
        "pitch_rate_limit": _config.pitch_rate_limit,
        "yaw_rate_limit": _config.yaw_rate_limit,
        "throttle_rate_limit": _config.throttle_rate_limit,
        "throttle_pwm_idle": _config.throttle_pwm_idle,
        "throttle_pwm_max": _config.throttle_pwm_max,
    }


def update_config(updates: dict) -> tuple[bool, str]:
    """Update configuration parameters."""
    global _config
    valid_keys = set(get_config().keys())
    for key, value in updates.items():
        if key not in valid_keys:
            return False, f"Invalid config key: {key}"
        if not isinstance(value, (int, float)):
            return False, f"Invalid value type for {key}"
        setattr(_config, key, float(value))

    # Update node config if running
    if _node:
        _node.config = _config

    return True, "Config updated"
