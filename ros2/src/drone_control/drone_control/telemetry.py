#!/usr/bin/env python3
"""
Telemetry receiver for Pixhawk via MAVROS.
Subscribes to all relevant topics, stores latest values.
"""

import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from mavros_msgs.msg import (
    State, ExtendedState, StatusText, RCIn, RCOut
)
from sensor_msgs.msg import (
    NavSatFix, Imu, MagneticField, FluidPressure, Temperature, BatteryState
)
from geometry_msgs.msg import (
    PoseStamped, TwistStamped, Vector3Stamped
)
from std_msgs.msg import Float64, UInt32


class Telemetry:
    """Stores all telemetry data from Pixhawk."""

    def __init__(self):
        self.state = {
            "connected": False,
            "armed": False,
            "mode": "",
            "system_status": 0
        }
        self.extended_state = {
            "landed_state": 0,
            "vtol_state": 0
        }
        self.battery = {
            "voltage": 0.0,
            "current": 0.0,
            "percentage": 0.0
        }
        self.gps = {
            "fix_type": 0,
            "latitude": 0.0,
            "longitude": 0.0,
            "altitude": 0.0,
            "satellites": 0,
            "hdop": 0.0
        }
        self.local_position = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        }
        self.orientation = {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0
        }
        self.velocity = {
            "vx": 0.0,
            "vy": 0.0,
            "vz": 0.0
        }
        self.imu = {
            "ax": 0.0,
            "ay": 0.0,
            "az": 0.0,
            "gx": 0.0,
            "gy": 0.0,
            "gz": 0.0
        }
        self.mag = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        }
        self.baro = {
            "pressure": 0.0,
            "temperature": 0.0
        }
        self.vfr_hud = {
            "airspeed": 0.0,
            "groundspeed": 0.0,
            "heading": 0,
            "throttle": 0.0,
            "altitude": 0.0,
            "climb_rate": 0.0
        }
        self.rc_in = {
            "channels": []
        }
        self.rc_out = {
            "channels": []
        }
        self.home = {
            "latitude": 0.0,
            "longitude": 0.0,
            "altitude": 0.0
        }
        self.relative_alt = 0.0
        self.heading = 0.0
        self.status_text = ""

    def to_dict(self) -> dict:
        return {
            "state": self.state,
            "extended_state": self.extended_state,
            "battery": self.battery,
            "gps": self.gps,
            "local_position": self.local_position,
            "orientation": self.orientation,
            "velocity": self.velocity,
            "imu": self.imu,
            "mag": self.mag,
            "baro": self.baro,
            "vfr_hud": self.vfr_hud,
            "rc_in": self.rc_in,
            "rc_out": self.rc_out,
            "home": self.home,
            "relative_alt": self.relative_alt,
            "heading": self.heading,
            "status_text": self.status_text
        }


class TelemetryNode(Node):
    """ROS2 node that subscribes to MAVROS topics."""

    def __init__(self, telemetry: Telemetry):
        super().__init__("telemetry_node")
        self.telem = telemetry

        # Sensor data QoS - compatible with most MAVROS topics
        qos_sensor = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        # State QoS - MAVROS state uses TRANSIENT_LOCAL
        qos_state = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        qos = qos_sensor

        # State (uses TRANSIENT_LOCAL)
        self.create_subscription(State, "/mavros/state", self._state_cb, qos_state)
        self.create_subscription(ExtendedState, "/mavros/extended_state", self._ext_state_cb, qos_state)
        self.create_subscription(BatteryState, "/mavros/battery", self._battery_cb, qos)
        self.create_subscription(StatusText, "/mavros/statustext/recv", self._status_cb, qos)

        # Position
        self.create_subscription(NavSatFix, "/mavros/global_position/global", self._gps_cb, qos)
        self.create_subscription(UInt32, "/mavros/global_position/raw/satellites", self._satellites_cb, qos)
        self.create_subscription(NavSatFix, "/mavros/global_position/raw/fix", self._gps_raw_cb, qos)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose", self._local_pos_cb, qos)
        self.create_subscription(TwistStamped, "/mavros/local_position/velocity_local", self._vel_cb, qos)
        self.create_subscription(Float64, "/mavros/global_position/rel_alt", self._rel_alt_cb, qos)
        self.create_subscription(Float64, "/mavros/global_position/compass_hdg", self._hdg_cb, qos)
        self.create_subscription(NavSatFix, "/mavros/home_position/home", self._home_cb, qos)

        # IMU
        self.create_subscription(Imu, "/mavros/imu/data", self._imu_cb, qos)
        self.create_subscription(MagneticField, "/mavros/imu/mag", self._mag_cb, qos)
        self.create_subscription(FluidPressure, "/mavros/imu/static_pressure", self._pressure_cb, qos)
        self.create_subscription(Temperature, "/mavros/imu/temperature_baro", self._temp_cb, qos)

        # RC
        self.create_subscription(RCIn, "/mavros/rc/in", self._rc_in_cb, qos)
        self.create_subscription(RCOut, "/mavros/rc/out", self._rc_out_cb, qos)

        self.get_logger().info("Telemetry node started")

    def _state_cb(self, msg: State):
        self.telem.state["connected"] = msg.connected
        self.telem.state["armed"] = msg.armed
        self.telem.state["mode"] = msg.mode
        self.telem.state["system_status"] = msg.system_status

    def _ext_state_cb(self, msg: ExtendedState):
        self.telem.extended_state["landed_state"] = msg.landed_state
        self.telem.extended_state["vtol_state"] = msg.vtol_state

    def _battery_cb(self, msg: BatteryState):
        self.telem.battery["voltage"] = msg.voltage
        self.telem.battery["current"] = msg.current
        self.telem.battery["percentage"] = msg.percentage

    def _status_cb(self, msg: StatusText):
        self.telem.status_text = msg.text

    def _gps_cb(self, msg: NavSatFix):
        self.telem.gps["fix_type"] = msg.status.status
        self.telem.gps["latitude"] = msg.latitude
        self.telem.gps["longitude"] = msg.longitude
        self.telem.gps["altitude"] = msg.altitude

    def _satellites_cb(self, msg: UInt32):
        self.telem.gps["satellites"] = msg.data

    def _gps_raw_cb(self, msg: NavSatFix):
        # Extract HDOP estimate from position covariance (diagonal elements)
        if msg.position_covariance_type > 0:
            import math
            hdop = math.sqrt(msg.position_covariance[0]) / 5.0  # Approximate HDOP
            self.telem.gps["hdop"] = round(hdop, 2)

    def _local_pos_cb(self, msg: PoseStamped):
        self.telem.local_position["x"] = msg.pose.position.x
        self.telem.local_position["y"] = msg.pose.position.y
        self.telem.local_position["z"] = msg.pose.position.z
        q = msg.pose.orientation
        self.telem.orientation["roll"] = _quat_to_euler_roll(q.x, q.y, q.z, q.w)
        self.telem.orientation["pitch"] = _quat_to_euler_pitch(q.x, q.y, q.z, q.w)
        self.telem.orientation["yaw"] = _quat_to_euler_yaw(q.x, q.y, q.z, q.w)

    def _vel_cb(self, msg: TwistStamped):
        self.telem.velocity["vx"] = msg.twist.linear.x
        self.telem.velocity["vy"] = msg.twist.linear.y
        self.telem.velocity["vz"] = msg.twist.linear.z

    def _rel_alt_cb(self, msg: Float64):
        self.telem.relative_alt = msg.data

    def _hdg_cb(self, msg: Float64):
        self.telem.heading = msg.data

    def _home_cb(self, msg: NavSatFix):
        self.telem.home["latitude"] = msg.latitude
        self.telem.home["longitude"] = msg.longitude
        self.telem.home["altitude"] = msg.altitude

    def _imu_cb(self, msg: Imu):
        self.telem.imu["ax"] = msg.linear_acceleration.x
        self.telem.imu["ay"] = msg.linear_acceleration.y
        self.telem.imu["az"] = msg.linear_acceleration.z
        self.telem.imu["gx"] = msg.angular_velocity.x
        self.telem.imu["gy"] = msg.angular_velocity.y
        self.telem.imu["gz"] = msg.angular_velocity.z

    def _mag_cb(self, msg: MagneticField):
        self.telem.mag["x"] = msg.magnetic_field.x
        self.telem.mag["y"] = msg.magnetic_field.y
        self.telem.mag["z"] = msg.magnetic_field.z

    def _pressure_cb(self, msg: FluidPressure):
        self.telem.baro["pressure"] = msg.fluid_pressure

    def _temp_cb(self, msg: Temperature):
        self.telem.baro["temperature"] = msg.temperature

    def _rc_in_cb(self, msg: RCIn):
        self.telem.rc_in["channels"] = list(msg.channels)

    def _rc_out_cb(self, msg: RCOut):
        self.telem.rc_out["channels"] = list(msg.channels)


def _quat_to_euler_roll(x, y, z, w) -> float:
    import math
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    return math.atan2(sinr, cosr)


def _quat_to_euler_pitch(x, y, z, w) -> float:
    import math
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        return math.copysign(math.pi / 2, sinp)
    return math.asin(sinp)


def _quat_to_euler_yaw(x, y, z, w) -> float:
    import math
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


# Singleton instances
_telemetry = Telemetry()
_node: TelemetryNode = None
_running = False


def start():
    """Initialize and start telemetry collection."""
    global _node, _running
    if _running:
        return

    rclpy.init()
    _node = TelemetryNode(_telemetry)
    _running = True

    def spin():
        rclpy.spin(_node)

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()


def stop():
    """Stop telemetry collection."""
    global _node, _running
    if _node:
        _node.destroy_node()
    rclpy.shutdown()
    _running = False


def get() -> dict:
    """Get current telemetry data."""
    return _telemetry.to_dict()


def get_raw() -> Telemetry:
    """Get raw telemetry object."""
    return _telemetry


def get_node() -> TelemetryNode:
    """Get ROS node for service calls."""
    return _node
