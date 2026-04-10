#!/usr/bin/env python3
"""
Minimal MAVROS drone control node.
Safety-critical: every function has explicit safety checks.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped


class DroneControl(Node):
    """Minimal drone control with MAVROS."""

    def __init__(self):
        super().__init__('drone_control')

        # State
        self._armed = False
        self._connected = False
        self._mode = ''

        # QoS for MAVROS
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribers
        self._state_sub = self.create_subscription(
            State, '/mavros/state', self._state_cb, qos)

        # Publishers
        self._setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)

        # Service clients
        self._arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self._mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.get_logger().info('Drone control node initialized')

    def _state_cb(self, msg: State):
        """Update internal state from FCU."""
        self._armed = msg.armed
        self._connected = msg.connected
        self._mode = msg.mode

    @property
    def is_ready(self) -> bool:
        """Check if FCU connection is established."""
        return self._connected

    @property
    def is_armed(self) -> bool:
        """Check armed status."""
        return self._armed

    def arm(self) -> bool:
        """Arm the vehicle. Returns success status."""
        if not self._connected:
            self.get_logger().error('Cannot arm: not connected to FCU')
            return False

        if self._armed:
            self.get_logger().warn('Already armed')
            return True

        req = CommandBool.Request()
        req.value = True
        future = self._arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() and future.result().success:
            self.get_logger().info('Armed successfully')
            return True

        self.get_logger().error('Arming failed')
        return False

    def disarm(self) -> bool:
        """Disarm the vehicle. Returns success status."""
        if not self._armed:
            return True

        req = CommandBool.Request()
        req.value = False
        future = self._arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() and future.result().success:
            self.get_logger().info('Disarmed successfully')
            return True

        self.get_logger().error('Disarm failed')
        return False

    def set_mode(self, mode: str) -> bool:
        """Set flight mode. Returns success status."""
        if not self._connected:
            self.get_logger().error('Cannot set mode: not connected')
            return False

        req = SetMode.Request()
        req.custom_mode = mode
        future = self._mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() and future.result().mode_sent:
            self.get_logger().info(f'Mode set to {mode}')
            return True

        self.get_logger().error(f'Failed to set mode {mode}')
        return False

    def send_position(self, x: float, y: float, z: float):
        """Send position setpoint in local frame."""
        if not self._connected:
            self.get_logger().error('Cannot send position: not connected')
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0

        self._setpoint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DroneControl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.disarm()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
