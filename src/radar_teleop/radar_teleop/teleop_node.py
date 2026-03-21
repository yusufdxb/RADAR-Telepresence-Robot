#!/usr/bin/env python3
"""
Joystick teleoperation node for RADAR.

Subscribes to /joy and publishes velocity commands to /cmd_vel.
Axes are configurable via ROS 2 parameters to support different
controller layouts (PS4, Xbox, Logitech F710, etc.).

Default layout (Logitech F710 in D mode):
  linear  — left stick Y  (axis 1, positive = forward)
  angular — right stick X (axis 3, positive = turn left)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


def apply_deadzone(value: float, deadzone: float) -> float:
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')

        self.declare_parameter('linear_axis',   1)
        self.declare_parameter('angular_axis',  3)
        self.declare_parameter('linear_scale',  0.5)
        self.declare_parameter('angular_scale', 1.0)
        self.declare_parameter('deadzone',      0.10)
        self.declare_parameter('turbo_button',  5)    # RB — hold for 2× speed
        self.declare_parameter('turbo_scale',   2.0)

        self.linear_axis   = self.get_parameter('linear_axis').value
        self.angular_axis  = self.get_parameter('angular_axis').value
        self.linear_scale  = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.deadzone      = self.get_parameter('deadzone').value
        self.turbo_btn     = self.get_parameter('turbo_button').value
        self.turbo_scale   = self.get_parameter('turbo_scale').value

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.get_logger().info(
            f'TeleopNode ready — linear axis {self.linear_axis}, '
            f'angular axis {self.angular_axis}'
        )

    def joy_callback(self, msg: Joy):
        twist = Twist()

        turbo = (
            len(msg.buttons) > self.turbo_btn and
            msg.buttons[self.turbo_btn] == 1
        )
        speed_mult = self.turbo_scale if turbo else 1.0

        if len(msg.axes) > self.linear_axis:
            raw = apply_deadzone(msg.axes[self.linear_axis], self.deadzone)
            twist.linear.x = raw * self.linear_scale * speed_mult

        if len(msg.axes) > self.angular_axis:
            raw = apply_deadzone(msg.axes[self.angular_axis], self.deadzone)
            twist.angular.z = raw * self.angular_scale * speed_mult

        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
