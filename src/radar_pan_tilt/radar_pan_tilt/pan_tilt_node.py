#!/usr/bin/env python3
"""
Pan-tilt servo node for RADAR.

Subscribes to /radar/pan_tilt/cmd (Float32MultiArray: [pan_deg, tilt_deg])
and drives two GPIO-connected servos via gpiozero.

This node is intended for software-commanded camera positioning (e.g. from
the operator GUI).  For direct joystick control use pan_tilt_joy_node instead.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

try:
    from gpiozero import AngularServo
    GPIO_AVAILABLE = True
except Exception:
    GPIO_AVAILABLE = False


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class PanTiltNode(Node):
    def __init__(self):
        super().__init__('pan_tilt_node')

        self.declare_parameter('pan_gpio',        13)
        self.declare_parameter('tilt_gpio',       12)
        self.declare_parameter('pan_min_angle',  -80.0)
        self.declare_parameter('pan_max_angle',   80.0)
        self.declare_parameter('tilt_min_angle', -45.0)
        self.declare_parameter('tilt_max_angle',  45.0)
        self.declare_parameter('min_pulse_width', 0.0005)
        self.declare_parameter('max_pulse_width', 0.0025)

        pan_gpio  = self.get_parameter('pan_gpio').value
        tilt_gpio = self.get_parameter('tilt_gpio').value
        self.pan_min  = self.get_parameter('pan_min_angle').value
        self.pan_max  = self.get_parameter('pan_max_angle').value
        self.tilt_min = self.get_parameter('tilt_min_angle').value
        self.tilt_max = self.get_parameter('tilt_max_angle').value
        min_pw = self.get_parameter('min_pulse_width').value
        max_pw = self.get_parameter('max_pulse_width').value

        if GPIO_AVAILABLE:
            self.pan_servo = AngularServo(
                pan_gpio,
                min_angle=self.pan_min, max_angle=self.pan_max,
                min_pulse_width=min_pw, max_pulse_width=max_pw,
            )
            self.tilt_servo = AngularServo(
                tilt_gpio,
                min_angle=self.tilt_min, max_angle=self.tilt_max,
                min_pulse_width=min_pw, max_pulse_width=max_pw,
            )
            self.get_logger().info(
                f'PanTiltNode: GPIO pan={pan_gpio}, tilt={tilt_gpio}'
            )
        else:
            self.pan_servo  = None
            self.tilt_servo = None
            self.get_logger().warn(
                'gpiozero not available — running in log-only mode'
            )

        self.sub = self.create_subscription(
            Float32MultiArray,
            '/radar/pan_tilt/cmd',
            self.cmd_callback,
            10,
        )

    def cmd_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            self.get_logger().warn('pan_tilt/cmd needs [pan, tilt] — got less')
            return

        pan  = clamp(msg.data[0], self.pan_min,  self.pan_max)
        tilt = clamp(msg.data[1], self.tilt_min, self.tilt_max)

        self.get_logger().debug(f'Pan: {pan:.1f} deg, Tilt: {tilt:.1f} deg')

        if self.pan_servo is not None:
            self.pan_servo.angle  = pan
            self.tilt_servo.angle = tilt

    def destroy_node(self):
        if self.pan_servo is not None:
            try:
                self.pan_servo.detach()
                self.tilt_servo.detach()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PanTiltNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
