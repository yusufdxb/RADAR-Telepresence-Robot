#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from gpiozero import AngularServo
import time

# ------------- Helpers -------------
def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def axis_to_range(axis: float, a_min: float, a_max: float) -> float:
    """
    Map joystick axis in [-1, 1] to angle in [a_min, a_max].
    """
    t = (axis + 1.0) / 2.0
    return a_min + t * (a_max - a_min)


# ------------- Pan/Tilt Node -------------
class PanTiltJoyNode(Node):
    def __init__(self):
        super().__init__('pan_tilt_joy_node')

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter('pan_gpio', 13)
        self.declare_parameter('tilt_gpio', 12)

        self.declare_parameter('pan_min_angle', -80.0)
        self.declare_parameter('pan_max_angle', 0.0)
        self.declare_parameter('tilt_min_angle', 0.0)
        self.declare_parameter('tilt_max_angle', 360.0)

        self.declare_parameter('min_pulse_width', 0.0005)
        self.declare_parameter('max_pulse_width', 0.0025)

        self.declare_parameter('pan_axis_index', 5)
        self.declare_parameter('tilt_axis_index', 4)

        self.declare_parameter('deadzone_pan', 0.15)
        self.declare_parameter('deadzone_tilt', 0.15)

        self.declare_parameter('update_hz', 25.0)
        self.declare_parameter('pan_alpha', 0.25)
        self.declare_parameter('tilt_alpha', 0.20)
        self.declare_parameter('axis_alpha', 0.30)

        # -------------------------
        # Load parameters
        # -------------------------
        self.pan_gpio = self.get_parameter('pan_gpio').value
        self.tilt_gpio = self.get_parameter('tilt_gpio').value

        self.pan_min = self.get_parameter('pan_min_angle').value
        self.pan_max = self.get_parameter('pan_max_angle').value
        self.tilt_min = self.get_parameter('tilt_min_angle').value
        self.tilt_max = self.get_parameter('tilt_max_angle').value

        min_pw = self.get_parameter('min_pulse_width').value
        max_pw = self.get_parameter('max_pulse_width').value

        self.pan_axis_idx = self.get_parameter('pan_axis_index').value
        self.tilt_axis_idx = self.get_parameter('tilt_axis_index').value

        self.deadzone_pan = self.get_parameter('deadzone_pan').value
        self.deadzone_tilt = self.get_parameter('deadzone_tilt').value

        update_hz = self.get_parameter('update_hz').value
        self.pan_alpha = self.get_parameter('pan_alpha').value
        self.tilt_alpha = self.get_parameter('tilt_alpha').value
        self.axis_alpha = self.get_parameter('axis_alpha').value

        self.dt = 1.0 / update_hz

        # -------------------------
        # Servo setup
        # -------------------------
        self.pan_servo = AngularServo(
            self.pan_gpio,
            min_angle=self.pan_min,
            max_angle=self.pan_max,
            min_pulse_width=min_pw,
            max_pulse_width=max_pw,
        )

        self.tilt_servo = AngularServo(
            self.tilt_gpio,
            min_angle=self.tilt_min,
            max_angle=self.tilt_max,
            min_pulse_width=min_pw,
            max_pulse_width=max_pw,
        )

        # Initial positions
        self.pan_angle = self.pan_min
        self.tilt_angle = (self.tilt_min + self.tilt_max) / 2.0
        self.pan_servo.angle = self.pan_angle
        self.tilt_servo.angle = self.tilt_angle

        self.pan_target = self.pan_angle
        self.tilt_target = self.tilt_angle

        # Axis state
        self.pan_axis_filt = 0.0
        self.tilt_axis_filt = 0.0

        self.idle_detached = False

        # -------------------------
        # ROS interfaces
        # -------------------------
        self.sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.timer = self.create_timer(self.dt, self.update_servos)

        self.get_logger().info(
            f"PanTiltJoyNode started | "
            f"PAN GPIO {self.pan_gpio} [{self.pan_min}, {self.pan_max}] | "
            f"TILT GPIO {self.tilt_gpio} [{self.tilt_min}, {self.tilt_max}]"
        )

    # -------------------------
    # Joystick callback
    # -------------------------
    def joy_callback(self, msg: Joy):
        if len(msg.axes) <= max(self.pan_axis_idx, self.tilt_axis_idx):
            return

        pan_raw = msg.axes[self.pan_axis_idx]
        tilt_raw = msg.axes[self.tilt_axis_idx]

        # Axis low-pass filtering
        self.pan_axis_filt = (
            (1.0 - self.axis_alpha) * self.pan_axis_filt +
            self.axis_alpha * pan_raw
        )
        self.tilt_axis_filt = (
            (1.0 - self.axis_alpha) * self.tilt_axis_filt +
            self.axis_alpha * tilt_raw
        )

        pan_axis = 0.0 if abs(self.pan_axis_filt) < self.deadzone_pan else self.pan_axis_filt
        tilt_axis = 0.0 if abs(self.tilt_axis_filt) < self.deadzone_tilt else self.tilt_axis_filt

        if pan_axis != 0.0:
            self.pan_target = axis_to_range(pan_axis, self.pan_min, self.pan_max)

        if tilt_axis != 0.0:
            self.tilt_target = axis_to_range(-tilt_axis, self.tilt_min, self.tilt_max)

    # -------------------------
    # Servo update loop
    # -------------------------
    def update_servos(self):
        pan_idle = abs(self.pan_axis_filt) < self.deadzone_pan
        tilt_idle = abs(self.tilt_axis_filt) < self.deadzone_tilt

        if pan_idle and tilt_idle:
            if not self.idle_detached:
                try:
                    self.pan_servo.detach()
                    self.tilt_servo.detach()
                except Exception:
                    pass
                self.idle_detached = True
            return

        if self.idle_detached:
            self.idle_detached = False

        # Smooth motion
        self.pan_angle += self.pan_alpha * (self.pan_target - self.pan_angle)
        self.pan_angle = clamp(self.pan_angle, self.pan_min, self.pan_max)
        self.pan_servo.angle = self.pan_angle

        self.tilt_angle += self.tilt_alpha * (self.tilt_target - self.tilt_angle)
        self.tilt_angle = clamp(self.tilt_angle, self.tilt_min, self.tilt_max)
        self.tilt_servo.angle = self.tilt_angle

    # -------------------------
    # Cleanup
    # -------------------------
    def destroy_node(self):
        try:
            self.pan_servo.detach()
            self.tilt_servo.detach()
        except Exception:
            pass
        super().destroy_node()


# ------------- main -------------
def main(args=None):
    rclpy.init(args=args)
    node = PanTiltJoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
