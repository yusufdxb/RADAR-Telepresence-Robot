#!/usr/bin/env python3
"""
Simulated vitals node for RADAR — development / testing only.

Publishes plausible-range heart rate and SpO2 values to the same topics
as pulse_ox_node so the rest of the system can be tested without hardware.
Use pulse_ox_node for real MAX30102 sensor readings.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import random
import math


class VitalsSimNode(Node):
    def __init__(self):
        super().__init__('vitals_node')

        self.declare_parameter('publish_hz', 1.0)

        hz = self.get_parameter('publish_hz').value

        self.pub_vitals = self.create_publisher(
            Float32MultiArray, '/radar/pulseox/vitals', 10
        )

        self._t = 0.0
        self._dt = 1.0 / hz
        self.timer = self.create_timer(self._dt, self.publish)

        self.get_logger().warn(
            'VitalsSimNode running — SIMULATED data on /radar/pulseox/vitals. '
            'Use pulse_ox_node for real sensor data.'
        )

    def publish(self):
        self._t += self._dt

        # Gentle sine variation to look realistic
        hr   = 72.0 + 8.0 * math.sin(self._t / 5.0) + random.gauss(0, 0.5)
        spo2 = 98.0 + 1.0 * math.sin(self._t / 8.0) + random.gauss(0, 0.1)
        spo2 = min(spo2, 100.0)

        msg = Float32MultiArray()
        msg.data = [float(hr), float(spo2)]
        self.pub_vitals.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VitalsSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
