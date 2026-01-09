#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SimpleCmdVelPub(Node):
    def __init__(self):
        super().__init__("simple_cmd_vel_pub")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(0.5, self.tick)
        self.toggle = True
        self.get_logger().info("Publishing demo /cmd_vel (forward/stop) every 0.5s")

    def tick(self):
        msg = Twist()
        if self.toggle:
            msg.linear.x = 0.10
        else:
            msg.linear.x = 0.0
        self.pub.publish(msg)
        self.toggle = not self.toggle


def main():
    rclpy.init()
    node = SimpleCmdVelPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

