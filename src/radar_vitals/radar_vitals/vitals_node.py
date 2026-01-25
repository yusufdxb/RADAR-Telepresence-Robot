import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import random

class VitalsNode(Node):
    def __init__(self):
        super().__init__('vitals_node')
        self.pub = self.create_publisher(Float32MultiArray, '/vitals', 10)
        self.timer = self.create_timer(1.0, self.publish)

    def publish(self):
        msg = Float32MultiArray()
        heart_rate = random.uniform(60.0, 100.0)
        spo2 = random.uniform(95.0, 100.0)
        msg.data = [heart_rate, spo2]
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = VitalsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
