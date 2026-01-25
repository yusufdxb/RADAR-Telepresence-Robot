import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class PanTiltNode(Node):
    def __init__(self):
        super().__init__('pan_tilt_node')
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/pan_tilt_cmd',
            self.callback,
            10
        )

    def callback(self, msg):
        pan, tilt = msg.data
        self.get_logger().info(f'Pan: {pan:.2f}, Tilt: {tilt:.2f}')

def main():
    rclpy.init()
    node = PanTiltNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
