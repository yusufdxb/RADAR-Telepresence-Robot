#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('frame_id', 'camera_link')

        idx  = self.get_parameter('camera_index').value
        fps  = self.get_parameter('frame_rate').value
        w    = self.get_parameter('image_width').value
        h    = self.get_parameter('image_height').value
        self.frame_id = self.get_parameter('frame_id').value

        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(idx)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv2.CAP_PROP_FPS,          fps)

        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open camera index {idx}')
        else:
            actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.get_logger().info(
                f'Camera opened: index={idx}, {actual_w}x{actual_h} @ {fps} Hz'
            )

        self.pub_image = self.create_publisher(Image, '/radar/camera/image_raw', 10)

        self.timer = self.create_timer(1.0 / fps, self.capture_and_publish)

    def capture_and_publish(self):
        if not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Frame capture failed — skipping', throttle_duration_sec=5.0)
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub_image.publish(msg)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
