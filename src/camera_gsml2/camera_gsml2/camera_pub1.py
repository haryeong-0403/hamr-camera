#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_pub1')

        # /dev/video2 오픈
        self.cap = cv2.VideoCapture(2)   
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open /dev/video2")
            exit(1)

        # 퍼블리셔 생성
        self.publisher_raw = self.create_publisher(Image, '/camera1/image_raw', 10)
        self.publisher_compressed = self.create_publisher(CompressedImage, '/camera1/image_raw/compressed', 10)
        self.bridge = CvBridge()

        # 30fps 기준 (≈ 33ms 주기)
        timer_period = 1.0 / 30.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame from camera")
            return

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # OpenCV → ROS Image 변환 (raw)
        msg_raw = self.bridge.cv2_to_imgmsg(frame_rgb, encoding='rgb8')
        msg_raw.header.stamp = self.get_clock().now().to_msg()
        msg_raw.header.frame_id = "camera_frame"
        self.publisher_raw.publish(msg_raw)

        # OpenCV → CompressedImage 변환 (jpeg)
        msg_compressed = self.bridge.cv2_to_compressed_imgmsg(frame_rgb, dst_format='jpeg')
        msg_compressed.header.stamp = msg_raw.header.stamp
        msg_compressed.header.frame_id = "camera_frame"
        self.publisher_compressed.publish(msg_compressed)

        self.get_logger().info(
            f'Publishing raw + compressed frame: {msg_raw.width}x{msg_raw.height}, encoding={msg_raw.encoding}'
        )

    def destroy_node(self):
        # 노드 종료 시 카메라 해제
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    print("-------------node start-----------------")
    node = CameraPublisher()
    print("-------------node end-----------------")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
