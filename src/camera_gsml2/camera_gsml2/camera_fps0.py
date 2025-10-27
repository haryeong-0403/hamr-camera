#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage


class Camera0Publisher(Node):
    def __init__(self):
        super().__init__('camera0_publisher')

        # /dev/video2 오픈
        self.cap = cv2.VideoCapture(2)
        
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open /dev/video2")
            exit(1)

        # 퍼블리셔 생성
        self.pub_raw = self.create_publisher(Image, '/camera0/image_raw', 10)
        self.pub_compressed = self.create_publisher(CompressedImage, '/camera0/image_raw/compressed', 10)

        self.bridge = CvBridge()

        # FPS 측정 변수
        self.last_time = time.time()
        self.frame_count = 0

    def loop(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("Failed to read frame from /dev/video2")
                continue

            # self.publish_frame(frame)
            self.calc_fps()

            rclpy.spin_once(self, timeout_sec=0.01)

# ----------------------------------------------------------------------------------------------- #
    def publish_frame(self, frame):
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        msg_raw = self.bridge.cv2_to_imgmsg(frame_rgb, encoding='rgb8')
        msg_raw.header.stamp = self.get_clock().now().to_msg()
        msg_raw.header.frame_id = "camera0_frame"
        self.pub_raw.publish(msg_raw)

        msg_compressed = self.bridge.cv2_to_compressed_imgmsg(frame_rgb, dst_format='jpeg')
        msg_compressed.header.stamp = msg_raw.header.stamp
        msg_compressed.header.frame_id = "camera0_frame"
        self.pub_compressed.publish(msg_compressed)

    def calc_fps(self):
        self.frame_count += 1
        if self.frame_count >= 30: # 30프레임마다 한 번씩 평균 FPS 측정
            print("frame count: ", self.frame_count)
            now = time.time()
            elapsed = now - self.last_time
            fps = self.frame_count / elapsed
            self.get_logger().info(f"[camera0] Average FPS: {fps:.2f}")
            self.last_time = now
            self.frame_count = 0
# ----------------------------------------------------------------------------------------------- #

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Camera0Publisher()
    try:
        node.loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
