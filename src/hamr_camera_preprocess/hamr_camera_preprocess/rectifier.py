#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2, numpy as np

K = np.array([[519.1, 0.0, 961.2],
              [  0.0, 520.2, 753.5],
              [  0.0,   0.0,   1.0]], dtype=np.float32)
D = np.array([0.142, 0.017, -0.192, 0.208], dtype=np.float32)

def euler_deg_to_R(yaw=0.0, pitch=0.0, roll=0.0):
    y,p,r = np.deg2rad([yaw,pitch,roll])
    Ry = np.array([[ np.cos(y),-np.sin(y),0],[np.sin(y),np.cos(y),0],[0,0,1]],np.float32)
    Rp = np.array([[ np.cos(p),0,np.sin(p)],[0,1,0],[-np.sin(p),0,np.cos(p)]],np.float32)
    Rr = np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0,np.sin(r),np.cos(r)],],np.float32)
    return Ry @ Rp @ Rr

class Rectifier(Node):
    def __init__(self):
        # 이름은 대충 'rectifier'로
        super().__init__('rectifier')

        self.bridge = CvBridge()

        # --- 파라미터 선언 ---
        # namespace 기준 상대 토픽 이름으로 사용
        self.declare_parameter('input_topic', 'image_raw')
        self.declare_parameter('output_topic', 'image_rect')
        self.declare_parameter('show', False)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.show = self.get_parameter('show').get_parameter_value().bool_value

        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        # ⚠️ 여기 중요: 앞에 '/' 안 붙음 → namespace를 따르는 상대 토픽
        self.get_logger().info(f"Subscribing to: {input_topic}")
        self.get_logger().info(f"Publishing to: {output_topic}")

        self.sub = self.create_subscription(
            Image, input_topic, self.cb, sensor_qos)

        self.pub = self.create_publisher(Image, output_topic, 10)

        # --- 미리 고정 출력 크기/맵 생성 ---
        self.out_w, self.out_h = 640, 480
        fov_deg = 110.0
        f = 0.5 * self.out_w / np.tan(np.deg2rad(fov_deg)/2)
        Knew = np.array([[f,0,self.out_w/2],[0,f,self.out_h/2],[0,0,1]], np.float32)
        R = euler_deg_to_R(0,0,-8).astype(np.float32)
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
            K, D, R, Knew, (self.out_w, self.out_h), cv2.CV_16SC2)

    def cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        rect = cv2.remap(frame, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)

        out = self.bridge.cv2_to_imgmsg(rect, encoding='bgr8')
        out.header = msg.header
        self.pub.publish(out)

        if self.show:
            try:
                cv2.imshow(f"Rectified ({self.get_fully_qualified_name()})", rect)
                cv2.waitKey(1)
            except cv2.error as e:
                self.get_logger().warn(f"cv2.imshow error: {e}")
                self.show = False

def main():
    rclpy.init()
    node = Rectifier()
    exec = MultiThreadedExecutor()
    exec.add_node(node)
    try:
        exec.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
