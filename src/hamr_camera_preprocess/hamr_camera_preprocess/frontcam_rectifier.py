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
    Rr = np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0,np.sin(r),np.cos(r)]],np.float32)
    return Ry @ Rp @ Rr

class Rectifier(Node):
    def __init__(self):
        super().__init__('cam1_rectifier')
        self.bridge = CvBridge()

        # 보기 여부 파라미터 (기본 True)
        self.declare_parameter('show', True)
        self.show = self.get_parameter('show').get_parameter_value().bool_value

        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        self.sub = self.create_subscription(
            Image, '/cam1/image_raw', self.cb, sensor_qos)

        self.pub = self.create_publisher(Image, '/cam1/image_raw/rectified', 10)

        # 미리 고정 출력 크기/맵 생성
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

        # 퍼블리시
        out = self.bridge.cv2_to_imgmsg(rect, encoding='bgr8')
        out.header = msg.header
        self.pub.publish(out)

        # 보기용 (옵션)
        if self.show:
            try:
                cv2.imshow("Rectified View (/cam1/image_raw/rectified)", rect)
                # 1ms 이벤트 처리 (창 응답성 확보)
                cv2.waitKey(1)
            except cv2.error as e:
                # 헤드리스일 때 등 에러 방지
                self.get_logger().warn(f"cv2.imshow error: {e}")
                self.show = False  # 이후 호출 비활성화

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
        # 창 정리
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
