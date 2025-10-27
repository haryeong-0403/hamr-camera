import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        
        # 1. YOLO 모델 로드 (가장 가벼운 nano 모델)
        # yolo11n.pt 파일이 없으면 처음 실행 시 자동으로 다운로드됩니다.
        self.model = YOLO('yolo11n.pt') 
        self.get_logger().info('✅ YOLOv11 model loaded successfully.')

        # 2. CV Bridge 초기화
        self.bridge = CvBridge()

        # 3. 이미지 토픽('/cam1/image_raw/rectified') 구독자 생성
        self.subscription = self.create_subscription(
            Image,
            '/cam1/image_raw/rectified',
            self.image_callback,
            10)
        self.get_logger().info('✅ Subscribed to /cam1/image_raw/rectified')

    def image_callback(self, msg):
        """이미지 메시지를 받았을 때 실행되는 콜백 함수"""
        self.get_logger().info('Image received...', throttle_duration_sec=2) # 2초에 한 번씩만 로그 출력
        try:
            # ROS 이미지 메시지를 OpenCV 이미지(bgr8)로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # YOLO 모델로 객체 탐지 수행
        results = self.model(cv_image)

        # 탐지 결과를 원본 이미지에 그리기
        annotated_frame = results[0].plot()

        # 결과 이미지를 화면에 표시
        cv2.imshow("YOLOv11 Real-time Detection", annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    yolo_detector_node = YoloDetectorNode()
    
    try:
        rclpy.spin(yolo_detector_node)
    except KeyboardInterrupt:
        self.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        # 노드 종료 시 자원 정리
        yolo_detector_node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
