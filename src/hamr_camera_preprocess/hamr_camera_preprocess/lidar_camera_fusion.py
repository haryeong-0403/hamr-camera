#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from pathlib import Path

from ultralytics import YOLO

from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge

import message_filters
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, TransformStamped
from rclpy.duration import Duration

from pyquaternion import Quaternion

from hamr_camera_preprocess_msgs.msg import FusedObject, FusedObjectArray

from visualization_msgs.msg import Marker, MarkerArray



def transform_points_numpy(points: np.ndarray, transform: TransformStamped) -> np.ndarray:
    translation = np.array([
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z
    ], dtype=np.float64)
    quat = Quaternion(
        transform.transform.rotation.w,
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z
    )
    rotation_matrix = quat.rotation_matrix  # 3x3
    return np.dot(points, rotation_matrix.T) + translation


def qos_from_str(s: str) -> ReliabilityPolicy:
    return ReliabilityPolicy.BEST_EFFORT if s.lower() == 'best_effort' else ReliabilityPolicy.RELIABLE


class AdvancedFusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')

        # === 파라미터들 ===
        self.declare_parameter('camera_topic', '/cam1/image_raw/rectified')
        self.declare_parameter('lidar_topic', '/merged_points')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('base_frame', 'base_scan')

        # 기존 단일 파라미터(하위호환): sensor_qos
        self.declare_parameter('sensor_qos', 'best_effort')  # 전체 기본
        # 새 파라미터: 토픽별 QoS
        self.declare_parameter('image_qos', None)  # None이면 sensor_qos를 따름
        self.declare_parameter('lidar_qos', None)

        self.declare_parameter('yolo_weights', '/home/nvidia/ros2_ws/yolo11n.pt')
        self.declare_parameter('conf_threshold', 0.25)

        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        lidar_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        sensor_qos_default = self.get_parameter('sensor_qos').get_parameter_value().string_value or 'best_effort'
        image_qos_param = self.get_parameter('image_qos').get_parameter_value().string_value
        lidar_qos_param = self.get_parameter('lidar_qos').get_parameter_value().string_value

        image_qos_str = image_qos_param if image_qos_param else sensor_qos_default
        lidar_qos_str = lidar_qos_param if lidar_qos_param else sensor_qos_default

        self.conf_threshold = float(self.get_parameter('conf_threshold').get_parameter_value().double_value)
        weights_path = Path(self.get_parameter('yolo_weights').get_parameter_value().string_value)

        self.get_logger().info(f"Camera Topic: {camera_topic}")
        self.get_logger().info(f"Lidar Topic: {lidar_topic}")
        self.get_logger().info(f"Camera Frame: {self.camera_frame}")
        self.get_logger().info(f"QoS(image): {image_qos_str}, QoS(lidar): {lidar_qos_str}")
        self.get_logger().info(f"YOLO Weights: {weights_path}")

        # YOLO 모델 로드 
        self.model = None
        if weights_path.exists():
            try:
                self.model = YOLO(str(weights_path))
                self.get_logger().info(f"YOLO weights loaded: {weights_path}")
            except Exception as e:
                self.get_logger().warn(f"YOLO load failed ({e}). Running WITHOUT detector.")
        else:
            self.get_logger().warn(f"YOLO weights not found at {weights_path}. Running WITHOUT detector.")

        # 도구 초기화
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 카메라 내부 파라미터 (Intrinsic Matrix)
        self.focal_length = 0.5 * 640 / np.tan(np.deg2rad(110/2))
        self.camera_k = np.array([
            [self.focal_length,   0.0, 320.0],
            [  0.0, self.focal_length, 240.0],
            [  0.0,   0.0,   1.0]
        ], dtype=np.float32)

        # Publisher 초기화 
        self.fusion_pub = self.create_publisher(FusedObjectArray, '/fusion/fused_objects', 10)
        self.image_pub = self.create_publisher(Image, '/fusion/annotated_image', 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/fusion/markers", 10)

        # Subscriber 초기화 (토픽별 QoS) 
        qos_image = QoSProfile(depth=10, reliability=qos_from_str(image_qos_str), history=HistoryPolicy.KEEP_LAST)
        qos_lidar = QoSProfile(depth=10, reliability=qos_from_str(lidar_qos_str), history=HistoryPolicy.KEEP_LAST)

        self.image_sub = message_filters.Subscriber(self, Image, camera_topic, qos_profile=qos_image)
        self.lidar_sub = message_filters.Subscriber(self, PointCloud2, lidar_topic, qos_profile=qos_lidar)

        # 동기화 설정
        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.lidar_sub],
            queue_size=20,
            slop=0.2
        )
        self.time_synchronizer.registerCallback(self.fusion_callback)

        self.get_logger().info("Advanced Fusion Node has started successfully.")

    def fusion_callback(self, image_msg: Image, lidar_msg: PointCloud2):
        # 1) TF: LiDAR 좌표계 -> 카메라 좌표계 변환
        try:
            transform_lidar_to_cam = self.tf_buffer.lookup_transform(
                self.camera_frame,
                lidar_msg.header.frame_id,
                image_msg.header.stamp,
                timeout=Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warn(
                f"Failed to get transform '{lidar_msg.header.frame_id}' -> '{self.camera_frame}': {e}"
            )
            return

        # 2) 이미지/포인트클라우드 준비
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        try:
            lidar_points = pc2.read_points_numpy(lidar_msg, field_names=("x", "y", "z"))
            if lidar_points.size == 0:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
                return
        except Exception as e:
            self.get_logger().warn(f"Failed to read PointCloud2: {e}")
            return

        # 3) LiDAR → Camera 좌표 변환
        try:
            points_in_camera_frame = transform_points_numpy(lidar_points, transform_lidar_to_cam)
        except Exception as e:
            self.get_logger().warn(f"Point transform failed: {e}")
            return

        # 4) YOLO 객체 탐지 (옵션)
        detected_boxes = []
        class_names = {}
        results = None

        if self.model is not None:
            try:
                results = self.model(cv_image, verbose=False, conf=self.conf_threshold)
                if len(results) > 0:
                    detected_boxes = list(results[0].boxes) if results[0].boxes is not None else []
                    class_names = results[0].names if hasattr(results[0], 'names') else getattr(self.model, 'names', {})
            except Exception as e:
                self.get_logger().warn(f"YOLO inference failed: {e}")

        # 5) 투영 및 필터링
        valid_indices = points_in_camera_frame[:, 2] > 0.1
        points_3d_valid = points_in_camera_frame[valid_indices]

        if points_3d_valid.shape[0] == 0:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            return

        projected_points = self.camera_k @ points_3d_valid.T
        points_2d = (projected_points[:2] / projected_points[2]).T
        points_2d = np.round(points_2d).astype(int)

        h, w, _ = cv_image.shape
        in_image = (points_2d[:, 0] >= 0) & (points_2d[:, 0] < w) & \
                   (points_2d[:, 1] >= 0) & (points_2d[:, 1] < h)

        points_2d_in_image = points_2d[in_image]
        points_3d_in_image = points_3d_valid[in_image]

        for (u, v) in points_2d_in_image:  # 노란색 점
            cv2.circle(cv_image, (u, v), 1, (0, 255, 255), -1)
        
        fused_objects_msg = FusedObjectArray()
        fused_objects_msg.header.frame_id = self.base_frame
        fused_objects_msg.header.stamp = image_msg.header.stamp

        # 6) 객체별 퓨전
        boxes = None
        names = {}

        if self.model is not None:
            # YOLO가 위에서 돌았고 results 변수가 살아있다는 전제
            if 'results' in locals() and len(results) > 0 and results[0].boxes is not None:
                boxes = results[0].boxes
                # 클래스 이름 사전/리스트 확보
                names = (results[0].names if hasattr(results[0], 'names') and results[0].names
                        else getattr(self.model, 'names', {}) or {})

        if boxes is not None:
            for i in range(len(boxes)):
                # 1) 박스 좌표
                x1, y1, x2, y2 = [int(v) for v in boxes.xyxy[i].tolist()]

                # 2) 박스 내부 포인트 필터
                inside = ((points_2d_in_image[:, 0] >= x1) & (points_2d_in_image[:, 0] <= x2) &
                        (points_2d_in_image[:, 1] >= y1) & (points_2d_in_image[:, 1] <= y2))
                if int(np.sum(inside)) <= 10:
                    continue

                # 3) 3D 위치(카메라 좌표계) 대표값
                obj_pts_cam = points_3d_in_image[inside]
                median_pos_in_cam = np.median(obj_pts_cam, axis=0)

                # 4) 클래스/신뢰도 스칼라화 + 이름 매핑
                cls_id = int(boxes.cls[i].item())          # Tensor -> int
                conf   = float(boxes.conf[i].item())       # Tensor -> float
                if isinstance(names, dict):
                    cls_name = names.get(cls_id, 'unknown')
                elif isinstance(names, (list, tuple)):
                    cls_name = names[cls_id] if 0 <= cls_id < len(names) else 'unknown'
                else:
                    cls_name = 'unknown'

                # 5) base_frame으로 좌표 변환
                fused_obj = FusedObject()
                fused_obj.class_name = cls_name
                fused_obj.confidence = conf
                fused_obj.x_min, fused_obj.y_min, fused_obj.x_max, fused_obj.y_max = x1, y1, x2, y2

                p_cam = PointStamped()
                p_cam.header.frame_id = self.camera_frame
                p_cam.header.stamp = image_msg.header.stamp
                p_cam.point.x, p_cam.point.y, p_cam.point.z = map(float, median_pos_in_cam)

                try:
                    transform_cam_to_base = self.tf_buffer.lookup_transform(
                        self.base_frame, self.camera_frame, image_msg.header.stamp,
                        timeout=Duration(seconds=0.1)
                    )
                    p_base = tf2_geometry_msgs.do_transform_point(p_cam, transform_cam_to_base).point
                    fused_obj.position_x = float(p_base.x)
                    fused_obj.position_y = float(p_base.y)
                    fused_obj.position_z = float(p_base.z)
                except Exception as e:
                    self.get_logger().warn(f"Transform '{self.camera_frame}' -> '{self.base_frame}' failed: {e}")
                    continue

                fused_objects_msg.objects.append(fused_obj)

                # 6) 시각화
                label = f"{cls_name} {conf:.2f}"
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(cv_image, label, (x1, max(0, y1 - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                dist_label = f"Dist(cam): {median_pos_in_cam[2]:.2f}m"
                cv2.putText(cv_image, dist_label, (x1, min(h-5, y2 + 20)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        markers = MarkerArray()

        # 이전 프레임 마커 싹 지우기(잔상 제거)
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        # 이번 프레임의 모든 객체에 대해 구(SPHERE) + 텍스트 라벨 추가
        for i, obj in enumerate(fused_objects_msg.objects):
            # 3D 위치 구(SPHERE)
            m = Marker()
            m.header.frame_id = self.base_frame            # 'base_scan' 권장 (PointCloud와 동일 프레임)
            m.header.stamp = image_msg.header.stamp
            m.ns = "fused_objects"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD

            m.pose.position.x = obj.position_x
            m.pose.position.y = obj.position_y
            m.pose.position.z = obj.position_z
            m.pose.orientation.x = 0.0
            m.pose.orientation.y = 0.0
            m.pose.orientation.z = 0.0
            m.pose.orientation.w = 1.0

            m.scale.x = 0.12   # 구 지름(m)
            m.scale.y = 0.12
            m.scale.z = 0.12

            m.color.r = 0.0
            m.color.g = 1.0    # 초록
            m.color.b = 0.0
            m.color.a = 1.0

            m.lifetime = Duration(seconds=0.3).to_msg()  # 약간의 잔상만 남기기
            markers.markers.append(m)

            # 텍스트 라벨(클래스명/신뢰도)
            t = Marker()
            t.header.frame_id = self.base_frame
            t.header.stamp = image_msg.header.stamp
            t.ns = "fused_labels"
            t.id = 10000 + i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD

            t.pose.position.x = obj.position_x
            t.pose.position.y = obj.position_y
            t.pose.position.z = obj.position_z + 0.25   # 구 위로 조금 띄우기

            t.scale.z = 0.18   # 텍스트 ‘폰트 높이’(m)

            t.color.r = 1.0
            t.color.g = 1.0
            t.color.b = 0.0
            t.color.a = 1.0

            t.text = f"{obj.class_name} {obj.confidence:.2f}"
            t.lifetime = Duration(seconds=0.3).to_msg()
            markers.markers.append(t)

        # 퍼블리시!
        self.marker_pub.publish(markers)

        # 7) 결과 발행
        if len(fused_objects_msg.objects) > 0:
            self.fusion_pub.publish(fused_objects_msg)
            self.get_logger().info(f"Published {len(fused_objects_msg.objects)} fused objects.")
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))


def main(args=None):
    rclpy.init(args=args)
    node = AdvancedFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("AdvancedFusionNode stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
