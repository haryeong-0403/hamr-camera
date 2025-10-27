#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber_node')

        # QoS 설정: LiDAR 토픽은 일반적으로 Reliable + SensorDataProfile 사용
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber 생성
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar1/cloud_unstructured_fullframe',
            self.pointcloud_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning

        self.get_logger().info('🚀 LiDAR subscriber started and listening to /lidar1/cloud_unstructured_fullframe')

    def pointcloud_callback(self, msg):
        # PointCloud2 메시지에서 일부 포인트만 출력해보기
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points_list = list(points)
        if len(points_list) > 0:
            first_point = points_list[0]
            self.get_logger().info(f'First point (x={first_point[0]:.2f}, y={first_point[1]:.2f}, z={first_point[2]:.2f})')
        else:
            self.get_logger().warn('⚠️ No points received in the cloud.')

def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('❌ Lidar subscriber stopped manually')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
