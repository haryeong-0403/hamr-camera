#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge

import cv2
import subprocess


class ImageToGstUdpNode(Node):
    """
    여러 카메라 토픽을 구독해 두고,
    /selected_cam_id 에서 받은 ID에 해당하는 카메라 영상만
    GStreamer(H.264/UDP)로 보내는 브릿지 노드.
    """

    def __init__(self):
        super().__init__('image_to_gst_udp')

        # --- Parameters ---
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('host', '192.168.0.131') # 받는 ip
        self.declare_parameter('port', 5001)

        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().double_value
        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value

        if self.fps <= 0:
            self.fps = 30.0

        self.bridge = CvBridge()

        # -----------------------------
        # 현재 선택된 카메라 ID (기본 1)
        # -----------------------------
        self.current_cam_id = 1

        # 사용할 카메라 ID ↔ 토픽 매핑
        # 필요하면 여기다 3,4번 카메라도 더 추가하면 됨
        self.cam_topics = {
            1: 'usb_cam/image_raw_1',
            2: 'usb_cam/image_raw_2',
        }

        # 각 카메라 토픽에 대해 개별 subscriber 생성
        for cam_id, topic in self.cam_topics.items():
            self.create_subscription(
                Image,
                topic,
                # lambda 캡쳐 주의: cam_id를 기본 인자로 넘겨줌
                lambda msg, cid=cam_id: self.image_callback(msg, cid),
                10
            )
            self.get_logger().info(f"Subscribed camera {cam_id}: {topic}")

        # 선택된 카메라 ID를 받는 토픽
        self.id_sub = self.create_subscription(
            Int32,
            'selected_cam_id',  # 다른 이름 원하면 여기만 바꾸면 됨
            self.id_callback,
            10
        )

        # --- GStreamer 파이프라인 구성 ---
        pipeline_str = (
            "gst-launch-1.0 -q "
            "fdsrc ! "
            f"videoparse format=rgb width={self.width} height={self.height} framerate={int(self.fps)}/1 ! "
            "videoconvert ! "
            "x264enc tune=zerolatency speed-preset=ultrafast bitrate=4000 ! "
            "rtph264pay config-interval=1 pt=96 ! "
            f"udpsink host={self.host} port={self.port}"
        )

        self.get_logger().info(f"GStreamer pipeline:\n{pipeline_str}")

        self.gst_process = subprocess.Popen(
            pipeline_str,
            shell=True,
            stdin=subprocess.PIPE
        )
        if self.gst_process.stdin is None:
            raise RuntimeError("Failed to open GStreamer stdin")

        self.get_logger().info(
            f"ImageToGstUdpNode started. "
            f"Active camera ID: {self.current_cam_id} "
            f"-> UDP {self.host}:{self.port} (H.264/RTP)"
        )

    # 선택 ID 콜백: /selected_cam_id에서 온 값으로 current_cam_id 변경
    def id_callback(self, msg: Int32):
        new_id = msg.data
        if new_id not in self.cam_topics:
            self.get_logger().warn(f"Received invalid cam_id={new_id}, known IDs={list(self.cam_topics.keys())}")
            return

        self.current_cam_id = new_id
        self.get_logger().info(f"Switched active camera to ID={self.current_cam_id} "
                               f"({self.cam_topics[self.current_cam_id]})")

    # 각 카메라 토픽에서 호출되는 콜백
    def image_callback(self, msg: Image, cam_id: int):
        # 현재 선택된 카메라가 아니면 무시
        if cam_id != self.current_cam_id:
            return

        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert Image msg: {e}")
            return

        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        try:
            self.gst_process.stdin.write(frame_rgb.tobytes())
        except BrokenPipeError:
            self.get_logger().error("GStreamer pipeline terminated (Broken pipe)")
        except Exception as e:
            self.get_logger().error(f"Failed to write frame to GStreamer: {e}")

    def destroy_node(self):
        try:
            if self.gst_process and self.gst_process.stdin:
                self.gst_process.stdin.close()
            if self.gst_process:
                self.gst_process.terminate()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ImageToGstUdpNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Exception in ImageToGstUdpNode: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
