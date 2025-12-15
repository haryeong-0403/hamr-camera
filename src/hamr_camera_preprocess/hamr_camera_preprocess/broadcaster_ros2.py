#!/usr/bin/env python3
"""
WebRTC 영상 송출기 (ROS2 + aiortc)
- ROS2 Image 토픽(/cam0/image_raw 등)을 subscribe 해서 WebRTC로 송출합니다.
- single: 단일 토픽 1채널 송출
- multi : 4개 토픽 4채널 + 2x2 합성 1채널(총 5채널) 송출

사용 예시:
  # 4채널 + 합성 1채널 (권장)
  python3 broadcaster_ros2.py --mode multi \
    --server ws://localhost:8083 \
    --topics /cam0/image_raw,/cam1/image_raw,/cam2/image_raw,/cam3/image_raw \
    --rooms amr-front,amr-back,amr-right,amr-left \
    --combined-room amr-all

  # 단일 채널
  python3 broadcaster_ros2.py --mode single --room robot-front --topic /cam0/image_raw

요구사항:
  pip install aiortc aiohttp av opencv-python numpy
  sudo apt install ros-$ROS_DISTRO-cv-bridge

주의:
  - 토픽 이미지 인코딩이 rgb8/bgr8/mono8 등 섞여도 desired_encoding="bgr8"로 통일해서 처리합니다.
  - ROS2는 별도 스레드로 spin 시켜 asyncio(WebRTC)와 같이 동작합니다.
"""

import asyncio
import argparse
import json
import logging
import threading
from fractions import Fraction
from datetime import datetime
from typing import Dict, Optional, List, Tuple
from std_msgs.msg import Int32   

import cv2
import numpy as np

# aiortc
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from av import VideoFrame

# WebSocket signaling
import aiohttp

# ROS2
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("broadcaster_ros2")


def _blank_rgb(width: int = 640, height: int = 480, text: str = "N/A") -> np.ndarray:
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    cv2.putText(frame, text, (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 0, 0), 3)
    return frame


def _put_label_rgb(frame: np.ndarray, label: str) -> np.ndarray:
    cv2.rectangle(frame, (0, 0), (240, 40), (0, 0, 0), thickness=-1)
    cv2.putText(frame, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    return frame


# class MultiCamBuffer(Node):
#     """
#     여러 ROS2 Image 토픽을 구독하고, topic별 최신 프레임(bgr8)을 저장합니다.
#     """

#     def __init__(self, topics: List[str]):
#         super().__init__("webrtc_multicam_buffer")
#         self.bridge = CvBridge()
#         self.lock = threading.Lock()
#         self.latest_bgr: Dict[str, Optional[np.ndarray]] = {t: None for t in topics}

#         qos = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=1,
#         )

#         for t in topics:
#             self.create_subscription(Image, t, lambda msg, tt=t: self._cb(msg, tt), qos)
#             self.get_logger().info(f"Subscribed: {t}")

#     def _cb(self, msg: Image, topic: str):
#         try:
#             # 원하는 포맷으로 통일(bgr8)
#             bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
#         except Exception as e:
#             # 너무 시끄럽지 않게 debug 수준
#             self.get_logger().debug(f"cv_bridge convert failed on {topic}: {e}")
#             return

#         with self.lock:
#             self.latest_bgr[topic] = bgr

#     def get_bgr(self, topic: str) -> Optional[np.ndarray]:
#         with self.lock:
#             f = self.latest_bgr.get(topic)
#             return None if f is None else f.copy()

class MultiCamBuffer(Node):
    """
    여러 ROS2 Image 토픽을 구독하고, topic별 최신 프레임(bgr8)을 저장합니다.
    + /selected_cam_id(Int32) 를 구독해서 현재 선택된 topic을 바꿔줌.
    """

    def __init__(self, topics: List[str]):
        super().__init__("webrtc_multicam_buffer")
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.topics = topics
        self.latest_bgr: Dict[str, Optional[np.ndarray]] = {t: None for t in topics}

        # 현재 선택된 topic (기본: 0번)
        self.current_index: int = 0 if topics else -1
        self.current_topic: Optional[str] = topics[0] if topics else None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 카메라 이미지 토픽들 구독
        for t in topics:
            self.create_subscription(Image, t, lambda msg, tt=t: self._cb(msg, tt), qos)
            self.get_logger().info(f"Subscribed: {t}")

        # 선택된 카메라 ID 구독 (/selected_cam_id)
        self.create_subscription(Int32, "selected_cam_id", self._id_cb, 10)
        self.get_logger().info("Subscribed: /selected_cam_id (Int32)")

    def _cb(self, msg: Image, topic: str):
        try:
            # 원하는 포맷으로 통일(bgr8)
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            # 너무 시끄럽지 않게 debug 수준
            self.get_logger().debug(f"cv_bridge convert failed on {topic}: {e}")
            return

        with self.lock:
            self.latest_bgr[topic] = bgr

    def _id_cb(self, msg: Int32):
        """
        /selected_cam_id 콜백
        msg.data 를 topics 리스트의 인덱스로 사용 (0 ~ len-1)
        """
        idx = msg.data
        if 0 <= idx < len(self.topics):
            with self.lock:
                self.current_index = idx
                self.current_topic = self.topics[idx]
            self.get_logger().info(
                f"Selected cam_id={idx} → topic={self.current_topic}"
            )
        else:
            self.get_logger().warn(
                f"Invalid cam_id={idx}. 유효 범위: 0 ~ {len(self.topics)-1}"
            )

    def get_bgr(self, topic: str) -> Optional[np.ndarray]:
        with self.lock:
            f = self.latest_bgr.get(topic)
            return None if f is None else f.copy()

    def get_current_topic(self) -> Optional[str]:
        with self.lock:
            return self.current_topic


# class Ros2ImageVideoTrack(VideoStreamTrack):
#     """
#     ROS2 토픽에서 받은 최신 프레임을 WebRTC용 VideoStreamTrack으로 제공합니다.
#     """

#     def __init__(self, buffer_node: MultiCamBuffer, topic: str, label: str, fps: int = 30,
#                  out_size: Tuple[int, int] = (640, 480)):
#         super().__init__()
#         self.buf = buffer_node
#         self.topic = topic
#         self.label = label
#         self.fps = fps
#         self.out_w, self.out_h = out_size
#         self._start_time = None
#         self._frame_count = 0

#     async def recv(self) -> VideoFrame:
#         if self._start_time is None:
#             self._start_time = asyncio.get_event_loop().time()

#         pts = self._frame_count
#         self._frame_count += 1

#         target = self._start_time + (pts / self.fps)
#         now = asyncio.get_event_loop().time()
#         if target > now:
#             await asyncio.sleep(target - now)

#         bgr = self.buf.get_bgr(self.topic)

#         if bgr is None:
#             rgb = _blank_rgb(width=self.out_w, height=self.out_h, text=f"{self.label}: N/A")
#         else:
#             # resize then convert for stable output size
#             if (bgr.shape[1], bgr.shape[0]) != (self.out_w, self.out_h):
#                 bgr = cv2.resize(bgr, (self.out_w, self.out_h))
#             rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
#             rgb = _put_label_rgb(rgb, self.label)

#         vf = VideoFrame.from_ndarray(rgb, format="rgb24")
#         vf.pts = pts
#         vf.time_base = Fraction(1, self.fps)
#         return vf

class Ros2ImageVideoTrack(VideoStreamTrack):
    """
    ROS2 토픽에서 받은 최신 프레임을 WebRTC용 VideoStreamTrack으로 제공합니다.
    - fixed_topic 이 None이면 MultiCamBuffer의 current_topic을 사용 (선택 모드)
    - fixed_topic 이 주어지면 해당 토픽만 계속 사용 (기존 single/multi 용)
    """

    def __init__(self, buffer_node: MultiCamBuffer,
                 topic: Optional[str],        # ← None이면 선택 모드
                 label: str,
                 fps: int = 30,
                 out_size: Tuple[int, int] = (640, 480)):
        super().__init__()
        self.buf = buffer_node
        self.fixed_topic = topic      # 고정 토픽 or None
        self.label = label
        self.fps = fps
        self.out_w, self.out_h = out_size
        self._start_time = None
        self._frame_count = 0

    async def recv(self) -> VideoFrame:
        if self._start_time is None:
            self._start_time = asyncio.get_event_loop().time()

        pts = self._frame_count
        self._frame_count += 1

        target = self._start_time + (pts / self.fps)
        now = asyncio.get_event_loop().time()
        if target > now:
            await asyncio.sleep(target - now)

        # 🔑 현재 사용할 topic 결정
        if self.fixed_topic is not None:
            topic = self.fixed_topic
        else:
            topic = self.buf.get_current_topic()

        bgr = self.buf.get_bgr(topic) if topic is not None else None

        if bgr is None:
            txt = f"{self.label}: N/A"
            rgb = _blank_rgb(width=self.out_w, height=self.out_h, text=txt)
        else:
            if (bgr.shape[1], bgr.shape[0]) != (self.out_w, self.out_h):
                bgr = cv2.resize(bgr, (self.out_w, self.out_h))
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            rgb = _put_label_rgb(rgb, self.label or (topic or "N/A"))

        vf = VideoFrame.from_ndarray(rgb, format="rgb24")
        vf.pts = pts
        vf.time_base = Fraction(1, self.fps)
        return vf



class Ros2CompositeQuadTrack(VideoStreamTrack):
    """
    4개 토픽 프레임을 2x2 그리드로 합성하여 송출합니다.
    """

    def __init__(self, buffer_node: MultiCamBuffer, topic_label_pairs: List[Tuple[str, str]],
                 fps: int = 30, cell_size: Tuple[int, int] = (640, 480)):
        super().__init__()
        self.buf = buffer_node
        self.pairs = topic_label_pairs
        self.fps = fps
        self.cell_w, self.cell_h = cell_size
        self._start_time = None
        self._frame_count = 0

    async def recv(self) -> VideoFrame:
        if self._start_time is None:
            self._start_time = asyncio.get_event_loop().time()

        pts = self._frame_count
        self._frame_count += 1

        target = self._start_time + (pts / self.fps)
        now = asyncio.get_event_loop().time()
        if target > now:
            await asyncio.sleep(target - now)

        frames: List[np.ndarray] = []
        for topic, label in self.pairs[:4]:
            bgr = self.buf.get_bgr(topic)
            if bgr is None:
                rgb = _blank_rgb(width=self.cell_w, height=self.cell_h, text=f"{label}: N/A")
            else:
                if (bgr.shape[1], bgr.shape[0]) != (self.cell_w, self.cell_h):
                    bgr = cv2.resize(bgr, (self.cell_w, self.cell_h))
                rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                rgb = _put_label_rgb(rgb, label)
            frames.append(rgb)

        while len(frames) < 4:
            frames.append(_blank_rgb(width=self.cell_w, height=self.cell_h, text="EMPTY"))

        top = np.concatenate(frames[:2], axis=1)
        bottom = np.concatenate(frames[2:], axis=1)
        grid = np.concatenate([top, bottom], axis=0)

        vf = VideoFrame.from_ndarray(grid, format="rgb24")
        vf.pts = pts
        vf.time_base = Fraction(1, self.fps)
        return vf


class WebRTCBroadcaster:
    def __init__(self, server_url: str, room_id: str, video_track: VideoStreamTrack):
        self.server_url = server_url
        self.room_id = room_id
        self.ws = None
        self.session: Optional[aiohttp.ClientSession] = None
        self.peer_connections: Dict[str, RTCPeerConnection] = {}  # viewerId -> pc
        self.video_track = video_track
        self.running = False

    async def connect(self):
        logger.info(f"시그널링 서버 연결: {self.server_url}")
        self.session = aiohttp.ClientSession()
        try:
            self.ws = await self.session.ws_connect(self.server_url)
            logger.info("시그널링 서버 연결 성공")

            # 송출자로 방 참가
            await self.ws.send_json({"type": "join-as-broadcaster", "roomId": self.room_id})

            self.running = True

            async for msg in self.ws:
                if msg.type == aiohttp.WSMsgType.TEXT:
                    data = json.loads(msg.data)
                    await self.handle_message(data)
                elif msg.type == aiohttp.WSMsgType.ERROR:
                    logger.error(f"WebSocket 오류: {self.ws.exception()}")
                    break

        except Exception as e:
            logger.error(f"연결 오류({self.room_id}): {e}")
        finally:
            await self.cleanup()

    async def handle_message(self, data):
        msg_type = data.get("type")

        if msg_type == "joined":
            logger.info(f"방 참가 완료: {data.get('roomId')}")
            print(f'\n{"=" * 50}')
            print(f"  송출 시작! (room: {self.room_id})")
            print(f'  ACS 영상 페이지에서 "{self.room_id}" 로 연결하세요')
            print(f'{"=" * 50}\n')

        elif msg_type == "viewer-joined":
            viewer_id = data.get("viewerId")
            logger.info(f"[{self.room_id}] 새 시청자: {viewer_id}")
            await self.create_peer_connection(viewer_id)

        elif msg_type == "viewer-left":
            viewer_id = data.get("viewerId")
            logger.info(f"[{self.room_id}] 시청자 퇴장: {viewer_id}")
            await self.close_peer_connection(viewer_id)

        elif msg_type == "answer":
            viewer_id = data.get("viewerId")
            answer = data.get("answer")
            await self.handle_answer(viewer_id, answer)

        elif msg_type == "ice-candidate":
            # aiortc는 일반적으로 setRemoteDescription 이후 candidate add 처리 가능하나,
            # 이 예제 signaling 서버/클라이언트 구현에 따라 생략되어도 동작하는 경우가 많습니다.
            # 필요하면 RTCIceCandidate로 변환해서 addIceCandidate를 구현하세요.
            pass

        elif msg_type == "error":
            logger.error(f"[{self.room_id}] 오류: {data.get('message')}")

    async def create_peer_connection(self, viewer_id: str):
        try:
            pc = RTCPeerConnection()
            self.peer_connections[viewer_id] = pc

            pc.addTrack(self.video_track)

            @pc.on("icecandidate")
            async def on_icecandidate(candidate):
                if candidate and self.ws:
                    await self.ws.send_json({
                        "type": "ice-candidate",
                        "viewerId": viewer_id,
                        "candidate": {
                            "candidate": candidate.candidate,
                            "sdpMid": candidate.sdpMid,
                            "sdpMLineIndex": candidate.sdpMLineIndex,
                        },
                    })

            @pc.on("connectionstatechange")
            async def on_connectionstatechange():
                logger.info(f"[{self.room_id}] {viewer_id} 연결 상태: {pc.connectionState}")
                if pc.connectionState == "failed":
                    await self.close_peer_connection(viewer_id)

            offer = await pc.createOffer()
            await pc.setLocalDescription(offer)

            await self.ws.send_json({
                "type": "offer",
                "viewerId": viewer_id,
                "offer": {"type": pc.localDescription.type, "sdp": pc.localDescription.sdp},
            })

            logger.info(f"[{self.room_id}] {viewer_id}에게 Offer 전송")

        except Exception as e:
            logger.error(f"[{self.room_id}] PeerConnection 생성 오류: {e}")

    async def handle_answer(self, viewer_id: str, answer):
        pc = self.peer_connections.get(viewer_id)
        if pc and answer:
            await pc.setRemoteDescription(RTCSessionDescription(type=answer["type"], sdp=answer["sdp"]))
            logger.info(f"[{self.room_id}] {viewer_id}로부터 Answer 수신")

    async def close_peer_connection(self, viewer_id: str):
        pc = self.peer_connections.pop(viewer_id, None)
        if pc:
            await pc.close()

    async def cleanup(self):
        if not self.running and self.session is None:
            return

        self.running = False

        # 모든 PeerConnection 종료
        for _, pc in list(self.peer_connections.items()):
            try:
                await pc.close()
            except Exception:
                pass
        self.peer_connections.clear()

        # WebSocket 종료
        try:
            if self.ws:
                await self.ws.close()
        except Exception:
            pass
        self.ws = None

        # aiohttp session 종료
        try:
            if self.session:
                await self.session.close()
        except Exception:
            pass
        self.session = None

        logger.info(f"[{self.room_id}] 송출 종료")


async def run_single(args):
    topics = [args.topic]
    rclpy.init(args=None)
    buf = MultiCamBuffer(topics)

    executor = MultiThreadedExecutor()
    executor.add_node(buf)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        track = Ros2ImageVideoTrack(buf, topic=args.topic, label=args.label or args.room, fps=args.fps)
        bc = WebRTCBroadcaster(args.server, args.room, video_track=track)
        await bc.connect()
    finally:
        executor.shutdown()
        buf.destroy_node()
        rclpy.shutdown()


async def run_multi(args):
    topics = [t.strip() for t in args.topics.split(",") if t.strip()]
    if len(topics) < 4:
        raise RuntimeError("multi 모드에서는 --topics에 4개 토픽을 콤마로 입력해야 합니다.")

    rooms = [r.strip() for r in args.rooms.split(",") if r.strip()]
    if len(rooms) < 4:
        # 부족하면 기본 이름 채움
        default_rooms = ["amr-front", "amr-back", "amr-right", "amr-left"]
        rooms = (rooms + default_rooms)[:4]

    rclpy.init(args=None)
    buf = MultiCamBuffer(topics[:4])

    executor = MultiThreadedExecutor()
    executor.add_node(buf)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    broadcasters: List[WebRTCBroadcaster] = []
    try:
        # 4개 개별 채널
        for room, topic in zip(rooms[:4], topics[:4]):
            track = Ros2ImageVideoTrack(buf, topic=topic, label=room, fps=args.fps)
            broadcasters.append(WebRTCBroadcaster(args.server, room, video_track=track))

        # 합성 채널
        composite_track = Ros2CompositeQuadTrack(buf, list(zip(topics[:4], rooms[:4])), fps=args.fps)
        broadcasters.append(WebRTCBroadcaster(args.server, args.combined_room, video_track=composite_track))

        await asyncio.gather(*[bc.connect() for bc in broadcasters])

    finally:
        # gather가 끝나면 이미 각 bc.connect에서 cleanup이 불리지만, 혹시 남아있으면 정리
        for bc in broadcasters:
            try:
                await bc.cleanup()
            except Exception:
                pass
        executor.shutdown()
        buf.destroy_node()
        rclpy.shutdown()

async def run_select(args):
    """
    /selected_cam_id(Int32) 로 0~N-1 카메라 ID를 받아서
    해당 topic만 단일 채널로 송출하는 모드.
    """
    topics = [t.strip() for t in args.topics.split(",") if t.strip()]
    if len(topics) == 0:
        raise RuntimeError("select 모드에서는 --topics에 최소 1개 토픽이 필요합니다.")

    # ROS2 초기화 및 버퍼 노드 생성
    rclpy.init(args=None)
    buf = MultiCamBuffer(topics)

    executor = MultiThreadedExecutor()
    executor.add_node(buf)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        # 🔑 fixed_topic=None → 항상 buf.current_topic 사용
        track = Ros2ImageVideoTrack(
            buf,
            topic=None,                # ← 선택 모드 (current_topic 사용)
            label=args.room,           # 화면에 찍힐 라벨 (예: "amr-selected")
            fps=args.fps
        )
        bc = WebRTCBroadcaster(args.server, args.room, video_track=track)
        await bc.connect()
    finally:
        executor.shutdown()
        buf.destroy_node()
        rclpy.shutdown()


async def main():
    parser = argparse.ArgumentParser(description="WebRTC 영상 송출기 (ROS2)")
    parser.add_argument("--server", default="ws://localhost:8083",
                        help="시그널링 서버 URL (기본: ws://localhost:8083)")
    parser.add_argument("--fps", type=int, default=30, help="송출 FPS (기본: 30)")

    # single
    parser.add_argument("--room", default="robot-front", help="single 모드 방 ID (기본: robot-front)")
    parser.add_argument("--topic", default="/cam0/image_raw", help="single 모드 ROS Image 토픽 (기본: /cam0/image_raw)")
    parser.add_argument("--label", default="", help="single 모드 라벨(옵션). 비우면 room 사용")

    # multi
    parser.add_argument("--topics", default="/cam0/image_raw,/cam1/image_raw,/cam2/image_raw,/cam3/image_raw",
                        help="multi 모드 4개 토픽 (콤마 구분)")
    parser.add_argument("--rooms", default="amr-front,amr-back,amr-right,amr-left",
                        help="multi 모드 4개 방 이름 (콤마 구분)")
    parser.add_argument("--combined-room", default="amr-all",
                        help="합성 채널 방 이름 (기본: amr-all)")

    parser.add_argument("--mode", choices=["single", "multi", "select"], default="multi",
                    help="single: 1채널, multi: 4채널+합성 1채널, select: ID로 카메라 선택")


    args = parser.parse_args()

    print()
    print("=" * 50)
    print("  WebRTC 영상 송출기 (ROS2)")
    print("=" * 50)
    print(f"  signaling server: {args.server}")
    print(f"  mode           : {args.mode}")
    if args.mode == "single":
        print(f"  room           : {args.room}")
        print(f"  topic          : {args.topic}")
    elif args.mode == "select":
        print(f"  room           : {args.room}")
        print(f"  topics         : {args.topics}")
    else:  # multi
        print(f"  rooms          : {args.rooms}")
        print(f"  topics         : {args.topics}")
        print(f"  combined room  : {args.combined_room}")
    print("  Ctrl+C로 종료")
    print()

    try:
        if args.mode == "single":
            await run_single(args)
        elif args.mode == "multi":
            await run_multi(args)
        else:  # select 모드
            await run_select(args)
    except KeyboardInterrupt:
        print("\n종료 요청됨...")


if __name__ == "__main__":
    asyncio.run(main())