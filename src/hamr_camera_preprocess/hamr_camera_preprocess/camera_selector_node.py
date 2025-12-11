#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import threading


class CameraSelectorNode(Node):
    """
    터미널에서 입력한 숫자를 /selected_cam_id 로 publish하는 노드.
    - 숫자 입력: 해당 ID를 publish (그 뒤로는 ImageToGstUdpNode가 그 ID를 계속 사용)
    - 'q', 'quit', 'exit' 입력: 노드 종료
    """

    def __init__(self):
        super().__init__('camera_selector')

        self.pub = self.create_publisher(Int32, 'selected_cam_id', 10)

        # 입력을 읽는 별도 스레드 시작
        input_thread = threading.Thread(target=self.input_loop, daemon=True)
        input_thread.start()

        self.get_logger().info(
            "CameraSelectorNode started.\n"
            " - 카메라 ID(정수)를 입력하면 /selected_cam_id 로 publish 됩니다.\n"
            " - 'q' 또는 'quit' 또는 'exit' 를 입력하면 노드가 종료됩니다."
        )

    def input_loop(self):
        """
        터미널 입력을 blocking 없이 계속 읽어서 publish
        """
        while rclpy.ok():
            try:
                user_input = input("Enter camera ID (or 'q' to quit): ").strip()

                # 종료 명령어 처리
                if user_input.lower() in ("q", "quit", "exit"):
                    print("종료 명령어 입력됨: CameraSelectorNode 종료합니다.")
                    self.get_logger().info("Shutdown requested from terminal input.")
                    # rclpy.shutdown() 호출 → main의 rclpy.spin()이 빠져나오게 됨
                    rclpy.shutdown()
                    break

                # 빈 입력은 무시
                if user_input == "":
                    print("아무 값도 입력되지 않았습니다.")
                    continue

                # 숫자 여부 확인
                if not user_input.isdigit():
                    print("숫자(ID) 또는 'q'만 입력하세요.")
                    continue

                cam_id = int(user_input)
                msg = Int32()
                msg.data = cam_id
                self.pub.publish(msg)

                self.get_logger().info(f"Published selected_cam_id={cam_id}")

            except Exception as e:
                print(f"Input error: {e}")
                break


def main(args=None):
    rclpy.init(args=args)
    node = CameraSelectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # rclpy.shutdown() 이 이미 호출되었을 수도 있으니 그냥 정리만
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
