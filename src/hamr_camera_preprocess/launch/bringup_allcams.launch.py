from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # 1) 카메라 노드들
    cam_front = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        namespace="cam_front",
        name="v4l2_front",
        parameters=[{
            "video_device": "/dev/video0",
            "output_encoding": "rgb8",
            "image_size": [640, 480],
        }]
    )

    cam_right = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        namespace="cam_right",
        name="v4l2_back",
        parameters=[{
            "video_device": "/dev/video1",
            "output_encoding": "rgb8",
            "image_size": [640, 480],
        }]
    )

    cam_back = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        namespace="cam_back",
        name="v4l2_left",
        parameters=[{
            "video_device": "/dev/video2",
            "output_encoding": "rgb8",
            "image_size": [640, 480],
        }]
    )

    cam_left = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        namespace="cam_left",
        name="v4l2_right",
        parameters=[{
            "video_device": "/dev/video3",
            "output_encoding": "rgb8",
            "image_size": [640, 480],
        }]
    )

    rect_front = Node(
        package="hamr_camera_preprocess",
        executable="rectifier",   
        namespace="cam_front",
        name="rectifier_front",
        parameters=[{
            "input_topic": "image_raw",
            "output_topic": "image_rect",
            "show": False,   # headless 환경이면 False
        }],
    )

    rect_left = Node(
        package="hamr_camera_preprocess",
        executable="rectifier",
        namespace="cam_left",
        name="rectifier_back",
        parameters=[{
            "input_topic": "image_raw",
            "output_topic": "image_rect",
            "show": False,
        }],
    )

    rect_back = Node(
        package="hamr_camera_preprocess",
        executable="rectifier",
        namespace="cam_back",
        name="rectifier_left",
        parameters=[{
            "input_topic": "image_raw",
            "output_topic": "image_rect",
            "show": False,
        }],
    )

    rect_right = Node(
        package="hamr_camera_preprocess",
        executable="rectifier",
        namespace="cam_right",
        name="rectifier_right",
        parameters=[{
            "input_topic": "image_raw",
            "output_topic": "image_rect",
            "show": False,
        }],
    )

    # Camera ID Select
    # python3 broadcaster_ros2.py   --mode select   --server ws://192.168.20.3:8083   --topics /cam_front/image_rect,/cam_back/image_rect,/cam_left/image_rect,/cam_right/image_rect   --room amr-selected
    # ID 선택은 위 명령어에서 토픽 순서에 따름
    # 즉, 0 -> front,,, 3 -> right
    # camera_ID = Node(
    #     package="hamr_camera_preprocess",
    #     executable="camera_selector_node",
    #     namespace="camera_ID",
    #     name="camera_selector_node",
    # )

    return LaunchDescription([
        cam_front, cam_back, cam_left, cam_right,
        rect_front, rect_back, rect_left, rect_right,
        # camera_ID,
    ])
