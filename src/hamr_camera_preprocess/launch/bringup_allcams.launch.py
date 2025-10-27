from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    cam1 = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        namespace="cam1",
        parameters=[{
            "video_device": "/dev/video0",   # 필요시 변경
            "output_encoding": "rgb8",
            "image_size": [640, 480],
        }]
        # 기본적으로 /cam1/image_raw 와 /cam1/image_raw/compressed 가 제공됨
    )

    rectifier = Node(
        package="hamr_camera_preprocess",
        executable="frontcam_rectifier",
        output="screen"
    )

    return LaunchDescription([cam1, rectifier])
