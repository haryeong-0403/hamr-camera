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

    cam_back = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        namespace="cam_back",
        name="v4l2_back",
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
        name="v4l2_left",
        parameters=[{
            "video_device": "/dev/video4",
            "output_encoding": "rgb8",
            "image_size": [640, 480],
        }]
    )

    cam_right = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        namespace="cam_right",
        name="v4l2_right",
        parameters=[{
            "video_device": "/dev/video6",
            "output_encoding": "rgb8",
            "image_size": [640, 480],
        }]
    )

    # 2) rectifier 노드들

    rect_front = Node(
        package="hamr_camera_preprocess",
        executable="frontcam_rectifier",
        namespace="cam_front",     
        name="rectifier_front",
        # output="screen",
    )

    rect_back = Node(
        package="hamr_camera_preprocess",
        executable="frontcam_rectifier",
        namespace="cam_back",
        name="rectifier_back",
        # output="screen",
    )

    rect_left = Node(
        package="hamr_camera_preprocess",
        executable="frontcam_rectifier",
        namespace="cam_left",
        name="rectifier_left",
        # output="screen",
    )

    rect_right = Node(
        package="hamr_camera_preprocess",
        executable="frontcam_rectifier",
        namespace="cam_right",
        name="rectifier_right",
        # output="screen",
    )

    # 3. Select Cam ID
    CamID = Node(
        package="hamr_camera_preprocess",
        executable="camera_selector_node",
        namespace="camera_selector_node",
        name="camera_selector_node",
        output="screen"

    )

    # 4. GStream 
    gstream = Node(
        package="hamr_camera_preprocess",
        executable="gstreamer",
        namespace="Gstreamer",
        name="gstream_node",
        # output="screen"

    )

    return LaunchDescription([
        cam_front, cam_back, cam_left, cam_right,
        rect_front, rect_back, rect_left, rect_right,
        gstream, CamID
    ])
