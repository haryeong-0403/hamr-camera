from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import yaml


def launch_setup(context, *args, **kwargs):
    def load_composable_node_param(param_path):
        with open(LaunchConfiguration(param_path).perform(context), "r") as f:
            return yaml.safe_load(f)["/**"]["ros__parameters"]

    composable_nodes = [
        ComposableNode(
            package="v4l2_camera",
            plugin="v4l2_camera::V4L2Camera",
            name=['v4l2_camera_', LaunchConfiguration("camera_name")],
            namespace=LaunchConfiguration("v4l2_camera_namespace"),
            remappings=[
                # raw 이미지 remap
                (
                    "image_raw",
                    [
                        LaunchConfiguration("camera_name"),
                        '/',
                        LaunchConfiguration("image_topic"),
                    ],
                ),
                # compressed 이미지 remap
                (
                    "image_raw/compressed",
                    [
                        LaunchConfiguration("camera_name"),
                        '/',
                        LaunchConfiguration("image_topic"),
                        '/compressed',
                    ],
                ),
                # 카메라 정보 remap
                (
                    "camera_info",
                    [
                        LaunchConfiguration("camera_name"),
                        '/camera_info'
                    ],
                ),
            ],
            parameters=[
                # load_composable_node_param("v4l2_camera_param_path"),
                # load_composable_node_param("rate_diagnostics_param_path"),
                {
                    "camera_info_url": LaunchConfiguration("camera_info_url"),
                    "use_sensor_data_qos": LaunchConfiguration("use_sensor_data_qos"),
                    "publish_rate": LaunchConfiguration("publish_rate"),
                    "use_v4l2_buffer_timestamps": LaunchConfiguration("use_v4l2_buffer_timestamps"),
                    "use_image_transport": LaunchConfiguration("use_image_transport"),
                    "hardware_id": LaunchConfiguration("hardware_id"),

                    # 런치 인자에서 반영
                    "video_device": LaunchConfiguration("video_device"),
                    "pixel_format": LaunchConfiguration("pixel_format"),
                    "output_encoding": LaunchConfiguration("output_encoding"),
                    "image_size": LaunchConfiguration("image_size"),

                },
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        ),
    ]

    v4l2_camera_container = ComposableNodeContainer(
        condition=LaunchConfigurationEquals('container', ''),
        name=['v4l2_camera_', LaunchConfiguration('camera_name'), '_container'],
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=composable_nodes,
        output='screen',
    )

    load_composable_nodes = LoadComposableNodes(
        condition=LaunchConfigurationNotEquals('container', ''),
        composable_node_descriptions=composable_nodes,
        target_container=LaunchConfiguration('container'),
    )

    return [v4l2_camera_container, load_composable_nodes]


def generate_launch_description():

    launch_arguments = []

    def add_arg(name: str, default_value=None, description=None):
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    # 기본 인자들
    add_arg('container', '', 'container name to load composable nodes into it.')
    add_arg('image_topic', 'image_raw', 'image topic name to be published')
    add_arg('camera_name', 'cam1', 'prefix to be added to the head of topic name')
    add_arg('v4l2_camera_namespace', '/sensing/camera', 'namespace in which the nodes launched')
    add_arg('v4l2_camera_param_path', '', 'path to yaml file for v4l2_camera node')
    add_arg('rate_diagnostics_param_path', '', 'path to yaml file for rate diagnostics')
    add_arg('camera_info_url', '', 'url to yaml file that contains camera intrinsic parameters')
    add_arg('use_intra_process', 'False', 'flag to use ROS2 intra process')
    add_arg('use_sensor_data_qos', 'False', 'flag to use sensor data QoS')
    add_arg('publish_rate', "30.0", 'publish frame number per second')
    add_arg('use_v4l2_buffer_timestamps', 'true', 'flag to use v4l2 buffer timestamps')
    add_arg('use_image_transport', 'true', 'flag to launch image_transport node')
    add_arg('hardware_id', 'camera1', 'hardware id of the camera')

    # 👇 하령이가 직접 쓰던 값들
    add_arg('video_device', '/dev/video2', 'Video device path')
    add_arg('pixel_format', 'UYVY', 'Pixel format')
    add_arg('output_encoding', 'rgb8', 'Output encoding')
    add_arg('image_size', '[640,480]', 'Width and height of the image')

    return LaunchDescription(
        [
            *launch_arguments,
            OpaqueFunction(function=launch_setup),
        ]
    )
