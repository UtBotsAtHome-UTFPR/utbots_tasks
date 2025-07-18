from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Declare the launch argument
    declared_camera_topic = DeclareLaunchArgument(
        'camera_topic',
        default_value='/image_raw',
        description='Camera topic to subscribe to'
    )

    camera_topic = LaunchConfiguration('camera_topic')

    realsense_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch')

    return LaunchDescription([
        declared_camera_topic,

        # YOLO Node 1
        Node(
            package='yolov8_ros',
            executable='yolo_node',
            name='yolo_node_coco',
            namespace='yolo_node_coco',
            output='screen',
            parameters=[{
                'camera_topic': camera_topic
            }]
        ),

        # YOLO Node 2 with custom weights
        Node(
            package='yolov8_ros',
            executable='yolo_node',
            name='yolo_node_home',
            namespace='yolo_node_home',
            output='screen',
            parameters=[{
                'camera_topic': camera_topic,
                'weights': '/home/laser/Downloads/best_drinks.pt'
            }]
        ),

        # Conditionally include RealSense launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(realsense_launch_path, 'rs_launch.py')
            ),
            condition=LaunchConfigurationEquals('camera_topic', '/camera/camera/color/image_raw')
        ),

        # Conditionally launch usb_cam if topic is /image_raw
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': '/dev/video2',
                'framerate': 30.0,
                'io_method': 'mmap',
                'frame_id': 'camera',
                'pixel_format': 'mjpeg2rgb',
                'av_device_format': 'YUV422P',
                'image_width': 1280,
                'image_height': 720,
                'camera_name': 'test_camera',
                'camera_info_url': 'file:///home/laser/.ros/camera_info/default_cam.yaml',
                'brightness': -1,
                'contrast': -1,
                'saturation': -1,
                'sharpness': -1,
                'gain': -1,
                'auto_white_balance': True,
                'white_balance': 4000,
                'autoexposure': True,
                'exposure': 100,
                'autofocus': False,
                'focus': -1
            }]
        ),

        Node(
            package='utbots_tasks',
            executable='basic_vision',
            name='basic_vision',
        )
    ])
