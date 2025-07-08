from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
import os

def generate_launch_description():
    # Paths to other launch files
    nav_launch_path = os.path.join(
        '/home/laser/ros2_ws/src/utbots_navigation/utbots_nav/launch',
        'nav.launch.py'
    )

    recognition_launch_path = os.path.join(
        '/home/laser/ros2_ws/src/utbots_vision/utbots_face_recognition/launch',
        'recognition.launch.py'
    )

    return LaunchDescription([
        # Include utbots_nav launch file with arguments
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch_path),
            launch_arguments={
                'use_sim_time': 'false',
                'use_imu': 'false',
                'map': '/home/laser/ros2_ws/src/utbots_navigation/utbots_nav/map/pitaco.yaml'
            }.items()
        ),

        # Include face recognition launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(recognition_launch_path)
        ),

        # Launch yolov8_ros yolo_node
        Node(
            package='yolov8_ros',
            executable='yolo_node',
            name='yolo_node',
            output='screen'
        ),

        # Launch usb_cam_node_exe with parameter
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': '/dev/video2'
            }]
        ),

        # # Launch receptionist node
        # Node(
        #     package='utbots_tasks',
        #     executable='receptionist',
        #     name='receptionist',
        #     output='screen'
        # )
    ])
