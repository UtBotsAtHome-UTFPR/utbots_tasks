from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory

import os
home_directory = os.environ['HOME']

def generate_launch_description():
    # Paths to other launch files
    # nav_launch_path = os.path.join(
    #     get_package_share_directory('utbots_nav'), 'launch', 'nav.launch.py')

    stt_launch_dir = os.path.join(
        get_package_share_directory('vad_ros'), 'launch')
    
    verbose = LaunchConfiguration('verbose',default="false")
    map_path = f'{home_directory}/ros2_ws/src/utbots_navigation/utbots_nav/map/pitaco.yaml'

    realsense_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'), 'examples', 'align_depth')

    camera_topic = LaunchConfiguration('camera_topic')

    return LaunchDescription([
 
        # Include utbots_nav launch file with arguments
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(nav_launch_path),
        #     launch_arguments={
        #         'use_sim_time': 'false',
        #         'use_imu': 'false',
        #         'map': map_path
        #     }.items()
        # ),

        # Launch yolov8_ros yolo_node
        Node(
            package='yolov8_ros',
            executable='yolo_node',
            name='yolo_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'weights': 'yolo11n.pt', #/ros2_ws/src/yolov8_ros/weights/best.pt',
                    'camera_topic': '/camera/camera/color/image_raw',
                    'device':'cuda',
                    'conf': 0.25,
                    'draw': True,
                    'target_category':'',
                    'segmentation': False,
                    'debug':False,
                    'enable_synchronous_startup':False,
                  }
            ]
        ),

        # Launch usb_cam_node_exe with parameter
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': '/dev/video2', # for laptop cam, 2 for usb cam
                'image_height': 1080,
                'image_width': 1920
            }]
        ),

        Node(
            package='ros_tts',
            executable='tts_node',
            name='tts_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                  }
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(stt_launch_dir, 'stt_launch.py')),
            launch_arguments={
                'verbose': verbose,
                'whisper_sync_start':'false',
                }.items() 
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(realsense_launch_path, 'rs_align_depth_launch.py')),
            #launch_arguments={}
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=[
                '0.0', '0.0', '0.1',   # x, y, z offset
                '0.0', '0.0', '0.0',   # roll, pitch, yaw (in radians)
                'base_footprint',       # parent frame
                'camera_link'           # child frame
            ]
        ),

    ])
