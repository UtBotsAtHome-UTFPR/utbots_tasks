from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # Paths to other launch files
    nav_launch_path = os.path.join(
        get_package_share_directory('utbots_nav'), 'launch')

    recognition_launch_path = os.path.join(
        get_package_share_directory('utbots_face_recognition'), 'launch')

    stt_launch_dir = os.path.join(
        get_package_share_directory('vad_ros'), 'launch')
    
    verbose = LaunchConfiguration('verbose',default="false")

    return LaunchDescription([
 

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(stt_launch_dir, 'stt_launch.py')),
            launch_arguments={
                'verbose': verbose,
                'whisper_sync_start':'false',
                }.items() 
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
                'video_device': '/dev/video0'
            }]
        ),
        
        Node(
            package='utbots_nlu',
            executable='rasa_nlu_interpreter',
            name='rasa_nlu_interpreter',
            # output='screen',
            emulate_tty=True,
            parameters=[
                {
                  }
            ]
        ),
        Node(
            package='ros_tts',
            executable='tts_node',
            name='tts_node',
            # output='screen',
            emulate_tty=True,
            parameters=[
                {
                  }
            ]
        ),

        # # Launch receptionist node
        # Node(
        #     package='utbots_tasks',
        #     executable='receptionist',
        #     name='receptionist',
        #     output='screen'
        # )
    ])
