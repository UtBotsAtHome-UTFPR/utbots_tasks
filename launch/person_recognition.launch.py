from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory

import os
home_dir = os.path.expanduser("~")

def generate_launch_description():
    # Paths to other launch files
    nav_launch_path = os.path.join(
        get_package_share_directory('utbots_nav'), 'launch', 'nav.launch.py')

    recognition_launch_path = os.path.join(
        get_package_share_directory('utbots_face_recognition'), 'launch', 'recognition.launch.py')

    stt_launch_dir = os.path.join(
        get_package_share_directory('vad_ros'), 'launch')
    
    realsense_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch')
    
    mediapipe_launch_path = os.path.join(
        get_package_share_directory('mediapipe_track'), 'launch', "mediapipe_node.launch.py"
    )
    
    verbose = LaunchConfiguration('verbose',default="false")

    return LaunchDescription([
 
        # Include utbots_nav launch file with arguments
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch_path),
            launch_arguments={
                'use_sim_time': 'false',
                'use_imu': 'false',
                'map': f'{home_dir}/ros2_ws/src/utbots_navigation/utbots_nav/map/cbr2025v2.yaml'
            }.items()
        ),

        # Include face recognition launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(recognition_launch_path)
        ),


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
            output='screen',
            parameters=[{
                'camera_topic': '/camera/camera/color/image_raw'
            }]
        ),

        # Launch usb_cam_node_exe with parameter
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(realsense_launch_path, 'rs_launch.py')),
            #launch_arguments={}
        ),
        
        Node(
            package='utbots_nlu',
            executable='rasa_nlu_interpreter',
            name='rasa_nlu_interpreter',
            # output='screen',
            emulate_tty=True,
            parameters=[
                {
                    # 'model_path':
                    f'{home_dir}/ros2_ws/src/utbots_nlu/rasa/models/20251015-143654-decidable-liqueur.tar.gz',
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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mediapipe_launch_path),
            launch_arguments={
                'rgb_topic': "/camera/camera/color/image_raw"   # <-- new value here
            }.items()
        ),
    ])
