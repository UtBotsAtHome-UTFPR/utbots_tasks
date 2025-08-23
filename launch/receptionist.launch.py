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
    
    # realsense_launch_path = os.path.join(
    #     get_package_share_directory('realsense2_camera'), 'launch')
    
    verbose = LaunchConfiguration('verbose',default="false")

    return LaunchDescription([
 
        # Include utbots_nav launch file with arguments
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch_path),
            launch_arguments={
                'use_sim_time': 'false',
                'use_imu': 'false',
                'map': f'{home_dir}/ros2_ws/src/utbots_navigation/utbots_nav/map/pitaco.yaml'
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

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(realsense_launch_path, 'rs_launch.py')),
        #     #launch_arguments={}
        # ),

        # Launch yolov8_ros yolo_node
        # YOLO node for Robot 1
        Node(
            package='yolov8_ros',
            executable='yolo_node',
            name='yolo_node1',
            namespace='yolo_node1',
            output='screen',
            parameters=[{
                'camera_topic': '/image_raw',
                'weights': f'{home_dir}/ros2_ws/src/yolo11n.pt'
            }]
        ),

        # YOLO node for Robot 2
        Node(
            package='yolov8_ros',
            executable='yolo_node',
            name='yolo_node2',
            namespace='yolo_node2',
            output='screen',
            parameters=[{
                'camera_topic': '/image_raw',
                'weights': f'{home_dir}/Downloads/best_drinks.pt'
            }]
        ),

        # Launch usb_cam_node_exe with parameter
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
                'camera_info_url': f'file://{home_dir}/.ros/camera_info/default_cam.yaml',
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
            package='utbots_nlu',
            executable='rasa_nlu_interpreter',
            name='rasa_nlu_interpreter',
            # output='screen',
            emulate_tty=True,
            parameters=[
                {
                    # 'model_path':
                    # f'{home_dir}/ros2_ws/src/utbots_nlu/rasa/models/20250716-120427-brass-queue.tar.gz',
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

        # #Launch receptionist node
        # Node(
        #     package='utbots_tasks',
        #     executable='receptionist',
        #     name='receptionist',
        #     output='screen'
        # )
    ])
