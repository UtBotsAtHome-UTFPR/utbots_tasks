from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import OpaqueFunction

def launch_setup(context, *args, **kwargs):
    sim = LaunchConfiguration('use_sim').perform(context)
    map_name = LaunchConfiguration('map_name').perform(context) if sim=='false' else 'world'
    lidar_port = LaunchConfiguration('lidar_port').perform(context)

    

    launch_dir_nav_utils = os.path.join(
        get_package_share_directory('utbots_nav_utils'), 'launch')
    
    launch_dir_stt = os.path.join(
        get_package_share_directory('vad_ros'), 'launch')

    return([
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir_nav_utils, 'wp_world.launch.py')),
            launch_arguments={'map_name': map_name}.items() 
        ) if sim=='true' else  \
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir_nav_utils, 'wp_hestia.launch.py')),
            launch_arguments={'map_name': map_name,
                              'lidar_port':lidar_port}.items() 
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir_stt, 'stt_launch.py')),
            # launch_arguments={'verbose': 'false'}.items() 
        ),
    ])
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true',description=''),
        DeclareLaunchArgument('map_name',default_value='',description='pitaco') ,
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0',description=''),

        OpaqueFunction(function=launch_setup),
        Node(
            package='utbots_tasks',
            executable='voice_nav_smach',
            name='voice_nav_smach',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                  }
            ]
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

    ])