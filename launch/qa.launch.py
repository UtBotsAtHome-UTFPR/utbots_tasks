from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    stt_launch_dir = os.path.join(
        get_package_share_directory('vad_ros'), 'launch')
    
    verbose = LaunchConfiguration('verbose',default="false")

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true',description=''),
        DeclareLaunchArgument('map_name',default_value='',description='pitaco') ,
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0',description=''),
        
        DeclareLaunchArgument('verbose',default_value="false"),
        #VAD,
        DeclareLaunchArgument('vad_timeout',default_value='3_000'),
        DeclareLaunchArgument('vad_threshold',default_value='0.5'),
        DeclareLaunchArgument('disable_denoiser',default_value='false'),
        #WHISPER,
        DeclareLaunchArgument('whisper_sync_start',default_value='false'),
        DeclareLaunchArgument('whisper_stt_timeout',default_value='15.0'),
        DeclareLaunchArgument('whisper_def_model',default_value='openai/whisper-large-v3-turbo'),    
            
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(stt_launch_dir, 'stt_launch.py')),
            launch_arguments={
                'verbose': verbose,
                'whisper_sync_start':'false',
                }.items() 
        ),

        Node(
            package='utbots_nlu',
            executable='rasa_nlu_interpreter',
            name='rasa_nlu_interpreter',
            # output='screen',
            emulate_tty=True,
            prefix=['bash -c "source /home/joao/nlu_env/bin/activate && $0 $@ " exec'],
            parameters=[
                {
                  }
            ]
        ),
        Node(
            package='utbots_llm',
            executable = 'llama_server',
            name='llama_server',
            emulate_tty=True,
            prefix=['bash -c "source /home/joao/llm_env/bin/activate && $0 $@ " exec'],
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

