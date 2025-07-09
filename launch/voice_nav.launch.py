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
    #GLOBAL
    verbose = LaunchConfiguration('verbose',default="false")#.perform(context)
    #VAD
    vad_timeout = int(LaunchConfiguration('vad_timeout',default='3_000').perform(context))
    vad_threshold = float(LaunchConfiguration('vad_threshold',default='0.5').perform(context))
    disable_denoiser = LaunchConfiguration('disable_denoiser',default=False)#.perform(context)
    #WHISPER
    whisper_startup = LaunchConfiguration('whisper_startup',default='true')#.perform(context)
    whisper_stt_timeout = float(LaunchConfiguration('whisper_stt_timeout',default='15.0').perform(context))
    whisper_def_model = LaunchConfiguration('whisper_def_model',default='openai/whisper-large-v3-turbo').perform(context)
    #NEED TO BE EVALUATED:


    

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
            launch_arguments={
                'verbose': verbose,
                'vad_timeout':vad_timeout,
                'vad_threshold':vad_threshold,
                'disable_denoiser':disable_denoiser,
                'whisper_startup':whisper_startup,
                'whisper_sync_start':'false',
                'whisper_stt_timeout':whisper_stt_timeout,
                'whisper_def_model':whisper_def_model,
                # 'whisper_cb_timer':'0.1',
                }.items() 
            ),
    ])
def generate_launch_description():
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

        #OpaqueFunction(function=launch_setup),
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