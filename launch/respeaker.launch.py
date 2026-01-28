from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Publish tf of respeaker'
    )

    launch_soundplay_arg = DeclareLaunchArgument(
        'launch_soundplay',
        default_value='true',
        description='Launch sound_play node'
    )

    language_arg = DeclareLaunchArgument(
        'language',
        default_value='en-US',
        description='Language used in speech_to_text.py'
    )

    self_cancellation_arg = DeclareLaunchArgument(
        'self_cancellation',
        default_value='true',
        description='Self cancellation means halting speech_to_text while the robot is playing sound'
    )

    static_transformer_node = Node(
        condition=IfCondition(LaunchConfiguration('publish_tf')),
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'respeaker_base', '100']
    )

    # Stop PulseAudio user services (daemon + socket activation) to avoid ALSA device contention.
    # Some desktop/audio applications can cause PulseAudio to open and hold the ReSpeaker capture PCM device,
    # which may lead to respeaker_node failing during initialization (e.g., reporting 0 channels).
    stop_pulseaudio = ExecuteProcess(
        cmd=[
            'bash', '-lc',
            'systemctl --user stop pulseaudio.socket pulseaudio.service 2>/dev/null || true'
        ],
        output='screen'
    )

    start_pulseaudio = ExecuteProcess(
        cmd=[
            'bash', '-lc',
            'systemctl --user start pulseaudio.socket pulseaudio.service 2>/dev/null || true'
        ],
        output='screen'
    )

    respeaker_node = Node(
        package='respeaker_ros2',
        executable='respeaker_node',
        output='screen'
    )

    sound_play_node = Node(
        condition=IfCondition(LaunchConfiguration('launch_soundplay')),
        package='sound_play',
        executable='soundplay_node.py'
    )

    speech_to_text_node = Node(
        package='respeaker_ros2',
        executable='speech_to_text',
        parameters=[{
            'language': LaunchConfiguration('language'),
            'self_cancellation': LaunchConfiguration('self_cancellation'),
            'tts_tolerance': 0.5
        }]
    )

    return LaunchDescription([
        # publish_tf_arg,
        launch_soundplay_arg,
        language_arg,
        self_cancellation_arg,
        # static_transformer_node,
        # Ensure PulseAudio can't claim the capture device before respeaker_node starts
        stop_pulseaudio,
        respeaker_node,
        sound_play_node,
        speech_to_text_node,
        # Restart pulse audio
        start_pulseaudio,
        # LogInfo(
        #     condition=IfCondition(LaunchConfiguration('publish_tf')),
        #     msg='Static transform publisher node will be launched.'
        # ),
        LogInfo(
            condition=IfCondition(LaunchConfiguration('launch_soundplay')),
            msg='Sound play node will be launched.'
        ),
    ])