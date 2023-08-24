from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
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
        respeaker_node,
        sound_play_node,
        speech_to_text_node,
        # LogInfo(
        #     condition=IfCondition(LaunchConfiguration('publish_tf')),
        #     msg='Static transform publisher node will be launched.'
        # ),
        LogInfo(
            condition=IfCondition(LaunchConfiguration('launch_soundplay')),
            msg='Sound play node will be launched.'
        ),
    ])