from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node

def generate_launch_description():
    # パラメータ用のLaunch引数
    message_arg = DeclareLaunchArgument(
        'message',
        default_value='Hello from manual launch',
        description='Message to publish'
    )
    
    frequency_arg = DeclareLaunchArgument(
        'publish_frequency',
        default_value='1.0',
        description='Publishing frequency in Hz'
    )
    
    # ライフサイクルノードの設定（自動遷移なし）
    lifecycle_talker = LifecycleNode(
        package='lifecycle_talker_example',
        executable='lifecycle_talker',
        name='my_lifecycle_node',
        namespace='',
        parameters=[{
            'message': LaunchConfiguration('message'),
            'publish_frequency': LaunchConfiguration('publish_frequency')
        }],
        output='screen'
    )
    
    # 通常のサブスクライバー
    lifecycle_subscriber = Node(
        package='lifecycle_talker_example',
        executable='lifecycle_subscriber',
        name='lifecycle_subscriber',
        output='screen'
    )
    
    return LaunchDescription([
        message_arg,
        frequency_arg,
        lifecycle_talker,
        lifecycle_subscriber
    ])
