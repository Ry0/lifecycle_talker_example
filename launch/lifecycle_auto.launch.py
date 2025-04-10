from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, TimerAction
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
import lifecycle_msgs.msg

def generate_launch_description():
    # パラメータ用のLaunch引数
    message_arg = DeclareLaunchArgument(
        'message',
        default_value='Hello from lifecycle launch',
        description='Message to publish'
    )
    
    frequency_arg = DeclareLaunchArgument(
        'publish_frequency',
        default_value='2.0',
        description='Publishing frequency in Hz'
    )
    
    # ライフサイクルノードの設定
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
    
    # 通常のサブスクライバー（後で状態イベントによって起動される）
    lifecycle_subscriber = Node(
        package='lifecycle_talker_example',
        executable='lifecycle_subscriber',
        name='lifecycle_subscriber',
        output='screen'
    )
    
    # Configure遷移イベント
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(lifecycle_talker),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
        )
    )
    
    # Configure完了後にActivateを実行
    activate_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(lifecycle_talker),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
        )
    )
    
    # Configure状態への遷移が成功したらActivateを実行
    configure_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lifecycle_talker,
            goal_state='configuring',
            entities=[],
        )
    )
    
    # Configure状態からinactive状態への遷移が成功したらActivateを実行
    activate_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lifecycle_talker,
            start_state='configuring',
            goal_state='inactive',
            entities=[activate_event],
        )
    )
    
    # ライフサイクルノードがアクティブ状態になったらサブスクライバーを起動
    subscriber_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lifecycle_talker,
            start_state='activating',
            goal_state='active',
            entities=[lifecycle_subscriber],
        )
    )
    
    return LaunchDescription([
        message_arg,
        frequency_arg,
        lifecycle_talker,
        # 最初にConfigureイベントを実行
        configure_event,
        # 状態遷移ハンドラを登録
        configure_handler,
        activate_handler,
        subscriber_handler,
        # lifecycle_subscriber はここでは起動せず、アクティブ状態への遷移が完了した後に起動される
    ])
