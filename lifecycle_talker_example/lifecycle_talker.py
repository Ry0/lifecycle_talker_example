#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.lifecycle import State
from std_msgs.msg import String

from rcl_interfaces.msg import SetParametersResult
from lifecycle_msgs.msg import State as msgState

import time
# import threading


class MyLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('my_lifecycle_node')
        self.get_logger().info('Lifecycle Node constructor called')
        
        # デフォルトパラメータを宣言
        self.declare_parameter('message', 'Hello World')
        self.declare_parameter('publish_frequency', 1.0)
        
        # 変数の初期化
        self.timer = None
        self.message = ''
        self.publish_frequency = 1.0
        self.count = 0
        
        # ライフサイクル対応パブリッシャーの作成（最初は無効状態）
        self.publisher = self.create_lifecycle_publisher(String, 'lifecycle_topic', 10)
        # パラメータ変更コールバックを追加
        self.add_on_set_parameters_callback(self.parameters_callback)


    def parameters_callback(self, params):
        result = SetParametersResult()
        result.successful = True
        state_id, _ = self.get_current_state()
        if state_id != msgState.PRIMARY_STATE_UNCONFIGURED:
            # INACTIVE状態以外では変更を拒否
            self.get_logger().warn('Can change parameter while in uncofigured state. Please change uncofigured state.')
            result.successful = False
            result.reason = 'Can change parameter while in uncofigured state'
                
        return result

    def get_current_state(self):
        state_id, label = self._state_machine.current_state
        return state_id, label

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        try:
            self.get_logger().info('on_configure() called')
            
            # パラメータを取得して内部変数に設定
            self.message = self.get_parameter('message').get_parameter_value().string_value
            self.publish_frequency = self.get_parameter('publish_frequency').get_parameter_value().double_value
            
            self.get_logger().info(f'Configured with message: "{self.message}" and frequency: {self.publish_frequency}Hz')
            
            # 設定が成功したことを返す
            return TransitionCallbackReturn.SUCCESS
        except Exception as ex:
            self.get_logger().info(f'Publisher configure error: {ex}')
            return TransitionCallbackReturn.ERROR

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        try:
            self.get_logger().info('on_activate() called')
            
            # パブリッシャーをアクティブ化
            self.publisher.on_activate(state)
            
            # タイマーを設定してパブリッシュを開始
            self.timer = self.create_timer(1.0 / self.publish_frequency, self.timer_callback)
            self.get_logger().info(f'Publisher activated with frequency: {self.publish_frequency}Hz')
            
            loop_cnt = 10
            for i in range(loop_cnt):
                time.sleep(1)
                self.get_logger().info(f'Dummy wait: {i+1} / {loop_cnt} [s]')

            return TransitionCallbackReturn.SUCCESS
        except Exception as ex:
            self.get_logger().info(f'Publisher activated error: {ex}')
            return TransitionCallbackReturn.ERROR

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        try:
            self.get_logger().info('on_deactivate() called')
            
            # タイマーを停止
            if self.timer:
                self.destroy_timer(self.timer)
                self.timer = None
                
            # パブリッシャーを非アクティブ化
            if self.publisher:
                self.publisher.on_deactivate(state)
                self.publisher = None
            self.get_logger().info('Publisher deactivated')
            
            return TransitionCallbackReturn.SUCCESS
        except Exception as ex:
            self.get_logger().info(f'Publisher deactivate error: {ex}')
            return TransitionCallbackReturn.ERROR
        
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        try:
            self.get_logger().info('on_cleanup() called')
            
            # リソースの解放
            if self.timer:
                self.destroy_timer(self.timer)
                self.timer = None
                
            # パラメータをリセット
            self.message = ''
            self.publish_frequency = 1.0
            self.count = 0
            
            self.get_logger().info('Resources cleaned up')
            return TransitionCallbackReturn.SUCCESS
        except Exception as ex:
            self.get_logger().info(f'Publisher cleanup error: {ex}')
            return TransitionCallbackReturn.ERROR

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        try:
            self.get_logger().info('on_shutdown() called')
            # on_deactivate(state)
            # on_cleanup(state)
            self._shutdown()
            # shutdown_thread = threading.Thread(target=self._shutdown)
            # shutdown_thread.daemon = True 
            # shutdown_thread.start()

            # シャットダウン処理
            return TransitionCallbackReturn.SUCCESS
        except Exception as ex:
            self.get_logger().info(f'Publisher shutdown error: {ex}')
            return TransitionCallbackReturn.ERROR

    def _shutdown(self):
        self.get_logger().info('Call executor.shutdown')

        if self.executor:
            self.executor.shutdown()

    def timer_callback(self):
        msg = String()
        self.count += 1
        msg.data = f'{self.message} (count: {self.count})'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    
    lifecycle_node = MyLifecycleNode()
    
    lifecycle_node.get_logger().info('Lifecycle node initialized. Waiting for lifecycle state transitions...')
    
    executor = rclpy.executors.SingleThreadedExecutor()
    # executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(lifecycle_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        lifecycle_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
