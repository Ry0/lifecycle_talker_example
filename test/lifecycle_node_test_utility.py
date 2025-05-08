import threading
import time
import pytest
import rclpy
from lifecycle_msgs.srv import ChangeState, GetState
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rclpy.lifecycle import LifecycleNode


class LifecycleNodeTestUtility():
    def __init__(self, test_target_node: LifecycleNode, spin_once_sec=0.1):
        self.test_target_node = test_target_node
        self.test_utility_node = rclpy.create_node('test_utility_node')

        self.spin_once_sec = spin_once_sec

        # エグゼキューターとスレッドの設定
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.test_target_node)
        self.executor.add_node(self.test_utility_node)

        # スレッド終了イベント
        self.spin_stop_event = threading.Event()
        # エグゼキューターを別スレッドで実行
        self.executor_thread = threading.Thread(target=self._spin, daemon=True)
        self.executor_thread.start()

        # GetStateとChangeStateクライアントの作成
        self.get_state_client = self.test_utility_node.create_client(GetState, f'/{self.test_target_node.get_name()}/get_state')
        self.change_state_client = self.test_utility_node.create_client(ChangeState, f'/{self.test_target_node.get_name()}/change_state')

        # パラメータ設定用クライアント
        self.set_param_client = self.test_utility_node.create_client(SetParameters, f'/{self.test_target_node.get_name()}/set_parameters')

        # 各クライアントが利用可能になるまで待機
        self.wait_for_service(self.get_state_client)
        self.wait_for_service(self.change_state_client)
        self.wait_for_service(self.set_param_client)

    def finalize(self):
        self.spin_stop_event.set()
        self.executor_thread.join()

        self.executor.remove_node(self.test_utility_node)
        self.executor.remove_node(self.test_target_node)
        self.test_utility_node.destroy_node()
        self.test_target_node.destroy_node()
        self.executor.shutdown()

    def _spin(self):
        while not self.spin_stop_event.is_set():
            self.executor.spin_once(timeout_sec=self.spin_once_sec)

    def wait_for_service(self, client, timeout=5.0):
        """サービスが利用可能になるまで待機する関数"""
        start_time = time.time()
        while not client.wait_for_service(timeout_sec=1.0):
            if time.time() - start_time > timeout:
                pytest.fail(f'Service {client.srv_name} not available within timeout')
            time.sleep(0.1)

    def get_current_state(self):
        """現在の状態を取得する関数"""
        req = GetState.Request()
        future = self.get_state_client.call_async(req)

        # 結果を待機
        rclpy.spin_until_future_complete(self.test_utility_node, future, timeout_sec=10.0)

        if future.done():
            return future.result().current_state.id
        else:
            pytest.fail('Failed to get current state')

    def change_state(self, transition_id):
        """状態を変更する関数"""
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = self.change_state_client.call_async(req)

        # 結果を待機
        rclpy.spin_until_future_complete(self.test_utility_node, future, timeout_sec=10.0)

        if future.done():
            return future.result().success
        else:
            pytest.fail('Failed to change state')

    def set_parameters(self, param_name, param_value, param_type):
        """パラメータを設定する関数"""
        req = SetParameters.Request()
        parameter = Parameter()
        parameter.name = param_name
        parameter.value.type = param_type

        if param_type == ParameterType.PARAMETER_STRING:
            parameter.value.string_value = param_value
        elif param_type == ParameterType.PARAMETER_DOUBLE:
            parameter.value.double_value = param_value
        else:
            raise NotImplementedError()

        req.parameters = [parameter]
        future = self.set_param_client.call_async(req)

        # 結果を待機
        rclpy.spin_until_future_complete(self.test_utility_node, future, timeout_sec=10.0)

        if future.done():
            return future.result().results[0].successful
        else:
            pytest.fail('Failed to set parameters')

    def initialize_subscriber(self, msg_type, topic_name):
        # メッセージのサブスクライバーを作成
        self.messages = []

        def message_callback(msg):
            self.messages.append(msg)

        self.test_utility_node.create_subscription(
            msg_type,
            topic_name,
            message_callback,
            10)

    def wait_messages(self, timeout=10.0):
        start_time = time.time()
        while True:
            if time.time() - start_time > timeout:
                pytest.fail('Could not subscribe within timeout')
            if len(self.messages) > 0:
                break
            time.sleep(1.0)
