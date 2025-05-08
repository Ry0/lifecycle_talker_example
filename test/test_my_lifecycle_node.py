import pytest
import rclpy
import time
from lifecycle_msgs.msg import State, Transition
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterType

from lifecycle_talker_example.lifecycle_talker import MyLifecycleNode

from .lifecycle_node_test_utility import LifecycleNodeTestUtility


@pytest.fixture(scope='module')
def rclpy_init():
    """ROSの初期化と終了を行うモジュールスコープのフィクスチャ"""
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def lifecycle_node_setup(rclpy_init):
    lifecycle_node = MyLifecycleNode()

    util = LifecycleNodeTestUtility(lifecycle_node, 0.1)

    yield util

    util.finalize()


def test_lifecycle_transitions(lifecycle_node_setup):
    """ライフサイクルの各状態間の遷移をテスト"""
    lifecycle_node_setup.initialize_subscriber(String, 'lifecycle_topic')

    # 初期状態の確認
    state = lifecycle_node_setup.get_current_state()
    assert state == State.PRIMARY_STATE_UNCONFIGURED

    # Configure遷移
    success = lifecycle_node_setup.change_state(Transition.TRANSITION_CONFIGURE)
    assert success is True
    state = lifecycle_node_setup.get_current_state()
    assert state == State.PRIMARY_STATE_INACTIVE

    # Activate遷移
    success = lifecycle_node_setup.change_state(Transition.TRANSITION_ACTIVATE)
    assert success is True
    state = lifecycle_node_setup.get_current_state()
    assert state == State.PRIMARY_STATE_ACTIVE

    # メッセージの発行を待機
    lifecycle_node_setup.wait_messages()

    # メッセージが発行されていることを確認
    assert any('Hello World' in msg.data for msg in lifecycle_node_setup.messages)

    # Deactivate遷移
    success = lifecycle_node_setup.change_state(Transition.TRANSITION_DEACTIVATE)
    assert success is True
    state = lifecycle_node_setup.get_current_state()
    assert state == State.PRIMARY_STATE_INACTIVE

    # Cleanup遷移
    success = lifecycle_node_setup.change_state(Transition.TRANSITION_CLEANUP)
    assert success is True
    state = lifecycle_node_setup.get_current_state()
    assert state == State.PRIMARY_STATE_UNCONFIGURED


def test_parameters(lifecycle_node_setup):
    """パラメータ設定と動作のテスト"""
    lifecycle_node_setup.initialize_subscriber(String, 'lifecycle_topic')

    # Unconfigured状態でパラメータを設定
    result = lifecycle_node_setup.set_parameters('message',
                                                 'Custom Message',
                                                 ParameterType.PARAMETER_STRING)
    assert result is True

    result = lifecycle_node_setup.set_parameters('publish_frequency',
                                                 2.0,
                                                 ParameterType.PARAMETER_DOUBLE)
    assert result is True

    # Configure遷移
    success = lifecycle_node_setup.change_state(Transition.TRANSITION_CONFIGURE)
    assert success is True

    # Inactive状態でのパラメータ設定は失敗する
    result = lifecycle_node_setup.set_parameters('message',
                                                 'Another Message',
                                                 ParameterType.PARAMETER_STRING)
    assert result is False

    # Activate遷移
    success = lifecycle_node_setup.change_state(Transition.TRANSITION_ACTIVATE)
    assert success is True
    state = lifecycle_node_setup.get_current_state()
    assert state == State.PRIMARY_STATE_ACTIVE

    # メッセージの発行を待機
    lifecycle_node_setup.wait_messages()

    # カスタムメッセージが発行されていることを確認
    assert any('Custom Message' in msg.data for msg in lifecycle_node_setup.messages)

    # Active状態でのパラメータ設定は失敗する
    result = lifecycle_node_setup.set_parameters('message',
                                                 'Another Message',
                                                 ParameterType.PARAMETER_STRING)
    assert result is False

    # Deactivate遷移
    success = lifecycle_node_setup.change_state(Transition.TRANSITION_DEACTIVATE)
    assert success is True
    state = lifecycle_node_setup.get_current_state()
    assert state == State.PRIMARY_STATE_INACTIVE

    # Inactive状態でのパラメータ設定は失敗する
    result = lifecycle_node_setup.set_parameters('message',
                                                 'Another Message',
                                                 ParameterType.PARAMETER_STRING)
    assert result is False

    # Cleanup遷移
    success = lifecycle_node_setup.change_state(Transition.TRANSITION_CLEANUP)
    assert success is True
    state = lifecycle_node_setup.get_current_state()
    assert state == State.PRIMARY_STATE_UNCONFIGURED

    # Unconfigured状態でのパラメータ設定は成功する
    result = lifecycle_node_setup.set_parameters('message',
                                                 'Another Message',
                                                 ParameterType.PARAMETER_STRING)
    assert result is True


def test_shutdown_unconfigured_shutdown(lifecycle_node_setup):
    """シャットダウン処理のテスト"""
    # Unconfigured状態を確認
    state = lifecycle_node_setup.get_current_state()
    assert state == State.PRIMARY_STATE_UNCONFIGURED

    # Shutdown遷移
    success = lifecycle_node_setup.change_state(Transition.TRANSITION_UNCONFIGURED_SHUTDOWN)
    assert success is True

    # Shutdown確認
    while True:
        if lifecycle_node_setup.test_target_node.is_shutdown:
            break
        time.sleep(1.0)

    lifecycle_node_setup.executor.remove_node(lifecycle_node_setup.test_utility_node)
    lifecycle_node_setup.test_utility_node.destroy_node()

    if len(lifecycle_node_setup.executor.get_nodes()) != 0:
        pytest.fail('executor has some nodes.')


def test_shutdown_inactive_shutdown(lifecycle_node_setup):
    """シャットダウン処理のテスト"""
    # Inactive状態を確認
    lifecycle_node_setup.change_state(Transition.TRANSITION_CONFIGURE)
    state = lifecycle_node_setup.get_current_state()
    assert state == State.PRIMARY_STATE_INACTIVE

    # Shutdown遷移
    success = lifecycle_node_setup.change_state(Transition.TRANSITION_INACTIVE_SHUTDOWN)
    assert success is True

    # Shutdown確認
    while True:
        if lifecycle_node_setup.test_target_node.is_shutdown:
            break
        time.sleep(1.0)

    lifecycle_node_setup.executor.remove_node(lifecycle_node_setup.test_utility_node)
    lifecycle_node_setup.test_utility_node.destroy_node()

    if len(lifecycle_node_setup.executor.get_nodes()) != 0:
        pytest.fail('executor has some nodes.')


def test_shutdown_active_shutdown(lifecycle_node_setup):
    """シャットダウン処理のテスト"""
    # Configure & Activate
    lifecycle_node_setup.change_state(Transition.TRANSITION_CONFIGURE)
    lifecycle_node_setup.change_state(Transition.TRANSITION_ACTIVATE)

    # Active状態を確認
    state = lifecycle_node_setup.get_current_state()
    assert state == State.PRIMARY_STATE_ACTIVE

    # Shutdown遷移
    success = lifecycle_node_setup.change_state(Transition.TRANSITION_ACTIVE_SHUTDOWN)
    assert success is True

    # Shutdown確認
    while True:
        if lifecycle_node_setup.test_target_node.is_shutdown:
            break
        time.sleep(1.0)

    lifecycle_node_setup.executor.remove_node(lifecycle_node_setup.test_utility_node)
    lifecycle_node_setup.test_utility_node.destroy_node()

    if len(lifecycle_node_setup.executor.get_nodes()) != 0:
        pytest.fail('executor has some nodes.')
