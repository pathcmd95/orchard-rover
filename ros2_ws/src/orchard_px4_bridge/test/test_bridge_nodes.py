"""노드 생성 스모크 테스트 (ROS 2 + px4_msgs 환경에서만 실행).

rclpy/px4_msgs 가 없는 환경(일반 pytest)에서는 건너뛴다. PX4 가 떠 있지 않아도 생성만 확인한다.
"""
import importlib.util

import pytest

HAS_ROS = all(importlib.util.find_spec(m) is not None for m in ('rclpy', 'px4_msgs'))


@pytest.mark.skipif(not HAS_ROS, reason='rclpy/px4_msgs 없음 (ROS 환경에서만 실행)')
@pytest.mark.parametrize('module,cls', [
    ('orchard_px4_bridge.offboard_node', 'OffboardBridge'),
    ('orchard_px4_bridge.state_node', 'Px4StateNode'),
])
def test_node_constructs(module, cls):
    """노드 클래스가 파라미터 선언·토픽 연결까지 예외 없이 생성되고 정리되는지."""
    import importlib

    import rclpy
    rclpy.init()
    try:
        node = getattr(importlib.import_module(module), cls)()
        node.destroy_node()
    finally:
        rclpy.shutdown()
