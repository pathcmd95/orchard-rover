"""노드 생성 스모크 테스트 (ROS 2 환경에서만 실행).

각 노드를 생성·파괴만 해 보고 파라미터 선언·발행자/구독자 생성 단계에서 예외가 없는지 확인한다.
rclpy/orchard_msgs/cv_bridge 가 없으면 (일반 PC 의 pytest) 건너뛴다.
"""
import importlib.util

import pytest

HAS_ROS = all(importlib.util.find_spec(m) is not None for m in ('rclpy', 'orchard_msgs', 'cv_bridge'))


@pytest.mark.skipif(not HAS_ROS, reason='rclpy/orchard_msgs/cv_bridge 없음 (ROS 환경에서만 실행)')
@pytest.mark.parametrize('module,cls', [
    ('orchard_perception.lidar_tree_node', 'LidarTreeNode'),
    ('orchard_perception.camera_tree_node', 'CameraTreeNode'),
    ('orchard_perception.tree_fusion_node', 'TreeFusionNode'),
    ('orchard_perception.seg_path_node', 'SegPathNode'),
    ('orchard_perception.seg_dataset_node', 'SegDatasetNode'),
])
def test_node_constructs(module, cls):
    """기본 파라미터로 노드를 만들고 바로 파괴: 예외 없이 끝나야 한다."""
    import importlib

    import rclpy
    rclpy.init()
    try:
        node = getattr(importlib.import_module(module), cls)()
        node.destroy_node()
    finally:
        rclpy.shutdown()
