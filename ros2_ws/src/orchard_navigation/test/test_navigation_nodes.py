"""노드 생성 스모크 테스트: ROS 2 환경(colcon test)에서 노드 생성자가 예외 없이 끝나는지 확인.
(생성자 오류는 Gazebo 를 띄워야 드러나므로 미리 잡는다)"""
import importlib.util
import json
import os

import numpy as np
import pytest

HAS_ROS = all(importlib.util.find_spec(m) is not None for m in ('rclpy', 'orchard_msgs'))


@pytest.mark.skipif(not HAS_ROS, reason='rclpy/orchard_msgs 없음 (ROS 환경에서만 실행)')
def test_row_navigator_node_constructs():
    """기본 파라미터로 RowNavigatorNode 생성 → 예외 없음, U턴 반경 > 0."""
    import rclpy
    from orchard_navigation.row_navigator_node import RowNavigatorNode
    rclpy.init()
    try:
        node = RowNavigatorNode()
        assert node.mission.radius > 0
        node.destroy_node()
    finally:
        rclpy.shutdown()


@pytest.mark.skipif(not HAS_ROS, reason='rclpy/orchard_msgs 없음 (ROS 환경에서만 실행)')
def test_orchard_mapper_node_constructs(tmp_path):
    """OrchardMapperNode 생성 → 나무 없으면 저장 안 함, 요 0.2 rad 틀어진 직진 odom + 나무 두 줄
    → 방위 오차 0.2 rad 추정·보정 후 2열 지도 저장."""
    import rclpy
    from rclpy.parameter import Parameter

    from orchard_navigation.orchard_mapper_node import OrchardMapperNode
    rclpy.init()
    try:
        node = OrchardMapperNode()
        node.set_parameters([Parameter('output_dir', value=str(tmp_path))])
        assert node.save() is None          # 나무가 없으면 저장하지 않음
        # odom(요가 0.2 rad 틀어진 직진) + 옆 나무 두 줄 → 방위 보정 후 지도 저장까지
        import math

        from builtin_interfaces.msg import Time as TimeMsg
        from nav_msgs.msg import Odometry
        from orchard_msgs.msg import Tree, TreeArray
        bias = 0.2
        for k in range(120):
            t = 0.1 * k
            o = Odometry()
            o.header.stamp = TimeMsg(sec=int(t), nanosec=int((t % 1) * 1e9))
            o.pose.pose.position.x = 0.5 * t
            o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(-bias / 2), math.cos(-bias / 2)
            node.on_odom(o)
            if k % 5 == 0 and k > 0:
                msg = TreeArray()
                msg.header.stamp = o.header.stamp
                msg.header.frame_id = 'base_link'
                for tx in np.arange(0, 12, 1.2):
                    for ty in (-1.9, 1.9):
                        wx, wy = tx - 0.5 * t, ty                         # 실제 로버 기준 (요 0)
                        if math.hypot(wx, wy) < 7:
                            tr = Tree()
                            tr.position.x, tr.position.y, tr.radius, tr.confidence = wx, wy, 0.08, 1.0
                            msg.trees.append(tr)
                node.on_trees(msg)
        assert abs(node.applied_bias - bias) < 0.02
        path = node.save()
        assert path is not None
        summary = json.load(open(os.path.join(path, 'summary.json')))
        assert len(summary['rows']) == 2
        node.destroy_node()
    finally:
        rclpy.shutdown()
