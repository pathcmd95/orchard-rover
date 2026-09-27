"""voxel_map_node 스모크 시험 (ROS 2 환경에서만): 생성, 원점 설정, 스캔 누적, 저장까지 예외 없이 동작하는지."""
import importlib.util
import json
import math
import os

import numpy as np
import pytest

HAS_ROS = all(importlib.util.find_spec(m) is not None for m in ('rclpy', 'orchard_msgs', 'vision_msgs'))


@pytest.mark.skipif(not HAS_ROS, reason='rclpy/orchard_msgs/vision_msgs 없음 (ROS 환경에서만 실행)')
def test_voxel_map_node_origin_integrate_save(tmp_path):
    """미션 시작 → 2 m 직진(odom) → 원점·방향 확정 → 스캔 누적 → 저장 파일 생성."""
    import rclpy
    from builtin_interfaces.msg import Time as TimeMsg
    from nav_msgs.msg import Odometry
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2 as pc2
    from std_msgs.msg import Header, String

    from orchard_mapping.voxel_map_node import VoxelMapNode
    rclpy.init()
    try:
        node = VoxelMapNode()
        # +x 축은 처음 달린 방향으로 (기본 row_frame 은 전역 계획의 /orchard/row_frame 을 기다려서 이 시험에는 안 맞음)
        node.set_parameters([Parameter('output_dir', value=str(tmp_path)), Parameter('axis_source', value='course')])
        node.out_dir = os.path.join(str(tmp_path), 'run')
        node.static_cache['livox_frame'] = (np.eye(3), np.zeros(3))   # TF 없이 센서 = base_link 로 가정

        def odom(t, x):
            o = Odometry()
            o.header.stamp = TimeMsg(sec=int(t), nanosec=int((t % 1) * 1e9))
            o.pose.pose.position.x = x
            o.pose.pose.orientation.w = 1.0
            return o
        node.on_odom(odom(0.0, 0.0))
        node.on_state(String(data='FOLLOW_ROW|lanes_done=0|시작|row=lidar'))
        for k in range(1, 30):
            node.on_odom(odom(0.1 * k, 0.1 * k))
        assert node.frame.ready and abs(node.frame.yaw0) < 1e-6
        pole = [[3.0, 1.0, z] for z in np.arange(-0.1, 0.8, 0.1)]
        ground = [[x, y, -0.1] for x in range(1, 6) for y in range(-2, 3)]
        pts = np.array(pole + ground, np.float32)
        cloud = pc2.create_cloud_xyz32(Header(frame_id='livox_frame', stamp=TimeMsg(sec=2, nanosec=0)), pts)
        assert isinstance(cloud, PointCloud2)
        node.on_cloud(cloud)
        assert len(node.vm) > 0
        out = node.save()
        meta = json.load(open(os.path.join(out, 'meta.json')))
        assert meta['voxels_saved'] >= 0 and math.isclose(meta['x_axis_odom_heading_deg'], 0.0, abs_tol=1e-6)
        for f in ('voxels.ply', 'voxel_map.npz', 'trunks.csv', 'trajectory.csv'):
            assert os.path.exists(os.path.join(out, f))
        node.destroy_node()
    finally:
        rclpy.shutdown()
