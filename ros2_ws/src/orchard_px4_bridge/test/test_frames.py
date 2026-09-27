"""frames.py 좌표 변환 단위 테스트 (ROS 없이 실행).

PX4 NED/FRD ↔ ROS ENU/FLU (REP-103) 변환에서 요 기준(북=0 ↔ 동=0), 회전 방향 부호,
회전행렬의 직교성, cmd_vel → NED 속도 setpoint 변환을 확인한다.
"""
import math

import numpy as np

from orchard_px4_bridge.frames import (attitude_ned_frd_to_enu_flu, frd_to_flu, ned_to_enu,
                                       ros_yaw_rate_to_px4)


def _q_yaw_ned(yaw):
    """NED 요 yaw [rad] 만 있는 PX4 자세 쿼터니언 (w, x, y, z): z 축 회전 = (cos ψ/2, 0, 0, sin ψ/2)."""
    return (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))   # w, x, y, z


def _yaw_enu(q_xyzw):
    """ROS 쿼터니언 (x, y, z, w) 의 ENU 요 [rad] (동=0, 반시계 +)."""
    x, y, z, w = q_xyzw
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def test_heading_north_is_enu_90deg():
    """북쪽을 보는 기체(NED 요 0)는 ENU 요 +90° 여야 한다."""
    q, _ = attitude_ned_frd_to_enu_flu(_q_yaw_ned(0.0))
    assert abs(_yaw_enu(q) - math.pi / 2) < 1e-9


def test_heading_east_is_enu_0deg():
    """동쪽을 보는 기체(NED 요 +90°)는 ENU 요 0 이어야 한다."""
    q, _ = attitude_ned_frd_to_enu_flu(_q_yaw_ned(math.pi / 2))
    assert abs(_yaw_enu(q)) < 1e-9


def test_position_and_body_vectors():
    """위치 NED→ENU (x/y 교환, z 부호 반전), 기체 FRD→FLU (y, z 부호 반전)."""
    assert np.allclose(ned_to_enu([1.0, 2.0, -3.0]), [2.0, 1.0, 3.0])
    assert np.allclose(frd_to_flu([1.0, 2.0, 3.0]), [1.0, -2.0, -3.0])


def test_left_turn_is_negative_px4_yaw_rate():
    """ROS 좌회전(+z) 요레이트는 PX4 NED 에서 음수(반시계)."""
    assert ros_yaw_rate_to_px4(0.5) == -0.5


def test_rotation_is_orthonormal():
    """임의 자세를 변환한 R_enu_flu 가 회전행렬(R Rᵀ = I, det = +1)인지."""
    q = (0.9, 0.1, -0.2, 0.37)
    n = math.sqrt(sum(v * v for v in q))
    _, r = attitude_ned_frd_to_enu_flu(tuple(v / n for v in q))
    assert np.allclose(r @ r.T, np.eye(3)) and abs(np.linalg.det(r) - 1) < 1e-9


def test_yaw_from_quat():
    """쿼터니언에서 뽑은 NED 요가 만든 요와 같은지 (±π 범위 포함)."""
    from orchard_px4_bridge.frames import yaw_from_quat_wxyz
    for yaw in (0.0, 0.7, -2.0, 3.0):
        assert abs(yaw_from_quat_wxyz(_q_yaw_ned(yaw)) - yaw) < 1e-9


def test_cmd_vel_to_ned_velocity():
    """cmd_vel → NED 속도: 직진 방향, 좌회전 시 목표 요 감소, 속도 크기 유지, 후진은 0."""
    from orchard_px4_bridge.frames import cmd_vel_to_ned_velocity
    # 동쪽(NED 요 +90°)을 보고 직진 0.8 m/s → 동쪽 속도
    vn, ve = cmd_vel_to_ned_velocity(0.8, 0.0, math.pi / 2, 1 / 3)
    assert abs(vn) < 1e-9 and abs(ve - 0.8) < 1e-9
    # 좌회전(ROS +z) 요청 → NED 목표 요는 현재보다 작아진다(반시계)
    vn, ve = cmd_vel_to_ned_velocity(0.5, 0.6, 0.0, 1 / 3)
    assert abs(math.hypot(vn, ve) - 0.5) < 1e-9
    assert abs(math.atan2(ve, vn) - (-0.2)) < 1e-9
    # 후진 요청은 정지
    assert cmd_vel_to_ned_velocity(-0.3, 0.0, 0.0, 0.3) == (0.0, 0.0)
