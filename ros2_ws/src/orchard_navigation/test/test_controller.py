"""controller.py (Pure Pursuit 행 추종 명령) 단위 시험.

중심선 쪽으로 도는 부호, 중심선 위 직진 속도, 장애물 정지/감속, 최소 회전반경에 의한 곡률 제한을 확인한다.
"""
import math

from orchard_navigation.controller import (FollowParams, obstacle_speed_factor, pure_pursuit_curvature,
                                           row_follow_command)


def test_turns_toward_centerline():
    """중심선이 왼쪽(offset +0.5 m)이면 w > 0 (좌회전), 오른쪽이면 w < 0."""
    p = FollowParams()
    v, w = row_follow_command(offset=0.5, heading=0.0, confidence=1.0, obstacle_distance=math.inf, p=p)
    assert v > 0 and w > 0          # 중심선이 왼쪽 → 좌회전(+)
    v, w = row_follow_command(offset=-0.5, heading=0.0, confidence=1.0, obstacle_distance=math.inf, p=p)
    assert w < 0


def test_on_centerline_goes_straight():
    """중심선 위·방향 일치·신뢰도 1 → w = 0, v = cruise_speed (0.8 m/s)."""
    v, w = row_follow_command(0.0, 0.0, 1.0, math.inf, FollowParams())
    assert abs(w) < 1e-9 and abs(v - 0.8) < 1e-9


def test_obstacle_stops_and_slows():
    """장애물이 정지 거리(1.2 m) 안이면 배율 0 · v = 0, 감속 구간(2.0 m)이면 0 < 배율 < 1."""
    p = FollowParams(stop_distance=1.2, slow_distance=3.0)
    assert obstacle_speed_factor(1.0, 1.2, 3.0) == 0.0
    assert 0.0 < obstacle_speed_factor(2.0, 1.2, 3.0) < 1.0
    v, _ = row_follow_command(0.0, 0.0, 1.0, 1.0, p)
    assert v == 0.0


def test_curvature_limited_by_min_turn_radius():
    """최소 회전반경 2 m → 명령 곡률 w/v 가 0.5 1/m 이하로 잘린다 (제한 전 곡률은 더 큼)."""
    p = FollowParams(min_turn_radius=2.0, max_yaw_rate=10.0, lookahead=1.0)
    v, w = row_follow_command(1.8, 0.0, 1.0, math.inf, p)
    assert abs(w / v) <= 0.5 + 1e-9
    assert pure_pursuit_curvature(1.8, 0.0, 1.0) > 0.5   # 제한 전 곡률은 더 크다
