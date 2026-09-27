"""headland.py (행 끝 U턴 계획) 단위 시험.

Dubins 경로가 목표 자세에 도달하는지, 반경 = 열 간격/2 이면 반원이 되는지, 줄기로 다음 통로를 찾는지
(두 열 / 가까운 열만 / 못 봄), PathTracker 가 회전 미끄러짐에서도 경로를 따라가는지 확인한다.
"""
import math

import numpy as np

from orchard_navigation.headland import PathTracker, dubins_path, estimate_next_lane, path_length


def test_dubins_reaches_goal():
    """임의 시작·목표 자세 200 쌍 → 경로 끝이 목표 위치 2 cm, 요 0.02 rad 안."""
    rng = np.random.default_rng(0)
    for _ in range(200):
        s = (0.0, 0.0, rng.uniform(-3, 3))
        g = (rng.uniform(-6, 6), rng.uniform(-6, 6), rng.uniform(-3, 3))
        P = dubins_path(s, g, 1.3)
        assert math.hypot(P[-1, 0] - g[0], P[-1, 1] - g[1]) < 0.02
        assert abs(math.atan2(math.sin(P[-1, 2] - g[2]), math.cos(P[-1, 2] - g[2]))) < 0.02


def test_uturn_path_is_semicircle_when_radius_is_half_spacing():
    """3.8 m 옆·반대 방향 목표, 반경 1.9 m → 길이 π·1.9 m 반원, 헤드랜드로 반경 넘게 나가지 않음."""
    P = dubins_path((0.0, 0.0, 0.0), (0.0, 3.8, math.pi), 1.9)
    assert abs(path_length(P) - math.pi * 1.9) < 0.05
    assert P[:, 0].max() < 1.95                        # 헤드랜드로 반경 이상 나가지 않음


def test_next_lane_from_two_rows_left_and_right():
    """좌·우 U턴 모두: 두 열(간격 3.4 m)을 보면 'lidar2', 통로 중심 ±3.4 m, 간격 실측 3.4 m, 입구 = 먼 쪽 끝 나무."""
    xs = np.arange(-10, 0.01, 1.2)
    for sign in (1.0, -1.0):
        trees = np.vstack([np.c_[xs, np.full_like(xs, sign * 1.7)],          # 가까운 열 (간격 3.4)
                           np.c_[xs - 0.6, np.full_like(xs, sign * 5.1)],    # 그 너머 열, 0.6 m 짧음
                           np.c_[xs, np.full_like(xs, -sign * 1.7)]])        # 반대편 열 (무시)
        nl = estimate_next_lane(trees, sign, row_spacing=3.8)
        assert nl.source == 'lidar2'
        assert abs(nl.v - sign * 3.4) < 0.05 and abs(nl.spacing - 3.4) < 0.05
        assert abs(nl.u_end - xs.max()) < 0.05          # 두 열 중 더 바깥의 마지막 나무


def test_next_lane_falls_back_when_far_row_missing():
    """가까운 열만 보이면 'lidar1' (중심 = 열 + 간격/2), 줄기가 하나도 없으면 'nominal'."""
    xs = np.arange(-10, 0.01, 1.2)
    nl = estimate_next_lane(np.c_[xs, np.full_like(xs, 1.9)], 1.0, 3.8)
    assert nl.source == 'lidar1' and abs(nl.v - 3.8) < 0.05 and nl.far_count == 0
    assert estimate_next_lane(np.zeros((0, 2)), -1.0, 3.8).source == 'nominal'


def test_tracker_follows_path_under_slip():
    """U턴 경로를 회전이 20 % 덜 되는 조건에서 추종 → 끝에서 y 오차 < 0.15 m, 경로 이탈 < 0.3 m."""
    P = dubins_path((0.0, 0.0, 0.0), (0.0, 3.4, math.pi), 1.7)
    tr = PathTracker(P, lookahead=1.0, max_curvature=1 / 0.9)
    x, y, yaw, dt = 0.0, 0.0, 0.0, 0.05
    for _ in range(2000):
        v, w = tr.command(x, y, yaw, 0.5)
        w *= 0.8                                           # 회전이 20 % 덜 됨
        x, y, yaw = x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt, yaw + w * dt
        if tr.remaining < 0.15:
            break
    assert abs(y - 3.4) < 0.15 and tr.cross_track < 0.3
