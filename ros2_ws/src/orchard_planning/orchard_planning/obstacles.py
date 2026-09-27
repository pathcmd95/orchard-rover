"""LiDAR 한 스캔 → 지역 계획용 장애물 점 — ROS 의존성 없음.

로버가 부딪힐 수 있는 것 = 지면 위 h_min ~ h_max 높이에 있는 물체 (잔디보다 높고, 차체보다 낮은 것).
처진 가지·수관처럼 차체 위로 지나가는 것은 장애물이 아니다 (h_max = 차체 높이 + 여유).

지면 높이 = RANSAC 평면 (orchard_perception.ground, lidar_tree_node 와 같은 방법) + 칸별 보정
  1) 스캔 전체에서 지면 평면을 RANSAC 으로 맞춘다 (nominal_ground_z 근처 점만 후보)
  2) xy 를 ground_cell [m] 칸으로 나눠 칸마다 가장 낮은 점 = 그 칸 지면 후보.
     평면과 local_tol 안이면 그 값을 (작은 둔덕·고랑 반영), 아니면 평면을 쓴다.
  왜 평면이 필요한가: Mid-360 은 아래로 −7° 까지만 보여 센서 둘레 약 5 m 안은 지면이 안 찍힌다.
  그 칸에서는 '가장 낮은 점' 이 줄기 중간(0.3~0.4 m)이라, 그걸 지면으로 쓰면 수관 아랫부분이 장애물로 잘못 잡힌다.

입력: base_link 점군 (N×3) [m]. 출력: base_link 장애물 xy (M×2), 2D 격자(grid [m])로 중복 제거.
"""
from __future__ import annotations

import numpy as np
from orchard_perception.ground import fit_ground_plane

_OFF = 1 << 20


def _cell_keys(ij: np.ndarray) -> np.ndarray:
    """칸 번호 (N×2) → 정수 키."""
    return (ij[:, 0].astype(np.int64) + _OFF) * (1 << 22) + (ij[:, 1].astype(np.int64) + _OFF)


def height_above_ground(P: np.ndarray, nominal_ground_z: float = -0.1, ground_cell: float = 0.5,
                        local_tol: float = 0.15) -> np.ndarray:
    """점마다 지면 위 높이 [m] (N,). P: N×3 (base_link), nominal_ground_z: 평지에서 base_link 기준 지면 z."""
    plane = fit_ground_plane(P, nominal_ground_z)
    h_plane = plane.height(P)
    ij = np.floor(P[:, :2] / ground_cell).astype(np.int64)
    keys = _cell_keys(ij)
    uk, inv = np.unique(keys, return_inverse=True)
    gmin = np.full(len(uk), np.inf)
    np.minimum.at(gmin, inv, h_plane)                      # 칸별 가장 낮은 점의 '평면 위 높이'
    g = gmin[inv]
    local = np.abs(g) < local_tol                          # 칸 지면이 평면과 가까우면 칸 지면 사용
    return np.where(local, h_plane - g, h_plane)


def obstacle_points(pts: np.ndarray, h_min: float = 0.2, h_max: float = 0.75, max_range: float = 7.0,
                    self_box=(-0.7, 0.7, 0.45), nominal_ground_z: float = -0.1, ground_cell: float = 0.5,
                    grid: float = 0.15) -> np.ndarray:
    """base_link 점군 → 장애물 xy (M×2).

    h_min / h_max: 장애물로 볼 지면 위 높이 [m], max_range: 수평 거리 한계 [m],
    self_box: (x_min, x_max, |y|_max) 차체 자기 점 제거 상자 [m], nominal_ground_z: 평지 지면 z (base_link) [m],
    grid: 결과 중복 제거 격자 [m].
    """
    P = np.asarray(pts, float).reshape(-1, 3)
    P = P[np.isfinite(P).all(1)]
    r = np.hypot(P[:, 0], P[:, 1])
    x0, x1, yb = self_box
    P = P[(r < max_range + ground_cell) & ~((P[:, 0] > x0) & (P[:, 0] < x1) & (np.abs(P[:, 1]) < yb))]
    if not len(P):
        return np.zeros((0, 2))
    hag = height_above_ground(P, nominal_ground_z, ground_cell)
    m = (hag >= h_min) & (hag <= h_max) & (np.hypot(P[:, 0], P[:, 1]) < max_range)
    xy = P[m, :2]
    if not len(xy):
        return np.zeros((0, 2))
    k = np.unique(np.floor(xy / grid).astype(np.int64), axis=0)
    return (k + 0.5) * grid
