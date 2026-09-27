"""Gazebo 없이 알고리즘을 시험하기 위한 간이 과수원 LiDAR 점군 생성기.

- Mid-360 처럼 수직 -7° ~ +52° 범위만 보인다.
- 줄기(원기둥), 수관(줄기 시작 높이 위), 잔디(0~15 cm), 지면을 만든다.
- 차체 롤/피치(요철에 의한 기울어짐)를 넣을 수 있다.
단위테스트와 폐루프(closed-loop) 주행 시험에서 사용한다. 가림(occlusion)은 무시한다.

좌표계·단위
  월드: x = 행 방향(0 ~ row_length), y = 행에 수직, z = 지면(0) 위 [m].
  열 j 는 y = (j - 0.5)·row_spacing 에 있어, 통로 k 의 중심은 y = k·row_spacing (lane_y).
  scan() 출력: base_link 기준 Nx3 [m] (x 전방 / y 왼쪽 / z 위), base_link 는 지면 위 base_height.
  → lidar_tree_node 가 TF 변환 후 받는 점군과 같은 형태이므로 ground/trunks/rows 를 그대로 시험할 수 있다.
조정 가능한 값: SyntheticOrchard 필드(행간·주간 거리, 결주 확률 등), scan() 인자(자세, 잔디 밀도, 잡음).
실제 Mid-360 의 비반복 스캔 패턴은 흉내 내지 않고, 거리에 따른 점 밀도 변화만 근사한다.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .geometry import rpy_to_matrix


@dataclass
class SyntheticOrchard:
    """가상 과수원 배치. 생성 시 나무 위치 self.trees (Mx2, 월드 xy [m]) 를 만든다."""

    rows: int = 4                  # 과수열 개수
    row_spacing: float = 3.8       # 행간거리 [m]
    tree_spacing: float = 1.2      # 주간거리(같은 열 나무 사이) [m]
    row_length: float = 30.0       # 열 길이 [m]
    trunk_radius: float = 0.07     # 줄기 반경 [m]
    canopy_base: float = 0.85      # 수관(잎) 시작 높이 [m] — 이 아래가 줄기만 보이는 구간
    missing_prob: float = 0.03     # 결주(나무 빠짐) 확률
    seed: int = 1                  # 배치 난수 seed (같으면 같은 과수원)

    def __post_init__(self):
        """열·주간 격자에 나무를 배치 (결주 확률만큼 빼고, 위치에 σ 3 cm 잡음)."""
        rnd = np.random.default_rng(self.seed)
        trees = []
        for j in range(self.rows):
            y = (j - 0.5) * self.row_spacing
            for x in np.arange(0.0, self.row_length + 1e-6, self.tree_spacing):
                if rnd.random() < self.missing_prob:
                    continue
                trees.append((x + rnd.normal(0, 0.03), y + rnd.normal(0, 0.03)))
        self.trees = np.array(trees, dtype=float).reshape(-1, 2)

    def lane_y(self, k: int) -> float:
        """k 번째 통로 중심선의 월드 y [m] (통로 0 은 y = 0, 열 0 과 열 1 사이)."""
        return k * self.row_spacing


def scan(orchard: SyntheticOrchard, x: float, y: float, yaw: float, sensor_height: float = 0.6,
         base_height: float = 0.1, sensor_x: float = 0.3, roll: float = 0.0, pitch: float = 0.0,
         max_range: float = 15.0, grass_density: float = 3.0, noise: float = 0.02,
         rng: np.random.Generator | None = None) -> np.ndarray:
    """base_link 기준 Nx3 점군을 반환. sensor_height 는 지면 기준 센서 높이.

    x, y, yaw: 로봇(base_link) 의 월드 위치 [m] · 방향 [rad]
    sensor_height: 지면 기준 센서 높이 [m], base_height: 지면 기준 base_link 높이 [m]
    sensor_x: base_link 기준 센서의 전방 위치 [m], roll/pitch: 차체 기울기 [rad]
    max_range: 최대 거리 [m], grass_density: 센서 주변 반경 8 m 잔디 점 밀도 [점/m²], noise: 점 위치 잡음 σ [m]
    rng: 난수 생성기 (재현성 필요 시 전달)
    근사: 센서 위치·시야 계산은 기울기를 무시하고 수평 기준으로 한 뒤, 마지막에 base_link 로 회전만 적용.
    """
    rng = rng or np.random.default_rng()
    # Mid-360 수직 시야 -7° ~ +52° (제조사 사양). 점의 (높이차 / 수평거리) 가 이 범위여야 보임
    tan_min = np.tan(np.radians(-7.0))
    tan_max = np.tan(np.radians(52.0))
    c, s = np.cos(yaw), np.sin(yaw)
    sx, sy = x + c * sensor_x, y + s * sensor_x      # 센서 월드 위치
    pts = []

    rel = orchard.trees - [sx, sy]
    d = np.hypot(rel[:, 0], rel[:, 1])
    for (tx, ty), dist in zip(orchard.trees[d < max_range], d[d < max_range]):
        if dist < 0.3:                  # 센서와 거의 겹친 나무는 무시
            continue
        # 줄기: 수평 각해상도 0.5° 가정 → 줄기 폭(2r)에 들어가는 빔 수 (최소 2). 멀수록 점이 줄어듦
        ang_res = np.radians(0.5)
        n_ang = max(2, int(2 * orchard.trunk_radius / (dist * ang_res)))
        face = np.arctan2(sy - ty, sx - tx)          # 줄기 중심에서 센서를 향한 방향
        angs = face + np.linspace(-1.2, 1.2, n_ang)  # 센서 쪽 반원 (±1.2 rad ≈ ±69°) 만 보임
        # 수직 점 간격: 수직 각해상도 약 0.94° 를 거리로 환산 (최소 3 cm)
        z_step = max(0.03, dist * np.radians(0.94))
        zs = np.arange(0.0, orchard.canopy_base, z_step)
        dz = zs - sensor_height
        zs = zs[(dz / dist >= tan_min) & (dz / dist <= tan_max)]    # 수직 시야 안 높이만
        if zs.size == 0:
            continue
        aa, zz = np.meshgrid(angs, zs)
        px = tx + orchard.trunk_radius * np.cos(aa)
        py = ty + orchard.trunk_radius * np.sin(aa)
        pts.append(np.c_[px.ravel(), py.ravel(), zz.ravel()])
        # 수관: 줄기 시작 높이 위 타원체 표면 일부
        # 수관 점 수도 거리에 반비례 (10~200 점), 반경 0.4~0.6 m, 높이 canopy_base ~ 2.8 m
        n_can = int(np.clip(400 / max(dist, 1.0), 10, 200))
        th = rng.uniform(0, 2 * np.pi, n_can)
        hz = rng.uniform(orchard.canopy_base, 2.8, n_can)
        rad = rng.uniform(0.4, 0.6, n_can)
        pts.append(np.c_[tx + rad * np.cos(th), ty + rad * np.sin(th), hz])

    # 잔디: 센서 주변 원형 영역, 높이 0~15 cm
    area = np.pi * 8.0 ** 2              # 반경 8 m 원
    n_g = int(area * grass_density)
    r = 8.0 * np.sqrt(rng.random(n_g))   # sqrt: 원 안에 면적 기준으로 균일하게 뿌리기
    th = rng.uniform(0, 2 * np.pi, n_g)
    gx, gy = sx + r * np.cos(th), sy + r * np.sin(th)
    gz = rng.uniform(0.0, 0.15, n_g) * (rng.random(n_g) < 0.4)   # 40% 만 풀잎(0~15 cm), 나머지는 바닥(z=0)
    grass = np.c_[gx, gy, gz]
    dz = grass[:, 2] - sensor_height
    vis = dz / np.maximum(r, 0.1) >= tan_min        # 수직 시야 아래쪽 한계보다 위만 보임
    pts.append(grass[vis])

    # 지면: 센서에서 보이는 거리(수직 -7°)부터 링 형태
    r_min = sensor_height / -tan_min     # -7° 빔이 지면에 닿는 거리 (센서 0.6 m 이면 약 4.9 m)
    rr = np.arange(r_min, max_range, 0.25)                     # 0.25 m 간격 링
    th = np.linspace(-np.pi, np.pi, 360, endpoint=False)       # 1° 간격 방위
    R, T = np.meshgrid(rr, th)
    pts.append(np.c_[sx + (R * np.cos(T)).ravel(), sy + (R * np.sin(T)).ravel(), np.zeros(R.size)])

    world = np.vstack(pts) if pts else np.zeros((0, 3))
    world = world + rng.normal(0, noise, world.shape)      # 거리 측정 잡음 (등방성 가우시안)
    # 월드 → base_link (차체 롤/피치 포함)
    rot = rpy_to_matrix(roll, pitch, yaw)
    base_pos = np.array([x, y, base_height])
    # (p - base) @ R 은 행벡터 표기로 R^T (p - base) = 월드 → 차체 좌표 역변환
    return (world - base_pos) @ rot
