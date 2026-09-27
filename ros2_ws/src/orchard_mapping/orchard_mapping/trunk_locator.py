"""YOLO 줄기 박스 → 복셀 맵에서 줄기 위치 찾기 — ROS 의존성 없음.

순서 (사용자 요청: YOLO 인식 → 복셀 맵 위치 확인 → 매칭 → 상대좌표)
  1) bbox_bearing(): 영상의 줄기 박스 가운데 열(u)을 카메라 광선으로 바꿔 수평 방위각(bearing) 과
     박스 폭에 해당하는 각도 폭(half_width) 을 구한다 (start 좌표).  ← 카메라는 거리를 모르므로 '방향'만 안다
  2) TrunkLocator.locate(): 복셀 맵에서 그 방향 쐐기(wedge) 안, 지면 위 줄기 높이 띠(0.15~0.7 m, 수관·처진 가지 아래) 에 있는
     복셀을 거리순으로 모아, 가장 가까운 '세로로 이어진 가느다란 덩어리' 를 줄기로 확정한다.  ← LiDAR 가 거리를 준다
  3) (trunk_registry.py) 이미 아는 줄기와 매칭하거나 새 줄기로 등록한다.

왜 가장 가까운 덩어리인가
  카메라는 앞의 물체에 가려진 뒤쪽을 보지 못한다. 그 방향에서 처음 만나는 줄기 모양 덩어리가 YOLO 가 본 줄기다.

줄기 모양 판정 (조정 가능한 값: TrunkLocator 인자, voxel_map_node 파라미터 trunk.*)
  - 띠(band) 안 복셀이 min_voxels 개 이상
  - 세로 길이(띠 안 최고-최저) ≥ min_height  → 잔디·돌(낮고 넓음)과 구분
  - 수평 퍼짐 ≤ max_width                   → 덤불·수관 아래 가지와 구분
  - 띠 아래쪽(지면 가까이)부터 이어짐        → 공중에 뜬 가지·잎 덩어리와 구분
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def bbox_bearing(u_center: float, v_center: float, width_px: float, k: np.ndarray,
                 r_start_cam: np.ndarray) -> tuple[float, float]:
    """박스 가운데 (u, v) [px] 와 폭 [px] → start 좌표 수평 방위각 [rad], 반폭 각도 [rad].

    k: 카메라 내부행렬 3×3, r_start_cam: 카메라 광학 프레임 → start 회전 3×3 (z 전방, x 오른쪽, y 아래).
    """
    fx, fy, cx, cy = k[0, 0], k[1, 1], k[0, 2], k[1, 2]

    def bearing(u):
        d = r_start_cam @ np.array([(u - cx) / fx, (v_center - cy) / fy, 1.0])   # 광선 방향 (start)
        return math.atan2(d[1], d[0])
    b0 = bearing(u_center)
    bl, br = bearing(u_center - 0.5 * width_px), bearing(u_center + 0.5 * width_px)
    half = 0.5 * abs(math.atan2(math.sin(bl - br), math.cos(bl - br)))
    return b0, half


@dataclass
class TrunkHit:
    """복셀 맵에서 확정한 줄기 하나."""
    x: float              # start 좌표 [m]
    y: float
    z: float              # 줄기 밑동(지면) 높이 [m]
    range: float          # 카메라에서 거리 [m]
    n_voxels: int         # 띠 안 복셀 수
    height: float         # 띠 안 세로 길이 [m]
    width: float          # 수평 퍼짐 [m]


class TrunkLocator:
    """복셀 맵의 '줄기 높이 띠' 복셀을 들고 있다가, 방향이 주어지면 그 방향의 줄기를 찾는다."""

    def __init__(self, band=(0.15, 0.7), r_min: float = 0.8, r_max: float = 10.0,
                 margin: float = 0.12, min_voxels: int = 3, min_height: float = 0.3,
                 max_width: float = 0.45, gap: float = 0.35):
        """band: 지면 위 줄기 높이 띠 [m], r_min/r_max: 탐색 거리 [m], margin: 쐐기 여유 [m]
        (박스 각도 폭에 거리×각 대신 더하는 옆 여유), gap: 거리 방향으로 이 간격 이상 떨어지면 다른 덩어리 [m]."""
        self.band, self.r_min, self.r_max = band, r_min, r_max
        self.margin, self.min_voxels, self.min_height = margin, min_voxels, min_height
        self.max_width, self.gap = max_width, gap
        self.xyz = np.zeros((0, 3))
        self.hag = np.zeros(0)
        self.ground = np.zeros(0)

    def set_voxels(self, centers: np.ndarray, hag: np.ndarray) -> int:
        """복셀 중심 (M×3) 과 지면 위 높이 (M,) 중 띠 안의 것만 보관. 보관한 개수 반환."""
        m = np.isfinite(hag) & (hag >= self.band[0]) & (hag <= self.band[1])
        self.xyz, self.hag = centers[m], hag[m]
        self.ground = self.xyz[:, 2] - self.hag
        return int(m.sum())

    def locate(self, cam_xy, bearing: float, half_width: float) -> TrunkHit | None:
        """카메라 위치 cam_xy (start [m]) 에서 bearing ± half_width 쐐기 안의 가장 가까운 줄기. 없으면 None."""
        if not len(self.xyz):
            return None
        rel = self.xyz[:, :2] - np.asarray(cam_xy, float)[:2]
        c, s = math.cos(bearing), math.sin(bearing)
        along = rel[:, 0] * c + rel[:, 1] * s             # 광선 방향 거리
        across = -rel[:, 0] * s + rel[:, 1] * c           # 광선에서 옆으로 떨어진 거리
        ok = (along > self.r_min) & (along < self.r_max) & \
             (np.abs(across) <= along * math.tan(half_width) + self.margin)
        if not ok.any():
            return None
        idx = np.nonzero(ok)[0][np.argsort(along[ok])]
        groups, cur = [], [idx[0]]
        for a, b in zip(idx[:-1], idx[1:]):                # 거리순으로 훑으며 gap 에서 끊기
            if along[b] - along[a] > self.gap:
                groups.append(cur)
                cur = []
            cur.append(b)
        groups.append(cur)
        for g in groups:                                    # 가까운 덩어리부터 줄기 모양인지 확인
            g = np.array(g)
            if len(g) < self.min_voxels:
                continue
            hz = self.hag[g]
            height = float(hz.max() - hz.min())
            xy = self.xyz[g, :2]
            width = float(np.hypot(*(xy.max(0) - xy.min(0))))
            if height < self.min_height or width > self.max_width or hz.min() > self.band[0] + 0.2:
                continue                                    # 줄기는 땅에서부터 이어진다 (공중 가지·잎 덩어리 제외)
            cx_, cy_ = np.median(xy, axis=0)                   # 중앙값: 옆 가지·잎 복셀 몇 개에 덜 흔들림
            return TrunkHit(float(cx_), float(cy_), float(np.median(self.ground[g])),
                            float(np.median(along[g])), len(g), height, width)
        return None

    def locate_near(self, xy, radius: float = 0.35) -> TrunkHit | None:
        """위치를 이미 아는 경우 (LiDAR 줄기 검출 등): 그 주변 radius 안 띠 복셀로 줄기 확인·위치 보정."""
        if not len(self.xyz):
            return None
        d = np.hypot(self.xyz[:, 0] - xy[0], self.xyz[:, 1] - xy[1])
        g = np.nonzero(d < radius)[0]
        if len(g) < self.min_voxels:
            return None
        hz = self.hag[g]
        height = float(hz.max() - hz.min())
        if height < self.min_height:
            return None
        p = self.xyz[g, :2].mean(0)
        w = float(np.hypot(*(self.xyz[g, :2].max(0) - self.xyz[g, :2].min(0))))
        return TrunkHit(float(p[0]), float(p[1]), float(np.median(self.ground[g])), float('nan'), len(g), height, w)
