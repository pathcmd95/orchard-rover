"""세그멘테이션 결과(주행가능 픽셀) → 지면 역투영 → 통로 중심선 — ROS 의존성 없음.

1) 주행가능 라벨 픽셀을 카메라 광선으로 바꿔 base_link 의 지면(z = ground_z)과 교차 → 지면 위 점
2) 전방 거리 구간마다 주행가능 영역의 좌/우 경계(백분위)와 가운데를 구함
3) 구간 가운데들을 직선 y = a + b·x 로 적합 → 중심선 offset(a), heading(atan b)
LiDAR 행 추정(rows.py)과 같은 규약: base_link x 전방 / y 왼쪽, offset + = 중심선이 왼쪽.

파이프라인 내 역할
  seg_path_node: 라벨 영상 → [seg_path.py] → /orchard/seg/row (RowCenterline) → row_navigator_node 가 LiDAR 중심선과 융합
입력 / 출력
  ground_points: 주행가능 마스크 H×W bool, 내부행렬 K (라벨 영상 크기에 맞춘 것), T_base_cam 4x4
                 → 지면 점 Nx2 [m, base_link] + 가장자리 여부
  centerline:    지면 점 Nx2 → SegRow (offset [m], heading [rad], width [m], confidence 0~1)
가정: 지면이 base_link 기준 수평 평면 z = ground_z (경사·차체 기울기는 무시 → 먼 거리일수록 오차 커짐)
주요 파라미터 (seg_path_node, config/sim.yaml · robot.yaml)
  ground_z  base_link 기준 지면 높이 [m] (lidar_tree_node 의 ground.nominal_z 와 같게)
  stride    역투영 픽셀 간격, x_min / x_max  중심선 적합 전방 거리 범위 [m]
  (bin_size, edge_pct, min_width 등은 ROS 파라미터로 노출되지 않음 → centerline 기본값 수정)
알고리즘: 핀홀 카메라 역투영 + 광선-평면 교차 + 가중 최소제곱 직선 적합 (일반 기하 기법).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class SegRow:
    """카메라 세그멘테이션 기반 중심선 추정 결과 (base_link 기준, rows.RowEstimate 와 같은 부호 규약)."""

    valid: bool = False
    offset: float = 0.0          # x=0 에서 중심선 y [m] (+: 왼쪽)
    heading: float = 0.0         # 중심선 방향 [rad] (+: 왼쪽으로 기울어짐)
    width: float = 0.0           # 주행가능 영역 폭의 중앙값 [m]
    confidence: float = 0.0      # 0~1
    bins_used: int = 0           # 적합에 쓴 거리 구간 수


def ground_points(mask: np.ndarray, k: np.ndarray, t_base_cam: np.ndarray, ground_z: float,
                  stride: int = 4, max_range: float = 15.0, border_px: int = 3):
    """mask(H×W bool) 의 픽셀을 지면 z=ground_z 평면에 역투영.
    반환: (base_link 좌표 Nx2, 영상 좌/우 가장자리 픽셀에서 온 점인지 N bool)
    가장자리 점은 주행가능 영역이 화면 밖으로 이어진다는 뜻 → 그 쪽 경계는 믿을 수 없다.
    t_base_cam: 카메라 광학좌표 → base_link 4x4 (광학: z 전방, x 오른쪽, y 아래).

    k: mask 크기에 맞춘 3x3 내부행렬, ground_z: 지면 z [m], stride: 가로·세로 몇 픽셀마다 하나씩 쓸지
    (계산량 1/stride² 로 감소), max_range: 이보다 먼 지면 점은 버림 [m] (먼 곳은 픽셀당 오차가 큼),
    border_px: 영상 좌/우 끝에서 몇 픽셀 이내를 '가장자리'로 볼지.
    """
    h, w = mask.shape
    vs, us = np.nonzero(mask[::stride, ::stride])      # 간격 샘플된 마스크에서 True 인 (행, 열)
    if len(us) == 0:
        return np.zeros((0, 2)), np.zeros(0, bool)
    # 샘플 인덱스 → 원래 픽셀 좌표 (stride 칸의 가운데 근처)
    us = us * stride + 0.5 * stride
    vs = vs * stride + 0.5 * stride
    border = (us < border_px + stride) | (us > w - border_px - stride)
    # 픽셀 (u, v) → 광학좌표 광선 방향 ((u-cx)/fx, (v-cy)/fy, 1)  (핀홀 모델의 역)
    rays = np.stack([(us - k[0, 2]) / k[0, 0], (vs - k[1, 2]) / k[1, 1], np.ones_like(us, float)], 1)
    R, t = t_base_cam[:3, :3], t_base_cam[:3, 3]      # t = 카메라 위치 (base_link)
    d = rays @ R.T                                # base 좌표 방향
    # 광선 p(s) = t + s·d 와 평면 z = ground_z 의 교점: t_z + s·d_z = ground_z
    with np.errstate(divide='ignore', invalid='ignore'):
        s = (ground_z - t[2]) / d[:, 2]
    ok = np.isfinite(s) & (s > 0)                 # s ≤ 0: 수평선 위쪽(하늘) 광선 → 지면과 만나지 않음
    p = t + d[ok] * s[ok, None]
    border = border[ok]
    near = np.hypot(p[:, 0], p[:, 1]) < max_range
    return p[near, :2], border[near]


def centerline(points: np.ndarray, border=None, x_min: float = 1.0, x_max: float = 8.0,
               bin_size: float = 0.5, min_width: float = 0.8, max_width: float = 6.0, min_points: int = 8,
               edge_pct: float = 5.0, edge_tol: float = 0.15) -> SegRow:
    """주행가능 지면 점(Nx2)에서 중심선 추정. border 가 주어지면 경계가 화면 밖에서 잘린 구간은 뺀다.

    points: 지면 점 Nx2 [m, base_link], border: ground_points 의 가장자리 여부 (N bool)
    x_min, x_max, bin_size: 전방 거리 구간 [m] (기본 1~8 m 를 0.5 m 씩 14 구간)
    min_width, max_width: 구간 폭이 이 범위 [m] 밖이면 버림 (잡음 조각 / 통로가 아닌 넓은 공터)
    min_points: 구간 최소 점 수, edge_pct: 좌/우 경계로 쓸 백분위 [%] (이상점 픽셀에 강인)
    edge_tol: 가장자리 점이 경계에서 이 거리 [m] 이내면 그 경계는 화면에 잘린 것으로 판단
    반환: SegRow (적합 구간 3개 미만이면 valid=False).
    """
    border = np.zeros(len(points), bool) if border is None else np.asarray(border, bool)
    xs, ys, ws = [], [], []
    for x0 in np.arange(x_min, x_max, bin_size):
        sel = (points[:, 0] >= x0) & (points[:, 0] < x0 + bin_size)
        if np.count_nonzero(sel) < min_points:
            continue
        y = points[sel, 1]
        # 최솟값·최댓값 대신 5%/95% 백분위를 경계로: 오분류 픽셀 몇 개에 경계가 끌려가지 않게
        lo, hi = np.percentile(y, [edge_pct, 100 - edge_pct])
        b = border[sel]
        if np.any(b & (y <= lo + edge_tol)) or np.any(b & (y >= hi - edge_tol)):
            continue                              # 한쪽 경계가 화면 밖 → 가운데를 알 수 없음
        w = hi - lo
        if not (min_width <= w <= max_width):
            continue
        xs.append(x0 + 0.5 * bin_size)            # 구간 가운데 x
        ys.append(0.5 * (lo + hi))                # 주행가능 영역 가운데 y
        ws.append(w)
    n = len(xs)
    if n < 3:                                     # 직선 적합에 최소 3 구간 (2 개면 잔차가 항상 0)
        return SegRow(bins_used=n)
    xs, ys, ws = map(np.asarray, (xs, ys, ws))
    A = np.c_[np.ones(n), xs]
    wt = 1.0 / (1.0 + 0.15 * xs)                  # 가까운 구간일수록 정확 → 가중치 큼
    # (x=1 m 에서 0.87, x=8 m 에서 0.45: 먼 곳은 픽셀 하나가 지면에서 더 넓은 영역이라 오차가 큼)
    (a, b), *_ = np.linalg.lstsq(A * wt[:, None], ys * wt, rcond=None)
    resid = ys - (a + b * xs)
    rms = float(np.sqrt(np.mean(resid ** 2)))
    cover = n / max(1, int((x_max - x_min) / bin_size))    # 쓸 수 있었던 구간 비율 (0~1)
    # 신뢰도 = 구간 비율 점수 (2/3 이상 쓰면 1) × 잔차 점수 (RMS 25 cm 이면 e^-1 ≈ 0.37)
    conf = float(np.clip(1.5 * cover, 0, 1) * math.exp(-(rms / 0.25) ** 2))
    return SegRow(True, float(a), float(math.atan(b)), float(np.median(ws)), conf, n)
