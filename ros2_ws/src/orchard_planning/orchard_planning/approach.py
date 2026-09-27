"""과수원 진입: 멀리서 LiDAR 로 본 줄기들로 첫 통로 입구를 찾는다 — ROS 의존성 없음.

무엇을 하나
  로버가 과수원 밖(입구에서 10 m 안팎, 비스듬해도 됨)에서 출발할 때, 앞에 보이는 줄기 배치로
    열 방향 → 열(횡방향 봉우리) → 통로(이웃한 두 열의 가운데) → 끝 통로 입구
  를 찾는다. global_planner 가 이 입구 앞 정렬점까지 Hybrid A*/Dubins 경로를 만들고, 도착하면 평소 출발 절차(bootstrap)로 넘긴다.

좌표
  입력 줄기는 odom (N×2). 반환 값도 odom. 열 방향 h 는 '로버 쪽에서 과수원 안으로' 향하게 정한다
  (로버의 열 방향 좌표 u 가 줄기들보다 작은 쪽 → 로버는 열 시작점 바깥에 있음).
  v(열에 수직, 왼쪽 +) 가 작은 쪽 = 과수원을 바라볼 때 오른쪽.

References
  - 열 방향·봉우리: orchard_navigation.treemap (_row_heading, _row_peaks) 재사용
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from orchard_navigation.treemap import _row_heading, _row_peaks


@dataclass
class Entrance:
    """찾은 통로 입구 (odom)."""
    heading: float                   # 열(통로) 방향, 과수원 안쪽으로 [rad]
    entry: np.ndarray                # 통로 중심선 위 열 시작점 (x, y)
    pre: np.ndarray                  # 정렬점 = entry − pre_dist·열 방향 (여기서 통로를 똑바로 보고 출발)
    lane_c: float                    # 통로 중심 v [m]
    n_rows: int                      # 본 열 수
    side: str                        # 고른 끝 통로 right / left
    rows: list = field(default_factory=list)       # [(p0, p1)] 열 선분 (표시용)
    lane_line: tuple = ()                          # (p0, p1) 고른 통로 중심선 (표시용)


def merge_close(P: np.ndarray, radius: float = 0.5) -> np.ndarray:
    """가까운 점(같은 줄기를 여러 스캔에서 본 것)을 하나로 합쳐 평균 위치 (M×2). 욕심쟁이 방식 — 수십~수백 개면 충분히 빠름.

    왜: 여러 스캔을 겹치면 한 줄기가 여러 점이 되어, '가장 가까운 이웃 = 같은 열 다음 나무' 가정(_row_heading)이 깨진다."""
    P = np.asarray(P, float).reshape(-1, 2)
    out, used = [], np.zeros(len(P), bool)
    for i in range(len(P)):
        if used[i]:
            continue
        m = (~used) & (np.hypot(P[:, 0] - P[i, 0], P[:, 1] - P[i, 1]) < radius)
        used |= m
        out.append(P[m].mean(0))
    return np.array(out).reshape(-1, 2)


def detect_entrance(P: np.ndarray, rover_xy, row_spacing: float, side: str = 'right', min_trees: int = 3,
                    pre_dist: float = 3.0) -> Entrance | None:
    """줄기 P (odom N×2) 에서 끝 통로 입구. side: 'right' | 'left' (과수원을 바라볼 때). 못 찾으면 None.

    절차: 열 방향(_row_heading) → 과수원 안쪽으로 부호 결정 → 열 수직 좌표 v 의 봉우리 = 열 (min_trees 그루 이상)
          → 간격이 열 간격의 0.6~1.5 배인 이웃 열 쌍 = 통로 → side 쪽 끝 통로 → 두 열의 시작 u 중 작은 값이 입구.
    """
    P = merge_close(P)
    if len(P) < 2 * min_trees:
        return None
    S = float(row_spacing)
    h = _row_heading(P, S)
    e_u = np.array([math.cos(h), math.sin(h)])
    r = np.asarray(rover_xy, float)
    if r @ e_u > float(np.median(P @ e_u)):          # 로버가 열 시작 바깥에 있도록 방향을 뒤집음
        h = math.atan2(-e_u[1], -e_u[0])
        e_u = -e_u
    e_v = np.array([-e_u[1], e_u[0]])
    u, v = P @ e_u, P @ e_v
    peaks = []
    for pk in _row_peaks(v, S):
        m = np.abs(v - pk) < 0.3 * S
        if m.sum() >= min_trees:
            peaks.append((float(np.mean(v[m])), m))
    peaks.sort(key=lambda t: t[0])
    pairs = [(a, b) for a, b in zip(peaks[:-1], peaks[1:]) if 0.6 * S <= b[0] - a[0] <= 1.5 * S]
    if not pairs:
        return None
    a, b = pairs[0] if side == 'right' else pairs[-1]
    vc = 0.5 * (a[0] + b[0])
    u0 = float(min(u[a[1]].min(), u[b[1]].min()))
    entry = u0 * e_u + vc * e_v
    rows = [((u[m].min() * e_u + pk * e_v), (u[m].max() * e_u + pk * e_v)) for pk, m in peaks]
    u1 = float(max(u[a[1]].max(), u[b[1]].max()))
    return Entrance(h, entry, entry - pre_dist * e_u, vc, len(peaks), side, rows,
                    (u0 * e_u + vc * e_v, u1 * e_u + vc * e_v))
