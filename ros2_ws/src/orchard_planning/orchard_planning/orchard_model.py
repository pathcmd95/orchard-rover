"""관측한 줄기 → 과수원 열·통로 모델 — ROS 의존성 없음.

전역 경로계획(global_planner.py)이 "통로 중심선이 어디고, 열이 어디서 끝나는가" 를 알기 위해 쓴다.

좌표
  model 좌표 = odom − (odom 흐름 보정). 주행 중 odom 이 옆으로 흐르면 global_planner 가 보정값을 갱신한다.
  row frame  = model 좌표를 '출발 위치 원점, 열 방향 u (앞), 수직 v (왼쪽)' 로 돌린 좌표. 계산은 대부분 여기서.

처리
  1) add(): 줄기 관측 (model 좌표) 을 0.2 m 칸에 모아 평균 (같은 줄기 여러 번 관측 → 한 점)
  2) rows(): 두 번 이상 관측된 점들의 v 분포에서 봉우리 = 열 (orchard_navigation.treemap._row_peaks)
  3) lanes(): 통로 0 (출발 통로) 부터 돌 방향(turn_sign) 쪽으로 통로 k 의 중심 v, 양쪽 열, 열의 u 범위
     - 통로 k 의 '뒤 열' = 통로 k−1 쪽 열, '앞 열' = 돌 방향 쪽 열. 통로 k+1 의 뒤 열 = 통로 k 의 앞 열
     - 두 열을 다 보면 중심 = 가운데, 한쪽만 보면 그 열 ± 열 간격/2, 못 보면 파라미터 간격으로 이어 붙임
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from orchard_navigation.treemap import _row_heading, _row_peaks


@dataclass
class Row:
    """열 하나 (row frame)."""
    v: float                 # 횡위치 [m]
    u_min: float             # 가장 뒤 줄기 [m]
    u_max: float             # 가장 앞 줄기 [m]
    n: int                   # 줄기 수


@dataclass
class Lane:
    """통로 하나 (row frame)."""
    k: int                   # 통로 번호 (출발 통로 0)
    c: float                 # 중심 횡위치 v [m]
    back: Row | None         # 통로 k−1 쪽 열 (없으면 None)
    front: Row | None        # 돌 방향 쪽 열
    direction: int           # +1 = u 증가 방향으로 달림, −1 = 반대 (지그재그)

    def rows(self) -> list:
        """관측된 양쪽 열 목록."""
        return [r for r in (self.back, self.front) if r is not None]

    def end_tree(self, d: int | None = None) -> float:
        """달리는 방향 d 쪽 끝 줄기 u [m] (열을 못 봤으면 NaN)."""
        d = self.direction if d is None else d
        rs = self.rows()
        if not rs:
            return float('nan')
        return max(r.u_max for r in rs) if d > 0 else min(r.u_min for r in rs)


class OrchardModel:
    """줄기 누적 → 열·통로. origin/heading 은 row frame 정의 (출발 자세)."""

    def __init__(self, origin, heading: float, row_spacing: float, turn_sign: int, cell: float = 0.2):
        """origin: row frame 원점 (model x, y) [m], heading: 열 방향 [rad], turn_sign: +1 왼쪽 / −1 오른쪽."""
        self.o = np.asarray(origin, float)
        self.theta = float(heading)
        self.theta0 = float(heading)
        self.S = float(row_spacing)
        self.s = int(turn_sign)
        self.cell = cell
        self.acc: dict = {}                   # 칸 → [x합, y합, 횟수]

    # ------------------------------------------------------------ 좌표
    @property
    def e_u(self) -> np.ndarray:
        """열 방향 단위벡터 (model)."""
        return np.array([math.cos(self.theta), math.sin(self.theta)])

    @property
    def e_v(self) -> np.ndarray:
        """열에 수직(왼쪽) 단위벡터 (model)."""
        return np.array([-math.sin(self.theta), math.cos(self.theta)])

    def to_uv(self, xy: np.ndarray) -> np.ndarray:
        """model (N×2) → row frame (N×2)."""
        d = np.asarray(xy, float).reshape(-1, 2) - self.o
        return np.c_[d @ self.e_u, d @ self.e_v]

    def to_xy(self, uv: np.ndarray) -> np.ndarray:
        """row frame (N×2) → model (N×2)."""
        uv = np.asarray(uv, float).reshape(-1, 2)
        return self.o + uv[:, :1] * self.e_u + uv[:, 1:2] * self.e_v

    def lane_yaw(self, direction: int) -> float:
        """방향 direction (+1/−1) 으로 달릴 때의 요 [rad] (model)."""
        return self.theta if direction > 0 else math.atan2(-math.sin(self.theta), -math.cos(self.theta))

    # ------------------------------------------------------------ 누적
    def add(self, xy: np.ndarray) -> None:
        """줄기 관측 (N×2, model) 누적."""
        for x, y in np.asarray(xy, float).reshape(-1, 2):
            k = (int(math.floor(x / self.cell)), int(math.floor(y / self.cell)))
            a = self.acc.get(k)
            if a is None:
                self.acc[k] = [x, y, 1]
            else:
                a[0] += x
                a[1] += y
                a[2] += 1

    def trees(self, min_hits: int = 2) -> np.ndarray:
        """min_hits 번 이상 관측된 줄기 위치 (M×2, model)."""
        P = [(a[0] / a[2], a[1] / a[2]) for a in self.acc.values() if a[2] >= min_hits]
        return np.array(P).reshape(-1, 2)

    def refine_heading(self, max_change_deg: float = 25.0) -> bool:
        """줄기 배치로 열 방향을 다시 추정 (출발 요 오차 보정). 바뀌면 True."""
        P = self.trees()
        if len(P) < 8:
            return False
        h = _row_heading(P, self.S)
        if math.cos(h - self.theta0) < 0:                  # 열 방향은 π 주기 → 출발 방향과 같은 쪽으로
            h += math.pi
        dh = math.atan2(math.sin(h - self.theta0), math.cos(h - self.theta0))
        if abs(dh) > math.radians(max_change_deg):
            return False
        self.theta = self.theta0 + dh
        return True

    # ------------------------------------------------------------ 열·통로
    def rows(self) -> list:
        """관측된 열 목록 (row frame, v 오름차순)."""
        P = self.trees()
        if len(P) < 2:
            return []
        uv = self.to_uv(P)
        out = []
        for pk in _row_peaks(uv[:, 1], self.S):
            m = np.abs(uv[:, 1] - pk) < 0.3 * self.S
            if m.sum() < 2:
                continue
            out.append(Row(float(np.median(uv[m, 1])), float(uv[m, 0].min()), float(uv[m, 0].max()), int(m.sum())))
        return out

    def spacing(self, rows: list) -> float:
        """이웃 열 사이 간격의 중앙값 (파라미터 간격 ±40 % 안의 것만, 없으면 파라미터 값)."""
        vs = sorted(r.v for r in rows)
        gaps = [b - a for a, b in zip(vs[:-1], vs[1:]) if 0.6 * self.S < b - a < 1.4 * self.S]
        return float(np.median(gaps)) if gaps else self.S

    def lanes(self, k_max: int, rows: list | None = None) -> list:
        """통로 0 ~ k_max 의 Lane 목록."""
        rows = self.rows() if rows is None else rows
        S = self.spacing(rows)
        s = self.s

        def find(v_expected):
            cand = [r for r in rows if abs(r.v - v_expected) < 0.35 * S]
            return min(cand, key=lambda r: abs(r.v - v_expected)) if cand else None

        out = []
        back = find(-s * 0.5 * S)
        front = find(s * 0.5 * S)
        if back and front:
            c = 0.5 * (back.v + front.v)
        elif front:
            c = front.v - s * 0.5 * S
        elif back:
            c = back.v + s * 0.5 * S
        else:
            c = 0.0
        out.append(Lane(0, c, back, front, +1))
        for k in range(1, k_max + 1):
            prev = out[-1]
            back = prev.front
            back_v = back.v if back else prev.c + s * 0.5 * S
            front = find(back_v + s * S)
            c = 0.5 * (back_v + front.v) if front else back_v + s * 0.5 * S
            out.append(Lane(k, c, back, front, +1 if k % 2 == 0 else -1))
        return out
