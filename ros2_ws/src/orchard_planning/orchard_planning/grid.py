"""점유 격자(occupancy grid) — ROS 의존성 없음.

전역 경로계획(hybrid_astar.py)이 "로버 중심이 이 칸에 들어가면 부딪히는가" 를 빠르게 묻는 데 쓴다.
장애물 점(나무 줄기 위치 등) 둘레를 inflate [m] 만큼 부풀려 칠해 두면, 로버를 점으로 보고 검사해도
로버 반경만큼의 여유가 자동으로 생긴다 (configuration space 방식).

좌표: 격자를 만든 좌표계 (전역 계획은 'model' 좌표 = odom − 흐름 보정). 단위 [m].
격자 밖은 '모름' → 비어 있다고 본다 (헤드랜드는 보통 빈 땅). 필요하면 unknown_is_free=False.

RViz 표시: to_int8() 가 nav_msgs/OccupancyGrid.data 형식(0 = 빈 칸, 100 = 점유)으로 바꿔 준다.
"""
from __future__ import annotations

import math

import numpy as np


def disc_offsets(radius_cells: int) -> np.ndarray:
    """반경 radius_cells 칸 안의 (di, dj) 오프셋 목록 (K×2). 부풀리기에 쓴다."""
    r = int(radius_cells)
    di, dj = np.meshgrid(np.arange(-r, r + 1), np.arange(-r, r + 1), indexing='ij')
    m = di * di + dj * dj <= r * r + 1e-9
    return np.c_[di[m], dj[m]]


class OccupancyGrid:
    """2D 점유 격자. data[i, j] = True 면 점유 (i = x 칸, j = y 칸)."""

    def __init__(self, x0: float, y0: float, nx: int, ny: int, res: float, unknown_is_free: bool = True):
        """x0, y0: 격자 (0,0) 칸의 왼쪽 아래 모서리 [m], nx, ny: 칸 수, res: 칸 크기 [m]."""
        self.x0, self.y0, self.res = float(x0), float(y0), float(res)
        self.nx, self.ny = int(nx), int(ny)
        self.data = np.zeros((self.nx, self.ny), bool)
        self.unknown_is_free = unknown_is_free

    @classmethod
    def around(cls, points: np.ndarray, extra_xy: np.ndarray | None = None, res: float = 0.1,
               margin: float = 4.0) -> 'OccupancyGrid':
        """points (장애물) 와 extra_xy (시작·목표 등 꼭 들어가야 할 점) 를 margin [m] 여유로 덮는 빈 격자."""
        allp = [np.asarray(points, float).reshape(-1, 2)]
        if extra_xy is not None:
            allp.append(np.asarray(extra_xy, float).reshape(-1, 2))
        P = np.vstack(allp)
        if not len(P):
            P = np.zeros((1, 2))
        lo, hi = P.min(0) - margin, P.max(0) + margin
        n = np.ceil((hi - lo) / res).astype(int) + 1
        return cls(lo[0], lo[1], n[0], n[1], res)

    def cell(self, xy: np.ndarray) -> np.ndarray:
        """좌표 (N×2) → 칸 번호 (N×2 int)."""
        xy = np.asarray(xy, float).reshape(-1, 2)
        return np.floor((xy - [self.x0, self.y0]) / self.res).astype(int)

    def add_discs(self, centers: np.ndarray, radius: float) -> None:
        """centers (N×2) 둘레 반경 radius [m] 원을 점유로 칠한다 (장애물 크기 + 로버 반경)."""
        c = self.cell(centers)
        if not len(c):
            return
        off = disc_offsets(int(math.ceil(radius / self.res)))
        idx = (c[:, None, :] + off[None, :, :]).reshape(-1, 2)
        ok = (idx[:, 0] >= 0) & (idx[:, 0] < self.nx) & (idx[:, 1] >= 0) & (idx[:, 1] < self.ny)
        idx = idx[ok]
        self.data[idx[:, 0], idx[:, 1]] = True

    def occupied(self, xy: np.ndarray) -> np.ndarray:
        """점들 (N×2) 이 점유 칸에 있는지 (N,) bool."""
        c = self.cell(xy)
        inside = (c[:, 0] >= 0) & (c[:, 0] < self.nx) & (c[:, 1] >= 0) & (c[:, 1] < self.ny)
        out = np.full(len(c), not self.unknown_is_free)
        out[inside] = self.data[c[inside, 0], c[inside, 1]]
        return out

    def path_free(self, path: np.ndarray) -> bool:
        """경로 점열 (N×≥2) 의 모든 점이 빈 칸이면 True."""
        return not self.occupied(np.asarray(path)[:, :2]).any()

    def to_int8(self) -> np.ndarray:
        """nav_msgs/OccupancyGrid.data 순서 (행 = y, 열 = x, 행 우선) 의 int8 배열 (0 / 100)."""
        return np.where(self.data.T, 100, 0).astype(np.int8).ravel()
