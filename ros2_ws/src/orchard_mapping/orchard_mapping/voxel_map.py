"""LiDAR 점군을 누적하는 복셀(voxel) 맵 — ROS 의존성 없음.

복셀 맵이란
  공간을 한 변 voxel [m] 인 정육면체 칸으로 나누고, 점이 들어온 칸을 '채워짐'으로 기록한 3D 격자 지도.
  여기서는 칸마다 "몇 번의 스캔에서 점이 들어왔는가(hits)" 를 센다. 한두 번만 잡힌 칸은 잡음·움직이는 물체일
  가능성이 커서 표시·탐색 때 min_hits 로 거른다. (OctoMap 처럼 빈 공간을 지우는 광선 투사는 하지 않는 단순형)

저장 구조
  칸 번호 (i, j, k) = floor(p / voxel) 를 하나의 64 비트 정수로 묶어 dict 키로 쓴다 (각 축 ±2^20 칸 = ±104 km @0.1 m).
  → 넓은 과수원도 채워진 칸만 메모리에 있다.

지면 높이
  xy 를 ground_cell [m] 격자로 나눠 각 칸의 가장 낮은 채워진 복셀 높이를 지면으로 본다 (잔디는 몇 cm 라 영향 작음).
  '지면 위 높이(HAG)' 는 줄기 탐색(지면 위 0.15~0.85 m 띠)과 높이별 색칠에 쓴다.

References
  - A. Hornung et al., "OctoMap: An Efficient Probabilistic 3D Mapping Framework Based on Octrees",
    Autonomous Robots 34(3), 2013 — 확률 점유 복셀 맵의 대표 구현 (여기서는 개념만 단순화해 사용)
"""
from __future__ import annotations

import numpy as np

_OFF = 1 << 20          # 음수 칸 번호를 양수로 옮기는 오프셋
_MASK = (1 << 21) - 1   # 축당 21 비트


def pack(idx: np.ndarray) -> np.ndarray:
    """칸 번호 (N×3 int) → 64 비트 키 (N,)."""
    i = idx.astype(np.int64) + _OFF
    return (i[:, 0] << 42) | (i[:, 1] << 21) | i[:, 2]


def unpack(keys: np.ndarray) -> np.ndarray:
    """64 비트 키 (N,) → 칸 번호 (N×3 int)."""
    k = np.asarray(keys, np.int64)
    return np.c_[(k >> 42) & _MASK, (k >> 21) & _MASK, k & _MASK] - _OFF


class VoxelMap:
    """점군 누적 복셀 맵. integrate() 로 스캔을 넣고 occupied() 로 채워진 칸을 꺼낸다."""

    def __init__(self, voxel: float = 0.1, ground_cell: float = 0.5):
        """voxel: 복셀 한 변 [m], ground_cell: 지면 높이를 추정하는 xy 격자 [m]."""
        self.voxel = float(voxel)
        self.ground_cell = float(ground_cell)
        self.hits: dict[int, int] = {}
        self.scans = 0

    def __len__(self) -> int:
        """채워진 복셀 수."""
        return len(self.hits)

    def integrate(self, pts: np.ndarray) -> int:
        """한 스캔의 점 (N×3, 맵 좌표 [m]) 을 넣는다. 스캔 하나에서 같은 칸은 1 번만 센다. 새 칸 수 반환."""
        P = np.asarray(pts, float).reshape(-1, 3)
        P = P[np.isfinite(P).all(axis=1)]
        if not len(P):
            return 0
        keys = np.unique(pack(np.floor(P / self.voxel).astype(np.int64)))
        before = len(self.hits)
        h = self.hits
        for k in keys.tolist():
            h[k] = h.get(k, 0) + 1
        self.scans += 1
        return len(self.hits) - before

    def occupied(self, min_hits: int = 1) -> tuple[np.ndarray, np.ndarray]:
        """채워진 칸의 중심 (M×3 [m]) 과 hits (M,)."""
        if not self.hits:
            return np.zeros((0, 3)), np.zeros(0, int)
        keys = np.fromiter(self.hits.keys(), np.int64, len(self.hits))
        hits = np.fromiter(self.hits.values(), np.int64, len(self.hits))
        m = hits >= min_hits
        centers = (unpack(keys[m]) + 0.5) * self.voxel
        return centers, hits[m]

    def ground_grid(self, centers: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """xy 격자 칸별 가장 낮은 복셀 중심 높이 [m] (지면으로 간주). (정렬된 칸 키, 높이) 반환."""
        if not len(centers):
            return np.zeros(0, np.int64), np.zeros(0)
        cell = np.floor(centers[:, :2] / self.ground_cell).astype(np.int64)
        key = (cell[:, 0] + _OFF) * (1 << 22) + (cell[:, 1] + _OFF)   # xy 칸 → 정수 하나
        order = np.lexsort((centers[:, 2], key))                        # 칸별로, 그 안에서 z 오름차순
        k, z = key[order], centers[order, 2]
        first = np.ones(len(k), bool)
        first[1:] = k[1:] != k[:-1]                                     # 칸별 첫 원소 = 가장 낮은 z
        return k[first], z[first]

    def height_above_ground(self, centers: np.ndarray, grid=None, step_tol: float = 0.3) -> np.ndarray:
        """각 복셀의 지면 위 높이 HAG [m]. 지면을 모르는 곳은 NaN.

        자기 칸의 가장 낮은 복셀을 지면으로 보되, 이웃 3×3 칸 최저보다 step_tol 이상 높으면
        (나무 밑이 가려져 수관만 보이는 칸 등) 이웃 최저를 쓴다. 이웃 모두 비었으면 NaN.
        """
        if not len(centers):
            return np.zeros(0)
        gk, gz = self.ground_grid(centers) if grid is None else grid
        cell = np.floor(centers[:, :2] / self.ground_cell).astype(np.int64)
        nbr = np.full(len(centers), np.inf)
        own = np.full(len(centers), np.inf)
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                q = (cell[:, 0] + di + _OFF) * (1 << 22) + (cell[:, 1] + dj + _OFF)
                j = np.clip(np.searchsorted(gk, q), 0, len(gk) - 1)
                cand = np.where(gk[j] == q, gz[j], np.inf)
                if di == 0 and dj == 0:
                    own = cand
                nbr = np.minimum(nbr, cand)
        ground = np.where(own <= nbr + step_tol, own, nbr)
        ground = np.where(np.isfinite(ground), ground, np.nan)
        return centers[:, 2] - ground

    # ------------------------------------------------------------ 저장
    def save_npz(self, path: str) -> None:
        """전체 맵을 npz 로 저장 (keys, hits, voxel). load_npz 로 다시 읽을 수 있다."""
        keys = np.fromiter(self.hits.keys(), np.int64, len(self.hits))
        hits = np.fromiter(self.hits.values(), np.int64, len(self.hits))
        np.savez_compressed(path, keys=keys, hits=hits, voxel=self.voxel, ground_cell=self.ground_cell)

    @classmethod
    def load_npz(cls, path: str) -> 'VoxelMap':
        """save_npz 로 저장한 맵 읽기."""
        d = np.load(path)
        vm = cls(float(d['voxel']), float(d['ground_cell']))
        vm.hits = dict(zip(d['keys'].tolist(), d['hits'].tolist()))
        return vm

    def save_ply(self, path: str, min_hits: int = 2) -> int:
        """채워진 복셀 중심을 높이 색이 입혀진 점군 PLY(ASCII) 로 저장 (CloudCompare·MeshLab 에서 열림). 점 수 반환."""
        c, _h = self.occupied(min_hits)
        rgb = height_colors(self.height_above_ground(c)) if len(c) else np.zeros((0, 3))
        with open(path, 'w') as f:
            f.write('ply\nformat ascii 1.0\n')
            f.write(f'comment voxel {self.voxel} m, frame start (출발점 기준 상대좌표)\n')
            f.write(f'element vertex {len(c)}\n')
            f.write('property float x\nproperty float y\nproperty float z\n')
            f.write('property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n')
            for (x, y, z), (r, g, b) in zip(c, (rgb * 255).astype(int)):
                f.write(f'{x:.3f} {y:.3f} {z:.3f} {r} {g} {b}\n')
        return len(c)


def height_colors(hag: np.ndarray) -> np.ndarray:
    """지면 위 높이 [m] → RGB (0~1). 지면(초록) → 줄기 높이(청록) → 수관(파랑) → 높은 곳(짙은 남색)."""
    stops = np.array([0.0, 0.3, 1.0, 2.0, 3.5])
    cols = np.array([[0.35, 0.80, 0.25],     # 지면·잔디: 초록
                     [0.25, 0.75, 0.70],     # 줄기 아래쪽: 청록
                     [0.20, 0.45, 0.85],     # 줄기 위·수관 아래: 파랑
                     [0.10, 0.20, 0.70],     # 수관: 남색
                     [0.05, 0.05, 0.45]])    # 높은 가지: 짙은 남색
    h = np.clip(np.asarray(hag, float), stops[0], stops[-1])
    return np.stack([np.interp(h, stops, cols[:, i]) for i in range(3)], axis=1)
