"""지면 기준 높이 밴드에서 과수 줄기 후보를 찾는다.

절차
1) 지면 높이 h 가 [band_min, band_max] 인 점만 남김 (잔디·수관 제거)
2) XY 평면 격자에 투영 후 연결요소(8-이웃) 군집화
3) 크기·점 개수 조건으로 줄기 후보 선별

파이프라인 내 역할
  lidar_tree_node: ground.py(점별 지면 높이) → [trunks.py 줄기 후보] → rows.py(좌/우 열 직선 적합)
입력 / 출력 (ROS 의존성 없음)
  입력: base_link 기준 Nx3 점군 [m] + 같은 길이의 지면 높이 배열 [m] (GroundPlane.height 결과)
  출력: TrunkCandidate 리스트 (x, y [m, base_link], 반경 [m], 신뢰도 0~1 등)
주요 튜닝 파라미터 (lidar_tree_node 의 trunk.* 파라미터, config/sim.yaml · robot.yaml)
  trunk.band_min / band_max  줄기만 남길 높이 구간 [m]. 잔디(~15 cm)보다 높게, 수관 시작보다 낮게 (품종별 조정)
  trunk.cell_size            군집화 격자 크기 [m]. 줄기 지름(10~20 cm) 보다 작게
  trunk.min_points           군집 최소 점 개수 (먼 줄기를 놓치면 줄임)
  trunk.max_diameter         이보다 넓으면 줄기가 아님 [m] (사람·울타리 제거)
  trunk.min_height_span      군집 높이 범위 최솟값 [m] (납작한 잔디 뭉치 제거)
알고리즘: 격자 기반 연결요소 라벨링 + union-find(서로소 집합) 자료구조. 특정 논문 구현이 아닌 일반 기법.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class TrunkCandidate:
    """줄기 후보 1개 (base_link 기준). orchard_msgs/Tree 로 변환되어 /orchard/trees 로 나간다."""

    x: float                 # 줄기 중심 x [m] (전방 +)
    y: float                 # 줄기 중심 y [m] (왼쪽 +)
    radius: float            # 추정 반경 [m]
    top_height: float        # 군집 점의 최대 지면높이 [m]
    num_points: int          # 군집 점 개수
    confidence: float        # 0~1, 행 적합(rows.fit_rows)의 가중치로 쓰임


def select_band(points: np.ndarray, heights: np.ndarray, band_min: float, band_max: float):
    """지면 높이가 [band_min, band_max] [m] 인 점과 그 높이만 골라 (points, heights) 로 반환."""
    mask = (heights >= band_min) & (heights <= band_max)
    return points[mask], heights[mask]


def _grid_components(cells: np.ndarray) -> np.ndarray:
    """정수 격자좌표(Nx2)의 8-이웃 연결요소 라벨을 반환 (union-find).

    cells: 중복 없는 격자 좌표 Nx2 (int). 반환: 길이 N 라벨 배열 (같은 값 = 같은 군집, 값은 대표 인덱스).
    """
    n = cells.shape[0]
    parent = np.arange(n)            # 처음에는 각 셀이 자기 자신의 대표

    def find(i):
        """셀 i 의 대표(root)를 찾고, 지나온 경로를 root 에 바로 연결 (경로 압축)."""
        root = i
        while parent[root] != root:
            root = parent[root]
        while parent[i] != root:
            parent[i], i = root, parent[i]
        return root

    index = {(int(c[0]), int(c[1])): i for i, c in enumerate(cells)}
    # 8-이웃 중 절반(→, ↑, ↗, ↘)만 검사해도 된다: 나머지 방향은 상대 셀에서 검사할 때 같은 쌍이 연결됨
    offsets = [(1, 0), (0, 1), (1, 1), (1, -1)]
    for i, c in enumerate(cells):
        cx, cy = int(c[0]), int(c[1])
        for dx, dy in offsets:
            j = index.get((cx + dx, cy + dy))
            if j is not None:
                ri, rj = find(i), find(j)
                if ri != rj:
                    parent[rj] = ri          # 두 집합 합치기 (union)
    return np.array([find(i) for i in range(n)])


def cluster_xy(points: np.ndarray, cell_size: float = 0.08) -> list[np.ndarray]:
    """점군을 XY 격자 연결요소로 군집화하여 점 인덱스 배열 리스트를 반환.

    points: Nx2 이상 (x, y 만 사용) [m]. cell_size: 격자 한 칸 [m].
    대각선까지 이웃으로 보므로 한 칸(cell_size) 이내로 떨어진 점들은 같은 군집이 된다.
    """
    if points.shape[0] == 0:
        return []
    cells = np.floor(points[:, :2] / cell_size).astype(np.int64)   # 점 → 정수 격자 좌표
    # 같은 셀의 점들은 하나로 묶어 셀 단위로 라벨링 (inverse: 점 → 셀 번호)
    uniq, inverse = np.unique(cells, axis=0, return_inverse=True)
    inverse = inverse.reshape(-1)    # numpy 버전에 따라 inverse 가 2차원으로 나오는 경우 대비
    labels = _grid_components(uniq)
    point_labels = labels[inverse]
    clusters = []
    for lab in np.unique(point_labels):
        clusters.append(np.nonzero(point_labels == lab)[0])
    return clusters


def detect_trunks(
    points: np.ndarray,
    heights: np.ndarray,
    band_min: float = 0.25,
    band_max: float = 0.7,
    cell_size: float = 0.08,
    min_points: int = 6,
    max_diameter: float = 0.45,
    min_height_span: float = 0.12,
) -> list[TrunkCandidate]:
    """줄기 후보 검출. points/heights 는 base_link 기준, 같은 길이.

    points: Nx3 [m], heights: 길이 N, 각 점의 지면 높이 [m].
    나머지 인자는 모듈 docstring 의 trunk.* 파라미터 설명 참고 (단위 m).
    반환: TrunkCandidate 리스트 (순서는 군집 라벨 순, 의미 없음).
    """
    band_pts, band_h = select_band(points, heights, band_min, band_max)
    result: list[TrunkCandidate] = []
    for idx in cluster_xy(band_pts, cell_size):
        if idx.size < min_points:
            continue
        cp = band_pts[idx]
        ch = band_h[idx]
        xy = cp[:, :2]
        extent = xy.max(axis=0) - xy.min(axis=0)      # 군집의 x, y 방향 폭 [m]
        diameter = float(np.max(extent))
        if diameter > max_diameter:
            continue          # 사람·수레·울타리처럼 넓은 물체는 줄기가 아님
        span = float(ch.max() - ch.min())
        if span < min_height_span:
            continue          # 잔디 뭉치처럼 납작한 물체 제거
        center = np.median(xy, axis=0)                # 평균보다 이상점(잎·가지)에 강인
        # 라이다는 줄기의 한쪽 면만 보므로 보이는 면 반경을 조금 보정
        # (보이는 폭의 절반을 반경으로, 최소 3 cm — 점이 몇 개 없을 때 0 에 가까운 반경 방지)
        radius = max(0.03, 0.5 * diameter)
        dist = float(np.hypot(center[0], center[1]))
        # 거리 대비 점 개수 기반 신뢰도 (먼 줄기일수록 점이 적은 것을 보정)
        # 줄기에 맞는 점 수는 대략 1/거리 로 줄어드므로 (점 수 × 거리) 는 거리와 무관한 값.
        # 60 [점·m] 이상이면 신뢰도 1 (예: 1 m 에서 60점, 10 m 에서 6점). 1 m 이내는 거리 1 로 취급.
        conf = float(np.clip(idx.size * max(dist, 1.0) / 60.0, 0.0, 1.0))
        # 높이 방향으로 밴드를 많이 채울수록 줄기다움. 밴드의 30% 미만이어도 최소 0.3 배는 유지
        conf *= float(np.clip(span / (band_max - band_min), 0.3, 1.0))
        result.append(TrunkCandidate(
            x=float(center[0]), y=float(center[1]), radius=radius,
            top_height=float(ch.max()), num_points=int(idx.size), confidence=conf))
    return result
