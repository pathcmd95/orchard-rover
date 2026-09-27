"""줄기 매칭·등록부 — ROS 의존성 없음.

복셀 맵에서 확정한 줄기(TrunkHit)를 이미 등록된 줄기와 매칭한다.
  - 가장 가까운 등록 줄기가 assoc_radius 안이면 같은 줄기 → 위치를 관측 횟수 가중 평균으로 갱신
  - 아니면 새 번호로 등록
좌표는 모두 start 프레임 (출발점 (0,0,0), 처음 달린 방향 +x) 기준 상대좌표 [m].
min_hits 번 이상 관측된 줄기만 '확정' 으로 보고 저장·표시한다 (한 번 잘못 잡힌 것 걸러내기).

저장 파일 (save_csv)
  id, x, y, z, hits, mean_conf, source, first_seen, last_seen
  source: yolo (카메라 검출 → 복셀 확인) / lidar (LiDAR 줄기 검출 → 복셀 확인, YOLO 모델이 없을 때)
"""
from __future__ import annotations

import csv
import math
from dataclasses import dataclass

import numpy as np
from orchard_navigation.treemap import _row_peaks


@dataclass
class Trunk:
    """등록된 줄기 하나 (start 좌표)."""
    id: int
    x: float
    y: float
    z: float
    hits: int = 1
    conf_sum: float = 0.0
    source: str = 'yolo'
    first_seen: float = 0.0
    last_seen: float = 0.0

    @property
    def mean_conf(self) -> float:
        """평균 검출 점수 (YOLO confidence, 0~1)."""
        return self.conf_sum / max(self.hits, 1)


class TrunkRegistry:
    """줄기 등록부. update() 로 관측을 넣고 confirmed() 로 확정 줄기를 얻는다."""

    def __init__(self, assoc_radius: float = 0.4, min_hits: int = 3):
        """assoc_radius: 같은 줄기로 볼 최대 거리 [m] (주간 거리 1.2 m 의 1/3 정도), min_hits: 확정 관측 수."""
        self.assoc_radius = assoc_radius
        self.min_hits = min_hits
        self.trunks: list[Trunk] = []

    def update(self, x: float, y: float, z: float, conf: float = 1.0, t: float = 0.0,
               source: str = 'yolo') -> Trunk:
        """관측 하나를 매칭·등록하고 해당 줄기를 돌려준다."""
        best, best_d = None, self.assoc_radius
        for tr in self.trunks:
            d = math.hypot(tr.x - x, tr.y - y)
            if d < best_d:
                best, best_d = tr, d
        if best is None:
            best = Trunk(len(self.trunks) + 1, x, y, z, 1, conf, source, t, t)
            self.trunks.append(best)
            return best
        w = 1.0 / (best.hits + 1)                         # 누적 평균: 새 관측 가중치 1/(n+1)
        best.x += w * (x - best.x)
        best.y += w * (y - best.y)
        best.z += w * (z - best.z)
        best.hits += 1
        best.conf_sum += conf
        best.last_seen = t
        if source == 'yolo':
            best.source = 'yolo'                          # 한 번이라도 YOLO 로 확인되면 yolo 로 표시
        return best

    def confirmed(self) -> list[Trunk]:
        """min_hits 이상 관측된 줄기."""
        return [t for t in self.trunks if t.hits >= self.min_hits]

    def on_rows(self, row_spacing: float = 3.8, tol: float = 0.4) -> tuple[list, list]:
        """확정 줄기를 (열 위에 있는 것, 열에서 벗어난 것) 으로 나눈다.

        start 좌표의 +x 는 처음 달린 통로 방향이라 열은 x 축과 나란하다 → y 분포의 봉우리 = 열 (treemap._row_peaks).
        봉우리에서 tol [m] 넘게 떨어진 줄기는 오검출(통로의 풀·돌을 YOLO 방향에서 잘못 고른 것 등)로 보고 뺀다.
        (Gazebo uturn 시험: 빼기 전 46개 중앙값 오차 0.18 m → 뺀 뒤 35개 0.09 m)
        """
        rows = self.confirmed()
        if len(rows) < 4:
            return rows, []
        peaks = _row_peaks(np.array([t.y for t in rows]), row_spacing)
        keep = [t for t in rows if np.min(np.abs(peaks - t.y)) <= tol]
        return keep, [t for t in rows if t not in keep]

    def save_csv(self, path: str, rows: list | None = None) -> int:
        """확정 줄기(rows 를 주면 그것)를 CSV 로 저장. 저장한 개수 반환."""
        rows = self.confirmed() if rows is None else rows
        with open(path, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(['id', 'x', 'y', 'z', 'hits', 'mean_conf', 'source', 'first_seen', 'last_seen'])
            for t in rows:
                w.writerow([t.id, f'{t.x:.3f}', f'{t.y:.3f}', f'{t.z:.3f}', t.hits, f'{t.mean_conf:.2f}',
                            t.source, f'{t.first_seen:.2f}', f'{t.last_seen:.2f}'])
        return len(rows)
