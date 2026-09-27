"""과수원 나무 지도 — ROS 의존성 없음.

파이프라인에서의 역할
  주행(행 추종 → 행 끝 → U턴 → 다음 통로)과 나란히 도는 '지도 만들기' 부분. orchard_mapper_node 가
  /orchard/trees 검출을 이 모듈로 누적·정리·저장한다. 주행 제어에는 쓰이지 않는다.

주행 중 검출된 줄기 위치(월드/odom 좌표)를 누적해
  1) 같은 나무를 한 점으로 합치고 (최근접 연관 + 이동평균),
  2) 행 방향을 추정해 열 번호 / 열 안 나무 번호를 붙이고,
  3) 나무 간격이 벌어진 곳을 결주(빠진 나무) 후보로 표시한다.
결과는 CSV(나무 목록)와 JSON(요약)으로 저장해 과원 관리(결주 보식, 수확량 조사 등)에 쓴다.

단위·좌표
  - 위치 [m]: odom(ENU, x 동 / y 북) 좌표 (TreeMap, organize, save). project() 입력은 base_link [m].
  - 각도 [rad], 시각 [s]. 위경도 [deg] (WGS84 근사).
보조 도구 (orchard_mapper_node 가 사용)
  - PoseHistory: odom 자세 기록 → 검출 시각의 자세 보간
  - YawBiasEstimator: 나침반 방위 오차 추정
  - best_time_offset / sharpness: odom–LiDAR 시간 지연 추정 (지도가 가장 뾰족해지는 보정값 탐색)

파라미터 (orchard_mapper_node, sim.yaml / robot.yaml 의 orchard_mapper_node 항목)
  row_spacing, assoc_radius, min_hits, max_range 등. organize() 의 gap_factor, merge_dist 등은 함수 기본값.
"""
from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass

import numpy as np


@dataclass
class Landmark:
    """지도 위 나무(줄기) 후보 하나: 위치 x, y [m] (odom), 줄기 반경 [m], 관측 횟수, 마지막 관측 시각 [s]."""

    x: float
    y: float
    radius: float
    hits: int = 1
    last_seen: float = 0.0


class TreeMap:
    """줄기 검출을 누적하는 나무 지도 (최근접 연관 + 관측 평균)."""

    def __init__(self, assoc_radius: float = 0.35, min_hits: int = 3):
        """Args:
            assoc_radius: 기존 나무와 같은 나무로 볼 최대 거리 [m] (나무 간격 1.2 m 의 약 1/3).
            min_hits: 이만큼 관측돼야 확정 나무 (가끔 잡히는 잡초·사람 등 제외).
        """
        self.assoc_radius = assoc_radius
        self.min_hits = min_hits
        self.items: list[Landmark] = []

    # ------------------------------------------------------------ 누적
    def update(self, xy: np.ndarray, radii=None, now: float = 0.0) -> int:
        """검출(Nx2, 월드 좌표)을 지도에 반영. 새로 만든 후보 수 반환.

        각 검출을 가장 가까운 기존 나무(assoc_radius 이내)에 붙이고 위치·반경을 누적 평균한다.
        한 프레임에서 한 나무에는 검출 하나만 붙인다 (두 줄기가 한 나무로 뭉치지 않게).
        Args:
            xy: 검출 위치 [m] (odom).
            radii: 줄기 반경 [m] (없으면 0.05 m 로 가정).
            now: 관측 시각 [s].
        """
        xy = np.asarray(xy, float).reshape(-1, 2)
        radii = np.full(len(xy), 0.05) if radii is None else np.asarray(radii, float)
        pts = np.array([[m.x, m.y] for m in self.items]).reshape(-1, 2)
        used = set()
        created = 0
        for (x, y), r in zip(xy, radii):
            j = -1
            if len(pts):
                d = np.hypot(pts[:, 0] - x, pts[:, 1] - y)
                j = int(np.argmin(d))
                if d[j] > self.assoc_radius or j in used:
                    j = -1
            if j < 0:
                self.items.append(Landmark(float(x), float(y), float(r), 1, now))
                created += 1
                continue
            used.add(j)
            m = self.items[j]
            w = 1.0 / (m.hits + 1)          # 누적 평균: n 번째 관측의 가중치 1/n
            m.x += w * (x - m.x)
            m.y += w * (y - m.y)
            m.radius += w * (r - m.radius)
            m.hits += 1
            m.last_seen = now
            pts[j] = (m.x, m.y)
        return created

    def confirmed(self) -> list[Landmark]:
        """min_hits 번 이상 관측된 확정 나무 목록."""
        return [m for m in self.items if m.hits >= self.min_hits]

    # ------------------------------------------------------------ 정리
    def organize(self, row_spacing: float, heading: float | None = None, gap_factor: float = 1.6,
                 merge_dist: float = 0.45, rel_min_hits: float = 0.3, row_tolerance: float = 0.6) -> dict:
        """열/나무 번호, 결주 후보를 계산한 요약 dict 반환.
        heading: 행 방향 [rad] (모르면 나무 배치에서 추정).
        merge_dist: 같은 열에서 이보다 가까운 두 점은 한 나무(가지·자세 오차로 둘로 잡힌 것)로 합침.
        rel_min_hits: 관측 횟수가 중앙값의 이 비율보다 적은 점은 버림 (잡초·돌 등 가끔 잡히는 것).
        row_tolerance: 열 중심선에서 이보다 먼 점은 이상점 [m].
        나무 사이 한가운데(간격 절반)에 있는 점은 콘크리트 기둥·지주로 보고 나무 번호에서 뺀다 ('posts').
        row_spacing: 열 간격 [m] (열 봉우리 분리·이상점 판정 기준).
        gap_factor: 주간 간격의 이 배수보다 벌어지면 결주 후보 (1.6 → 한 그루 빠지면 간격 2배라 잡힘).

        반환 dict: heading, num_trees, outliers,
          rows  = [{row, num_trees, length, spacing, offset, missing[{after_tree, x, y}], posts[{x, y}]}],
          trees = [{row, tree, x, y, radius, hits}] (열 번호는 행에 수직인 v 가 작은 열부터 1, 나무는 u 순)."""
        trees = self.confirmed()
        if len(trees) >= 4:
            hits = np.array([m.hits for m in trees])
            trees = [m for m in trees if m.hits >= rel_min_hits * np.median(hits)]
        if len(trees) < 2:
            return {'rows': [], 'trees': [], 'heading': heading, 'num_trees': len(trees), 'outliers': 0}
        P = np.array([[m.x, m.y] for m in trees])
        H = np.array([m.hits for m in trees], float)
        if heading is None:
            heading = _row_heading(P, row_spacing)
        # u = 행 방향 좌표, v = 행에 수직(왼쪽) 좌표 [m]
        u_dir = np.array([math.cos(heading), math.sin(heading)])
        v_dir = np.array([-u_dir[1], u_dir[0]])
        u, v = P @ u_dir, P @ v_dir
        # 횡방향 v 의 밀도 봉우리 = 열. 봉우리에서 열 간격 30% 안의 점만 그 열로, 나머지는 이상점
        peaks = _row_peaks(v, row_spacing)
        groups: dict[int, list[int]] = {}
        outliers = 0
        for i in range(len(v)):
            d = np.abs(peaks - v[i])
            k = int(np.argmin(d))
            if d[k] <= min(row_tolerance, 0.3 * row_spacing):
                groups.setdefault(k, []).append(i)
            else:
                outliers += 1
        rows, table = [], []
        for r, k in enumerate(sorted(groups, key=lambda k: peaks[k])):
            idx = sorted(groups[k], key=lambda i: u[i])
            merged = []                               # [u, v, x, y, radius, hits] 관측 횟수 가중 평균
            for i in idx:
                m = trees[i]
                if merged and u[i] - merged[-1][0] < merge_dist:
                    a = merged[-1]
                    w0, w1 = a[5], H[i]
                    f = lambda p, q: (p * w0 + q * w1) / (w0 + w1)  # noqa: E731
                    merged[-1] = [f(a[0], u[i]), f(a[1], v[i]), f(a[2], m.x), f(a[3], m.y),
                                  f(a[4], m.radius), w0 + w1]
                else:
                    merged.append([u[i], v[i], m.x, m.y, m.radius, H[i]])
            merged, posts = _split_posts(merged)
            M = np.array(merged)
            us = M[:, 0]
            gaps = np.diff(us)
            spacing = _tree_spacing(gaps)
            pv = float(np.mean(M[:, 1]))              # 이 열의 평균 횡위치 [m]
            missing = []
            if len(gaps) and spacing > 0:
                for j, g in enumerate(gaps):
                    n_miss = int(round(g / spacing)) - 1      # 이 간격에 빠진 나무 수 (간격 ≈ (n+1)·주간)
                    if g > gap_factor * spacing and n_miss >= 1:
                        for q in range(1, n_miss + 1):
                            # 빠진 자리를 간격 안에 고르게 배치
                            p = (us[j] + q * g / (n_miss + 1)) * u_dir + pv * v_dir
                            missing.append({'after_tree': j + 1, 'x': round(float(p[0]), 3),
                                            'y': round(float(p[1]), 3)})
            for j, a in enumerate(merged):
                table.append({'row': r + 1, 'tree': j + 1, 'x': round(float(a[2]), 3), 'y': round(float(a[3]), 3),
                              'radius': round(float(a[4]), 3), 'hits': int(a[5])})
            rows.append({'row': r + 1, 'num_trees': len(merged), 'length': round(float(us[-1] - us[0]), 2),
                         'spacing': round(spacing, 3), 'offset': round(pv, 3), 'missing': missing,
                         'posts': [{'x': round(float(a[2]), 3), 'y': round(float(a[3]), 3)} for a in posts]})
        return {'heading': round(float(heading), 4), 'num_trees': len(table), 'rows': rows, 'trees': table,
                'outliers': outliers}

    def save(self, summary: dict, csv_path: str, json_path: str, geo=None):
        """geo: (lat0, lon0) 이면 odom(ENU) 원점의 위경도로 보고 각 나무의 위경도를 함께 저장.

        summary: organize() 결과. CSV = 나무 한 그루 한 줄 (row, tree, x, y, radius, hits[, lat, lon]),
        JSON = trees 를 뺀 요약 (열별 나무 수·주간·결주·기둥 등).
        """
        with open(csv_path, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            head = ['row', 'tree', 'x', 'y', 'radius', 'hits'] + (['lat', 'lon'] if geo else [])
            w.writerow(head)
            for t in summary['trees']:
                row = [t[k] for k in ('row', 'tree', 'x', 'y', 'radius', 'hits')]
                if geo:
                    row += list(enu_to_latlon(t['x'], t['y'], *geo))
                w.writerow(row)
        with open(json_path, 'w', encoding='utf-8') as f:
            json.dump({k: v for k, v in summary.items() if k != 'trees'}, f, ensure_ascii=False, indent=1)


def _row_heading(P: np.ndarray, row_spacing: float) -> float:
    """나무 배치에서 행 방향 추정.
    1) 가까운 이웃끼리 잇는 선분 방향의 평균(0~π)으로 대략 방향 (열 안 간격 < 열 간격)
    2) 그 ±20° 안에서 행과 수직으로 투영한 분포가 가장 뾰족한 각도로 다듬음 (중복·이상점에 강함)
    P: 나무 위치 (N×2) [m], row_spacing: 열 간격 [m]. 반환: 행 방향 [rad] (π 주기라 앞/뒤 구분 없음)."""
    d = np.hypot(P[:, None, 0] - P[None, :, 0], P[:, None, 1] - P[None, :, 1])
    np.fill_diagonal(d, np.inf)
    ang = []
    for i in range(len(P)):
        j = int(np.argmin(d[i]))
        if d[i, j] < 0.6 * row_spacing:              # 같은 열 안의 이웃
            ang.append(math.atan2(P[j, 1] - P[i, 1], P[j, 0] - P[i, 0]) % math.pi)
    if ang:
        a2 = np.array(ang) * 2                       # 방향(π 주기) 평균
        h0 = float(math.atan2(np.mean(np.sin(a2)), np.mean(np.cos(a2))) / 2)
    else:
        # 이웃이 없으면 주성분(PCA) 축: 가장 퍼진 방향을 행 방향으로
        c = np.cov((P - P.mean(0)).T)
        _w, vecs = np.linalg.eigh(c)
        h0 = float(math.atan2(vecs[1, -1], vecs[0, -1]))
    if len(P) < 4:
        return h0

    def sharpness(h):
        """행 방향 h [rad] 로 봤을 때 횡위치 분포의 뾰족함 (클수록 나무가 열로 잘 모임)."""
        v = P @ np.array([-math.sin(h), math.cos(h)])
        z = (v[:, None] - v[None, :]) / 0.15         # 0.15 m: 가우스 커널 폭 (줄기 위치 오차 수준)
        return float(np.exp(-0.5 * z * z).sum())   # 가우스 커널 쌍 합: 한 줄로 모일수록 큼
    # 거친 탐색 (±20°, 0.5° 간격) → 고운 탐색 (±0.5°, 0.05° 간격)
    best = max(np.radians(np.arange(-20.0, 20.01, 0.5)) + h0, key=sharpness)
    return max(best + np.radians(np.arange(-0.5, 0.51, 0.05)), key=sharpness)


def _tree_spacing(gaps: np.ndarray) -> float:
    """주간 간격: 기둥 때문에 생긴 절반 간격을 빼고 중앙값.

    gaps: 열 안 이웃 점 사이 간격 [m]. 중앙값의 0.7 배보다 작은 간격(기둥 사이 절반 간격)은 뺀다.
    반환: 주간 [m] (간격이 없으면 NaN)."""
    if not len(gaps):
        return float('nan')
    m = float(np.median(gaps))
    big = gaps[gaps >= 0.7 * m]
    return float(np.median(big)) if len(big) else m


def _split_posts(merged: list) -> tuple[list, list]:
    """열 안의 점 [u, ...] (u 순) → (나무, 기둥). 기둥 = 양옆 나무와의 간격이 둘 다 주간의 절반쯤인 점,
    또는 열 끝에서 주간의 절반보다 가까이 붙은 점.
    (문턱: 양옆 간격 < 0.7·주간 이고 두 간격 합이 주간의 ±30% 이내 / 열 끝은 < 0.6·주간.)"""
    if len(merged) < 3:
        return merged, []
    u = np.array([a[0] for a in merged])
    s = _tree_spacing(np.diff(u))
    posts = set()
    for j in range(len(u)):
        gp = u[j] - u[j - 1] if j > 0 else None          # 앞 점과의 간격 [m]
        gn = u[j + 1] - u[j] if j < len(u) - 1 else None  # 뒤 점과의 간격 [m]
        if gp is not None and gn is not None:
            if gp < 0.7 * s and gn < 0.7 * s and abs(gp + gn - s) < 0.3 * s:
                posts.add(j)
        elif (gp if gp is not None else gn) < 0.6 * s:
            posts.add(j)
    return [a for j, a in enumerate(merged) if j not in posts], [a for j, a in enumerate(merged) if j in posts]


def _row_peaks(v: np.ndarray, row_spacing: float, sigma: float = 0.2) -> np.ndarray:
    """횡방향 좌표 v 의 커널 밀도 봉우리 (서로 열 간격 절반 이상 떨어진 것, 2그루 이상).

    v: 나무들의 횡위치 [m], sigma: 가우스 커널 폭 [m]. 반환: 열 중심 횡위치 [m] (오름차순)."""
    grid = np.arange(v.min() - 1.0, v.max() + 1.0, 0.05)      # 5 cm 격자
    dens = np.exp(-0.5 * ((grid[:, None] - v[None, :]) / sigma) ** 2).sum(1)
    cand = [i for i in range(1, len(grid) - 1) if dens[i] >= dens[i - 1] and dens[i] > dens[i + 1]]
    cand.sort(key=lambda i: -dens[i])
    peaks = []
    for i in cand:
        if dens[i] < 1.5:           # 나무 한 그루 봉우리 높이 = 1 → 1.5 미만은 2그루도 안 되는 봉우리
            break
        if all(abs(grid[i] - p) >= 0.5 * row_spacing for p in peaks):
            peaks.append(float(grid[i]))
    return np.array(sorted(peaks)) if peaks else np.array([float(np.median(v))])


class YawBiasEstimator:
    """자세 요(나침반 기반)의 고정 방위 오차 추정: 곧게 전진한 구간의 이동 방향(위치 차, GPS) − 평균 요.
    과수원 철제 지주·배터리 등으로 자력계 방위가 수~수십 도 틀어지는 경우 대비."""

    def __init__(self, segment: float = 1.0, max_rate: float = 0.05, max_spread: float = 0.05,
                 max_bias: float = 0.6, keep: int = 30):
        """Args:
            segment: 표본 하나를 만드는 직진 거리 [m] (ROS 파라미터 bias_segment).
            max_rate: 요레이트가 이보다 크면 도는 중 → 구간 버림 [rad/s].
            max_spread: 구간 안 요가 평균에서 이보다 벗어나면 곧지 않음 → 버림 [rad] (약 3°).
            max_bias: 이보다 큰 오차는 후진·옆미끄럼으로 보고 버림 [rad] (약 34°).
            keep: 최근 표본 몇 개로 중앙값을 낼지.
        """
        self.segment, self.max_rate, self.max_spread, self.max_bias, self.keep = \
            segment, max_rate, max_spread, max_bias, keep
        self.seg = None                               # 진행 중 구간 [시작 x, 시작 y, 요 목록, 시작 시각]
        self.samples: list[float] = []
        self.valid_since: float | None = None      # 첫 유효 구간이 시작된 시각 (이전 요는 믿지 않음)

    @property
    def bias(self) -> float:
        """방위 오차 추정값 [rad] = 표본 중앙값 (이동 방향 − 자세 요). 표본이 없으면 0."""
        return float(np.median(self.samples)) if self.samples else 0.0

    def update(self, x: float, y: float, yaw: float, yaw_rate: float, t: float = 0.0) -> bool:
        """odom 한 개 반영. 새 표본이 생기면 True.

        Args: x, y [m] (odom 위치, GPS 기반), yaw [rad] (자세 요, 나침반 기반), yaw_rate [rad/s], t [s].
        """
        if abs(yaw_rate) > self.max_rate:            # 도는 중이면 구간 버림
            self.seg = None
            return False
        if self.seg is None:
            self.seg = [x, y, [yaw], t]
            return False
        self.seg[2].append(yaw)
        dx, dy = x - self.seg[0], y - self.seg[1]
        if math.hypot(dx, dy) < self.segment:
            return False
        yaws = np.array(self.seg[2])
        ym = math.atan2(np.mean(np.sin(yaws)), np.mean(np.cos(yaws)))     # 원형 평균 요
        spread = float(np.max(np.abs(np.arctan2(np.sin(yaws - ym), np.cos(yaws - ym)))))
        t0 = self.seg[3]
        self.seg = [x, y, [yaw], t]
        b = math.atan2(dy, dx) - ym                  # 이동 방향 − 자세 요
        b = math.atan2(math.sin(b), math.cos(b))
        if spread > self.max_spread or abs(b) > self.max_bias:   # 곧게 전진한 구간만 (후진·옆미끄럼 제외)
            return False
        self.samples = (self.samples + [b])[-self.keep:]
        if self.valid_since is None:
            self.valid_since = t0
        return True


class PoseHistory:
    """(t, x, y, yaw) 기록과 선형 보간. 기록 범위 밖(tol 넘게)은 None."""

    def __init__(self, min_dt: float = 0.02):
        """min_dt: 이 간격 [s] 보다 촘촘한 기록은 건너뜀 (메모리 절약, 50 Hz)."""
        self.t: list[float] = []
        self.p: list[tuple[float, float, float]] = []
        self.min_dt = min_dt

    def add(self, t: float, x: float, y: float, yaw: float):
        """자세 하나 기록 (t [s] 증가 순, x, y [m], yaw [rad])."""
        if self.t and t < self.t[-1] + self.min_dt:
            return
        self.t.append(t)
        self.p.append((x, y, yaw))

    def at(self, t: float, tol: float = 0.3):
        """시각 t [s] 의 자세 (x, y, yaw) 선형 보간. 기록 범위에서 tol [s] 넘게 벗어나면 None."""
        import bisect
        if not self.t or t < self.t[0] - tol or t > self.t[-1] + tol:
            return None
        i = bisect.bisect_left(self.t, t)
        if i <= 0:
            return self.p[0]
        if i >= len(self.t):
            return self.p[-1]
        (xa, ya, wa), (xb, yb, wb) = self.p[i - 1], self.p[i]
        f = (t - self.t[i - 1]) / max(self.t[i] - self.t[i - 1], 1e-9)
        dw = math.atan2(math.sin(wb - wa), math.cos(wb - wa))     # ±π 경계를 넘는 요도 짧은 쪽으로 보간
        return (xa + f * (xb - xa), ya + f * (yb - ya), wa + f * dw)


def project(records, hist: PoseHistory, offset: float = 0.0, bias: float = 0.0):
    """검출 기록 [(t, pts(로봇 기준 Nx2), radii)] → 지도 좌표 [(t, Nx2, radii)].
    offset: 검출 시각에 더할 자세 시각 보정 [s] (odom 이 실제보다 앞서면 음수), bias: 요 보정 [rad].
    자세 기록 범위 밖 검출은 빠진다."""
    out = []
    for t, pts, radii in records:
        pose = hist.at(t + offset)
        if pose is None:
            continue
        x, y, yaw = pose
        c, s = math.cos(yaw + bias), math.sin(yaw + bias)
        out.append((t, np.c_[x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]], radii))
    return out


def sharpness(points: np.ndarray, cell: float = 0.1) -> float:
    """점들이 얼마나 한곳에 겹치는지 (같은 나무가 여러 번 같은 자리에 찍힐수록 큼).

    cell [m] 격자 히스토그램에서 각 칸 × 주변 3×3 칸 합을 더한 값을 9·N² 로 나눈 상대 점수.
    범위 0 ~ 1/9 (모든 점이 한 칸에 모이면 1/9). 값 자체보다 보정값끼리 비교할 때 쓴다."""
    if len(points) < 2:
        return 0.0
    ij = np.floor(points / cell).astype(np.int64)
    ij -= ij.min(0)
    h = np.zeros(ij.max(0) + 3)                      # 가장자리 한 칸씩 여유 (np.roll 이 반대편으로 넘어가지 않게)
    np.add.at(h, (ij[:, 0] + 1, ij[:, 1] + 1), 1.0)
    b = h.copy()                                     # 3×3 합 (칸 경계에 걸친 점 보정)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if dx or dy:
                b += np.roll(np.roll(h, dx, 0), dy, 1)
    return float((h * b).sum() / (9.0 * len(points) ** 2))


def best_time_offset(records, hist: PoseHistory, bias: float = 0.0, lo: float = -1.5, hi: float = 0.3,
                     step: float = 0.05, max_records: int = 600):
    """지도가 가장 뾰족해지는 자세 시각 보정 [s]과 (그 점수, 보정 0 점수). 속도가 바뀐 구간이 있어야 구별된다.

    lo~hi [s] 를 step [s] 간격으로 모두 시험 (격자 탐색). 계산량을 줄이려 기록은 최대 max_records 개만 고르게 뽑는다.
    반환: (보정 [s], 최고 점수, 보정 0 점수) — 호출자는 최고 점수가 보정 0 보다 뚜렷이 클 때만 적용한다."""
    if len(records) > max_records:
        idx = np.linspace(0, len(records) - 1, max_records).astype(int)
        records = [records[i] for i in idx]

    def score(off):
        """시각 보정 off [s] 로 투영한 지도의 sharpness."""
        pr = project(records, hist, off, bias)
        return sharpness(np.vstack([p for _, p, _ in pr])) if pr else 0.0
    offs = np.arange(lo, hi + 1e-9, step)
    scores = [score(o) for o in offs]
    k = int(np.argmax(scores))
    return float(offs[k]), scores[k], score(0.0)


def enu_to_latlon(east: float, north: float, lat0: float, lon0: float) -> tuple[float, float]:
    """작은 영역용 근사 (수 km 이내 오차 cm 수준).

    원점 (lat0, lon0) [deg] 에서 동쪽 east, 북쪽 north [m] 떨어진 점의 (lat, lon) [deg].
    구면 근사 (R = WGS84 적도 반경 6378137 m), 소수 8자리 (약 1 mm) 로 반올림."""
    R = 6378137.0
    lat = lat0 + math.degrees(north / R)
    lon = lon0 + math.degrees(east / (R * math.cos(math.radians(lat0))))
    return round(lat, 8), round(lon, 8)
