"""Hybrid A* 전역 경로 탐색 — ROS 의존성 없음.

왜 Hybrid A* 인가
  보통 A* 는 격자 칸 중심을 잇는 꺾인 선을 내놓아 자동차(Ackermann)가 따라갈 수 없다.
  Hybrid A* 는 칸마다 연속 자세 (x, y, yaw) 를 들고 다니고, 한 번에 '최소 회전반경을 지키는 짧은 원호/직선'
  (motion primitive) 만큼 전진하며 탐색한다. 그래서 나온 경로는 차가 실제로 따라갈 수 있다.
  목표 가까이서는 Dubins 경로(장애물 무시 최단 경로)를 바로 이어 보고, 부딪히지 않으면 거기서 끝낸다
  (analytic expansion) → 빈 헤드랜드에서는 거의 즉시 Dubins 경로가 답이 된다.

알고리즘 (Dolgov et al., 2010 을 단순화)
  1) 상태 = (x, y, yaw), 방문 표시는 칸 (x/xy_res, y/xy_res, yaw/yaw_res) 단위
  2) 확장: 곡률 {−κ, −κ/2, 0, +κ/2, +κ} (κ = 1/회전반경) 로 step [m] 전진 (전진만 — PX4 로버 Offboard 는 후진 안 씀)
  3) 비용 g = 이동 거리 + 곡선 벌점 + 방향 바꿈 벌점, 휴리스틱 h = max(직선 거리, Dubins 길이)
  4) 몇 번 확장할 때마다 현재 자세 → 목표 Dubins 경로(6가지 모두, 짧은 순)를 검사해 빈 것이 있으면 연결
  5) 점유 검사는 grid.OccupancyGrid (장애물을 로버 반경만큼 부풀린 격자) 로 경로 점마다

입력/출력
  hybrid_astar(start, goal, grid, radius) → (N×3) [x, y, yaw] 점열 (간격 약 0.1 m) 또는 None (못 찾음)

References
  - D. Dolgov, S. Thrun, M. Montemerlo, J. Diebel, "Path Planning for Autonomous Vehicles in Unknown
    Semi-structured Environments", International Journal of Robotics Research 29(5), 2010.
  - L. E. Dubins (1957), A. M. Shkel & V. Lumelsky (2001) — Dubins 경로 (orchard_navigation/headland.py)
"""
from __future__ import annotations

import heapq
import math

import numpy as np
from orchard_navigation.headland import _dubins_words, _mod2pi, _wrap

from .grid import OccupancyGrid


def sample_word(start, word: tuple, radius: float, step: float = 0.1) -> np.ndarray:
    """Dubins 경로 한 종류 (이름, t, p, q) 를 start 에서 step [m] 간격 점열 (N×3) 로 그린다."""
    x, y, yaw = start
    pts = [(x, y, yaw)]
    name, *lengths = word
    for seg, length in zip(name, lengths):
        L = length * radius
        n = max(1, int(math.ceil(L / step)))
        ds = L / n
        for _ in range(n):
            if seg == 'S':
                x += ds * math.cos(yaw)
                y += ds * math.sin(yaw)
            else:
                k = (1.0 if seg == 'L' else -1.0) / radius      # 원호를 정확히 적분 (누적 오차 없음)
                dyaw = k * ds
                x += (math.sin(yaw + dyaw) - math.sin(yaw)) / k
                y += (-math.cos(yaw + dyaw) + math.cos(yaw)) / k
                yaw += dyaw
            pts.append((x, y, _wrap(yaw)))
    return np.array(pts)


def dubins_words_sorted(start, goal, radius: float) -> list:
    """start → goal 의 Dubins 경로 종류들을 길이 순으로. 각 원소 (길이 [m], word)."""
    x0, y0, th0 = start
    x1, y1, th1 = goal
    dx, dy = x1 - x0, y1 - y0
    D = math.hypot(dx, dy)
    th = math.atan2(dy, dx) if D > 1e-9 else 0.0
    words = _dubins_words(D / radius, _mod2pi(th0 - th), _mod2pi(th1 - th))
    return sorted(((radius * (w[1] + w[2] + w[3]), w) for w in words), key=lambda a: a[0])


def dubins_length(start, goal, radius: float) -> float:
    """장애물을 무시한 Dubins 최단 길이 [m] (휴리스틱용)."""
    w = dubins_words_sorted(start, goal, radius)
    return w[0][0] if w else math.hypot(goal[0] - start[0], goal[1] - start[1])


def free_dubins(start, goal, grid: OccupancyGrid, radius: float, step: float = 0.1):
    """부딪히지 않는 가장 짧은 Dubins 경로 (N×3), 없으면 None."""
    for _length, word in dubins_words_sorted(start, goal, radius):
        path = sample_word(start, word, radius, step)
        if grid.path_free(path):
            return path
    return None


def _arc(x: float, y: float, yaw: float, k: float, length: float, ds: float = 0.1) -> np.ndarray:
    """곡률 k [1/m] 로 length [m] 전진한 점열 (시작점 제외, M×3)."""
    n = max(1, int(math.ceil(length / ds)))
    s = np.arange(1, n + 1) * (length / n)
    if abs(k) < 1e-9:
        return np.c_[x + s * math.cos(yaw), y + s * math.sin(yaw), np.full(n, yaw)]
    yy = yaw + k * s
    return np.c_[x + (np.sin(yy) - math.sin(yaw)) / k, y + (-np.cos(yy) + math.cos(yaw)) / k,
                 np.arctan2(np.sin(yy), np.cos(yy))]


def hybrid_astar(start, goal, grid: OccupancyGrid, radius: float, step: float = 0.4,
                 xy_res: float = 0.25, yaw_bins: int = 72, max_expansions: int = 20000,
                 analytic_every: int = 5, curve_penalty: float = 0.1, switch_penalty: float = 0.2):
    """start → goal (각 (x, y, yaw)) 의 전진 경로 (N×3) 또는 None.

    radius: 최소 회전반경 [m], step: 한 번 확장 길이 [m], xy_res / yaw_bins: 방문 표시 해상도,
    max_expansions: 탐색 한도 (넘으면 None), analytic_every: 몇 번 확장마다 Dubins 연결을 시도할지.
    """
    start = tuple(float(v) for v in start)
    goal = tuple(float(v) for v in goal)
    if grid.occupied(np.array([start[:2]]))[0] or grid.occupied(np.array([goal[:2]]))[0]:
        return None
    direct = free_dubins(start, goal, grid, radius)          # 대부분의 헤드랜드: 바로 해결
    if direct is not None:
        return direct
    kmax = 1.0 / radius
    curvs = (-kmax, -0.5 * kmax, 0.0, 0.5 * kmax, kmax)
    yaw_res = 2 * math.pi / yaw_bins

    def key(x, y, yaw):
        return (int(math.floor(x / xy_res)), int(math.floor(y / xy_res)), int(_mod2pi(yaw) // yaw_res))

    def h(x, y, yaw):
        return max(math.hypot(goal[0] - x, goal[1] - y), dubins_length((x, y, yaw), goal, radius))

    nodes = [(start, 0.0, -1, None, 0.0)]     # (자세, g, 부모 번호, 부모→여기 점열, 곡률)
    heap = [(h(*start), 0, 0)]
    closed = set()
    expansions = 0
    while heap and expansions < max_expansions:
        _f, _c, ni = heapq.heappop(heap)
        (x, y, yaw), g, _parent, _seg, k_prev = nodes[ni]
        kk = key(x, y, yaw)
        if kk in closed:
            continue
        closed.add(kk)
        expansions += 1
        if expansions % analytic_every == 0 or math.hypot(goal[0] - x, goal[1] - y) < 3 * radius:
            tail = free_dubins((x, y, yaw), goal, grid, radius)
            if tail is not None:
                return _backtrack(nodes, ni, tail)
        for k in curvs:
            seg = _arc(x, y, yaw, k, step)
            if grid.occupied(seg[:, :2]).any():
                continue
            nx_, ny_, nyaw = seg[-1]
            if key(nx_, ny_, nyaw) in closed:
                continue
            g2 = g + step * (1.0 + curve_penalty * abs(k) * radius) + switch_penalty * (k != k_prev)
            nodes.append(((nx_, ny_, nyaw), g2, ni, seg, k))
            heapq.heappush(heap, (g2 + h(nx_, ny_, nyaw), len(nodes) - 1, len(nodes) - 1))
    return None


def _backtrack(nodes: list, ni: int, tail: np.ndarray) -> np.ndarray:
    """마지막 노드 ni 에서 부모를 따라 올라가 점열을 잇고, Dubins 꼬리 tail 을 붙인다."""
    parts = [tail]
    while ni > 0:
        _pose, _g, parent, seg, _k = nodes[ni]
        parts.append(seg)
        ni = parent
    s = nodes[0][0]
    parts.append(np.array([s]))
    return np.vstack(parts[::-1])
