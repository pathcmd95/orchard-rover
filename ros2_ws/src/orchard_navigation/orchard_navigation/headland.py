"""행 끝(헤드랜드) U턴 계획 — ROS 의존성 없음.

파이프라인에서의 역할
  행 추종 → 행 끝 → [U턴 ← 이 모듈] → 다음 통로 진입.
  mission.OrchardMission._plan_turn() 이 EXIT_ROW 가 끝날 때 호출하고, TURN 상태에서 PathTracker.command() 를
  매 주기 부른다 (turn_mode='planned' 일 때).

기존 방식: 행 간격/2 반경의 반원을 요 각도만 보고 돈다 (센서 없이, 간격·미끄러짐 오차가 그대로 입구 오차가 됨).
이 모듈:
  1) 행 끝에서 LiDAR 로 본 줄기로 다음 통로를 직접 잡는다
     - 돌 방향 쪽 가까운 열(지금 통로의 한쪽 열)과 그 너머 열의 횡위치 → 두 열의 가운데 = 다음 통로 중심
     - 두 열의 마지막 나무 → 통로 입구 위치
  2) 현재 자세 → 입구 자세(반대 방향)로 Dubins 경로(전진만, 최소 회전반경 지정)를 만든다
  3) 경로를 Pure Pursuit 로 따라간다 (odom 폐루프라 미끄러짐·조향 오차를 보정)

좌표: 'row frame' = 행 끝 판정 순간의 로버 자세 기준 (u = 행 방향 앞, v = 왼쪽).
      dubins_path / PathTracker 는 odom 좌표 (x, y [m], yaw [rad]) 를 쓴다.

관련 파라미터 (mission.MissionParams, ROS 이름 'mission.*' — sim.yaml / robot.yaml 의 row_navigator_node)
  - mission.row_spacing (줄기를 못 볼 때 쓰는 열 간격), mission.turn_radius, mission.entry_margin,
    follow.min_turn_radius (Dubins 최소 반경·추종 곡률 한계)
  - turn_lookahead, max_turn_deviation 은 ROS 파라미터로 노출되지 않음 → MissionParams 기본값 수정.

References
  L. E. Dubins, "On Curves of Minimal Length with a Constraint on Average Curvature, and with Prescribed
    Initial and Terminal Positions and Tangents", American Journal of Mathematics 79(3), 1957.
  A. M. Shkel & V. Lumelsky, "Classification of the Dubins set", Robotics and Autonomous Systems 34, 2001
    (_dubins_words 의 닫힌 식 LSL/RSR/LSR/RSL/RLR/LRL).
  R. C. Coulter, "Implementation of the Pure Pursuit Path Tracking Algorithm", CMU-RI-TR-92-01, 1992
    (PathTracker).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def _mod2pi(a: float) -> float:
    """각도를 [0, 2π) 로."""
    return a % (2.0 * math.pi)


def _wrap(a: float) -> float:
    """각도를 (−π, π] 로."""
    return math.atan2(math.sin(a), math.cos(a))


# ---------------------------------------------------------------- Dubins
def _dubins_words(d: float, a: float, b: float):
    """정규화된 Dubins (시작 (0,0,a) → 끝 (d,0,b), 반경 1). (이름, t, p, q) 목록.

    Shkel & Lumelsky (2001) 의 닫힌 식. 시작점을 원점, 끝점을 x 축 위 (d, 0) 로 옮기고 반경 1 로 정규화했다.
    Args:
        d: 시작~끝 거리 / 회전반경 (무차원).
        a, b: 시작/끝 방향 − 시작→끝 직선 방향 [rad], [0, 2π).
    Returns:
        가능한 경로 종류(word)마다 (이름, t, p, q). L = 좌회전 호, R = 우회전 호, S = 직선.
        t, p, q 는 세 구간 길이 (호는 회전각 [rad] = 반경 1 에서의 호 길이, 직선은 거리/반경).
        식의 근호 안이 음수(p2 < 0)거나 acos 인자가 |·|>1 이면 그 종류는 존재하지 않아 빠진다.
    """
    sa, sb, ca, cb = math.sin(a), math.sin(b), math.cos(a), math.cos(b)
    cab = math.cos(a - b)
    out = []
    p2 = 2 + d * d - 2 * cab + 2 * d * (sa - sb)                 # LSL
    if p2 >= 0:
        tmp = math.atan2(cb - ca, d + sa - sb)
        out.append(('LSL', _mod2pi(-a + tmp), math.sqrt(p2), _mod2pi(b - tmp)))
    p2 = 2 + d * d - 2 * cab + 2 * d * (sb - sa)                 # RSR
    if p2 >= 0:
        tmp = math.atan2(ca - cb, d - sa + sb)
        out.append(('RSR', _mod2pi(a - tmp), math.sqrt(p2), _mod2pi(-b + tmp)))
    p2 = -2 + d * d + 2 * cab + 2 * d * (sa + sb)                # LSR
    if p2 >= 0:
        p = math.sqrt(p2)
        tmp = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
        out.append(('LSR', _mod2pi(-a + tmp), p, _mod2pi(-_mod2pi(b) + tmp)))
    p2 = d * d - 2 + 2 * cab - 2 * d * (sa + sb)                 # RSL
    if p2 >= 0:
        p = math.sqrt(p2)
        tmp = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
        out.append(('RSL', _mod2pi(a - tmp), p, _mod2pi(b - tmp)))
    tmp = (6 - d * d + 2 * cab + 2 * d * (sa - sb)) / 8          # RLR (가운데 호의 cos 값)
    if abs(tmp) <= 1:
        p = _mod2pi(2 * math.pi - math.acos(tmp))
        t = _mod2pi(a - math.atan2(ca - cb, d - sa + sb) + p / 2)
        out.append(('RLR', t, p, _mod2pi(a - b - t + p)))
    tmp = (6 - d * d + 2 * cab + 2 * d * (sb - sa)) / 8          # LRL
    if abs(tmp) <= 1:
        p = _mod2pi(2 * math.pi - math.acos(tmp))
        t = _mod2pi(-a - math.atan2(ca - cb, d + sa - sb) + p / 2)
        out.append(('LRL', t, p, _mod2pi(b - a - t + p)))
    return out


def dubins_path(start, goal, radius: float, step: float = 0.05) -> np.ndarray:
    """전진만 하는 최단 Dubins 경로 (x, y, yaw) 점열. start/goal = (x, y, yaw).

    Args:
        start, goal: (x [m], y [m], yaw [rad]) — odom 좌표.
        radius: 회전반경 [m] (차의 최소 회전반경 이상이어야 따라갈 수 있다).
        step: 점 간격 [m] (0.05 m → PathTracker 의 60 점 탐색창 = 약 3 m).
    Returns:
        (N, 3) 배열 [x, y, yaw]. 첫 점 = start, 마지막 점 ≈ goal.
    Raises:
        ValueError: 가능한 경로가 하나도 없을 때 (수치적으로 드묾).
    """
    x0, y0, th0 = start
    x1, y1, th1 = goal
    dx, dy = x1 - x0, y1 - y0
    D = math.hypot(dx, dy)
    d = D / radius                                       # 반경 1 로 정규화한 거리
    th = math.atan2(dy, dx) if D > 1e-9 else 0.0         # 시작→끝 직선 방향 (정규화 좌표의 x 축)
    a, b = _mod2pi(th0 - th), _mod2pi(th1 - th)
    words = _dubins_words(d, a, b)
    if not words:
        raise ValueError('Dubins 경로 없음')
    name, t, p, q = min(words, key=lambda w: w[1] + w[2] + w[3])   # 총 길이가 가장 짧은 종류
    pts = [(x0, y0, th0)]
    x, y, yaw = x0, y0, th0
    for seg, length in zip(name, (t, p, q)):
        L = length * radius                              # 정규화 길이 → 실제 길이 [m]
        n = max(1, int(math.ceil(L / step)))
        ds = L / n
        for _ in range(n):
            if seg == 'S':
                x += ds * math.cos(yaw)
                y += ds * math.sin(yaw)
            else:
                # 곡률 k 인 원호를 ds 만큼 정확히 적분 (오일러 적분처럼 누적 오차가 생기지 않음)
                k = (1.0 if seg == 'L' else -1.0) / radius
                dyaw = k * ds
                x += (math.sin(yaw + dyaw) - math.sin(yaw)) / k
                y += (-math.cos(yaw + dyaw) + math.cos(yaw)) / k
                yaw += dyaw
            pts.append((x, y, _wrap(yaw)))
    return np.array(pts)


def path_length(path: np.ndarray) -> float:
    """점열 경로 (N×≥2, [x, y, ...]) 의 길이 [m]."""
    return float(np.sum(np.hypot(np.diff(path[:, 0]), np.diff(path[:, 1])))) if len(path) > 1 else 0.0


# ---------------------------------------------------------------- 다음 통로 추정
@dataclass
class NextLane:
    """estimate_next_lane 결과 (row frame 기준)."""

    v: float                 # 다음 통로 중심 횡위치 [m] (row frame)
    u_end: float             # 통로 입구 (두 열 마지막 나무 중 먼 쪽) [m]
    spacing: float           # 측정한 열 간격 (못 재면 파라미터 값)
    near_count: int
    far_count: int
    source: str              # 'lidar2' (두 열) / 'lidar1' (가까운 열만) / 'nominal'


def estimate_next_lane(trees_uv: np.ndarray, turn_sign: float, row_spacing: float,
                       u_min: float = -12.0, u_max: float = 3.0, min_trees: int = 2) -> NextLane:
    """row frame 줄기 좌표 (N×2: u, v) 에서 돌 방향 쪽 다음 통로를 찾는다.

    지금 통로 중심이 v = 0 이라면 (열 간격 S)
      - 가까운 열 (지금 통로의 돌 방향 쪽 열): 명목상 v = s·0.5S
      - 너머 열 (다음 통로의 먼 쪽 열):       명목상 v = s·1.5S
      - 다음 통로 중심 = 두 열의 가운데:        명목상 v = s·S
    Args:
        trees_uv: 최근 관측한 줄기 좌표 [m] (row frame, u 앞 / v 왼쪽).
        turn_sign: +1 = 왼쪽으로 U턴, −1 = 오른쪽.
        row_spacing: 파라미터 열 간격 S [m] (탐색 창 크기와 못 볼 때의 대체값).
        u_min, u_max: 이 u 범위 [m] 의 줄기만 쓴다 (행 끝 12 m 뒤 ~ 3 m 앞: 행 끝 근처만).
        min_trees: 열로 인정할 최소 줄기 수.
    Returns:
        NextLane. source 가 'lidar2' 면 두 열 모두 봄 (간격 실측), 'lidar1' 은 가까운 열만,
        'nominal' 은 아무것도 못 봐서 파라미터 간격만 사용.
    """
    S, s = row_spacing, turn_sign
    P = np.asarray(trees_uv, float).reshape(-1, 2)
    P = P[(P[:, 0] > u_min) & (P[:, 0] < u_max)]
    sv = s * P[:, 1]                                                  # 돌 방향을 + 로 뒤집은 횡위치
    near = P[(sv > 0.2 * S) & (sv < 0.9 * S)]                         # 명목 0.5S 둘레 창
    near_v = float(np.median(near[:, 1])) if len(near) >= min_trees else s * 0.5 * S
    far_lo, far_hi = s * near_v + 0.6 * S, s * near_v + 1.4 * S   # 가까운 열 너머 한 칸
    far = P[(sv > far_lo) & (sv < far_hi)]
    ends = []
    if len(near) >= min_trees:
        ends.append(float(near[:, 0].max()))
    if len(far) >= min_trees:
        far_v = float(np.median(far[:, 1]))
        ends.append(float(far[:, 0].max()))
        spacing = abs(far_v - near_v)
        v = 0.5 * (near_v + far_v)
        src = 'lidar2'
    elif len(near) >= min_trees:
        spacing, v, src = S, near_v + s * 0.5 * S, 'lidar1'
    else:
        spacing, v, src = S, s * S, 'nominal'
    u_end = max(ends) if ends else 0.0             # 못 보면 행 끝 판정 위치(u=0)를 입구로
    return NextLane(v, u_end, spacing, len(near), len(far), src)


# ---------------------------------------------------------------- 경로 추종
class PathTracker:
    """Pure Pursuit (전진). 경로 끝에 가까워지면 done.

    경로(odom 좌표 점열) 위에서 로봇에 가장 가까운 점을 찾고, 거기서 호 길이 lookahead 만큼 앞의 점을
    목표로 곡률 2·ly / L² (Coulter 1992) 을 낸다. 끝남 판정은 호출자가 remaining 으로 한다.
    """

    def __init__(self, path: np.ndarray, lookahead: float = 1.0, max_curvature: float = 1.0):
        """Args:
            path: (N, 3) [x, y, yaw] 경로 (dubins_path 결과, 점 간격 약 0.05 m).
            lookahead: 전방 주시 호 길이 [m] (MissionParams.turn_lookahead).
            max_curvature: 곡률 한계 [1/m] (= 1 / 최소 회전반경).
        """
        self.path = path
        # 각 점까지의 누적 호 길이 s [m]
        self.s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(path[:, 0]), np.diff(path[:, 1])))])
        self.lookahead = lookahead
        self.max_k = max_curvature
        self.i = 0                   # 현재 가장 가까운 경로 점 번호 (앞으로만 증가)
        self.cross_track = 0.0       # 경로와의 최근 거리 [m] (안전정지 판정용)

    @property
    def remaining(self) -> float:
        """가장 가까운 점에서 경로 끝까지 남은 호 길이 [m]."""
        return float(self.s[-1] - self.s[self.i])

    def command(self, x: float, y: float, yaw: float, v: float) -> tuple[float, float]:
        """현재 자세 (odom: x, y [m], yaw [rad]) 와 속도 v [m/s] → (v [m/s], w [rad/s], +w = 좌회전)."""
        P = self.path
        hi = min(len(P), self.i + 60)                         # 앞쪽 3 m 안에서만 가장 가까운 점 (되돌아감 방지)
        d = np.hypot(P[self.i:hi, 0] - x, P[self.i:hi, 1] - y)
        self.i += int(np.argmin(d))
        self.cross_track = float(d.min())
        target_s = self.s[self.i] + self.lookahead
        j = int(np.searchsorted(self.s, target_s))
        if j >= len(P):                                         # 끝 너머는 끝 방향으로 연장
            ex, ey, eyaw = P[-1]
            extra = target_s - self.s[-1]
            tx, ty = ex + extra * math.cos(eyaw), ey + extra * math.sin(eyaw)
        else:
            tx, ty = P[j, 0], P[j, 1]
        # 목표점을 로봇 좌표(lx 앞, ly 왼쪽)로 회전
        c, s_ = math.cos(yaw), math.sin(yaw)
        lx = c * (tx - x) + s_ * (ty - y)
        ly = -s_ * (tx - x) + c * (ty - y)
        L2 = max(lx * lx + ly * ly, 1e-6)
        k = max(-self.max_k, min(self.max_k, 2.0 * ly / L2))  # Pure Pursuit 곡률, 차의 한계로 자름
        return v, v * k
