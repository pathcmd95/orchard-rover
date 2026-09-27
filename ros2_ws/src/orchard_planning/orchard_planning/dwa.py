"""지역 경로계획: Dynamic Window Approach (DWA) — ROS 의존성 없음.

무엇을 하나
  전역 경로(global_planner) 를 따라가되, LiDAR 로 본 장애물(obstacles.py)을 피하는 속도·회전 명령을 매 주기 고른다.

알고리즘 (Fox, Burgard, Thrun 1997 을 Ackermann 차에 맞게)
  1) 후보 만들기: 곡률 [−κmax, +κmax] 여러 개 (Ackermann 은 제자리 회전을 못 하므로 요레이트 대신 곡률을 고른다)
     각 후보 = '곡률 k1 로 arc1 [m] → 곡률 k2 로 arc2 [m]' 두 호를 이은 궤적. 둘째 호가 있어서
     '장애물을 비켰다가 경로로 돌아오는' S 자 궤적도 평가할 수 있다 (한 호만 쓰는 기본 DWA 는 좁은 통로에서 갇힘).
     궤적 길이는 속도와 무관하게 정한다 → 느리게 가고 있어도 앞의 장애물을 미리 보고 비킨다.
  2) 버리기: 궤적을 따라 차체(앞뒤로 늘어선 원 3개로 근사)가 장애물에 safety 보다 가까우면 탈락
  3) 비용 = w_path·(전역 경로와의 거리, 가까운 미래일수록 큰 가중 평균) + w_goal·(goal_lookahead 만큼 간 궤적 점 ↔ 경로 위 목표점 거리)
           + w_heading·(그 점의 방향 − 목표 방향) + w_obstacle·(장애물 근접) + w_smooth·(곡률 변화)
  4) 비용이 가장 작은 후보의 첫 곡률 k1 을 명령으로. 속도는 한계 속도에서 장애물 여유에 비례해 줄이고,
     dynamic window (한 제어 주기에 낼 수 있는 가·감속) 로 자른다 → cmd_vel (v, ω = v·k1)
  모든 후보가 탈락하면 먼저 둘째 호를 1.5 m, 0.5 m 로 줄여 다시 평가(멀리 막힌 것 때문에 지금 비킬 길까지 버리지 않게, 천천히).
  그래도 모두 탈락하면: 차체가 이미 safety 안에 들어와 있어서라면 지금보다 가까워지지 않는 궤적 중 비용이 가장 작은 것을
  escape_speed 로 (status 'escape' — 나무 옆에 붙어 영영 못 움직이는 교착 방지), 그런 궤적도 없으면 멈춘다 (status 'blocked').

속도 제한
  경로가 휘면(U턴) 가로 가속도 max_lat_accel 를 넘지 않게 v ≤ √(a_lat / κ_path), 경로 끝이 가까우면 감속.

References
  - D. Fox, W. Burgard, S. Thrun, "The Dynamic Window Approach to Collision Avoidance",
    IEEE Robotics & Automation Magazine 4(1), 1997.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


def _wrap(a):
    """각도 (배열 가능) 를 (−π, π] 로."""
    return np.arctan2(np.sin(a), np.cos(a))


@dataclass
class DWAParams:
    """DWA 파라미터 (local_planner_node 의 'dwa.*' ROS 파라미터). 거리 [m], 속도 [m/s], 시간 [s]."""
    max_speed: float = 0.8            # 최고 속도
    max_accel: float = 0.5            # 가속 한계 [m/s²]
    max_decel: float = 1.0            # 감속 한계 [m/s²]
    control_dt: float = 0.1           # 제어 주기 (dynamic window = 이 시간 안에 낼 수 있는 속도)
    min_turn_radius: float = 0.9      # 최소 회전반경 → 최대 곡률 1/이 값
    n_curvatures: int = 21            # 첫 호 곡률 후보 수
    n_curvatures2: int = 7            # 둘째 호 곡률 후보 수 (피한 뒤 경로로 돌아오는 S 자 궤적)
    arc1: float = 1.5                 # 첫 호 길이 [m] (속도 × 1.5 s 와 큰 쪽)
    arc2: float = 3.0                 # 둘째 호 길이 [m] (길수록 멀리 있는 장애물을 일찍 보고 비킴)
    ds: float = 0.1                   # 궤적 점 간격 [m]
    footprint_offsets: tuple = (-0.3, 0.0, 0.3)   # 차체를 덮는 원들의 중심 (base_link 앞뒤 x [m])
    footprint_radius: float = 0.35    # 그 원들의 반경 (차체 반폭 0.3 + 여유) — 원 하나로 덮으면 통로가 너무 좁아짐
    safety: float = 0.15              # 차체 원과 장애물 사이 최소 여유 [m] (이보다 가까운 궤적은 탈락)
    obstacle_influence: float = 0.8   # 이 여유 안이면 가까울수록 비용·감속
    max_lat_accel: float = 0.15       # 휜 경로에서 가로 가속도 한계 [m/s²] (U턴 속도 ≈ 0.5 m/s)
    goal_lookahead: float = 1.5       # 목표점 = 경로 위 가장 가까운 점에서 이만큼 앞 [m] (작을수록 경로로 빨리 붙음)
    path_window: float = 8.0          # 비용 계산에 쓰는 경로 앞 길이
    path_decay: float = 1.5           # 경로 거리 비용의 가중치를 궤적 앞쪽에 더 줌: exp(−호길이/이 값) [m]
    w_path: float = 1.0               # 경로와의 평균 거리
    w_goal: float = 1.0               # goal_lookahead 지점 ↔ 경로 위 목표점 거리
    w_heading: float = 0.5            # 그 지점 방향 ↔ 목표 방향
    w_obstacle: float = 1.0           # 장애물 근접
    w_smooth: float = 0.2             # 지난 주기 곡률과의 차이 (흔들림 줄이기)
    escape_speed: float = 0.2         # 빠져나오기: 이미 safety 안으로 들어와 모든 후보가 탈락했을 때 이 속도로
    escape_margin: float = 0.03       # 지금 여유보다 이만큼 이상 가까워지지 않는 궤적만 허용 (멀어지는 쪽)


@dataclass
class DWAResult:
    """plan() 결과."""
    v: float                          # 선속도 [m/s]
    w: float                          # 요레이트 [rad/s] (+ 좌회전)
    status: str                       # 'ok' | 'short'(짧은 후보로) | 'escape' | 'blocked' | 'done' | 'no_path'
    best: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))   # 고른 궤적 (odom)
    candidates: list = field(default_factory=list)   # [(궤적 K×3, 비용, 통과여부)]
    speed_limit: float = 0.0
    clearance: float = math.inf       # 고른 궤적의 장애물 여유 [m]


def rollout2(x: float, y: float, yaw: float, k1: np.ndarray, k2: np.ndarray, L1: float, L2: float,
             ds: float = 0.1) -> np.ndarray:
    """곡률 k1 로 L1 [m], 이어서 k2 로 L2 [m] 달린 궤적들 (C×T×3). k1, k2 는 같은 길이 (C,). 원호를 정확히 적분."""
    def arcs(x0, y0, th0, k, L):
        s = np.arange(1, int(round(L / ds)) + 1) * ds          # (T,)
        kk = k[:, None]
        straight = np.abs(kk) < 1e-6
        ksafe = np.where(straight, 1.0, kk)
        th = th0[:, None] + kk * s[None, :]
        X = np.where(straight, x0[:, None] + s * np.cos(th0)[:, None],
                     x0[:, None] + (np.sin(th) - np.sin(th0)[:, None]) / ksafe)
        Y = np.where(straight, y0[:, None] + s * np.sin(th0)[:, None],
                     y0[:, None] + (-np.cos(th) + np.cos(th0)[:, None]) / ksafe)
        return np.stack([X, Y, th], axis=-1)
    C = len(k1)
    a = arcs(np.full(C, x), np.full(C, y), np.full(C, yaw), k1, L1)
    b = arcs(a[:, -1, 0], a[:, -1, 1], a[:, -1, 2], k2, L2)
    t = np.concatenate([a, b], axis=1)
    t[..., 2] = _wrap(t[..., 2])
    return t


class DWAPlanner:
    """plan(자세, 현재 속도, 전역 경로, 장애물) → DWAResult."""

    def __init__(self, p: DWAParams):
        """파라미터 보관."""
        self.p = p
        self.last_k = 0.0

    def plan(self, x: float, y: float, yaw: float, v_cur: float, path: np.ndarray,
             obstacles: np.ndarray) -> DWAResult:
        """x, y, yaw: 로버 (odom), v_cur: 현재 속도, path: 전역 경로 (N×3, odom), obstacles: 장애물 xy (M×2, odom)."""
        p = self.p
        if path is None or len(path) < 2:
            return DWAResult(0.0, 0.0, 'no_path')
        seg = np.hypot(np.diff(path[:, 0]), np.diff(path[:, 1]))
        s_path = np.concatenate([[0.0], np.cumsum(seg)])
        i0 = int(np.argmin(np.hypot(path[:, 0] - x, path[:, 1] - y)))
        remaining = float(s_path[-1] - s_path[i0])
        if remaining < 0.15:
            return DWAResult(0.0, 0.0, 'done')
        win = path[(s_path >= s_path[i0]) & (s_path <= s_path[i0] + p.path_window)]
        # 속도 한계: 앞 2 m 경로의 곡률 (가로 가속도), 남은 거리 (멈출 수 있게)
        ahead = path[(s_path >= s_path[i0]) & (s_path <= s_path[i0] + 2.0)]
        v_lim = p.max_speed
        if len(ahead) > 2:
            L = max(float(np.sum(np.hypot(np.diff(ahead[:, 0]), np.diff(ahead[:, 1])))), 1e-3)
            kappa = float(np.abs(np.sum(_wrap(np.diff(ahead[:, 2]))))) / L
            if kappa > 1e-3:
                v_lim = min(v_lim, math.sqrt(p.max_lat_accel / kappa))
        v_lim = min(v_lim, math.sqrt(2 * p.max_decel * max(remaining - 0.1, 0.0)) + 0.05)
        # 후보 궤적 (속도와 무관한 길이로 굴린다 — 느릴 때도 앞의 장애물을 보고 미리 비킨다)
        kmax = 1.0 / p.min_turn_radius
        L1 = max(p.arc1, v_cur * 1.5)
        target_s = s_path[i0] + p.goal_lookahead
        j = min(int(np.searchsorted(s_path, target_s)), len(path) - 1)
        tx, ty, tyaw = path[j]
        c, s_ = math.cos(yaw), math.sin(yaw)
        lx, ly = c * (tx - x) + s_ * (ty - y), -s_ * (tx - x) + c * (ty - y)
        k_pp = float(np.clip(2 * ly / max(lx * lx + ly * ly, 1e-6), -kmax, kmax))   # Pure Pursuit 곡률
        k1s = np.unique(np.r_[np.linspace(-kmax, kmax, p.n_curvatures), k_pp])
        obs = np.zeros((0, 2))
        if obstacles is not None and len(obstacles):
            obs = np.asarray(obstacles, float).reshape(-1, 2)
            obs = obs[np.hypot(obs[:, 0] - x, obs[:, 1] - y) < L1 + p.arc2 + 1.5]
        clear0 = math.inf                                   # 지금 자세의 여유
        if len(obs):
            clear0 = min(float(np.min(np.hypot(x + off * c - obs[:, 0], y + off * s_ - obs[:, 1])))
                         for off in p.footprint_offsets) - p.footprint_radius
        # 둘째 호 길이를 줄여 가며 시도: 멀리(3 m) 막힌 것 때문에 지금 움직일 수 있는 길까지 버리지 않게
        # (예: 운반차 앞에서 4.5 m 후보가 모두 나무·차에 걸려도 짧은 후보로 비키며 다시 계획)
        horizons = [(p.arc2, p.n_curvatures2), (0.5 * p.arc2, 11), (0.5, 11)]
        for hi, (arc2, nk2) in enumerate(horizons):
            k2s = np.linspace(-kmax, kmax, nk2)
            K1, K2 = np.meshgrid(k1s, k2s, indexing='ij')
            K1, K2 = K1.ravel(), K2.ravel()
            trajs = rollout2(x, y, yaw, K1, K2, L1, arc2, p.ds)
            C, T, _ = trajs.shape
            pts = trajs[:, :, :2].reshape(-1, 2)
            d_path = np.min(np.hypot(pts[:, None, 0] - win[None, :, 0], pts[:, None, 1] - win[None, :, 1]), 1)
            wts = np.exp(-np.arange(1, T + 1) * p.ds / p.path_decay)       # 가까운 미래의 경로 이탈을 더 크게
            cost_path = (d_path.reshape(C, T) * wts).sum(1) / wts.sum()
            ig = min(int(round(p.goal_lookahead / p.ds)) - 1, T - 1)          # 궤적에서 목표점과 같은 호 길이의 점
            mid = trajs[:, ig, :]
            cost_goal = np.hypot(mid[:, 0] - tx, mid[:, 1] - ty)
            cost_head = np.abs(_wrap(mid[:, 2] - tyaw))
            clear = np.full(C, math.inf)
            if len(obs):
                th = trajs[:, :, 2].reshape(-1)
                dmin = np.full(len(pts), np.inf)
                for off in p.footprint_offsets:                 # 차체를 덮는 원마다 장애물까지 거리
                    cx, cy = pts[:, 0] + off * np.cos(th), pts[:, 1] + off * np.sin(th)
                    d = np.hypot(cx[:, None] - obs[None, :, 0], cy[:, None] - obs[None, :, 1])
                    dmin = np.minimum(dmin, np.min(d, 1))
                clear = dmin.reshape(C, T).min(1) - p.footprint_radius
            ok = clear > p.safety
            escape = False
            if not ok.any() and clear0 <= p.safety:             # 이미 safety 안: 멀어지는 궤적으로 빠져나오기
                ok = (clear >= clear0 - p.escape_margin) & (clear > 0.02)
                escape = bool(ok.any())
            if ok.any():
                break
        short = hi > 0 and not escape
        if short:                                           # 짧은 후보로 가는 중: 천천히
            v_lim = min(v_lim, max(0.2, p.max_speed * arc2 / p.arc2))
        cost_obs = np.clip(p.obstacle_influence - clear, 0.0, None) / p.obstacle_influence
        cost = (p.w_path * cost_path + p.w_goal * cost_goal + p.w_heading * cost_head
                + p.w_obstacle * cost_obs + p.w_smooth * np.abs(K1 - self.last_k))
        cands = [(trajs[i], float(cost[i]), bool(ok[i])) for i in range(C)]
        if not ok.any():
            return DWAResult(0.0, 0.0, 'blocked', candidates=cands, speed_limit=v_lim)
        i = int(np.argmin(np.where(ok, cost, np.inf)))
        # 속도: 한계 속도, 장애물이 가까우면 비례 감속, dynamic window (가·감속 한계) 로 자름
        v_des = v_lim * float(np.clip((clear[i] - p.safety) / p.obstacle_influence, 0.3, 1.0))
        if escape:
            v_des = min(v_lim, p.escape_speed)
        v = float(np.clip(v_des, v_cur - p.max_decel * p.control_dt, v_cur + p.max_accel * p.control_dt))
        v = max(v, 0.0)
        k = float(K1[i])
        self.last_k = k
        return DWAResult(v, v * k, 'escape' if escape else ('short' if short else 'ok'), trajs[i], cands, v_lim,
                         float(clear[i]))
