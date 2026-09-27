"""전역 경로계획: 과수원 통로 순회 (커버리지) + 헤드랜드 U턴 Hybrid A* — ROS 의존성 없음.

무엇을 하나
  로버가 출발한 통로(통로 0)부터 돌 방향(first_turn) 쪽으로 통로를 지그재그로 모두 도는 전역 경로를 만든다.
    통로 k 직선 (중심선)  →  행 끝 U턴 (Hybrid A*, 나무를 피하고 최소 회전반경 지킴)  →  통로 k+1 직선 (반대 방향) → …
  과수원 지도를 미리 몰라도 되도록, 달리면서 본 줄기로 열·통로 모델(orchard_model.py)을 계속 고치고
  경로를 매 주기(약 2 Hz) 다시 만든다. 아직 끝을 못 본 통로는 로버 앞 ahead [m] 까지만 잠정 경로로 둔다.
  지역 계획(dwa.py)이 이 경로를 따라가며 장애물을 피한다.

진행 상태 (/orchard/mission/state 로 발행 — 기존 row_navigator 와 같은 이름을 써서 지도 노드들과 호환)
  IDLE → (APPROACH: 과수원 밖에서 입구 찾아 정렬, approach 가 켜졌을 때) → FOLLOW_ROW(통로 k) → TURN(k → k+1) → … → DONE
    - 통로 끝 확정: 로버가 열의 마지막 줄기에 end_confirm_dist [m] 안으로 다가왔는데 더 먼 줄기가 안 보이면
    - TURN 시작: 통로 끝 (마지막 줄기 + exit_margin) 에 닿으면. 마지막 통로면 DONE
    - TURN 끝: U턴 경로 끝에 닿거나, 거의 돌았고(handover_angle) LiDAR 가 새 통로 중심을 보면
    - 탐사 (lanes = 0): 다음 통로의 먼 쪽 열이 안 보이면 지금 통로가 마지막 → 끝에서 DONE

odom 흐름 보정 (drift)
  PX4 odom 은 U턴 중 옆으로 0.5~1 m 흐른다 (Gazebo 확인). 통로 안에서 LiDAR 가 본 통로 중심(/orchard/row)과
  모델의 통로 중심이 다르면 그 차이를 odom 흐름으로 보고 천천히(drift_gain) 추정한다.
  model 좌표 = odom − drift. 경로는 model 에서 만들고 odom 으로 바꿔(+drift) 내보낸다.

References
  - H. Choset, "Coverage of Known Spaces: The Boustrophedon Cellular Decomposition", Autonomous Robots 9, 2000
    (통로를 지그재그로 덮는 순회 — 과수원은 통로가 이미 '셀' 이라 순서만 정하면 된다)
  - D. Dolgov et al., IJRR 29(5), 2010 — Hybrid A* (hybrid_astar.py)
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from orchard_navigation.headland import _wrap

from .approach import Entrance, detect_entrance
from .grid import OccupancyGrid
from .hybrid_astar import hybrid_astar, sample_word, dubins_words_sorted
from .orchard_model import Lane, OrchardModel

IDLE, APPROACH, FOLLOW_ROW, TURN, DONE, STOPPED = 'IDLE', 'APPROACH', 'FOLLOW_ROW', 'TURN', 'DONE', 'STOPPED'


@dataclass
class PlannerParams:
    """전역 계획 파라미터 (global_planner_node 의 'plan.*' ROS 파라미터로 채워진다). 거리 [m], 각도 [rad]."""
    lanes: int = 3                    # 돌 통로 수. 0 = 탐사 (끝 열까지)
    max_lanes: int = 40               # 탐사 안전 한도
    first_turn: str = 'left'          # 첫 U턴 방향 left | right
    row_spacing: float = 3.8          # 파라미터 열 간격 (측정 전·못 볼 때)
    min_turn_radius: float = 0.9      # 차의 최소 회전반경 (U턴 반경 = max(이 값, 측정 간격/2))
    exit_margin: float = 0.8          # 통로 끝 = 마지막 줄기 + 이만큼 (여기서 U턴 시작)
    entry_margin: float = 0.8         # 다음 통로 입구 = 그 통로 마지막 줄기 + 이만큼 바깥
    end_confirm_dist: float = 5.0     # 마지막 줄기까지 이 거리 안이면 '더 없음' 을 믿고 통로 끝 확정
    ahead: float = 10.0               # 끝을 모르는 통로의 잠정 경로 길이 (로버 앞)
    bootstrap_dist: float = 3.0       # 출발 후 이 거리는 LiDAR 통로 중심선을 바로 따라감 (PX4 요가 정지 중 틀어져 있다 달리면서 바로잡힘)
    min_row_trees: int = 3            # 탐사: 다음 통로 먼 쪽 열로 인정할 최소 줄기 수
    tree_radius: float = 0.25         # 줄기 + 낮은 가지 반경 (U턴 경로 충돌 검사)
    robot_radius: float = 0.45        # 로버를 감싸는 원 반경
    drift_gain: float = 0.2           # odom 흐름 추정 갱신 비율 (주기마다)
    yaw_gain: float = 0.2             # odom 요 오차 추정 갱신 비율 (주기마다)
    yaw_step_max: float = 0.02        # 요 오차 한 주기 최대 변화 [rad] (5 Hz 면 약 6°/s)
    track_heading_tol: float = 0.14   # 로버 방향이 통로 방향과 이 각도 [rad] 안일 때만 요 오차·흐름·줄기·열 방향 갱신 (장애물 비키는 중엔 멈춤)
    track_lateral_tol: float = 0.5    # 요 오차·줄기 누적: 로버가 통로 중심에서 이 거리 [m] 안일 때만
    heading_baseline: float = 4.0     # 열 방향 추정: 통로 중심점들이 이 길이 [m] 이상 퍼졌을 때부터
    heading_max_change: float = 0.14  # 열 방향: 출발 이동 방향에서 최대 이만큼 [rad] 까지만 고침
    handover_angle: float = 0.5       # U턴 중 새 통로 방향과 이 각도 안이면 LiDAR 통로 중심으로 넘겨받기
    reach_tol: float = 0.8            # U턴 경로 끝 도달 판정 거리
    max_turn_deviation: float = 1.5   # U턴 경로에서 이만큼 벗어나면 현재 자세에서 다시 계획
    path_step: float = 0.1            # 경로 점 간격
    approach: bool = False            # True: 과수원 밖에서 출발 → LiDAR 로 끝 통로 입구를 찾아 정렬한 뒤 통로 0 시작 (approach.py)
    approach_side: str = 'auto'       # 들어갈 끝 통로 right | left | auto (auto: 첫 U턴 left 면 오른쪽 끝, right 면 왼쪽 끝)
    approach_pre: float = 3.0         # 정렬점 = 열 시작 앞 이 거리 [m] (여기서 통로를 똑바로 보고 평소 출발 절차 시작)
    approach_radius: float = 2.0      # 진입 경로 회전반경 [m] (최소 회전반경보다 크게 → 부드럽게)
    approach_memory: float = 1.5      # 입구 찾기에 쓰는 줄기 관측 기억 시간 [s] (달리면서 PX4 요 오차가 바뀌므로 짧게)
    approach_max_dist: float = 25.0   # 이만큼 달려도 입구를 못 찾으면 멈춤 [m]


@dataclass
class RowSeen:
    """LiDAR 통로 중심선 관측 (orchard_msgs/RowCenterline 에서 필요한 것만)."""
    valid: bool
    offset: float                     # base_link x=0 에서 중심선 y [m]
    heading: float                    # 중심선 방향 − 로버 방향 [rad]
    confidence: float
    left_count: int = 0
    right_count: int = 0


@dataclass
class PlanOutput:
    """update() 결과. 좌표는 모두 odom."""
    state: str
    lanes_done: int
    reason: str
    path: np.ndarray                  # (N×3) 남은 전역 경로 [x, y, yaw]
    lane: int = 0
    drift: tuple = (0.0, 0.0)
    yaw_bias: float = 0.0             # 추정한 odom 요 오차 [rad] (참 요 ≈ odom 요 + 이 값)
    rows: list = field(default_factory=list)     # [(p0 xy, p1 xy)] 열 선분 (표시용)
    lane_lines: list = field(default_factory=list)   # [(p0, p1, k)] 통로 중심선 (표시용)
    trees: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    turn_grid: OccupancyGrid | None = None


class OrchardPlanner:
    """전역 계획기. start() 후 update() 를 주기적으로 부른다."""

    def __init__(self, p: PlannerParams):
        """파라미터 보관. 상태 IDLE."""
        self.p = p
        self.state = IDLE
        self.reason = ''
        self.k = 0
        self.lanes_done = 0
        self.model: OrchardModel | None = None
        self.drift = np.zeros(2)
        self.last_err = 0.0
        self.turn_path: np.ndarray | None = None      # model 좌표
        self.turn_goal = None
        self.turn_i = 0
        self.turn_grid: OccupancyGrid | None = None
        self.last_heading_refine = -1e9
        self.centers: list = []                       # 통로 0 에서 본 LiDAR 통로 중심점 (odom) → 열 방향 추정
        self.tracking = False                         # 로버가 통로를 따라 곧게 달리는 중인가 (갱신 허용)
        self.last_turn_plan = -1e9
        self.end_confirmed = False
        # odom 요 오차 [rad] (참 요 ≈ odom 요 + 이 값). 출발 구간 끝에 '이동 방향 − 요' 로 정하고,
        # 통로 안에서는 LiDAR 가 본 통로 방향과 모델 통로 방향의 차이로 계속 고친다 (PX4 가 요를 바로잡는 것도 따라감)
        self.yaw_bias = 0.0

    # ------------------------------------------------------------ 시작/정지
    def start(self, now: float, x: float, y: float, yaw: float) -> None:
        """미션 시작. approach 면 APPROACH(입구 찾기)부터, 아니면 바로 통로 0 (_begin_lanes)."""
        if self.p.approach:
            self.model = None
            self.yaw_bias = 0.0
            self.drift = np.zeros(2)
            self.state, self.reason, self.k, self.lanes_done = APPROACH, '입구 찾는 중', 0, 0
            self.app_mem: list = []                 # [(시각, 줄기 odom N×2)]
            self.app_ent: Entrance | None = None   # 지금 목표 입구 (걸러 낸 값)
            self.app_path: np.ndarray | None = None
            self.app_cand, self.app_cand_n = None, 0
            self.app_plan_t = -1e9
            self.app_start = np.array([x, y], float)
            self.turn_grid = None
            return
        self._begin_lanes(now, x, y, yaw)

    def _begin_lanes(self, now: float, x: float, y: float, yaw: float) -> None:
        """통로 0 주행 시작. 처음 bootstrap_dist [m] 는 LiDAR 중심선을 바로 따라가고, 그 뒤 모델을 만든다.

        왜: PX4 는 정지 중 요(yaw)가 약 15° 틀어져 있다가 달리기 시작하면 GPS 로 바로잡힌다 (Gazebo 확인).
        출발 순간의 요로 열 방향(모델 좌표축)을 정하면 통로 전체가 그만큼 돌아간 채 계획된다.
        """
        self.model = None
        self.yaw_bias = 0.0
        self.boot_xy = np.array([x, y], float)
        self.drift = np.zeros(2)
        self.state, self.reason, self.k, self.lanes_done = FOLLOW_ROW, '출발 (LiDAR 중심선 따라가며 요 안정 대기)', 0, 0
        self.turn_path = None
        self.turn_grid = None
        self.end_confirmed = False
        self.centers = []

    # ------------------------------------------------------------ 과수원 진입
    def _approach(self, now, x, y, yaw, trees_base, row) -> PlanOutput:
        """APPROACH: 최근 줄기로 끝 통로 입구 찾기 → 정렬점까지 경로 → 도착하면 통로 0 시작. 못 보면 직진하며 찾기."""
        p = self.p
        if trees_base is not None and len(trees_base):
            T = np.asarray(trees_base, float).reshape(-1, 2)
            T = T[np.hypot(T[:, 0], T[:, 1]) < 20.0]
            c, s_ = math.cos(yaw), math.sin(yaw)
            self.app_mem.append((now, np.c_[x + c * T[:, 0] - s_ * T[:, 1], y + s_ * T[:, 0] + c * T[:, 1]]))
        self.app_mem = [(t, P) for t, P in self.app_mem if now - t <= p.approach_memory]
        P = np.vstack([P for _, P in self.app_mem]) if self.app_mem else np.zeros((0, 2))
        side = p.approach_side if p.approach_side in ('right', 'left') else \
            ('right' if p.first_turn == 'left' else 'left')
        ent = detect_entrance(P, (x, y), p.row_spacing, side, pre_dist=p.approach_pre)
        if ent is not None:
            self._filter_entrance(ent, (x, y))
        if self.app_ent is None:
            if np.hypot(x - self.app_start[0], y - self.app_start[1]) > p.approach_max_dist:
                self.stop(f'입구 못 찾음 ({p.approach_max_dist:.0f} m 달림)')
                return self._output(np.zeros((0, 3)))
            self.reason = '입구 찾는 중 (열이 아직 안 보임 → 천천히 직진)'
            xb = np.arange(0.0, 4.0 + 1e-9, p.path_step)
            return self._output(np.c_[x + math.cos(yaw) * xb, y + math.sin(yaw) * xb, np.full(len(xb), yaw)])
        e = self.app_ent
        e_u = np.array([math.cos(e.heading), math.sin(e.heading)])
        # 도착: 정렬점 가까이(1.5 m) 또는 지나왔고, 통로 방향과 0.5 rad 안 → 평소 출발 절차로 넘김
        # (남은 각도는 출발 구간이 LiDAR 중심선을 따라가며 맞추고, 열 방향 기준 거리는 로버가 중심선과 나란해진 뒤부터 잰다)
        rel = np.array([x, y]) - e.pre
        d_pre = float(np.hypot(*rel))
        along = float(rel @ e_u)
        lateral = abs(float(rel @ np.array([-e_u[1], e_u[0]])))
        dyaw = abs(_wrap(yaw - e.heading))
        if (d_pre < 1.5 and dyaw < 0.5) or (along > -0.5 and lateral < 1.0 and dyaw < 0.6):
            self._begin_lanes(now, x, y, yaw)
            self.reason = f'입구 도착 ({e.side} 끝 통로) → 통로 0 출발'
            path = self._bootstrap(x, y, yaw, row)             # 같은 주기에 바로 출발 구간 경로 (멈칫하지 않게)
            return self._output(path if path is not None else np.zeros((0, 3)))
        # 경로: 목표가 바뀌었거나, 1 s 지났고 경로에서 1 m 넘게 벗어났으면 다시 계획 (정렬점을 지난 뒤에는 다시 계획하지 않음)
        need = self.app_path is None or getattr(self, 'app_goal_moved', False)
        if not need and now - self.app_plan_t > 1.0 and along < -1.0:
            dev = float(np.min(np.hypot(self.app_path[:, 0] - x, self.app_path[:, 1] - y)))
            need = dev > 1.0
        if need:
            start, goal = (x, y, yaw), (float(e.pre[0]), float(e.pre[1]), e.heading)
            path, _ = self._search_trees(start, goal, p.approach_radius, P)
            ins = np.arange(p.path_step, p.approach_pre + 2.0, p.path_step)       # 정렬점 → 통로 안 2 m (지역 계획 목표점용)
            tail = np.c_[e.pre[0] + e_u[0] * ins, e.pre[1] + e_u[1] * ins, np.full(len(ins), e.heading)]
            self.app_path = np.vstack([path, tail])
            self.app_plan_t = now
            self.app_goal_moved = False
        i = int(np.argmin(np.hypot(self.app_path[:, 0] - x, self.app_path[:, 1] - y)))
        self.reason = f'입구로 이동 ({e.side} 끝 통로, 열 {e.n_rows}개 봄, 정렬점까지 {d_pre:.1f} m)'
        out = self._output(self.app_path[i:])
        out.rows = [(a, b) for a, b in e.rows]
        if e.lane_line:
            out.lane_lines = [(e.lane_line[0], e.lane_line[1], 0)]
        out.trees = P
        out.turn_grid = self.turn_grid
        return out

    def _filter_entrance(self, ent: Entrance, rover_xy=None) -> Entrance:
        """입구 추정 걸러 내기. 비슷하면(1 m, 0.2 rad 안) 천천히 따라가고(0.3), 크게 다르면:
        아직 안정 전(비슷한 추정 5번 미만)에는 3번 연속일 때 바꾸고, 안정된 뒤에는 무시한다
        (입구 가까이에서 열을 비스듬히·옆에서 보면 방향을 90° 틀리게 잡는 일이 있음)."""
        cur = self.app_ent
        if cur is None:                                  # 처음: 비슷한 추정이 3번 이어져야 목표로 삼음 (한 번 잘못 본 것 무시)
            c0 = getattr(self, 'app_cand', None)
            if (c0 is not None and float(np.hypot(*(ent.pre - c0.pre))) < 1.0
                    and abs(_wrap(ent.heading - c0.heading)) < 0.2):
                self.app_cand_n += 1
            else:
                self.app_cand, self.app_cand_n = ent, 1
            if self.app_cand_n >= 3:
                self.app_ent, self.app_goal_moved, self.app_jump, self.app_stable = ent, True, 0, 0
            return self.app_ent
        dp = float(np.hypot(*(ent.pre - cur.pre)))
        dh = abs(_wrap(ent.heading - cur.heading))
        # 같은 방향인데 고른 쪽(side)으로 한 통로 이상 더 바깥 통로가 보이면(멀리서는 끝 열이 안 보일 수 있음) 2번 이어질 때 그쪽으로 바꿈.
        # 정렬점에 2 m 안으로 들어온 뒤에는 바꾸지 않음 (끝 열을 늦게 보면 한 칸 안쪽 통로로 들어가는 것을 막음)
        S = self.p.row_spacing
        outer = (ent.lane_c < cur.lane_c - 0.5 * S) if ent.side == 'right' else (ent.lane_c > cur.lane_c + 0.5 * S)
        far = rover_xy is None or float(np.hypot(rover_xy[0] - cur.pre[0], rover_xy[1] - cur.pre[1])) > 2.0
        if dh < 0.2 and outer and far:
            self.app_outer = getattr(self, 'app_outer', 0) + 1
            if self.app_outer >= 2:
                self.app_ent, self.app_goal_moved, self.app_jump, self.app_stable, self.app_outer = ent, True, 0, 0, 0
            return self.app_ent
        self.app_outer = 0
        if dp < 1.0 and dh < 0.2:
            a = 0.3
            cur.pre = cur.pre + a * (ent.pre - cur.pre)
            cur.entry = cur.entry + a * (ent.entry - cur.entry)
            cur.heading = _wrap(cur.heading + a * _wrap(ent.heading - cur.heading))
            cur.rows, cur.lane_line, cur.n_rows = ent.rows, ent.lane_line, ent.n_rows
            self.app_goal_moved = self.app_goal_moved or dp > 0.3
            self.app_jump = 0
            self.app_stable += 1
        elif self.app_stable < 5:
            self.app_jump += 1
            if self.app_jump >= 3:
                self.app_ent, self.app_goal_moved, self.app_jump, self.app_stable = ent, True, 0, 0
        return self.app_ent

    def _search_trees(self, start, goal, radius, P):
        """줄기 P 를 장애물로 한 Hybrid A* (못 찾으면 Dubins). _search 와 같지만 모델 대신 주어진 줄기를 쓴다."""
        near = P[np.hypot(P[:, 0] - start[0], P[:, 1] - start[1]) < 25.0] if len(P) else P
        grid = OccupancyGrid.around(near, [start[:2], goal[:2]], 0.1, 3.0 * radius)
        grid.add_discs(near, self.p.tree_radius + self.p.robot_radius)
        self.turn_grid = grid
        path = hybrid_astar(start, goal, grid, radius)
        if path is None:
            w = dubins_words_sorted(start, goal, radius)
            path = sample_word(start, w[0][1], radius) if w else np.array([start, goal])
        return path, goal

    def _bootstrap(self, x, y, yaw, row) -> np.ndarray:
        """출발 구간: LiDAR 중심선 (base_link: y = offset + tan(heading)·x) 을 odom 경로로. 충분히 달렸으면 모델 생성."""
        p = self.p
        ok = row is not None and row.valid and row.confidence >= 0.5
        if ok and abs(row.heading) > 0.12:
            self.boot_xy = np.array([x, y], float)      # 아직 중심선과 나란하지 않음 → 이동 방향 기준 거리를 여기서부터 다시 잼
        if ok and np.hypot(x - self.boot_xy[0], y - self.boot_xy[1]) >= p.bootstrap_dist \
                and row.left_count >= 2 and row.right_count >= 2:
            s = +1 if p.first_turn == 'left' else -1
            cx, cy = x - math.sin(yaw) * row.offset, y + math.cos(yaw) * row.offset     # 로버 옆 통로 중심 (odom)
            # 열 방향: 중심선을 따라 달린 이동 방향 (GPS 위치 차 — 요 오차와 무관). 요 + 중심선 각과 크게 다르면 그쪽이 틀린 것
            course = math.atan2(y - self.boot_xy[1], x - self.boot_xy[0])
            self.model = OrchardModel((cx, cy), course, p.row_spacing, s)
            self.yaw_bias = _wrap(course - (yaw + row.heading))            # 이 순간 odom 요 오차
            self.reason = '통로 0 주행'
            return None
        off, head = (row.offset, row.heading) if ok else (0.0, 0.0)
        xb = np.arange(0.0, 6.0 + 1e-9, p.path_step)
        yb = off + math.tan(head) * xb
        c, s_ = math.cos(yaw), math.sin(yaw)
        return np.c_[x + c * xb - s_ * yb, y + s_ * xb + c * yb, np.full(len(xb), yaw + head)]

    def stop(self, reason: str = '사용자 정지') -> None:
        """정지 (경로 비움)."""
        self.state, self.reason = STOPPED, reason

    # ------------------------------------------------------------ 주기 갱신
    def update(self, now: float, x: float, y: float, yaw: float, trees_base=None, row: RowSeen | None = None
               ) -> PlanOutput:
        """odom 자세 (x, y, yaw), 이번 주기 줄기 (base_link N×2), 통로 관측 → 새 전역 경로."""
        if self.state in (IDLE, DONE, STOPPED):
            return self._output(np.zeros((0, 3)))
        if self.state == APPROACH:
            return self._approach(now, x, y, yaw, trees_base, row)
        if self.model is None:                                 # 출발 구간 (LiDAR 중심선 직접 추종)
            path = self._bootstrap(x, y, yaw, row)
            if path is not None:
                return self._output(path)
        m = self.model
        lanes = m.lanes(self.k + 1)
        # 갱신 허용 판단: 로버가 통로 방향으로 곧게 달릴 때만 (장애물을 비키는 중에는 LiDAR 중심선·줄기가 흐트러져
        # 요 오차·열 방향이 서로 쫓아가며 틀어진다)
        self.tracking = self._is_tracking(x, y, yaw + self.yaw_bias, lanes)
        if self.tracking:
            self._update_yaw_bias(yaw, row)
        yaw = yaw + self.yaw_bias                               # 이후 계산은 모두 보정한 요
        self._update_drift(x, y, yaw, row, lanes)
        px, py = x - self.drift[0], y - self.drift[1]           # model 좌표 로버 위치
        # 줄기 누적: 통로 안에서, 흐름 보정이 안정되고 곧게 달릴 때만 (U턴 중·보정 직후·장애물 회피 중은 지도가 번짐)
        if trees_base is not None and len(trees_base) and self.state == FOLLOW_ROW and abs(self.last_err) < 0.1 \
                and self.tracking:
            T = np.asarray(trees_base, float).reshape(-1, 2)
            T = T[np.hypot(T[:, 0], T[:, 1]) < 9.0]
            c, s_ = math.cos(yaw), math.sin(yaw)
            m.add(np.c_[px + c * T[:, 0] - s_ * T[:, 1], py + s_ * T[:, 0] + c * T[:, 1]])
        if self.state == FOLLOW_ROW and self.k == 0:
            self._collect_center(x, y, yaw, row)
            if now - self.last_heading_refine > 2.0:
                self._refine_heading()   # 열 방향은 첫 통로에서만 다듬고 고정
                self.last_heading_refine = now
        lanes = m.lanes(self.k + 1)
        u_r = float(m.to_uv([[px, py]])[0, 0])
        if self.state == FOLLOW_ROW:
            path = self._follow(now, lanes, u_r, px, py, yaw)
        else:
            path = self._turn(now, lanes, px, py, yaw, row)
        if self.state in (DONE, STOPPED):
            path = np.zeros((0, 3))
        return self._output(path, lanes)

    # ------------------------------------------------------------ 통로 주행
    def _next_exists(self, lanes: list) -> bool:
        """지금 통로 다음 통로를 돌지 (통로 수 지정 또는 탐사: 먼 쪽 열이 보이는지)."""
        nxt = lanes[self.k + 1]
        if self.p.lanes > 0:                     # 통로 수 지정: 남았어도 다음 통로 먼 쪽 열이 전혀 안 보이면 과수원 밖 → 끝
            return self.k + 1 < self.p.lanes and nxt.front is not None
        return (self.k + 1 < self.p.max_lanes and nxt.front is not None and nxt.front.n >= self.p.min_row_trees)

    def _follow(self, now, lanes, u_r, px, py, yaw) -> np.ndarray:
        """FOLLOW_ROW: 통로 끝 확정·U턴 계획·전이, 남은 경로 (model) 반환."""
        m, p = self.model, self.p
        lane = lanes[self.k]
        d = lane.direction
        end_tree = lane.end_tree()
        self.end_confirmed = bool(np.isfinite(end_tree) and d * (end_tree - u_r) < p.end_confirm_dist)
        if self.end_confirmed:
            u_end = end_tree + d * p.exit_margin
        else:
            u_end = u_r + d * p.ahead
        segs = [self._lane_path(lane, u_r, u_end)]
        if self.end_confirmed:
            if self._next_exists(lanes):
                if now - self.last_turn_plan > 1.0 or self.turn_path is None:
                    self._plan_turn(lanes, u_end)
                    self.last_turn_plan = now
                if self.turn_path is not None:
                    segs.append(self.turn_path)
                    nxt = lanes[self.k + 1]
                    nu0 = float(m.to_uv(self.turn_path[-1:, :2])[0, 0])
                    far = nxt.end_tree(nxt.direction)
                    nu1 = far + nxt.direction * p.exit_margin if np.isfinite(far) else nu0 + nxt.direction * p.ahead
                    segs.append(self._lane_path(nxt, nu0, nu1))
                if d * (u_r - u_end) > -0.2:
                    self.state, self.reason = TURN, f'U턴 {self.k}→{self.k + 1} (Hybrid A*)'
                    self.turn_i = 0
            else:
                self.reason = '마지막 통로'
                if d * (u_r - u_end) > -0.2:
                    self.lanes_done += 1
                    self.state, self.reason = DONE, ('과수원 끝 열 도달' if p.lanes <= 0 else '모든 통로 주행 완료')
        else:
            self.reason = f'통로 {self.k} 주행'
        return np.vstack([s for s in segs if len(s)])

    def _lane_path(self, lane: Lane, u0: float, u1: float) -> np.ndarray:
        """통로 중심선 u0 → u1 직선 점열 (model, N×3)."""
        m = self.model
        n = max(2, int(abs(u1 - u0) / self.p.path_step) + 1)
        u = np.linspace(u0, u1, n)
        xy = m.to_xy(np.c_[u, np.full(n, lane.c)])
        yaw = m.lane_yaw(1 if u1 >= u0 else -1)
        return np.c_[xy, np.full(n, yaw)]

    def _plan_turn(self, lanes: list, u_end: float) -> None:
        """통로 k 끝 → 통로 k+1 입구 U턴 경로 (Hybrid A*, model)."""
        m, p = self.model, self.p
        lane, nxt = lanes[self.k], lanes[self.k + 1]
        d = lane.direction
        entry_tree = nxt.end_tree(d)                        # 다음 통로의, 지금 달리는 쪽 끝 줄기
        u_entry = entry_tree + d * p.entry_margin if np.isfinite(entry_tree) else u_end
        start_xy = m.to_xy([[u_end, lane.c]])[0]
        goal_xy = m.to_xy([[u_entry, nxt.c]])[0]
        start = (start_xy[0], start_xy[1], m.lane_yaw(d))
        goal = (goal_xy[0], goal_xy[1], m.lane_yaw(-d))
        radius = max(p.min_turn_radius, 0.5 * abs(nxt.c - lane.c))
        self.turn_path, self.turn_goal = self._search(start, goal, radius)

    def _search(self, start, goal, radius):
        """나무를 장애물로 한 격자 위 Hybrid A*. 못 찾으면 장애물 무시 Dubins (reason 에 표시)."""
        P = self.model.trees()
        near = P[np.hypot(P[:, 0] - start[0], P[:, 1] - start[1]) < 15.0] if len(P) else P
        grid = OccupancyGrid.around(near, [start[:2], goal[:2]], 0.1, 3.0 * radius)
        grid.add_discs(near, self.p.tree_radius + self.p.robot_radius)
        self.turn_grid = grid
        path = hybrid_astar(start, goal, grid, radius)
        if path is None:
            w = dubins_words_sorted(start, goal, radius)
            path = sample_word(start, w[0][1], radius) if w else np.array([start, goal])
            self.reason = 'U턴 경로 충돌 없는 해 없음 → Dubins (주의)'
        return path, goal

    # ------------------------------------------------------------ U턴
    def _turn(self, now, lanes, px, py, yaw, row) -> np.ndarray:
        """TURN: U턴 경로 진행, 끝/넘겨받기 판정, 벗어나면 재계획. 남은 경로 (model) 반환."""
        m, p = self.model, self.p
        P = self.turn_path
        hi = min(len(P), self.turn_i + 80)
        dist = np.hypot(P[self.turn_i:hi, 0] - px, P[self.turn_i:hi, 1] - py)
        self.turn_i += int(np.argmin(dist))
        nxt = lanes[self.k + 1]
        yaw_m = yaw
        aligned = abs(_wrap(yaw_m - m.lane_yaw(nxt.direction))) < p.handover_angle
        remaining = float(np.sum(np.hypot(np.diff(P[self.turn_i:, 0]), np.diff(P[self.turn_i:, 1]))))
        if remaining < p.reach_tol or (aligned and self._row_matches(px, py, yaw, row, nxt)):
            self.k += 1
            self.lanes_done += 1
            self.state, self.reason = FOLLOW_ROW, f'{self.k + 1}번째 통로 주행'
            self.turn_path = None
            self.end_confirmed = False
            lanes = m.lanes(self.k + 1)
            u_r = float(m.to_uv([[px, py]])[0, 0])
            return self._follow(now, lanes, u_r, px, py, yaw)
        if dist.min() > p.max_turn_deviation:                  # 크게 벗어남 → 지금 자세에서 다시 계획
            radius = max(p.min_turn_radius, 0.5 * abs(nxt.c - lanes[self.k].c))
            self.turn_path, _ = self._search((px, py, yaw), self.turn_goal, radius)
            self.turn_i = 0
            self.reason = 'U턴 재계획 (경로 이탈)'
            P = self.turn_path
        u0 = float(m.to_uv(P[-1:, :2])[0, 0])
        far = nxt.end_tree(nxt.direction)
        u1 = far + nxt.direction * p.exit_margin if np.isfinite(far) else u0 + nxt.direction * p.ahead
        return np.vstack([P[self.turn_i:], self._lane_path(nxt, u0, u1)])

    def _row_matches(self, x, y, yaw, row, lane: Lane) -> bool:
        """LiDAR 가 본 통로 중심이 모델의 통로 lane 중심과 0.35·열간격 안인가 (양쪽 열 다 보일 때)."""
        if row is None or not row.valid or row.confidence < 0.5 or row.left_count < 2 or row.right_count < 2:
            return False
        v_obs = self._row_v(x, y, yaw, row)
        return abs(v_obs - lane.c) < 0.35 * self.model.S

    def _row_v(self, px, py, yaw, row) -> float:
        """LiDAR 통로 중심의 row frame 횡위치 v (model 좌표 로버 위치 px, py 기준)."""
        cx, cy = px - math.sin(yaw) * row.offset, py + math.cos(yaw) * row.offset
        return float(self.model.to_uv([[cx, cy]])[0, 1])

    # ------------------------------------------------------------ odom 요 오차
    def _update_yaw_bias(self, yaw_raw: float, row) -> None:
        """통로 안: 모델 통로 방향 − (odom 요 + 오차 + LiDAR 중심선 각) 을 오차 추정에 조금씩 반영 (한 주기 최대 yaw_step_max)."""
        if self.state != FOLLOW_ROW or row is None or not row.valid or row.confidence < 0.5 \
                or row.left_count < 3 or row.right_count < 3 or abs(row.heading) > 0.3:
            return
        d = self.model.lanes(self.k)[self.k].direction
        e = _wrap(self.model.lane_yaw(d) - (yaw_raw + self.yaw_bias + row.heading))
        if abs(e) < 0.5:
            step = float(np.clip(self.p.yaw_gain * e, -self.p.yaw_step_max, self.p.yaw_step_max))
            self.yaw_bias = _wrap(self.yaw_bias + step)

    def _is_tracking(self, x, y, yaw, lanes) -> bool:
        """FOLLOW_ROW 에서 로버(보정 요 yaw)가 지금 통로 방향과 track_heading_tol 안, 중심에서 track_lateral_tol 안인가."""
        if self.state != FOLLOW_ROW:
            return False
        m, p = self.model, self.p
        lane = lanes[self.k]
        if abs(_wrap(yaw - m.lane_yaw(lane.direction))) > p.track_heading_tol:
            return False
        v = float(m.to_uv([[x - self.drift[0], y - self.drift[1]]])[0, 1])
        return abs(v - lane.c) < p.track_lateral_tol

    # ------------------------------------------------------------ 열 방향
    def _collect_center(self, x, y, yaw, row) -> None:
        """통로 0: LiDAR 통로 중심점을 odom 좌표로 모은다. 위치(GPS)만 쓰고 요 오차는 offset 에만 곱해져 영향이 작다."""
        if row is None or not row.valid or row.confidence < 0.5 or row.left_count < 3 or row.right_count < 3 \
                or abs(row.heading) > 0.3 or not self.tracking:
            return
        self.centers.append((x - math.sin(yaw) * row.offset, y + math.cos(yaw) * row.offset))
        if len(self.centers) > 600:
            self.centers = self.centers[::2]

    def _refine_heading(self) -> bool:
        """통로 0 중심점들에 직선 맞춤(주성분, 벗어난 점 한 번 제거) → 열 방향 theta. 출발 이동 방향에서 heading_max_change 까지만.

        누적 줄기 배치로 다듬지 않는 이유: 줄기 위치는 요 오차로 돌려 놓은 값이라 요 오차와 열 방향이
        함께 돌아간다 (둘의 차이만 관측됨). 통로 중심점은 GPS 위치에서 나오므로 요 오차와 독립.
        """
        m, p = self.model, self.p
        if len(self.centers) < 10:
            return False
        C = np.asarray(self.centers, float)
        for _ in range(2):
            mu = C.mean(0)
            _, _, vt = np.linalg.svd(C - mu, full_matrices=False)
            d = vt[0]
            r = np.abs((C - mu) @ np.array([-d[1], d[0]]))
            keep = r < max(0.15, 2.5 * float(np.median(r)))
            if keep.sum() < 10:
                return False
            C = C[keep]
        along = (C - C.mean(0)) @ d
        if along.max() - along.min() < p.heading_baseline:
            return False
        h = math.atan2(d[1], d[0])
        if math.cos(h - m.theta0) < 0:
            h += math.pi
        dh = _wrap(h - m.theta0)
        if abs(dh) > p.heading_max_change:
            return False
        m.theta = m.theta0 + dh
        return True

    # ------------------------------------------------------------ odom 흐름
    def _update_drift(self, x, y, yaw, row, lanes) -> None:
        """통로 안(또는 U턴 막바지)에서 LiDAR 통로 중심 − 모델 통로 중심 = odom 흐름 (v 방향) 추정."""
        m, p = self.model, self.p
        if row is None or not row.valid or row.confidence < 0.5 or row.left_count < 3 or row.right_count < 3 \
                or abs(row.heading) > 0.3:
            return                                          # 양쪽 열이 3그루 이상씩 보일 때만 (행 끝 근처 추정은 불안정)
        if self.state == FOLLOW_ROW:
            lane = lanes[self.k]
            if abs(_wrap(yaw - m.lane_yaw(lane.direction))) > p.track_heading_tol:
                return                                      # 장애물 비키는 중 (통로와 비스듬히 달림)
        elif self.state == TURN and abs(_wrap(yaw - m.lane_yaw(lanes[self.k + 1].direction))) < p.handover_angle:
            lane = lanes[self.k + 1]
        else:
            return
        if lane.back is None and lane.front is None:
            return
        err = self._row_v(x - self.drift[0], y - self.drift[1], yaw, row) - lane.c
        if abs(err) > 0.4 * m.S:
            return
        self.last_err = err
        self.drift = self.drift + p.drift_gain * err * m.e_v

    # ------------------------------------------------------------ 출력
    def _output(self, path_model: np.ndarray, lanes: list | None = None) -> PlanOutput:
        """model 경로·열·통로를 odom 으로 바꿔 PlanOutput."""
        dx, dy = self.drift
        path = path_model.copy()
        if len(path):
            path[:, 0] += dx
            path[:, 1] += dy
        out = PlanOutput(self.state, self.lanes_done, self.reason, path, self.k, (float(dx), float(dy)),
                         float(self.yaw_bias))
        if self.model is not None:
            m = self.model
            for r in m.rows():
                a, b = m.to_xy([[r.u_min, r.v], [r.u_max, r.v]]) + self.drift
                out.rows.append((a, b))
            for ln in lanes or []:
                us = [u for r in ln.rows() for u in (r.u_min, r.u_max)]
                if us:
                    a, b = m.to_xy([[min(us), ln.c], [max(us), ln.c]]) + self.drift
                    out.lane_lines.append((a, b, ln.k))
            T = m.trees()
            out.trees = T + self.drift if len(T) else T
            out.turn_grid = self.turn_grid
        return out
