"""과수원 행 순회 미션 상태기계 — ROS 의존성 없음.

파이프라인에서의 역할
  행 추종 → 행 끝 → U턴 → 다음 통로 진입 → 행 추종 ... 전체 흐름을 결정하는 '두뇌'.
  row_navigator_node 가 20 Hz 로 step() 을 부르고, 돌려받은 (v, w) 를 /cmd_vel 로 낸다.
  실제 제어 계산은 controller.py (행 추종·요 유지), headland.py (U턴 계획·경로 추종) 에 맡긴다.

입력 (step 인자)
  - now [s]: 시각 (ROS 시계, 시뮬에서는 sim time)
  - pose (Pose2D): odom 좌표 로버 자세 x, y [m], yaw [rad] (/odom, PX4 EKF)
  - row (RowObs): 행 인식 결과 (/orchard/row, 필요하면 카메라와 융합) — base_link 기준
  - add_trees(): /orchard/trees 줄기 위치 (base_link [m]) — U턴 계획용으로 odom 좌표로 모아 둔다
출력
  - (v [m/s], w [rad/s]) — ROS 규약 w>0 좌회전
  - state / reason / lanes_done — /orchard/mission/state 로 발행 (orchard_mapper_node 는 DONE 이면 지도 저장)

상태 흐름 (lanes = 주행할 행간 통로 수)

  IDLE ─start→ FOLLOW_ROW ─행 끝→ EXIT_ROW ─exit_distance 주행→ TURN ─180°→ ENTER_ROW ─행 인식→ FOLLOW_ROW
                                     └─ 마지막 통로면 ────────────→ DONE
  어느 상태든 stop() → STOPPED,  ENTER_ROW 에서 행을 못 찾으면 → STOPPED(안전정지)

탐사 모드 (lanes <= 0, 통로 수를 모름)
  통로 수 제한 없이 U턴을 계속하다가, U턴 뒤 들어가려는 곳의 '먼 쪽'(방금 돈 방향 쪽)에 나무 열이 없으면
  = 과수원 끝 열 바깥 → DONE('과수원 끝 열 도달'). 한쪽 열만 보고 끝 열 바깥을 따라 달리는 것을 막는다.

U턴 (turn_mode)
  'planned' (기본): 행 끝에서 LiDAR 줄기로 다음 통로 중심·입구를 직접 잡고(headland.estimate_next_lane),
      현재 자세 → 입구(반대 방향) Dubins 경로를 만들어 Pure Pursuit 로 따라간다. 간격이 파라미터와 달라도,
      잔디에서 미끄러져도 입구를 맞춘다. 줄기를 못 보면 파라미터 간격으로 같은 방식.
  'arc': 행 간격/2 반경의 반원을 요 각도만 보고 돈다 (예전 방식, 비교용).

학생이 주로 바꾸는 파라미터 (MissionParams)
  - ros2_ws/src/orchard_bringup/config/sim.yaml (시뮬), robot.yaml (실차) 의 row_navigator_node 항목에서
    'mission.*' 이름으로 설정: lanes, first_turn, row_spacing, turn_mode, turn_radius, turn_speed,
    exit_distance, end_trigger_x, min_row_travel, enter_speed, max_enter_distance, max_lanes, entry_margin
    (노드가 declare_parameter 하는 목록은 row_navigator_node.py 참고).
  - 나머지(end_confirm_frames, lost_timeout, enter_confirm_frames, handover_angle, max_turn_deviation 등)는
    ROS 파라미터로 노출되지 않음 → 아래 MissionParams 기본값을 고친다.

References (이 모듈이 호출하는 알고리즘)
  Pure Pursuit: R. C. Coulter, CMU-RI-TR-92-01, 1992 (controller.py, headland.PathTracker).
  Dubins 경로: L. E. Dubins, American Journal of Mathematics 79(3), 1957 (headland.dubins_path).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from .controller import FollowParams, heading_hold_command, obstacle_speed_factor, row_follow_command
from .headland import PathTracker, dubins_path, estimate_next_lane, path_length

# 상태 이름 (문자열 그대로 /orchard/mission/state 에 찍힌다)
IDLE, FOLLOW_ROW, EXIT_ROW, TURN, ENTER_ROW, DONE, STOPPED = (
    'IDLE', 'FOLLOW_ROW', 'EXIT_ROW', 'TURN', 'ENTER_ROW', 'DONE', 'STOPPED')


@dataclass
class RowObs:
    """행 인식 한 프레임 (RowCenterline 메시지와 같은 뜻, base_link: x 전방 / y 왼쪽).

    offset [m]: x=0 에서 중심선의 y (+: 중심선이 왼쪽), heading [rad]: 중심선 방향 − 진행방향,
    confidence 0~1, last_tree_ahead [m]: 행 추정에 쓴 줄기 중 가장 앞 x (없으면 −inf, 행 끝 판정),
    obstacle_distance [m]: 통로 안 최근접 장애물 (없으면 +inf).
    """

    valid: bool = False
    offset: float = 0.0
    heading: float = 0.0
    confidence: float = 0.0
    last_tree_ahead: float = float('-inf')
    obstacle_distance: float = float('inf')
    left_count: int = -1              # 왼쪽(+y) 열 줄기 수, -1 = 모름
    right_count: int = -1


@dataclass
class Pose2D:
    """2D 자세 (odom 좌표): x, y [m], yaw [rad] (x 축에서 반시계 +)."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class MissionParams:
    """미션 상태기계 파라미터. 거리 [m], 속도 [m/s], 시간 [s], 각도 [rad]."""

    lanes: int = 3                    # 0 이하 = 탐사 모드 (통로 수를 모르고 끝 열까지)
    max_lanes: int = 40               # 탐사 모드 안전 한도
    first_turn: str = 'left'          # 첫 U턴 방향 'left' / 'right'
    row_spacing: float = 3.8          # [m] 열 간격 (나무 열 중심 사이). 줄기를 못 볼 때 U턴 계획에 사용
    turn_radius: float = 0.0          # 0 이면 row_spacing/2
    turn_speed: float = 0.5           # [m/s] U턴 속도
    exit_distance: float = 2.2        # 마지막 나무를 지난 뒤 직진 거리 (차체 길이 + 여유)
    end_trigger_x: float = 0.3        # 전방 마지막 나무 x 가 이 값보다 작아지면 행 끝
    end_confirm_frames: int = 5       # 행 끝 조건이 이만큼 연속이어야 확정 (20 Hz → 0.25 s)
    lost_timeout: float = 1.5         # 행 인식 실패가 이 시간 이상이면 행 끝 후보
    min_row_travel: float = 3.0       # 행 진입 후 이 거리 이상 달려야 행 끝 판정
    enter_speed: float = 0.4          # [m/s] 행 끝 직진·통로 진입 속도
    enter_confirm_frames: int = 5     # 새 통로 인식이 이만큼 연속이어야 FOLLOW_ROW 로
    enter_min_conf: float = 0.4       # 새 통로로 인정할 최소 행 인식 신뢰도
    max_enter_distance: float = 6.0   # [m] ENTER_ROW 에서 이만큼 가도 통로를 못 찾으면 안전정지
    boundary_check_distance: float = 1.0   # 탐사: U턴 뒤 이만큼 들어간 다음부터 끝 열 판정
    boundary_confirm_frames: int = 8
    min_side_trees: int = 2           # 한쪽 열로 인정할 최소 줄기 수
    turn_mode: str = 'planned'        # 'planned' (LiDAR 입구 + Dubins 경로 추종) / 'arc' (반원, 예전 방식)
    entry_margin: float = 0.8         # 입구 목표 = 다음 통로 마지막 나무보다 이만큼 바깥
    turn_lookahead: float = 1.0       # [m] U턴 경로 Pure Pursuit 주시거리
    max_turn_deviation: float = 1.2   # 경로에서 이만큼 벗어나면 안전정지 [m]
    handover_angle: float = 0.6       # 180° 에서 이 각도[rad] 안으로 돌았고 LiDAR 가 새 통로를 보면 경로 대신 LiDAR 로
    follow: FollowParams = field(default_factory=FollowParams)


def _dist(a: Pose2D, b: Pose2D) -> float:
    """두 자세 사이 평면 거리 [m]."""
    return math.hypot(a.x - b.x, a.y - b.y)


def _wrap(a: float) -> float:
    """각도를 (−π, π] 로."""
    return math.atan2(math.sin(a), math.cos(a))


class OrchardMission:
    """과수원 행 순회 상태기계. 사용법: start() 한 번 → 매 주기 step() → (v, w)."""

    def __init__(self, params: MissionParams):
        """params: 미션 파라미터 (row_navigator_node 가 ROS 파라미터로 채워 넘긴다)."""
        self.p = params
        self.state = IDLE
        self.reason = ''                  # 마지막 상태 전이 이유 (로그·상태 토픽용)
        self.lanes_done = 0               # 다 달린 통로 수
        self.turn_sign = 1.0 if params.first_turn == 'left' else -1.0   # +1 = 다음 U턴 왼쪽
        self._mark = Pose2D()             # 현재 상태에 들어온 순간의 자세 (주행 거리·회전각 기준)
        self._yaw_ref = 0.0               # 요 유지 목표 [rad]
        self._counter = 0                 # 조건 연속 프레임 수 (행 끝 확정·진입 확정)
        self._last_valid_t = None         # 마지막으로 행 인식이 유효했던 시각 [s]
        self._last_tree_seen = float('inf')   # 마지막 유효 프레임의 last_tree_ahead [m]
        self._boundary = 0                # 탐사 모드 끝 열 조건 연속 프레임 수
        self._trees: list[tuple[float, float, float]] = []   # (t, x, y) odom 좌표 줄기 (최근 것만)
        self._row_end = Pose2D()          # 행 끝 판정 순간 자세 (row frame 원점)
        self.tracker: PathTracker | None = None
        self.plan = None                  # 마지막 U턴 계획 정보 (시각화·로그)

    # ---- 줄기 관측 (U턴 계획용) ----
    def add_trees(self, now: float, pose: Pose2D, xy_base) -> None:
        """base_link 줄기 좌표를 odom 좌표로 바꿔 최근 20초만 보관.

        Args:
            now: 시각 [s].
            pose: 관측 순간 로버 자세 (odom).
            xy_base: [(x, y), ...] base_link 줄기 위치 [m].
        12 m 보다 먼 줄기는 위치 오차가 커서 버린다. 20 s 보관이면 0.8 m/s 로 행 끝 16 m 전까지의
        줄기가 남아 U턴 계획(estimate_next_lane 의 u_min = −12 m)에 충분하다.
        """
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        for bx, by in xy_base:
            if math.hypot(bx, by) < 12.0:
                self._trees.append((now, pose.x + c * bx - s * by, pose.y + s * bx + c * by))
        while self._trees and self._trees[0][0] < now - 20.0:
            self._trees.pop(0)

    def _plan_turn(self, pose: Pose2D):
        """행 끝 자세 기준 다음 통로 입구로 Dubins 경로. 실패하면 None (반원 방식으로).

        1) 모아 둔 줄기를 row frame (행 끝 자세 원점, u 앞 / v 왼쪽) 으로 옮겨 다음 통로를 추정.
        2) 회전반경 R = 다음 통로까지 횡거리의 절반 (반원) 을 [최소 회전반경, radius] 로 자름.
        3) 목표 = 다음 통로 중심, 마지막 나무보다 entry_margin 바깥, 방향은 반대(행 끝 요 + π).
        Returns:
            (NextLane, R [m]) — 성공 시 self.tracker, self.plan 도 채운다. 실패 시 None.
        """
        o = self._row_end
        c, s = math.cos(o.yaw), math.sin(o.yaw)
        # odom → row frame: 행 끝 자세만큼 평행이동 후 −yaw 회전
        uv = [(c * (x - o.x) + s * (y - o.y), -s * (x - o.x) + c * (y - o.y)) for _, x, y in self._trees]
        nl = estimate_next_lane(uv, self.turn_sign, self.p.row_spacing)
        u0 = c * (pose.x - o.x) + s * (pose.y - o.y)          # 현재 위치 (row frame)
        v0 = -s * (pose.x - o.x) + c * (pose.y - o.y)
        R_min = self.p.follow.min_turn_radius
        R = max(R_min, min(self.radius, 0.5 * abs(nl.v - v0)))
        # 입구 u: 마지막 나무 + 여유. 단, 현재 위치보다 3 m 넘게 행 안쪽(뒤)으로는 잡지 않음
        ug = max(nl.u_end + self.p.entry_margin, u0 - 3.0)
        gx, gy = o.x + c * ug - s * nl.v, o.y + s * ug + c * nl.v   # row frame → odom
        gyaw = _wrap(o.yaw + math.pi)
        try:
            path = dubins_path((pose.x, pose.y, pose.yaw), (gx, gy, gyaw), R)
        except ValueError:
            return None
        self.tracker = PathTracker(path, self.p.turn_lookahead, 1.0 / R_min)
        self.plan = dict(next_lane=nl, radius=R, goal=(gx, gy, gyaw), length=path_length(path), path=path)
        return nl, R

    @property
    def explore(self) -> bool:
        """탐사 모드 여부 (lanes <= 0: 통로 수를 모르고 끝 열까지)."""
        return self.p.lanes <= 0

    # ---- 외부 명령 ----
    def start(self, now: float, pose: Pose2D):
        """미션 시작: 통로 수·U턴 방향을 초기화하고 FOLLOW_ROW 로 (로버는 첫 통로 안/입구에 있어야 함)."""
        self.lanes_done = 0
        self.turn_sign = 1.0 if self.p.first_turn == 'left' else -1.0
        self._enter(FOLLOW_ROW, now, pose, '시작')

    def stop(self, reason: str = '사용자 정지'):
        """즉시 STOPPED (이후 step() 은 항상 (0, 0)). 다시 달리려면 start()."""
        self.state = STOPPED
        self.reason = reason

    def _enter(self, state: str, now: float, pose: Pose2D, reason: str = ''):
        """상태 전이: 진입 자세(_mark)를 기록하고 카운터를 초기화한다."""
        self.state = state
        self.reason = reason
        self._mark = Pose2D(pose.x, pose.y, pose.yaw)
        self._counter = 0
        self._boundary = 0
        if state == FOLLOW_ROW:
            self._last_valid_t = now
            self._last_tree_seen = float('inf')

    @property
    def radius(self) -> float:
        """반원 U턴 반경 [m] (turn_radius 가 0 이면 열 간격의 절반)."""
        return self.p.turn_radius if self.p.turn_radius > 0 else 0.5 * self.p.row_spacing

    # ---- 주기 호출 ----
    def step(self, now: float, pose: Pose2D, row: RowObs) -> tuple[float, float]:
        """(v, w) 반환. ROS 규약 w>0 좌회전.

        Args:
            now: 시각 [s].
            pose: odom 자세.
            row: 이번 주기 행 인식 결과 (끊겼으면 valid=False).
        상태가 바뀌면 같은 주기 안에서 step() 을 한 번 더 불러 새 상태의 명령을 바로 낸다
        (전이 순간 한 주기 동안 0 명령으로 멈칫하지 않게).
        """
        f = self.p.follow
        # 장애물 속도 배율 (0~1): 행 추종 외 상태(직진·U턴·진입)에서 곱한다
        obs = obstacle_speed_factor(row.obstacle_distance, f.stop_distance, f.slow_distance)

        # ---- IDLE / DONE / STOPPED: 정지. start() 가 불릴 때까지 그대로.
        if self.state in (IDLE, DONE, STOPPED):
            return 0.0, 0.0

        # ---- FOLLOW_ROW: LiDAR 중심선을 Pure Pursuit 로 추종.
        #  진입: start() 또는 ENTER_ROW 에서 새 통로 확정.
        #  탈출 → EXIT_ROW: (a) 마지막 나무가 옆/뒤로 지나감(last_tree_ahead < end_trigger_x) 이
        #        end_confirm_frames 연속, 또는 (b) 행 끝 근처(마지막 나무 3 m 이내)에서 인식이 lost_timeout 넘게 끊김.
        #        둘 다 행 진입 후 min_row_travel 이상 달린 뒤에만 (진입 직후 오판 방지).
        if self.state == FOLLOW_ROW:
            travelled = _dist(pose, self._mark)
            if row.valid:
                self._last_valid_t = now
                if math.isfinite(row.last_tree_ahead):
                    self._last_tree_seen = row.last_tree_ahead
            end_by_trees = (row.valid and travelled > self.p.min_row_travel
                            and row.last_tree_ahead < self.p.end_trigger_x)
            lost = now - (self._last_valid_t or now) > self.p.lost_timeout
            # 3.0 m: 끊기기 직전 마지막 나무가 이만큼 가까웠으면 행 끝에서 끊긴 것으로 본다 (행 중간 끊김과 구분)
            end_by_lost = lost and travelled > self.p.min_row_travel and self._last_tree_seen < 3.0
            self._counter = self._counter + 1 if (end_by_trees or end_by_lost) else 0
            # 인식 끊김은 이미 lost_timeout 동안 확인했으므로 1 프레임으로 확정
            if self._counter >= self.p.end_confirm_frames or (end_by_lost and self._counter >= 1):
                self._yaw_ref = pose.yaw
                self._row_end = Pose2D(pose.x, pose.y, pose.yaw)
                self._enter(EXIT_ROW, now, pose, '행 끝 감지')
                return self.step(now, pose, row)
            if not row.valid:
                return 0.0, 0.0          # 잠깐 인식 실패 → 정지 대기
            return row_follow_command(row.offset, row.heading, row.confidence,
                                      row.obstacle_distance, f)

        # ---- EXIT_ROW: 행 끝 요를 유지하며 exit_distance 만큼 직진해 차체를 헤드랜드로 뺀다.
        #  탈출: exit_distance 주행 후 → 마지막 통로면 DONE, 아니면 U턴 계획 후 TURN.
        if self.state == EXIT_ROW:
            if _dist(pose, self._mark) >= self.p.exit_distance:
                limit = self.p.max_lanes if self.explore else self.p.lanes
                if self.lanes_done + 1 >= limit:
                    self.lanes_done += 1
                    self._enter(DONE, now, pose, '모든 통로 주행 완료')
                    return 0.0, 0.0
                self._yaw_ref = _wrap(pose.yaw)
                self.tracker, reason = None, 'U턴 (반원)'
                if self.p.turn_mode == 'planned':
                    res = self._plan_turn(pose)
                    if res is not None:
                        nl, R = res
                        reason = (f'U턴 (계획: 다음 통로 {nl.source}, 줄기 관측 {nl.near_count}+{nl.far_count}, '
                                  f'간격 {nl.spacing:.2f} m, 반경 {R:.2f} m, 경로 {self.plan["length"]:.1f} m)')
                self._enter(TURN, now, pose, reason)
                return self.step(now, pose, row)
            v, w = heading_hold_command(pose.yaw, self._yaw_ref, self.p.enter_speed)
            return v * obs, w

        # ---- TURN (planned, tracker 있음): Dubins 경로를 Pure Pursuit 로 추종.
        #  탈출 → ENTER_ROW: (a) 180° − handover_angle 이상 돌았고 LiDAR 가 새 통로 중심 근처를 보면 바로 넘김,
        #        또는 (b) 경로 끝까지 0.15 m 이내.
        #  탈출 → STOPPED: 경로에서 max_turn_deviation 이상 벗어남 (안전정지).
        if self.state == TURN and self.tracker is not None:
            v, w = self.tracker.command(pose.x, pose.y, pose.yaw, self.p.turn_speed)
            turned = abs(_wrap(pose.yaw - self._mark.yaw))
            # 0.35 × 열 간격: 중심선이 이보다 멀면 새 통로가 아니라 옆 통로/열을 본 것일 수 있음
            if (turned >= math.pi - self.p.handover_angle and row.valid
                    and row.confidence >= self.p.enter_min_conf
                    and abs(row.offset) < 0.35 * self.p.row_spacing):
                # 거의 다 돌았고 LiDAR 가 새 통로를 보면 바로 넘김: odom(GPS) 위치가 U턴 중 수십 cm 흘러도
                # 입구 정렬은 LiDAR 중심선으로 한다 (Gazebo 에서 odom 기준 끝까지 가면 입구 0.6~1 m 어긋남 확인)
                self._yaw_ref = self.plan['goal'][2]
                self.tracker = None
                self._enter(ENTER_ROW, now, pose, '다음 통로 진입 (LiDAR 인수)')
                return self.step(now, pose, row)
            if self.tracker.cross_track > self.p.max_turn_deviation:
                self.stop(f'U턴 경로 이탈 {self.tracker.cross_track:.2f} m (안전정지)')
                return 0.0, 0.0
            if self.tracker.remaining < 0.15:           # 0.15 m: 경로 끝 도달로 보는 거리
                self._yaw_ref = self.plan['goal'][2]
                self.tracker = None
                self._enter(ENTER_ROW, now, pose, '다음 통로 진입')
                return self.step(now, pose, row)
            return v * obs, w * obs          # v, w 를 같이 줄여 곡률(경로 모양)은 유지

        # ---- TURN (arc, 또는 planned 계획 실패): 반경 radius 의 반원을 일정 곡률로 돈다.
        #  탈출 → ENTER_ROW: 요가 180° − 0.2 rad (약 169°) 이상 바뀜. 목표 요 = 시작 요 + π.
        if self.state == TURN:
            turned = abs(_wrap(pose.yaw - self._mark.yaw))
            if turned >= math.pi - 0.2:
                self._yaw_ref = _wrap(self._mark.yaw + math.pi)
                self._enter(ENTER_ROW, now, pose, '다음 통로 진입')
                return self.step(now, pose, row)
            v = self.p.turn_speed * obs
            return v, self.turn_sign * v / self.radius     # w = v / R (일정 곡률 원호)

        # ---- ENTER_ROW: 새 통로로 들어가며 행 인식이 안정될 때까지 확인.
        #  탈출 → FOLLOW_ROW: 신뢰도 enter_min_conf 이상 인식이 enter_confirm_frames 연속 + 0.5 m 이상 진입.
        #        이때 통로 하나 완료, 다음 U턴 방향을 뒤집는다 (지그재그).
        #  탈출 → DONE (탐사 모드): 가까운 쪽 열만 있고 먼 쪽 열이 없음이 boundary_confirm_frames 연속.
        #  탈출 → STOPPED: max_enter_distance 넘게 가도 통로를 확정 못함.
        if self.state == ENTER_ROW:
            ok = row.valid and row.confidence >= self.p.enter_min_conf
            travelled = _dist(pose, self._mark)
            if self.explore and row.left_count >= 0:
                # 방금 왼쪽으로 돌았으면 새 통로의 먼 쪽 열은 오른쪽에 있어야 한다
                far = row.right_count if self.turn_sign > 0 else row.left_count
                near = row.left_count if self.turn_sign > 0 else row.right_count
                ok = ok and far >= self.p.min_side_trees
                edge = (travelled > self.p.boundary_check_distance and row.valid
                        and near >= self.p.min_side_trees and far == 0)
                self._boundary = self._boundary + 1 if edge else 0
                if self._boundary >= self.p.boundary_confirm_frames:
                    self.lanes_done += 1
                    self._enter(DONE, now, pose, f'과수원 끝 열 도달 (통로 {self.lanes_done}개 주행)')
                    return 0.0, 0.0
            self._counter = self._counter + 1 if ok else 0
            if self._counter >= self.p.enter_confirm_frames and travelled > 0.5:   # 0.5 m: 헤드랜드에서 오판 방지
                self.lanes_done += 1
                self.turn_sign = -self.turn_sign        # 지그재그 순회
                self._enter(FOLLOW_ROW, now, pose, f'{self.lanes_done + 1}번째 통로 주행')
                return self.step(now, pose, row)
            if travelled > self.p.max_enter_distance:
                self.stop('다음 통로를 찾지 못함 (안전정지)')
                return 0.0, 0.0
            if ok:              # 새 통로가 보이면 LiDAR 중심선으로 들어간다 (odom 요만 믿으면 비스듬히 들어감)
                v, w = row_follow_command(row.offset, row.heading, row.confidence, row.obstacle_distance, f)
                v = min(v, self.p.enter_speed)
                return v, max(-f.max_yaw_rate, min(f.max_yaw_rate, w))
            # 아직 통로가 안 보이면 U턴 목표 요를 유지하며 천천히 전진
            v, w = heading_hold_command(pose.yaw, self._yaw_ref, self.p.enter_speed)
            return v * obs, w

        return 0.0, 0.0
