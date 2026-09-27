"""행간중심선 추종 제어 (Pure Pursuit) — ROS 의존성 없는 순수 함수.

파이프라인에서의 역할
  행 추종(FOLLOW_ROW) 단계의 조향·속도 명령을 만든다. mission.py 의 상태기계가 매 주기 호출한다.
  (행 추종 → 행 끝 → U턴 → 다음 통로 진입 중 '행 추종'과 '행 끝 직진/진입'의 요 유지를 담당.
   U턴 경로 추종은 headland.PathTracker 가 따로 한다.)

입력 (단위·좌표계)
  - offset [m], heading [rad]: 행간중심선 y = offset + tan(heading)·x (base_link, x 전방 / y 왼쪽).
    /orchard/row (RowCenterline) 의 lateral_offset, heading_error 와 같은 값.
  - confidence [0~1]: 행 인식 신뢰도 → 속도를 줄이는 데 쓴다.
  - obstacle_distance [m]: 통로 안 최근접 장애물까지 전방 거리 (없으면 +inf).
출력
  - (v [m/s], w [rad/s]) — ROS 규약 (+w = 좌회전). row_navigator_node 가 /cmd_vel 로 낸다.

학생이 주로 바꾸는 파라미터 (FollowParams, ROS 이름은 'follow.*')
  - ros2_ws/src/orchard_bringup/config/sim.yaml (시뮬), robot.yaml (실차) 의 row_navigator_node 항목
  - follow.cruise_speed, follow.lookahead, follow.min_turn_radius, follow.stop_distance, follow.slow_distance
  - lookahead 를 키우면 부드럽지만 느리게 중심선으로 붙고, 줄이면 빨리 붙지만 좌우로 흔들린다.

References
  R. C. Coulter, "Implementation of the Pure Pursuit Path Tracking Algorithm", CMU-RI-TR-92-01, 1992.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class FollowParams:
    """행 추종 제어 파라미터 (row_navigator_node 의 'follow.*' ROS 파라미터로 채워진다)."""

    cruise_speed: float = 0.8        # [m/s] 행 안 순항 속도
    min_speed: float = 0.2           # [m/s] 신뢰도가 낮을 때 최저 속도
    lookahead: float = 2.5           # [m] 전방 주시거리
    max_yaw_rate: float = 0.8        # [rad/s]
    min_turn_radius: float = 0.9     # [m] Ackermann 최소 회전반경 = wheelbase / tan(최대조향각)
    stop_distance: float = 1.2       # [m] 통로 장애물 정지 거리
    slow_distance: float = 3.0       # [m] 감속 시작 거리
    low_confidence: float = 0.3      # 이 값 아래면 min_speed


def obstacle_speed_factor(distance: float, stop: float, slow: float) -> float:
    """장애물 거리에 따른 속도 배율 (0~1).

    Args:
        distance: 통로 안 최근접 장애물까지 전방 거리 [m] (inf/NaN = 장애물 없음).
        stop: 이 거리 이하면 0 (정지) [m].
        slow: 이 거리 이상이면 1 (감속 없음) [m].
    Returns:
        stop~slow 사이는 선형 보간한 배율.
    """
    if not math.isfinite(distance) or distance >= slow:
        return 1.0
    if distance <= stop:
        return 0.0
    return (distance - stop) / (slow - stop)


def confidence_speed(confidence: float, p: FollowParams) -> float:
    """행 인식 신뢰도 → 목표 속도 [m/s].

    신뢰도가 low_confidence 이하면 min_speed, 1.0 이면 cruise_speed, 그 사이는 선형 보간.
    (인식이 불안하면 천천히 가서 틀려도 피해를 줄인다.)
    """
    c = max(0.0, min(1.0, confidence))
    if c <= p.low_confidence:
        return p.min_speed
    ratio = (c - p.low_confidence) / (1.0 - p.low_confidence)
    return p.min_speed + ratio * (p.cruise_speed - p.min_speed)


def pure_pursuit_curvature(offset: float, heading: float, lookahead: float) -> float:
    """중심선 y = offset + tan(heading) x 위의 x=L 지점을 향하는 곡률 (base_link 기준).

    Pure Pursuit (Coulter 1992): 로봇 원점에서 목표점 (x, y) 를 지나는 원의 곡률 = 2·y / (x² + y²).
    여기서는 목표점을 '전방 x = lookahead 에서의 중심선 위 점'으로 잡는다 (호 길이 대신 x 로 근사).

    Args:
        offset: x=0 에서 중심선의 y [m] (+: 중심선이 왼쪽).
        heading: 중심선 방향 − 로봇 진행방향 [rad] (+: 왼쪽으로 틀어짐).
        lookahead: 전방 주시거리 [m].
    Returns:
        곡률 [1/m] (+: 좌회전).
    """
    y_l = offset + math.tan(heading) * lookahead
    d2 = lookahead * lookahead + y_l * y_l          # 목표점까지 거리² [m²]
    return 2.0 * y_l / d2


def row_follow_command(offset: float, heading: float, confidence: float,
                       obstacle_distance: float, p: FollowParams) -> tuple[float, float]:
    """(선속도 v [m/s], 요레이트 w [rad/s], ROS 규약: +w = 좌회전).

    1) Pure Pursuit 곡률을 구하고 Ackermann 최소 회전반경(1/min_turn_radius)으로 자른다.
    2) 속도 = 신뢰도 속도 × 장애물 배율.
    3) 요레이트 = v·κ (곡률 운동학), 그리고 max_yaw_rate 로 자른다.
    """
    kappa = pure_pursuit_curvature(offset, heading, p.lookahead)
    k_max = 1.0 / p.min_turn_radius                  # 차가 낼 수 있는 최대 곡률 [1/m]
    kappa = max(-k_max, min(k_max, kappa))
    v = confidence_speed(confidence, p) * obstacle_speed_factor(
        obstacle_distance, p.stop_distance, p.slow_distance)
    w = max(-p.max_yaw_rate, min(p.max_yaw_rate, v * kappa))
    return v, w


def heading_hold_command(yaw: float, yaw_ref: float, speed: float, k_yaw: float = 1.5,
                         max_yaw_rate: float = 0.8) -> tuple[float, float]:
    """목표 요(yaw_ref)를 유지하며 직진하는 P 제어 (행 끝 직진·통로 진입 때 사용).

    Args:
        yaw, yaw_ref: 현재/목표 요 [rad] (odom 좌표).
        speed: 선속도 [m/s] (그대로 반환).
        k_yaw: 비례 이득 [1/s] — 요 오차 1 rad 당 요레이트 1.5 rad/s.
        max_yaw_rate: 요레이트 한계 [rad/s].
    Returns:
        (v [m/s], w [rad/s]).
    """
    err = math.atan2(math.sin(yaw_ref - yaw), math.cos(yaw_ref - yaw))   # 오차를 −π~π 로 감쌈
    return speed, max(-max_yaw_rate, min(max_yaw_rate, k_yaw * err))
