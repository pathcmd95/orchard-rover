"""출발점 기준 상대좌표계('start' 프레임) — ROS 의존성 없음.

왜 따로 두나
  /odom 좌표(PX4 EKF)는 원점이 PX4 가 켜진 곳, x 축이 동쪽(ENU)이다. 사람이 과수원에서 쓰기에는
  "내가 출발한 곳이 (0, 0, 0), 처음 달린 방향이 +x" 인 좌표가 알아보기 쉽다. 모든 기록(복셀 맵, 줄기 위치,
  주행 궤적)은 이 좌표로 저장한다.

정의
  원점   = 미션 시작 순간 로버(base_link) 위치 (odom 좌표)
  +x 축  = 첫 통로 방향. 전역 계획(nav_mode planner)이 추정한 통로 방향(/orchard/row_frame)을 받으면 그것을,
           못 받으면 출발 후 처음 달린 방향 (course_distance 만큼 달린 이동 방향, 위치 차로 계산)
  +z 축  = 위 (odom 과 같음, 기울기는 무시)
  나침반(자력계) 요는 정지 중 수십 도 틀릴 수 있어 요 대신 실제 이동 방향을 쓴다.

좌표 변환
  p_start = R(-yaw0) · (p_odom - p0)      (z 는 z_odom - z0)
"""
from __future__ import annotations

import math

import numpy as np


class StartFrame:
    """odom ↔ start 좌표 변환. begin() 으로 원점을, observe() 로 +x 방향을 정한다."""

    def __init__(self, course_distance: float = 1.5):
        """course_distance: +x 방향을 정하기 위해 곧게 달릴 거리 [m]."""
        self.course_distance = course_distance
        self.origin: np.ndarray | None = None      # 원점 (odom x, y, z)
        self.yaw0: float | None = None             # start +x 축의 odom 방위 [rad]

    @property
    def ready(self) -> bool:
        """원점과 +x 방향이 모두 정해졌으면 True (그 전에는 변환을 쓰지 않는다)."""
        return self.origin is not None and self.yaw0 is not None

    def begin(self, x: float, y: float, z: float) -> None:
        """미션 시작 순간 odom 위치를 원점으로 기억."""
        self.origin = np.array([x, y, z], float)
        self.yaw0 = None

    def observe(self, x: float, y: float) -> bool:
        """odom 위치를 넣어 주면, 원점에서 course_distance 이상 멀어진 순간 +x 방향을 정한다. 정해지면 True."""
        if self.origin is None or self.yaw0 is not None:
            return self.ready
        dx, dy = x - self.origin[0], y - self.origin[1]
        if math.hypot(dx, dy) >= self.course_distance:
            self.yaw0 = math.atan2(dy, dx)
        return self.ready

    def set_axis(self, yaw: float) -> None:
        """+x 방향을 직접 지정 (전역 계획이 추정한 통로 방향을 받을 때). 원점은 begin() 값 그대로."""
        if self.origin is not None:
            self.yaw0 = float(yaw)

    def set(self, x: float, y: float, z: float, yaw: float) -> None:
        """원점과 방향을 직접 지정 (시험·재생용)."""
        self.origin = np.array([x, y, z], float)
        self.yaw0 = float(yaw)

    def to_start(self, pts: np.ndarray) -> np.ndarray:
        """odom 좌표 점 (N×3 또는 N×2) → start 좌표."""
        P = np.asarray(pts, float)
        c, s = math.cos(self.yaw0), math.sin(self.yaw0)
        d = P[:, :2] - self.origin[:2]
        out = np.empty_like(P)
        out[:, 0] = c * d[:, 0] + s * d[:, 1]           # R(-yaw0) 적용
        out[:, 1] = -s * d[:, 0] + c * d[:, 1]
        if P.shape[1] > 2:
            out[:, 2] = P[:, 2] - self.origin[2]
        return out

    def yaw_to_start(self, yaw_odom: float) -> float:
        """odom 방위 → start 방위 [rad]."""
        return math.atan2(math.sin(yaw_odom - self.yaw0), math.cos(yaw_odom - self.yaw0))

    def matrix_odom_from_start(self) -> np.ndarray:
        """start → odom 4×4 변환 (TF 'odom → start' 로 발행할 때 사용)."""
        c, s = math.cos(self.yaw0), math.sin(self.yaw0)
        T = np.eye(4)
        T[:2, :2] = [[c, -s], [s, c]]
        T[:3, 3] = self.origin
        return T
