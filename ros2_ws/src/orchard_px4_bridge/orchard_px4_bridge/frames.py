"""PX4(NED/FRD) ↔ ROS(ENU/FLU) 좌표 변환 — ROS 의존성 없음.

PX4: 월드 NED(북-동-아래), 기체 FRD(앞-오른쪽-아래)
ROS(REP-103): 월드 ENU(동-북-위), 기체 FLU(앞-왼쪽-위)

역할
  offboard_node / state_node 가 쓰는 순수 수학 함수 모음. numpy 만 쓰므로 ROS 없이 pytest 로 시험한다
  (test/test_frames.py).
입력/출력 규약
  - 벡터: 길이 3 (x, y, z) 배열/리스트, 단위는 호출하는 쪽 그대로 (위치 [m], 속도 [m/s], 각속도 [rad/s]).
  - 쿼터니언: PX4(px4_msgs VehicleOdometry.q) 는 (w, x, y, z) 순서, ROS(geometry_msgs/Quaternion) 는 (x, y, z, w).
    둘 다 Hamilton 규약이며 '기체 → 월드' 회전 (PX4: FRD→NED, ROS: FLU→ENU).
  - 각도: [rad]. PX4 요는 북=0, 시계방향(+) / ROS 요는 동=0, 반시계방향(+).
조정할 곳
  이 파일에는 파라미터가 없다. 요레이트→목표 요 변환의 horizon 은 offboard_node 의 'yaw_horizon' 파라미터.

참고
  - REP-103 "Standard Units of Measure and Coordinate Conventions" (ENU/FLU, 단위)
  - REP-105 "Coordinate Frames for Mobile Platforms" (odom → base_link)
  - PX4 User Guide, "ROS 2 User Guide" 의 ROS 2/PX4 frame conventions 절 (NED/FRD ↔ ENU/FLU)
  - S. W. Shepperd, "Quaternion from Rotation Matrix", Journal of Guidance and Control 1(3), 1978
    (matrix_to_quat_xyzw 의 수치적으로 안정한 분기 방식)
"""
from __future__ import annotations

import numpy as np

# 월드 좌표 NED → ENU: x/y 를 맞바꾸고(북↔동) z 부호를 뒤집는다(아래→위).
#   (x_E, y_N, z_U) = (y_NED(동), x_NED(북), -z_NED(아래))  → 행렬이 대칭이고 자기 자신이 역행렬.
NED_TO_ENU = np.array([[0.0, 1.0, 0.0],
                       [1.0, 0.0, 0.0],
                       [0.0, 0.0, -1.0]])
# 기체 좌표 FRD → FLU: x(앞)는 그대로, y(오른쪽→왼쪽)·z(아래→위) 부호만 뒤집는다 (x 축 180° 회전). 역행렬도 자기 자신.
FRD_TO_FLU = np.diag([1.0, -1.0, -1.0])


def quat_wxyz_to_matrix(q) -> np.ndarray:
    """쿼터니언 (w, x, y, z) → 3×3 회전행렬 (Hamilton 규약, 기체 벡터를 월드로 돌리는 행렬).

    q 가 단위 쿼터니언이 아니어도 s = 2/|q|² 로 정규화된 결과를 준다. |q|≈0 이면 단위행렬.
    """
    w, x, y, z = q
    n = w * w + x * x + y * y + z * z
    if n < 1e-12:          # 0 쿼터니언(값 없음) → 회전 없음으로 처리
        return np.eye(3)
    s = 2.0 / n
    # 표준 공식 R = I + 2w[v]× + 2[v]×²  (v = (x, y, z)) 를 성분으로 푼 것
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - w * z), s * (x * z + w * y)],
        [s * (x * y + w * z), 1 - s * (x * x + z * z), s * (y * z - w * x)],
        [s * (x * z - w * y), s * (y * z + w * x), 1 - s * (x * x + y * y)],
    ])


def matrix_to_quat_xyzw(r: np.ndarray):
    """회전행렬 -> (x, y, z, w). Shepperd 방법.

    trace 와 대각 성분 중 가장 큰 것을 골라 그 성분으로 나누므로 0 에 가까운 수로 나누는 일이 없다.
    결과는 정규화하고 w ≥ 0 이 되게 부호를 맞춘다 (q 와 -q 는 같은 회전이므로 출력을 하나로 고정).
    """
    tr = np.trace(r)
    if tr > 0:                                           # w 가 가장 큰 경우: s = 4w
        s = np.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:       # x 가 가장 큰 경우: s = 4x
        s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:                              # y 가 가장 큰 경우: s = 4y
        s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:                                                # z 가 가장 큰 경우: s = 4z
        s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    q /= np.linalg.norm(q)
    if q[3] < 0:
        q = -q
    return tuple(float(v) for v in q)


def ned_to_enu(v) -> np.ndarray:
    """월드 벡터 NED → ENU (위치 [m], 속도 [m/s] 등). (N, E, D) → (E, N, -D)."""
    return NED_TO_ENU @ np.asarray(v, dtype=float)


def frd_to_flu(v) -> np.ndarray:
    """기체 벡터 FRD → FLU (속도, 각속도, 가속도 등). (F, R, D) → (F, -R, -D)."""
    return FRD_TO_FLU @ np.asarray(v, dtype=float)


def attitude_ned_frd_to_enu_flu(q_wxyz):
    """PX4 자세(FRD→NED, w,x,y,z) -> ROS 자세(FLU→ENU, x,y,z,w).

    반환: (쿼터니언 (x, y, z, w), 회전행렬 R_enu_flu 3×3). R 은 기체(FLU) 벡터를 ENU 로 돌린다
    (state_node 는 R.T 로 월드 속도를 기체 속도로 바꾸는 데 쓴다).
    """
    # R_enu←flu = R_enu←ned · R_ned←frd · R_frd←flu.  FRD↔FLU 는 자기 자신이 역행렬이라 FRD_TO_FLU 를 그대로 쓴다.
    r = NED_TO_ENU @ quat_wxyz_to_matrix(q_wxyz) @ FRD_TO_FLU
    return matrix_to_quat_xyzw(r), r


def ros_yaw_rate_to_px4(yaw_rate_flu: float) -> float:
    """ROS +z(반시계, 좌회전) 요레이트 -> PX4 NED 요레이트(+ 시계, 우회전)."""
    # FLU 의 z(위) 와 FRD 의 z(아래) 가 반대 방향이므로 부호만 바뀐다 [rad/s].
    return -float(yaw_rate_flu)


def yaw_from_quat_wxyz(q_wxyz) -> float:
    """PX4 자세 쿼터니언(FRD→NED) → NED 요 [rad] (북=0, 동=+π/2)."""
    w, x, y, z = (float(v) for v in q_wxyz)
    # ZYX(요-피치-롤) 오일러 분해의 요 성분: atan2(R[1,0], R[0,0]). 범위 (-π, π].
    return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def cmd_vel_to_ned_velocity(v: float, yaw_rate_flu: float, yaw_ned: float, horizon: float):
    """cmd_vel(전진 속도, ROS 요레이트) → PX4 로버 Offboard TrajectorySetpoint.velocity (북, 동) [m/s].

    PX4 v1.17 Ackermann Offboard(velocity) 는 속도 = |v_NED|, 목표 요 = atan2(v_E, v_N) 로 해석한다.
    목표 요를 '현재 요 + 요레이트 × horizon' 으로 주면 PX4 요 P 제어(RO_YAW_P)가
    요레이트 ≈ RO_YAW_P × horizon × 요청 요레이트 를 만든다 → horizon = 1 / RO_YAW_P 로 두면 요청과 같아진다.
    후진은 표현할 수 없으므로 v < 0 은 0 으로 처리한다.

    인자: v 전진 속도 [m/s] (FLU x), yaw_rate_flu ROS 요레이트 [rad/s] (+ = 좌회전),
          yaw_ned 현재 NED 요 [rad], horizon 예측 시간 [s].
    반환: (v_N, v_E) [m/s].
    """
    v = max(0.0, float(v))
    target = yaw_ned + ros_yaw_rate_to_px4(yaw_rate_flu) * horizon      # 목표 NED 요 [rad]
    # 크기 v, 방향 target 인 수평 속도 벡터. NED 에서 요 ψ 방향 단위벡터 = (cos ψ, sin ψ) (북, 동).
    return v * float(np.cos(target)), v * float(np.sin(target))
