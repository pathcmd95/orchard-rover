"""좌표 변환 유틸리티 (ROS 의존성 없음, numpy 만 사용).

파이프라인 내 역할
  모든 인식 노드가 공통으로 쓰는 기하 함수 모음. TF 에서 받은 (평행이동, 쿼터니언) 을
  4x4 동차변환행렬로 바꾸고, 점군을 센서 좌표계 → base_link 로 옮기는 데 쓴다.
  (lidar_tree_node, tree_fusion_node, seg_path_node, synthetic 이 사용)

좌표계·단위 규약
  - ROS REP-103: base_link 는 x 전방 / y 왼쪽 / z 위, 단위 m, 각도 rad.
  - 쿼터니언 순서는 ROS 메시지와 같은 (x, y, z, w).
  - 4x4 행렬 T_a_b 는 "b 좌표계의 점을 a 좌표계로" 바꾼다: p_a = R · p_b + t.
  - 점군은 Nx3 numpy 배열 (행 하나가 점 하나).

튜닝 파라미터 없음. 모든 함수는 단위테스트가 가능하도록 순수 함수로 작성한다.
"""
from __future__ import annotations

import numpy as np


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """쿼터니언(x, y, z, w) -> 3x3 회전행렬.

    정규화되지 않은 쿼터니언도 받을 수 있게 s = 2/|q|^2 로 나눠 준다.
    길이가 0 에 가까운(잘못된) 쿼터니언은 단위행렬(회전 없음)로 처리한다.
    """
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:               # 사실상 0 벡터 → 회전 정의 불가, 항등으로 대체
        return np.eye(3)
    s = 2.0 / n                 # 단위 쿼터니언이면 s = 2
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    # 표준 쿼터니언 → 회전행렬 공식
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


def make_transform(translation, quaternion_xyzw) -> np.ndarray:
    """평행이동 + 쿼터니언 -> 4x4 동차변환행렬.

    translation: (x, y, z) [m], quaternion_xyzw: (x, y, z, w).
    TF lookup_transform(target, source) 결과를 넣으면 source → target 변환이 된다.
    """
    t = np.eye(4)
    t[:3, :3] = quaternion_to_matrix(*quaternion_xyzw)
    t[:3, 3] = np.asarray(translation, dtype=float)
    return t


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ROS 규약(고정축 X-Y-Z 순서) roll/pitch/yaw [rad] -> 3x3 회전행렬.

    고정축 X→Y→Z 회전은 R = Rz(yaw) · Ry(pitch) · Rx(roll) 과 같다 (URDF <origin rpy> 와 동일).
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    """Nx3 점군에 4x4 변환 적용 (p' = R p + t). 빈 입력은 (0, 3) 배열로 돌려준다."""
    if points.size == 0:
        return points.reshape(0, 3)
    # 행벡터 표기이므로 R 대신 R^T 를 오른쪽에 곱한다
    return points @ transform[:3, :3].T + transform[:3, 3]


def invert_transform(transform: np.ndarray) -> np.ndarray:
    """강체 변환의 역행렬 (T_a_b → T_b_a).

    회전은 직교행렬이므로 R^-1 = R^T, 평행이동은 -R^T t. np.linalg.inv 보다 빠르고 수치적으로 안정.
    """
    inv = np.eye(4)
    r = transform[:3, :3]
    inv[:3, :3] = r.T
    inv[:3, 3] = -r.T @ transform[:3, 3]
    return inv


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """쿼터니언에서 yaw(z 축 회전, rad, -π~π) 만 뽑는다 (ZYX 오일러각 공식)."""
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def wrap_angle(a: float) -> float:
    """각도 a [rad] 를 [-π, π) 범위로 감싼다 (예: 3π/2 → -π/2)."""
    return float((a + np.pi) % (2.0 * np.pi) - np.pi)
