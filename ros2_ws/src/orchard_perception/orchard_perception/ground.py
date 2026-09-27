"""3D 점군에서 지면 평면 추정 (RANSAC).

과수원 바닥은 잔디(5~15cm)와 요철이 있고 차체도 기울어지므로
'센서 높이 고정' 가정 대신 매 프레임 지면 평면을 추정한다.
(2D LiDAR 의 지면 오검출 문제를 3D 로 해결하는 핵심 단계)

파이프라인 내 역할
  lidar_tree_node: 점군(base_link) → [ground.py 지면 평면] → 점별 지면 높이 → trunks.py 줄기 검출
입력 / 출력 (ROS 의존성 없음)
  입력: base_link 기준 Nx3 점군 [m] (x 전방 / y 왼쪽 / z 위)
  출력: GroundPlane (법선·d). GroundPlane.height(points) 로 각 점의 '지면으로부터 높이' [m] 를 얻는다.
주요 튜닝 파라미터 (lidar_tree_node 의 ground.* 파라미터, config/sim.yaml · robot.yaml)
  ground.nominal_z           평지에서 base_link 기준 지면 z [m]. ★ 실차 장착 높이에 맞게 반드시 수정
  ground.search_band         지면 후보로 볼 z 범위 (nominal_z ± band) [m]
  ground.distance_threshold  평면까지 거리가 이보다 작으면 인라이어 [m]. 잔디 높이 정도로 설정
  ground.max_tilt_deg        허용 지면 기울기 [deg]. 이보다 기운 평면은 벽·수관으로 보고 버림
  ground.iterations / ground.min_inliers  RANSAC 반복 횟수 / 성공 판정 최소 인라이어 수

References
  M. A. Fischler, R. C. Bolles, "Random Sample Consensus: A Paradigm for Model Fitting with
  Applications to Image Analysis and Automated Cartography", Comm. ACM 24(6), 1981.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class GroundPlane:
    """지면 평면 normal · p + d = 0 (base_link 기준). fitted=False 면 추정 실패로 기본 평면 사용."""

    normal: np.ndarray       # 단위 법선 (위쪽 +z 방향으로 정규화)
    d: float                 # normal . p + d = 0
    num_inliers: int
    fitted: bool             # False 이면 기본값(차량 기준 평면) 사용

    def height(self, points: np.ndarray) -> np.ndarray:
        """각 점의 지면으로부터 높이 [m].

        normal 이 단위벡터이므로 normal·p + d 가 곧 평면까지의 부호 있는 거리 (+: 지면 위).
        points: base_link 기준 Nx3. 반환: 길이 N 배열.
        """
        if points.size == 0:
            return np.zeros(0)
        return points @ self.normal + self.d

    def tilt_deg(self) -> float:
        """지면 법선과 base_link z 축 사이 각도 [deg] (= 차체가 지면에 대해 기운 정도)."""
        # 두 단위벡터의 내적 = normal·(0,0,1) = normal[2] → arccos 로 각도. clip 은 반올림 오차 대비
        return float(np.degrees(np.arccos(np.clip(self.normal[2], -1.0, 1.0))))


def nominal_plane(ground_z: float) -> GroundPlane:
    """base_link 기준 z = ground_z 인 수평 평면 (추정 실패 시 대체값).

    법선 (0,0,1), d = -ground_z 이므로 height = z - ground_z.
    """
    return GroundPlane(normal=np.array([0.0, 0.0, 1.0]), d=-ground_z, num_inliers=0, fitted=False)


def fit_ground_plane(
    points: np.ndarray,
    nominal_ground_z: float,
    search_band: float = 0.4,
    distance_threshold: float = 0.08,
    max_tilt_deg: float = 20.0,
    iterations: int = 60,
    min_inliers: int = 150,
    rng: np.random.Generator | None = None,
) -> GroundPlane:
    """RANSAC 지면 평면 추정.

    points: base_link 기준 Nx3 점군
    nominal_ground_z: 차량이 평지에 있을 때 지면의 z (보통 -바퀴중심높이 쯤)
    search_band: 지면 후보로 쓸 z 범위 (nominal ± band)
    distance_threshold: 인라이어 판정 거리 [m] (기본 8 cm ≈ 잔디 높이 + 센서 잡음)
    max_tilt_deg: 허용 최대 지면 기울기 [deg]
    iterations: RANSAC 반복 횟수. min_inliers: 성공으로 인정할 최소 인라이어 수
    rng: 난수 생성기 (재현성을 위해 기본 seed 0)
    반환: GroundPlane. 실패하면 nominal 평면(fitted=False)을 돌려준다.

    절차: (1) z 가 nominal 근처인 점만 후보로 → (2) 3점 무작위 샘플로 평면 가설 → 인라이어 수 최대인 가설 선택
    → (3) 인라이어 전체로 최소제곱 재추정 (RANSAC 의 표준 마무리 단계).
    """
    rng = rng or np.random.default_rng(0)
    fallback = nominal_plane(nominal_ground_z)
    if points.shape[0] < 3:
        return fallback

    # 1) 지면 후보: nominal 높이 ± band. 수관·줄기 윗부분 점을 미리 빼서 RANSAC 이 엉뚱한 평면에 붙지 않게
    cand = points[np.abs(points[:, 2] - nominal_ground_z) < search_band]
    if cand.shape[0] < max(3, min_inliers):
        return fallback

    best_count = 0
    best_normal = None
    best_d = 0.0
    # 법선의 z 성분 = cos(기울기). cos_max 보다 작으면 max_tilt_deg 보다 많이 기운 평면
    cos_max = np.cos(np.radians(max_tilt_deg))
    n = cand.shape[0]
    # 2) RANSAC 가설-검증 반복
    for _ in range(iterations):
        idx = rng.choice(n, 3, replace=False)
        p0, p1, p2 = cand[idx]
        normal = np.cross(p1 - p0, p2 - p0)       # 세 점이 이루는 평면의 법선
        norm = np.linalg.norm(normal)
        if norm < 1e-9:                           # 세 점이 거의 한 직선 위 → 평면 정의 불가
            continue
        normal = normal / norm
        if normal[2] < 0:                         # 법선이 항상 위(+z)를 향하게 → height 부호 일관
            normal = -normal
        if normal[2] < cos_max:                   # 너무 기운 평면은 지면이 아님
            continue
        d = -float(normal @ p0)
        count = int(np.count_nonzero(np.abs(cand @ normal + d) < distance_threshold))
        if count > best_count:
            best_count, best_normal, best_d = count, normal, d

    if best_normal is None or best_count < min_inliers:
        return fallback

    # 인라이어로 최소제곱 재추정 (z = ax + by + c)
    # 3점 가설은 잡음에 민감하므로 인라이어 전체로 다시 맞춰 정확도를 높인다.
    inl = cand[np.abs(cand @ best_normal + best_d) < distance_threshold]
    a_mat = np.c_[inl[:, 0], inl[:, 1], np.ones(inl.shape[0])]
    coef, *_ = np.linalg.lstsq(a_mat, inl[:, 2], rcond=None)
    # z = a x + b y + c  ⇔  -a x - b y + z - c = 0  → 법선 (-a, -b, 1), d = -c (정규화 전)
    normal = np.array([-coef[0], -coef[1], 1.0])
    scale = np.linalg.norm(normal)
    normal /= scale
    d = -coef[2] / scale
    if normal[2] < cos_max:                       # 재추정 결과가 기울기 한계를 넘으면 신뢰하지 않음
        return fallback
    return GroundPlane(normal=normal, d=float(d), num_inliers=int(inl.shape[0]), fitted=True)
