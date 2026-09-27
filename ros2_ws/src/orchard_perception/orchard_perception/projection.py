"""LiDAR 줄기 → 카메라 영상 투영 (LiDAR-Vision Calibration 결과 활용).

카메라 광학좌표계(optical frame): z 전방, x 오른쪽, y 아래.
T_cam_base: base_link 좌표를 카메라 광학좌표로 바꾸는 4x4 행렬.

파이프라인 내 역할
  tree_fusion_node 가 사용. LiDAR 줄기(/orchard/trees, base_link) 를 영상 박스로 투영해
  (1) 카메라 YOLO 검출과 짝지어 camera_confirmed 표시, (2) YOLO 학습 라벨 자동 생성에 쓴다.
  Box 클래스는 yolo.py(검출 결과) 에서도 공통으로 쓴다.
입력 / 출력 (ROS 의존성 없음)
  입력: 줄기 위치·반경·높이 [m, base_link], T_cam_base (4x4), 내부행렬 K (3x3, CameraInfo.k), 영상 크기 [px]
  출력: 픽셀 좌표 박스 Box(x1, y1, x2, y2) [px, 원점 = 영상 왼쪽 위], 매칭 쌍, YOLO 라벨 문자열
주요 튜닝 파라미터 (tree_fusion_node, config/sim.yaml · robot.yaml 또는 노드 기본값)
  trunk_label_height  투영할 줄기 높이 [m] (지면~수관 시작), iou_threshold  매칭 IoU 기준
  카메라 외부 파라미터(T_cam_base) 는 URDF/TF (orchard_description) 에서 오므로 캘리브레이션 결과를 거기에 반영.

References
  핀홀 카메라 투영 모델: R. Hartley, A. Zisserman, "Multiple View Geometry in Computer Vision",
  2nd ed., Cambridge University Press, 2004. (렌즈 왜곡은 무시 — 왜곡이 큰 카메라를 쓰면 CameraInfo.d 로 보정 필요)
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class Box:
    """영상 축정렬 박스 [px]. (x1, y1) 왼쪽 위, (x2, y2) 오른쪽 아래. score: 신뢰도, cls: 클래스 번호."""

    x1: float
    y1: float
    x2: float
    y2: float
    score: float = 1.0
    cls: int = 0

    @property
    def w(self):
        """박스 너비 [px]."""
        return self.x2 - self.x1

    @property
    def h(self):
        """박스 높이 [px]."""
        return self.y2 - self.y1

    def as_tuple(self):
        """(x1, y1, x2, y2) 튜플."""
        return (self.x1, self.y1, self.x2, self.y2)


def project(points_cam: np.ndarray, k: np.ndarray, min_depth: float = 0.2):
    """광학좌표 Nx3 -> 픽셀 Nx2, 유효 마스크.

    핀홀 모델: u = fx·X/Z + cx, v = fy·Y/Z + cy (K = [[fx,0,cx],[0,fy,cy],[0,0,1]]).
    min_depth [m]: 이보다 가깝거나 카메라 뒤(Z ≤ 0)인 점은 투영하지 않음 (0 나누기·뒤집힘 방지).
    반환: (uv Nx2 [px], 무효 점은 NaN), valid 길이 N bool.
    """
    z = points_cam[:, 2]
    valid = z > min_depth
    uv = np.full((points_cam.shape[0], 2), np.nan)
    uv[valid, 0] = k[0, 0] * points_cam[valid, 0] / z[valid] + k[0, 2]
    uv[valid, 1] = k[1, 1] * points_cam[valid, 1] / z[valid] + k[1, 2]
    return uv, valid


def trunk_box(
    base_xy: tuple[float, float],
    ground_z: float,
    radius: float,
    height: float,
    t_cam_base: np.ndarray,
    k: np.ndarray,
    image_size: tuple[int, int],
    min_box_px: int = 6,
    samples: int = 12,
) -> Box | None:
    """base_link 좌표의 원기둥(줄기)을 영상 박스로 투영. 화면 밖이면 None.

    base_xy: 줄기 중심 (x, y) [m], ground_z: 그 위치 지면 z [m], radius/height: 원기둥 반경·높이 [m]
    t_cam_base: base_link → 카메라 광학 4x4, k: 3x3 내부행렬, image_size: (너비, 높이) [px]
    min_box_px: 이보다 작은(너무 먼) 박스는 버림 [px], samples: 원 둘레 샘플 수
    원기둥의 아래·위 원 둘레 점들을 투영한 뒤 그 외접 사각형을 박스로 쓴다.
    """
    w_img, h_img = image_size
    ang = np.linspace(0, 2 * np.pi, samples, endpoint=False)
    ring = np.c_[base_xy[0] + radius * np.cos(ang), base_xy[1] + radius * np.sin(ang)]
    # 아래 원(지면) + 위 원(지면 + height) → 2·samples 개 점
    pts = np.vstack([
        np.c_[ring, np.full(samples, ground_z)],
        np.c_[ring, np.full(samples, ground_z + height)],
    ])
    cam = pts @ t_cam_base[:3, :3].T + t_cam_base[:3, 3]    # base_link → 카메라 광학좌표
    uv, valid = project(cam, k)
    if np.count_nonzero(valid) < samples:      # 절반 이상이 카메라 뒤쪽이면 버림
        return None
    uv = uv[valid]
    x1, y1 = uv.min(axis=0)
    x2, y2 = uv.max(axis=0)
    if x2 < 0 or y2 < 0 or x1 >= w_img or y1 >= h_img:     # 박스 전체가 화면 밖
        return None
    # 화면에 걸친 박스는 화면 안쪽으로 잘라냄
    x1, x2 = np.clip([x1, x2], 0, w_img - 1)
    y1, y2 = np.clip([y1, y2], 0, h_img - 1)
    if (x2 - x1) < min_box_px or (y2 - y1) < min_box_px:
        return None
    return Box(float(x1), float(y1), float(x2), float(y2))


def iou(a: Box, b: Box) -> float:
    """두 박스의 IoU (교집합 넓이 / 합집합 넓이, 0~1)."""
    ix = max(0.0, min(a.x2, b.x2) - max(a.x1, b.x1))    # 가로 겹침 길이 (겹치지 않으면 0)
    iy = max(0.0, min(a.y2, b.y2) - max(a.y1, b.y1))    # 세로 겹침 길이
    inter = ix * iy
    union = a.w * a.h + b.w * b.h - inter
    return float(inter / union) if union > 0 else 0.0


def horizontal_overlap(a: Box, b: Box) -> float:
    """줄기는 세로로 길고 수관에 가려지기 쉬워 가로 겹침 비율도 함께 쓴다.

    가로 겹침 길이 / 두 박스 중 좁은 쪽 너비 (0~1). 세로 길이가 달라도 같은 줄기면 1 에 가깝다.
    """
    ix = max(0.0, min(a.x2, b.x2) - max(a.x1, b.x1))
    denom = min(a.w, b.w)
    return float(ix / denom) if denom > 0 else 0.0


def greedy_match(proj: list[Box], dets: list[Box], iou_thr: float = 0.2, hov_thr: float = 0.5):
    """투영 박스와 영상 검출 박스를 탐욕적으로 짝지음. [(i_proj, j_det), ...]

    IoU ≥ iou_thr 또는 가로 겹침 ≥ hov_thr 인 쌍만 후보로 두고, 점수(IoU + 0.5·가로겹침)가 높은 쌍부터
    1:1 로 확정한다. 헝가리안 최적 할당보다 단순하지만 줄기 수가 적어 충분.
    """
    pairs = []
    for i, p in enumerate(proj):
        for j, d in enumerate(dets):
            s = iou(p, d)
            if s >= iou_thr or horizontal_overlap(p, d) >= hov_thr:
                # 가로 겹침은 보조 지표라 가중치 0.5
                pairs.append((s + 0.5 * horizontal_overlap(p, d), i, j))
    pairs.sort(reverse=True)                 # 점수 높은 순
    used_i, used_j, out = set(), set(), []
    for _, i, j in pairs:
        if i in used_i or j in used_j:       # 이미 짝지어진 박스는 건너뜀 (1:1 보장)
            continue
        used_i.add(i)
        used_j.add(j)
        out.append((i, j))
    return out


def yolo_label(box: Box, image_size: tuple[int, int], cls: int = 0) -> str:
    """YOLO 학습 라벨 한 줄: 'cls cx cy w h' (0~1 정규화).

    Ultralytics YOLO 데이터셋 형식: 박스 중심·크기를 영상 너비/높이로 나눈 값. image_size: (너비, 높이) [px].
    """
    w_img, h_img = image_size
    cx = (box.x1 + box.x2) / 2.0 / w_img
    cy = (box.y1 + box.y2) / 2.0 / h_img
    return f"{cls} {cx:.6f} {cy:.6f} {box.w / w_img:.6f} {box.h / h_img:.6f}"
