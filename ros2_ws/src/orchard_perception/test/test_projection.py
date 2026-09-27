"""projection.py(줄기 → 영상 투영, IoU, 매칭, YOLO 라벨) · yolo.py(출력 해석) · geometry.py(역변환) 테스트.

카메라는 base_link 앞 0.45 m, 높이 0.40 m, 정면을 보는 640×480 영상 (fx = fy = 320 px, 수평 화각 90°).
"""
import math

import numpy as np

from orchard_perception.geometry import invert_transform, make_transform, rpy_to_matrix
from orchard_perception.projection import Box, greedy_match, iou, trunk_box, yolo_label
from orchard_perception.yolo import decode_output, letterbox_params

K = np.array([[320.0, 0, 320.0], [0, 320.0, 240.0], [0, 0, 1]])    # fx=fy=320, (cx, cy)=(320, 240)


def _t_cam_base(cam_xyz=(0.45, 0.0, 0.40), pitch=0.0):
    """base_link → 카메라 광학좌표 4x4 (T_cam_base). cam_xyz: 카메라 위치 [m], pitch: 아래로 숙임 [rad]."""
    # base_link -> camera_link (x 전방) -> optical (z 전방)
    t_base_cam = np.eye(4)
    # rpy(-90°, 0, -90°) = ROS 표준 camera_link(x 전방) → optical(z 전방, x 오른쪽, y 아래) 회전
    t_base_cam[:3, :3] = rpy_to_matrix(0.0, pitch, 0.0) @ rpy_to_matrix(-math.pi / 2, 0.0, -math.pi / 2)
    t_base_cam[:3, 3] = cam_xyz
    return invert_transform(t_base_cam)


def test_tree_straight_ahead_projects_to_center_column():
    """정면 5 m 줄기: 박스 가로 중심이 영상 중앙 열(u=320) ±2 px, 박스 아래쪽이 영상 중앙보다 아래."""
    box = trunk_box((5.0, 0.0), -0.1, 0.07, 0.8, _t_cam_base(), K, (640, 480))
    assert box is not None
    assert abs((box.x1 + box.x2) / 2 - 320) < 2
    assert box.y2 > 240        # 줄기는 카메라보다 낮으므로 화면 아래쪽


def test_tree_on_left_projects_left_and_behind_is_none():
    """왼쪽(+y) 줄기는 영상 왼쪽 절반에 투영되고, 로봇 뒤(x=-3 m) 줄기는 None."""
    left = trunk_box((4.0, 1.9), -0.1, 0.07, 0.8, _t_cam_base(), K, (640, 480))
    assert left is not None and left.x2 < 320
    assert trunk_box((-3.0, 0.0), -0.1, 0.07, 0.8, _t_cam_base(), K, (640, 480)) is None


def test_iou_and_match():
    """IoU 계산값 (반쯤 겹친 두 박스 = 1/3) 과 greedy_match 가 올바른 1:1 쌍을 찾는지 확인."""
    a, b = Box(0, 0, 10, 10), Box(5, 0, 15, 10)
    assert abs(iou(a, b) - 1 / 3) < 1e-9
    pairs = greedy_match([a, Box(100, 100, 110, 140)], [Box(101, 95, 111, 150), b])
    assert set(pairs) == {(0, 1), (1, 0)}


def test_yolo_label_normalized():
    """YOLO 라벨 문자열: 클래스 0, 중심·크기가 영상 크기로 나눈 0~1 값."""
    line = yolo_label(Box(100, 200, 140, 400), (640, 480))
    cls, cx, cy, w, h = line.split()
    assert cls == '0' and abs(float(cx) - 120 / 640) < 1e-6 and abs(float(h) - 200 / 480) < 1e-6


def test_decode_yolov8_output():
    """YOLOv8 형식 (1, 4+nc, N) 출력: 점수 0.9 후보 1개만 남고, letterbox 여백을 뺀 원본 좌표로 복원."""
    scale, px, py, _, _ = letterbox_params(640, 480, 640)
    n = 50
    out = np.zeros((1, 5, n), dtype=np.float32)       # (4 + 1클래스, N)
    out[0, :, 7] = [320, 320, 40, 200, 0.9]            # 네트워크 입력 좌표
    boxes, scores, cls = decode_output(out, 0.5, scale, px, py, 640, 480)
    assert len(scores) == 1 and cls[0] == 0
    x, y, w, h = boxes[0]
    assert abs(x + w / 2 - 320) < 1e-3 and abs(y + h / 2 - (320 - py)) < 1e-3


def test_transform_roundtrip():
    """T · T^-1 = 단위행렬 (invert_transform 이 강체 변환의 역을 정확히 계산)."""
    t = make_transform((1, 2, 3), (0, 0, math.sin(0.3), math.cos(0.3)))
    assert np.allclose(t @ invert_transform(t), np.eye(4))
