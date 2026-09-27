"""rows.py (좌/우 열 적합, RowTracker, 통로 장애물) 및 ground → trunks → rows 전체 LiDAR 파이프라인 테스트.

정확한 좌표로 만든 줄기 배열(해석해 비교)과 synthetic.py 가상 점군(잔디·수관·차체 기울기 포함)을 함께 쓴다.
"""
import math

import numpy as np

from orchard_perception.ground import fit_ground_plane
from orchard_perception.rows import RowTracker, corridor_obstacle_distance, fit_rows
from orchard_perception.synthetic import SyntheticOrchard, scan
from orchard_perception.trunks import detect_trunks


def _pipeline(pts, prior=(0.0, 0.0)):
    """lidar_tree_node 와 같은 순서 (지면 → 줄기 → 행 적합) 를 기본 파라미터로 실행. 반환 (행 추정, 줄기, 지면)."""
    g = fit_ground_plane(pts, nominal_ground_z=-0.1)
    h = g.height(pts)
    trunks = detect_trunks(pts, h)
    xy = np.array([[t.x, t.y] for t in trunks]) if trunks else np.zeros((0, 2))
    w = np.array([t.confidence for t in trunks]) if trunks else np.zeros(0)
    return fit_rows(xy, w, 3.8, *prior), trunks, g


def test_fit_rows_exact():
    """잡음 없는 좌/우 열 (offset 0.4 m, heading 8°, 폭 3.8 m): 세 값을 1e-6 이내로 복원."""
    xs = np.arange(-2, 10, 1.2)
    head = math.radians(8)
    off = 0.4
    # 수직거리 1.9 m (= 3.8/2) 인 평행선의 y 절편 차이는 1.9 / cos(heading)
    left = np.c_[xs, off + 1.9 / math.cos(head) + math.tan(head) * xs]
    right = np.c_[xs, off - 1.9 / math.cos(head) + math.tan(head) * xs]
    est = fit_rows(np.vstack([left, right]), prior_offset=0.3, prior_heading=math.radians(5))
    assert est.valid
    assert abs(est.offset - off) < 1e-6
    assert abs(est.heading - head) < 1e-6
    assert abs(est.width - 3.8) < 1e-6


def test_one_side_only_uses_expected_width():
    """왼쪽 열(y=1.9 m)만 보임: expected_width 로 반대 열을 가정해 offset ≈ 0, 오른쪽 줄기 수 0."""
    xs = np.arange(0, 10, 1.2)
    left = np.c_[xs, 1.9 + 0 * xs]
    est = fit_rows(left)
    assert est.valid
    assert abs(est.offset) < 1e-6 and est.right_count == 0


def test_outlier_tree_rejected():
    """통로 안쪽 이상점 줄기 1개 (y=1.0 m): 잔차 기반 제거로 offset 오차 3 cm 미만 유지."""
    xs = np.arange(0, 10, 1.2)
    pts = np.vstack([np.c_[xs, 1.9 + 0 * xs], np.c_[xs, -1.9 + 0 * xs], [[4.0, 1.0]]])
    est = fit_rows(pts)
    assert est.valid and abs(est.offset) < 0.03


def test_synthetic_scan_pipeline_offset_and_heading():
    """잔디·수관·차체기울기가 있는 가상 점군에서 중심선 오차 10 cm / 3° 이내."""
    orchard = SyntheticOrchard(rows=4)
    rng = np.random.default_rng(3)
    for y_off, yaw_deg, pitch_deg in [(0.0, 0.0, 0.0), (0.4, 8.0, 3.0), (-0.5, -10.0, -4.0)]:
        yaw = math.radians(yaw_deg)
        pts = scan(orchard, x=8.0, y=y_off, yaw=yaw, pitch=math.radians(pitch_deg), rng=rng)
        est, trunks, g = _pipeline(pts)
        assert g.fitted
        assert len(trunks) >= 10
        assert est.valid, (y_off, yaw_deg)
        # 로봇 기준 중심선: y_c(0) = -y_off / cos(yaw), 방향 = -yaw
        assert abs(est.heading - (-yaw)) < math.radians(3.0), (est.heading, yaw)
        assert abs(est.offset - (-y_off / math.cos(yaw))) < 0.10, (est.offset, y_off)
        assert abs(est.width - 3.8) < 0.25


def test_grass_is_not_detected_as_trunk():
    """나무 없이 잔디만 빽빽한 점군 (20 점/m²): 줄기 후보가 하나도 나오지 않아야 한다."""
    orchard = SyntheticOrchard(rows=0)          # 나무 없음, 잔디만
    pts = scan(orchard, 0.0, 0.0, 0.0, grass_density=20.0, rng=np.random.default_rng(0))
    _, trunks, _ = _pipeline(pts)
    assert len(trunks) == 0


def test_tracker_holds_briefly():
    """검출이 끊겨도 hold_time(0.5 s) 이내 (0.3 s) 에는 이전 추정 유지, 지나면 (1.0 s) 무효."""
    tr = RowTracker(alpha=0.5, hold_time=0.5)
    xs = np.arange(0, 10, 1.2)
    est = fit_rows(np.vstack([np.c_[xs, 1.9 + 0 * xs], np.c_[xs, -1.9 + 0 * xs]]))
    tr.update(est, 0.0)
    held = tr.update(fit_rows(np.zeros((0, 2))), 0.3)
    assert held.valid
    gone = tr.update(fit_rows(np.zeros((0, 2))), 1.0)
    assert not gone.valid


def test_corridor_obstacle():
    """통로 정면 x=3.0~3.4 m 상자 → 거리 약 3 m, 같은 상자를 열 위치(y=+1.9 m)로 옮기면 inf."""
    rng = np.random.default_rng(0)
    box = np.c_[rng.uniform(3.0, 3.4, 200), rng.uniform(-0.3, 0.3, 200), rng.uniform(0.0, 1.2, 200)]
    heights = box[:, 2]
    d = corridor_obstacle_distance(box, heights, 0.0, 0.0, 0.5)
    assert 2.95 < d < 3.2
    # 통로 밖(나무 열 위치)의 물체는 장애물이 아님
    side = box + [0, 1.9, 0]
    assert math.isinf(corridor_obstacle_distance(side, heights, 0.0, 0.0, 0.5))
