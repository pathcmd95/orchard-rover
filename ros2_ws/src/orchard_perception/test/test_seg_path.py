"""seg_path.py (주행가능 마스크 → 지면 역투영 → 통로 중심선) 테스트.

가상 카메라(높이 0.5 m, 5.7° 숙임, 수평 화각 90°) 로 폭 2.6 m 통로 마스크를 만들어 중심선을 복원해 본다.
"""
import math

import numpy as np

from orchard_perception.seg_path import centerline, ground_points


def _camera(height=0.5, pitch=0.10, w=320, h=240, hfov=math.radians(90)):
    """base_link 에서 카메라(광학) 변환과 내부 행렬."""
    f = (w / 2) / math.tan(hfov / 2)
    k = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1.0]])
    # 광학(z 전방, x 오른쪽, y 아래) → base(x 전방, y 왼쪽, z 위), 아래로 pitch 만큼 숙임
    r_opt = np.array([[0, 0, 1.0], [-1.0, 0, 0], [0, -1.0, 0]])
    cp, sp = math.cos(pitch), math.sin(pitch)
    r_pitch = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    T = np.eye(4)
    T[:3, :3] = r_pitch @ r_opt
    T[:3, 3] = (0.45, 0.0, height)
    return k, T, (w, h)


def _lane_mask(k, T, size, offset, heading, half_width=1.3, ground_z=-0.1):
    """모든 픽셀을 지면에 쏴서, 통로(중심선 y = offset + tan(heading)·x, 폭 ±half_width) 안이면 True."""
    w, h = size
    u, v = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)
    rays = np.stack([(u - k[0, 2]) / k[0, 0], (v - k[1, 2]) / k[1, 1], np.ones_like(u)], -1) @ T[:3, :3].T
    s = (ground_z - T[2, 3]) / rays[..., 2]
    x = T[0, 3] + rays[..., 0] * s
    y = T[1, 3] + rays[..., 1] * s
    c = math.cos(heading)
    lateral = (y - offset - math.tan(heading) * x) * c      # 중심선까지 수직 거리
    return (s > 0) & (np.abs(lateral) < half_width) & (x < 15)


def test_recovers_offset_and_heading():
    """여러 (offset, heading) 통로: offset 5 cm, heading 1.5°, 폭 35 cm 이내로 복원, 신뢰도 > 0.5."""
    k, T, size = _camera()
    for off, head in [(0.0, 0.0), (0.4, math.radians(8)), (-0.5, math.radians(-10))]:
        mask = _lane_mask(k, T, size, off, head)
        pts, border = ground_points(mask, k, T, ground_z=-0.1, stride=2)
        est = centerline(pts, border)
        assert est.valid, (off, head)
        assert abs(est.offset - off) < 0.05, (est.offset, off)
        assert abs(est.heading - head) < math.radians(1.5), (math.degrees(est.heading), math.degrees(head))
        assert abs(est.width - 2.6) < 0.35
        assert est.confidence > 0.5, est


def test_clipped_edges_do_not_bias_center():
    """통로가 화면 한쪽 밖으로 나가 있어도(로봇이 치우침) 중심 추정이 끌려가지 않는다."""
    k, T, size = _camera()
    mask = _lane_mask(k, T, size, 1.0, 0.0)
    est = centerline(*ground_points(mask, k, T, -0.1, stride=2))
    assert est.valid and abs(est.offset - 1.0) < 0.08, est


def test_no_drivable_pixels_is_invalid():
    """주행가능 픽셀이 하나도 없으면 valid=False."""
    k, T, size = _camera()
    est = centerline(*ground_points(np.zeros((size[1], size[0]), bool), k, T, -0.1))
    assert not est.valid
