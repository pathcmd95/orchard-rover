"""ground.fit_ground_plane (RANSAC 지면 추정) 테스트.

평지·기울어진 지면에서 평면을 정확히 찾는지, 지면 점이 없을 때 nominal 평면으로 대체하는지 확인.
"""
import numpy as np

from orchard_perception.geometry import rpy_to_matrix
from orchard_perception.ground import fit_ground_plane


def _plane_points(roll, pitch, n=3000, z0=-0.1, noise=0.01, seed=0):
    """z = z0 평면 위 점 n 개를 roll/pitch [rad] 만큼 회전하고 잡음 σ=noise [m] 를 더한 base_link 점군."""
    rng = np.random.default_rng(seed)
    xy = rng.uniform([-2, -6], [15, 6], (n, 2))
    pts = np.c_[xy, np.full(n, z0)]
    rot = rpy_to_matrix(roll, pitch, 0.0)
    return pts @ rot.T + rng.normal(0, noise, pts.shape)


def test_flat_ground():
    """평지: 평면 추정 성공, 기울기 0.5° 미만, 지면 점의 높이 오차 3 cm 미만."""
    g = fit_ground_plane(_plane_points(0, 0), nominal_ground_z=-0.1)
    assert g.fitted
    assert g.tilt_deg() < 0.5
    assert abs(g.height(np.array([[5.0, 0.0, -0.1]]))[0]) < 0.03


def test_tilted_ground_5deg():
    """요철로 차체가 5° 기울어도 지면을 제대로 찾아야 한다 (2D LiDAR 지면 오검출 문제의 해결)."""
    pts = _plane_points(np.radians(3), np.radians(-5))
    g = fit_ground_plane(pts, nominal_ground_z=-0.1)
    assert g.fitted
    # roll 3°, pitch 5° 를 함께 준 평면의 법선 기울기 = arccos(cos3°·cos5°) ≈ 5.8°
    assert abs(g.tilt_deg() - np.degrees(np.arccos(np.cos(np.radians(3)) * np.cos(np.radians(5))))) < 0.7
    h = g.height(pts)
    assert np.percentile(np.abs(h), 95) < 0.05


def test_fallback_when_no_ground():
    """지면 후보 점이 없음: fitted=False, nominal 평면(z = -0.2) 기준으로 높이를 계산."""
    pts = np.array([[1.0, 0.0, 1.0], [2.0, 0.0, 1.5]])
    g = fit_ground_plane(pts, nominal_ground_z=-0.2)
    assert not g.fitted
    assert abs(g.height(np.array([[0.0, 0.0, -0.2]]))[0]) < 1e-9
