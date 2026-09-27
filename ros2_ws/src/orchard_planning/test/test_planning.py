"""orchard_planning 순수 모듈 단위 시험: 격자, Hybrid A*, 열·통로 모델, 장애물 점, DWA (몇 초)."""
import math

import numpy as np
from orchard_perception.synthetic import SyntheticOrchard, scan

from orchard_planning.approach import detect_entrance, merge_close
from orchard_planning.dwa import DWAParams, DWAPlanner
from orchard_planning.grid import OccupancyGrid
from orchard_planning.hybrid_astar import hybrid_astar
from orchard_planning.obstacles import obstacle_points
from orchard_planning.orchard_model import OrchardModel


def test_grid_inflation_and_path_check():
    """점 둘레를 반경만큼 칠하고, 경로가 그 안을 지나면 막힘."""
    g = OccupancyGrid.around(np.array([[2.0, 0.0]]), [[0, 0], [4, 0]], 0.1, 1.0)
    g.add_discs(np.array([[2.0, 0.0]]), 0.5)
    assert g.occupied(np.array([[2.3, 0.0]]))[0] and not g.occupied(np.array([[2.7, 0.0]]))[0]
    assert not g.path_free(np.c_[np.linspace(0, 4, 41), np.zeros(41)])
    assert g.to_int8().shape == (g.nx * g.ny,)


def test_hybrid_astar_goes_around_wall_with_min_radius():
    """벽이 가로막으면 돌아가는 경로, 점 간격 ≤ 0.1 m, 곡률 ≤ 1/반경 (차가 따라갈 수 있음)."""
    wall = np.c_[np.full(30, 3.0), np.linspace(-3, 3, 30)]
    g = OccupancyGrid.around(wall, [[0, 0], [6, 0]], 0.1, 4.0)
    g.add_discs(wall, 0.5)
    p = hybrid_astar((0, 0, 0), (6, 0, 0), g, 1.0)
    assert p is not None and g.path_free(p)
    assert np.hypot(*(p[-1, :2] - [6, 0])) < 0.05
    ds = np.hypot(np.diff(p[:, 0]), np.diff(p[:, 1]))
    assert ds.max() < 0.11
    dyaw = np.abs(np.arctan2(np.sin(np.diff(p[:, 2])), np.cos(np.diff(p[:, 2]))))
    assert (dyaw[ds > 1e-6] / ds[ds > 1e-6]).max() < 1.0 / 1.0 + 0.05


def test_hybrid_astar_headland_uturn_is_dubins():
    """빈 헤드랜드: 통로 끝 → 옆 통로 입구 (간격 3.8, 반경 1.9) = 반원 (길이 π·1.9)."""
    g = OccupancyGrid.around(np.zeros((0, 2)), [[0, 0], [0, 3.8]], 0.1, 4.0)
    p = hybrid_astar((0, 0, 0), (0, 3.8, math.pi), g, 1.9)
    L = np.sum(np.hypot(np.diff(p[:, 0]), np.diff(p[:, 1])))
    assert abs(L - math.pi * 1.9) < 0.1


def test_orchard_model_lanes_from_trees():
    """가상 과수원 줄기 → 통로 0,1,2 중심 (0, 3.8, 7.6) 과 행 끝 u."""
    o = SyntheticOrchard(rows=4, row_length=12.0, seed=1, missing_prob=0.0)
    m = OrchardModel((0.0, 0.0), 0.0, 3.8, +1)
    m.add(o.trees)
    m.add(o.trees)                                   # 두 번 관측
    lanes = m.lanes(2)
    assert [round(ln.c, 1) for ln in lanes] == [0.0, 3.8, 7.6]
    assert abs(lanes[0].end_tree() - 12.0) < 0.1 and abs(lanes[1].end_tree() - 0.0) < 0.1
    assert lanes[1].direction == -1


def test_obstacle_points_ignore_grass_and_canopy():
    """잔디(≤15 cm)·수관(0.85 m 위)은 장애물이 아니고, 통로 가운데 사람 크기 물체는 장애물."""
    o = SyntheticOrchard(rows=2, row_length=15.0, seed=3)
    rng = np.random.default_rng(0)
    pts = scan(o, 3.0, 0.0, 0.0, rng=rng)
    ob = obstacle_points(pts)
    assert not (np.abs(ob[:, 1]) < 1.6).any(), ob[np.abs(ob[:, 1]) < 1.6]     # 통로 안에는 없음 (줄기는 ±1.9)
    blob = np.c_[4.0 + rng.uniform(-0.2, 0.2, 300), rng.uniform(-0.2, 0.2, 300), rng.uniform(0, 1.4, 300) - 0.1]
    ob = obstacle_points(np.vstack([pts, blob]))
    near = ob[np.hypot(ob[:, 0] - 4.0, ob[:, 1]) < 0.4]
    assert len(near) >= 4


def test_dwa_follows_path_and_blocks_on_wall():
    """직선 경로 → 곧게 전진. 앞이 막힌 벽이면 멈춤 (blocked)."""
    path = np.c_[np.linspace(0, 10, 101), np.zeros(101), np.zeros(101)]
    d = DWAPlanner(DWAParams())
    r = d.plan(0.0, 0.3, 0.0, 0.5, path, np.zeros((0, 2)))
    assert r.status == 'ok' and r.v > 0.4 and r.w < 0          # 경로가 오른쪽 → 오른쪽으로
    wall = np.c_[np.full(61, 1.2), np.linspace(-3, 3, 61)]
    r = DWAPlanner(DWAParams()).plan(0.0, 0.0, 0.0, 0.3, path, wall)
    assert r.status == 'blocked' and r.v == 0.0


def test_dwa_escapes_when_already_inside_safety():
    """나무 줄(y = −0.55)에 차체가 safety 안으로 붙어 있고 비스듬히 서 있어도 멈추지 않고 멀어지는 궤적으로 빠져나온다
    (막으려는 현상: 나무 옆에 붙어 'blocked' 로 계속 멈춤)."""
    dwa = DWAPlanner(DWAParams())
    path = np.c_[np.linspace(0, 10, 101), np.zeros(101), np.zeros(101)]
    wall = np.c_[np.arange(-2.0, 12.0, 0.1), np.full(140, -0.55)]
    res = dwa.plan(1.0, 0.0, math.radians(-20), 0.0, path, wall)
    assert res.status == 'escape' and res.v > 0 and res.w > 0, (res.status, res.v, res.w)
    # 안쪽에 있지 않으면 평소대로 (ok) — 탈출 모드는 교착일 때만
    res2 = dwa.plan(1.0, 0.8, 0.0, 0.0, path, wall)
    assert res2.status == 'ok'


def test_detect_entrance_right_and_left_end_lane():
    """3열(y = −1.9, 1.9, 5.7) 을 x = 0 부터 본 줄기 (스캔 3번 겹쳐 같은 줄기가 여러 점) → 오른쪽 끝 통로 입구 (0, 0),
    왼쪽 끝 통로 입구 (0, 3.8), 정렬점은 입구 3 m 앞, 열 방향 0 (과수원 안쪽)."""
    rng = np.random.default_rng(0)
    trees = np.array([[x, y] for y in (-1.9, 1.9, 5.7) for x in np.arange(0.0, 7.3, 1.2)])
    P = np.vstack([trees + rng.normal(0, 0.03, trees.shape) for _ in range(3)])
    assert len(merge_close(P)) == len(trees)
    r = detect_entrance(P, (-9.0, -3.5), 3.8, 'right')
    assert r is not None and r.n_rows == 3 and abs(r.heading) < 0.05
    assert np.hypot(*(r.entry - [0.0, 0.0])) < 0.15 and np.hypot(*(r.pre - [-3.0, 0.0])) < 0.2
    left = detect_entrance(P, (-9.0, -3.5), 3.8, 'left')
    assert abs(left.entry[1] - 3.8) < 0.15
    # 과수원 반대편(열 끝 너머)에서 보면 열 방향이 뒤집혀 안쪽(−x)을 향한다
    back = detect_entrance(P, (16.0, 0.0), 3.8, 'right')
    assert abs(abs(back.heading) - math.pi) < 0.05 and back.entry[0] > 7.0


def test_dwa_uses_shorter_horizon_before_blocking():
    """좁은 통로(좌우 벽 ±1.1 m) 3.8 m 앞을 막은 것이 있어 4.5 m 후보가 모두 걸려도, 앞 1.5+0.5 m 가 비었으면
    짧은 후보로 천천히 다가간다 ('short'). 그 자리에 멈추면 옆으로 비킬 기회도 없다."""
    dwa = DWAPlanner(DWAParams())
    path = np.c_[np.linspace(0, 20, 201), np.zeros(201), np.zeros(201)]
    xs = np.arange(-2.0, 20.0, 0.2)
    walls = np.vstack([np.c_[xs, np.full(len(xs), 1.1)], np.c_[xs, np.full(len(xs), -1.1)]])
    cross = np.c_[np.full(23, 3.8), np.linspace(-1.1, 1.1, 23)]
    res = dwa.plan(0.0, 0.0, 0.0, 0.0, path, np.vstack([walls, cross]))
    assert res.status == 'short' and res.v > 0, res.status
