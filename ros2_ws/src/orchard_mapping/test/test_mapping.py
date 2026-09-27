"""orchard_mapping 순수 모듈 시험: 출발점 좌표, 복셀 맵, 줄기 찾기, 매칭, 가상 과수원 전체 흐름."""
import math

import numpy as np

from orchard_mapping.start_frame import StartFrame
from orchard_mapping.trunk_locator import TrunkLocator, bbox_bearing
from orchard_mapping.trunk_registry import TrunkRegistry
from orchard_mapping.trunk_table import build, write_tables
from orchard_mapping.voxel_map import VoxelMap, height_colors, pack, unpack


def test_pack_roundtrip():
    """음수 포함 칸 번호를 64 비트 키로 묶었다 풀면 그대로."""
    idx = np.array([[0, 0, 0], [-5, 7, -1], [1000, -2000, 30]])
    assert (unpack(pack(idx)) == idx).all()


def test_start_frame_uses_driven_direction():
    """원점 = 출발 위치, +x = 처음 1.5 m 달린 방향 (나침반 요와 무관)."""
    sf = StartFrame(course_distance=1.5)
    sf.begin(10.0, 5.0, 0.2)
    assert not sf.observe(10.5, 5.5)
    heading = math.radians(30)
    assert sf.observe(10 + 2 * math.cos(heading), 5 + 2 * math.sin(heading))
    p = sf.to_start(np.array([[10 + 4 * math.cos(heading), 5 + 4 * math.sin(heading), 1.2]]))
    assert np.allclose(p, [[4.0, 0.0, 1.0]], atol=1e-9)


def test_voxel_counts_scans_not_points():
    """한 스캔 안의 같은 칸 점 여러 개는 1 번만, 스캔마다 1 씩 증가."""
    vm = VoxelMap(0.1)
    pts = np.array([[0.01, 0.01, 0.01], [0.02, 0.03, 0.04], [1.05, 0, 0]])
    vm.integrate(pts)
    vm.integrate(pts)
    c, h = vm.occupied()
    assert len(c) == 2 and set(h.tolist()) == {2}


def test_height_above_ground_on_slope():
    """경사진 지면에서도 지면 위 높이(HAG) 는 기둥 높이로 나온다."""
    vm = VoxelMap(0.1, ground_cell=0.5)
    xs, ys = np.meshgrid(np.arange(0, 4, 0.1), np.arange(0, 2, 0.1))
    ground = np.c_[xs.ravel(), ys.ravel(), 0.2 * xs.ravel()]            # 경사 0.2
    pole = np.c_[np.full(8, 2.02), np.full(8, 1.02), 0.4 + 0.1 * np.arange(8)]   # x=2 지면 0.4 위 기둥
    vm.integrate(np.vstack([ground, pole]))
    c, _ = vm.occupied()
    hag = vm.height_above_ground(c)
    top = np.argmax(np.where(np.hypot(c[:, 0] - 2.05, c[:, 1] - 1.05) < 0.06, c[:, 2], -1))
    assert abs(hag[top] - 0.7) < 0.15, hag[top]
    assert height_colors(np.array([0.0, 3.0])).shape == (2, 3)


def _cam_rotation(yaw):
    """카메라 광학 프레임(z 전방, x 오른쪽, y 아래) → start 회전 (카메라가 yaw 방향을 봄)."""
    c, s = math.cos(yaw), math.sin(yaw)
    fwd, left = np.array([c, s, 0.0]), np.array([-s, c, 0.0])
    return np.c_[-left, [0, 0, -1.0], fwd]           # 열 = 광학 x(오른쪽), y(아래), z(전방) 의 start 방향


def test_bbox_bearing_center_and_edge():
    """영상 가운데 박스는 카메라가 보는 방향, 오른쪽 끝(u=640)은 hfov 90° 의 절반만큼 오른쪽."""
    k = np.array([[320.0, 0, 320], [0, 320.0, 240], [0, 0, 1]])
    R = _cam_rotation(math.radians(20))
    b, half = bbox_bearing(320, 240, 64, k, R)
    assert abs(b - math.radians(20)) < 1e-6 and abs(half - math.atan(32 / 320)) < 1e-6
    b2, _ = bbox_bearing(640, 240, 10, k, R)
    assert abs(b2 - math.radians(20 - 45)) < 1e-6


def test_locator_picks_nearest_trunk_not_grass_or_canopy():
    """쐐기 안에 잔디(낮음)·줄기 두 개·수관(높음)이 있으면 가까운 줄기를 고른다."""
    vm = VoxelMap(0.1)
    g = np.c_[np.random.default_rng(0).uniform(0, 10, (4000, 2)), np.zeros(4000)]
    grass = np.c_[np.linspace(1.5, 2.5, 20), np.zeros(20), np.full(20, 0.1)]
    t1 = np.c_[np.full(9, 4.0), np.full(9, 0.1), np.linspace(0.1, 0.9, 9)]
    t2 = np.c_[np.full(9, 7.0), np.full(9, -0.1), np.linspace(0.1, 0.9, 9)]
    rng = np.random.default_rng(1)
    canopy = np.c_[rng.uniform(3.5, 4.5, (200, 2)), rng.uniform(1.2, 2.5, 200)]
    vm.integrate(np.vstack([g, grass, t1, t2, canopy]))
    c, _ = vm.occupied()
    loc = TrunkLocator()
    loc.set_voxels(c, vm.height_above_ground(c))
    hit = loc.locate((0.0, 0.0), 0.0, math.radians(3))
    assert hit is not None and abs(hit.x - 4.05) < 0.1 and abs(hit.y - 0.15) < 0.1, hit
    assert loc.locate((0.0, 0.0), math.pi / 2, math.radians(3)) is None     # 옆에는 아무것도 없음


def test_registry_matches_and_confirms():
    """0.4 m 안 관측은 같은 줄기로 합치고, 3 번 이상이면 확정."""
    reg = TrunkRegistry(0.4, 3)
    for dx in (0.0, 0.05, -0.05):
        reg.update(2.0 + dx, 1.0, 0.0, 0.8)
    reg.update(3.2, 1.0, 0.0, 0.9)
    assert len(reg.trunks) == 2 and len(reg.confirmed()) == 1
    assert abs(reg.confirmed()[0].x - 2.0) < 0.03


def test_pipeline_on_synthetic_orchard():
    """가상 과수원: 출발점이 (5, 0) 에서 x 방향. LiDAR 로 복셀 맵을 만들고, 정답 줄기로 만든 카메라 박스로
    위치를 찾아 매칭하면 출발점 기준 상대좌표가 정답과 약 10 cm 안에서 맞는다."""
    from orchard_perception.synthetic import SyntheticOrchard, scan
    orchard = SyntheticOrchard(rows=2, row_length=14.0, missing_prob=0.0, seed=3)
    rng = np.random.default_rng(0)
    sf = StartFrame()
    sf.set(5.0, 0.0, 0.0, 0.0)
    vm = VoxelMap(0.1)
    loc = TrunkLocator()
    reg = TrunkRegistry(0.4, 2)
    k = np.array([[320.0, 0, 320], [0, 320.0, 240], [0, 0, 1]])
    for step in range(12):
        x = 5.0 + 0.5 * step
        pts = scan(orchard, x, 0.0, 0.0, rng=rng) + [x, 0.0, 0.1]      # base_link → 월드 (요 0)
        vm.integrate(sf.to_start(pts))
        c, _ = vm.occupied()
        loc.set_voxels(c, vm.height_above_ground(c))
        cam = np.array([x + 0.45 - 5.0, 0.0])                          # 카메라 위치 (start)
        R = _cam_rotation(0.0)
        for tx, ty in orchard.trees:                                   # 정답 줄기 → 가상 YOLO 박스
            rel = np.array([tx - 5.0, ty]) - cam
            if not (1.0 < rel[0] < 8.0) or abs(math.atan2(rel[1], rel[0])) > math.radians(40):
                continue
            u = 320 - 320 * rel[1] / rel[0]
            w = 320 * 0.14 / rel[0]
            b, half = bbox_bearing(u, 300, w, k, R)
            hit = loc.locate(cam, b, half)
            if hit:
                reg.update(hit.x, hit.y, hit.z, 0.9)
    got = np.array([[t.x, t.y] for t in reg.confirmed()])
    truth = orchard.trees - [5.0, 0.0]
    assert len(got) >= 8
    d = np.min(np.hypot(got[:, None, 0] - truth[None, :, 0], got[:, None, 1] - truth[None, :, 1]), axis=1)
    assert np.median(d) < 0.08 and d.max() < 0.15, d        # 복셀 0.1 m 양자화 + 센서 쪽 표면만 보임


def test_registry_drops_off_row_trunks():
    """열(y = −1.9, 1.9) 위 줄기는 남기고, 통로 가운데(y = 0.1) 오검출은 뺀다."""
    reg = TrunkRegistry(0.4, 1)
    for x in np.arange(0, 10, 1.2):
        reg.update(x, -1.9, 0, 0.9)
        reg.update(x, 1.9, 0, 0.9)
    reg.update(4.0, 0.1, 0, 0.5)
    keep, rej = reg.on_rows(3.8, 0.4)
    assert len(rej) == 1 and abs(rej[0].y - 0.1) < 1e-9 and len(keep) == 18


def test_trunk_table_groups_rows_orders_and_flags_gap(tmp_path):
    """두 열(y = ±1.9), 주간 1.2 m, 오른쪽 열 x = 3.6 결주 → 열 번호(y 작은 쪽부터)·순번(x 순)·결주 의심 1 · 파일 3종."""
    trunks, i = [], 1
    for y in (1.9, -1.9):
        for x in np.arange(0.0, 9.7, 1.2):
            if y < 0 and abs(x - 3.6) < 1e-6:
                continue
            trunks.append(dict(id=i, x=x + 0.02 * (i % 3), y=y, z=0.0, hits=5, mean_conf=0.8, source='yolo'))
            i += 1
    truth = np.array([[x, y] for y in (1.9, -1.9) for x in np.arange(0.0, 9.7, 1.2)])
    rows = build(trunks, 3.8, truth)
    assert [r.row for r in rows] == [1, 2] and rows[0].y < 0 < rows[1].y
    assert rows[0].n == 8 and rows[1].n == 9 and rows[0].missing == 1 and rows[1].missing == 0
    assert [it.order for it in rows[1].items] == list(range(1, 10))
    assert abs(rows[1].gap_med - 1.2) < 0.05
    assert max(it.err for r in rows for it in r.items) < 0.05
    write_tables(str(tmp_path), trunks, 3.8, truth)
    for f in ('trunks_by_row.csv', 'trunks_table.html', 'trunks_table.md'):
        assert (tmp_path / f).stat().st_size > 200
    assert '결주 의심' in (tmp_path / 'trunks_table.md').read_text(encoding='utf-8')
    # 과수원 밖에 두 개만 잡힌 '열' 은 오검출 의심으로 표시
    extra = trunks + [dict(id=90, x=5.0, y=-6.0, z=0.0, hits=3, mean_conf=0.5, source='yolo'),
                      dict(id=91, x=5.4, y=-6.0, z=0.0, hits=3, mean_conf=0.5, source='yolo')]
    rows = build(extra, 3.8)
    assert rows[0].n == 2 and '오검출' in rows[0].note and rows[1].note == ''
    # 나무 사이 기둥(간격 0.6 m)을 줄기로 잡으면 '기둥·중복 의심'
    post = trunks + [dict(id=95, x=6.6, y=1.9, z=0.0, hits=3, mean_conf=0.5, source='yolo')]
    rows = build(post, 3.8)
    assert rows[1].short == 1 and any('기둥' in it.note for it in rows[1].items) and rows[0].short == 0


def test_start_frame_axis_from_planner_heading():
    """원점을 정한 뒤 전역 계획의 통로 방향(0.1 rad)을 받으면 그 방향이 +x — 비스듬히 달린 이동 방향과 무관."""
    f = StartFrame(1.5)
    f.begin(5.0, 3.0, 0.0)
    assert not f.observe(5.5, 3.0)
    f.set_axis(0.1)
    assert f.ready and abs(f.yaw0 - 0.1) < 1e-9
    q = f.to_start(np.array([[5.0 + 2 * math.cos(0.1), 3.0 + 2 * math.sin(0.1), 0.0]]))[0]
    assert abs(q[0] - 2.0) < 1e-9 and abs(q[1]) < 1e-9
