"""treemap.py (나무 지도 누적·열 정리·저장·보정 추정) 단위 시험.

열/나무 번호·결주, 가끔 잡히는 점 무시, CSV/JSON 저장, 중복·열 사이 잡음, 방위 오차 추정,
odom 시간 지연 추정, 지주(기둥) 분리를 확인한다. 거리 [m], 각도 [rad], 시각 [s].
"""
import math

import numpy as np

from orchard_navigation.treemap import TreeMap, enu_to_latlon


def _orchard(rows=4, n=20, spacing=1.2, row_spacing=3.8, heading=0.3, missing=((1, 5), (2, 11), (2, 12))):
    """참 나무 위치 (N×2) 생성: rows 열 × n 그루, 행 방향 heading [rad], missing = 빠진 (열, 번호) 목록."""
    u_dir = np.array([math.cos(heading), math.sin(heading)])
    v_dir = np.array([-u_dir[1], u_dir[0]])
    trees = []
    for r in range(rows):
        for k in range(n):
            if (r, k) in missing:
                continue
            trees.append(k * spacing * u_dir + r * row_spacing * v_dir + [5.0, -2.0])
    return np.array(trees)


def test_organize_rows_and_missing_trees():
    """4열, 결주 3그루, 잡음 4 cm 관측 8회 → 열 4개, 나무 수 [20,19,18,20], 결주 위치·행 방향·주간 1.2 m 복원."""
    truth = _orchard()
    rng = np.random.default_rng(0)
    m = TreeMap()
    for t in range(8):                                   # 같은 나무를 여러 번, 잡음과 함께 관측
        seen = truth[rng.random(len(truth)) < 0.85]
        m.update(seen + rng.normal(0, 0.04, seen.shape), now=t)
    s = m.organize(row_spacing=3.8)
    assert len(s['rows']) == 4
    assert [r['num_trees'] for r in s['rows']] == [20, 19, 18, 20]
    assert abs(((s['heading'] - 0.3) + math.pi / 2) % math.pi - math.pi / 2) < 0.03
    miss = {r['row']: len(r['missing']) for r in s['rows']}
    assert miss == {1: 0, 2: 1, 3: 2, 4: 0}
    assert all(abs(r['spacing'] - 1.2) < 0.05 for r in s['rows'])
    # 위치 오차 (평균 효과로 잡음보다 작아야)
    P = np.array([[t['x'], t['y']] for t in s['trees']])
    d = np.min(np.hypot(P[:, None, 0] - truth[None, :, 0], P[:, None, 1] - truth[None, :, 1]), axis=1)
    assert d.max() < 0.08


def test_spurious_single_hits_are_ignored():
    """min_hits=3: 3번 관측된 점만 확정, 한 번 잡힌 점은 버린다."""
    m = TreeMap(min_hits=3)
    m.update(np.array([[0.0, 0.0], [10.0, 10.0]]))
    m.update(np.array([[0.02, 0.0]]))
    m.update(np.array([[0.0, 0.03]]))
    assert len(m.confirmed()) == 1


def test_save_csv_json(tmp_path):
    """CSV 머리줄 + 나무 10줄 저장, enu_to_latlon 이 북쪽 100 m ≈ 위도 100/111320° 로 변환."""
    m = TreeMap(min_hits=1)
    m.update(_orchard(rows=2, n=5, missing=()))
    s = m.organize(3.8)
    m.save(s, str(tmp_path / 't.csv'), str(tmp_path / 's.json'), geo=(35.5727, 129.1904))
    lines = (tmp_path / 't.csv').read_text().splitlines()
    assert lines[0].startswith('row,tree,x,y') and len(lines) == 11
    lat, lon = enu_to_latlon(100.0, 100.0, 35.5727, 129.1904)
    assert abs(lat - 35.5727 - 100 / 111320) < 2e-5 and lon > 129.1904


def test_duplicates_and_between_row_clutter():
    """가지·자세 오차로 한 나무가 둘로 잡히고, 열 사이에 잡초 같은 이상점이 있어도 열 3개·나무 10그루."""
    truth = _orchard(rows=3, n=10, heading=0.25, missing=())
    rng = np.random.default_rng(1)
    m = TreeMap()
    dup = truth + rng.normal(0, 0.12, truth.shape) + 0.25 * np.array([math.cos(0.25), math.sin(0.25)])
    clutter = truth[::3] + 1.4 * np.array([-math.sin(0.25), math.cos(0.25)])
    for t in range(10):
        m.update(truth + rng.normal(0, 0.03, truth.shape), now=t)
        if t % 2:
            m.update(dup[rng.random(len(dup)) < 0.6], now=t)
        if t < 4:
            m.update(clutter + rng.normal(0, 0.03, clutter.shape), now=t)
    s = m.organize(row_spacing=3.8)
    assert [r['num_trees'] for r in s['rows']] == [10, 10, 10]
    assert all(abs(r['spacing'] - 1.2) < 0.1 for r in s['rows'])
    assert all(not r['missing'] for r in s['rows'])


def test_yaw_bias_estimator():
    """나침반 15° 오차로 곧게 전진 → 표본 5개 이상, 추정 오차 < 0.01 rad, 도는 중에는 표본을 만들지 않음."""
    from orchard_navigation.treemap import YawBiasEstimator
    est = YawBiasEstimator()
    bias = math.radians(15)                               # 나침반이 15° 틀어짐: 실제 진행 0.1 rad, 요는 0.1-bias
    for k in range(80):
        s = 0.1 * k
        assert est.bias == 0.0 or abs(est.bias - bias) < 0.02
        est.update(s * math.cos(0.1), s * math.sin(0.1), 0.1 - bias + 0.005 * math.sin(k), 0.0, t=0.2 * k)
    assert len(est.samples) >= 5 and abs(est.bias - bias) < 0.01
    assert est.valid_since == 0.0
    n = len(est.samples)
    est.update(9.0, 9.0, 1.5, 0.5)                        # 도는 중: 표본 없음
    assert len(est.samples) == n


def test_time_offset_recovers_odom_latency():
    """odom 이 실제보다 0.8 s 앞선 값을 낼 때: 달릴 때/설 때 같은 나무가 어긋나 두 번 찍힘 → 보정 추정으로 한 번."""
    from orchard_navigation.treemap import PoseHistory, best_time_offset, project
    lead = 0.8
    trees = np.array([[x, y] for x in np.arange(0, 20, 1.2) for y in (-1.9, 1.9)])

    def true_x(t):                                     # 0.8 m/s, 5~9 s 는 정지, 다시 0.8 m/s
        """시각 t [s] 의 실제 로버 x [m]."""
        return 0.8 * min(t, 5) + 0.8 * max(0.0, t - 9)
    hist = PoseHistory()
    for k in range(0, 1600):
        t = 0.01 * k
        hist.add(t, true_x(t + lead), 0.0, 0.0)          # odom 은 lead 초 뒤의 실제 위치를 낸다
    recs = []
    rng = np.random.default_rng(3)
    for k in range(10, 150):
        t = 0.1 * k
        rel = trees - [true_x(t), 0.0]
        rel = rel[np.hypot(rel[:, 0], rel[:, 1]) < 7]
        recs.append((t, rel + rng.normal(0, 0.02, rel.shape), np.full(len(rel), 0.08)))
    off, sc, sc0 = best_time_offset(recs, hist)
    assert abs(off + lead) < 0.1 and sc > 1.2 * sc0

    def rows_after(offset):
        """시각 보정 offset [s] 로 지도를 만들어 organize 요약 반환."""
        m = TreeMap()
        for t, w, r in project(recs, hist, offset):
            m.update(w, r, t)
        return m.organize(row_spacing=3.8)
    bad, good = rows_after(0.0), rows_after(off)
    assert max(abs(r['spacing'] - 1.2) for r in good['rows']) < 0.1
    assert min(r['spacing'] for r in bad['rows']) < 1.0     # 보정 전에는 중복으로 간격이 좁게 나온다


def test_trellis_posts_are_not_counted_as_trees():
    """나무 사이 한가운데 콘크리트 기둥, 열 끝 기둥은 'posts' 로 따로.
    기대: 나무 6그루, 주간 1.2 m, 결주 없음, 기둥 2개 (x = 3.0, 6.6)."""
    xs = [0.0, 1.2, 2.4, 3.6, 4.8, 6.0]
    posts = [3.0, 6.6]
    m = TreeMap()
    for t in range(5):
        m.update(np.array([[x, 0.0] for x in xs + posts]), now=t)
    s = m.organize(row_spacing=3.8, heading=0.0)
    r = s['rows'][0]
    assert r['num_trees'] == 6 and abs(r['spacing'] - 1.2) < 0.01 and not r['missing']
    assert sorted(p['x'] for p in r['posts']) == posts
