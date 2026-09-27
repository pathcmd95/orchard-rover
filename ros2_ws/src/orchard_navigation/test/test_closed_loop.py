"""폐루프 주행 시험: 가상 LiDAR → 줄기/행 인식 → 미션 상태기계 → 운동학 모델.

Gazebo/ROS 없이 알고리즘 전체가 3개 통로를 지그재그로 순회하는지,
행 안에서 통로 중심선 횡오차가 작은지 확인한다.
또 장애물 정지, 탐사 모드(끝 열 판정)와 주행 중 나무 지도, 계획 U턴(LiDAR 입구 + Dubins)이
열 간격 오차·미끄러짐·odom 흐름에서도 입구를 맞추는지 본다. (orchard_perception 모듈 필요, 약 2분)

단위: 거리 [m], 시간 [s] (시뮬 시각, dt = 0.1 s), 각도 [rad]. 좌표는 과수원 월드 = 운동학 모델의 참값.
"""
import math

import numpy as np

from orchard_navigation.controller import FollowParams
from orchard_navigation.mission import DONE, MissionParams, OrchardMission, Pose2D, RowObs
from orchard_navigation.treemap import TreeMap
from orchard_perception.ground import fit_ground_plane
from orchard_perception.rows import RowTracker, corridor_obstacle_distance, fit_rows
from orchard_perception.synthetic import SyntheticOrchard, scan
from orchard_perception.trunks import detect_trunks


def perceive(pts, tracker, t):
    """가상 LiDAR 점군 → (RowObs, 줄기 xy[base_link]). lidar_tree_node 와 같은 인식 순서를 흉내낸다."""
    g = fit_ground_plane(pts, nominal_ground_z=-0.1)
    h = g.height(pts)
    trunks = detect_trunks(pts, h)
    xy = np.array([[k.x, k.y] for k in trunks]) if trunks else np.zeros((0, 2))
    w = np.array([k.confidence for k in trunks]) if trunks else np.zeros(0)
    raw = fit_rows(xy, w, 3.8, *tracker.prior())
    est = tracker.update(raw, t)
    off, head = (est.offset, est.heading) if est.valid else (0.0, 0.0)
    obs = corridor_obstacle_distance(pts, h, off, head, 0.5)
    return RowObs(est.valid, est.offset, est.heading, est.confidence, raw.last_tree_ahead, obs,
                  est.left_count, est.right_count), xy


def run(orchard, lanes=3, max_time=400.0, obstacle=None, seed=0, treemap=None, start=(-3.0, 0.0, 0.0),
        turn_mode='planned', row_spacing=None, turn_slip=1.0, entry_log=None, odom_drift=(0.0, 0.0)):
    """미션을 폐루프로 max_time [s] 까지 (또는 DONE/STOPPED 까지) 돌린다.

    obstacle: 통로 안 장애물 월드 위치 (x, y), treemap: 주면 줄기를 누적, start: 출발 자세 (x, y, yaw),
    row_spacing: 미션 파라미터 열 간격 (None = 실제 값), turn_slip: U턴 중 실제 회전/명령 회전 비,
    entry_log: U턴 뒤 입구 횡오차를 모을 리스트, odom_drift: U턴 중 odom 위치가 흐르는 최대량 (dx, dy) [m].
    반환: (mission, 행 추종 중 횡오차 배열, 상태 기록, 최종 참 자세, 걸린 시각).
    """
    rng = np.random.default_rng(seed)
    mission = OrchardMission(MissionParams(lanes=lanes, row_spacing=row_spacing or orchard.row_spacing,
                                           turn_mode=turn_mode, follow=FollowParams()))
    tracker = RowTracker()
    pose = Pose2D(*start)
    dt, t = 0.1, 0.0
    lateral, states = [], []
    mission.start(t, pose)
    drift = np.zeros(2)
    while t < max_time and mission.state not in (DONE, 'STOPPED'):
        roll, pitch = rng.normal(0, math.radians(1.5), 2)     # 요철에 의한 흔들림
        pts = scan(orchard, pose.x, pose.y, pose.yaw, roll=roll, pitch=pitch, rng=rng)
        if obstacle is not None:
            ox, oy = obstacle
            c, s = math.cos(pose.yaw), math.sin(pose.yaw)
            dx, dy = ox - pose.x, oy - pose.y
            bx, by = c * dx + s * dy, -s * dx + c * dy
            blob = np.c_[bx + rng.uniform(-0.2, 0.2, 300), by + rng.uniform(-0.2, 0.2, 300),
                         rng.uniform(0.0, 1.4, 300) - 0.1]
            pts = np.vstack([pts, blob])
        row, xy = perceive(pts, tracker, t)
        if treemap is not None and len(xy):
            c, s_ = math.cos(pose.yaw), math.sin(pose.yaw)
            world = np.c_[pose.x + c * xy[:, 0] - s_ * xy[:, 1], pose.y + s_ * xy[:, 0] + c * xy[:, 1]]
            treemap.update(world[np.hypot(xy[:, 0], xy[:, 1]) < 8.0], now=t)
        if mission.state == 'TURN':                     # U턴 중 odom 위치가 서서히 흐름 (Gazebo PX4 EKF 에서 관찰)
            drift += 0.1 * np.array(odom_drift) * dt / 1.0      # 초당 최대량의 10 % 씩 흐름
            n = np.hypot(*odom_drift)
            if n > 0 and np.hypot(*drift) > n:
                drift *= n / np.hypot(*drift)
        odom = Pose2D(pose.x + drift[0], pose.y + drift[1], pose.yaw)
        mission.add_trees(t, odom, xy)
        v, w = mission.step(t, odom, row)
        if mission.state == 'TURN':
            w *= turn_slip                              # 잔디 위 미끄러짐: 실제 회전이 명령보다 작음
        if entry_log is not None and mission.lanes_done >= 0 and 'TURN' in states:
            # 입구 오차: U턴 뒤 행 안(끝에서 0.3~3 m) 에 들어와 있는 동안 가장 가까운 통로 중심과의 거리
            if orchard.row_length - 3.0 < pose.x < orchard.row_length - 0.3 and abs(math.cos(pose.yaw)) > 0.7:
                entry_log.append(min((pose.y - orchard.lane_y(k) for k in range(orchard.rows - 1)), key=abs))
        pose = Pose2D(pose.x + v * math.cos(pose.yaw) * dt, pose.y + v * math.sin(pose.yaw) * dt,
                      math.atan2(math.sin(pose.yaw + w * dt), math.cos(pose.yaw + w * dt)))
        t += dt
        states.append(mission.state)
        if mission.state == 'FOLLOW_ROW' and 1.0 < pose.x < orchard.row_length - 1.0:
            lateral.append(min((pose.y - orchard.lane_y(k) for k in range(orchard.rows - 1)), key=abs))
    return mission, np.array(lateral), states, pose, t


def test_traverses_three_lanes_zigzag():
    """4열(통로 3개) 과수원, lanes=3 → 지그재그로 3개 통로 주행 후 DONE, 횡오차 최대 < 0.30 m, RMS < 0.12 m."""
    orchard = SyntheticOrchard(rows=4, row_length=15.0, seed=2)
    mission, lateral, states, pose, t = run(orchard, lanes=3)
    assert mission.state == DONE, (mission.state, mission.reason, t)
    assert mission.lanes_done == 3
    assert 'TURN' in states and 'ENTER_ROW' in states
    assert abs(lateral).max() < 0.30, abs(lateral).max()
    assert math.sqrt(np.mean(lateral ** 2)) < 0.12
    # 3번째 통로(중심 y = 7.6)에서 끝나야 한다
    assert abs(pose.y - orchard.lane_y(2)) < 0.6


def test_stops_for_obstacle_in_lane():
    """통로 중심 x=7 m 에 장애물 → 장애물 1 m 이상 앞에서 멈추고 FOLLOW_ROW 에 머문다."""
    orchard = SyntheticOrchard(rows=2, row_length=15.0, seed=3)
    mission, _, states, pose, _ = run(orchard, lanes=1, max_time=40.0, obstacle=(7.0, 0.0))
    assert pose.x < 7.0 - 1.0, pose.x          # 장애물 1.2 m 앞에서 멈춤
    assert mission.state == 'FOLLOW_ROW'


def test_explore_mode_finds_all_lanes_and_stops_at_edge():
    """통로 수를 모르는 탐사 모드: 5열(통로 4개) 과수원을 모두 돌고 끝 열 바깥에서 멈춘다."""
    orchard = SyntheticOrchard(rows=5, row_length=12.0, seed=4)
    tm = TreeMap()
    mission, lateral, states, pose, t = run(orchard, lanes=0, max_time=600.0, treemap=tm)
    assert mission.state == DONE, (mission.state, mission.reason, t)
    assert '끝 열' in mission.reason
    assert mission.lanes_done == 4, mission.lanes_done
    assert abs(lateral).max() < 0.30
    # 주행하며 만든 나무 지도: 5열, 나무 수·위치가 실제와 맞아야 한다
    s = tm.organize(orchard.row_spacing)
    assert len(s['rows']) == 5, [r['num_trees'] for r in s['rows']]
    P = np.array([[q['x'], q['y']] for q in s['trees']])
    d = np.min(np.hypot(P[:, None, 0] - orchard.trees[None, :, 0], P[:, None, 1] - orchard.trees[None, :, 1]), 1)
    assert np.median(d) < 0.05 and d.max() < 0.25
    assert len(P) >= 0.95 * len(orchard.trees), (len(P), len(orchard.trees))
    assert len(P) <= len(orchard.trees) + 2


def test_edge_scenario_short():
    """구간 시험 'edge' 와 같은 조건: 3열(통로 2개) 과수원의 마지막 통로 중간에서 출발 → 끝 열 판정 후 정지."""
    orchard = SyntheticOrchard(rows=3, row_length=12.0, seed=5)
    mission, _, states, pose, t = run(orchard, lanes=0, max_time=120.0, start=(6.0, orchard.lane_y(1), 0.0))
    assert mission.state == DONE and '끝 열' in mission.reason, (mission.state, mission.reason, t)
    assert mission.lanes_done == 1 and 'TURN' in states
    assert t < 60.0, t                      # 시뮬 1분 안에 끝나는 짧은 시험


def _entry_error(**kw):
    """2개 통로 주행을 돌리고 (mission, U턴 뒤 입구 횡오차 최대값 [m]) 반환 (기록이 없으면 inf)."""
    orchard = kw.pop('orchard')
    log = []
    mission, lateral, states, pose, t = run(orchard, lanes=2, max_time=200.0, entry_log=log, **kw)
    return mission, (max(abs(e) for e in log) if log else float('inf'))


def test_planned_uturn_beats_arc_with_spacing_error_and_slip():
    """uturn 구간과 같은 조건 (3열·12 m). 열 간격이 파라미터와 다르고(3.4 vs 3.8 m) 잔디에서 회전이 20 % 덜 될 때:
    반원 방식은 입구를 크게 빗나가고, LiDAR 로 입구를 잡는 계획 U턴은 통로 중심으로 들어간다."""
    orchard = SyntheticOrchard(rows=3, row_length=12.0, row_spacing=3.4, seed=6)
    common = dict(orchard=orchard, row_spacing=3.8, turn_slip=0.8, start=(5.0, 0.0, 0.0))
    m_plan, e_plan = _entry_error(turn_mode='planned', **common)
    m_arc, e_arc = _entry_error(turn_mode='arc', **common)
    assert m_plan.state == DONE and m_plan.lanes_done == 2, (m_plan.state, m_plan.reason)
    assert 'lidar2' in str(m_plan.plan['next_lane'].source)
    assert abs(m_plan.plan['next_lane'].spacing - 3.4) < 0.15
    assert e_plan < 0.3, e_plan
    assert e_arc > 1.5 * e_plan, (e_arc, e_plan)


def test_planned_uturn_nominal():
    """열 간격이 파라미터와 같고 미끄러짐 없음 → 계획 U턴으로 2개 통로 완주, 입구 오차 < 0.3 m."""
    orchard = SyntheticOrchard(rows=3, row_length=12.0, seed=7)
    m, e = _entry_error(orchard=orchard, start=(5.0, 0.0, 0.0))
    assert m.state == DONE and m.lanes_done == 2, (m.state, m.reason)
    assert e < 0.3, e


def test_planned_uturn_hands_over_to_lidar_when_odom_drifts():
    """U턴 중 odom 위치가 옆으로 0.7 m 흘러도 (Gazebo PX4 에서 입구 0.6~1 m 어긋남 관찰) LiDAR 인수로 통로 중심에 들어간다."""
    orchard = SyntheticOrchard(rows=3, row_length=12.0, seed=8)
    m, e = _entry_error(orchard=orchard, start=(5.0, 0.0, 0.0), odom_drift=(0.0, 0.7))
    assert m.state == DONE and m.lanes_done == 2, (m.state, m.reason)
    assert e < 0.35, e
