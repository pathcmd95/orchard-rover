"""전역(통로 순회 + Hybrid A*) · 지역(DWA) 계획 폐루프 시험 — Gazebo/ROS 없이.

가상 LiDAR(orchard_perception.synthetic) → 인식(지면·줄기·통로 중심선) → 전역 계획 → 장애물 점 → DWA → 운동학 모델.
통로를 모두 도는지, 통로 안 횡오차, U턴 뒤 입구, 통로 장애물 회피, U턴 중 odom 흐름 보정을 본다.
단위: [m], [s], [rad]. 좌표는 과수원 월드 = 운동학 모델의 참값. (약 1~2분)
"""
import math

import numpy as np
from orchard_perception.ground import fit_ground_plane
from orchard_perception.rows import RowTracker, fit_rows
from orchard_perception.synthetic import SyntheticOrchard, scan
from orchard_perception.trunks import detect_trunks

from orchard_planning.dwa import DWAParams, DWAPlanner
from orchard_planning.global_planner import DONE, TURN, OrchardPlanner, PlannerParams, RowSeen
from orchard_planning.obstacles import obstacle_points


def perceive(pts, tracker, t):
    """가상 점군 → (RowSeen, 줄기 xy[base_link]). lidar_tree_node 와 같은 순서."""
    g = fit_ground_plane(pts, nominal_ground_z=-0.1)
    h = g.height(pts)
    trunks = detect_trunks(pts, h)
    xy = np.array([[k.x, k.y] for k in trunks]) if trunks else np.zeros((0, 2))
    w = np.array([k.confidence for k in trunks]) if trunks else np.zeros(0)
    est = tracker.update(fit_rows(xy, w, 3.8, *tracker.prior()), t)
    return RowSeen(est.valid, est.offset, est.heading, est.confidence, est.left_count, est.right_count), xy


def run(orchard, lanes=3, start=(1.0, 0.0, 0.0), obstacle=None, odom_drift=(0.0, 0.0), max_time=500.0,
        seed=0, turn_slip=1.0, yaw_err0=0.0, **plan_kw):
    """planner 모드 폐루프. yaw_err0: 출발 때 odom 요 오차 [rad] — 1 m 까지 그대로, 9 m 에서 0 으로 줄어듦
    (PX4 가 정지 중 요가 틀어져 있다 달리면서 바로잡는 것 흉내). 반환: (planner, 통로 안 횡오차 배열, 상태 기록, 참 궤적, 장애물 최소 거리, 시각)."""
    rng = np.random.default_rng(seed)
    run.reasons = []
    gp = OrchardPlanner(PlannerParams(lanes=lanes, row_spacing=orchard.row_spacing, **plan_kw))
    dwa = DWAPlanner(DWAParams())
    tracker = RowTracker()
    x, y, yaw = start
    v_cur, t, dt = 0.0, 0.0, 0.1
    drift = np.zeros(2)
    gp.start(t, x, y, yaw)
    lateral, states, traj, obs_min = [], [], [], math.inf
    out = None
    step = 0
    obs = np.zeros((0, 2))
    trav, last_xy = 0.0, (x, y)
    while t < max_time and gp.state not in (DONE, 'STOPPED'):
        roll, pitch = rng.normal(0, math.radians(1.5), 2)
        pts = scan(orchard, x, y, yaw, roll=roll, pitch=pitch, rng=rng)
        if obstacle is not None:
            ox, oy = obstacle
            c, s = math.cos(yaw), math.sin(yaw)
            bx, by = c * (ox - x) + s * (oy - y), -s * (ox - x) + c * (oy - y)
            blob = np.c_[bx + rng.uniform(-0.2, 0.2, 300), by + rng.uniform(-0.2, 0.2, 300),
                         rng.uniform(0.0, 1.4, 300) - 0.1]
            pts = np.vstack([pts, blob])
            obs_min = min(obs_min, math.hypot(ox - x, oy - y))
        row, xy = perceive(pts, tracker, t)
        if gp.state == TURN:                                  # U턴 중 odom 이 흐름 (Gazebo PX4 EKF 에서 관찰)
            n = np.hypot(*odom_drift)
            if n > 0:
                drift += np.array(odom_drift) * dt / 5.0     # 5 s 에 걸쳐 최대량까지
                if np.hypot(*drift) > n:
                    drift *= n / np.hypot(*drift)
        ox_, oy_ = x + drift[0], y + drift[1]                 # odom 자세
        trav += math.hypot(x - last_xy[0], y - last_xy[1])
        last_xy = (x, y)
        oyaw = yaw + yaw_err0 * min(1.0, max(0.0, (9.0 - trav) / 8.0))    # odom 요: 1 m 까지 yaw_err0, 9 m 에서 0
        if step % 2 == 0 or out is None:                      # 전역 5 Hz
            out = gp.update(t, ox_, oy_, oyaw, xy, row)
        if step % 2 == 0:                                      # 장애물 5 Hz
            ob = obstacle_points(pts)
            c, s = math.cos(oyaw + out.yaw_bias), math.sin(oyaw + out.yaw_bias)     # 계획기가 추정한 요 오차 보정
            obs = np.c_[ox_ + c * ob[:, 0] - s * ob[:, 1], oy_ + s * ob[:, 0] + c * ob[:, 1]] if len(ob) else ob
        res = dwa.plan(ox_, oy_, oyaw + out.yaw_bias, v_cur, out.path, obs)
        v, w = res.v, res.w
        if gp.state == TURN:
            w *= turn_slip
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw = math.atan2(math.sin(yaw + w * dt), math.cos(yaw + w * dt))
        v_cur = v
        t += dt
        step += 1
        states.append(gp.state)
        run.reasons = getattr(run, 'reasons', [])
        if not run.reasons or run.reasons[-1] != gp.reason:
            run.reasons.append(gp.reason)
        traj.append((x, y, yaw))
        if gp.state == 'FOLLOW_ROW' and 1.0 < x < orchard.row_length - 1.0 and abs(math.cos(yaw)) > 0.9:
            lateral.append(min((y - orchard.lane_y(k) for k in range(orchard.rows - 1)), key=abs))
    run.true_drift = drift.copy()
    return gp, np.array(lateral), states, np.array(traj), obs_min, t


def test_planner_traverses_three_lanes():
    """4열(통로 3개) → 전역 경로로 3통로 지그재그 순회 후 DONE, 통로 안 횡오차 RMS < 0.08 m, 최대 < 0.25 m."""
    orchard = SyntheticOrchard(rows=4, row_length=15.0, seed=2)
    gp, lateral, states, traj, _, t = run(orchard, lanes=3)
    assert gp.state == DONE, (gp.state, gp.reason, t)
    assert gp.lanes_done == 3 and TURN in states
    assert math.sqrt(np.mean(lateral ** 2)) < 0.08, math.sqrt(np.mean(lateral ** 2))
    assert abs(lateral).max() < 0.25, abs(lateral).max()
    assert abs(traj[-1, 1] - orchard.lane_y(2)) < 0.5
    # 나무와 부딪히지 않았는지: 궤적과 줄기 최소 거리 > 0.6 m (로버 반경 0.45 + 여유)
    d = np.min(np.hypot(traj[:, None, 0] - orchard.trees[None, :, 0], traj[:, None, 1] - orchard.trees[None, :, 1]))
    assert d > 0.6, d


def test_planner_avoids_obstacle_in_lane():
    """통로 가운데 x=7 m 장애물 → 멈추지 않고 옆으로 비켜 지나가 통로를 끝낸다 (차체 중심이 장애물 중심과 0.6 m 이상: 장애물 반폭 0.2 + 차체 반폭 0.3 + 여유 약 0.1)."""
    orchard = SyntheticOrchard(rows=2, row_length=15.0, seed=3)
    gp, _, _, traj, obs_min, t = run(orchard, lanes=1, obstacle=(7.0, 0.0), max_time=80.0)
    assert gp.state == DONE, (gp.state, gp.reason, t)
    assert traj[:, 0].max() > 14.0
    assert obs_min > 0.6, obs_min


def test_planner_corrects_odom_drift_in_turn():
    """U턴 중 odom 이 옆으로 0.8 m 흐르고 회전이 20 % 덜 돼도, 다음 통로에서 참 횡오차 최대 < 0.35 m."""
    orchard = SyntheticOrchard(rows=3, row_length=12.0, seed=5)
    gp, lateral, states, traj, _, t = run(orchard, lanes=2, odom_drift=(0.0, 0.8), turn_slip=0.8)
    assert gp.state == DONE, (gp.state, gp.reason, t)
    second = traj[len(traj) // 2:]
    inside = second[(second[:, 0] > 1.0) & (second[:, 0] < orchard.row_length - 1.0)]
    assert abs(inside[:, 1] - orchard.lane_y(1)).max() < 0.35, abs(inside[:, 1] - orchard.lane_y(1)).max()
    assert np.hypot(*(gp.drift - run.true_drift)) < 0.2, (gp.drift, run.true_drift)   # 흐름을 실제로 추정했는지


def test_planner_explore_stops_at_edge():
    """탐사(lanes=0): 4열(통로 3개) 과수원을 모두 돌고 끝 열에서 멈춘다."""
    orchard = SyntheticOrchard(rows=4, row_length=12.0, seed=4)
    gp, _, _, traj, _, t = run(orchard, lanes=0)
    assert gp.state == DONE and '끝 열' in gp.reason, (gp.state, gp.reason, t)
    assert gp.lanes_done == 3, gp.lanes_done


def test_planner_start_with_yaw_error():
    """출발 때 odom 요가 17° 틀려 있다가 달리면서 바로잡혀도 (Gazebo 에서 본 PX4 동작), 두 통로를 가운데로 달린다."""
    orchard = SyntheticOrchard(rows=3, row_length=12.0, seed=5)
    gp, lateral, _, _, _, t = run(orchard, lanes=2, start=(3.0, 0.0, 0.0), yaw_err0=math.radians(17))
    assert gp.state == DONE, (gp.state, gp.reason, t)
    rms = math.sqrt(np.mean(lateral ** 2))
    assert abs(lateral).max() < 0.25 and rms < 0.08, (abs(lateral).max(), rms)


def test_planner_obstacle_with_start_yaw_error_keeps_heading():
    """출발 요 오차 16° + 통로 가운데(10 m) 사람 → 비켜 지나가 끝내고, 요 오차 추정이 0 으로 돌아오며 흐름·열 방향이 안 틀어진다.
    (막으려는 현상: 장애물 비키는 중 요 오차와 열 방향이 함께 발산해 나무 줄로 들어가 멈춤)."""
    orchard = SyntheticOrchard(rows=2, row_length=15.0, seed=3)
    gp, _, _, traj, obs_min, t = run(orchard, lanes=1, start=(-3.0, 0.0, 0.0), obstacle=(10.0, 0.06),
                                     max_time=120.0, yaw_err0=math.radians(16))
    assert gp.state == DONE, (gp.state, gp.reason, t)
    assert obs_min > 0.6, obs_min
    assert abs(math.degrees(gp.yaw_bias)) < 3.0, math.degrees(gp.yaw_bias)
    assert abs(math.degrees(gp.model.theta)) < 3.0, math.degrees(gp.model.theta)
    assert np.hypot(*gp.drift) < 0.2, gp.drift
    d = np.min(np.hypot(traj[:, None, 0] - orchard.trees[None, :, 0], traj[:, None, 1] - orchard.trees[None, :, 1]))
    assert d > 0.6, d


def test_planner_approach_finds_entrance_from_outside():
    """과수원 밖(입구 9 m 앞, 오른쪽 3.5 m, 20° 비스듬히 / 반대쪽 −17°)에서 출발 + 출발 요 오차 16°
    → LiDAR 로 오른쪽 끝 통로 입구를 찾아 정렬 → 통로 0 을 가운데로 달리고 DONE."""
    for start in ((-9.0, -3.5, 0.35), (-10.0, 1.5, -0.3)):
        orchard = SyntheticOrchard(rows=3, row_length=12.0, seed=6)
        gp, _, states, traj, _, t = run(orchard, lanes=1, start=start, max_time=120.0,
                                        yaw_err0=math.radians(16), approach=True)
        assert gp.state == DONE and 'APPROACH' in states, (start, gp.state, gp.reason, t)
        inside = traj[(traj[:, 0] > 2.0) & (traj[:, 0] < 11.0)]
        assert len(inside) > 10 and abs(inside[:, 1]).max() < 0.3, (start, abs(inside[:, 1]).max())
        T = orchard.trees
        d = np.min(np.hypot(traj[:, None, 0] - T[None, :, 0], traj[:, None, 1] - T[None, :, 1]))
        assert d > 0.6, (start, d)
