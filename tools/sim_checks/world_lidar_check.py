#!/usr/bin/env python3
"""생성한 과수원 월드에서 Livox Mid-360 을 광선추적으로 흉내 내 LiDAR 인식(지면·줄기·행)을 점검한다.

Gazebo 없이 노트북에서 돌릴 수 있다 (월드를 바꿨을 때 인식이 망가지지 않았는지 빠르게 확인용).
  pip install trimesh embreex        # embreex 가 없으면 느리지만 동작
  python3 tools/sim_checks/world_lidar_check.py --lite
결과: 위치별 중심선 오차(offset, heading), 줄기 검출 수, 통로 장애물 거리(오검출 확인)

동작
  1) orchard_gazebo.world_gen 으로 sim.launch.py 와 같은 과수원 월드(메시)를 생성
  2) 각 통로의 여러 위치(x 3.5 m 간격) × 자세(중앙 / ±0.35 m 옆 + ±6° 틀어짐)에 로버를 지면에 맞춰 놓고
  3) Mid-360 광선(수평 720 × 수직 64, -7°~+52°)을 메시와 교차시켜 점군 생성 (+거리 잡음 2 cm)
  4) lidar_tree_node 와 같은 함수(지면 적합 → 줄기 검출 → 행 적합 → 통로 장애물)를 적용
  5) 참값(로버를 놓은 위치)과 비교해 표와 요약(RMS 오차, 오검출 수) 출력
인자
  --lite      저사양 월드(잔디·열매 축소) — 삼각형 수가 줄어 훨씬 빠름
  --out-dir   월드 생성 폴더 (기본: 임시폴더/orchard_check)
  --seed      월드 난수 시드 (sim.launch.py 기본 7 과 같음)

References
  - trimesh (메시 불러오기·광선 교차): https://github.com/mikedh/trimesh
  - Intel Embree (trimesh 가 embreex 패키지를 통해 쓰는 고속 광선추적 커널, 없으면 느린 순수 파이썬 경로 사용)
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import tempfile

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # 저장소 최상위 (세 단계 위)
# ROS 빌드 없이 저장소 소스의 파이썬 모듈을 직접 import
for pkg in ('orchard_gazebo', 'orchard_perception'):
    sys.path.insert(0, os.path.join(ROOT, 'ros2_ws', 'src', pkg))

from orchard_gazebo.world_gen import config_from_args, generate_world, ground_for  # noqa: E402
from orchard_perception.ground import fit_ground_plane  # noqa: E402
from orchard_perception.rows import corridor_obstacle_distance, fit_rows  # noqa: E402
from orchard_perception.trunks import detect_trunks  # noqa: E402

# lidar_tree_node 기본값 + sim.yaml 과 같게 유지 (obstacle.h_max 0.75 는 sim.yaml)
#   roi   = (x_min, x_max, |y| 최대, z_min, z_max) 관심영역 [m]
#   body  = (x_min, x_max, |y| 최대) 차체 자기 점 제거 박스 [m]
#   nominal_z = 평지 지면 높이 [m, base_link 기준], band = 줄기 높이 밴드 [m], width = 예상 행간 [m]
P = dict(roi=(-4.0, 15.0, 6.0, -1.5, 2.5), body=(-0.7, 0.7, 0.45), nominal_z=-0.1, band=(0.25, 0.7),
         width=3.8)
LIDAR_XYZ = np.array([0.30, 0.0, 0.50])          # rover.yaml sensors.livox.xyz
BASE_H = 0.10                                      # PX4 rover_ackermann base_link 높이


def load_scene_mesh(meta):
    """월드 생성기가 만든 scene.json 의 지형·나무줄·지면 덮개 메시를 모두 읽어 하나의 trimesh 로 합친다."""
    import json

    import trimesh
    adir = meta['assets_dir']
    scene = json.load(open(os.path.join(adir, 'scene.json')))
    files = [scene['terrain_core']] + [f for r in scene['rows'] for f in r['files'].values()] \
        + list(scene['cover'].values())
    meshes = [trimesh.load(os.path.join(adir, f), force='mesh', process=False) for f in files]
    return trimesh.util.concatenate(meshes)


def rays(h=720, v=64, vmin=-7.0, vmax=52.0):
    """센서 좌표계 광선 방향 단위벡터 (h*v, 3). 기본값은 rover.yaml sensors.livox.sim 과 같다.

    Args:
        h: 수평 광선 수 (360° 균등).
        v: 수직 광선 수.
        vmin, vmax: 수직 시야각 범위 [deg].
    """
    # 방위각(az) × 고도각(el) 격자 → 구면좌표를 직교좌표 (x 전방, y 왼쪽, z 위) 로 변환
    az = np.linspace(-math.pi, math.pi, h, endpoint=False)
    el = np.radians(np.linspace(vmin, vmax, v))
    A, E = np.meshgrid(az, el)
    return np.stack([np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)], -1).reshape(-1, 3)


def pose_matrix(ground, x, y, yaw):
    """지면 법선에 맞춘 로버 자세 (world ← base).

    기복 있는 지형에서 로버가 지면에 붙어 기울어진 상태를 흉내 낸다.
    Returns:
        (R, t): 회전행렬 3x3 (열 = base 의 x/y/z 축을 월드에서 본 방향), base_link 원점 위치 [m].
    """
    n = ground.normal(np.array([x]), np.array([y]))[0]
    # 진행 방향을 지면 평면에 투영 → 전방축, 법선 × 전방 = 왼쪽축 (오른손 좌표계)
    fwd = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    fwd = fwd - n * fwd.dot(n)
    fwd /= np.linalg.norm(fwd)
    left = np.cross(n, fwd)
    R = np.column_stack([fwd, left, n])
    # base_link 는 지면에서 법선 방향으로 BASE_H 위
    t = np.array([x, y, float(ground.height(x, y))]) + n * BASE_H
    return R, t


def main():
    """월드 생성 → 통로 여러 위치에서 가상 LiDAR 스캔 → 인식 결과를 참값과 비교해 표·요약 출력."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--lite', action='store_true')
    ap.add_argument('--out-dir', default=os.path.join(tempfile.gettempdir(), 'orchard_check'))
    ap.add_argument('--seed', type=int, default=7)
    a = ap.parse_args()
    # 월드 생성기에 sim.launch.py 와 같은 형식의 인자를 넘긴다 (나머지는 생성기 기본값)
    argv = ['--out-dir', a.out_dir, '--seed', str(a.seed)] + (['--lite'] if a.lite else [])
    cfg, out = config_from_args(argv)
    _, meta_path = generate_world(cfg, out)
    import json
    meta = json.load(open(meta_path))
    ground = ground_for(cfg)                  # 지형 높이·법선 함수 (월드와 같은 수식)
    mesh = load_scene_mesh(meta)
    print(f'삼각형 {len(mesh.faces) / 1e6:.2f}M')
    dirs_b = rays()
    rng = np.random.default_rng(0)
    rows = []
    # 시험 위치: 모든 통로 k × 행 방향 x(3.5 m 간격) × 자세 3가지(중앙 / 왼쪽 0.35 m·+6° / 오른쪽 0.35 m·-6°)
    for k in range(cfg.lanes):
        yc = cfg.lane_y(k)                    # 통로 k 의 중심 y [m]
        for x in np.arange(-1.0, cfg.row_length - 1.0, 3.5):
            for dy, dyaw in ((0.0, 0.0), (0.35, 6.0), (-0.35, -6.0)):
                yaw = math.radians(dyaw)
                R, t = pose_matrix(ground, x, yc + dy, yaw)
                origin = t + R @ LIDAR_XYZ    # LiDAR 광원 위치 (월드)
                d = dirs_b @ R.T              # 광선 방향 (센서 → 월드 회전)
                # 광선마다 첫 번째 교차점만 (실제 LiDAR 처럼 가장 가까운 면에서 반사)
                loc, idx, _ = mesh.ray.intersects_location(np.repeat(origin[None], len(d), 0), d,
                                                           multiple_hits=False)
                # 측정 가능 거리 0.1~40 m 만 남기고, 광선 방향으로 거리 잡음 σ = 2 cm 추가
                rng_m = np.linalg.norm(loc - origin, axis=1)
                ok = (rng_m > 0.1) & (rng_m < 40.0)
                loc = loc[ok] + rng.normal(0, 0.02, (ok.sum(), 3)) * (d[idx[ok]])
                pts = (loc - t) @ R                  # world → base
                # lidar_tree_node 와 같은 전처리: 관심영역(ROI) 안, 차체 박스 밖의 점만
                x0, x1, ya, z0, z1 = P['roi']
                roi = (pts[:, 0] > x0) & (pts[:, 0] < x1) & (np.abs(pts[:, 1]) < ya) & \
                      (pts[:, 2] > z0) & (pts[:, 2] < z1)
                b = P['body']
                body = (pts[:, 0] > b[0]) & (pts[:, 0] < b[1]) & (np.abs(pts[:, 1]) < b[2])
                pts = pts[roi & ~body]
                # 아래 숫자 인자는 lidar_tree_node 의 ground.* / trunk.* / obstacle.* 기본값과 같은 순서
                #   지면: search_band 0.4, distance_threshold 0.08, max_tilt_deg 20, iterations 60, min_inliers 150
                #   줄기: cell_size 0.08, min_points 6, max_diameter 0.45, min_height_span 0.12
                #   장애물: half_width 0.5, x_min 0.3, x_max 6.0, h_min 0.2, h_max 0.75(sim.yaml), min_points 5
                g = fit_ground_plane(pts, P['nominal_z'], 0.4, 0.08, 20.0, 60, 150, rng)
                hts = g.height(pts)
                tr = detect_trunks(pts, hts, *P['band'], 0.08, 6, 0.45, 0.12)
                xy = np.array([[q.x, q.y] for q in tr]) if tr else np.zeros((0, 2))
                cf = np.array([q.confidence for q in tr]) if tr else np.zeros(0)
                est = fit_rows(xy, cf, P['width'])
                obs = corridor_obstacle_distance(pts, hts, est.offset if est.valid else 0.0,
                                                 est.heading if est.valid else 0.0, 0.5, 0.3, 6.0, 0.2, 0.75, 5)
                # 참값: 로봇에서 본 통로 중심까지 횡거리(= -dy, 틀어진 각도만큼 보정). heading 참값은 -yaw
                true_off = (yc - (yc + dy)) / math.cos(yaw)
                in_rows = 0.5 < x < cfg.row_length - 3   # 행 안쪽 위치만 요약 통계에 포함 (입구·끝 제외)
                # 기록: (통로, x, dy, dyaw, 점 수, 줄기 수, 행 인식, offset 오차, heading 오차[°], 장애물 거리, 행 안, 지면 적합)
                rows.append((k, x, dy, dyaw, len(pts), len(tr), est.valid, est.offset - true_off,
                             math.degrees(est.heading + yaw), obs, in_rows, g.fitted))
    print(' 통로   x    dy  dyaw  점수 줄기  행  off오차[cm] head오차[°]  장애물[m]')
    for r in rows:
        k, x, dy, dyaw, npt, ntr, v, eo, eh, obs, _, _ = r
        print(f'{k:4d} {x:5.1f} {dy:+.2f} {dyaw:+4.0f} {npt:6d} {ntr:4d} {"OK" if v else "--":>3} '
              f'{eo * 100:+8.1f} {eh:+9.1f} {obs:9.2f}')
    # 요약: 행 안 위치만. 통로 장애물이 6 m 안에 잡히면(장애물이 없는 월드이므로) 오검출
    inside = [r for r in rows if r[10]]
    valid = [r for r in inside if r[6]]
    eo = np.array([r[7] for r in valid])
    eh = np.array([r[8] for r in valid])
    false_obs = sum(1 for r in inside if r[9] < 6.0)
    print(f'\n행 안 {len(inside)}곳: 행 인식 {len(valid)}/{len(inside)}, '
          f'offset 오차 RMS {np.sqrt(np.mean(eo ** 2)) * 100:.1f} cm (최대 {np.abs(eo).max() * 100:.1f}), '
          f'heading 오차 RMS {np.sqrt(np.mean(eh ** 2)):.1f}° (최대 {np.abs(eh).max():.1f}), '
          f'통로 장애물 오검출 {false_obs}곳, 지면 적합 {sum(r[11] for r in inside)}/{len(inside)}')


if __name__ == '__main__':
    main()
