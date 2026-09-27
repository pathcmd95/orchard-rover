"""저장된 복셀 맵 + 줄기 지도를 그림(PNG)으로 보기 — ROS 없이, 노트북에서도 실행 가능.

입력: voxel_map_node 가 저장한 폴더 (data/voxel_maps/voxel_map_<날짜시각>/)
  voxel_map.npz  복셀 전체 (orchard_mapping.voxel_map.VoxelMap.save_npz)
  trunks.csv     확정 줄기 (id,x,y,z,hits,mean_conf,source,...)
  trajectory.csv 주행 궤적 (t,x,y,z,yaw)
  (선택) --truth orchard_meta.json  Gazebo 월드 정답 나무 위치 → 줄기 위치 오차 계산 (시뮬 전용)

출력: <폴더>/voxel_view.png — 왼쪽 위에서 본 지도(높이 색 복셀 + 줄기 번호 + 궤적), 오른쪽 3D 비스듬히 본 모습

사용 예
  python3 tools/mapping/view_voxel_map.py data/voxel_maps/voxel_map_<날짜시각>
  python3 tools/mapping/view_voxel_map.py <위 폴더> --truth ~/.cache/orchard_rover/orchard_meta.json   # 시뮬 정답과 비교

좌표: 모든 값은 'start' 프레임 — 출발점 (0,0,0), 처음 곧게 달린 방향 +x, 왼쪽 +y, 위 +z [m].
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, 'ros2_ws', 'src', 'orchard_mapping'))
import matplotlib  # noqa: E402
matplotlib.use('Agg')                       # 화면 없이 PNG 로만 저장
import matplotlib.pyplot as plt  # noqa: E402

from orchard_mapping.voxel_map import VoxelMap, height_colors  # noqa: E402


def read_csv(path: str) -> list[dict]:
    """CSV → 딕셔너리 목록 (없으면 빈 목록)."""
    if not os.path.isfile(path):
        return []
    with open(path, encoding='utf-8') as f:
        return list(csv.DictReader(f))


def truth_in_start(meta_path: str, map_meta: dict) -> np.ndarray:
    """Gazebo 월드 정답 나무 (월드 좌표) → start 좌표 (N×2).

    시뮬의 odom 원점 = 로버 스폰 위치(월드), odom 축 = 월드 축 이라고 보고
    start = R(-yaw0) · (월드 − 스폰 − origin_odom) 으로 바꾼다.
    """
    with open(meta_path, encoding='utf-8') as f:
        world = json.load(f)
    trees = np.array([[t['x'], t['y']] for t in world.get('trees', [])], float)
    spawn = np.array(world.get('spawn', [0.0, 0.0, 0.0])[:2], float)     # 로버 스폰 위치 (월드) = 시뮬 odom 원점
    o = np.array(map_meta['origin_odom_xyz'][:2], float)
    yaw0 = math.radians(map_meta['x_axis_odom_heading_deg'])
    c, s = math.cos(-yaw0), math.sin(-yaw0)
    d = trees - spawn - o
    return np.c_[c * d[:, 0] - s * d[:, 1], s * d[:, 0] + c * d[:, 1]]


def main() -> None:
    """폴더를 읽어 그림 저장, 줄기 수·(정답이 있으면) 오차를 출력."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('folder', help='voxel_map_<날짜시각> 폴더')
    ap.add_argument('--min-hits', type=int, default=2, help='이 횟수 이상 관측된 복셀만 그림')
    ap.add_argument('--truth', default='', help='(시뮬) Gazebo orchard_meta.json — 줄기 오차 계산')
    ap.add_argument('--margin', type=float, default=4.0, help='궤적·줄기 둘레 몇 m 까지 그릴지 (0 이면 전체)')
    ap.add_argument('--max-points', type=int, default=60000, help='3D 그림에 쓸 최대 복셀 수 (많으면 느림)')
    a = ap.parse_args()

    vm = VoxelMap.load_npz(os.path.join(a.folder, 'voxel_map.npz'))
    c, _h = vm.occupied(a.min_hits)
    hag = vm.height_above_ground(c)
    rgb = height_colors(np.nan_to_num(hag, nan=0.0))
    trunks = read_csv(os.path.join(a.folder, 'trunks.csv'))
    traj = np.array([[float(r['x']), float(r['y'])] for r in read_csv(os.path.join(a.folder, 'trajectory.csv'))])
    meta_path = os.path.join(a.folder, 'meta.json')
    meta = json.load(open(meta_path, encoding='utf-8')) if os.path.isfile(meta_path) else {}

    fig = plt.figure(figsize=(16, 7))
    ax = fig.add_subplot(1, 2, 1)
    order = np.argsort(c[:, 2])                                   # 낮은 것부터 그려 높은 복셀(수관)이 위에 보이게
    ax.scatter(c[order, 0], c[order, 1], c=rgb[order], s=2, marker='s', linewidths=0)
    if len(traj):
        ax.plot(traj[:, 0], traj[:, 1], 'k-', lw=1, label='trajectory')
    ax.plot(0, 0, 'r*', ms=14, label='start (0,0)')
    for t in trunks:
        col = 'orange' if t['source'] == 'yolo' else 'gray'
        ax.plot(float(t['x']), float(t['y']), 'o', mfc='none', mec=col, ms=9, mew=2)
        ax.text(float(t['x']) + 0.15, float(t['y']) + 0.15, f"T{t['id']}", fontsize=7)
    if a.truth and meta:
        gt = truth_in_start(a.truth, meta)
        ax.plot(gt[:, 0], gt[:, 1], 'm+', ms=6, label='ground truth')
        if trunks:
            p = np.array([[float(t['x']), float(t['y'])] for t in trunks])
            d = np.min(np.hypot(p[:, None, 0] - gt[None, :, 0], p[:, None, 1] - gt[None, :, 1]), axis=1)
            print(f'줄기 {len(p)} 개, 정답까지 거리 중앙값 {np.median(d):.3f} m, 최대 {d.max():.3f} m')
    keep = [traj] + ([np.array([[float(t['x']), float(t['y'])] for t in trunks])] if trunks else [])
    keep = np.vstack([k for k in keep if len(k)]) if any(len(k) for k in keep) else np.zeros((0, 2))
    if len(keep) and a.margin > 0:                                # 궤적·줄기 주변만 확대 (먼 지면은 잘라냄)
        lo, hi = keep.min(0) - a.margin, keep.max(0) + a.margin
        ax.set_xlim(lo[0], hi[0])
        ax.set_ylim(lo[1], hi[1])
        m = (c[:, 0] > lo[0]) & (c[:, 0] < hi[0]) & (c[:, 1] > lo[1]) & (c[:, 1] < hi[1])
        c, rgb = c[m], rgb[m]                                    # 3D 그림도 같은 범위만
    ax.set_aspect('equal')
    ax.set_xlabel('x [m] (first driving direction)')
    ax.set_ylabel('y [m] (left)')
    ax.set_title(f'Top view — {len(c)} voxels, {len(trunks)} trunks (orange=YOLO, gray=LiDAR)')
    ax.legend(loc='upper right', fontsize=8)

    ax3 = fig.add_subplot(1, 2, 2, projection='3d')
    k = np.random.default_rng(0).permutation(len(c))[:a.max_points]
    ax3.scatter(c[k, 0], c[k, 1], c[k, 2], c=rgb[k], s=1, marker='s', linewidths=0)
    for t in trunks:
        ax3.plot([float(t['x'])] * 2, [float(t['y'])] * 2, [float(t['z']), float(t['z']) + 1.0], color='orange', lw=2)
    ax3.view_init(elev=35, azim=-60)
    ax3.set_title('3D view (height colour: green ground → blue canopy)')
    span = np.ptp(c, axis=0) if len(c) else np.ones(3)
    ax3.set_box_aspect((span[0], span[1], max(span[2], 1.0)))
    out = os.path.join(a.folder, 'voxel_view.png')
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f'저장: {out}  (복셀 {len(c)}, 줄기 {len(trunks)})')


if __name__ == '__main__':
    main()
