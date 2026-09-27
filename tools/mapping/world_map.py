"""Gazebo 과수원 월드 정답 지도(위에서 본 그림) — 결과(주행 궤적·찾은 줄기)와 겹쳐 보기. ROS 없이 실행.

입력
  orchard_meta.json   월드 생성기(orchard_gazebo/world_gen.py)가 만든 정답: 나무·열·통로·장애물(종류·위치·방향·크기)·바위·농로·출발 위치
  (선택) --voxel 폴더  voxel_map_node 결과 (trajectory.csv, trunks.csv, meta.json) → 월드 좌표로 바꿔 겹침
  (선택) --timeline    녹화 timeline.txt (Gazebo 참 위치 x, y 기록) → 참 궤적 점 (과수원 진입 구간 포함)
출력
  <out>_truth.png    정답 지도만 (나무·열 번호·통로 번호·장애물·바위·농로·출발 자세)
  <out>_result.png   정답 + 주행 궤적 + 찾은 줄기(정답까지 거리) — --voxel 을 줄 때만

좌표: Gazebo 월드 [m] (x = 열 방향, y = 왼쪽). voxel 결과(start 좌표)는 start → odom(meta.json 원점·방향) → 월드(odom 원점 = 출발 위치) 로 바꾼다.

사용 예
  python3 tools/mapping/world_map.py orchard_meta.json --out data/media/full/world_map \\
      --voxel data/voxel_maps/voxel_map_<날짜시각> --timeline data/media/full/timeline.txt
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon, Rectangle  # noqa: E402


def load_voxel(folder: str, spawn_xy):
    """voxel_map 폴더 → (궤적 N×2, 줄기 M×2, 줄기 행 목록) 월드 좌표."""
    meta = json.load(open(os.path.join(folder, 'meta.json'), encoding='utf-8'))
    yaw0 = math.radians(meta['x_axis_odom_heading_deg'])
    o = np.array(meta['origin_odom_xyz'][:2]) + np.asarray(spawn_xy, float)
    c, s = math.cos(yaw0), math.sin(yaw0)

    def to_world(P):
        P = np.asarray(P, float).reshape(-1, 2)
        return np.c_[o[0] + c * P[:, 0] - s * P[:, 1], o[1] + s * P[:, 0] + c * P[:, 1]]
    traj, trunks = np.zeros((0, 2)), np.zeros((0, 2))
    rows = []
    fn = os.path.join(folder, 'trajectory.csv')
    if os.path.isfile(fn):
        traj = to_world([[float(r['x']), float(r['y'])] for r in csv.DictReader(open(fn, encoding='utf-8'))])
    fn = os.path.join(folder, 'trunks.csv')
    if os.path.isfile(fn):
        rows = list(csv.DictReader(open(fn, encoding='utf-8')))
        if rows:
            trunks = to_world([[float(r['x']), float(r['y'])] for r in rows])
    return traj, trunks, rows


def load_timeline(fn: str) -> np.ndarray:
    """녹화 timeline.txt 의 Gazebo 참 위치 'x: .. y: ..' 들 (N×2)."""
    pts = re.findall(r'^x: (-?[\d.e+-]+) y: (-?[\d.e+-]+)', open(fn, encoding='utf-8').read(), re.M)
    return np.array([[float(a), float(b)] for a, b in pts]).reshape(-1, 2)


def draw_world(ax, m: dict, label_rows: bool = True):
    """정답 월드: 농로, 나무 밑 띠, 나무, 열·통로 번호, 장애물, 바위, 출발 자세."""
    T = np.array([[t['x'], t['y']] for t in m['trees']])
    row_y = m.get('row_y', sorted({round(t['y'], 1) for t in m['trees']}))
    x0, x1 = m.get('row_x_range', [float(T[:, 0].min()), float(T[:, 0].max())])
    y_lo, y_hi = min(row_y) - 5.0, max(row_y) + 5.0
    for a, b in m.get('roads_x', []):                                  # 자갈 농로 (회전 공간)
        ax.add_patch(Rectangle((a, y_lo), b - a, y_hi - y_lo, color='#c8c2b4', zorder=0))
    for y in row_y:                                                    # 나무 밑 제초 띠
        ax.add_patch(Rectangle((x0 - 0.5, y - 0.5), x1 - x0 + 1.0, 1.0, color='#d9c7a7', zorder=1))
    lanes = m.get('lane_centers_y', [])
    for k, y in enumerate(lanes):                                      # 통로 중심선
        ax.plot([x0 - 1.0, x1 + 1.0], [y, y], ls=':', color='#5a8fd6', lw=1.0, zorder=2)
        ax.text(x1 + 1.3, y, f'lane {k}', color='#2f6fc4', va='center', fontsize=9, zorder=6)
    ax.scatter(T[:, 0], T[:, 1], s=26, c='#2e7d32', edgecolors='#1b4d1f', linewidths=0.5, zorder=4,
               label=f'tree (truth, {len(T)})')
    miss = []                                                          # 결주 자리 (같은 열 이웃 간격이 주간 거리의 1.6 배 넘는 곳)
    sp_t = float(m.get('config', {}).get('tree_spacing', 1.2) or 1.2)
    for y in row_y:
        xs = np.sort(T[np.abs(T[:, 1] - y) < 0.5, 0])
        for a, b in zip(xs[:-1], xs[1:]):
            n = int(round((b - a) / sp_t)) - 1
            miss += [(a + sp_t * (i + 1), y) for i in range(n)] if (b - a) > 1.6 * sp_t else []
    if miss:
        M = np.array(miss)
        ax.scatter(M[:, 0], M[:, 1], s=40, facecolors='none', edgecolors='#8d6e63', linewidths=1.2, zorder=4,
                   label=f'missing tree (truth, {len(M)})')
    if label_rows:
        for j, y in enumerate(sorted(row_y)):
            ax.text(x0 - 1.3, y, f'row {j + 1}', ha='right', va='center', fontsize=9, color='#1b4d1f', zorder=6)
    for ob in m.get('obstacles', []):                                  # 장애물 (충돌 상자)
        L, W = ob.get('size', [0.45, 0.3] if ob['type'] == 'person' else [1.0, 0.6])
        yaw = ob.get('yaw', 0.0)
        c, s = math.cos(yaw), math.sin(yaw)
        corners = [(ob['x'] + c * u - s * v, ob['y'] + s * u + c * v)
                   for u, v in ((L / 2, W / 2), (-L / 2, W / 2), (-L / 2, -W / 2), (L / 2, -W / 2))]
        col = '#d62728' if ob['type'] == 'person' else '#1f5fbf'
        ax.add_patch(Polygon(corners, closed=True, color=col, zorder=5))
        ax.text(ob['x'], ob['y'] + 0.55, ob['type'], ha='center', fontsize=8, color=col, zorder=6)
    R = [(r['x'], r['y']) for r in m.get('rocks', [])]
    if R:
        R = np.array(R)
        ax.scatter(R[:, 0], R[:, 1], s=10, c='#777777', marker='o', zorder=3, label='rock')
    sp = m.get('spawn')
    if sp:
        yaw = float(m.get('config', {}).get('spawn_yaw', 0.0) or 0.0)
        ax.annotate('', xy=(sp[0] + 1.5 * math.cos(yaw), sp[1] + 1.5 * math.sin(yaw)), xytext=(sp[0], sp[1]),
                    arrowprops=dict(arrowstyle='-|>', color='k', lw=2), zorder=7)
        ax.plot(sp[0], sp[1], 'k*', ms=12, zorder=7, label='rover start')
    ax.set_aspect('equal')
    ax.set_xlabel('x [m] (Gazebo world, row direction)')
    ax.set_ylabel('y [m]')
    ax.grid(alpha=0.25)
    return T


def main() -> None:
    """정답 지도(와 결과 겹침) PNG 저장."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('meta', help='orchard_meta.json (월드 정답)')
    ap.add_argument('--out', required=True, help='출력 경로 앞부분 (… _truth.png, _result.png)')
    ap.add_argument('--voxel', default='', help='voxel_map_<날짜시각> 폴더 (궤적·줄기 겹치기)')
    ap.add_argument('--timeline', default='', help='녹화 timeline.txt (Gazebo 참 위치 점)')
    ap.add_argument('--title', default='Gazebo orchard world (ground truth)')
    a = ap.parse_args()
    m = json.load(open(a.meta, encoding='utf-8'))
    n_rows = len(m.get('row_y', []))
    L = m.get('row_x_range', [0, 0])[1]

    fig, ax = plt.subplots(figsize=(14, 8))
    draw_world(ax, m)
    ax.set_title(f'{a.title} — {n_rows} rows x {L:.0f} m, {len(m["trees"])} trees, '
                 f'{len(m.get("obstacles", []))} obstacles')
    ax.legend(loc='lower right', fontsize=8)
    fig.tight_layout()
    fig.savefig(a.out + '_truth.png', dpi=130)
    print('저장:', a.out + '_truth.png')

    if a.voxel:
        traj, trunks, rows = load_voxel(a.voxel, m['spawn'][:2])
        fig, ax = plt.subplots(figsize=(14, 8))
        T = draw_world(ax, m)
        if a.timeline and os.path.isfile(a.timeline):
            G = load_timeline(a.timeline)
            ax.plot(G[:, 0], G[:, 1], '.', color='#555555', ms=4, zorder=6, label='rover (Gazebo truth, sampled)')
        if len(traj):
            ax.plot(traj[:, 0], traj[:, 1], '-', color='#1565c0', lw=1.4, zorder=6, label='rover path (odom)')
        info = ''
        if len(trunks):
            d = np.min(np.hypot(trunks[:, None, 0] - T[None, :, 0], trunks[:, None, 1] - T[None, :, 1]), axis=1)
            ok = d <= 0.3
            ax.scatter(trunks[ok, 0], trunks[ok, 1], s=40, marker='x', c='#ff8f00', zorder=7,
                       label=f'trunk found (≤0.3 m, {ok.sum()})')
            ax.scatter(trunks[~ok, 0], trunks[~ok, 1], s=40, marker='x', c='#c2185b', zorder=7,
                       label=f'trunk found (>0.3 m, {(~ok).sum()})')
            matched = len({int(np.argmin(np.hypot(T[:, 0] - p[0], T[:, 1] - p[1]))) for p in trunks[ok]})
            info = (f' — trunks found {len(trunks)} (median error {np.median(d):.2f} m), '
                    f'true trees matched {matched}/{len(T)}')
        ax.set_title(f'Result on ground truth{info}')
        ax.legend(loc='lower right', fontsize=8)
        fig.tight_layout()
        fig.savefig(a.out + '_result.png', dpi=130)
        print('저장:', a.out + '_result.png', info)


if __name__ == '__main__':
    main()
