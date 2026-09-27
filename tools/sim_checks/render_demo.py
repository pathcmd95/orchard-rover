#!/usr/bin/env python3
"""알고리즘 데모 영상/사진 생성 (Gazebo·PX4 없이).

가상 과수원 LiDAR(synthetic.py) → 지면추정·줄기검출·행 추정 → 미션 상태기계 → 운동학 모델
의 폐루프를 돌리면서 매 프레임을 그림으로 남긴다. 결과는 실제 알고리즘 출력이지만,
차량 동역학과 센서 렌더링은 단순화된 모델이다(Gazebo 영상이 아님).

    python3 tools/sim_checks/render_demo.py --out media/

언제 쓰나: 알고리즘(인식·미션)을 고친 뒤 Gazebo 없이 빠르게 동작을 눈으로 확인하거나 발표 자료를 만들 때.
필요: numpy, matplotlib, ffmpeg (한글 글꼴 Noto Sans CJK 또는 나눔고딕이 있으면 제목이 깨지지 않음).
      ROS 설치는 필요 없다 (ros2_ws/src 의 순수 파이썬 모듈을 sys.path 로 직접 불러온다).
인자: --out 결과 폴더 (기본 <저장소>/media)
출력 (--out 폴더)
    demo_row_following.mp4   3통로 행 추종 + U턴 영상
    demo_obstacle_stop.mp4   통로 안 장애물 앞 정지 영상
    photo_perception.png     행 추종 중 한 장면
    photo_full_path.png      전체 주행 궤적
    stats.txt                완료 통로 수, 통로 중심 횡오차(RMS/최대), 장애물 정지 위치
    *_frames/                영상용 프레임 PNG (지워도 됨)
"""
from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # 저장소 최상위 (세 단계 위)
# ROS 빌드 없이 orchard_perception / orchard_navigation 파이썬 모듈을 import 할 수 있도록 경로 추가
sys.path[:0] = [os.path.join(ROOT, 'ros2_ws/src', p) for p in ('orchard_perception', 'orchard_navigation')]

import matplotlib  # noqa: E402

matplotlib.use('Agg')   # 화면 없이 파일로만 그리기 (서버·컨테이너에서도 동작)
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402
from matplotlib.patches import Polygon  # noqa: E402

from orchard_navigation.controller import FollowParams  # noqa: E402
from orchard_navigation.mission import DONE, MissionParams, OrchardMission, Pose2D, RowObs  # noqa: E402
from orchard_perception.ground import fit_ground_plane  # noqa: E402
from orchard_perception.rows import RowTracker, corridor_obstacle_distance, fit_rows  # noqa: E402
from orchard_perception.synthetic import SyntheticOrchard, scan  # noqa: E402
from orchard_perception.trunks import detect_trunks  # noqa: E402

# 한글 글꼴을 찾아 matplotlib 기본 글꼴로 등록 (없으면 한글이 네모로 표시됨)
for f in font_manager.findSystemFonts():
    if 'NotoSansCJK-Regular' in f or 'NanumGothic' in f:
        font_manager.fontManager.addfont(f)
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=f).get_name()
        break
plt.rcParams['axes.unicode_minus'] = False   # 한글 글꼴에서 음수 부호(−)가 깨지지 않게

# 미션 상태 이름 → 화면 표시용 한국어
STATE_KO = {'FOLLOW_ROW': '행 추종', 'EXIT_ROW': '행 이탈', 'TURN': 'U턴', 'ENTER_ROW': '다음 통로 진입',
            'DONE': '완료', 'STOPPED': '정지', 'IDLE': '대기'}


def simulate(orchard, lanes, max_time, obstacle=None, seed=0):
    """가상 LiDAR → 인식 → 미션 → 운동학 모델 폐루프를 0.1 s 간격으로 돌린다.

    Args:
        orchard: SyntheticOrchard (나무 배치).
        lanes: 주행할 통로 수.
        max_time: 최대 시뮬 시간 [s]. 미션이 DONE/STOPPED 가 되면 더 일찍 끝난다.
        obstacle: (x, y) 월드 좌표 장애물 위치 [m] 또는 None.
        seed: 잡음 난수 시드.
    Returns:
        (frames, mission): 프레임별 기록 dict 목록, 최종 미션 객체.
    """
    rng = np.random.default_rng(seed)
    mission = OrchardMission(MissionParams(lanes=lanes, row_spacing=orchard.row_spacing, follow=FollowParams()))
    tracker = RowTracker()
    pose = Pose2D(-3.0, 0.0, 0.0)          # 첫 통로 입구 3 m 앞에서 +x 방향으로 출발
    dt, t = 0.1, 0.0                       # 제어 주기 0.1 s (10 Hz, LiDAR 주기와 같음)
    mission.start(t, pose)
    frames = []
    while t < max_time and mission.state not in (DONE, 'STOPPED'):
        # 울퉁불퉁한 지면을 흉내 내려고 차체 기울기(roll/pitch)를 매 프레임 무작위로 준다 (σ = 1.5°)
        roll, pitch = rng.normal(0, math.radians(1.5), 2)
        pts = scan(orchard, pose.x, pose.y, pose.yaw, roll=roll, pitch=pitch, rng=rng)
        if obstacle is not None:
            # 장애물(사람 크기: 40 cm 폭, 높이 1.4 m)을 로봇 좌표로 바꿔 점 300개로 점군에 추가
            c, s = math.cos(pose.yaw), math.sin(pose.yaw)
            dx, dy = obstacle[0] - pose.x, obstacle[1] - pose.y
            bx, by = c * dx + s * dy, -s * dx + c * dy     # 월드 → 로봇 좌표 회전
            blob = np.c_[bx + rng.uniform(-0.2, 0.2, 300), by + rng.uniform(-0.2, 0.2, 300),
                         rng.uniform(0.0, 1.4, 300) - 0.1]
            pts = np.vstack([pts, blob])
        # --- lidar_tree_node 와 같은 인식 파이프라인 ---
        g = fit_ground_plane(pts, nominal_ground_z=-0.1, rng=rng)   # 지면 평면 추정
        h = g.height(pts)                                           # 각 점의 지면 위 높이
        trunks = detect_trunks(pts, h)
        xy = np.array([[k.x, k.y] for k in trunks]) if trunks else np.zeros((0, 2))
        w = np.array([k.confidence for k in trunks]) if trunks else np.zeros(0)
        raw = fit_rows(xy, w, orchard.row_spacing, *tracker.prior())   # 이번 프레임 행(좌·우 나무줄) 적합
        est = tracker.update(raw, t)                                    # 시간 평활화된 중심선
        off, head = (est.offset, est.heading) if est.valid else (0.0, 0.0)
        obs = corridor_obstacle_distance(pts, h, off, head, 0.5)
        row = RowObs(est.valid, est.offset, est.heading, est.confidence, raw.last_tree_ahead, obs)
        v, wz = mission.step(t, pose, row)      # 미션 상태기계 → 속도 명령 (v [m/s], wz [rad/s])
        # 평가: 행 추종 중(행 양 끝 1 m 제외)일 때 가장 가까운 통로 중심선과의 횡오차 [m]
        lat = None
        if mission.state == 'FOLLOW_ROW' and 1.0 < pose.x < orchard.row_length - 1.0:
            lat = min((pose.y - orchard.lane_y(k) for k in range(orchard.rows - 1)), key=abs)
        sub = rng.choice(pts.shape[0], min(2500, pts.shape[0]), replace=False)   # 그리기용 점 2500개만 표본
        frames.append(dict(t=t, pose=pose, state=mission.state, lanes=mission.lanes_done, v=v, w=wz,
                           pts=pts[sub], h=h[sub], trunks=xy, sides=raw.sides.copy(),
                           est=(est.valid, est.offset, est.heading, est.left_offset, est.right_offset,
                                est.confidence),
                           obs=obs, lat=lat, tilt=g.tilt_deg()))
        # 단순 운동학 모델(유니사이클)로 다음 자세 계산, yaw 는 -π~π 로 정규화
        pose = Pose2D(pose.x + v * math.cos(pose.yaw) * dt, pose.y + v * math.sin(pose.yaw) * dt,
                      math.atan2(math.sin(pose.yaw + wz * dt), math.cos(pose.yaw + wz * dt)))
        t += dt
    return frames, mission


def rover_poly(pose, length=1.0, width=0.6):
    """로버를 나타내는 삼각형(앞이 뾰족)의 월드 좌표 꼭짓점 3개."""
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    pts = [(length / 2, 0), (-length / 2, width / 2), (-length / 2, -width / 2)]
    return [(pose.x + c * x - s * y, pose.y + s * x + c * y) for x, y in pts]


def draw_frame(fig, orchard, fr, trail, title, obstacle=None):
    """한 프레임을 그린다: 왼쪽 = 과수원 전체(위에서 본 모습), 오른쪽 = 로봇 기준 LiDAR 점군과 중심선.

    Args:
        fig: matplotlib Figure (매번 지우고 다시 그림).
        orchard: SyntheticOrchard.
        fr: simulate() 가 만든 프레임 dict.
        trail: 지금까지의 (x, y) 궤적 목록.
        title: 그림 제목 앞부분.
        obstacle: (x, y) 장애물 위치 또는 None.
    """
    fig.clf()
    ax1 = fig.add_axes([0.04, 0.08, 0.55, 0.80])
    ax2 = fig.add_axes([0.64, 0.08, 0.34, 0.80])
    # --- 월드(위에서 본) ---
    ax1.scatter(orchard.trees[:, 0], orchard.trees[:, 1], s=28, c='#3c7a2c', edgecolors='#24461a',
                linewidths=0.5, zorder=2, label='과수(줄기)')
    for k in range(orchard.rows - 1):
        ax1.plot([0, orchard.row_length], [orchard.lane_y(k)] * 2, color='#c9b27a', lw=0.8, ls='--', zorder=1)
    if obstacle is not None:
        ax1.scatter([obstacle[0]], [obstacle[1]], s=160, marker='s', c='#2255aa', zorder=3, label='장애물(사람)')
    if trail:
        tr = np.array(trail)
        ax1.plot(tr[:, 0], tr[:, 1], color='#d9480f', lw=1.6, zorder=3, label='주행 궤적')
    p = fr['pose']
    c, s = math.cos(p.yaw), math.sin(p.yaw)
    if len(fr['trunks']):                  # 로봇 기준 줄기 검출 위치 → 월드 좌표로 변환해 표시
        wx = p.x + c * fr['trunks'][:, 0] - s * fr['trunks'][:, 1]
        wy = p.y + s * fr['trunks'][:, 0] + c * fr['trunks'][:, 1]
        ax1.scatter(wx, wy, s=70, facecolors='none', edgecolors='#f08c00', linewidths=1.4, zorder=4,
                    label='LiDAR 줄기 검출')
    ax1.add_patch(Polygon(rover_poly(p), closed=True, color='#e03131', zorder=5))
    ax1.set_xlim(-6, orchard.row_length + 8)
    ax1.set_ylim(-3.5, orchard.lane_y(orchard.rows - 2) + 3.5)
    ax1.set_aspect('equal')
    ax1.set_xlabel('x [m] (행 방향)')
    ax1.set_ylabel('y [m]')
    ax1.set_title('과수원 전체 (위에서 본 모습)', fontsize=11)
    ax1.legend(loc='upper left', fontsize=7, ncol=5, framealpha=0.9)
    # --- 로봇 기준 LiDAR ---
    pts, h = fr['pts'], fr['h']
    # 지면 위 높이로 점 색 구분: <8 cm 지면, <25 cm 잔디, 25~70 cm 줄기 밴드(trunk.band), 그 위 수관
    # 화면은 로봇 전방(x)이 위쪽, 왼쪽(y+)이 화면 왼쪽이 되도록 (-y, x) 로 그린다
    cls = np.where(h < 0.08, 0, np.where(h < 0.25, 1, np.where(h <= 0.7, 2, 3)))
    colors = np.array(['#adb5bd', '#8ce99a', '#e8590c', '#2f9e44'])
    labels = ['지면', '잔디(<25cm)', '줄기 밴드(25~70cm)', '수관']
    for i in range(4):
        m = cls == i
        ax2.scatter(-pts[m, 1], pts[m, 0], s=2 if i != 2 else 5, c=colors[i], label=labels[i], zorder=2 + i)
    valid, off, head, lo, ro, conf = fr['est']
    xs = np.array([-2.0, 12.0])
    if valid:                              # 좌·우 나무줄(파랑)과 통로 중심선(노랑): y = offset + tan(heading)·x
        b = math.tan(head)
        for o, col, lw in ((lo, '#1c7ed6', 1.2), (ro, '#1c7ed6', 1.2), (off, '#fab005', 2.5)):
            if np.isfinite(o):
                ax2.plot(-(o + b * xs), xs, color=col, lw=lw, zorder=8)
    if len(fr['trunks']):
        side = fr['sides']
        col = np.where(side > 0, '#1c7ed6', np.where(side < 0, '#1c7ed6', '#868e96'))
        ax2.scatter(-fr['trunks'][:, 1], fr['trunks'][:, 0], s=60, facecolors='none', edgecolors=col,
                    linewidths=1.3, zorder=9)
    if np.isfinite(fr['obs']):             # 통로 장애물까지 거리 (빨간 가로선)
        ax2.axhline(fr['obs'], color='#e03131', lw=2, zorder=9)
        ax2.text(3.2, fr['obs'] + 0.2, f"장애물 {fr['obs']:.1f} m", color='#e03131', fontsize=8)
    ax2.add_patch(Polygon([(0, 0.5), (-0.3, -0.5), (0.3, -0.5)], closed=True, color='#e03131', zorder=10))
    ax2.set_xlim(-5, 5)
    ax2.set_ylim(-3, 12)
    ax2.set_aspect('equal')
    ax2.set_xlabel('← 왼쪽   [m]   오른쪽 →')
    ax2.set_title('로봇 기준 Mid-360 점군 + 행간중심선', fontsize=11)
    ax2.legend(loc='lower left', fontsize=6.5, markerscale=3, framealpha=0.9)
    lat = f"{fr['lat'] * 100:+.0f} cm" if fr['lat'] is not None else '—'
    fig.suptitle(f"{title}   t={fr['t']:5.1f}s   상태: {STATE_KO.get(fr['state'], fr['state'])}   "
                 f"완료 통로 {fr['lanes']}   속도 {fr['v']:.2f} m/s   통로중심 오차 {lat}   "
                 f"지면기울기 {fr['tilt']:.1f}°", fontsize=10)


def render_video(frames, orchard, path, title, step=2, fps=15, obstacle=None):
    """step 프레임마다 PNG 로 그린 뒤 ffmpeg 로 mp4(H.264)를 만든다.

    Returns:
        (프레임 PNG 폴더, 저장한 프레임 수).
    """
    tmp = path + '_frames'
    os.makedirs(tmp, exist_ok=True)
    fig = plt.figure(figsize=(12.8, 5.6), dpi=100)
    trail = []
    n = 0
    for i, fr in enumerate(frames):
        trail.append((fr['pose'].x, fr['pose'].y))
        if i % step:
            continue
        draw_frame(fig, orchard, fr, trail, title, obstacle)
        fig.savefig(os.path.join(tmp, f'{n:05d}.png'))
        n += 1
    plt.close(fig)
    # 00000.png, 00001.png ... → mp4. yuv420p 는 대부분의 플레이어 호환, crf 28 = 적당한 압축
    subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-framerate', str(fps), '-i', os.path.join(tmp, '%05d.png'),
                    '-vf', 'scale=1280:-2', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '28', path], check=True)
    return tmp, n


def main():
    """행 추종 데모(4열·3통로)와 장애물 정지 데모(2열·1통로)를 만들고 영상·사진·통계를 저장한다."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(ROOT, 'media'))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    # --- 데모 1: 4열(3통로) × 20 m 과수원을 3통로 주행 ---
    orchard = SyntheticOrchard(rows=4, row_length=20.0, seed=2)
    frames, mission = simulate(orchard, lanes=3, max_time=300.0)
    lat = np.array([f['lat'] for f in frames if f['lat'] is not None])
    print(f'주행: {mission.state} 통로 {mission.lanes_done}, {frames[-1]["t"]:.1f}s, '
          f'횡오차 RMS {np.sqrt(np.mean(lat ** 2)) * 100:.1f} cm 최대 {np.abs(lat).max() * 100:.1f} cm')
    tmp, n = render_video(frames, orchard, os.path.join(a.out, 'demo_row_following.mp4'),
                          '과수원 3통로 자율주행 (알고리즘 폐루프)')
    # 대표 사진: 행 추종 중 한 장
    #   x > 8 m 인 첫 FOLLOW_ROW 프레임. render_video 가 step=2 로 저장했으므로 PNG 번호는 // 2
    k = next(i for i, f in enumerate(frames) if f['state'] == 'FOLLOW_ROW' and f['pose'].x > 8) // 2
    os.replace(os.path.join(tmp, f'{k:05d}.png'), os.path.join(a.out, 'photo_perception.png'))
    fig = plt.figure(figsize=(12.8, 5.6), dpi=100)
    draw_frame(fig, orchard, frames[-1], [(f['pose'].x, f['pose'].y) for f in frames], '3통로 주행 완료 궤적')
    fig.savefig(os.path.join(a.out, 'photo_full_path.png'))
    plt.close(fig)

    # --- 데모 2: 첫 통로 중앙 x = 8 m 에 장애물(사람) → 앞에서 정지해야 함 ---
    ob = (8.0, 0.0)
    orchard2 = SyntheticOrchard(rows=2, row_length=20.0, seed=3)
    frames2, m2 = simulate(orchard2, lanes=1, max_time=25.0, obstacle=ob)
    print(f'장애물: 최종 x={frames2[-1]["pose"].x:.2f} (장애물 x={ob[0]}), 거리 {frames2[-1]["obs"]:.2f} m')
    render_video(frames2, orchard2, os.path.join(a.out, 'demo_obstacle_stop.mp4'), '통로 안 장애물 정지',
                 obstacle=ob)
    # 수치 요약 (문서·회귀 비교용)
    with open(os.path.join(a.out, 'stats.txt'), 'w', encoding='utf-8') as f:
        f.write(f'lanes={mission.lanes_done} state={mission.state} time={frames[-1]["t"]:.1f}\n')
        f.write(f'lat_rms_cm={np.sqrt(np.mean(lat ** 2)) * 100:.1f} lat_max_cm={np.abs(lat).max() * 100:.1f}\n')
        stop_x = frames2[-1]['pose'].x
        f.write(f'obstacle_stop_x={stop_x:.2f} obstacle_x={ob[0]} gap={ob[0] - stop_x:.2f}\n')


if __name__ == '__main__':
    main()
