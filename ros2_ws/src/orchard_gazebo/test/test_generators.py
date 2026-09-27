"""orchard_gazebo 월드·모델 생성기 테스트 (ROS·Gazebo 없이 실행).

작은 과수원(4열, 12 m, lite)을 임시 폴더에 만들어 SDF 구조, 지형·나무 배치, 지면 구역, 캐시,
로버 모델 센서 추가, 세그멘테이션 라벨을 확인한다. PX4 모델은 FAKE_PX4_MODEL 로 흉내 낸다.
"""
import json
import os
import xml.etree.ElementTree as ET

import numpy as np

from orchard_gazebo.model_gen import generate_model
from orchard_gazebo.world_gen import OrchardConfig, generate_world, ground_for

HERE = os.path.dirname(os.path.abspath(__file__))
ROVER_YAML = os.path.join(HERE, '..', '..', 'orchard_description', 'config', 'rover.yaml')

# PX4 rover_ackermann 모델의 최소 흉내: base_link 의 PX4 센서 2개 + 조향/바퀴 관절 제어기 각 1개
FAKE_PX4_MODEL = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="rover_ackermann">
    <link name="base_link">
      <sensor name="imu_sensor" type="imu"/>
      <sensor name="navsat_sensor" type="navsat"/>
    </link>
    <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
      <joint_name>wheel_front_left_steering_joint</joint_name>
      <sub_topic>servo_0</sub_topic>
      <p_gain>1</p_gain>
    </plugin>
    <plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController">
      <joint_name>wheel_rear_left_joint</joint_name>
    </plugin>
  </model>
</sdf>
"""


def _small_cfg(**kw):
    """테스트용 작은 설정 (4열, 12 m, 장애물 2, lite, 숲 20그루). kw 로 필드를 덮어쓴다."""
    cfg = OrchardConfig(rows=4, row_length=12.0, obstacles=2, seed=1, **kw).apply_lite()
    cfg.forest_trees = 20
    return cfg


def test_world_layout(tmp_path):
    """월드 SDF 의 모델 구성, 통로 중심 y, 나무 위치, 참조 파일 존재 여부."""
    cfg = _small_cfg()
    world, meta = generate_world(cfg, str(tmp_path), log=lambda *a: None)
    root = ET.parse(world).getroot()
    w = root.find('world')
    assert w.get('name') == 'orchard'
    names = [m.get('name') for m in w.findall('model')]
    assert sum(n.startswith('tree_row_') for n in names) == 4
    assert sum(n.startswith('obstacle_') for n in names) == 2
    assert 'terrain' in names and 'ground_cover' in names
    inc = w.find('include')
    assert inc.find('name').text == 'orchard_rover'
    m = json.load(open(meta))
    assert m['lane_centers_y'] == [0.0, 3.8, 7.6]
    for t in m['trees']:                       # 모든 나무는 자기 열 y 근처
        assert abs(t['y'] - m['row_y'][t['row']]) < 0.1
    assert len(m['trees']) > 4 * 9             # 주간 1.2 m, 결주 3% → 열당 대략 10그루
    # SDF 가 가리키는 메시·텍스처 파일이 모두 있어야 한다
    for uri in root.iter('uri'):
        if uri.text.startswith('/'):
            assert os.path.isfile(uri.text), uri.text
    for tex in root.iter('albedo_map'):
        assert os.path.isfile(tex.text), tex.text


def test_terrain_is_uneven_and_trees_stand_on_it(tmp_path):
    """지형 기복·최대 경사, 나무·출발 지점이 지면 높이에 놓였는지, 두둑이 통로보다 높은지."""
    cfg = _small_cfg()
    _, meta = generate_world(cfg, str(tmp_path), log=lambda *a: None)
    m = json.load(open(meta))
    g = ground_for(cfg)
    xs = np.linspace(0, cfg.row_length, 200)
    lane = g.height(xs, np.zeros_like(xs))
    assert np.ptp(lane) > 0.08                  # 통로를 따라 높낮이 변화가 있어야 한다
    slope = np.degrees(np.arctan(np.abs(np.diff(lane)) / np.diff(xs)))
    assert slope.max() < 12.0                   # 로버가 오를 수 있는 경사
    for t in m['trees']:
        assert abs(t['z'] - float(g.height(t['x'], t['y']))) < 1e-3
    # 두둑: 나무 열이 통로 중심보다 높다 (평균)
    ys_row = np.full_like(xs, cfg.row_y(1))
    ys_lane = np.full_like(xs, cfg.lane_y(1))
    assert np.mean(g.height(xs, ys_row) - g.height(xs, ys_lane)) > 0.03
    # 스폰 지점 높이 = 지형 높이
    assert abs(m['spawn'][2] - float(g.height(-cfg.spawn_back, 0.0))) < 1e-6


def test_ground_zones_are_irregular(tmp_path):
    """지면 구역 비율이 그럴듯한지 (풀 위주 + 자갈·맨흙·제초 띠가 일정 비율 이상)."""
    cfg = _small_cfg()
    g = ground_for(cfg)
    rng = np.random.default_rng(0)
    x = rng.uniform(0, cfg.row_length, 20000)
    y = rng.uniform(cfg.row_y(0), cfg.row_y(cfg.rows - 1), 20000)
    z = g.zones(x, y)
    assert 0.3 < np.mean(z['grass']) < 0.9      # 풀밭이지만 군데군데 맨흙·자갈
    assert np.mean(z['gravel'] > 0.5) > 0.005
    assert np.mean(z['bare'] > 0.5) > 0.01
    assert np.mean(z['strip'] > 0.5) > 0.1      # 나무 밑 제초 띠


def test_assets_are_cached(tmp_path):
    """같은 설정은 캐시를 다시 쓰고, 메시에 영향 주는 설정(relief)이 바뀌면 새로 만든다."""
    cfg = _small_cfg()
    generate_world(cfg, str(tmp_path), log=lambda *a: None)
    calls = []
    generate_world(cfg, str(tmp_path), log=calls.append)
    assert calls == []                          # 두 번째는 다시 만들지 않는다
    cfg2 = _small_cfg(relief=0.1)
    generate_world(cfg2, str(tmp_path), log=calls.append)
    assert len(calls) == 1                      # 설정이 바뀌면 새로 만든다


def test_model_adds_sensors_and_keeps_px4_sensors(tmp_path):
    """로버 모델: PX4 센서 유지 + LiDAR·카메라 추가, frame_id·토픽, 조향 제어기만 속도 명령 방식."""
    px4 = tmp_path / 'model.sdf'
    px4.write_text(FAKE_PX4_MODEL)
    sdf = generate_model(str(px4), ROVER_YAML, str(tmp_path / 'models'))
    root = ET.parse(sdf).getroot()
    model = root.find('model')
    assert model.get('name') == 'orchard_rover'
    sensors = {s.get('name'): s for s in model.iter('sensor')}
    assert {'imu_sensor', 'navsat_sensor', 'livox_mid360', 'realsense_d455'} <= set(sensors)
    lidar = sensors['livox_mid360']
    assert lidar.get('type') == 'gpu_lidar'
    assert lidar.find('gz_frame_id').text == 'livox_frame'
    assert lidar.find('topic').text == '/livox/lidar'
    cam = sensors['realsense_d455']
    assert cam.find('gz_frame_id').text == 'camera_color_optical_frame'
    joints = {j.get('name') for j in model.findall('joint')}
    assert {'livox_joint', 'camera_joint'} <= joints
    assert os.path.isfile(os.path.join(os.path.dirname(sdf), 'model.config'))
    # 조향 제어기는 속도 명령 방식으로 (바퀴 속도 제어기는 그대로)
    joints_pl = [p for p in model.findall('plugin') if p.find('joint_name') is not None]
    steer = [p for p in joints_pl if 'steering' in p.find('joint_name').text]
    assert steer and all(p.find('use_velocity_commands').text == 'true' for p in steer)
    wheel = [p for p in joints_pl if 'rear_left' in p.find('joint_name').text]
    assert wheel[0].find('use_velocity_commands') is None


def test_segmentation_labels(tmp_path):
    """모든 보이는 메시에 세그멘테이션 라벨이 붙고, 라벨 번호가 인식 쪽 클래스 정의와 같다."""
    from orchard_gazebo.world_gen import LABELS
    cfg = _small_cfg()
    world, _ = generate_world(cfg, str(tmp_path), log=lambda *a: None)
    root = ET.parse(world).getroot()
    labels = {}
    for vis in root.iter('visual'):
        if vis.find('geometry/mesh') is None:
            continue
        pl = vis.find('plugin')
        assert pl is not None and pl.get('filename') == 'gz-sim-label-system', vis.get('name')
        labels[vis.get('name')] = int(pl.find('label').text)
    assert labels['terrain_drive'] == LABELS['drivable'] and labels['terrain_strip'] == LABELS['tree_strip']
    assert labels['bark'] == labels['leaf_a'] == LABELS['tree'] and labels['post'] == LABELS['structure']
    try:
        from orchard_perception.seg_classes import CLASSES
    except ImportError:
        return
    assert [LABELS[c] for c in CLASSES] == list(range(len(CLASSES)))
