"""PX4 rover_ackermann Gazebo 모델에 Livox Mid-360 / RealSense D455 센서를 붙여
orchard_rover 모델을 생성한다.

PX4 모델 파일을 복사하지 않고 실행 시점에 읽어서 변환하므로
PX4 버전을 올려도 차체·구동계는 PX4 쪽 정의를 그대로 따라간다.
PX4 gz_bridge 가 찾는 IMU/GPS/지자기/기압 센서(base_link 아래)는 그대로 유지된다.

입력
  - PX4 모델: <PX4_DIR>/Tools/simulation/gz/models/rover_ackermann/model.sdf (차체·바퀴·조향·PX4 센서)
  - orchard_description/config/rover.yaml 의 sensors.livox / sensors.camera
    (장착 위치 xyz [m]·rpy [rad] 는 base_link 기준 FLU, 시뮬 센서 사양은 각 sim: 절)
생성 파일
  <out_models_dir>/<name>/model.sdf, model.config  (sim.launch.py 가 GZ_SIM_RESOURCE_PATH 에 추가)
Gazebo 토픽 (ros_gz_bridge 설정: orchard_gazebo/config/bridge.yaml 이 실차와 같은 ROS 이름으로 바꿈)
  /livox/lidar/points (gpu_lidar), /camera/image·/camera/depth_image·/camera/camera_info (rgbd_camera),
  /camera/segmentation/* (seg_camera 켰을 때), /model/<name>/odometry (정답 위치 → /ground_truth/odom)
바꿀 곳
  센서 위치·해상도·주기·잡음 → rover.yaml (URDF 와 공유되므로 TF 도 함께 바뀜).
  카메라 저사양 모드 → sim.launch.py camera_lite:=true, 세그멘테이션 카메라 → seg_camera:=true.
참고: SDFormat 명세(sdformat.org, <sensor>/<lidar>/<camera>), Gazebo Harmonic 시스템 플러그인 문서.
"""
from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET

import yaml


def _pose(xyz, rpy) -> str:
    """SDF <pose> 문자열 'x y z roll pitch yaw' ([m], [rad])."""
    return ' '.join(f'{v:.6g}' for v in list(xyz) + list(rpy))


def _sub(parent, tag, text=None, **attrib):
    """parent 아래에 XML 자식 요소 <tag attrib...>text</tag> 를 만들어 반환."""
    el = ET.SubElement(parent, tag, attrib)
    if text is not None:
        el.text = str(text)
    return el


def _small_inertial(link, mass=0.2):
    """센서 링크용 작은 관성 (질량 [kg], 관성모멘트 1e-4 kg·m² 대각). 고정 관절이라 값은 물리에 거의 영향 없음."""
    inertial = _sub(link, 'inertial')
    _sub(inertial, 'mass', mass)
    inertia = _sub(inertial, 'inertia')
    for k in ('ixx', 'iyy', 'izz'):
        _sub(inertia, k, 1e-4)
    for k in ('ixy', 'ixz', 'iyz'):
        _sub(inertia, k, 0)


def _fixed_joint(model, name, child):
    """base_link 에 child 링크를 고정하는 관절."""
    j = _sub(model, 'joint', name=name, type='fixed')
    _sub(j, 'parent', 'base_link')
    _sub(j, 'child', child)


def add_livox(model, cfg):
    """Livox Mid-360 을 흉내 낸 gpu_lidar 센서 링크 추가.

    cfg: rover.yaml 의 sensors.livox (frame, xyz, rpy, sim: rate [Hz], horizontal/vertical_samples,
    vertical_min/max_deg [deg], range_min/max [m], noise_stddev [m]). 수평 360°, 수직 −7°~+52°(기본값).
    Mid-360 의 비반복 스캔 패턴은 흉내 내지 않고 균일 격자로 근사한다.
    """
    s = cfg['sim']
    link = _sub(model, 'link', name='livox_link')
    _sub(link, 'pose', _pose(cfg['xyz'], cfg['rpy']), relative_to='base_link')
    _small_inertial(link, 0.265)                  # Mid-360 무게 약 265 g
    vis = _sub(link, 'visual', name='visual')
    geo = _sub(_sub(vis, 'geometry'), 'cylinder')
    _sub(geo, 'radius', 0.0325)                   # 보이기용 원통 (외형 약 65 mm)
    _sub(geo, 'length', 0.065)
    mat = _sub(vis, 'material')
    _sub(mat, 'diffuse', '0.1 0.1 0.1 1')
    _sub(mat, 'ambient', '0.1 0.1 0.1 1')

    sensor = _sub(link, 'sensor', name='livox_mid360', type='gpu_lidar')
    _sub(sensor, 'gz_frame_id', cfg['frame'])     # 발행 메시지의 frame_id (URDF 의 livox_frame 과 같게)
    _sub(sensor, 'topic', '/livox/lidar')
    _sub(sensor, 'update_rate', s['rate'])
    _sub(sensor, 'always_on', 1)
    _sub(sensor, 'visualize', 'false')
    lidar = _sub(sensor, 'lidar')
    scan = _sub(lidar, 'scan')
    h = _sub(scan, 'horizontal')
    _sub(h, 'samples', s['horizontal_samples'])
    _sub(h, 'resolution', 1)
    _sub(h, 'min_angle', f'{-math.pi:.6f}')       # 수평 −π ~ +π [rad] = 360°
    _sub(h, 'max_angle', f'{math.pi:.6f}')
    v = _sub(scan, 'vertical')
    _sub(v, 'samples', s['vertical_samples'])
    _sub(v, 'resolution', 1)
    _sub(v, 'min_angle', f"{math.radians(s['vertical_min_deg']):.6f}")
    _sub(v, 'max_angle', f"{math.radians(s['vertical_max_deg']):.6f}")
    rng = _sub(lidar, 'range')
    _sub(rng, 'min', s['range_min'])
    _sub(rng, 'max', s['range_max'])
    _sub(rng, 'resolution', 0.01)                 # 거리 분해능 [m]
    noise = _sub(lidar, 'noise')
    _sub(noise, 'type', 'gaussian')
    _sub(noise, 'mean', 0.0)
    _sub(noise, 'stddev', s['noise_stddev'])
    _fixed_joint(model, 'livox_joint', 'livox_link')


def add_camera(model, cfg):
    """RealSense D455 를 흉내 낸 rgbd_camera(RGB + 깊이) 센서 링크 추가.

    cfg: rover.yaml 의 sensors.camera (optical_frame, xyz, rpy, sim: rate [Hz], width/height [px],
    hfov_deg [deg], near/far [m], depth_near/depth_far [m]). RGB 와 깊이가 같은 광학 중심을 쓴다
    (실차의 aligned_depth_to_color 와 같은 형태).
    """
    s = cfg['sim']
    link = _sub(model, 'link', name='camera_link')
    _sub(link, 'pose', _pose(cfg['xyz'], cfg['rpy']), relative_to='base_link')
    _small_inertial(link, 0.39)                   # D455 무게 약 390 g
    vis = _sub(link, 'visual', name='visual')
    geo = _sub(_sub(vis, 'geometry'), 'box')
    _sub(geo, 'size', '0.029 0.124 0.029')        # 보이기용 상자 [m] (폭 124 mm)
    mat = _sub(vis, 'material')
    _sub(mat, 'diffuse', '0.25 0.25 0.25 1')
    _sub(mat, 'ambient', '0.25 0.25 0.25 1')

    sensor = _sub(link, 'sensor', name='realsense_d455', type='rgbd_camera')
    _sub(sensor, 'gz_frame_id', cfg['optical_frame'])   # 이미지 frame_id = 광학 프레임 (REP-103 _optical: z 앞)
    _sub(sensor, 'topic', '/camera')
    _sub(sensor, 'update_rate', s['rate'])
    _sub(sensor, 'always_on', 1)
    cam = _sub(sensor, 'camera')
    _sub(cam, 'horizontal_fov', f"{math.radians(s['hfov_deg']):.6f}")
    img = _sub(cam, 'image')
    _sub(img, 'width', s['width'])
    _sub(img, 'height', s['height'])
    _sub(img, 'format', 'R8G8B8')
    clip = _sub(cam, 'clip')
    _sub(clip, 'near', s['near'])
    _sub(clip, 'far', s['far'])
    noise = _sub(cam, 'noise')
    _sub(noise, 'type', 'gaussian')
    _sub(noise, 'mean', 0.0)
    _sub(noise, 'stddev', 0.007)                  # 픽셀 값(0~1) 가우시안 잡음 표준편차
    depth = _sub(_sub(cam, 'depth_camera'), 'clip')
    _sub(depth, 'near', s['depth_near'])
    _sub(depth, 'far', s['depth_far'])
    _fixed_joint(model, 'camera_joint', 'camera_link')


def add_segmentation_camera(model, cfg, rate: float = 5.0):
    """RGB 카메라와 같은 위치·화각의 Gazebo 세그멘테이션 카메라 (학습 정답 라벨 이미지).
    발행: /camera/segmentation/labels_map (픽셀값 = 라벨 번호), /camera/segmentation/colored_map

    라벨 번호는 world_gen.LABELS (각 visual 의 gz-sim-label-system 플러그인) 에서 온다.
    add_camera 가 먼저 불려 camera_link 가 있어야 한다. rate [Hz] 는 렌더링 부하 때문에 RGB 보다 낮게 둔다.
    """
    s = cfg['sim']
    link = model.find("link[@name='camera_link']")
    sensor = _sub(link, 'sensor', name='realsense_seg', type='segmentation')
    _sub(sensor, 'gz_frame_id', cfg['optical_frame'])
    _sub(sensor, 'topic', '/camera/segmentation')
    _sub(sensor, 'update_rate', rate)
    _sub(sensor, 'always_on', 1)
    cam = _sub(sensor, 'camera')
    _sub(cam, 'segmentation_type', 'semantic')
    _sub(cam, 'horizontal_fov', f"{math.radians(s['hfov_deg']):.6f}")
    img = _sub(cam, 'image')
    _sub(img, 'width', s['width'])
    _sub(img, 'height', s['height'])
    clip = _sub(cam, 'clip')
    _sub(clip, 'near', s['near'])
    _sub(clip, 'far', s['far'])


def add_ground_truth(model, name):
    """Gazebo OdometryPublisher 로 정답 위치(월드 좌표, 20 Hz) 발행 → /ground_truth/odom (평가용, 시뮬 전용)."""
    plugin = _sub(model, 'plugin', filename='gz-sim-odometry-publisher-system',
                  name='gz::sim::systems::OdometryPublisher')
    _sub(plugin, 'odom_frame', 'world')
    _sub(plugin, 'robot_base_frame', name)
    _sub(plugin, 'odom_publish_frequency', 20)
    _sub(plugin, 'odom_topic', f'/model/{name}/odometry')
    _sub(plugin, 'dimensions', 3)                 # 3 = x, y, z 와 롤·피치·요 모두 (2 면 평면 운동만)


def stabilize_steering(model, rate_max: float = 5.0) -> int:
    """PX4 rover_ackermann 조향 관절 제어기를 속도 명령 방식으로 바꾼다.

    원본은 힘(PID) 방식이라 좌/우 조향 링크(관성 매우 작음)가 4 ms 스텝에서 발산했다
    (조향 관절 속도가 수천 rad/s 로 튀어 바퀴가 제자리에서 돌며 차체가 명령 없이 미끄러짐).
    속도 명령(use_velocity_commands)은 물리엔진이 암묵적으로 풀어 안정하다. 바뀐 제어기 수 반환.
    rate_max: 조향 관절 속도 명령 한계 [rad/s] (cmd_max/cmd_min 으로 들어감).
    """
    n = 0
    # 조향 관절(이름에 'steering')의 JointPositionController 만 바꾸고 바퀴 구동 제어기는 그대로 둔다
    for pl in model.findall('plugin'):
        if 'joint-position-controller' not in (pl.get('filename') or ''):
            continue
        jn = pl.find('joint_name')
        if jn is None or 'steering' not in (jn.text or ''):
            continue
        for tag, val in (('use_velocity_commands', 'true'), ('cmd_max', str(rate_max)), ('cmd_min', str(-rate_max))):
            el = pl.find(tag)
            if el is None:
                el = ET.SubElement(pl, tag)
            el.text = val
        n += 1
    return n


def generate_model(px4_model_sdf: str, rover_yaml: str, out_models_dir: str,
                   name: str = 'orchard_rover', segmentation: bool = False, camera_lite: bool = False) -> str:
    """out_models_dir/<name>/model.sdf, model.config 생성 후 model.sdf 경로 반환.

    px4_model_sdf: PX4 rover_ackermann model.sdf, rover_yaml: 센서 설정, name: Gazebo 모델 이름
    (world_gen 의 <include> 와 PX4_GZ_MODEL_NAME 이 같은 이름을 써야 PX4 가 붙는다).
    segmentation: 세그멘테이션 카메라 추가, camera_lite: 카메라 해상도·주기 낮춤.
    """
    with open(rover_yaml, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    if camera_lite:          # 느린 PC(소프트웨어 렌더링): 카메라 320×240, 10 Hz (LiDAR 는 그대로)
        cs = cfg['sensors']['camera']['sim']
        cs.update(width=320, height=240, rate=min(cs['rate'], 10))
    tree = ET.parse(px4_model_sdf)
    root = tree.getroot()
    model = root.find('model')
    if model is None:
        raise ValueError(f'{px4_model_sdf} 에 <model> 이 없습니다')
    model.set('name', name)                   # PX4 모델 이름을 orchard_rover 로 바꿔 우리 모델로 만든다
    if model.find("link[@name='base_link']") is None:
        raise ValueError('PX4 모델에 base_link 가 없습니다 (PX4 버전 확인)')

    stabilize_steering(model)
    add_livox(model, cfg['sensors']['livox'])
    add_camera(model, cfg['sensors']['camera'])
    if segmentation:
        add_segmentation_camera(model, cfg['sensors']['camera'])
    add_ground_truth(model, name)

    out_dir = os.path.join(out_models_dir, name)
    os.makedirs(out_dir, exist_ok=True)
    ET.indent(tree, space='  ')                # 사람이 읽기 좋게 들여쓰기 (Python 3.9+)
    sdf_path = os.path.join(out_dir, 'model.sdf')
    tree.write(sdf_path, encoding='utf-8', xml_declaration=True)
    with open(os.path.join(out_dir, 'model.config'), 'w', encoding='utf-8') as f:
        f.write(f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>PX4 rover_ackermann + Livox Mid-360 + RealSense D455 (자동 생성 파일, 직접 수정하지 말 것)</description>
</model>
""")
    return sdf_path
