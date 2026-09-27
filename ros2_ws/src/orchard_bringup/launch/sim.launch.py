"""과수원 로버 시뮬레이션 전체 실행 (명령 하나로 모두 뜬다).

  ros2 launch orchard_bringup sim.launch.py                 # GUI + RViz
  ros2 launch orchard_bringup sim.launch.py headless:=true  # 화면 없이 (서버/CI)
  ros2 launch orchard_bringup sim.launch.py rows:=8 obstacles:=1 lite:=true
  ros2 launch orchard_bringup sim.launch.py relief:=0 bump_amp:=0     # 평탄한 과수원(비교용)
  ros2 launch orchard_bringup sim.launch.py explore:=true              # 탐사: 끝 열까지 + 나무 지도
  ros2 launch orchard_bringup sim.launch.py scenario:=uturn            # 구간 시험 (follow|uturn|edge|obstacle|approach)
  ros2 launch orchard_bringup sim.launch.py record_seg:=true           # 세그멘테이션 학습 데이터 수집

실행 순서
  1) 과수원 월드 + 로버 모델 생성 (~/.cache/orchard_rover)
  2) Gazebo Harmonic 서버(+GUI)
  3) PX4 SITL (에어프레임 51010, Gazebo 에 붙는 standalone 모드)
  4) Micro XRCE-DDS Agent (PX4 ↔ ROS 2)
  5) ros_gz_bridge, robot_state_publisher, 인식/주행/PX4 브리지 노드, RViz

언제 쓰나
  sim 컨테이너(./scripts/docker_run.sh sim) 안에서 ./scripts/build_ws.sh 로 빌드한 뒤 실행한다.
  (컨테이너 셸에서는 별칭 'sim' = ros2 launch orchard_bringup sim.launch.py)

launch 인자 (자주 바꾸는 것)
  화면/실행 환경
    px4_dir             PX4-Autopilot 소스 경로 (기본: 환경변수 PX4_DIR 또는 /opt/PX4-Autopilot)
    cache_dir           생성한 월드·모델을 저장할 폴더 (기본 ~/.cache/orchard_rover)
    headless            'true' 면 Gazebo GUI·RViz 없이 서버만 (서버/CI)
    rviz                'false' 면 RViz 를 띄우지 않음
    headless_rendering  GUI 는 띄우되 센서 렌더링은 EGL(GPU, 화면 없음)로
    foxglove            'true' 면 foxglove_bridge(ws://localhost:8765) 실행 (Mac/Windows 에서 보기)
    gz_verbose          Gazebo 로그 상세도 0~4
    lite / camera_lite  저사양 PC 용: 잔디·열매 축소 / 카메라 320x240 10 Hz
  과수원 월드 (orchard_gazebo.generate 로 매번 새로 생성)
    rows, row_spacing[m], row_length[m]  열 수, 행간 거리, 행 길이
    obstacles           통로에 넣을 장애물(사람 등) 개수
    seed                나무 배치·지형 난수 시드 (같은 값이면 같은 월드)
    relief, bump_amp    지형 기복 진폭 / 지면 요철 진폭 [m]. 0 이면 평탄
    spawn_x, spawn_lane, spawn_yaw  로버 출발 위치 x[m], 통로 번호, 방향[rad]
    spawn_y             출발 y [m] (주면 spawn_lane 무시 — 과수원 밖 출발 시험)
    approach            'true' 면 과수원 밖에서 입구를 찾아 들어감 (orchard_planning/approach.py)
  미션
    lanes               주행할 통로 수 (비우면 min(3, rows-1))
    explore             'true' 면 탐사 모드(lanes=0): 끝 열까지 순회 + 나무 지도 저장
    first_turn          첫 U턴 방향 left/right (비우면 sim.yaml)
    scenario            full | follow | uturn | edge | obstacle | approach — 아래 SCENARIOS 프리셋으로 짧은 시험
    nav_mode            planner (기본: 전역 통로 순회 + Hybrid A* U턴 + DWA 지역 계획) | row (예전 행 추종)
  인식/데이터
    camera_model        YOLO 줄기 검출 ONNX (비우면 카메라 검출 끔)
    seg_model           세그멘테이션 ONNX (비우면 sim.yaml 값)
    seg_camera          Gazebo 세그멘테이션 정답 카메라 켜기
    seg_gt              모델 대신 정답 라벨로 카메라 중심선 계산
    record_dataset      YOLO 자동 라벨링 데이터 저장
    record_seg          세그멘테이션 학습 데이터 저장 (data/seg_dataset_sim)

결과
  RViz/Gazebo 화면, 터미널 로그(lane_error_monitor 가 통로 중심선 횡오차 출력),
  explore 모드면 /workspace/data/maps 에 나무 지도 저장.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _truthy(value: str) -> bool:
    """launch 인자 문자열을 bool 로 해석한다 ('1', 'true', 'yes', 'on' → True, 대소문자 무시)."""
    return str(value).lower() in ('1', 'true', 'yes', 'on')


# 구간 시험(scenario): 전체 과수원을 다 돌지 않고 보고 싶은 동작만 짧게 (맥 등 느린 PC 에서 수 분)
#   인자를 따로 주면 그 값이 우선한다 (기본값 그대로일 때만 아래 값으로 바뀜)
SCENARIOS = {
    'follow': dict(rows='2', row_length='15.0', lanes='1'),                    # 통로 하나 따라가기
    'uturn': dict(rows='3', row_length='12.0', lanes='2', spawn_x='5.0'),      # 행 끝 → U턴 → 다음 통로
    'edge': dict(rows='3', row_length='12.0', explore='true', spawn_lane='1',  # 탐사: 끝 열 판정 후 정지
                 spawn_x='6.0', first_turn='left'),
    'obstacle': dict(rows='2', row_length='15.0', lanes='1', obstacles='1'),   # 통로 장애물 비켜 가기
    'approach': dict(rows='3', row_length='12.0', lanes='1', spawn_x='-9.0',   # 과수원 밖(입구 9 m 앞, 오른쪽 3.5 m,
                     spawn_y='-3.5', spawn_yaw='0.35', approach='true'),       # 20° 비스듬히)에서 입구 찾아 통로 0 진입
}
# 아래 DEFAULTS 는 generate_launch_description() 의 default_value 와 같아야 한다.
# 사용자가 인자를 따로 줬는지(=기본값과 다른지) 판단하는 데 쓰인다.
DEFAULTS = dict(rows='6', row_length='30.0', lanes='', explore='false', spawn_x='', spawn_lane='0',
                spawn_yaw='0.0', first_turn='', obstacles='0', spawn_y='', approach='false')


def _setup(context, *args, **kwargs):
    """월드 생성 → Gazebo·PX4·XRCE Agent·브리지·자율주행 노드 실행 액션 목록을 만든다.

    OpaqueFunction 안에서 실행되므로 launch 인자 값을 파이썬 문자열로 다룰 수 있다.
    월드(SDF)는 실행할 때마다 인자에 맞게 cache_dir 아래에 새로 생성된다.
    """
    # ROS 패키지(orchard_gazebo)가 빌드·source 된 뒤에만 import 가능하므로 함수 안에서 import
    from orchard_gazebo.generate import generate_all

    # raw('이름') → 사용자가 준 launch 인자 값 그대로
    raw = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    preset = SCENARIOS.get(raw('scenario'), {}) if raw('scenario') not in ('', 'full') else {}
    if raw('scenario') not in ('', 'full') and not preset:
        raise RuntimeError(f"scenario 는 {', '.join(['full'] + list(SCENARIOS))} 중 하나")

    def lc(name):
        """scenario 프리셋을 반영한 인자 값. 사용자가 기본값을 그대로 두었을 때만 프리셋 값으로 바꾼다."""
        v = raw(name)
        if name in preset and v == DEFAULTS.get(name, v):
            return preset[name]
        return v
    px4_dir = lc('px4_dir')
    cache = os.path.expanduser(lc('cache_dir'))
    # 과수원 월드 생성기(orchard_gazebo/generate.py)에 넘길 명령행 인자
    argv = ['--out-dir', os.path.join(cache, 'worlds'), '--rows', lc('rows'),
            '--row-spacing', lc('row_spacing'), '--row-length', lc('row_length'),
            '--obstacles', lc('obstacles'), '--seed', lc('seed'),
            '--relief', lc('relief'), '--bump-amp', lc('bump_amp'),
            '--spawn-lane', lc('spawn_lane'), '--spawn-yaw', lc('spawn_yaw')]
    if lc('spawn_x'):
        argv += ['--spawn-x', lc('spawn_x')]
    if lc('spawn_y'):
        argv += ['--spawn-y', lc('spawn_y')]
    if _truthy(lc('lite')):
        argv.append('--lite')
    # 차체는 PX4 가 제공하는 rover_ackermann Gazebo 모델을 바탕으로, 우리 센서(LiDAR·카메라)를 붙여 새 모델을 만든다
    px4_model = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'models', 'rover_ackermann', 'model.sdf')
    if not os.path.isfile(px4_model):
        raise RuntimeError(f'PX4 로버 모델이 없습니다: {px4_model}\n'
                           f'  cd {px4_dir} && git submodule update --init Tools/simulation/gz')
    # 세그멘테이션 정답 카메라는 정답 라벨이 필요한 기능(record_seg, seg_gt) 중 하나라도 켜지면 자동으로 켠다
    seg_camera = _truthy(lc('seg_camera')) or _truthy(lc('record_seg')) or _truthy(lc('seg_gt'))
    world, meta, _model, models_dir = generate_all(argv, px4_model=px4_model, seg_camera=seg_camera,
                                                   camera_lite=_truthy(lc('camera_lite')))

    # Gazebo 가 model:// 경로를 찾을 폴더: 생성한 모델 → PX4 모델 → 기존 GZ_SIM_RESOURCE_PATH 순
    px4_models = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'models')
    resource = ':'.join(p for p in [models_dir, px4_models, os.environ.get('GZ_SIM_RESOURCE_PATH', '')] if p)
    headless = _truthy(lc('headless'))
    # 주행할 통로 수: 지정하지 않으면 min(3, 열 수 - 1)
    lanes = lc('lanes') or str(max(1, min(3, int(lc('rows')) - 1)))
    if _truthy(lc('explore')):
        lanes = '0'                  # 탐사 모드: 통로 수를 모르는 상태로 끝 열까지 순회
    use_sim_time = {'use_sim_time': True}
    bringup = get_package_share_directory('orchard_bringup')
    params = os.path.join(bringup, 'config', 'sim.yaml')

    # Gazebo 서버: -r 바로 시뮬 시작(일시정지 아님), -s 서버만(GUI 는 아래에서 따로), -v 로그 상세도
    gz_server = ['gz', 'sim', '-r', '-s', '-v', lc('gz_verbose'), world]
    if headless or _truthy(lc('headless_rendering')):
        gz_server.insert(3, '--headless-rendering')     # 센서 렌더링을 화면 없이 GPU(EGL)로

    # PX4 SITL 실행 파일 (docker/scripts/install_sim.sh 에서 make px4_sitl_default 로 빌드됨)
    px4_bin = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'bin', 'px4')
    # PX4 SITL 환경변수
    #   PX4_GZ_STANDALONE=1   PX4 가 Gazebo 를 직접 띄우지 않고, 이미 떠 있는 월드에 붙는다
    #   PX4_SYS_AUTOSTART     에어프레임 번호 (px4/airframes/51010_gz_orchard_rover)
    #   PX4_GZ_WORLD          붙을 Gazebo 월드 이름 (생성기가 만든 <world name="orchard">)
    #   PX4_GZ_MODEL_NAME     제어할 Gazebo 모델 이름 (월드에 이미 스폰된 로버)
    px4_env = {
        'PX4_GZ_STANDALONE': '1',
        'PX4_SYS_AUTOSTART': '51010',
        'PX4_GZ_WORLD': 'orchard',
        'PX4_GZ_MODEL_NAME': 'orchard_rover',
        'PX4_SIMULATOR': 'gz',
        'HEADLESS': '1',
        # 느린 PC(맥 RTF 약 6%)에서 시뮬 배터리가 'unhealthy' 로 떠 시동 후 바퀴 명령이 멈출 수 있음
        # → 시뮬 배터리 최소 잔량을 높이고, 저전압 시 동작은 경고만. PX4 는 PX4_PARAM_* 환경변수를 시작 때 적용
        'PX4_PARAM_SIM_BAT_MIN_PCT': '80',
        'PX4_PARAM_COM_LOW_BAT_ACT': '0',
    }

    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource),
        ExecuteProcess(cmd=gz_server, output='screen', name='gz_server'),
    ]
    # Gazebo GUI 는 서버와 별도 프로세스(-g)로 띄운다 → headless 면 생략
    if not headless:
        actions.append(ExecuteProcess(cmd=['gz', 'sim', '-g'], output='log', name='gz_gui'))
    actions += [
        # PX4 는 Gazebo 월드가 뜬 뒤 붙는다 (스크립트가 최대 30초 대기)
        #   -i 0: SITL 인스턴스 번호 0 (여러 대면 1, 2 ...), -d: 대화형 pxh> 셸 없이 데몬으로 실행
        TimerAction(period=3.0, actions=[ExecuteProcess(
            cmd=[px4_bin, '-i', '0', '-d'], cwd=px4_dir, additional_env=px4_env,
            output='screen', name='px4_sitl')]),
        # PX4 SITL 의 uXRCE-DDS 클라이언트는 UDP 8888 로 Agent 에 접속 → /fmu/in/*, /fmu/out/* 토픽 생성
        ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'], output='log', name='xrce_agent'),
        # Gazebo 센서 토픽 → 실차와 같은 ROS 토픽 이름 (orchard_gazebo/config/bridge.yaml)
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge', output='screen',
             parameters=[{'config_file': os.path.join(get_package_share_directory('orchard_gazebo'),
                                                      'config', 'bridge.yaml')}, use_sim_time]),
        # 로봇 TF (sim:=true 면 카메라 광학 프레임 TF 도 발행)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('orchard_description'), 'launch', 'description.launch.py')),
            launch_arguments={'sim': 'true', 'use_sim_time': 'true'}.items()),
        # 인식·주행·PX4 브리지 노드 묶음 (sim.yaml 파라미터)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'autonomy.launch.py')),
            launch_arguments={'params_file': params, 'use_sim_time': 'true',
                              'camera_model': lc('camera_model'),
                              'record_dataset': lc('record_dataset'),
                              'seg_model': lc('seg_model'),
                              'seg_gt': lc('seg_gt'),
                              'record_seg': lc('record_seg'),
                              'row_spacing': lc('row_spacing'),
                              'first_turn': lc('first_turn'),
                              'voxel_map': lc('voxel_map'),
                              'nav_mode': lc('nav_mode'),
                              'approach': lc('approach'),
                              'lanes': lanes}.items()),
        # 평가용: Gazebo 정답 위치와 월드 메타데이터(통로 중심 y)를 비교해 통로 중심선 횡오차를 출력
        Node(package='orchard_gazebo', executable='lane_error_monitor', output='screen',
             parameters=[params, {'meta_file': meta}]),
    ]
    return actions


def generate_launch_description():
    """시뮬 launch 인자를 선언하고, 월드·프로세스 구성은 _setup 에, RViz·Foxglove 는 조건부 Node 로 둔다."""
    rviz_cfg = os.path.join(get_package_share_directory('orchard_bringup'), 'rviz', 'orchard.rviz')
    return LaunchDescription([
        # --- 실행 환경 / 화면 ---
        DeclareLaunchArgument('px4_dir', default_value=os.environ.get('PX4_DIR', '/opt/PX4-Autopilot')),
        DeclareLaunchArgument('cache_dir', default_value='~/.cache/orchard_rover'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('headless_rendering', default_value='false',
                              description='GUI 는 띄우되 센서 렌더링은 EGL 로 (가상 화면 녹화용)'),
        DeclareLaunchArgument('foxglove', default_value='false',
                              description='Foxglove 브리지(ws://localhost:8765) — Mac/Windows 에서 보기'),
        # --- 과수원 월드 (DEFAULTS 와 기본값을 같게 유지할 것) ---
        DeclareLaunchArgument('rows', default_value='6'),
        DeclareLaunchArgument('row_spacing', default_value='3.8'),
        DeclareLaunchArgument('row_length', default_value='30.0'),
        DeclareLaunchArgument('lanes', default_value='', description='주행할 통로 수 (기본 min(3, rows-1))'),
        DeclareLaunchArgument('explore', default_value='false',
                              description='탐사 모드: 통로 수 없이 끝 열까지 순회 + 나무 지도 저장'),
        DeclareLaunchArgument('obstacles', default_value='0'),
        DeclareLaunchArgument('seed', default_value='7'),
        DeclareLaunchArgument('scenario', default_value='full',
                              description='구간 시험: full | follow | uturn | edge | obstacle | approach (짧게 특정 동작만)'),
        DeclareLaunchArgument('spawn_x', default_value='', description='출발 x [m] (비우면 통로 0 입구)'),
        DeclareLaunchArgument('spawn_lane', default_value='0'),
        DeclareLaunchArgument('spawn_y', default_value='', description='출발 y [m] (주면 spawn_lane 무시, 과수원 밖 출발용)'),
        DeclareLaunchArgument('approach', default_value='false',
                              description='true: 과수원 밖에서 출발 → LiDAR 로 끝 통로 입구를 찾아 들어감 (nav_mode planner)'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0', description='[rad] 3.1416 = -x 방향'),
        DeclareLaunchArgument('first_turn', default_value='', description='첫 U턴 방향 left/right (비우면 sim.yaml)'),
        DeclareLaunchArgument('relief', default_value='0.30', description='지형 기복 진폭 [m] (0 = 평탄)'),
        DeclareLaunchArgument('bump_amp', default_value='0.045', description='지면 요철 진폭 [m]'),
        DeclareLaunchArgument('lite', default_value='false', description='저사양 PC: 잔디/열매 축소'),
        DeclareLaunchArgument('gz_verbose', default_value='1'),
        # --- 인식 모델 / 학습 데이터 수집 ---
        DeclareLaunchArgument('camera_model', default_value='', description='YOLO ONNX 경로 (비우면 카메라 검출 끔)'),
        DeclareLaunchArgument('record_dataset', default_value='false', description='자동 라벨링 데이터 저장'),
        DeclareLaunchArgument('nav_mode', default_value='planner',
                              description='planner: 전역·지역 경로계획 (기본) / row: 예전 행 추종 상태기계'),
        DeclareLaunchArgument('voxel_map', default_value='true',
                              description='복셀 맵 + 줄기 매핑 (출발점 기준 상대좌표, data/voxel_maps 에 저장)'),
        DeclareLaunchArgument('seg_model', default_value='',
                              description='세그멘테이션 ONNX (비우면 sim.yaml: /workspace/models/orchard_seg.onnx)'),
        DeclareLaunchArgument('camera_lite', default_value='false',
                              description='느린 PC: 카메라 320x240 10 Hz 로 낮춰 시뮬 속도 향상'),
        DeclareLaunchArgument('seg_camera', default_value='false',
                              description='Gazebo 세그멘테이션(정답 라벨) 카메라 켜기 (렌더링 부하 증가)'),
        DeclareLaunchArgument('seg_gt', default_value='false',
                              description='모델 대신 정답 라벨로 카메라 중심선 (기하 파이프라인 시험)'),
        DeclareLaunchArgument('record_seg', default_value='false',
                              description='세그멘테이션 학습 데이터 저장 (data/seg_dataset_sim)'),
        OpaqueFunction(function=_setup),
        # RViz: rviz:=true 이고 headless 가 아닐 때만 (조건식은 launch 가 실행 시점에 평가)
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg],
             parameters=[{'use_sim_time': True}], output='log',
             condition=IfCondition(PythonExpression([
                 "'", LaunchConfiguration('rviz'), "' == 'true' and '",
                 LaunchConfiguration('headless'), "' != 'true'"]))),
        # Foxglove 브리지: foxglove:=true 일 때만
        Node(package='foxglove_bridge', executable='foxglove_bridge', output='log',
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('foxglove'))),
    ])
