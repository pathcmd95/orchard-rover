"""실차(Jetson Orin) 실행: 센서 드라이버 + PX4 연결 + 인식/주행.

  ros2 launch orchard_bringup robot.launch.py
  ros2 launch orchard_bringup robot.launch.py lidar_ip:=192.168.1.157 xrce_device:=/dev/ttyTHS1
  ros2 launch orchard_bringup robot.launch.py autonomy:=false     # 센서만 (데이터 수집)

안전: 실차에서는 자동 시동/Offboard 전환을 하지 않는다 (config/robot.yaml).
      조종기로 시동 → Offboard 스위치 → ros2 service call /orchard/mission/start std_srvs/srv/Trigger

언제 쓰나
  robot 컨테이너(./scripts/docker_run.sh robot) 안에서 워크스페이스를 빌드한 뒤 Orin 에서 실행한다.
  처음 필드에 나가기 전 docs/03_robot_orin.md 체크리스트와 config/robot.yaml 의 ★ 표시 값을 확인할 것.

launch 인자
  params_file     노드 파라미터 YAML. 비우면 orchard_bringup/config/robot.yaml.
  lidar           'true' 면 Livox Mid-360 드라이버(livox_ros_driver2) 실행 → /livox/lidar
  lidar_ip        Mid-360 의 IP. 기본 규칙: 192.168.1.1 + 시리얼번호 끝 두 자리 (예: ...57 → 192.168.1.157)
  host_ip         Orin 유선랜(LiDAR 쪽) IP. LiDAR 가 이 주소로 점군을 보낸다.
  camera          'true' 면 RealSense D455 드라이버(realsense2_camera) 실행 → /camera/...
  xrce_transport  PX4 연결 방식 'serial'(Pixhawk TELEM ↔ Orin UART) | 'udp'(이더넷)
  xrce_device     serial 일 때 장치 파일 (Orin 40핀 UART = /dev/ttyTHS1)
  xrce_baud       serial 속도. PX4 의 UXRCE_DDS / SER_TELx_BAUD 설정과 같아야 한다.
  xrce_port       udp 일 때 Agent 가 듣는 포트 (PX4 기본 8888)
  autonomy        'true' 면 autonomy.launch.py(인식·주행·PX4 브리지 노드)까지 실행. false = 센서만.
  camera_model    YOLO 줄기 검출 ONNX (비우면 robot.yaml 값)
  record_dataset  'true' 면 현장 자동 라벨링 데이터 저장 (tree_fusion_node)
  nav_mode        row (실차 기본, 예전 행 추종) | planner (전역·지역 경로계획 — 현장 검증 후 기본으로 바꿀 것)
  foxglove        'true' 면 foxglove_bridge(포트 8765) 실행 → 노트북에서 원격 모니터링

띄우는 것 (인자에 따라)
  livox_ros_driver2_node, realsense2_camera(rs_launch.py include), MicroXRCEAgent(PX4 ↔ ROS 2),
  robot_state_publisher(description.launch.py), autonomy.launch.py 노드들, foxglove_bridge
"""
import json
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    """launch 인자 값을 읽어 실차용 액션(센서 드라이버·XRCE Agent·자율주행 노드) 목록을 만든다."""
    # lc('이름') → 해당 launch 인자의 현재 문자열 값
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    bringup = get_package_share_directory('orchard_bringup')
    params = lc('params_file') or os.path.join(bringup, 'config', 'robot.yaml')
    actions = []

    # --- Livox Mid-360: IP 를 launch 인자로 바꿔 임시 설정파일 생성 ---
    # livox_ros_driver2 는 IP 를 JSON 파일로만 받으므로, 저장소의 MID360_config.json 을 읽어
    # host_ip / lidar_ip 만 바꾼 복사본을 /tmp 에 쓰고 그 경로를 드라이버에 넘긴다.
    if lc('lidar').lower() == 'true':
        with open(os.path.join(bringup, 'config', 'MID360_config.json'), encoding='utf-8') as f:
            cfg = json.load(f)
        host = cfg['MID360']['host_net_info']
        # LiDAR 가 명령·상태·점군·IMU 데이터를 보낼 목적지 = Orin(host) IP
        for key in ('cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip'):
            host[key] = lc('host_ip')
        cfg['lidar_configs'][0]['ip'] = lc('lidar_ip')
        tmp = os.path.join(tempfile.gettempdir(), 'orchard_mid360_config.json')
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(cfg, f, indent=2)
        # 드라이버 파라미터: PointCloud2, 10 Hz, frame_id=livox_frame (rover.yaml sensors.livox.frame 과 같아야 TF 가 맞음)
        actions.append(Node(
            package='livox_ros_driver2', executable='livox_ros_driver2_node', name='livox_lidar_publisher',
            output='screen',
            parameters=[{'xfer_format': 0,            # 0: sensor_msgs/PointCloud2
                         'multi_topic': 0, 'data_src': 0, 'publish_freq': 10.0,
                         'output_data_type': 0, 'frame_id': 'livox_frame',
                         'user_config_path': tmp, 'cmdline_input_bd_code': 'livox0000000001'}]))

    # --- RealSense D455 ---
    # camera_namespace='' + camera_name='camera' → 토픽이 /camera/color/image_raw 등이 되어
    # 시뮬(bridge.yaml)과 같은 이름이 된다. 848x480 @15 fps 는 D455 에서 컬러·깊이 모두 지원하는 조합.
    # align_depth: 깊이 영상을 컬러 영상 좌표에 맞춤 → /camera/aligned_depth_to_color/image_raw
    if lc('camera').lower() == 'true':
        rs_launch = os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch),
            launch_arguments={
                'camera_namespace': '', 'camera_name': 'camera',
                'enable_color': 'true', 'enable_depth': 'true',
                'align_depth.enable': 'true',
                'rgb_camera.color_profile': '848,480,15',
                'depth_module.depth_profile': '848,480,15',
                'enable_gyro': 'false', 'enable_accel': 'false',
                'publish_tf': 'true',
            }.items()))

    # --- PX4 연결 (Pixhawk ↔ Orin) ---
    # Micro XRCE-DDS Agent: PX4 의 uXRCE-DDS 클라이언트와 통신해 /fmu/in/*, /fmu/out/* ROS 2 토픽을 만든다.
    if lc('xrce_transport') == 'serial':
        agent = ['MicroXRCEAgent', 'serial', '--dev', lc('xrce_device'), '-b', lc('xrce_baud')]
    else:
        agent = ['MicroXRCEAgent', 'udp4', '-p', lc('xrce_port')]
    actions.append(ExecuteProcess(cmd=agent, output='screen', name='xrce_agent'))

    # 로봇 TF (base_link → livox_frame / camera_link / gps_link). 실차이므로 sim:=false
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('orchard_description'), 'launch', 'description.launch.py')),
        launch_arguments={'sim': 'false', 'use_sim_time': 'false'}.items()))

    # 인식·주행·PX4 브리지 노드 묶음 (robot.yaml 파라미터, 실제 시간)
    if lc('autonomy').lower() == 'true':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'autonomy.launch.py')),
            launch_arguments={'params_file': params, 'use_sim_time': 'false',
                              'camera_model': lc('camera_model'),
                              'record_dataset': lc('record_dataset'),
                              'nav_mode': lc('nav_mode')}.items()))

    # 원격 모니터링: 노트북 Foxglove Studio 에서 ws://<orin-ip>:8765 로 접속
    if lc('foxglove').lower() == 'true':
        actions.append(Node(package='foxglove_bridge', executable='foxglove_bridge', output='log',
                            parameters=[{'port': 8765}]))
    return actions


def generate_launch_description():
    """실차 launch 인자를 선언하고 실제 구성은 _setup(OpaqueFunction)에 맡긴다."""
    return LaunchDescription([
        # 비우면 config/robot.yaml
        DeclareLaunchArgument('params_file', default_value=''),
        # --- LiDAR ---
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('lidar_ip', default_value='192.168.1.12',
                              description='Mid-360 IP = 192.168.1.1 + 시리얼번호 끝 두 자리'),
        DeclareLaunchArgument('host_ip', default_value='192.168.1.50', description='Orin 유선랜 IP'),
        # --- 카메라 ---
        DeclareLaunchArgument('camera', default_value='true'),
        # --- PX4 연결 (Micro XRCE-DDS Agent) ---
        DeclareLaunchArgument('xrce_transport', default_value='serial', description='serial | udp'),
        DeclareLaunchArgument('xrce_device', default_value='/dev/ttyTHS1'),
        DeclareLaunchArgument('xrce_baud', default_value='921600'),
        DeclareLaunchArgument('xrce_port', default_value='8888'),
        # --- 자율주행 노드 ---
        DeclareLaunchArgument('autonomy', default_value='true'),
        DeclareLaunchArgument('camera_model', default_value=''),
        DeclareLaunchArgument('record_dataset', default_value='false'),
        # 실차는 Gazebo·현장 검증 전까지 예전 행 추종(row) 을 기본으로. 경로계획은 nav_mode:=planner
        DeclareLaunchArgument('nav_mode', default_value='row',
                              description='row: 행 추종 상태기계 (실차 기본) / planner: 전역·지역 경로계획'),
        # --- 모니터링 ---
        DeclareLaunchArgument('foxglove', default_value='true',
                              description='노트북 Foxglove 에서 ws://<orin-ip>:8765 로 모니터링'),
        OpaqueFunction(function=_setup),
    ])
