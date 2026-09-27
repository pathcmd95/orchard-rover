"""인식 + 주행 + PX4 브리지 노드 묶음 (시뮬/실차 공통). params_file 로 차이를 준다.

역할
  과수원 자율주행에 필요한 ROS 2 노드들을 한꺼번에 띄우는 "공통 부품" launch 파일이다.
  보통 직접 실행하지 않고 sim.launch.py(시뮬) 또는 robot.launch.py(실차)가 include 해서 쓴다.
  시뮬/실차의 차이(토픽 이름, 속도, 자동 시동 여부 등)는 전부 params_file(YAML)로 준다.
    - 시뮬: orchard_bringup/config/sim.yaml
    - 실차: orchard_bringup/config/robot.yaml

직접 실행 예 (센서 토픽이 이미 나오고 있을 때만 의미가 있다)
  ros2 launch orchard_bringup autonomy.launch.py \\
      params_file:=$(ros2 pkg prefix orchard_bringup)/share/orchard_bringup/config/sim.yaml use_sim_time:=true

launch 인자
  params_file     (필수) 노드 파라미터 YAML 경로. 기본값이 없으므로 반드시 줘야 한다.
  use_sim_time    'true' 면 모든 노드가 Gazebo 의 /clock 시간을 쓴다 (시뮬에서 true, 실차 false).
  camera_model    YOLO 줄기 검출 ONNX 경로. 비우면 params_file 의 camera_tree_node.model_path 사용.
  record_dataset  'true' 면 tree_fusion_node 가 LiDAR 자동 라벨링 YOLO 학습 데이터를 저장.
  seg_model       세그멘테이션(주행가능 영역) ONNX 경로. 비우면 params_file 값.
  seg_gt          (시뮬 전용) 모델 대신 Gazebo 정답 라벨로 카메라 중심선을 계산 → 기하 파이프라인만 시험.
  record_seg      (시뮬 전용) seg_dataset_node 를 추가로 띄워 세그멘테이션 학습 데이터(영상+라벨) 저장.
  row_spacing     행간 거리 [m]. 주면 LiDAR 인식·미션·지도 노드 세 곳에 같은 값을 넣는다.
  lanes           주행할 통로 수. 0 = 탐사 모드(통로 수를 모르고 끝 열까지 순회).
  first_turn      첫 U턴 방향 'left' 또는 'right'.
  approach        'true' 면 과수원 밖에서 출발해 입구를 찾아 들어감 (plan.approach).
  nav_mode        'planner' (기본): 전역 계획(통로 순회 + Hybrid A* U턴) + 지역 계획(DWA 장애물 회피)
                  'row'           : 예전 방식 — 통로 중심선 Pure Pursuit + 미션 상태기계 (row_navigator_node)
  (row_spacing/lanes/first_turn/seg_model/camera_model 은 비워 두면 params_file 값이 그대로 쓰인다.)

띄우는 노드
  orchard_perception : lidar_tree_node(LiDAR 줄기·행 중심선·장애물), seg_path_node(카메라 세그멘테이션 중심선),
                       camera_tree_node(YOLO 줄기 검출), tree_fusion_node(LiDAR-카메라 융합·자동 라벨링),
                       [선택] seg_dataset_node
  orchard_planning   : [nav_mode=planner] global_planner_node(통로 순회·U턴 경로, 진행 상태), local_planner_node(DWA → /cmd_vel)
  orchard_navigation : [nav_mode=row] row_navigator_node(행 추종·U턴 미션 상태기계), orchard_mapper_node(나무 지도)
  orchard_px4_bridge : px4_offboard_bridge(/cmd_vel → PX4 Offboard 명령), px4_state_node(PX4 odom → TF)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context, *args, **kwargs):
    """launch 인자 값을 실제 문자열로 읽어 노드별 덮어쓰기 파라미터를 만들고 Node 목록을 돌려준다.

    OpaqueFunction 안에서 실행되므로 인자 값을 파이썬 if 문으로 검사할 수 있다.
    각 노드에는 [params_file, extra] 순서로 파라미터를 넘기므로 extra(launch 인자)가 YAML 보다 우선한다.
    """
    # lc('이름') → 해당 launch 인자의 현재 문자열 값
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    params = lc('params_file')
    # 모든 노드 공통 덮어쓰기: 시뮬 시간 사용 여부
    common = {'use_sim_time': lc('use_sim_time').lower() == 'true'}
    camera = dict(common)
    if lc('camera_model'):
        camera['model_path'] = lc('camera_model')      # launch 인자가 params 파일보다 우선
    fusion = dict(common)
    if lc('record_dataset').lower() == 'true':
        fusion['record_dataset'] = True
    # 노드마다 별도 dict 를 만들어야 한 노드용 덮어쓰기가 다른 노드로 새지 않는다
    lidar, nav, mapper, seg = dict(common), dict(common), dict(common), dict(common)
    if lc('seg_model'):
        seg['model_path'] = lc('seg_model')
    if lc('seg_gt').lower() == 'true':
        seg['use_gt_labels'] = True
    if lc('row_spacing'):                               # 시뮬 월드 행간과 인식/미션 값을 일치시킴
        lidar['row.expected_width'] = float(lc('row_spacing'))
        nav['mission.row_spacing'] = float(lc('row_spacing'))
        mapper['row_spacing'] = float(lc('row_spacing'))
    gplan, voxel = dict(common), dict(common)            # 전역 계획 (nav_mode=planner), 복셀 맵
    if lc('row_spacing'):
        gplan['plan.row_spacing'] = float(lc('row_spacing'))
        voxel['row_spacing'] = float(lc('row_spacing'))
    if lc('lanes'):
        nav['mission.lanes'] = int(lc('lanes'))
        gplan['plan.lanes'] = int(lc('lanes'))
    if lc('first_turn'):
        nav['mission.first_turn'] = lc('first_turn')
        gplan['plan.first_turn'] = lc('first_turn')
    if lc('approach').lower() in ('1', 'true', 'yes', 'on'):
        gplan['plan.approach'] = True                     # 과수원 밖 출발 → 입구 찾기 (nav_mode planner 만)

    def node(pkg, exe, name, extra):
        """Node 생성 도우미: params_file 을 먼저, extra(덮어쓰기) 를 나중에 적용한다."""
        return Node(package=pkg, executable=exe, name=name, output='screen', parameters=[params, extra])

    # 선택 노드: 세그멘테이션 학습 데이터 수집기 (record_seg:=true 일 때만)
    extra = []
    if lc('record_seg').lower() == 'true':
        extra.append(node('orchard_perception', 'seg_dataset_node', 'seg_dataset_node', dict(common)))
    # 선택 노드: 복셀 맵 + 줄기 매핑 (voxel_map:=true, 기본 켬). 출발점 기준 상대좌표로 기록
    if lc('voxel_map').lower() == 'true':
        extra.append(node('orchard_mapping', 'voxel_map_node', 'voxel_map_node', voxel))
    # 주행 방식: planner = 전역(통로 순회·U턴 Hybrid A*) + 지역(DWA) / row = 예전 행 추종 상태기계
    mode = lc('nav_mode').lower()
    if mode == 'planner':
        driving = [node('orchard_planning', 'global_planner_node', 'global_planner_node', gplan),
                   node('orchard_planning', 'local_planner_node', 'local_planner_node', dict(common))]
    elif mode == 'row':
        driving = [node('orchard_navigation', 'row_navigator_node', 'row_navigator_node', nav)]
    else:
        raise RuntimeError(f"nav_mode 는 planner 또는 row (받은 값: {mode})")
    # 항상 띄우는 노드. name 은 YAML 의 최상위 키(노드 이름)와 같아야 파라미터가 적용된다
    return extra + driving + [
        node('orchard_perception', 'lidar_tree_node', 'lidar_tree_node', lidar),
        node('orchard_perception', 'seg_path_node', 'seg_path_node', seg),
        node('orchard_perception', 'camera_tree_node', 'camera_tree_node', camera),
        node('orchard_perception', 'tree_fusion_node', 'tree_fusion_node', fusion),
        node('orchard_navigation', 'orchard_mapper_node', 'orchard_mapper_node', mapper),
        node('orchard_px4_bridge', 'offboard_node', 'px4_offboard_bridge', common),
        node('orchard_px4_bridge', 'state_node', 'px4_state_node', common),
    ]


def generate_launch_description():
    """launch 인자를 선언하고, 실제 노드 생성은 _nodes(OpaqueFunction)에 맡긴다."""
    return LaunchDescription([
        # 필수 인자 (기본값 없음): 노드 파라미터 YAML
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # 카메라 YOLO 모델 경로 (비우면 YAML 값)
        DeclareLaunchArgument('camera_model', default_value=''),
        # LiDAR 자동 라벨링 YOLO 데이터 저장 여부
        DeclareLaunchArgument('record_dataset', default_value='false'),
        DeclareLaunchArgument('seg_model', default_value='', description='세그멘테이션 ONNX (비우면 params 파일 값)'),
        DeclareLaunchArgument('seg_gt', default_value='false', description='시뮬: 정답 라벨로 카메라 중심선'),
        DeclareLaunchArgument('record_seg', default_value='false', description='시뮬: 세그멘테이션 학습 데이터 저장'),
        DeclareLaunchArgument('row_spacing', default_value='', description='비우면 params 파일 값 사용'),
        DeclareLaunchArgument('lanes', default_value='', description='비우면 params 파일 값 사용. 0 = 탐사 모드'),
        DeclareLaunchArgument('first_turn', default_value='', description='비우면 params 파일 값 (left/right)'),
        DeclareLaunchArgument('approach', default_value='false',
                              description='true: 과수원 밖 출발 → LiDAR 로 끝 통로 입구 찾기 (nav_mode planner)'),
        DeclareLaunchArgument('voxel_map', default_value='true',
                              description='복셀 맵 + 줄기 매핑 노드 (orchard_mapping/voxel_map_node) 실행'),
        DeclareLaunchArgument('nav_mode', default_value='planner',
                              description='planner: 전역(통로 순회·Hybrid A*)+지역(DWA) 경로계획 / row: 예전 행 추종'),
        OpaqueFunction(function=_nodes),
    ])
