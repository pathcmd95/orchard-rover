# 02. 시뮬레이션 구동

## 1. 실행

```bash
./scripts/docker_run.sh sim
source ros2_ws/install/setup.bash
ros2 launch orchard_bringup sim.launch.py
```

명령 하나로 아래가 순서대로 뜹니다.

1. 과수원 월드·로버 모델 생성 (`~/.cache/orchard_rover/`)
2. Gazebo Harmonic 서버 + 화면
3. PX4 SITL (에어프레임 51010, Gazebo 에 붙는 standalone 모드)
4. Micro XRCE-DDS Agent (UDP 8888)
5. ros_gz_bridge, robot_state_publisher, 인식·주행·PX4 브리지 노드, RViz

PX4 가 GPS 를 잡고 EKF 가 수렴하면(10~20초) 자동으로 Offboard 전환·시동되고, 행이 인식된 뒤 5초 후 출발합니다.

### 자주 쓰는 인자

| 인자 | 기본값 | 설명 |
|---|---|---|
| `headless:=true` | false | Gazebo 화면 없이 (원격 서버). `rviz:=false` 와 함께 |
| `lite:=true` | false | 저사양 PC(맥 Docker 등): 잎·풀·자갈 수와 지면 텍스처 해상도 축소 (모양은 같음) |
| `camera_lite:=true` | false | 저사양 PC: 카메라 320×240·10 Hz (LiDAR 는 그대로) |
| `rows:=8` | 6 | 나무 열 수 (통로 = 열 - 1) |
| `row_spacing:=3.5` | 3.8 | 행간 거리 [m] (인식·U턴 반경에도 자동 반영) |
| `lanes:=2` | min(3, rows-1) | 주행할 통로 수 |
| `row_length:=50` | 30 | 열 길이 [m] |
| `obstacles:=2` | 0 | 통로 안에 사람/운반차 배치 |
| `seed:=3` | 7 | 월드 무작위 배치 |
| `relief:=0.5` / `bump_amp:=0.06` | 0.30 / 0.045 | 지형 기복·요철 진폭 [m] (0 이면 평탄) |
| `foxglove:=true` | false | Foxglove 로 원격 보기 (ws://PC주소:8765) |
| `camera_model:=/workspace/models/trunk_yolo.onnx` | '' | 카메라 YOLO 검출 켜기 |
| `record_dataset:=true` | false | 자동 라벨링 데이터 저장 (docs/04) |
| `explore:=true` | false | 탐사 모드: 통로 수 없이 끝 열까지 + 나무 지도 저장 |
| `record_seg:=true` | false | 세그멘테이션 학습 데이터 저장 (정답 라벨 카메라 켜짐, docs/04 §5) |
| `seg_gt:=true` | false | 모델 대신 정답 라벨로 카메라 중심선 (시험용) |

그 밖의 미션 설정(첫 U턴 방향, 속도 등)은 `ros2_ws/src/orchard_bringup/config/sim.yaml` 에서 바꿉니다.

### 구간 시험 (빠른 개발·디버깅용)

전체 과수원(6열 × 30 m)을 다 돌면 느린 PC 에서 1시간 가까이 걸립니다. 고친 동작만 짧은 월드에서 확인하세요 (맥 Docker 기준 수 분).

| `scenario:=` | 월드 | 확인하는 동작 |
|---|---|---|
| `follow` | 2열 × 15 m, 통로 1개 | 행 따라가기, 횡오차 |
| `uturn` | 3열 × 12 m, 통로 0 의 5 m 지점 출발 | 행 끝 감지 → U턴 → 다음 통로 진입 |
| `edge` | 3열 × 12 m, 마지막 통로 6 m 지점 출발, 탐사 모드 | 끝 열 판정 후 정지, 나무 지도 저장 |
| `obstacle` | 2열 × 15 m, 통로 안 장애물 1개 | planner: 비켜 지나감 / row: 감속·정지 |
| `approach` | 3열 × 12 m, 과수원 밖(입구 9 m 앞, 오른쪽 3.5 m, 20° 비스듬히) 출발 | 입구 찾기 → 정렬점 → 통로 0 진입 |

`spawn_x`, `spawn_y`, `spawn_lane`, `spawn_yaw`, `first_turn` 으로 출발 위치를 직접 정할 수도 있습니다. 데모 녹화(`scripts/gazebo_demo_job.sh`)는 미션이 DONE/STOPPED 가 되면 자동으로 일찍 끝납니다.

### 과수원 월드 (자동 생성)

울산 애플팜 같은 M9 왜성 사과 밀식 과원을 절차적으로 만듭니다 (`orchard_gazebo/world_gen.py`, `terrain.py`, `vegetation.py`).

| 요소 | 내용 |
|---|---|
| 지형 | 배수 경사 + 수십 m 파장의 완만한 기복(±0.3 m) + 1~3 m 요철(±4.5 cm) + 나무 열 두둑(+10 cm) + 통로 바퀴자국(−3.5 cm). 충돌 메시라 차체가 실제로 흔들림 |
| 지면 | 불규칙한 잔디(깎은 곳/웃자란 곳/마른 곳), 맨흙 패치, 나무 밑 제초 띠와 낙엽·낙과, 자갈 농로(회전 공간)와 통로 속 자갈 패치, 충돌 있는 바위 |
| 나무 | 높이 3.0~3.5 m, 접목부 혹, 첫 가지 0.8~1.0 m, 처진 곁가지, 잎 수백~천여 장, 사과. 나무마다 쇠파이프 지주, 8그루마다 콘크리트 기둥 + 철선 3줄, 점적 호스 |
| 배경 | 과수원 밖 언덕과 숲 |

- 처음 실행할 때 메시·텍스처를 만들어 `~/.cache/orchard_rover/worlds/assets_<해시>/` 에 저장합니다 (lite 10~15초, 기본 25초). 같은 설정이면 다음부터 바로 뜹니다.
- 월드를 바꾼 뒤 Gazebo 없이 LiDAR 인식이 괜찮은지 확인: `pip install trimesh embreex` 후 `python3 tools/sim_checks/world_lidar_check.py --lite` (Mid-360 광선추적 → 지면·줄기·중심선 오차 표)
- 지형을 평탄하게 비교하고 싶으면 `ros2 launch orchard_bringup sim.launch.py relief:=0 bump_amp:=0`.

## 2. 무엇을 봐야 하나

RViz 에서
- 점군: Livox Mid-360 (수직 -7° ~ +52°)
- 갈색 원기둥: 검출된 줄기, 초록 선: 좌/우 과수열, 노란 선: 행간중심선, 빨간 상자: 통로 장애물
- 이미지 패널: LiDAR 줄기를 카메라에 투영한 박스 (초록 = 카메라 YOLO 와 일치)

터미널에서

```bash
ros2 topic echo /orchard/row --once                 # 행간중심선
ros2 topic echo /orchard/mission/state              # 미션 상태
ros2 topic hz /livox/lidar                          # 10 Hz 확인
ros2 topic echo /fmu/out/vehicle_status_v1 --once   # PX4 상태 (arming_state 2 = 시동)
```

| 토픽 | 형식 | 내용 |
|---|---|---|
| `/livox/lidar` | PointCloud2 | Mid-360 점군 (실차와 같은 이름) |
| `/camera/color/image_raw`, `/camera/color/camera_info` | Image, CameraInfo | D455 컬러 |
| `/orchard/trees` | orchard_msgs/TreeArray | 줄기 위치 (base_link) |
| `/orchard/row` | orchard_msgs/RowCenterline | 중심선 offset/heading, 행 끝, 장애물 거리 |
| `/orchard/detections` | vision_msgs/Detection2DArray | 카메라 YOLO 결과 |
| `/orchard/trees_fused` | TreeArray | 카메라로 확인된 줄기 표시 |
| `/cmd_vel` | Twist | 주행 명령 |
| `/odom`, `/gps/fix`, `/imu/data_raw` | 표준 메시지 | PX4 추정값 (ENU/FLU 로 변환됨) |
| `/orchard/eval/lateral_error` | Float32 | 정답 대비 통로 중심 횡오차 (시뮬 전용) |

## 3. 미션 제어

```bash
ros2 service call /orchard/mission/stop  std_srvs/srv/Trigger   # 정지
ros2 service call /orchard/mission/start std_srvs/srv/Trigger   # 다시 시작 (현재 통로부터)
ros2 service call /orchard/px4/disarm    std_srvs/srv/Trigger   # 시동 해제
```

상태 흐름: `FOLLOW_ROW → EXIT_ROW → TURN → ENTER_ROW → FOLLOW_ROW … → DONE`
다음 통로를 6 m 안에 찾지 못하면 `STOPPED` 로 안전정지합니다.

### 탐사 모드 (통로 수를 모를 때)

```bash
ros2 launch orchard_bringup sim.launch.py explore:=true
```
- 통로 수 없이 U턴을 계속하다가, U턴 뒤 들어가려는 곳의 먼 쪽(방금 돈 방향 쪽)에 나무 열이 없으면 **끝 열에 도달한 것으로 보고 멈춥니다** (`DONE|…|과수원 끝 열 도달`).
- 첫 U턴 방향(`mission.first_turn`)을 과수원 안쪽으로 두고 출발하세요. 반대로 두면 첫 통로만 돌고 끝납니다.

### 행 끝 U턴 (`mission.turn_mode`)
- `planned`(기본): 행 끝에서 LiDAR 줄기로 **다음 통로를 직접 잡습니다**. 돌 방향 쪽 가까운 열과 그 너머 열의 가운데가 다음 통로 중심이고, 두 열의 마지막 나무 바깥 0.8 m 가 입구입니다.
  현재 자세에서 입구(반대 방향)까지 Dubins 경로(전진만, 반경 = 측정 간격/2, 최소 0.9 m)를 만들고 Pure Pursuit 로 따라갑니다. RViz 에 하늘색 선(`/orchard/turn_path`)으로 보입니다.
  너머 열이 안 보이면 가까운 열 + 파라미터 간격/2, 줄기를 못 보면 파라미터 간격으로 같은 방식. 경로에서 1.2 m 넘게 벗어나면 안전정지.
- `arc`: 예전 방식. 반경 = 행간/2 반원을 요 각도만 보고 돕니다. 행간 파라미터가 틀리거나 잔디에서 미끄러지면 입구를 빗나갑니다.
- 폐루프 시험(3열·12 m, 입구 최대 횡오차): 간격 3.4 m(파라미터 3.8) + 회전 20 % 덜 됨 → planned 0.08 m / arc 1.35 m
- 실차: robot.yaml 의 `mission.lanes: 0`.

### 과수원 나무 지도

`orchard_mapper_node` 가 주행 중 검출한 줄기를 odom 좌표에 누적해 **열 번호-나무 번호**를 붙이고, 간격이 벌어진 곳을 **결주 후보**로 표시합니다.
- RViz 'Orchard Map': 초록 원기둥 + `열-나무` 라벨, 빨간 공 = 결주 후보
- 저장: 미션이 DONE 이 되거나 노드가 꺼질 때, 또는 `ros2 service call /orchard/map/save std_srvs/srv/Trigger`
  → `data/maps/orchard_map_<시각>/trees.csv` (열, 번호, x, y, 반경, 관측횟수, 위도, 경도), `summary.json` (열별 나무 수·간격·결주 위치). 실행 중에도 30초마다 같은 폴더에 덮어써 저장(`autosave_period`)

### 복셀 맵 + 줄기 지도 (출발점 기준 상대좌표)

`voxel_map_node`(기본 켬, 끄려면 `voxel_map:=false`)가 LiDAR 복셀 맵을 쌓고, YOLO 줄기 박스 → 복셀 위치 확인 → 매칭 순서로 줄기 위치를 기록합니다.
좌표는 미션 시작 위치가 (0,0,0), 처음 곧게 달린 방향이 +x 입니다. YOLO 모델이 없으면 LiDAR 줄기로 대신합니다.
- RViz 'Voxel map'(높이 색), 'Trunk map'(주황 = YOLO 확인), 'Start-frame path'. Fixed Frame 을 `start` 로 두면 출발점 기준으로 보입니다.
- 저장: 60초마다·DONE·종료 때 `data/voxel_maps/voxel_map_<시각>/` (voxels.ply, trunks.csv, trajectory.csv, meta.json)
- 그림: `python3 tools/mapping/view_voxel_map.py data/voxel_maps/voxel_map_<시각>` — 자세히는 [07. 복셀 맵](07_voxel_map.md)

## 4. 성능 지표

`lane_error_monitor` 가 10초마다 로그를 남깁니다.

```
[lane_error_monitor]: 통로 중심선 횡오차: RMS 4.8 cm, 최대 13.2 cm (샘플 1520)
```

보고서에는 RMS·최대 횡오차, 통로 완주율, 장애물 정지 거리(`/orchard/row` 의 `obstacle_distance`)를 함께 적습니다. 기록은 `ros2 bag record -s mcap /orchard/row /orchard/eval/lateral_error /odom` 로 남깁니다.

## 5. 실험 과제 예시

1. **LiDAR 장착 높이 trade-off**: `orchard_description/config/rover.yaml` 의 `sensors.livox.xyz` z 값을 0.3 / 0.5 / 0.8 로 바꿔 줄기 검출 개수와 횡오차 비교. (URDF 와 Gazebo 모델이 함께 바뀜)
2. **차체 흔들림**: 월드 생성 인자 `bumps`(요철 개수)를 늘리고 `/orchard/row` 의 신뢰도 변화 관찰. 로그의 `ground=fit(tilt x.x°)` 로 지면 추정이 기울기를 따라가는지 확인.
3. **줄기 검출 밴드**: `trunk.band_min / band_max` 를 잔디 높이·수관 높이에 맞춰 조정.
4. **장애물(OOD) 정지**: `obstacles:=2` 로 실행해 정지 거리와 재출발 동작 확인.
5. **행간 변화**: `row_spacing:=3.2 / 4.5` 에서 U턴 성공 여부 (planned U턴은 측정 간격/2 반경, 최소 회전반경 0.9 m).

## 6. 끄기

`Ctrl+C` 한 번이면 launch 가 띄운 프로세스가 모두 종료됩니다. 남은 프로세스가 있으면:

```bash
pkill -f "gz sim"; pkill -f px4; pkill -f MicroXRCEAgent
```

다음: [03. 실차 Orin 배포](03_robot_orin.md)
