# orchard_gazebo — 과수원 월드·로버 모델 생성, 시뮬 평가

Gazebo Harmonic 용 **과수원 월드**(울산 애플팜 조건: 행간 3.8 m, 주간 1.2 m, 잔디 5~15 cm)와
PX4 `rover_ackermann` 에 Livox Mid-360·RealSense D455 를 붙인 **로버 모델**을 코드로 만듭니다.
`sim.launch.py` 가 실행할 때마다 `generate_orchard` 를 불러 `~/.cache/orchard_rover/` 에 새로 만듭니다.

## 실행 파일

| 이름 | 설명 |
|---|---|
| `generate_orchard` | 월드(SDF + 메시·텍스처) + 로버 모델 생성. `ros2 run orchard_gazebo generate_orchard --help` |
| `lane_error_monitor` | 정답 위치(`/ground_truth/odom`)와 통로 중심선 사이 **횡오차**를 주기적으로 출력 (시뮬 성능 지표) |

## 파일 지도

| 파일 | 하는 일 | 고칠 때 |
|---|---|---|
| `world_gen.py` | 월드 전체 조립(`generate_world`), 설정(`OrchardConfig`, `config_from_args`), 세그멘테이션 라벨 번호, 정답 메타(`orchard_meta.json`) | 열 수·간격, 장애물, 라벨 |
| `vegetation.py` | 사과나무·지주·철선·풀·돌 절차적 생성 (`RowBuilder`, `apple_tree`) | 나무 모양, 결주 확률 |
| `terrain.py` | 지형 높이·지면 구역·텍스처 (`OrchardGround`) | 경사, 바퀴자국 |
| `meshes.py` | OBJ 메시 도구 | |
| `model_gen.py` | 로버 모델에 센서 추가 (`add_livox`, `add_camera`, `stabilize_steering`) | 센서 사양 (위치는 `orchard_description/config/rover.yaml`) |
| `config/bridge.yaml` | Gazebo ↔ ROS 토픽 연결 (ros_gz_bridge) | 새 센서 토픽 |

월드 정답(`orchard_meta.json`)에는 나무·지주·장애물 위치가 있어 지도 채점에 씁니다 (`tools/mapping/view_voxel_map.py --truth`).

## 시험

```bash
cd ros2_ws/src/orchard_gazebo && python3 -m pytest -q test/
python3 tools/sim_checks/world_lidar_check.py    # Gazebo 없이 월드에 LiDAR 광선을 쏴서 인식 점검
```
