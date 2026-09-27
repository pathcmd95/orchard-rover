# orchard_description — 로버 치수·센서 장착 위치 (URDF)

**센서 장착 위치의 단일 출처는 `config/rover.yaml` 입니다.**
URDF(`urdf/orchard_rover.urdf.xacro`)와 Gazebo 로버 모델 생성기(`orchard_gazebo/model_gen.py`)가 모두 이 파일을 읽으므로,
여기만 고치면 시뮬레이션 센서 위치와 TF 가 함께 바뀝니다.

| 파일 | 내용 |
|---|---|
| `config/rover.yaml` | 차체(휠베이스 0.5 m, 윤거 0.6 m 등), Livox Mid-360·RealSense D455 의 위치(xyz)·자세(rpy)와 시뮬 사양 |
| `urdf/orchard_rover.urdf.xacro` | base_link → livox_frame, camera_link, camera_color_optical_frame 등 TF 트리 |
| `launch/description.launch.py` | robot_state_publisher 실행 (정적 TF 발행) |

## 실차에 맞출 때

1. 줄자로 base_link(뒤차축 중심 바닥 기준 등, 팀에서 정한 기준점) → 각 센서 위치를 잰다
2. `config/rover.yaml` 의 `sensors.livox.xyz/rpy`, `sensors.camera.xyz/rpy` 수정
3. 차체 치수는 PX4 파라미터(`RA_WHEEL_BASE` 등)와 같게
4. `./scripts/build_ws.sh` → `ros2 run tf2_ros tf2_echo base_link livox_frame` 으로 확인

좌표: x 전방, y 왼쪽, z 위 [m], rpy [rad] (ROS REP-103).
