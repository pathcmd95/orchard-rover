# orchard_bringup — 실행(launch)·파라미터·RViz 설정

모든 노드를 한 번에 띄우는 launch 파일과, 노드들의 파라미터 파일이 모여 있습니다.
**파라미터를 바꾸고 싶으면 대부분 여기 `config/*.yaml` 을 고치면 됩니다.**

| 파일 | 내용 |
|---|---|
| `launch/sim.launch.py` | 시뮬 전체: 월드 생성 → Gazebo → PX4 SITL → XRCE Agent → ros_gz_bridge → 인식·주행·지도 노드 → RViz. `scenario:=follow|uturn|edge|obstacle|approach` 로 짧은 구간 시험 |
| `launch/robot.launch.py` | 실차(Orin): Livox 드라이버, RealSense, XRCE Agent(직렬), 자율주행 노드, foxglove |
| `launch/autonomy.launch.py` | 시뮬·실차 공통 노드 묶음 (인식·주행·나무 지도·복셀 맵·PX4 브리지) |
| `config/sim.yaml` | 시뮬 파라미터 (use_sim_time: true) |
| `config/robot.yaml` | 실차 파라미터 (★ 표시 항목은 실측값으로) |
| `config/MID360_config.json` | Livox 드라이버 네트워크 설정 (IP) |
| `rviz/orchard.rviz` | RViz 화면 구성 (점군, 줄기, 중심선, U턴 경로, 나무 지도, 복셀 맵, 줄기 지도, 카메라 디버그 영상) |

## 자주 쓰는 실행

```bash
ros2 launch orchard_bringup sim.launch.py                          # 기본: 6열 × 30 m, 3통로
ros2 launch orchard_bringup sim.launch.py scenario:=uturn          # 행 끝 → U턴 → 다음 통로 (약 5분)
ros2 launch orchard_bringup sim.launch.py explore:=true            # 통로 수 몰라도 끝 열까지 + 나무 지도
ros2 launch orchard_bringup sim.launch.py scenario:=approach       # 과수원 밖에서 입구 찾아 진입
ros2 launch orchard_bringup sim.launch.py explore:=true obstacles:=3 approach:=true spawn_x:=-9.0 spawn_y:=-3.5 spawn_yaw:=0.35   # 전체 시연 설정
ros2 launch orchard_bringup sim.launch.py camera_model:=/workspace/models/trunk_yolo.onnx   # YOLO 켜기
ros2 launch orchard_bringup sim.launch.py voxel_map:=false         # 복셀 맵 끄기 (저사양 PC)
ros2 launch orchard_bringup robot.launch.py                        # 실차
```

인자 전체 목록은 `sim.launch.py` 맨 위 설명 또는 `ros2 launch orchard_bringup sim.launch.py --show-args`.

## 파라미터를 바꿀 때 규칙

기본값을 바꾸면 `config/sim.yaml`, `config/robot.yaml`, 노드의 `declare_parameter`, 문서를 **함께** 고칩니다.
