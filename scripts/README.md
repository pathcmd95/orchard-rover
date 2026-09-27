# scripts — 설치·빌드·실행·시험·녹화 스크립트

모두 **저장소 최상위에서** `./scripts/<이름>` 으로 실행합니다. 각 파일 맨 위 주석에 사용법이 있습니다.

| 스크립트 | 어디서 | 언제 / 하는 일 |
|---|---|---|
| `docker_build.sh [sim\|robot]` | 호스트 | 이미지 빌드. sim = PC·맥 (30~90분), robot = Orin |
| `docker_run.sh [sim\|robot] [명령]` | 호스트 | 컨테이너 진입. 저장소가 `/workspace` 로 마운트됨. GPU 자동 감지. `ORCHARD_IMAGE=orchard-rover:sim-full` 로 공유 이미지 사용 |
| `docker_share.sh build-full\|save\|load\|push\|pull` | 호스트 | 학생 배포용 완성 이미지 만들기·나누기 ([docs/08](../docs/08_docker_share.md)) |
| `build_ws.sh [colcon 인자]` | 컨테이너 | ROS 2 워크스페이스 빌드 (`--symlink-install`: 파이썬 수정은 대부분 재빌드 불필요) |
| `test_ws.sh [--ros]` | 어디서나 / 컨테이너 | 단위 시험 (ROS 없이 pytest). `--ros` 는 colcon test 까지 |
| `gazebo_demo_job.sh` | 호스트 (맥·리눅스) | 컨테이너 `bts_sils` 를 만들어 Gazebo+RViz 를 가상 화면에서 **녹화** → `data/media/gazebo_<날짜시각>/` |
| `inner_gazebo_demo.sh` | 컨테이너 (자동) | 위 작업의 컨테이너 안 부분. 직접 실행하지 않음 |
| `demo.env` | (설정 파일, git 제외) | 녹화 설정: `DURATION`, `FPS`, `TAIL`, `LAUNCH_ARGS="scenario:=uturn camera_lite:=true"` 등 |
| `build_px4_firmware.sh <보드>` | 컨테이너 | Pixhawk 용 PX4 로버 펌웨어 빌드 → `firmware/` |
| `../run_gazebo_demo.command` | 맥 Finder | 더블클릭으로 `gazebo_demo_job.sh` 실행 |

## 맥에서 짧은 구간 시험 녹화 순서

1. `scripts/demo.env` 에서 `LAUNCH_ARGS` 를 고른다 (예: `scenario:=uturn camera_lite:=true`)
2. Docker Desktop 에서 `bts_sils` 컨테이너 ▶(Start) — 또는 `./scripts/gazebo_demo_job.sh`
3. 끝나면 `data/media/gazebo_<날짜시각>/` 에 `gazebo.mp4`, `rviz.mp4`, `lane_error.txt`, `errors.txt`

## 컨테이너 안 진단 팁

`ros2` CLI 를 여러 번 부르면 Fast DDS 공유메모리 포트가 고갈될 수 있어, 스크립트는 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` 로 CLI 를 부릅니다.
직접 확인할 때도 같은 방법을 쓰세요: `FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ros2 topic echo /orchard/mission/state`
