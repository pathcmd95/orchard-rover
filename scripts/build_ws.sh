#!/usr/bin/env bash
# 컨테이너 안에서 ROS 2 워크스페이스 빌드
#   ./scripts/build_ws.sh                                          # 전체 빌드
#   ./scripts/build_ws.sh --packages-select orchard_perception     # 추가 인자는 colcon build 로 그대로 전달
# 언제: 컨테이너에 처음 들어왔을 때, 또는 새 파일·패키지·launch·config 를 추가했을 때.
# --symlink-install: 설치 폴더가 소스를 가리키므로 파이썬 코드 수정은 대부분 다시 빌드하지 않아도 반영된다.
# 결과: ros2_ws/build, ros2_ws/install, ros2_ws/log
# (-u 를 쓰지 않는 이유: ROS setup.bash 가 정의 안 된 변수를 참조하기 때문)
set -eo pipefail
# 기반 환경: ROS 2 Jazzy → px4_msgs(/opt/px4_ws) → Livox 드라이버(/opt/livox_ws, robot 이미지에만) 순서로 source
source /opt/ros/jazzy/setup.bash
[ -f /opt/px4_ws/install/setup.bash ] && source /opt/px4_ws/install/setup.bash
[ -f /opt/livox_ws/install/setup.bash ] && source /opt/livox_ws/install/setup.bash
# 저장소의 ros2_ws 폴더로 이동해 빌드
cd "$(dirname "$0")/../ros2_ws"
colcon build --symlink-install "$@"
echo "빌드 완료 → source $(pwd)/install/setup.bash"
