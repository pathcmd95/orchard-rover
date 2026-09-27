#!/usr/bin/env bash
# 알고리즘 단위시험 + ROS 패키지 시험
#   ./scripts/test_ws.sh          # ROS 없이 순수 파이썬 시험만 (노트북에서도 가능)
#   ./scripts/test_ws.sh --ros    # 컨테이너 안에서 colcon test 까지
# 언제: 코드를 고친 뒤 commit 전에. ROS 없이 도는 pytest 시험이 인식·주행 알고리즘(순수 파이썬)을 검사한다.
# 필요: python3, pytest, numpy (--ros 는 sim 컨테이너 안에서만)
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SRC="$ROOT/ros2_ws/src"
# ROS 빌드 없이 패키지 소스 폴더를 PYTHONPATH 에 추가 → 순수 파이썬 모듈을 바로 import
export PYTHONPATH="$SRC/orchard_perception:$SRC/orchard_navigation:$SRC/orchard_mapping:$SRC/orchard_planning:$SRC/orchard_px4_bridge:$SRC/orchard_gazebo:${PYTHONPATH:-}"
python3 -m pytest -q \
  "$SRC/orchard_perception/test" "$SRC/orchard_navigation/test" "$SRC/orchard_mapping/test" "$SRC/orchard_planning/test" \
  "$SRC/orchard_px4_bridge/test" "$SRC/orchard_gazebo/test"
# --ros: 워크스페이스를 빌드하고 colcon test 로 패키지 시험까지 실행
#   px4_msgs 는 외부 메시지 패키지라 시험에서 제외
if [ "${1:-}" = "--ros" ]; then
  source /opt/ros/jazzy/setup.bash
  source /opt/px4_ws/install/setup.bash
  cd "$ROOT/ros2_ws"
  colcon build --symlink-install
  colcon test --packages-skip px4_msgs --python-testing pytest --event-handlers console_direct+
  colcon test-result --verbose
fi
