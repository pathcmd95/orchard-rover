#!/usr/bin/env bash
# 컨테이너 진입 시 ROS 환경을 순서대로 불러온다.
# Dockerfile 의 ENTRYPOINT. 'docker run <이미지> <명령>' 의 명령보다 먼저 항상 실행된다.
# 순서: ROS 2 Jazzy → px4_msgs → Livox 드라이버(robot 이미지만) → 저장소 워크스페이스(빌드된 경우만)
# ROS setup.bash 가 정의 안 된 변수를 참조하므로 -u 를 끈다
set +u
source /opt/ros/jazzy/setup.bash
[ -f /opt/px4_ws/install/setup.bash ] && source /opt/px4_ws/install/setup.bash
[ -f /opt/livox_ws/install/setup.bash ] && source /opt/livox_ws/install/setup.bash
[ -f /workspace/ros2_ws/install/setup.bash ] && source /workspace/ros2_ws/install/setup.bash
export PX4_DIR=${PX4_DIR:-/opt/PX4-Autopilot}
# 받은 명령(기본 bash)으로 이 셸 프로세스를 교체 → Ctrl+C 같은 신호가 명령에 바로 전달된다
exec "$@"
