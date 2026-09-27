#!/usr/bin/env bash
# Ubuntu 24.04 + ROS 2 Jazzy 공통 기반 (x86_64 / arm64 공용)
# 언제: docker/Dockerfile 의 base 단계에서 자동 실행된다 (직접 실행하지 않음, root 권한 필요).
# 설치 내용: 기본 개발 도구, ROS 2 Jazzy(ros-base + 개발도구), onnxruntime,
#            Micro XRCE-DDS Agent (소스 빌드 → /usr/local), px4_msgs (소스 빌드 → /opt/px4_ws)
# 환경변수: ROS_APT_SOURCE_VERSION, XRCE_AGENT_VERSION, PX4_MSGS_VERSION, BUILD_JOBS (아래 기본값 참고)
set -euo pipefail
# --- 버전 고정 (바꿀 때는 PX4 펌웨어 · px4_msgs · XRCE Agent 조합을 함께 확인) ---
ROS_APT_SOURCE_VERSION=${ROS_APT_SOURCE_VERSION:-1.3.0}
XRCE_AGENT_VERSION=${XRCE_AGENT_VERSION:-v2.4.3}     # ROS 2 Jazzy + PX4 v1.x 조합 (v3.x 는 비호환)
PX4_MSGS_VERSION=${PX4_MSGS_VERSION:-v1.17.0}       # PX4 펌웨어 버전과 반드시 일치
JOBS=${BUILD_JOBS:-0}; [ "$JOBS" -gt 0 ] 2>/dev/null || JOBS=$(nproc)   # 메모리가 작으면 BUILD_JOBS=4 등으로 제한

# 기본 도구: 빌드(git, cmake, ninja), 편집기·네트워크 진단, 파이썬(numpy, yaml, opencv, pytest)
apt-get update
apt-get install -y --no-install-recommends \
  ca-certificates curl wget gnupg lsb-release locales tzdata sudo software-properties-common \
  git build-essential cmake ninja-build pkg-config \
  nano vim-tiny less htop iputils-ping iproute2 net-tools usbutils \
  python3-pip python3-venv python3-numpy python3-yaml python3-opencv python3-pytest
update-ca-certificates   # docker/certs/*.crt (교내 프록시 인증서 등) 반영

# ROS 2 apt 저장소 (ros2-apt-source 패키지 방식, 2025~ 공식 권장)
# VERSION_CODENAME (Ubuntu 24.04 = noble) 을 얻기 위해 OS 정보를 읽는다
. /etc/os-release
# ros2-apt-source: ROS apt 저장소 주소와 서명 키를 설치해 주는 .deb (https://github.com/ros-infrastructure/ros-apt-source)
curl -fsSL -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
apt-get install -y /tmp/ros2-apt-source.deb
rm /tmp/ros2-apt-source.deb
apt-get update
# ROS 2 Jazzy: ros-base(GUI 없는 핵심) + ros-dev-tools(colcon, rosdep), URDF/TF, 영상(cv_bridge),
#   foxglove_bridge(원격 모니터링), rosbag2 MCAP 저장 형식
apt-get install -y --no-install-recommends \
  ros-jazzy-ros-base ros-dev-tools \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-tf2-ros ros-jazzy-tf2-tools \
  ros-jazzy-vision-msgs ros-jazzy-cv-bridge ros-jazzy-sensor-msgs-py \
  ros-jazzy-foxglove-bridge ros-jazzy-rosbag2-storage-mcap

# YOLO 추론용 onnxruntime (CPU). 시스템 numpy(1.26)를 바꾸지 않도록 numpy<2 고정
pip3 install --break-system-packages --no-cache-dir "onnxruntime>=1.20" "numpy<2"

# rosdep: 패키지 의존성 자동 설치 도구 초기화 (이미 초기화돼 있으면 오류 무시)
rosdep init 2>/dev/null || true
rosdep update --rosdistro jazzy

# Micro XRCE-DDS Agent (PX4 uXRCE-DDS ↔ ROS 2)
#   저장소: https://github.com/eProsima/Micro-XRCE-DDS-Agent   버전: ${XRCE_AGENT_VERSION} (기본 v2.4.3)
#   PX4 v1.x 펌웨어의 uXRCE-DDS 클라이언트 + ROS 2 Jazzy 조합에서 쓰는 v2.x 계열로 고정 (v3.x 는 비호환)
#   역할: PX4 데이터를 ROS 2 DDS 토픽(/fmu/out/*)으로, ROS 2 명령(/fmu/in/*)을 PX4 로 중계
#   --depth 1: 해당 태그의 커밋만 받아 용량·시간 절약
git clone --depth 1 --branch "${XRCE_AGENT_VERSION}" https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/xrce
# CMake Release 빌드 → /usr/local 설치 → 공유 라이브러리 캐시 갱신(ldconfig) → 소스 삭제
cmake -S /tmp/xrce -B /tmp/xrce/build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/xrce/build -j"$JOBS"
cmake --install /tmp/xrce/build
ldconfig
rm -rf /tmp/xrce

# px4_msgs (PX4 v1.17.0 과 같은 메시지 정의)
#   저장소: https://github.com/PX4/px4_msgs   버전: ${PX4_MSGS_VERSION} (기본 v1.17.0)
#   px4_msgs 는 PX4 uORB 메시지의 ROS 2 정의다. 펌웨어(SITL·Pixhawk) 버전과 다르면 메시지 구조가 달라
#   토픽이 들어오지 않거나 잘못 해석되므로, PX4_VERSION(install_sim.sh)과 반드시 같은 버전을 쓴다.
mkdir -p /opt/px4_ws/src
git clone --depth 1 --branch "${PX4_MSGS_VERSION}" https://github.com/PX4/px4_msgs.git /opt/px4_ws/src/px4_msgs
# colcon 빌드를 위해 ROS 환경 source (setup.bash 가 정의 안 된 변수를 쓰므로 잠시 set +u)
set +u; source /opt/ros/jazzy/setup.bash; set -u
cd /opt/px4_ws
# 메시지 수가 많아 빌드 메모리를 많이 쓴다 → 패키지는 하나씩, make 병렬 수는 JOBS
MAKEFLAGS=-j"$JOBS" colcon build --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_direct-
# 빌드 중간 파일 삭제 (install 만 남겨 이미지 크기 절약)
rm -rf /opt/px4_ws/build /opt/px4_ws/log

# apt 캐시 삭제 (이미지 크기 절약)
apt-get clean
rm -rf /var/lib/apt/lists/*
