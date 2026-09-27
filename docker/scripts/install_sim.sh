#!/usr/bin/env bash
# x86_64 개발 PC 전용: PX4 SITL(v1.17.0) + Gazebo Harmonic + ROS-Gazebo 브리지 + RViz
# 언제: docker/Dockerfile 의 sim 단계에서 자동 실행된다 (docker compose build sim, root 권한 필요, 30~60분).
# 설치 내용: PX4-Autopilot 소스 + 빌드 도구, Gazebo Harmonic, ros_gz 브리지, RViz, Xvfb/ffmpeg(녹화),
#            과수원 로버 에어프레임 등록 후 PX4 SITL 빌드
# 환경변수: PX4_VERSION (기본 v1.17.0), PX4_DIR (기본 /opt/PX4-Autopilot), BUILD_JOBS
set -euo pipefail
PX4_VERSION=${PX4_VERSION:-v1.17.0}
PX4_DIR=${PX4_DIR:-/opt/PX4-Autopilot}
JOBS=${BUILD_JOBS:-0}; [ "$JOBS" -gt 0 ] 2>/dev/null || JOBS=$(nproc)   # 메모리가 작으면 BUILD_JOBS=4 등으로 제한

apt-get update
# PX4-Autopilot 소스
#   저장소: https://github.com/PX4/PX4-Autopilot   버전: ${PX4_VERSION} (기본 v1.17.0)
#   v1.17.0 으로 고정하는 이유: px4_msgs(v1.17.0, install_base.sh)와 메시지 정의가 일치해야 하고,
#   로버 Offboard 동작과 에어프레임 파라미터(RA_*, RO_*)를 이 버전에서 확인했기 때문.
#   버전을 바꿀 때는 PX4_MSGS_VERSION 과 px4/airframes/51010_gz_orchard_rover 도 함께 확인할 것.
git clone --depth 1 --branch "${PX4_VERSION}" https://github.com/PX4/PX4-Autopilot.git "${PX4_DIR}"
cd "${PX4_DIR}"
# 서브모듈은 Gazebo 모델 폴더만 받는다 (전체 서브모듈은 매우 큼).
#   저장소: https://github.com/PX4/PX4-gazebo-models   버전: PX4 ${PX4_VERSION} 태그가 가리키는 커밋
git submodule update --init --depth 1 Tools/simulation/gz        # Gazebo 모델(rover_ackermann 등)
# PX4 공식 설치 스크립트: 빌드 도구 + Gazebo Harmonic(OSRF 저장소)
# numpy<2 고정: 최신 contourpy(1.4, py3.12)가 numpy>=2 를 요구해 pip 가 데비안 numpy 1.26 을
# 지우려다 실패함("Cannot uninstall numpy ... installed by debian"). ROS/OpenCV 도 numpy 1.x 기준.
echo "numpy<2" > /tmp/pip-constraints.txt
PIP_CONSTRAINT=/tmp/pip-constraints.txt RUNS_IN_DOCKER=true bash Tools/setup/ubuntu.sh --no-nuttx

# ROS-Gazebo 연동(ros_gz: 토픽 브리지·스폰·영상), RViz·rqt 도구, OpenGL 도구, 가상 화면(Xvfb)과 녹화(ffmpeg)
apt-get install -y --no-install-recommends \
  ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-image \
  ros-jazzy-rviz2 ros-jazzy-rqt-image-view ros-jazzy-rqt-graph \
  mesa-utils libgl1 x11-apps xvfb ffmpeg

# 과수원 로버 에어프레임 추가 후 SITL 빌드
bash /opt/orchard/scripts/add_px4_airframe.sh "${PX4_DIR}" /opt/orchard/px4/airframes/*
# SITL 빌드 → build/px4_sitl_default/bin/px4 (sim.launch.py 가 실행)
make -j"$JOBS" px4_sitl_default
# 빌드 결과 확인 (실행 파일이 없으면 이미지 빌드 실패)
test -x "${PX4_DIR}/build/px4_sitl_default/bin/px4"

# apt 캐시 삭제 (이미지 크기 절약)
apt-get clean
rm -rf /var/lib/apt/lists/*
