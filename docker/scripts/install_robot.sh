#!/usr/bin/env bash
# Jetson Orin(arm64) 실차용: Livox Mid-360 드라이버 + RealSense D455 드라이버
# 언제: docker/Dockerfile 의 robot 단계에서 자동 실행된다 (docker compose build robot, root 권한 필요).
# 설치 내용: RealSense ROS 드라이버(apt), Livox-SDK2 (소스 빌드 → /usr/local),
#            livox_ros_driver2 (소스 빌드 → /opt/livox_ws)
# 환경변수: LIVOX_SDK_VERSION (기본 v1.3.1), LIVOX_DRIVER_VERSION (기본 1.2.6), BUILD_JOBS
set -euo pipefail
# --- 버전 고정: livox_ros_driver2 1.2.6 + Livox-SDK2 v1.3.1 조합으로 빌드를 확인했다 ---
LIVOX_SDK_VERSION=${LIVOX_SDK_VERSION:-v1.3.1}
LIVOX_DRIVER_VERSION=${LIVOX_DRIVER_VERSION:-1.2.6}
JOBS=${BUILD_JOBS:-0}; [ "$JOBS" -gt 0 ] 2>/dev/null || JOBS=$(nproc)   # 메모리가 작으면 BUILD_JOBS=4 등으로 제한

apt-get update
# RealSense: ROS 배포판 패키지 (librealsense2 포함)
apt-get install -y --no-install-recommends ros-jazzy-realsense2-camera ros-jazzy-pcl-conversions libpcl-dev libapr1-dev

# Livox-SDK2
#   저장소: https://github.com/Livox-SDK/Livox-SDK2   버전: ${LIVOX_SDK_VERSION} (기본 v1.3.1)
#   Mid-360 과 이더넷(UDP)으로 통신하는 C++ 라이브러리. livox_ros_driver2 가 이것을 링크한다.
git clone --depth 1 --branch "${LIVOX_SDK_VERSION}" https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2
# GCC 13(Ubuntu 24.04)에서 <cstdint> 누락으로 빌드가 실패하므로 강제 include (v1.3.1 에서 확인)
cmake -S /tmp/Livox-SDK2 -B /tmp/Livox-SDK2/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-include cstdint"
# 빌드 → /usr/local 설치 → 공유 라이브러리 캐시 갱신 → 소스 삭제
cmake --build /tmp/Livox-SDK2/build -j"$JOBS"
cmake --install /tmp/Livox-SDK2/build
ldconfig
rm -rf /tmp/Livox-SDK2

# livox_ros_driver2 (ROS 2 Jazzy 빌드는 build.sh 대신 아래와 같이 직접 수행)
#   저장소: https://github.com/Livox-SDK/livox_ros_driver2   버전: ${LIVOX_DRIVER_VERSION} (기본 1.2.6, 태그에 v 없음)
#   Mid-360 점군을 sensor_msgs/PointCloud2 (/livox/lidar) 로 발행하는 ROS 드라이버. Livox-SDK2 v1.3.1 과 짝.
mkdir -p /opt/livox_ws/src
git clone --depth 1 --branch "${LIVOX_DRIVER_VERSION}" https://github.com/Livox-SDK/livox_ros_driver2.git \
  /opt/livox_ws/src/livox_ros_driver2
# 저장소가 ROS 1/ROS 2 공용이라 ROS 2 용 package.xml 과 launch 폴더를 제자리에 복사 (원래 build.sh 가 하던 일)
cd /opt/livox_ws/src/livox_ros_driver2
cp -f package_ROS2.xml package.xml
cp -rf launch_ROS2 launch
set +u; source /opt/ros/jazzy/setup.bash; set -u
cd /opt/livox_ws
# 부족한 의존성을 rosdep 으로 설치 (-r: 일부 실패해도 계속, || true: 실패해도 스크립트 중단 안 함)
rosdep install --from-paths src --ignore-src -y -r || true
# ROS_EDITION/DISTRO_ROS 는 드라이버 CMakeLists 가 요구하는 옵션. -include cstdint 는 SDK 와 같은 이유(GCC 13)
colcon build --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=jazzy -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-include cstdint" \
  --event-handlers console_direct-
# 빌드 중간 파일 삭제 (install 만 남김)
rm -rf /opt/livox_ws/build /opt/livox_ws/log

apt-get clean
rm -rf /var/lib/apt/lists/*
