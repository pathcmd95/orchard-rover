# syntax=docker/dockerfile:1.7
# 과수원 로버(유니주행차) 개발/시뮬/실차 이미지 — Ubuntu 24.04 + ROS 2 Jazzy
#
#   docker compose build sim     # x86_64 PC: PX4 SITL + Gazebo Harmonic + 개발도구
#   docker compose build robot   # Jetson Orin(JetPack 7.2, arm64): Livox/RealSense 드라이버
#
# 소스코드는 이미지에 넣지 않고 /workspace 로 마운트해서 컨테이너 안에서 빌드한다(scripts/build_ws.sh).
#
# 단계(stage) 구조 — multi-stage build
#   base  : Ubuntu 24.04 + ROS 2 Jazzy + Micro XRCE-DDS Agent v2.4.3 + px4_msgs v1.17.0   (install_base.sh)
#   sim   : base + PX4 SITL v1.17.0 + Gazebo Harmonic + ros_gz + RViz                     (install_sim.sh)
#   robot : base + Livox-SDK2 / livox_ros_driver2 + RealSense 드라이버                      (install_robot.sh)
# 빌드 인자 (--build-arg)
#   BASE_IMAGE   기반 이미지 (기본 ubuntu:24.04)
#   BUILD_JOBS   컴파일 병렬 수. 0 = CPU 코어 수. 메모리가 작은 PC(맥 Docker 등)는 4 정도로 줄일 것
#   PX4_VERSION  (sim) PX4 소스 태그. px4_msgs 버전(install_base.sh)과 맞춰야 한다
# 직접 빌드 예: docker build -f docker/Dockerfile --target sim --build-arg BUILD_JOBS=4 -t orchard-rover:sim .

ARG BASE_IMAGE=ubuntu:24.04

# ===== base 단계: sim/robot 공통 =====
FROM ${BASE_IMAGE} AS base
# ARG 는 단계(FROM)마다 다시 선언해야 그 단계에서 쓸 수 있다
ARG BUILD_JOBS=0
# RUN 명령을 bash 로 실행하고, 파이프 중간 명령이 실패해도 오류로 처리
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
# 비대화형 apt 설치, UTF-8 (한글 출력), 한국 시간대, ROS 배포판 이름
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 LC_ALL=C.UTF-8 TZ=Asia/Seoul \
    ROS_DISTRO=jazzy
# (선택) 교내/사내 네트워크가 TLS 검사를 하면 docker/certs/ 에 *.crt 를 넣는다
COPY docker/certs/ /usr/local/share/ca-certificates/extra/
# 설치 스크립트를 이미지에 복사 (install_*.sh, add_px4_airframe.sh)
COPY docker/scripts/ /opt/orchard/scripts/
# 공통 기반 설치
RUN BUILD_JOBS=${BUILD_JOBS} bash /opt/orchard/scripts/install_base.sh
# 컨테이너 시작 스크립트(ROS 환경 source)와 대화형 셸 설정(별칭 cb, sim)
COPY docker/entrypoint.sh /usr/local/bin/orchard-entrypoint.sh
COPY docker/bashrc /etc/orchard.bashrc
RUN chmod +x /usr/local/bin/orchard-entrypoint.sh && echo 'source /etc/orchard.bashrc' >> /root/.bashrc
# 저장소가 마운트될 작업 폴더. 모든 명령은 entrypoint 가 ROS 환경을 불러온 뒤 실행된다
WORKDIR /workspace
ENTRYPOINT ["/usr/local/bin/orchard-entrypoint.sh"]
CMD ["bash"]

# ===== sim 단계: x86_64 개발 PC 용 시뮬레이션 이미지 =====
FROM base AS sim
ARG PX4_VERSION=v1.17.0
ARG BUILD_JOBS=0
# PX4 소스 위치. QT_X11_NO_MITSHM=1: Qt 앱(Gazebo GUI·RViz)이 X11 공유메모리를 쓰지 않게 (컨테이너 화면 깨짐 방지)
ENV PX4_DIR=/opt/PX4-Autopilot \
    QT_X11_NO_MITSHM=1
# 과수원 로버 PX4 에어프레임 (install_sim.sh 가 PX4 소스트리에 등록)
COPY px4/ /opt/orchard/px4/
RUN PX4_VERSION=${PX4_VERSION} BUILD_JOBS=${BUILD_JOBS} bash /opt/orchard/scripts/install_sim.sh

# ===== robot 단계: Jetson Orin(arm64) 실차 이미지 =====
FROM base AS robot
ARG BUILD_JOBS=0
RUN BUILD_JOBS=${BUILD_JOBS} bash /opt/orchard/scripts/install_robot.sh
