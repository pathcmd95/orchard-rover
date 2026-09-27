#!/usr/bin/env bash
# Pixhawk 용 PX4 로버 펌웨어 빌드 (sim 컨테이너 안에서 실행)
#   ./scripts/build_px4_firmware.sh px4_fmu-v6c
# 결과: firmware/<보드>_rover.px4 → QGroundControl 에서 사용자 지정 펌웨어로 업로드
#
# 언제: Pixhawk(실차)에 올릴 PX4 펌웨어가 필요할 때. 시뮬만 할 때는 필요 없다.
# 인자: $1 = PX4 보드 이름 (Pixhawk 6C → px4_fmu-v6c, 6X → px4_fmu-v6x). 틀린 이름을 주면 가능한 목록을 출력한다.
# 환경변수: PX4_DIR  PX4-Autopilot 소스 경로 (기본 /opt/PX4-Autopilot — sim 이미지에 v1.17.0 소스가 들어 있음)
# 주의: 시뮬(SITL)과 같은 PX4 v1.17.0 소스로 빌드하므로 컨테이너의 px4_msgs(v1.17.0)와 메시지 정의가 일치한다.
#       다른 버전의 펌웨어를 올리면 ROS 2 쪽 /fmu/* 토픽이 맞지 않을 수 있다.
#
# set -e: 오류 시 즉시 중단 / -u: 정의 안 된 변수 사용 시 오류 / -o pipefail: 파이프 중간 실패도 오류
set -euo pipefail
# 보드 이름 인자가 없으면 안내 문구를 출력하고 종료
BOARD=${1:?보드 이름 (예: px4_fmu-v6c, px4_fmu-v6x)}
PX4_DIR=${PX4_DIR:-/opt/PX4-Autopilot}
# 저장소 최상위 폴더 (이 스크립트의 한 단계 위)
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

# PX4 는 보드별 빌드 설정을 boards/<제조사>/<모델>/<변형>.px4board 에 둔다.
#   ${BOARD//_//} = 밑줄을 / 로 바꿈: px4_fmu-v6c → boards/px4/fmu-v6c/rover.px4board
# 이 보드에 rover 변형이 없으면 rover 를 지원하는 보드 목록을 보여 주고 종료
if [ ! -f "$PX4_DIR/boards/${BOARD//_//}/rover.px4board" ]; then
  echo "이 보드에는 rover 빌드 설정이 없습니다: $BOARD" >&2
  ls "$PX4_DIR"/boards/*/*/rover.px4board | sed "s#$PX4_DIR/boards/##; s#/rover.px4board##; s#/#_#" >&2
  exit 1
fi
# Pixhawk(NuttX, ARM Cortex-M)용 크로스 컴파일러가 없으면 PX4 공식 설치 스크립트로 설치
# (sim 이미지는 install_sim.sh 에서 --no-nuttx 로 설치했으므로 처음 한 번 필요)
if ! command -v arm-none-eabi-gcc >/dev/null 2>&1; then
  echo "NuttX(ARM) 빌드 도구 설치 (처음 한 번)"
  (cd "$PX4_DIR" && RUNS_IN_DOCKER=true bash Tools/setup/ubuntu.sh --no-sim-tools)
fi
# 빌드: make <보드>_rover → build/<보드>_rover/<보드>_rover.px4
cd "$PX4_DIR"
make "${BOARD}_rover"
# 결과 펌웨어를 저장소 firmware/ 폴더로 복사 (호스트에서 QGroundControl 로 업로드)
mkdir -p "$ROOT/firmware"
cp "build/${BOARD}_rover/${BOARD}_rover.px4" "$ROOT/firmware/"
echo "완료: firmware/${BOARD}_rover.px4"
