#!/usr/bin/env bash
# 사용법: ./scripts/docker_run.sh [sim|robot] [명령...]
#   ./scripts/docker_run.sh sim                  # 셸 진입
#   ./scripts/docker_run.sh sim ./scripts/build_ws.sh
#   ./scripts/docker_run.sh robot                # Orin 에서 실차 컨테이너 셸
# 먼저 ./scripts/docker_build.sh 로 이미지를 만들어 둘 것.
# 저장소 폴더가 컨테이너의 /workspace 로 마운트되므로 호스트에서 고친 코드가 컨테이너에 바로 보인다.
# --rm: 셸을 나가면 컨테이너는 삭제된다 (/workspace 저장소와 orchard-cache 볼륨의 내용은 남음).
set -euo pipefail
cd "$(dirname "$0")/.."
# 첫 인자 = 서비스 이름(sim|robot), 나머지 인자 = 컨테이너 안에서 실행할 명령
TARGET=${1:-sim}
shift || true
# 직접 빌드한 orchard-rover:sim 이 없고 받은 완성 이미지(orchard-rover:sim-full)만 있으면 그것을 쓴다 (compose.yaml 의 ORCHARD_IMAGE)
if [ "$TARGET" = sim ] && [ -z "${ORCHARD_IMAGE:-}" ] && ! docker image inspect orchard-rover:sim > /dev/null 2>&1 \
    && docker image inspect orchard-rover:sim-full > /dev/null 2>&1; then
  export ORCHARD_IMAGE=orchard-rover:sim-full
  echo "orchard-rover:sim 없음 → 받은 완성 이미지 orchard-rover:sim-full 사용"
fi
# 겹쳐 쓸 compose 파일 목록 (뒤 파일이 앞 파일 설정에 덧붙음)
FILES=(-f compose.yaml)
# NVIDIA GPU 가 있고 드라이버가 동작하면 GPU 설정 추가 (Gazebo 렌더링 가속)
if [ "$TARGET" = sim ] && command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
  FILES+=(-f compose.gpu.yaml)
  echo "NVIDIA GPU 감지 → compose.gpu.yaml 적용"
fi
# Intel/AMD GPU 장치(/dev/dri)가 있으면 장치 공유 설정 추가
if [ "$TARGET" = sim ] && [ -e /dev/dri ]; then
  FILES+=(-f compose.dri.yaml)
fi
# 호스트에 X11 화면(DISPLAY)이 있을 때만
if [ -n "${DISPLAY:-}" ] && command -v xhost >/dev/null 2>&1; then
  xhost +local:root >/dev/null 2>&1 || true   # 컨테이너가 화면(X11)에 창을 띄울 수 있게 허용
fi
# 명령을 주면 그 명령만 실행하고 끝, 없으면 대화형 bash 셸
if [ $# -gt 0 ]; then
  docker compose "${FILES[@]}" run --rm "$TARGET" "$@"
else
  docker compose "${FILES[@]}" run --rm "$TARGET" bash
fi
