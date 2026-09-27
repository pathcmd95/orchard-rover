#!/usr/bin/env bash
# 사용법: ./scripts/docker_build.sh [sim|robot]
#   sim   : 개발 PC / 맥 (PX4 SITL + Gazebo Harmonic) — 처음 빌드 30~90분
#           x86_64(amd64) 와 Apple Silicon(arm64) 모두 빌드됨 (맥은 GPU 가 없어 RTF 약 5~10%)
#   robot : Jetson Orin 에서 실행 (Livox/RealSense 드라이버) — 20~40분
#   결과 이미지: orchard-rover:sim 또는 orchard-rover:robot
#   (compose.yaml 의 서비스 build 설정 → docker/Dockerfile 의 같은 이름 target 단계를 빌드)
# 언제: 처음 한 번, 또는 docker/ 아래 파일(Dockerfile, scripts/install_*.sh)이 바뀌었을 때.
# 소스코드(ros2_ws)는 이미지에 넣지 않으므로 코드만 바꿨을 때는 이미지를 다시 빌드할 필요가 없다.
set -euo pipefail
# 저장소 최상위 폴더로 이동 (compose.yaml 이 있는 곳)
cd "$(dirname "$0")/.."
# 첫 인자가 없으면 sim
TARGET=${1:-sim}
ARCH=$(uname -m)
# arm64(맥) 에서도 빌드되지만 GPU 가 없어 Gazebo 가 실시간의 5~7% 속도로 느리다는 안내만 출력
if [ "$TARGET" = sim ] && [ "$ARCH" != x86_64 ]; then
  echo "안내: $ARCH 에서 sim 이미지를 빌드합니다. 맥 Docker 는 GPU 가 없어 시뮬레이션이 느립니다 (docs/01 참고)." >&2
fi
# BuildKit 사용 (캐시·병렬 빌드, Dockerfile 첫 줄의 syntax 지시문 지원)
DOCKER_BUILDKIT=1 docker compose build "$TARGET"
