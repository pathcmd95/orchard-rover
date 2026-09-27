#!/usr/bin/env bash
# Docker 이미지 공유 도우미 — 학생들에게 실행 환경을 이미지로 나눠 줄 때 사용.
#
# 사용법
#   ./scripts/docker_share.sh build-full            orchard-rover:sim 위에 코드·모델·빌드를 얹은 orchard-rover:sim-full 생성
#   ./scripts/docker_share.sh save [이미지]         이미지를 파일로 저장 → dist/<이미지>_<아키텍처>.tar.gz (USB·드라이브 공유)
#   ./scripts/docker_share.sh load <파일.tar.gz>    받은 파일을 docker 에 불러오기
#   ./scripts/docker_share.sh push [이미지]         GitHub Container Registry(ghcr.io) 로 올리기 (docker login ghcr.io 먼저)
#   ./scripts/docker_share.sh pull                  ghcr.io 에서 받아 orchard-rover:sim-full 로 이름 붙이기
# 기본 이미지: orchard-rover:sim-full
# 환경 변수
#   REGISTRY   기본 ghcr.io/pathcmd95   (저장소 주인 계정. 학교 계정 등으로 바꿔 쓸 수 있음)
# 주의
#   - 이미지는 빌드한 컴퓨터의 CPU 아키텍처용이다. 맥(Apple Silicon)에서 만들면 arm64, 일반 PC 는 amd64.
#     그래서 태그 끝에 -arm64 / -amd64 를 붙여 올린다. 학생 PC 와 같은 아키텍처 이미지를 받아야 한다.
#   - 이미지 크기 약 10 GB 이상 (PX4·Gazebo 포함). 파일 공유 시 압축해도 수 GB.
#   - ghcr.io 패키지는 처음에 비공개(private). 학생에게 공개하려면 GitHub 의 Packages 설정에서 public 으로 바꾸거나
#     학생 계정을 추가한다. push 에는 write:packages 권한이 있는 토큰으로 docker login ghcr.io 가 필요하다.
set -euo pipefail
ORIG_PWD=$PWD                       # load 에 준 상대 경로를 원래 폴더 기준으로 해석하려고 기억
cd "$(dirname "$0")/.."
CMD=${1:-help}
IMAGE=${2:-orchard-rover:sim-full}
REGISTRY=${REGISTRY:-ghcr.io/pathcmd95}
case "$(uname -m)" in
  x86_64|amd64) ARCH=amd64 ;;
  arm64|aarch64) ARCH=arm64 ;;
  *) ARCH=$(uname -m) ;;
esac
NAME=${IMAGE%%:*}; TAG=${IMAGE##*:}

case "$CMD" in
  build-full)
    # 기반 이미지(orchard-rover:sim)가 없으면 먼저 만든다
    docker image inspect orchard-rover:sim > /dev/null 2>&1 || ./scripts/docker_build.sh sim
    DOCKER_BUILDKIT=1 docker build -f docker/Dockerfile.full -t orchard-rover:sim-full .
    echo "완료: orchard-rover:sim-full  (실행: docker run --rm -it orchard-rover:sim-full bash)" ;;
  save)
    mkdir -p dist
    OUT="dist/${NAME//\//_}_${TAG}_${ARCH}.tar.gz"
    echo "저장 중 → $OUT (수 분 걸림)"
    docker save "$IMAGE" | gzip -1 > "$OUT"
    ls -lh "$OUT"
    echo "받는 쪽: ./scripts/docker_share.sh load $(basename "$OUT")   또는   gunzip -c 파일 | docker load" ;;
  load)
    FILE=${2:?불러올 파일 경로 필요}
    [[ "$FILE" = /* ]] || FILE="$ORIG_PWD/$FILE"
    gunzip -c "$FILE" | docker load ;;
  push)
    REMOTE="$REGISTRY/$NAME:$TAG-$ARCH"
    docker tag "$IMAGE" "$REMOTE"
    docker push "$REMOTE"
    echo "올림: $REMOTE" ;;
  pull)
    REMOTE="$REGISTRY/orchard-rover:sim-full-$ARCH"
    docker pull "$REMOTE"
    docker tag "$REMOTE" orchard-rover:sim-full
    echo "받음: $REMOTE → orchard-rover:sim-full" ;;
  *)
    sed -n '2,21p' "$0" ;;
esac
