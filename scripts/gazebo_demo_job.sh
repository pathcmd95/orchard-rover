#!/usr/bin/env bash
# Gazebo 데모 녹화 작업 (맥 Docker Desktop / 리눅스 공용, 비대화형 → 작업 큐에 그대로 넣을 수 있음)
#
#   ./scripts/gazebo_demo_job.sh                       # 컨테이너 이름 bts_sils
#   CONTAINER_NAME=bts_sils DURATION=300 ./scripts/gazebo_demo_job.sh
#
# 하는 일
#   1) orchard-rover:sim 이미지가 없으면 빌드 (맥은 메모리 보호를 위해 병렬 4)
#   2) 이름이 CONTAINER_NAME 인 컨테이너를 새로 만들어 실행 (Docker Desktop 목록에 보임)
#   3) 컨테이너 안 가상 화면(Xvfb)에 Gazebo GUI·RViz 를 띄워 mp4·스크린샷·지표 저장
# 결과: data/media/gazebo_<날짜시각>/   (저장소 폴더라 호스트에서 바로 보임)
#
# 환경변수 (모두 선택)
#   CONTAINER_NAME  만들 컨테이너 이름 (기본 bts_sils. 같은 이름이 있으면 지우고 새로 만듦)
#   IMAGE           사용할 이미지 (기본 orchard-rover:sim, 없고 orchard-rover:sim-full 만 있으면 그것)
#   DURATION        최대 녹화 시간 [s] (기본 맥 600, 리눅스 240)
#   WARMUP          녹화 전 준비 대기 [s] (기본 맥 120, 리눅스 60)
#   LITE            저사양 월드 true|false (기본 true)
#   BUILD_JOBS      이미지 빌드 병렬 수 (기본 맥 4, 리눅스 0 = CPU 코어 수)
#   GPU_LOCK        GPU 작업 잠금 파일 경로 (기본 /tmp/orchard_gpu.lock)
# 녹화 세부 설정(FPS, TAIL, LAUNCH_ARGS 등)은 scripts/demo.env 에 적는다 (inner_gazebo_demo.sh 가 읽음).
set -euo pipefail
cd "$(dirname "$0")/.."

# --- 설정 (환경변수로 바꿀 수 있음, ${변수:-기본값}) ---
NAME=${CONTAINER_NAME:-bts_sils}
# 이미지: 직접 빌드한 orchard-rover:sim 이 없고 받은 완성 이미지(orchard-rover:sim-full)만 있으면 그것을 쓴다
if [ -z "${IMAGE:-}" ]; then
  if ! docker image inspect orchard-rover:sim > /dev/null 2>&1 && docker image inspect orchard-rover:sim-full > /dev/null 2>&1; then
    IMAGE=orchard-rover:sim-full
  else
    IMAGE=orchard-rover:sim
  fi
fi
LITE=${LITE:-true}
GPU_LOCK=${GPU_LOCK:-/tmp/orchard_gpu.lock}
# 운영체제별 기본값 (Darwin = macOS)
#   HEADLESS_RENDERING: 리눅스는 센서 렌더링을 EGL(화면 없는 GPU 렌더링)로, 맥은 GPU 가 없어 false
OS=$(uname -s)
if [ "$OS" = Darwin ]; then
  # 맥 Docker 는 GPU 가 없어 소프트웨어 렌더링 → 실시간 대비 약 5~10% 속도라 녹화를 10분으로
  DURATION=${DURATION:-600}; WARMUP=${WARMUP:-120}; JOBS=${BUILD_JOBS:-4}; HEADLESS_RENDERING=false
else
  DURATION=${DURATION:-240}; WARMUP=${WARMUP:-60}; JOBS=${BUILD_JOBS:-0}; HEADLESS_RENDERING=true
fi
# 결과 폴더(날짜·시각)를 만들고, 이후 모든 출력을 화면과 job.log 에 함께 기록
OUT="data/media/gazebo_$(date +%Y%m%d_%H%M)"
mkdir -p "$OUT"
exec > >(tee -a "$OUT/job.log") 2>&1
echo "[$(date '+%F %T')] 시작: OS=$OS NAME=$NAME DURATION=$DURATION LITE=$LITE OUT=$OUT"

# 1) 이미지가 없으면 빌드 (nice -n 10: 다른 작업보다 낮은 CPU 우선순위)
if ! docker image inspect "$IMAGE" > /dev/null 2>&1; then
  echo "$IMAGE 이미지 없음 → 빌드 (BUILD_JOBS=$JOBS, 40~90분)"
  nice -n 10 docker build -f docker/Dockerfile --target sim --build-arg BUILD_JOBS="$JOBS" -t "$IMAGE" .
fi

# 다른 GPU 작업과 겹치지 않게 잠금 (리눅스 flock, 맥은 mkdir 잠금)
# (flock: 파일 디스크립터 9 로 잠금을 잡고, 스크립트가 끝나면 자동으로 풀림)
if command -v flock > /dev/null 2>&1; then
  exec 9> "$GPU_LOCK"; echo "잠금 대기: $GPU_LOCK"; flock 9
else
  until mkdir "$GPU_LOCK.d" 2> /dev/null; do echo "잠금 대기: $GPU_LOCK.d"; sleep 15; done
  trap 'rmdir "$GPU_LOCK.d" 2>/dev/null || true' EXIT
fi

# 리눅스 + NVIDIA GPU 면 컨테이너에 GPU 전달
GPU_ARGS=()
if [ "$OS" = Linux ] && command -v nvidia-smi > /dev/null 2>&1 && nvidia-smi > /dev/null 2>&1; then
  GPU_ARGS=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all)
fi

# 2) 같은 이름의 이전 컨테이너를 지우고 새로 실행
#    --rm 을 쓰지 않으므로 끝난 뒤에도 Docker Desktop 에서 로그 확인·재시작이 가능하다.
docker rm -f "$NAME" > /dev/null 2>&1 || true
echo "[$(date '+%F %T')] 컨테이너 $NAME 실행"
# docker run 옵션
#   ${GPU_ARGS[@]+"${GPU_ARGS[@]}"}  배열이 비어 있어도 set -u 오류가 나지 않게 하는 bash 관용구
#   --shm-size=1g                    ROS 2/Gazebo 공유메모리 여유 (Docker 기본 64 MB 는 부족할 수 있음)
#   -v "$PWD:/workspace"             저장소 마운트 → 결과가 호스트 data/media 에 바로 저장됨
#   -v orchard-cache:...             생성한 월드·모델 캐시를 이름 있는 볼륨에 보관 (다음 실행 때 재사용)
#   -e ...                           녹화 설정을 컨테이너 안 inner_gazebo_demo.sh 로 전달
docker run --name "$NAME" ${GPU_ARGS[@]+"${GPU_ARGS[@]}"} --shm-size=1g \
  -v "$PWD:/workspace" -v orchard-cache:/root/.cache/orchard_rover \
  -e OUT="/workspace/$OUT" -e DURATION="$DURATION" -e WARMUP="$WARMUP" -e LITE="$LITE" \
  -e HEADLESS_RENDERING="$HEADLESS_RENDERING" \
  "$IMAGE" bash /workspace/scripts/inner_gazebo_demo.sh

# 3) 완료 시각과 결과 폴더 목록 출력
echo "[$(date '+%F %T')] 완료 → $OUT"
ls -la "$OUT"
