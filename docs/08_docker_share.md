# 08. Docker 이미지로 환경 공유하기

처음 오는 학생이 40~90분 걸리는 이미지 빌드(PX4·Gazebo 컴파일)를 건너뛰고 바로 시뮬레이션을 돌릴 수 있도록,
관리자가 **완성 이미지**를 만들어 나눠 줍니다.

## 1. 이미지 종류

| 이미지 | 만드는 방법 | 들어 있는 것 | 용도 |
|---|---|---|---|
| `orchard-rover:sim` | `./scripts/docker_build.sh sim` | Ubuntu 24.04, ROS 2 Jazzy, PX4 v1.17.0 SITL, Gazebo Harmonic, Micro XRCE-DDS Agent, px4_msgs | 개발용 기반 이미지. 코드는 저장소를 마운트해서 씀 |
| `orchard-rover:sim-full` | `./scripts/docker_share.sh build-full` | 위 + 이 저장소 코드·`models/*.onnx` + 빌드된 ROS 워크스페이스 | **학생 배포용**. 저장소 없이도 바로 실행 |
| `orchard-rover:robot` | `./scripts/docker_build.sh robot` (Orin 에서) | ROS 2, Livox 드라이버, RealSense 드라이버 | 실차 |

> 이미지는 **만든 컴퓨터의 CPU 종류**용입니다. 맥(Apple Silicon)에서 만들면 `arm64`, 일반 PC 는 `amd64`.
> 학생 PC 와 같은 종류를 받아야 합니다. 그래서 올릴 때 태그 끝에 `-arm64` / `-amd64` 를 붙입니다.

## 2. 관리자: 이미지 만들기

```bash
cd orchard-rover
./scripts/docker_build.sh sim             # 기반 이미지 (처음 한 번)
# models/ 에 배포할 ONNX 를 넣어 둔다 (git 에는 안 올라감): orchard_seg.onnx, trunk_yolo.onnx
./scripts/docker_share.sh build-full      # → orchard-rover:sim-full  (코드가 바뀌면 다시 실행, 수 분)
```

`docker/Dockerfile.full` 이 `orchard-rover:sim` 위에 저장소를 복사하고 `colcon build` 를 합니다.
복사에서 빼는 파일은 `docker/Dockerfile.full.dockerignore` (data/, 빌드 산출물, dist/ 제외 · models/ 포함).

## 3. 관리자: 나눠 주기 — 세 가지 방법

### 방법 0. GitHub Actions 로 자동 빌드 (이 저장소에서 쓰는 방법)

`.github/workflows/docker-publish.yml` 이 GitHub 서버에서 amd64·arm64 를 각각 빌드해 ghcr.io 에 올리고 `sim-full` 태그 하나로 묶습니다 (1~2시간).

```bash
git tag v1.0 && git push origin v1.0      # 또는 GitHub → Actions → 'Docker 배포 이미지' → Run workflow
```

결과: `ghcr.io/<계정>/orchard-rover:sim-full` (자기 CPU 에 맞는 것을 받음), `:sim-full-amd64`, `:sim-full-arm64`.
처음 올린 패키지는 비공개이므로 GitHub 프로필 → Packages → orchard-rover → Package settings → Change visibility → **Public**.

### 방법 A. 파일로 (USB·구글 드라이브, 인터넷 계정 필요 없음)

```bash
./scripts/docker_share.sh save            # → dist/orchard-rover_sim-full_<arm64|amd64>.tar.gz (수 GB)
```

### 방법 B. GitHub Container Registry (ghcr.io)

```bash
# GitHub → Settings → Developer settings → Personal access tokens 에서 write:packages 권한 토큰 발급
echo <토큰> | docker login ghcr.io -u pathcmd95 --password-stdin
./scripts/docker_share.sh push            # → ghcr.io/pathcmd95/orchard-rover:sim-full-<arch>
```

- 처음 올린 패키지는 **비공개**입니다. GitHub 의 Packages → orchard-rover → Package settings 에서 public 으로 바꾸거나 학생 계정을 추가하세요.
- 다른 계정으로 올리려면 `REGISTRY=ghcr.io/<계정> ./scripts/docker_share.sh push`.
- 맥과 x86 PC 학생이 섞여 있으면 **각 컴퓨터에서 한 번씩** build-full → push 합니다.

## 4. 학생: 받아서 실행하기

```bash
# A. 파일을 받은 경우
./scripts/docker_share.sh load orchard-rover_sim-full_amd64.tar.gz
#   (저장소가 없으면: gunzip -c orchard-rover_sim-full_amd64.tar.gz | docker load)

# B. ghcr.io 에서 받는 경우
./scripts/docker_share.sh pull            # → orchard-rover:sim-full 로 이름 붙음
#   (저장소가 없으면: docker pull ghcr.io/pathcmd95/orchard-rover:sim-full-amd64)
```

### 4-1. 코드를 고치지 않고 바로 돌려 보기

```bash
docker run --rm -it --network host orchard-rover:sim-full bash
# 컨테이너 안
source ros2_ws/install/setup.bash
ros2 launch orchard_bringup sim.launch.py headless:=true scenario:=follow
```

### 4-2. 코드를 고치면서 쓰기 (권장)

저장소를 받아 `/workspace` 로 마운트하면, 호스트에서 고친 코드가 컨테이너에 바로 보입니다.

```bash
git clone https://github.com/pathcmd95/orchard-rover.git && cd orchard-rover
ORCHARD_IMAGE=orchard-rover:sim-full ./scripts/docker_run.sh sim
# 컨테이너 안 — 마운트한 저장소가 이미지 안 빌드 결과를 가리므로 한 번 빌드
./scripts/build_ws.sh && source ros2_ws/install/setup.bash
ros2 launch orchard_bringup sim.launch.py scenario:=uturn
```

맥에서 화면 녹화까지 하려면: `IMAGE=orchard-rover:sim-full ./scripts/gazebo_demo_job.sh` (결과 `data/media/`).

## 5. 자주 묻는 것

| 증상 | 해결 |
|---|---|
| `exec format error` | CPU 종류가 다른 이미지. 자기 PC 용(`-amd64`/`-arm64`)을 받는다 |
| `no space left on device` | Docker Desktop → Settings → Resources 에서 디스크 한도를 64 GB 이상으로 |
| `denied` (pull) | ghcr 패키지가 비공개. 관리자에게 public 전환 또는 계정 추가 요청 |
| 코드를 고쳤는데 반영 안 됨 | 4-2 방식으로 마운트했는지, `./scripts/build_ws.sh` 를 다시 했는지 확인 |
