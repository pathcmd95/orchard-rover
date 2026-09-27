# 01. 설치

## 1. 필요한 것

| 구분 | 권장 | 최소 |
|---|---|---|
| 시뮬레이션 PC | Ubuntu 24.04, NVIDIA GPU(RTX 3060 이상), RAM 32 GB | Ubuntu 22.04/24.04, 내장 GPU, RAM 16 GB (`lite:=true`) |
| 디스크 | 여유 60 GB 이상 (sim 이미지 약 15~20 GB) | 40 GB |
| 실차 | Jetson Orin (AGX / NX / Nano), JetPack 7.2 | — |
| 계정 | GitHub 계정 (Issue·PR 용) | — |

- **Windows**: WSL2 + Ubuntu 24.04 에 Docker 를 설치하면 동작합니다(WSLg 로 화면 표시). GPU 가속은 NVIDIA 드라이버가 WSL 을 지원해야 합니다.
- **Apple Silicon Mac**: 맥용(arm64) 이미지가 있어 그대로 돌아갑니다. 다만 Docker 에서 GPU 를 못 써서 시뮬레이션이 실시간의 5~10 % 로 느리고, 화면 대신 녹화 방식(`scripts/gazebo_demo_job.sh`, 02 문서)으로 봅니다. 긴 시험은 NVIDIA GPU 우분투 PC 를 권장합니다.

## 2. Docker 설치 (Ubuntu)

```bash
# Docker Engine (공식 저장소)
sudo apt-get update && sudo apt-get install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] \
https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) stable" | \
sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo usermod -aG docker $USER      # 로그아웃 후 다시 로그인
docker run --rm hello-world         # 확인
```

### NVIDIA GPU PC 추가 설정

```bash
nvidia-smi                          # 드라이버 확인
# NVIDIA Container Toolkit
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker
```

`./scripts/docker_run.sh` 가 `nvidia-smi` 를 감지하면 `compose.gpu.yaml` 을, `/dev/dri` 가 있으면 `compose.dri.yaml`(내장 GPU 가속)을 자동으로 붙입니다.

## 2-1. Docker Desktop 설치 (맥·윈도우)

1. https://www.docker.com/products/docker-desktop/ 에서 받아 설치하고 실행합니다 (맥은 Apple Silicon 용).
2. Settings → Resources 에서 메모리 **12 GB 이상**(최소 8 GB), 디스크 **80 GB 이상**으로 올립니다. 메모리가 작으면 이미지 빌드·Gazebo 가 도중에 죽습니다.
3. 윈도우는 Settings → Resources → WSL integration 에서 Ubuntu 24.04 를 켜고, 이후 명령은 모두 WSL 터미널에서 실행합니다.
4. 터미널에서 `docker run --rm hello-world` 로 확인합니다.

## 3. 저장소 받기

```bash
git clone https://github.com/pathcmd95/orchard-rover.git
cd orchard-rover
```

## 4. 이미지 준비 — 둘 중 하나

**(권장) 완성 이미지 받기** — PX4·Gazebo·ROS 2 에 이 저장소 코드·모델까지 들어 있습니다. 컴퓨터에 맞는 것(일반 PC amd64 / 맥 arm64)을 자동으로 받습니다 (약 10 GB, 로그인 필요 없음).

```bash
./scripts/docker_share.sh pull     # ghcr.io/pathcmd95/orchard-rover:sim-full → orchard-rover:sim-full
```

**직접 빌드** — 30~90분. `docker/` 아래 설치 스크립트를 고쳤을 때만 필요합니다.

```bash
./scripts/docker_build.sh sim
```

`./scripts/docker_run.sh`, `./scripts/gazebo_demo_job.sh` 는 직접 빌드한 `orchard-rover:sim` 이 없으면 받은 `orchard-rover:sim-full` 을 자동으로 씁니다.

이미지 안에 들어 있는 것:

| 구성 | 버전 | 비고 |
|---|---|---|
| Ubuntu | 24.04 | `docker/Dockerfile` 의 `BASE_IMAGE` |
| ROS 2 | Jazzy | ros-base + xacro, tf2, vision_msgs, cv_bridge, foxglove_bridge |
| Micro XRCE-DDS Agent | v2.4.3 | Jazzy 용 버전 (v3.x 는 PX4 와 비호환) |
| px4_msgs | v1.17.0 | PX4 펌웨어 버전과 반드시 같아야 함 |
| PX4-Autopilot | v1.17.0 | SITL 빌드 + 과수원 에어프레임 51010 |
| Gazebo | Harmonic | PX4 설치 스크립트가 설치 |

학교/회사 네트워크가 HTTPS 검사를 해서 빌드 중 인증서 오류가 나면, 네트워크 관리자에게 받은 루트 인증서(`*.crt`)를 `docker/certs/` 에 넣고 다시 빌드합니다.

## 5. 컨테이너 진입과 워크스페이스 빌드

```bash
./scripts/docker_run.sh sim        # 저장소 폴더가 /workspace 로 연결됨
./scripts/build_ws.sh              # 컨테이너 안
source ros2_ws/install/setup.bash
```

호스트에서 코드를 고치면 컨테이너 안에 바로 반영됩니다. 파이썬 파일은 `--symlink-install` 이라 다시 빌드할 필요가 없고, 메시지(`orchard_msgs`)나 `setup.py` 를 바꿨을 때만 `./scripts/build_ws.sh` 를 다시 실행합니다.

## 6. 설치 확인

```bash
./scripts/test_ws.sh               # 알고리즘 단위시험 (약 1분)
gz sim --versions                  # 8.x 가 나오면 Harmonic
ls $PX4_DIR/build/px4_sitl_default/bin/px4
```

다음: [02. 시뮬레이션 구동](02_simulation.md)
