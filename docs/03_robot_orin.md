# 03. 실차(Jetson Orin) 배포

> 이 문서의 실차 절차는 아직 실제 차량에서 검증되지 않았습니다. 처음 적용할 때 결과를 GitHub Issue 나 팀 채널에 남겨 주세요.

## 1. 하드웨어 연결

```
 [Livox Mid-360] ──이더넷── [Jetson Orin] ──USB3── [RealSense D455]
   (9~27 V 전원)               │  UART(TELEM2) 또는 USB-시리얼
                               └────────────── [Pixhawk (PX4 rover)] ── GPS, ESC, 조향 서보, RC 수신기
```

| 연결 | Orin 쪽 | Pixhawk/센서 쪽 | 비고 |
|---|---|---|---|
| PX4 ↔ Orin | 40핀 UART (`/dev/ttyTHS1` 가 일반적) 또는 USB-시리얼(`/dev/ttyUSB0`) | TELEM2 | TX↔RX 교차, GND 공통. 장치명은 `ls /dev/ttyTHS* /dev/ttyUSB*` 로 확인 |
| Mid-360 | 유선랜 (고정 IP 192.168.1.50) | 기본 IP 192.168.1.1XX | XX = 시리얼번호 끝 두 자리 |
| D455 | USB 3 포트 (파란 포트) | — | USB2 로 연결되면 해상도/프레임이 제한됨 |

## 2. Orin 준비 (JetPack 7.2)

1. JetPack 7.2 이상을 설치합니다 (Ubuntu 24.04, L4T 39.2). Orin Nano 개발키트는 JetPack 7 부터 SD 카드 이미지가 없고 USB 설치 방식입니다. 설치 후 확인:
   ```bash
   cat /etc/nv_tegra_release      # R39 이면 JetPack 7.x
   lsb_release -rs                # 24.04
   ```
2. Docker 확인: `docker run --rm hello-world`. compose 가 없으면 `sudo apt install docker-compose-plugin`.
3. 시리얼 콘솔이 UART 를 점유하면 해제: `sudo systemctl disable --now nvgetty` (해당 서비스가 있을 때만).
4. 유선랜 고정 IP (Mid-360 용):
   ```bash
   nmcli con add type ethernet ifname eth0 con-name livox ipv4.method manual ipv4.addresses 192.168.1.50/24
   nmcli con up livox
   ping 192.168.1.1XX              # Mid-360 응답 확인
   ```
5. 이미지 빌드 (Orin 에서 직접, 20~40분):
   ```bash
   git clone https://github.com/pathcmd95/orchard-rover.git && cd orchard-rover
   ./scripts/docker_build.sh robot
   ./scripts/docker_run.sh robot
   ./scripts/build_ws.sh && source ros2_ws/install/setup.bash
   ```

JetPack 6.2(Ubuntu 22.04)를 유지해야 하는 경우에도 이 이미지는 CPU 로는 동작하지만, 컨테이너 안에서 GPU(CUDA)를 쓰는 조합은 NVIDIA 공식 지원 범위가 아닙니다. 현재 인식 코드는 CPU(onnxruntime, OpenCV)만 사용합니다.

## 3. Pixhawk (PX4 로버 펌웨어)

로버 기능은 기본 펌웨어에 없으므로 **로버 빌드**를 직접 올려야 합니다 (PX4 문서: Flashing the Rover Build).

```bash
# sim 컨테이너(x86 PC)에서, 보드 이름은 사용하는 Pixhawk 에 맞게 (예: px4_fmu-v6c, px4_fmu-v6x)
./scripts/build_px4_firmware.sh px4_fmu-v6c
# → firmware/px4_fmu-v6c_rover.px4 를 QGroundControl > 펌웨어 > 사용자 지정 파일로 업로드
```

QGroundControl 에서 설정할 파라미터:

| 파라미터 | 값 | 이유 |
|---|---|---|
| 에어프레임 | Generic Ackermann Rover (51000) | 조향형 로버 |
| `MAV_1_CONFIG` | 0 | TELEM2 의 MAVLink 끄기 |
| `UXRCE_DDS_CFG` | 102 (TELEM2) | ROS 2 연결 포트 (이더넷이면 1000) |
| `SER_TEL2_BAUD` | 921600 | Orin 쪽 `xrce_baud` 와 같게 |
| `UXRCE_DDS_SYNCT` | 1 | 시간 동기화 |
| `RA_WHEEL_BASE`, `RA_MAX_STR_ANG` | 실측값 | `rover.yaml`, `robot.yaml` 의 `follow.min_turn_radius` 와 일치 |
| `RO_SPEED_LIM` | 1.0 이하 | 과수원 안전 속도 |
| RC 채널 | 킬스위치 + 모드 스위치(Manual / Offboard) | **필수** |

그다음 PX4 로버 문서의 튜닝 순서(기본 설정 → 요레이트 → 자세 → 속도)를 Manual 모드에서 먼저 마칩니다. Offboard 는 튜닝이 끝난 뒤에 시험합니다.

## 4. 센서 확인 (주행 없이)

```bash
ros2 launch orchard_bringup robot.launch.py lidar_ip:=192.168.1.1XX autonomy:=false
```

다른 터미널(`docker exec -it orchard-robot bash`)에서:

```bash
ros2 topic hz /livox/lidar                   # 약 10 Hz
ros2 topic hz /camera/color/image_raw         # 약 15 Hz
ros2 topic echo /fmu/out/vehicle_status_v1 --once   # PX4 연결 확인
ros2 run tf2_ros tf2_echo base_link livox_frame     # 장착 위치 확인
```

노트북의 Foxglove Studio 에서 `ws://<Orin IP>:8765` 로 접속하면 점군·영상·마커를 볼 수 있습니다.

## 5. 실차 파라미터 맞추기 (`ros2_ws/src/orchard_bringup/config/robot.yaml`)

| 항목 | 방법 |
|---|---|
| `ground.nominal_z` | 평지에서 지면~base_link 높이를 재서 음수로 입력 |
| `self_filter.*` | 차체 크기 + 여유 10 cm (자기 몸에 맞은 점 제거) |
| `trunk.band_min/max` | 잔디 최대 높이보다 위, 첫 가지(수관) 높이보다 아래 |
| `row.expected_width` | 현장 행간 (울산 애플팜 3.8 m) |
| `follow.min_turn_radius` | 휠베이스 / tan(최대 조향각) |
| 센서 장착 위치 | `orchard_description/config/rover.yaml` 의 `xyz`, `rpy` 를 실측 |

LiDAR-카메라 정합 확인: `/orchard/debug/fusion` 영상에서 투영 박스가 실제 줄기에 겹치는지 봅니다. 어긋나면 `rover.yaml` 의 camera `xyz/rpy` 를 조정합니다.

## 6. 안전 점검표 (주행 전 매번)

- [ ] RC 킬스위치로 모터가 즉시 멈추는지 확인
- [ ] 첫 시험은 바퀴를 띄운 상태에서 조향 방향(좌/우 부호) 확인
- [ ] `robot.yaml` 의 `auto_arm`, `auto_offboard` 가 false 인지 확인
- [ ] 속도 제한: `max_speed` 0.8 m/s 이하, PX4 `RO_SPEED_LIM` 1.0 이하
- [ ] 2인 1조 (조종기 담당 + 노트북 담당), 주행 경로 안에 사람 없음
- [ ] 첫 주행은 `mission.lanes: 1` (한 통로만)

## 7. 주행 순서

1. `ros2 launch orchard_bringup robot.launch.py lidar_ip:=192.168.1.1XX`
2. 로버를 통로 입구에 행 방향으로 놓고 `/orchard/row` 의 `valid: true` 확인
3. 조종기로 시동 → 모드 스위치를 Offboard 로 (브리지가 setpoint 를 계속 보내고 있어 전환됨)
4. `ros2 service call /orchard/mission/start std_srvs/srv/Trigger`
5. 멈출 때: `ros2 service call /orchard/mission/stop std_srvs/srv/Trigger` 또는 조종기 Manual/킬스위치

## 8. 현장 데이터 기록

```bash
ros2 bag record -s mcap -o data/field_$(date +%m%d_%H%M) \
  /livox/lidar /camera/color/image_raw /camera/color/camera_info /tf /tf_static \
  /odom /gps/fix /orchard/row /orchard/trees
```

기록 파일은 용량이 커서 Git 에 올리지 않습니다. 파일은 드라이브로 전달하고, 팀 채널에 경로·설명을 남깁니다.
