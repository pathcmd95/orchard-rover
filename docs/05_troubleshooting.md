# 05. 문제 해결

| 증상 | 원인 / 해결 |
|---|---|
| `docker: permission denied` | `sudo usermod -aG docker $USER` 후 재로그인 |
| 빌드 중 `certificate verify failed` | 교내 네트워크 인증서 → `docker/certs/*.crt` 에 넣고 재빌드 |
| Gazebo 창이 안 뜸 / `could not connect to display` | 호스트에서 `xhost +local:root`, `echo $DISPLAY` 확인. 원격이면 `headless:=true rviz:=false foxglove:=true` |
| Gazebo 가 매우 느림 (RTF < 0.3) | `lite:=true`, `rows:=4`. NVIDIA PC 는 `nvidia-smi` 가 컨테이너 안에서 되는지 확인 |
| `PX4 로버 모델이 없습니다` | `cd $PX4_DIR && git submodule update --init Tools/simulation/gz` |
| PX4 로그 `Timed out waiting for Gazebo world` | Gazebo 서버가 늦게 뜸. launch 를 다시 실행하거나 `gz topic -l` 로 월드 이름(`orchard`) 확인 |
| `/fmu/out/...` 토픽이 없음 | `MicroXRCEAgent` 실행 여부, PX4 콘솔의 `uxrce_dds_client status`. 실차는 `UXRCE_DDS_CFG`, 보드레이트, TX/RX 교차 확인 |
| `/fmu/out/vehicle_status` 는 없고 `_v1` 만 있음 | 정상. PX4 v1.16 부터 버전이 붙은 메시지는 토픽 이름 뒤에 `_v1` 이 붙음 |
| px4_msgs 관련 역직렬화 오류 | PX4 펌웨어와 px4_msgs 버전 불일치. 둘 다 v1.17.0 인지 확인 |
| 시동이 안 걸림 | `ros2 topic echo /fmu/out/vehicle_status_v1` 의 arming 상태 확인. 시뮬은 GPS 수렴까지 10~20초 대기. 실차는 PX4 사전점검(QGC 메시지) 확인 |
| 로버가 출발하지 않음 | `/orchard/row` 의 `valid` 가 false → 통로 입구에 행 방향으로 놓였는지, LiDAR TF(`base_link→livox_frame`) 가 있는지 확인 |
| 줄기가 거의 안 잡힘 | 로그의 `ground=nominal` 이면 지면 추정 실패 → `ground.nominal_z` 확인. `trunk.band_min/max` 가 실제 줄기 높이와 맞는지 확인 |
| 잔디가 줄기로 잡힘 | `trunk.band_min` 을 잔디 최대 높이 + 10 cm 로 올림 |
| 옆 통로로 들어감 | 로그 `U턴 계획:` 의 다음 통로 출처(lidar2/lidar1/nominal)·간격 확인. nominal 이면 행 끝에서 줄기를 못 본 것. `row.expected_width` 가 실제 행간과 다른지, 간격/2 가 최소 회전반경(0.9 m)보다 작은지 확인 |
| 장애물이 아닌데 멈춤 | `/orchard/markers` 의 빨간 상자 위치 확인. 처진 가지(차체보다 높음)면 `obstacle.h_max` 를 차체 높이+여유로 낮추고(시뮬 0.75 m), 잡초면 `obstacle.h_min` 을 올리거나 `obstacle.half_width` 를 줄임 |
| RealSense 가 안 잡힘 (Orin) | USB3 포트 확인, 컨테이너가 `privileged` 인지 확인. 그래도 안 되면 RealSense 의 Jetson 설치 문서(RSUSB 백엔드)를 따름 |
| Livox 점군이 안 옴 | Orin 유선랜 IP 192.168.1.50/24, `ping 192.168.1.1XX`, 방화벽(UDP 56100~56501) 확인 |

도움 요청은 GitHub Issue(또는 팀 채널)에 **실행한 명령 + 오류 전문 + `git rev-parse --short HEAD`** 를 함께 올려 주세요. 코드 문제는 GitHub Issue 로 남깁니다.
