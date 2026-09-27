# orchard_px4_bridge — ROS 2 ↔ PX4 로버 연결

ROS 쪽의 `/cmd_vel` 을 PX4 v1.17 Ackermann 로버 Offboard 명령으로 바꾸고,
PX4 의 자세·GPS·IMU 를 표준 ROS 메시지(`/odom`, `/gps/fix`, `/imu/data_raw`)로 바꿉니다.
PX4 와 ROS 2 사이는 Micro XRCE-DDS Agent(v2.4.3)가 이어 줍니다.

## 노드

| 노드 (실행 이름) | 입력 | 출력 |
|---|---|---|
| `offboard_node` (`px4_offboard_bridge`) | `/cmd_vel`, `/fmu/out/vehicle_odometry`, `/fmu/out/vehicle_status_v1` | `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint`, `/fmu/in/vehicle_command`. 서비스 `/orchard/px4/arm`, `/disarm`, `/offboard` |
| `state_node` (`px4_state_node`) | `/fmu/out/vehicle_odometry`, `/fmu/out/vehicle_gps_position`, `/fmu/out/sensor_combined` | `/odom`, TF `odom → base_link`, `/gps/fix`, `/imu/data_raw` |

## 꼭 알아야 할 것 (PX4 v1.17)

- 로버 Offboard 는 **속도 벡터**(TrajectorySetpoint.velocity, NED)만 받습니다. PX4 가 속도 = |v|, 목표 요 = atan2(vE, vN) 로 바꿔 자체 제어합니다.
  `RoverSpeedSetpoint` 를 직접 보내면 PX4 가 덮어써서 속도 0 이 됩니다 (Gazebo 에서 확인).
- 그래서 `cmd_vel(v, ω)` → 목표 요 = 현재 요 + ω·horizon → NED 속도 벡터 (`frames.cmd_vel_to_ned_velocity`).
  `yaw_horizon` = 1 / `RO_YAW_P` (에어프레임 51010 에서 3 s).
- 좌표: PX4 는 NED/FRD, ROS 는 ENU/FLU. 변환은 모두 `frames.py` 에 있습니다.
- 버전 붙은 토픽 이름: `/fmu/out/vehicle_status_v1`, `/fmu/out/vehicle_local_position_v1`.

## 파일 지도

| 파일 | 고칠 때 |
|---|---|
| `frames.py` | 좌표 변환, cmd_vel → NED 속도 (`cmd_vel_to_ned_velocity`, `ned_to_enu`, `attitude_ned_frd_to_enu_flu`) |
| `offboard_node.py` | 시동·모드 전환 순서, 명령 끊김(`cmd_timeout`) 처리, 속도 제한 |
| `state_node.py` | odom/TF 발행, GPS·IMU 변환 |

파라미터: `orchard_bringup/config/*.yaml` 의 `px4_offboard_bridge`, `px4_state_node`. 실차는 `auto_arm: false` (사람이 시동).

## 시험

```bash
cd ros2_ws/src/orchard_px4_bridge && python3 -m pytest -q test/
```

## 참고

PX4 User Guide "Offboard Mode", "ROS 2 User Guide", ROS REP-103. [`docs/REFERENCES.md`](../../../docs/REFERENCES.md)
