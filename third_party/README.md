# third_party — 제3자 라이선스 고지

이 저장소의 코드는 Apache-2.0 (`LICENSE`) 입니다. 아래 파일은 예외로, 원본의 라이선스를 그대로 따릅니다.

## 저장소에 들어 있는 파생 파일·모델

| 파일 | 원본 | 라이선스 | 전문 |
|---|---|---|---|
| `px4/airframes/51010_gz_orchard_rover` | PX4-Autopilot v1.17.0 `ROMFS/px4fmu_common/init.d-posix/airframes/51000_gz_rover_ackermann` 을 수정 | BSD-3-Clause (Copyright (c) 2012 - 2023, PX4 Development Team) | `licenses/PX4-Autopilot-BSD-3-Clause.txt` |
| `ros2_ws/src/orchard_bringup/config/MID360_config.json` | livox_ros_driver2 1.2.6 `config/MID360_config.json` 에서 IP 만 수정 | MIT (Copyright (c) 2022 Livox) | `licenses/livox_ros_driver2-MIT.txt` |
| `models/trunk_yolo.onnx` | Ultralytics YOLO11n 을 시뮬레이션 데이터로 학습해 ONNX 로 내보낸 모델 | AGPL-3.0 | `../models/LICENSE-AGPL-3.0.txt` |
| `models/orchard_seg.onnx` | torchvision (BSD-3-Clause) 의 LR-ASPP + MobileNetV3-Small 구조로 직접 학습 | 구조·사전학습 가중치는 torchvision BSD-3-Clause, 학습 결과는 이 저장소 | https://github.com/pytorch/vision/blob/main/LICENSE |

## Ultralytics YOLO 모델 (AGPL-3.0) 사용 시 주의

- Ultralytics 는 자사 도구로 학습한 YOLO 모델이 기본적으로 AGPL-3.0 을 따르고, 준수하려면 그 모델을 쓰는 더 큰 애플리케이션까지 대응 소스 전체를 공개해야 한다는 입장입니다 (https://www.ultralytics.com/license).
- 이 저장소와 Docker 이미지(`sim-full`)는 이 모델과 그것을 불러 쓰는 코드를 함께 배포하므로, **함께 배포되는 전체에는 AGPL-3.0 조건도 적용되는 것으로 보고** 대응 소스(이 저장소 전체와 학습 스크립트 `tools/training/train_yolo.py`)를 공개해 둡니다. Apache-2.0 코드 파일 자체의 라이선스는 바뀌지 않습니다.
- 모델은 선택 기능입니다. 시뮬레이션 기본 설정은 쓰지 않고 (`camera_model:=` 로 켤 때만), 실로봇 설정 `config/robot.yaml` 은 기본으로 씁니다. 모델 없이도 LiDAR 줄기 검출로 동작합니다.
- 이 모델이나 Ultralytics 로 새로 학습한 모델을 **비공개·상용 제품, 로봇 등 장비 탑재, 온라인 서비스**에 쓰려면 Ultralytics Enterprise 라이선스를 받거나 AGPL 이 아닌 검출 모델로 바꾸세요.

## Docker 빌드 때 원본 저장소에서 받는 것 (저장소에는 없음)

PX4-Autopilot · PX4-gazebo-models · px4_msgs (BSD-3-Clause), Micro-XRCE-DDS-Agent (Apache-2.0), ROS 2 Jazzy · Gazebo Harmonic (Apache-2.0), onnxruntime (MIT), Livox-SDK2 · livox_ros_driver2 (MIT), realsense-ros (Apache-2.0).
각 라이선스 전문은 원본 저장소에 있고, Docker 이미지 안에서는 각 소스 폴더나 `/usr/share/doc` 에 있습니다. 버전·링크는 `docs/REFERENCES.md`.
