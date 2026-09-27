# 참고 자료: 외부 코드 저장소 · 논문

이 저장소가 가져다 쓰는 외부 코드와 알고리즘의 원문 논문을 정리했습니다.
공부할 때는 **"이 파일 → 이 논문/저장소"** 순서로 따라가 보세요. 각 소스 파일 맨 위 설명(docstring)의 `References` 에도 같은 내용이 있습니다.

## 1. 외부 코드 저장소 (Docker 빌드 때 원본에서 받음, 이 저장소에 복사하지 않음)

| 이름 | 버전 | 원문 저장소 | 라이선스 | 어디서 쓰나 |
|---|---|---|---|---|
| PX4-Autopilot | v1.17.0 | https://github.com/PX4/PX4-Autopilot | BSD-3-Clause | 로버 오토파일럿 (SITL·Pixhawk 펌웨어). `docker/scripts/install_sim.sh`, `scripts/build_px4_firmware.sh` |
| PX4 Gazebo 모델 (`rover_ackermann`) | PX4 v1.17.0 에 포함 | https://github.com/PX4/PX4-gazebo-models | BSD-3-Clause | `orchard_gazebo/model_gen.py` 가 읽어 LiDAR·카메라를 붙인 모델 생성 |
| px4_msgs | v1.17.0 | https://github.com/PX4/px4_msgs | BSD-3-Clause | PX4 ↔ ROS 2 메시지. `orchard_px4_bridge` |
| Micro XRCE-DDS Agent | v2.4.3 | https://github.com/eProsima/Micro-XRCE-DDS-Agent | Apache-2.0 | PX4 uORB ↔ ROS 2 DDS 연결 |
| Livox-SDK2 | v1.3.1 | https://github.com/Livox-SDK/Livox-SDK2 | MIT | Mid-360 통신 (실차). GCC 13 은 `-include cstdint` 필요 |
| livox_ros_driver2 | 1.2.6 | https://github.com/Livox-SDK/livox_ros_driver2 | MIT | Mid-360 → `/livox/lidar` (실차) |
| realsense-ros (`ros-jazzy-realsense2-camera`) | Jazzy apt | https://github.com/IntelRealSense/realsense-ros | Apache-2.0 | D455 → `/camera/color/*` (실차) |
| ros_gz (`ros_gz_bridge`, `ros_gz_sim`) | Jazzy apt | https://github.com/gazebosim/ros_gz | Apache-2.0 | Gazebo ↔ ROS 2 토픽 연결 (시뮬) |
| Gazebo Harmonic | apt | https://github.com/gazebosim/gz-sim | Apache-2.0 | 시뮬레이터 |
| ROS 2 Jazzy | apt | https://github.com/ros2/ros2 | Apache-2.0 | 미들웨어 |
| foxglove_bridge | Jazzy apt | https://github.com/foxglove/ros-foxglove-bridge | MIT | 맥·윈도에서 원격 보기 (선택) |
| onnxruntime | ≥1.20 (pip) | https://github.com/microsoft/onnxruntime | MIT | YOLO·세그멘테이션 ONNX 추론 (`yolo.py`, `seg_model.py`) |
| Ultralytics | 8.x (학습 PC 에서 pip) | https://github.com/ultralytics/ultralytics | AGPL-3.0 | YOLO 학습·ONNX 내보내기 (`tools/training/train_yolo.py`). **학습 도구로만** 쓰고 코드는 포함하지 않음. 학습한 모델 `models/trunk_yolo.onnx` 는 AGPL-3.0 |
| PyTorch / torchvision | 학습 PC 에서 pip | https://github.com/pytorch/pytorch , https://github.com/pytorch/vision | BSD-3-Clause | 세그멘테이션 LR-ASPP MobileNetV3 학습 (`tools/training/train_segmentation.py`) |
| trimesh (+ embreex) | pip | https://github.com/mikedh/trimesh | MIT | Gazebo 없이 월드 메시에 광선을 쏴 LiDAR 흉내 (`tools/sim_checks/world_lidar_check.py`) |
| OpenCV (`python3-opencv`) | Ubuntu 24.04 apt | https://github.com/opencv/opencv | Apache-2.0 | 영상 처리, NMS |

> 이 저장소의 코드(`ros2_ws/src/orchard_*`, `tools/`, `scripts/`)는 팀이 직접 작성했습니다.
> 외부 저장소에서 **코드를 복사해 온 파일은 없고**, 위 저장소들은 Docker 빌드·pip 설치로 받아 **라이브러리로 호출**합니다.
> 예외로 설정 파일 2개(PX4 에어프레임, Livox MID360 설정)는 원본을 수정한 파생 파일이며 원본 라이선스를 따릅니다 ([third_party/README.md](../third_party/README.md)).
> 알고리즘을 논문 식에서 직접 구현한 곳은 아래 2절에 파일별로 적었습니다.

## 2. 알고리즘 · 논문 (파일별)

| 파일 | 알고리즘 | 논문 / 자료 |
|---|---|---|
| `orchard_perception/ground.py` | 지면 평면 추정 RANSAC | M. A. Fischler, R. C. Bolles, "Random Sample Consensus: A Paradigm for Model Fitting with Applications to Image Analysis and Automated Cartography", *Communications of the ACM* 24(6), 1981 |
| `orchard_perception/projection.py`, `orchard_mapping/trunk_locator.py` (`bbox_bearing`) | 핀홀 카메라 투영·역투영 | R. Hartley, A. Zisserman, *Multiple View Geometry in Computer Vision*, 2nd ed., Cambridge Univ. Press, 2004 |
| `orchard_perception/yolo.py`, `camera_tree_node.py` | YOLO 검출 후처리 (letterbox, NMS) | J. Redmon, S. Divvala, R. Girshick, A. Farhadi, "You Only Look Once: Unified, Real-Time Object Detection", CVPR 2016 · 모델 구조는 Ultralytics YOLO11 (G. Jocher, J. Qiu, 2024, 위 저장소) |
| `orchard_perception/seg_model.py`, `tools/training/train_segmentation.py` | LR-ASPP + MobileNetV3-Small 세그멘테이션 | A. Howard et al., "Searching for MobileNetV3", ICCV 2019 |
| `orchard_navigation/controller.py`, `headland.PathTracker` | Pure Pursuit 경로 추종 | R. C. Coulter, "Implementation of the Pure Pursuit Path Tracking Algorithm", CMU-RI-TR-92-01, 1992 |
| `orchard_navigation/headland.py` (`dubins_path`) | Dubins 최단 경로 (최소 회전반경) | L. E. Dubins, "On Curves of Minimal Length with a Constraint on Average Curvature, and with Prescribed Initial and Terminal Positions and Tangents", *American Journal of Mathematics* 79(3), 1957 |
| `orchard_navigation/headland.py` (`_dubins_words`) | Dubins 6가지 경로(LSL/RSR/LSR/RSL/RLR/LRL) 닫힌 식 | A. M. Shkel, V. Lumelsky, "Classification of the Dubins set", *Robotics and Autonomous Systems* 34(4), 2001 |
| `orchard_planning/hybrid_astar.py` | Hybrid A* (차량 회전반경을 지키는 격자 탐색) | D. Dolgov, S. Thrun, M. Montemerlo, J. Diebel, "Path Planning for Autonomous Vehicles in Unknown Semi-structured Environments", *International Journal of Robotics Research* 29(5), 2010 |
| `orchard_planning/dwa.py` | Dynamic Window Approach (지역 장애물 회피) | D. Fox, W. Burgard, S. Thrun, "The Dynamic Window Approach to Collision Avoidance", *IEEE Robotics & Automation Magazine* 4(1), 1997 |
| `orchard_planning/global_planner.py` | 통로 지그재그 순회 (boustrophedon 커버리지) | H. Choset, "Coverage of Known Spaces: The Boustrophedon Cellular Decomposition", *Autonomous Robots* 9, 2000 |
| `orchard_mapping/voxel_map.py` | 복셀 점유 지도 (개념을 단순화) | A. Hornung, K. M. Wurm, M. Bennewitz, C. Stachniss, W. Burgard, "OctoMap: An Efficient Probabilistic 3D Mapping Framework Based on Octrees", *Autonomous Robots* 34(3), 2013 |
| `orchard_gazebo/terrain.py` (`_pebble_field`) | 공간 해시 | M. Teschner et al., "Optimized Spatial Hashing for Collision Detection of Deformable Objects", VMV 2003 |
| `orchard_px4_bridge/frames.py` | NED/FRD ↔ ENU/FLU 좌표 변환 | ROS REP-103 "Standard Units of Measure and Coordinate Conventions" (https://www.ros.org/reps/rep-0103.html), PX4 문서 "ROS 2 User Guide – Frame conventions" |

## 3. 더 공부할 거리 (학생 개선 과제와 연결)

| 과제 | 참고 |
|---|---|
| U턴 뒤 지도 어긋남 → LiDAR-관성 오도메트리 | W. Xu, Y. Cai, D. He, J. Lin, F. Zhang, "FAST-LIO2: Fast Direct LiDAR-Inertial Odometry", *IEEE Transactions on Robotics* 38(4), 2022 · https://github.com/hku-mars/FAST_LIO (Mid-360 지원) |
| 복셀 맵에서 빈 공간 지우기(광선 투사) | OctoMap 논문(위) · https://github.com/OctoMap/octomap |
| 좁은 헤드랜드에서 전진·후진 회전 | J. A. Reeds, L. A. Shepp, "Optimal paths for a car that goes both forwards and backwards", *Pacific Journal of Mathematics* 145(2), 1990 |
| 행 추종 대안: MPC | 예) acados (https://github.com/acados/acados) 로 운동학 자전거 모델 MPC |
| 지역 계획 대안: TEB, MPPI | C. Rösmann et al., "Trajectory modification considering dynamic constraints of autonomous robots", ROBOTIK 2012 (TEB) · G. Williams et al., "Information Theoretic MPC for Model-Based Reinforcement Learning", ICRA 2017 (MPPI) · Nav2 구현 https://github.com/ros-navigation/navigation2 |
| 전역 경로·장애물 회피 | Nav2 (https://github.com/ros-navigation/navigation2) |
| 과수원 행 인식 연구 동향 | 농업 로봇 행 추종 리뷰 논문을 "orchard row following LiDAR" 등으로 검색 |
