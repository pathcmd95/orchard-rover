# orchard_perception — 과수 인식 (LiDAR 줄기·행 중심선, 카메라 YOLO·세그멘테이션)

센서 데이터를 받아 **"나무가 어디 있나"**, **"통로 가운데가 어디인가"** 를 계산합니다.
계산은 ROS 없는 순수 파이썬 모듈에 있고, `*_node.py` 는 ROS 입출력만 맡습니다 → 모듈은 pytest 로 바로 시험할 수 있습니다.

## 노드

| 노드 | 입력 | 출력 | 한 줄 설명 |
|---|---|---|---|
| `lidar_tree_node` | `/livox/lidar` | `/orchard/trees`, `/orchard/row`, `/orchard/markers` | 3D 점군 → 지면(RANSAC) → 줄기 → 좌/우 열 → 통로 중심선 + 통로 장애물 |
| `camera_tree_node` | `/camera/color/image_raw` | `/orchard/detections`, `/orchard/debug/detections` | YOLO ONNX 줄기 박스 (모델 없으면 꺼짐) |
| `tree_fusion_node` | `/orchard/trees`, `/orchard/detections`, 영상 | `/orchard/trees_fused`, `/orchard/debug/fusion`, (학습 데이터) | LiDAR 줄기를 영상에 투영해 YOLO 와 맞춤, **YOLO 학습 라벨 자동 생성** |
| `seg_path_node` | 영상 + camera_info | `/orchard/seg/row`, `/orchard/seg/labels`, `/orchard/debug/seg` | 주행가능 영역 세그멘테이션 → 지면 역투영 → 카메라 중심선 |
| `seg_dataset_node` | 영상 + Gazebo 정답 라벨 | `data/seg_dataset_sim/` | (시뮬 전용) 세그멘테이션 학습 데이터 수집 |

## 파일 지도 — "이걸 고치려면 이 파일"

| 고치고 싶은 것 | 파일 | 핵심 함수/클래스 |
|---|---|---|
| 지면 추정이 기울어짐, 경사지 대응 | `ground.py` | `fit_ground_plane`, `GroundPlane` |
| 잔디·가지를 줄기로 착각 / 줄기를 놓침 | `trunks.py` | `select_band`(높이 띠), `cluster_xy`, `detect_trunks` |
| 통로 중심선이 흔들림, 한쪽 열만 보일 때 | `rows.py` | `fit_rows`, `estimate_heading`, `RowTracker`(평활), `corridor_obstacle_distance` |
| LiDAR ↔ 카메라 박스가 어긋남 | `projection.py` | `project`, `trunk_box`, `greedy_match`, `yolo_label` |
| YOLO 후처리(점수·NMS·letterbox) | `yolo.py` | `YoloOnnxDetector`, `decode_output` |
| 세그멘테이션 클래스 추가 | `seg_classes.py` (+ `orchard_gazebo/world_gen.py` LABELS 도 같이) | `CLASSES` |
| 세그멘테이션 → 중심선 | `seg_path.py` | `ground_points`, `centerline` |
| Gazebo 없이 가짜 점군 만들기 (시험용) | `synthetic.py` | `SyntheticOrchard`, `scan` |
| 좌표 변환 도구 | `geometry.py` | `make_transform`, `transform_points`, `wrap_angle` |

## 파라미터

`orchard_bringup/config/sim.yaml`(시뮬) / `robot.yaml`(실차) 의 같은 이름 절. 실차에서 **반드시** 맞출 것:
`lidar_tree_node.ground.nominal_z`(base_link 높이), `self_filter.*`(차체 크기), `seg_path_node.ground_z`.

## 시험

```bash
cd ros2_ws/src/orchard_perception && python3 -m pytest -q test/
```

`test_perception_nodes.py` 처럼 ROS 가 필요한 시험은 컨테이너 안에서만 돌고, 밖에서는 건너뜁니다.

## 공부할 자료

RANSAC (Fischler & Bolles 1981), 핀홀 카메라 (Hartley & Zisserman 2004), YOLO (Redmon et al. 2016),
MobileNetV3 / LR-ASPP (Howard et al. 2019) — 자세한 서지는 [`docs/REFERENCES.md`](../../../docs/REFERENCES.md).
학습 절차: [`docs/04_perception_training.md`](../../../docs/04_perception_training.md).
