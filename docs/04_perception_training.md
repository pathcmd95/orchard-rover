# 04. 과수 인식: 자동 라벨링 → YOLO 학습 → 배포

카메라만으로는 거리를 알기 어렵고, LiDAR 만으로는 "무엇인지"를 알기 어렵습니다. 이 저장소는 두 센서를 이렇게 나눠 씁니다.

| 단계 | 담당 | 결과 |
|---|---|---|
| 줄기 위치(거리·방향) | LiDAR (`lidar_tree_node`) | `/orchard/trees` |
| 줄기 → 영상 박스 | 보정값(TF)으로 투영 (`tree_fusion_node`) | 학습 라벨 자동 생성 |
| 영상에서 줄기 인식 | YOLO (`camera_tree_node`) | `/orchard/detections` |
| 두 결과 일치 여부 | `tree_fusion_node` | `camera_confirmed` |

즉, **LiDAR 가 카메라 학습 데이터를 만들어 주는 구조**입니다. 사람이 박스를 그리지 않아도 시뮬레이션과 현장에서 데이터를 모을 수 있습니다.

## 1. 데이터 모으기

시뮬레이션:

```bash
ros2 launch orchard_bringup sim.launch.py record_dataset:=true seed:=1
# seed, rows, lite 를 바꿔 여러 번 실행하면 배경이 다양해진다
```

실차 (주행 없이 손으로 밀거나 조종기로 천천히):

```bash
ros2 launch orchard_bringup robot.launch.py record_dataset:=true
```

저장 위치는 `sim.yaml` / `robot.yaml` 의 `dataset_dir` (기본 `/workspace/data/...`) 이고, `images/*.jpg` 와 `labels/*.txt`(YOLO 형식) 가 쌍으로 쌓입니다. `record_every_n`(기본 5 프레임마다 1장)으로 양을 조절합니다.

### 라벨 품질 확인

- 투영 박스는 지면부터 `trunk_label_height`(0.8 m, 첫 가지 높이)까지입니다. 품종마다 다르면 조정합니다.
- 9 m 보다 먼 줄기는 라벨에서 뺍니다 (`max_label_distance`).
- 저장 전 `/orchard/debug/fusion` 영상에서 박스가 줄기에 잘 맞는지 꼭 확인합니다. 어긋나면 `rover.yaml` 의 카메라 장착값부터 고칩니다.
- 현장 데이터는 몇십 장이라도 사람이 직접 검수해 잘못된 박스를 지웁니다 (예: [labelImg](https://github.com/HumanSignal/labelImg), Roboflow 등).

## 2. 학습 (GPU PC)

```bash
python3 -m venv ~/yolo && source ~/yolo/bin/activate
pip install ultralytics
python3 tools/training/train_yolo.py --data data/orchard_dataset_sim --epochs 80
# 현장 데이터가 생기면 두 폴더를 합쳐 다시 학습 (sim 만으로 학습하면 실제 영상에서 성능이 떨어짐)
```

결과: `models/trunk_yolo.onnx` (mAP50 이 로그에 출력됨)

`--imgsz` 는 추론 때 `camera_tree_node.input_size` 와 같아야 합니다. 시뮬을 `camera_lite:=true`(320×240)로 모았다면 `--imgsz 320`
(sim.yaml 의 `input_size` 도 320), 실차 D455 640×480 이면 640.
들어 있는 모델(`models/trunk_yolo.onnx`): 시뮬 117장, imgsz 320, 60 epoch → mAP50 0.84. 실사용에는 현장 데이터 수백 장 이상이 필요합니다.

## 3. 배포

```bash
ros2 launch orchard_bringup sim.launch.py camera_model:=/workspace/models/trunk_yolo.onnx camera_lite:=true
```

YOLO 박스는 `voxel_map_node` 가 받아 복셀 맵에서 줄기 위치를 확인하고 출발점 기준 좌표로 기록합니다 ([07](07_voxel_map.md)).

실차는 `robot.yaml` 의 `camera_tree_node.model_path` 를 바꾸거나 같은 인자를 줍니다. 추론은 onnxruntime(CPU)으로 하며 없으면 OpenCV DNN 을 씁니다(`backend` 파라미터). Orin 의 CPU 로 640 입력 기준 수 FPS 수준이므로 `max_rate_hz: 5.0` 으로 제한해 두었습니다. 더 빠르게 하려면 TensorRT 변환을 다음 단계로 검토합니다.

## 4. 평가 지표

| 지표 | 계산 |
|---|---|
| 검출 정확도 | YOLO 검증셋 mAP50 / mAP50-95 (학습 로그) |
| LiDAR-카메라 일치율 | `/orchard/trees_fused` 중 `camera_confirmed` 비율 (영상 안에 보이는 줄기 기준) |
| 도메인 차이 | 같은 모델로 sim 검증셋과 현장 검증셋 mAP 를 따로 계산해 비교 |

## 5. 주행가능 영역 세그멘테이션 (카메라)

줄기 하나하나 대신 **과수열·통로 전체를 인식**하는 부분입니다. 카메라 영상의 모든 픽셀을 아래 7개 클래스로 나누고, **주행가능** 픽셀을 지면에 역투영해 통로 중심선을 구한 뒤 LiDAR 중심선과 융합합니다.

| 번호 | 클래스 | 시뮬에서 붙는 대상 |
|---|---|---|
| 0 | 미분류/하늘 | 하늘, 로버 자신 |
| 1 | 주행가능 | 통로 지면, 자갈 농로, 풀, 자갈, 바깥 풀밭 |
| 2 | 나무 밑 띠 | 두둑 위 제초 띠, 낙과 |
| 3 | 나무 | 줄기, 가지, 잎, 사과 |
| 4 | 구조물 | 지주, 콘크리트 기둥, 철선, 점적 호스 |
| 5 | 장애물 | 바위, 사람, 운반차 |
| 6 | 배경 | 과수원 밖 숲 |

### 5-1. 데이터 모으기 (라벨링 없이)

Gazebo 세그멘테이션 카메라가 픽셀마다 정답 라벨을 만들어 줍니다.

```bash
ros2 launch orchard_bringup sim.launch.py record_seg:=true explore:=true seed:=3
# → data/seg_dataset_sim/images/*.jpg, labels/*.png (시뮬 시간 1초마다 1장)
```
seed, relief, lite 를 바꿔 여러 번 모으면 배경이 다양해집니다. 현장 영상은 CVAT 등으로 같은 형식(라벨 PNG 픽셀값 = 번호)으로 라벨링합니다.

### 5-2. 학습 → ONNX

```bash
pip install torch torchvision onnx
python3 tools/training/train_segmentation.py --data data/seg_dataset_sim --out models/orchard_seg.onnx
# 현장 데이터로 미세조정
python3 tools/training/train_segmentation.py --data data/seg_dataset_sim data/seg_field --init models/orchard_seg.pt --epochs 20 --lr 3e-4
```
모델은 LR-ASPP + MobileNetV3-Small (입력 320×240). 검증 IoU 표가 마지막에 나옵니다.

### 5-3. 주행에 쓰기

`models/orchard_seg.onnx` 가 있으면 `seg_path_node` 가 자동으로 켜져 `/orchard/seg/row` 를 내고, 주행 노드가 LiDAR 중심선과 융합합니다 (`/orchard/mission/state` 끝의 `row=fused|lidar|disagree`).
- 융합 원칙: 행 끝·U턴 판단은 LiDAR 만, 둘 다 유효하고 비슷할 때만 가중 평균, 크게 다르면 LiDAR 사용 (`orchard_navigation/fusion.py`)
- 모델 없이 기하 계산만 시험: `sim.launch.py seg_gt:=true` (Gazebo 정답 라벨로 중심선 계산)
- RViz 'Segmentation' 패널: 라벨 색 + 노란 중심선

## 6. 확장: OOD

- OOD: YOLO 가 모르는 물체가 통로 안에 있으면 `obstacle_distance` 는 LiDAR 로 이미 잡히므로, 카메라 쪽 OOD 점수는 "정지할지"가 아니라 "무엇인지"를 기록하는 보조 정보로 쓰는 것이 안전합니다.
