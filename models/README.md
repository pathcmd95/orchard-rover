# models

시뮬레이션 데이터로 학습한 시험용 모델입니다.

| 파일 | 용도 |
|---|---|
| `trunk_yolo.onnx` | 카메라 YOLO 줄기 검출 (입력 320, `camera_tree_node.input_size`) |
| `orchard_seg.onnx` | 카메라 주행가능 영역 세그멘테이션 |

새로 학습한 모델은 같은 이름으로 바꿔 넣으면 됩니다. 학습 방법은 `docs/04_perception_training.md` 참고.

라이선스: `trunk_yolo.onnx` 는 Ultralytics YOLO11n 을 학습한 모델이라 **AGPL-3.0** (https://ultralytics.com/license, 전문 `LICENSE-AGPL-3.0.txt`) 을 따릅니다. `orchard_seg.onnx` 는 torchvision(BSD-3-Clause) 구조로 학습한 모델입니다. 제3자 구성요소 전체는 저장소 루트 `NOTICE` 참고. 저장소의 나머지 코드는 Apache-2.0.
