# tools — ROS 밖에서 돌리는 도구 (학습·점검·지도 보기)

| 폴더/파일 | 하는 일 | 실행 예 |
|---|---|---|
| `training/train_yolo.py` | 자동 라벨링 데이터 → YOLO 줄기 검출 학습 → ONNX (`models/trunk_yolo.onnx`) | `python3 tools/training/train_yolo.py --data data/orchard_dataset_sim --epochs 80 --imgsz 320` |
| `training/train_segmentation.py` | 세그멘테이션(LR-ASPP MobileNetV3) 학습 → ONNX (`models/orchard_seg.onnx`) | `python3 tools/training/train_segmentation.py --help` |
| `sim_checks/world_lidar_check.py` | Gazebo 없이 생성 월드 메시에 LiDAR 광선을 쏴 지면·줄기·행 인식 점검 (trimesh) | `python3 tools/sim_checks/world_lidar_check.py` |
| `sim_checks/render_demo.py` | 알고리즘 데모 그림·영상 (Gazebo·PX4 없이) | `python3 tools/sim_checks/render_demo.py` |
| `mapping/view_voxel_map.py` | 저장된 복셀 맵 + 줄기 지도 → PNG (위에서 본 지도 + 3D), 시뮬 정답과 오차 | `python3 tools/mapping/view_voxel_map.py data/voxel_maps/voxel_map_<날짜시각>` |
| `mapping/trunk_table.py` | 예전 결과 폴더의 줄기를 열별 표로 (csv·html·md, `--truth` 로 정답 오차) | `python3 tools/mapping/trunk_table.py data/voxel_maps/voxel_map_<날짜시각>` |
| `mapping/world_map.py` | 시뮬 월드 정답 지도 PNG (나무·결주·열/통로 번호·장애물·출발 자세) + 결과 궤적·줄기 겹침 | `python3 tools/mapping/world_map.py orchard_meta.json --out <앞부분> [--voxel <복셀 맵 폴더> --timeline <timeline.txt>]` |

학습 도구는 GPU PC 의 가상환경에서 권장합니다 (`pip install ultralytics` / `torch torchvision`). 절차는 [docs/04](../docs/04_perception_training.md).

`--imgsz` 는 추론할 때 `camera_tree_node.input_size` 와 같아야 합니다 (시뮬 `camera_lite` 320×240 → 320).
