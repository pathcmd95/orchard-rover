# orchard_mapping — 복셀 맵 + 과수 줄기 위치 지도 (출발점 기준 상대좌표)

LiDAR 점군을 3D **복셀 맵**으로 쌓고, 카메라 **YOLO 줄기 검출 → 복셀 맵 위치 확인 → 매칭 → 상대좌표** 순서로
각 과수 줄기의 위치를 기록합니다. 출발점이 (0, 0, 0), 처음 곧게 달린 방향이 +x 입니다.

자세한 설명: [`docs/07_voxel_map.md`](../../../docs/07_voxel_map.md)

## 노드 `voxel_map_node`

| 구분 | 토픽 / 서비스 | 형식 |
|---|---|---|
| 입력 | `/livox/lidar` | PointCloud2 |
| 입력 | `/odom` | Odometry |
| 입력 | `/orchard/detections` | vision_msgs/Detection2DArray (YOLO) |
| 입력 | `/camera/color/camera_info` | CameraInfo |
| 입력 | `/orchard/trees` | TreeArray (YOLO 가 없을 때 대신) |
| 입력 | `/orchard/mission/state` | String (FOLLOW_ROW 에 원점, DONE 에 저장) |
| 출력 | `/orchard/voxel_map` | MarkerArray (높이 색 복셀, frame `start`) |
| 출력 | `/orchard/trunk_map` | MarkerArray (주황 = YOLO 확인 줄기, 회색 = LiDAR 만) |
| 출력 | `/orchard/start_path` | Path (출발점 기준 궤적) |
| 출력 | TF `odom → start` | 정적 |
| 서비스 | `/orchard/voxel_map/save` | std_srvs/Trigger |

저장: `data/voxel_maps/voxel_map_<날짜시각>/` — `voxels.ply`, `voxel_map.npz`, `trunks.csv`, `trajectory.csv`, `meta.json`

## 파일 지도

| 파일 | 하는 일 | 고칠 때 |
|---|---|---|
| `voxel_map.py` | 복셀 누적(`VoxelMap.integrate`), 지면 높이(`height_above_ground`), 저장(PLY/NPZ), 높이 색 | 복셀 크기, 광선 투사 추가, 색 |
| `start_frame.py` | 출발점 좌표계(`StartFrame`: 원점, +x = 이동 방향) | 원점 규칙, GPS 위경도 병기 |
| `trunk_locator.py` | 박스 → 방위각(`bbox_bearing`) → 쐐기 안 줄기(`TrunkLocator.locate`) | 줄기 높이 띠, 모양 판정 |
| `trunk_registry.py` | 매칭·확정(`TrunkRegistry.update`, `confirmed`), CSV | 매칭 거리, 칼만 필터로 바꾸기 |
| `voxel_map_node.py` | ROS 연결, TF, RViz 표시, 저장 | 토픽 이름, 저장 형식 |

## 시험

```bash
cd ros2_ws/src/orchard_mapping && python3 -m pytest -q test/
# 결과 그림: python3 tools/mapping/view_voxel_map.py data/voxel_maps/voxel_map_<날짜시각>
```

`test_mapping.py::test_pipeline_on_synthetic_orchard` — 가상 과수원에서 전체 흐름을 돌려 줄기 위치 오차 중앙값 < 8 cm 확인.

## 참고

OctoMap (Hornung et al., 2013) 의 복셀 개념을 단순화했습니다 (빈 공간 지우기 없음). [`docs/REFERENCES.md`](../../../docs/REFERENCES.md)
