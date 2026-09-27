# orchard_msgs — 이 프로젝트 전용 ROS 2 메시지

| 메시지 | 발행 | 내용 |
|---|---|---|
| `Tree` | (TreeArray 안) | 줄기 하나: 위치(z = 그 위치 지면 높이), 반경, 높이, 점 수, 신뢰도, 카메라 확인 여부, 좌/우 열 |
| `TreeArray` | `lidar_tree_node` → `/orchard/trees`, `tree_fusion_node` → `/orchard/trees_fused` | Header + Tree[] (frame 보통 base_link) |
| `RowCenterline` | `lidar_tree_node` → `/orchard/row`, `seg_path_node` → `/orchard/seg/row` | 통로 중심선 `y = lateral_offset + tan(heading_error)·x` (base_link), 행 폭, 신뢰도, 좌/우 줄기 수, 행 끝(last_tree_ahead), 통로 장애물 거리 |

필드 설명은 `msg/*.msg` 의 주석에 있습니다.

## 메시지를 바꾸거나 추가할 때

1. `msg/새이름.msg` 작성 → `CMakeLists.txt` 의 `rosidl_generate_interfaces` 목록에 추가
2. `./scripts/build_ws.sh` 로 다시 빌드 (메시지를 쓰는 모든 패키지도 다시 빌드됨)
3. 필드를 바꾸면 발행·구독하는 노드를 모두 찾아 고칩니다: `grep -rn "RowCenterline" ros2_ws/src`
