# orchard_navigation — 행 순회 자율주행 + 나무 지도

인식 결과(통로 중심선·줄기)를 받아 **어디로 얼마나 빨리 갈지**(`/cmd_vel`)를 정하고,
주행과 나란히 **나무 지도**(열·나무 번호·결주)를 만듭니다.

## 노드

| 노드 | 입력 | 출력 | 설명 |
|---|---|---|---|
| `row_navigator_node` | `/orchard/row`, `/orchard/seg/row`, `/odom`, `/orchard/trees` | `/cmd_vel`, `/orchard/mission/state`, `/orchard/turn_path` | 행 추종 → 행 끝 → U턴 → 다음 통로 → … → DONE. 서비스 `/orchard/mission/start`, `/stop` |
| `orchard_mapper_node` | `/orchard/trees`, `/odom`, `/gps/fix`, `/orchard/mission/state` | `/orchard/map/markers`, `data/maps/orchard_map_*/` | 줄기를 odom 에 누적 → 열 나누기 → 나무 번호·결주·기둥 → CSV/JSON(위경도 포함) |

## 미션 상태기계 (`mission.py`)

```
IDLE ─start→ FOLLOW_ROW ─행 끝→ EXIT_ROW ─exit_distance 주행→ TURN ─180°→ ENTER_ROW ─행 인식→ FOLLOW_ROW …
                                    └─ 마지막 통로면(또는 탐사 모드에서 끝 열 도달) ─────────→ DONE
어느 상태든 stop() → STOPPED,  ENTER_ROW 에서 max_enter_distance 안에 통로를 못 찾으면 → STOPPED(안전정지)
통로 안 장애물은 상태를 바꾸지 않고 controller.obstacle_speed_factor 로 감속·정지
```

상태 문자열: `/orchard/mission/state` = `상태|lanes_done=N|이유|row=lidar|fused|camera`.

## 파일 지도 — "이걸 고치려면 이 파일"

| 고치고 싶은 것 | 파일 | 핵심 |
|---|---|---|
| 통로 안에서 좌우로 흔들림, 속도 | `controller.py` | `pure_pursuit_curvature`, `row_follow_command`, `FollowParams`, `obstacle_speed_factor` |
| 행 끝 판단, U턴 시점, 상태 전이 | `mission.py` | `OrchardMission.step`, `_plan_turn`, `add_trees`, `MissionParams` |
| U턴 경로 모양·반경, 다음 통로 찾기 | `headland.py` | `estimate_next_lane`, `dubins_path`, `PathTracker` |
| LiDAR·카메라 중심선 섞는 비율 | `fusion.py` | `fuse_rows` |
| 나무 지도 정리(열·번호·결주·기둥) | `treemap.py` | `TreeMap.organize`, `_row_peaks`, `_split_posts` |
| 지도 번짐(방위·시각 보정) | `treemap.py` | `YawBiasEstimator`, `PoseHistory`, `best_time_offset` |
| ROS 토픽·파라미터 연결 | `row_navigator_node.py`, `orchard_mapper_node.py` | |

### 행 끝에서 다음 통로를 찾는 방법 (`mission.turn_mode: planned`)

1. 행 끝에 오면 지금까지 본 줄기(`/orchard/trees`)를 odom 에 모아 **가까운 열**과 **그 너머 열**의 직선을 구합니다.
2. 두 열의 가운데 = 다음 통로 중심선, 마지막 나무 + 0.8 m = 진입점 (`estimate_next_lane`).
3. 현재 자세 → 진입점까지 **Dubins 경로**(최소 회전반경 = 측정한 행간/2, 최소 0.9 m)를 만들고 Pure Pursuit 로 따라갑니다.
4. 거의 돌아서(π−0.6 rad) LiDAR 가 새 통로를 보면 LiDAR 중심선으로 넘겨받습니다 (odom 이 U턴 중 0.5~1 m 밀리기 때문).
5. 줄기를 못 봤으면 예전 방식(`arc`: 반경 = 행간/2 반원)으로 돌아갑니다.

## 파라미터

`orchard_bringup/config/sim.yaml` / `robot.yaml` 의 `row_navigator_node`(mission.*, follow.*), `orchard_mapper_node`.
여기서 선언하지 않은 `MissionParams` 항목은 `mission.py` 기본값을 고쳐야 바뀝니다.

## 시험

```bash
cd ros2_ws/src/orchard_navigation && python3 -m pytest -q test/
```

- `test_closed_loop.py` — 가상 LiDAR + 단순 차량 모델로 3통로 순회를 끝까지 돌려 봄 (odom 드리프트·U턴 미끄러짐 포함)
- `test_headland.py` — Dubins 경로·다음 통로 추정
- `test_treemap.py` — 지도 정리·보정

## 공부할 자료

Pure Pursuit (Coulter 1992), Dubins (1957), Shkel & Lumelsky (2001) — [`docs/REFERENCES.md`](../../../docs/REFERENCES.md).
