# orchard_planning — 전역·지역 경로계획

`nav_mode:=planner` (시뮬 기본)일 때 주행을 맡는 패키지입니다.
전역 계획이 **과수원 전체를 도는 경로**를 만들고, 지역 계획이 그 경로를 따라가며 **장애물을 피하는 `/cmd_vel`** 을 냅니다.
예전 방식(통로 중심선 Pure Pursuit + 상태기계, `orchard_navigation/row_navigator_node`)은 `nav_mode:=row` 로 그대로 쓸 수 있습니다.

자세한 설명: [`docs/09_path_planning.md`](../../../docs/09_path_planning.md)

## 노드

| 노드 | 입력 | 출력 |
|---|---|---|
| `global_planner_node` | `/odom`, `/orchard/trees`, `/orchard/row` | `/orchard/global_path`, `/orchard/mission/state`, `/orchard/plan/markers`, `/orchard/plan/turn_grid`. 서비스 `/orchard/mission/start`, `/stop` |
| `local_planner_node` | `/orchard/global_path`, `/orchard/mission/state`, `/odom`, `/livox/lidar` | `/cmd_vel`, `/orchard/local_path`, `/orchard/local/markers`, `/orchard/local/status` |

## RViz 에서 보이는 것

| 표시 (RViz Displays 이름) | 토픽 | 모양 |
|---|---|---|
| Global path (planner) | `/orchard/global_path` | 초록 굵은 선 — 남은 통로 직선 + U턴 + 다음 통로 |
| U-turn search grid (Hybrid A*) | `/orchard/plan/turn_grid` | 나무를 로버 반경만큼 부풀린 점유 격자 (반투명) |
| Orchard model | `/orchard/plan/markers` | 열 선분(갈색), 통로 중심선(하늘색)+`lane k`, 누적 줄기(초록 점), U턴 목표(주황 화살표), 상태 글자, odom 흐름 보정(빨간 화살표) |
| Local planner | `/orchard/local/markers` | DWA 후보 궤적(초록 = 통과, 진할수록 좋음 / 빨강 = 장애물로 탈락), 장애물 점(주황 상자), 차체 원 3개(파랑), 상태 글자 |
| Local path (DWA best) | `/orchard/local_path` | 고른 궤적 (노랑) |

## 파일 지도

| 파일 | 하는 일 | 고칠 때 |
|---|---|---|
| `orchard_model.py` | 관측 줄기 → 열(row frame v 봉우리) → 통로 중심·양쪽 열·행 끝 | 열 찾기, 통로 번호 매기기 |
| `global_planner.py` | 통로 순회 순서, 통로 직선 경로, 행 끝 확정, U턴 계획·진행, odom 흐름·요 오차 보정(곧게 달릴 때만), 열 방향(통로 0 중심점 직선), DONE 판정 | 순회 순서(건너뛰기 등), 흐름 보정 |
| `approach.py` | 과수원 밖에서 줄기 배치로 끝 통로 입구·정렬점 찾기 (plan.approach) | 입구 고르는 규칙 |
| `hybrid_astar.py` | Hybrid A* (회전반경을 지키는 격자 탐색 + Dubins 해석적 확장) | 후진 추가(Reeds-Shepp), 비용 |
| `grid.py` | 점유 격자 (부풀리기, 경로 검사, RViz 형식) | 해상도, 장애물 반영 |
| `obstacles.py` | LiDAR 한 스캔 → 지면 위 0.2~0.75 m 장애물 점 (RANSAC 지면 + 칸별 보정) | 장애물 높이 범위 |
| `dwa.py` | DWA: 두 호(S 자) 후보 → 충돌 검사(차체 원 3개) → 비용 최소 → (v, ω), 이미 너무 붙었으면 멀어지는 궤적으로 빠져나오기(escape) | 가중치, 후보 수, 속도 제한 |
| `global_planner_node.py` / `local_planner_node.py` | ROS 연결·표시 | 토픽 이름, 표시 모양 |

파라미터: `orchard_bringup/config/sim.yaml` · `robot.yaml` 의 `global_planner_node` (`plan.*`), `local_planner_node` (`obstacle.*`, `dwa.*`).

## 시험

```bash
cd ros2_ws/src/orchard_planning && python3 -m pytest -q test/          # 약 3분
```

- `test_planning.py`: 격자, Hybrid A*(벽 돌아가기, 헤드랜드 반원), 열·통로 모델, 장애물 점(잔디·수관 제외), DWA(정지·빠져나오기)
- `test_planner_closed_loop.py`: 가상 LiDAR + 차량 모델 폐루프 — 3통로 순회(횡오차 RMS < 8 cm), 통로 장애물 비켜 가기,
  U턴 중 odom 0.8 m 흐름 보정, 탐사(끝 열에서 멈춤), 출발 요 오차 17°, 요 오차 16° + 통로 사람(요 오차·열 방향이 안 틀어지는지), 과수원 밖 출발 → 입구 진입

## 참고 논문

Hybrid A* (Dolgov et al., IJRR 2010), DWA (Fox, Burgard, Thrun, IEEE RAM 1997), Dubins (1957), Shkel & Lumelsky (2001),
Boustrophedon 커버리지 (Choset, Autonomous Robots 2000) — [`docs/REFERENCES.md`](../../../docs/REFERENCES.md)
