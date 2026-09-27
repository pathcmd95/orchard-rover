"""orchard_planning: 과수원 전역 경로계획(통로 순회 + Hybrid A*) · 지역 경로계획(DWA).

모듈 (ROS 의존성 없는 것부터 읽을 것)
  grid.py            점유 격자 (나무·장애물 → 로봇 반경만큼 부풀린 격자)
  hybrid_astar.py    Hybrid A* (차의 최소 회전반경을 지키는 격자 탐색, Dubins 해석적 확장)
  orchard_model.py   관측한 줄기 → 열·통로 모델 (통로 중심선, 행 끝)
  global_planner.py  전역 계획: 통로 순회 순서 + 통로 직선 + 헤드랜드 U턴(Hybrid A*), 진행 관리, odom 흐름 보정
  obstacles.py       LiDAR 한 스캔 → 지면 위 장애물 점 (로버 높이 범위)
  dwa.py             지역 계획: Dynamic Window Approach (속도·곡률 후보를 굴려 보고 비용 최소 선택)
  global_planner_node.py / local_planner_node.py   ROS 연결
"""
