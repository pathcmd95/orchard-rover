# 행간중심선 추정 결과 (header.frame_id = base_link, x 전방 / y 왼쪽)
# 중심선 모델: y = lateral_offset + tan(heading_error) * x
std_msgs/Header header
bool valid
float32 lateral_offset      # [m] x=0 에서 중심선의 y (+: 중심선이 로봇 왼쪽)
float32 heading_error       # [rad] 중심선 방향 - 로봇 진행방향 (+: 왼쪽으로 틀어짐)
float32 row_width           # [m] 좌우 과수열 사이 수직 거리
float32 confidence          # 0~1
uint32 left_count           # 왼쪽 열 줄기 개수
uint32 right_count          # 오른쪽 열 줄기 개수
float32 left_offset         # [m] 왼쪽 열 직선의 x=0 에서 y
float32 right_offset        # [m] 오른쪽 열 직선의 x=0 에서 y
float32 last_tree_ahead     # [m] 행 추정에 쓰인 줄기 중 가장 앞쪽 x (행 끝 판단), 없으면 -inf
float32 obstacle_distance   # [m] 주행 통로 안 최근접 장애물 x 거리, 없으면 +inf
