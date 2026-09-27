# 과수 1그루(줄기) 검출 결과. 좌표는 TreeArray.header.frame_id (보통 base_link) 기준.
geometry_msgs/Point position   # 줄기 중심 (z = 그 위치의 지면 높이)
float32 radius                 # [m] 추정 줄기 반경
float32 height                 # [m] 검출에 쓰인 점의 최대 높이 (지면 기준)
uint32 num_points              # 줄기 군집 점 개수
float32 confidence             # 0~1
bool camera_confirmed          # 카메라 YOLO 검출과 매칭되면 true
int8 row_side                  # +1 왼쪽 열, -1 오른쪽 열, 0 행 추정에 미사용
