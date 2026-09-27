# orchard_navigation: 과수원 행 순회 자율주행 (행 추종 → 행 끝 → U턴 → 다음 통로) 과 나무 지도.
#   ROS 없는 모듈: controller.py (Pure Pursuit 행 추종), headland.py (Dubins U턴), mission.py (상태기계),
#                  fusion.py (LiDAR·카메라 중심선 융합), treemap.py (나무 지도)
#   ROS 노드:     row_navigator_node.py (/cmd_vel 발행), orchard_mapper_node.py (나무 지도 저장)
