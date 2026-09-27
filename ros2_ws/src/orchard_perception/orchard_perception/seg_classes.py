"""세그멘테이션 클래스 (Gazebo 라벨·학습·추론 공통). orchard_gazebo/world_gen.py 의 LABELS 와 같아야 한다.

사용처: seg_dataset_node(classes.json 저장), seg_path_node(DRIVABLE 픽셀 선택·색칠),
tools/training/train_segmentation.py(클래스 수·mIoU). 모두 이 파일을 import 하므로 클래스를 추가·변경할 때는
이 파일과 world_gen.py 의 LABELS 를 함께 고치고 데이터 재수집·모델 재학습 (라벨 영상의 픽셀값 = 이 리스트의 인덱스).
"""
# 인덱스 = 라벨 픽셀값 (0~6)
CLASSES = ['unlabeled', 'drivable', 'tree_strip', 'tree', 'structure', 'obstacle', 'background']
CLASS_KO = ['미분류/하늘', '주행가능(통로·농로·풀)', '나무 밑 띠', '나무', '구조물(지주·철선)', '장애물', '배경(숲)']
NUM_CLASSES = len(CLASSES)
DRIVABLE = 1                 # seg_path_node 가 통로 중심선 계산에 쓰는 클래스 번호
# 시각화 색 (BGR)
PALETTE_BGR = [(0, 0, 0), (80, 200, 80), (40, 90, 150), (20, 110, 20), (200, 200, 200), (40, 40, 230),
               (120, 80, 40)]
