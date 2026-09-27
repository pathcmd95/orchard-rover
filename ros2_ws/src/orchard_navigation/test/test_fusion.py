"""fusion.fuse_rows (LiDAR 주·카메라 보조 중심선 융합) 규칙 시험."""
from orchard_navigation.fusion import fuse_rows
from orchard_navigation.mission import RowObs


def test_fusion_rules():
    """일치 → 'fused'(가중 평균, 행 끝·줄기 수는 LiDAR 값) / 불일치 → 'disagree'(LiDAR 값, 신뢰도 하락) /
    LiDAR 무효 → 카메라가 유효해도 무효 유지 / 카메라 없음 → LiDAR 그대로."""
    lidar = RowObs(True, 0.10, 0.02, 0.8, 5.0, float('inf'), 6, 6)
    cam = RowObs(True, 0.00, 0.00, 0.8)
    f, st = fuse_rows(lidar, cam, 1.0)
    assert st == 'fused' and 0.0 < f.offset < 0.10 and f.last_tree_ahead == 5.0 and f.left_count == 6
    f, st = fuse_rows(lidar, RowObs(True, 1.0, 0.0, 0.9))
    assert st == 'disagree' and f.offset == 0.10 and f.confidence < 0.8
    f, st = fuse_rows(RowObs(False), cam)
    assert st == 'lidar' and not f.valid                # 카메라만으로는 유효하게 만들지 않는다
    f, st = fuse_rows(lidar, None)
    assert st == 'lidar' and f is lidar
