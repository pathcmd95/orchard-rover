"""LiDAR 중심선과 카메라(세그멘테이션) 중심선 융합 — ROS 의존성 없음.

파이프라인에서의 역할
  row_navigator_node 가 매 주기(tick) 미션 상태기계에 넘기기 전에 호출한다.
  행 추종 품질(offset/heading)만 카메라로 보강하고, 행 끝·U턴 판단은 LiDAR 값 그대로 둔다.

입력/출력 (mission.RowObs, base_link: x 전방 / y 왼쪽)
  - lidar: /orchard/row (LiDAR 행 인식) → RowObs
  - camera: /orchard/seg/row (카메라 세그멘테이션 중심선) → RowObs, 없거나 오래되면 None
  - 반환: (융합된 RowObs, 상태 문자열) — 상태는 /orchard/mission/state 의 'row=' 뒤에 찍힌다.

파라미터
  - camera_weight: ROS 파라미터 camera_row.weight (row_navigator_node, 기본 0.5).
    camera_row.enable / camera_row.timeout 도 같은 노드. sim.yaml / robot.yaml 에 넣어 바꾼다.
  - max_offset_diff [m], max_heading_diff [rad]: 불일치 판정 문턱 (ROS 파라미터 아님, 이 함수 기본값).

원칙 (LiDAR 주, 카메라 보조)
- 행 끝·U턴 판단(last_tree_ahead, 줄기 수)은 LiDAR 만 쓴다.
- 둘 다 유효하고 서로 크게 다르지 않을 때만 offset/heading 을 신뢰도 가중 평균한다.
- 크게 다르면 LiDAR 를 쓰고 신뢰도를 낮춘다 (잔디·그림자·처진 가지 등으로 한쪽이 틀린 상황).
- LiDAR 가 무효면 카메라만으로 '유효'를 만들지 않는다 (행 끝 판단이 망가짐).
"""
from __future__ import annotations

from dataclasses import replace

from .mission import RowObs


def fuse_rows(lidar: RowObs, camera: RowObs | None, camera_weight: float = 0.5,
              max_offset_diff: float = 0.4, max_heading_diff: float = 0.2) -> tuple[RowObs, str]:
    """(융합 결과, 상태 문자열) 반환. 상태: 'lidar' | 'fused' | 'disagree'.

    Args:
        lidar: LiDAR 행 인식 결과 (주 센서).
        camera: 카메라 중심선 (없으면 None).
        camera_weight: 카메라 가중치 배율 (LiDAR 가중치 = LiDAR 신뢰도, 카메라 = 이 값 × 카메라 신뢰도).
        max_offset_diff: offset 차이가 이보다 크면 불일치 [m].
        max_heading_diff: heading 차이가 이보다 크면 불일치 [rad] (0.2 rad ≈ 11°).
    Returns:
        'lidar': 카메라 없음/무효 또는 LiDAR 무효 → LiDAR 결과 그대로.
        'disagree': 둘이 크게 다름 → LiDAR 값, 신뢰도 × 0.7 (속도가 줄어든다).
        'fused': 가중 평균한 offset/heading, 신뢰도 상향. 나머지 필드(행 끝·줄기 수)는 LiDAR 값.
    """
    if camera is None or not camera.valid or not lidar.valid:
        return lidar, 'lidar'
    if abs(camera.offset - lidar.offset) > max_offset_diff or abs(camera.heading - lidar.heading) > max_heading_diff:
        return replace(lidar, confidence=lidar.confidence * 0.7), 'disagree'   # 0.7: 불일치 벌점 (경험값)
    wl = max(lidar.confidence, 1e-3)          # 0 으로 나누기 방지
    wc = camera_weight * max(camera.confidence, 0.0)
    s = wl + wc
    return replace(lidar,
                   offset=(wl * lidar.offset + wc * camera.offset) / s,
                   heading=(wl * lidar.heading + wc * camera.heading) / s,
                   # 두 센서가 맞으면 신뢰도를 올린다 (카메라 가중치의 절반만큼, 최대 1)
                   confidence=min(1.0, lidar.confidence + 0.5 * wc)), 'fused'
