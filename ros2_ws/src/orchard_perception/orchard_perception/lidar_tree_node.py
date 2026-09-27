"""Livox Mid-360 3D 점군 → 과수 줄기 검출 + 행간중심선 추정 노드.

입력: sensor_msgs/PointCloud2 (/livox/lidar)
출력:
  /orchard/trees  (orchard_msgs/TreeArray)      base_link 기준 줄기 위치
  /orchard/row    (orchard_msgs/RowCenterline)  행간중심선, 행 끝, 통로 장애물 거리
  /orchard/markers (visualization_msgs/MarkerArray) RViz/Foxglove 시각화

파이프라인 내 역할 (LiDAR 인식의 중심 노드)
  점군 → TF 로 base_link 변환 → ROI·차체 점 제거 → ground.py(RANSAC 지면) → trunks.py(줄기 후보)
  → rows.py(좌/우 열 직선 + 중심선, RowTracker 평활) → rows.corridor_obstacle_distance(통로 장애물)
  /orchard/row 는 orchard_navigation 의 row_navigator_node 가 추종하고,
  /orchard/trees 는 row_navigator_node · orchard_mapper_node · tree_fusion_node 가 사용한다.
필요 TF: 점군 frame_id(예: livox_frame) → base_link (정적 변환, URDF 에서 발행)
주요 파라미터 (config/sim.yaml · robot.yaml 의 lidar_tree_node, 설명은 __init__ 의 선언부 참고)
  ground.nominal_z  ★ 실차 base_link 높이에 맞게 반드시 수정 (지면 기준 -높이 [m])
  self_filter.*     ★ 실차 차체 크기에 맞게 수정
  trunk.band_min / band_max  줄기 높이 구간, row.expected_width  행간거리, obstacle.*  통로 장애물 검사 범위
각 단계의 알고리즘·파라미터 의미는 ground.py, trunks.py, rows.py 의 docstring 참고.
"""
from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Point
from orchard_msgs.msg import RowCenterline, Tree, TreeArray
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .geometry import make_transform, transform_points
from .ground import fit_ground_plane
from .rows import RowTracker, corridor_obstacle_distance, fit_rows
from .trunks import detect_trunks


def cloud_to_xyz(msg: PointCloud2) -> np.ndarray:
    """PointCloud2 → Nx3 float64 배열 (센서 좌표계, m). NaN/inf 점은 제거.

    Livox 점군에는 intensity 등 다른 필드도 있지만 x, y, z 만 읽는다 (구조화 배열 → 일반 배열).
    """
    arr = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
    if arr.size == 0:
        return np.zeros((0, 3))
    pts = np.stack([arr['x'], arr['y'], arr['z']], axis=-1).astype(np.float64)
    return pts[np.all(np.isfinite(pts), axis=1)]


class LidarTreeNode(Node):
    """점군 한 프레임마다 지면 → 줄기 → 행 중심선 → 통로 장애물을 계산해 발행하는 노드."""

    def __init__(self):
        """파라미터 선언, TF 리스너·RowTracker·발행자·구독자 생성."""
        super().__init__('lidar_tree_node')
        p = self.declare_parameter
        p('input_topic', '/livox/lidar')
        p('target_frame', 'base_link')
        # 관심영역 (base_link 기준)
        p('roi.x_min', -4.0)            # [m] 뒤쪽 줄기도 행 적합에 씀 (row.fit_x_min -3 m 참고)
        p('roi.x_max', 15.0)            # [m] 멀수록 점이 성겨 줄기 검출이 어렵고 계산량만 늘어남
        p('roi.y_abs_max', 6.0)         # [m] 좌우 6 m ≈ 행간 3.8 m 의 1.5 배 (바로 옆 열 + 여유)
        p('roi.z_min', -1.5)            # [m] base_link 기준 높이 범위 (경사지 지면 포함)
        p('roi.z_max', 2.5)
        # 차체 자기 점 제거 박스
        p('self_filter.x_min', -0.7)    # [m] 차체 + 여유. 이 박스 안 점(차체·센서 거치대)은 버림
        p('self_filter.x_max', 0.7)
        p('self_filter.y_abs_max', 0.45)
        # 지면 추정
        p('ground.nominal_z', -0.1)     # 평지에서 base_link 기준 지면 높이
        p('ground.search_band', 0.4)    # [m] nominal_z ± 이 범위의 점만 지면 후보
        p('ground.distance_threshold', 0.08)  # [m] RANSAC 인라이어 거리 (잔디 높이 + 잡음 정도)
        p('ground.max_tilt_deg', 20.0)  # [deg] 이보다 기운 평면은 지면으로 인정 안 함
        p('ground.iterations', 60)      # RANSAC 반복 횟수 (늘리면 정확↑, 계산시간↑)
        p('ground.min_inliers', 150)    # 이보다 인라이어가 적으면 nominal 평면 사용
        # 줄기 검출
        p('trunk.band_min', 0.25)       # 잔디(5~15cm)보다 높게
        p('trunk.band_max', 0.7)        # 수관 시작 높이보다 낮게
        p('trunk.cell_size', 0.08)      # [m] XY 군집화 격자 크기
        p('trunk.min_points', 6)        # 줄기 군집 최소 점 수
        p('trunk.max_diameter', 0.45)   # [m] 이보다 넓은 군집은 줄기 아님 (사람·울타리 등)
        p('trunk.min_height_span', 0.12)  # [m] 군집 높이 범위 최소값 (납작한 잔디 뭉치 제거)
        # 행 추정
        p('row.expected_width', 3.8)    # [m] 행간거리 (과수원마다 수정)
        p('row.fit_x_min', -3.0)        # [m] 행 적합에 쓰는 줄기 x 범위
        p('row.fit_x_max', 12.0)
        p('row.max_residual', 0.35)     # [m] 열 직선에서 이보다 먼 줄기는 이상점
        p('row.smoothing_alpha', 0.4)   # 프레임간 EMA 계수 (0~1, 클수록 빠르고 흔들림)
        p('row.hold_time', 0.6)         # [s] 검출 실패 시 이전 추정 유지 시간
        # 통로 장애물
        p('obstacle.half_width', 0.5)   # [m] 중심선 ± 이 폭 안을 주행 통로로 봄 (차체 반폭 + 여유)
        p('obstacle.x_min', 0.3)        # [m] 전방 검사 거리 범위
        p('obstacle.x_max', 6.0)
        p('obstacle.h_min', 0.2)        # [m] 지면 기준 장애물 높이 범위 (h_min 아래는 잔디)
        p('obstacle.h_max', 1.6)        # [m] 이보다 높은 점(처진 가지·수관)은 무시. sim.yaml 은 차체 높이 기준 0.75
        p('obstacle.min_points', 5)     # 장애물로 판정할 최소 점 수
        p('publish_markers', True)      # RViz 마커 발행 (실차에서 CPU 아끼려면 false)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tracker = RowTracker(self.g('row.smoothing_alpha'), self.g('row.hold_time'))
        self.rng = np.random.default_rng(0)     # RANSAC 난수 (seed 고정 → 같은 입력이면 같은 결과)

        self.pub_trees = self.create_publisher(TreeArray, '/orchard/trees', 10)
        self.pub_row = self.create_publisher(RowCenterline, '/orchard/row', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/orchard/markers', 10)
        self.sub = self.create_subscription(PointCloud2, self.g('input_topic'), self.on_cloud,
                                            qos_profile_sensor_data)
        self._tf_cache: dict[str, np.ndarray] = {}     # frame_id → 4x4 (센서는 차체에 고정이므로 한 번만 조회)
        self._last_log = 0.0                           # 상태 로그 주기 제한용 (벽시계)
        self.get_logger().info(f"subscribing {self.g('input_topic')} → /orchard/trees, /orchard/row")

    def g(self, name):
        """파라미터 값 읽기 단축 함수 (실행 중 ros2 param set 으로 바꾼 값도 바로 반영)."""
        return self.get_parameter(name).value

    def lookup(self, source_frame: str) -> np.ndarray | None:
        """센서→base_link 정적 변환 (한 번 찾으면 캐시).

        source_frame: 점군 header.frame_id. 반환: 4x4 (센서 → target_frame) 또는 TF 가 아직 없으면 None.
        Time() (= 0) 으로 조회하면 '가장 최근 변환' 을 뜻한다 (정적 TF 라 시각 무관).
        """
        target = self.g('target_frame')
        if source_frame == target:
            return np.eye(4)
        if source_frame in self._tf_cache:
            return self._tf_cache[source_frame]
        try:
            tf = self.tf_buffer.lookup_transform(target, source_frame, Time(), Duration(seconds=0.2))
        except TransformException as exc:
            self.get_logger().warn(f'TF {source_frame}->{target} 없음: {exc}', throttle_duration_sec=2.0)
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        mat = make_transform((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        self._tf_cache[source_frame] = mat
        return mat

    def on_cloud(self, msg: PointCloud2):
        """점군 콜백: 인식 파이프라인 전체를 한 번 실행하고 결과를 발행한다."""
        t0 = time.perf_counter()
        tf = self.lookup(msg.header.frame_id)
        if tf is None:
            return
        pts = transform_points(cloud_to_xyz(msg), tf)    # 센서 좌표 → base_link

        # ROI + 차체 제거
        # roi: 관심영역 안 점, body: 차체 박스 안 점 (높이 무관) → roi 이면서 body 가 아닌 점만 남김
        roi = ((pts[:, 0] > self.g('roi.x_min')) & (pts[:, 0] < self.g('roi.x_max'))
               & (np.abs(pts[:, 1]) < self.g('roi.y_abs_max'))
               & (pts[:, 2] > self.g('roi.z_min')) & (pts[:, 2] < self.g('roi.z_max')))
        body = ((pts[:, 0] > self.g('self_filter.x_min')) & (pts[:, 0] < self.g('self_filter.x_max'))
                & (np.abs(pts[:, 1]) < self.g('self_filter.y_abs_max')))
        pts = pts[roi & ~body]

        ground = fit_ground_plane(
            pts, self.g('ground.nominal_z'), self.g('ground.search_band'),
            self.g('ground.distance_threshold'), self.g('ground.max_tilt_deg'),
            self.g('ground.iterations'), self.g('ground.min_inliers'), self.rng)
        heights = ground.height(pts)                      # 각 점의 지면 기준 높이 [m]

        trunks = detect_trunks(
            pts, heights, self.g('trunk.band_min'), self.g('trunk.band_max'),
            self.g('trunk.cell_size'), self.g('trunk.min_points'),
            self.g('trunk.max_diameter'), self.g('trunk.min_height_span'))

        xy = np.array([[t.x, t.y] for t in trunks]) if trunks else np.zeros((0, 2))
        conf = np.array([t.confidence for t in trunks]) if trunks else np.zeros(0)
        prior_off, prior_head = self.tracker.prior()     # 이전 프레임 추정 (heading 만 실제로 사용됨)
        raw = fit_rows(xy, conf, self.g('row.expected_width'), prior_off, prior_head,
                       self.g('row.fit_x_min'), self.g('row.fit_x_max'), self.g('row.max_residual'))
        # 평활·유지 시간은 메시지 시각 [s] 기준 (시뮬 시간·rosbag 재생에서도 일관)
        now = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        est = self.tracker.update(raw, now)

        # 통로 장애물은 중심선을 따라 검사. 행 추정이 없으면 로봇 정면 (offset 0, heading 0) 을 검사
        obs_off, obs_head = (est.offset, est.heading) if est.valid else (0.0, 0.0)
        obstacle = corridor_obstacle_distance(
            pts, heights, obs_off, obs_head, self.g('obstacle.half_width'),
            self.g('obstacle.x_min'), self.g('obstacle.x_max'), self.g('obstacle.h_min'),
            self.g('obstacle.h_max'), self.g('obstacle.min_points'))

        # 결과는 입력 점군과 같은 시각, 좌표계만 base_link 로
        header = msg.header
        header.frame_id = self.g('target_frame')
        # 줄기의 row_side 는 평활 전 raw 추정의 분류를 그대로 사용 (평활 결과에는 이번 프레임 분류가 없을 수 있음)
        self.publish_trees(header, trunks, raw.sides, ground)
        self.publish_row(header, est, raw, obstacle)
        if self.g('publish_markers'):
            self.publish_markers(header, trunks, est, obstacle, ground)

        dt = (time.perf_counter() - t0) * 1000.0          # 처리 시간 [ms]
        # 5 초마다 한 번 상태 요약 로그 (매 프레임 출력하면 터미널이 넘침)
        if time.monotonic() - self._last_log > 5.0:
            self._last_log = time.monotonic()
            self.get_logger().info(
                f'pts={pts.shape[0]} ground={"fit" if ground.fitted else "nominal"}'
                f'(tilt {ground.tilt_deg():.1f}°) trunks={len(trunks)} '
                f'row={"OK" if est.valid else "--"} off={est.offset:+.2f} '
                f'head={math.degrees(est.heading):+.1f}° obs={obstacle:.1f} {dt:.0f}ms')

    @staticmethod
    def ground_z_at(ground, x, y):
        """지면 평면에서 (x, y) [m] 위치의 z [m] (base_link 기준).

        평면식 nx·x + ny·y + nz·z + d = 0 을 z 에 대해 푼 것. 법선이 거의 수직(nz ≈ 1)이라 나누기 안전.
        """
        n = ground.normal
        return float(-(n[0] * x + n[1] * y + ground.d) / n[2])

    def publish_trees(self, header, trunks, sides, ground):
        """줄기 후보 → orchard_msgs/TreeArray 발행 (/orchard/trees).

        sides: fit_rows 의 줄기별 열 분류 (+1 왼쪽 / -1 오른쪽 / 0 미사용). position.z 는 그 위치의 지면 높이.
        """
        out = TreeArray()
        out.header = header
        for i, t in enumerate(trunks):
            tree = Tree()
            tree.position = Point(x=t.x, y=t.y, z=self.ground_z_at(ground, t.x, t.y))
            tree.radius = float(t.radius)
            tree.height = float(t.top_height)
            tree.num_points = int(t.num_points)
            tree.confidence = float(t.confidence)
            tree.camera_confirmed = False
            tree.row_side = int(sides[i]) if i < len(sides) else 0
            out.trees.append(tree)
        self.pub_trees.publish(out)

    def publish_row(self, header, est, raw, obstacle):
        """행 추정 → orchard_msgs/RowCenterline 발행 (/orchard/row).

        est: 평활된 추정, raw: 이번 프레임 원추정 (last_tree_ahead 는 평활하지 않은 현재 값을 써야 행 끝을 빨리 감지),
        obstacle: 통로 장애물 x 거리 [m] (없으면 inf).
        """
        m = RowCenterline()
        m.header = header
        m.valid = bool(est.valid)
        m.lateral_offset = float(est.offset)
        m.heading_error = float(est.heading)
        m.row_width = float(est.width)
        m.confidence = float(est.confidence)
        m.left_count = int(est.left_count)
        m.right_count = int(est.right_count)
        m.left_offset = float(est.left_offset)
        m.right_offset = float(est.right_offset)
        m.last_tree_ahead = float(raw.last_tree_ahead)
        m.obstacle_distance = float(obstacle)
        self.pub_row.publish(m)

    def publish_markers(self, header, trunks, est, obstacle, ground):
        """RViz/Foxglove 시각화 마커 발행: 줄기(갈색 원기둥), 좌/우 열(초록 선), 중심선(노란 선), 장애물(빨간 판)."""
        arr = MarkerArray()
        clear = Marker(header=header, action=Marker.DELETEALL)   # 이전 프레임 마커를 먼저 지움 (줄기 수가 줄어도 잔상 없게)
        arr.markers.append(clear)
        for i, t in enumerate(trunks):
            mk = Marker(header=header, ns='trunks', id=i, type=Marker.CYLINDER, action=Marker.ADD)
            gz = self.ground_z_at(ground, t.x, t.y)
            # 높이 1 m 원기둥의 중심을 지면 + 0.5 m 에 두면 지면~1 m 를 덮는다
            mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = t.x, t.y, gz + 0.5
            mk.pose.orientation.w = 1.0
            mk.scale.x = mk.scale.y = max(0.08, 2 * t.radius)    # 지름 [m], 너무 가늘면 안 보여 최소 8 cm
            mk.scale.z = 1.0
            mk.color = ColorRGBA(r=0.55, g=0.35, b=0.15, a=0.9)
            arr.markers.append(mk)
        if est.valid:
            b = math.tan(est.heading)
            for ns, off, color in (('left', est.left_offset, (0.1, 0.8, 0.1)),
                                   ('right', est.right_offset, (0.1, 0.8, 0.1)),
                                   ('center', est.offset, (1.0, 0.85, 0.0))):
                if not math.isfinite(off):
                    continue
                mk = Marker(header=header, ns=ns, id=0, type=Marker.LINE_STRIP, action=Marker.ADD)
                mk.pose.orientation.w = 1.0
                mk.scale.x = 0.06 if ns == 'center' else 0.03
                mk.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0)
                for x in (-2.0, 10.0):          # 뒤 2 m ~ 앞 10 m 구간을 직선으로 그림
                    mk.points.append(Point(x=x, y=off + b * x, z=0.0))
                arr.markers.append(mk)
        if math.isfinite(obstacle):
            mk = Marker(header=header, ns='obstacle', id=0, type=Marker.CUBE, action=Marker.ADD)
            mk.pose.position.x = obstacle
            mk.pose.position.y = est.offset + math.tan(est.heading) * obstacle if est.valid else 0.0
            mk.pose.position.z = 0.5
            mk.pose.orientation.w = 1.0
            mk.scale.x, mk.scale.y, mk.scale.z = 0.1, 1.0, 1.0
            mk.color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.6)
            arr.markers.append(mk)
        self.pub_markers.publish(arr)


def main(args=None):
    """노드 실행 진입점 (ros2 run orchard_perception lidar_tree_node)."""
    rclpy.init(args=args)
    node = LidarTreeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
