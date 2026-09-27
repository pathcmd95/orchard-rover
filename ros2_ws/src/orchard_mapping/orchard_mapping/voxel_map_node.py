"""복셀 맵 + 과수 줄기 매핑 노드 (출발점 기준 상대좌표).

처리 순서 (사용자 요청 흐름)
  ① YOLO 줄기 인식  : camera_tree_node 의 /orchard/detections (영상 박스)
  ② 복셀 맵 위치 확인: 박스 방향(bearing) 쐐기 안에서 복셀 맵의 '땅에서 올라온 가느다란 기둥' 을 찾음 (trunk_locator)
  ③ 매칭            : 이미 등록된 줄기와 0.4 m 안이면 같은 줄기, 아니면 새 번호 (trunk_registry)
  ④ 상대좌표 기록    : 출발점 (0,0,0), 처음 달린 방향 +x 인 'start' 좌표로 저장 (start_frame)
  YOLO 모델이 아직 없으면(detection_source=auto 에서 검출이 안 들어오면) LiDAR 줄기(/orchard/trees)로 ②~④ 를 대신한다.

입력 (ROS topic)
  /livox/lidar              sensor_msgs/PointCloud2   복셀 맵에 누적할 점군 (Mid-360)
  /odom                     nav_msgs/Odometry         로버 자세 (PX4 EKF → orchard_px4_bridge)
  /orchard/detections       vision_msgs/Detection2DArray   YOLO 줄기 박스 (camera_tree_node)
  /camera/color/camera_info sensor_msgs/CameraInfo    카메라 내부행렬 K, 광학 frame_id
  /orchard/trees            orchard_msgs/TreeArray    LiDAR 줄기 (YOLO 가 없을 때 대신 사용)
  /orchard/mission/state    std_msgs/String           미션 시작(FOLLOW_ROW) 에 원점 설정, DONE 에 자동 저장
  /orchard/odom_drift       geometry_msgs/PointStamped  (선택) 전역 계획이 추정한 odom 흐름 → 빼고 쌓음 (use_odom_drift)
  /orchard/row_frame        geometry_msgs/PoseStamped   (선택) 전역 계획이 추정한 첫 통로 방향 → start 좌표 +x 축 (axis_source)
출력
  /orchard/voxel_map        visualization_msgs/MarkerArray  높이별 색 복셀 (초록 지면 → 파랑 수관), frame 'start'
  /orchard/trunk_map        visualization_msgs/MarkerArray  확정 줄기 (주황 = YOLO 확인, 회색 = LiDAR 만) + 번호
  /orchard/start_path       nav_msgs/Path                   출발점 기준 주행 궤적
  TF odom → start (정적)     원점·방향이 정해지면 한 번 발행 (RViz 에서 start 프레임으로 보기)
  서비스 /orchard/voxel_map/save (std_srvs/Trigger)
  저장 <output_dir>/voxel_map_<날짜시각>/
    voxels.ply (높이 색 점군, CloudCompare·MeshLab), voxel_map.npz (전체 맵), trunks.csv (id,x,y,z,hits,...),
    trunks_by_row.csv / trunks_table.html / trunks_table.md (열별 줄기 표: 열·순번·좌표·간격·결주 의심, trunk_table.py),
    trajectory.csv (t,x,y,z,yaw), meta.json (원점 odom 좌표, 방향, 복셀 크기, 좌표계 설명, 열 요약)

주요 파라미터 (config/sim.yaml · robot.yaml 의 voxel_map_node)
  voxel_size [m], integrate_period [s], min_range/max_range [m], origin_trigger (mission_start|first_odom),
  detection_source (auto|yolo|lidar), min_conf, trunk.band_min/band_max [m], trunk.max_range [m],
  assoc_radius [m], min_hits, display_max_voxels, output_dir, autosave_period [s]

한계 (학생 개선 과제)
  - 차체 롤·피치를 무시하고 요(yaw)만으로 점을 돌린다 → 경사가 심하면 복셀이 기울어 쌓임
  - 복셀은 '점이 들어온 칸' 만 센다 (빈 공간 지우기 없음). 움직이는 사람은 흔적이 남음 → OctoMap 방식 광선 투사로 개선 가능
  - 위치는 odom(GPS+IMU) 을 그대로 믿는다 → U턴 뒤 0.5 m 정도 어긋날 수 있음 (FAST-LIO 같은 LiDAR 오도메트리로 개선)
"""
from __future__ import annotations

import csv
import json
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from orchard_msgs.msg import TreeArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray

from .start_frame import StartFrame
from .trunk_locator import TrunkLocator, bbox_bearing
from .trunk_registry import TrunkRegistry
from .trunk_table import write_tables
from .voxel_map import VoxelMap, height_colors


def _yaw(q) -> float:
    """쿼터니언 → 요 [rad]."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _quat_to_matrix(q) -> np.ndarray:
    """geometry_msgs 쿼터니언 → 3×3 회전행렬."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _rot_z(a: float) -> np.ndarray:
    """z 축 회전 3×3."""
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class PoseTrack:
    """odom 자세 기록 (t, x, y, z, yaw) 과 시각 보간. 검출·스캔 시각의 자세를 쓰기 위함."""

    def __init__(self, keep: float = 30.0):
        """keep: 보관 시간 [s]."""
        self.keep = keep
        self.t: list[float] = []
        self.p: list[tuple[float, float, float, float]] = []

    def add(self, t, x, y, z, yaw):
        """자세 하나 추가 (시간 순)."""
        if self.t and t <= self.t[-1]:
            return
        self.t.append(t)
        self.p.append((x, y, z, yaw))
        while self.t and self.t[0] < t - self.keep:
            self.t.pop(0)
            self.p.pop(0)

    def at(self, t: float, tol: float = 0.3):
        """시각 t 의 (x, y, z, yaw). 기록 범위 밖(tol 초 넘게)이면 None."""
        if not self.t or t < self.t[0] - tol or t > self.t[-1] + tol:
            return None
        i = int(np.searchsorted(self.t, t))
        if i <= 0:
            return self.p[0]
        if i >= len(self.t):
            return self.p[-1]
        (x0, y0, z0, w0), (x1, y1, z1, w1) = self.p[i - 1], self.p[i]
        f = (t - self.t[i - 1]) / max(self.t[i] - self.t[i - 1], 1e-9)
        dw = math.atan2(math.sin(w1 - w0), math.cos(w1 - w0))
        return (x0 + f * (x1 - x0), y0 + f * (y1 - y0), z0 + f * (z1 - z0), w0 + f * dw)


class VoxelMapNode(Node):
    """LiDAR 복셀 맵을 쌓고, YOLO(또는 LiDAR) 줄기 검출을 복셀 맵에서 확인·매칭해 상대좌표로 기록한다."""

    def __init__(self):
        """파라미터 선언, 맵·등록부 생성, 구독·발행·서비스·타이머 설정."""
        super().__init__('voxel_map_node')
        d = self.declare_parameter
        d('lidar_topic', '/livox/lidar')
        d('detections_topic', '/orchard/detections')
        d('camera_info_topic', '/camera/color/camera_info')
        d('trees_topic', '/orchard/trees')
        d('base_frame', 'base_link')
        d('odom_frame', 'odom')
        d('map_frame', 'start')              # 출발점 기준 상대좌표 프레임 이름
        d('voxel_size', 0.1)                 # [m]
        d('ground_cell', 0.5)                # 지면 높이 추정 xy 격자 [m]
        d('integrate_period', 0.5)           # 스캔 누적 간격 [s, 시뮬 시간] (매 스캔 누적은 느린 PC 에서 무거움)
        d('min_range', 0.6)                  # 차체에 맞은 점 제외 [m]
        d('max_range', 15.0)                 # 먼 점은 부정확해 제외 [m]
        d('origin_trigger', 'mission_start')  # mission_start: 미션 시작 순간 / first_odom: 노드가 처음 받은 odom
        d('course_distance', 1.5)            # +x 방향을 정할 직진 거리 [m] (axis_source course 또는 대체용)
        d('axis_source', 'row_frame')        # row_frame: 전역 계획의 통로 방향(/orchard/row_frame) / course: 처음 달린 방향
        d('axis_wait', 20.0)                 # row_frame 이 이 시간 [s] 안에 안 오면 처음 달린 방향으로 대체 (nav_mode row 등)
        d('detection_source', 'auto')        # auto: YOLO 검출이 들어오면 YOLO, 없으면 LiDAR / yolo / lidar
        d('min_conf', 0.35)                  # YOLO 점수 하한
        d('trunk.band_min', 0.15)            # 줄기 높이 띠 [m, 지면 위]
        d('trunk.band_max', 0.7)
        d('trunk.max_range', 10.0)           # YOLO 방향으로 줄기를 찾을 최대 거리 [m]
        d('assoc_radius', 0.4)               # 같은 줄기로 볼 거리 [m]
        d('min_hits', 3)                     # 확정 관측 수
        d('row_spacing', 3.8)                # 열 간격 [m] — 열에서 벗어난 오검출 줄기 거르기
        d('row_filter_tol', 0.4)             # 열 중심선에서 이보다 먼 줄기는 뺌 [m] (0 이면 끔)
        d('display_min_hits', 2)             # RViz 에 보일 복셀의 최소 관측 스캔 수
        d('display_max_voxels', 60000)       # RViz 표시 최대 복셀 수 (많으면 RViz 가 느려짐)
        d('publish_period', 3.0)             # 표시 갱신 [s]
        d('display_alpha', 0.4)              # 복셀 투명도 (0~1). 낮으면 아래의 경로·줄기 표시가 비쳐 보임
        d('output_dir', os.path.expanduser('~/orchard_voxel_maps'))
        d('autosave_period', 60.0)           # [s], 0 이면 끔
        d('use_odom_drift', True)            # 전역 계획이 추정한 odom 흐름(/orchard/odom_drift)을 빼고 쌓기 (U턴 뒤 어긋남 줄임)

        g = self.g
        self.frame = StartFrame(g('course_distance'))
        self.vm = VoxelMap(g('voxel_size'), g('ground_cell'))
        self.loc = TrunkLocator(band=(g('trunk.band_min'), g('trunk.band_max')), r_max=g('trunk.max_range'))
        self.reg = TrunkRegistry(g('assoc_radius'), int(g('min_hits')))
        self.track = PoseTrack()
        self.path: list[tuple[float, float, float, float, float]] = []   # (t, x, y, z, yaw) start 좌표
        self.k = None
        self.cam_frame = None
        self.static_cache: dict[str, tuple[np.ndarray, np.ndarray] | None] = {}
        self.last_integrate = -1e9
        self.last_yolo = -1e9
        self.dirty = False
        self.origin_sent = False
        self.saved_done = False
        self.out_dir = os.path.join(g('output_dir'), time.strftime('voxel_map_%Y%m%d_%H%M%S'))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_br = StaticTransformBroadcaster(self)
        self.pub_vox = self.create_publisher(MarkerArray, '/orchard/voxel_map', 1)
        self.pub_trunks = self.create_publisher(MarkerArray, '/orchard/trunk_map', 1)
        self.pub_path = self.create_publisher(Path, '/orchard/start_path', 1)
        self.create_subscription(PointCloud2, g('lidar_topic'), self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(Detection2DArray, g('detections_topic'), self.on_detections, 10)
        self.create_subscription(CameraInfo, g('camera_info_topic'), self.on_info, qos_profile_sensor_data)
        self.create_subscription(TreeArray, g('trees_topic'), self.on_trees, 10)
        self.create_subscription(String, '/orchard/mission/state', self.on_state, 10)
        self.row_yaw = None                  # 전역 계획이 추정한 첫 통로 방향 [rad, odom]
        self.axis_wait_since = None
        self.create_subscription(PoseStamped, '/orchard/row_frame', self.on_row_frame, 10)
        self.drift = (0.0, 0.0)
        self.yaw_bias = 0.0
        if g('use_odom_drift'):
            self.create_subscription(PointStamped, '/orchard/odom_drift', self.on_drift, 10)
        self.create_service(Trigger, '/orchard/voxel_map/save', self.srv_save)
        self.create_timer(1.0, self.refresh_locator)
        self.create_timer(g('publish_period'), self.publish)
        if g('autosave_period') > 0:
            self.create_timer(g('autosave_period'), self.autosave)
        self.get_logger().info(f"복셀 맵 시작: 복셀 {g('voxel_size')} m, 원점 = {g('origin_trigger')}, "
                               f"줄기 검출 = {g('detection_source')}, 저장 {self.out_dir}")

    def g(self, name):
        """파라미터 값."""
        return self.get_parameter(name).value

    @staticmethod
    def stamp(msg) -> float:
        """메시지 header.stamp → 초."""
        return Time.from_msg(msg.header.stamp).nanoseconds * 1e-9

    # ------------------------------------------------------------ 자세·원점
    def on_drift(self, msg: PointStamped):
        """전역 계획(global_planner_node)이 추정한 odom 흐름 [m] (odom 좌표). 참 위치 ≈ odom − 흐름."""
        self.drift = (msg.point.x, msg.point.y)
        self.yaw_bias = float(msg.point.z)             # odom 요 오차 [rad]

    def on_odom(self, msg: Odometry):
        """자세 기록 (odom 흐름 보정 후), 원점·방향 결정, 궤적 기록."""
        q = msg.pose.pose.position
        p = Point(x=q.x - self.drift[0], y=q.y - self.drift[1], z=q.z)      # 흐름을 뺀 위치
        yaw = _yaw(msg.pose.pose.orientation) + self.yaw_bias
        t = self.stamp(msg)
        self.track.add(t, p.x, p.y, p.z, yaw)
        if self.frame.origin is None and self.g('origin_trigger') == 'first_odom':
            self.frame.begin(p.x, p.y, p.z)
        if self.frame.origin is not None and not self.frame.ready:
            if self.g('axis_source') == 'row_frame' and self.row_yaw is not None:
                self.frame.set_axis(self.row_yaw)          # 통로 방향 (비스듬히 출발·진입해도 +x = 열 방향)
                self.send_origin_tf()
            elif self.g('axis_source') != 'row_frame':
                if self.frame.observe(p.x, p.y):
                    self.send_origin_tf()
            else:                                          # 통로 방향을 기다리다 axis_wait 넘으면 처음 달린 방향으로
                far = math.hypot(p.x - self.frame.origin[0], p.y - self.frame.origin[1]) >= self.g('course_distance')
                if far and self.axis_wait_since is None:
                    self.axis_wait_since = t
                if self.axis_wait_since is not None and t - self.axis_wait_since > self.g('axis_wait') \
                        and self.frame.observe(p.x, p.y):
                    self.get_logger().warn('통로 방향(/orchard/row_frame) 이 안 와서 처음 달린 방향으로 +x 결정')
                    self.send_origin_tf()
        if self.frame.ready and (not self.path or t - self.path[-1][0] > 0.5):
            q = self.frame.to_start(np.array([[p.x, p.y, p.z]]))[0]
            self.path.append((t, q[0], q[1], q[2], self.frame.yaw_to_start(yaw)))

    def on_row_frame(self, msg: PoseStamped):
        """전역 계획의 통로 모델 (odom 위치·방향). 방향만 start +x 축으로 쓴다."""
        self.row_yaw = _yaw(msg.pose.orientation)

    def on_state(self, msg: String):
        """미션 시작에 원점 설정, DONE 에 자동 저장."""
        if msg.data.startswith('FOLLOW_ROW') and self.frame.origin is None \
                and self.g('origin_trigger') == 'mission_start' and self.track.p:
            x, y, z, _ = self.track.p[-1]
            self.frame.begin(x, y, z)
            self.get_logger().info(f'원점 설정 (odom {x:.2f}, {y:.2f}, {z:.2f}) — '
                                   '+x 방향은 통로 방향(전역 계획) 또는 처음 달린 방향으로 결정')
        if msg.data.startswith('DONE') and not self.saved_done:
            self.saved_done = True
            self.save()

    def send_origin_tf(self):
        """TF odom → start 를 정적 변환으로 한 번 발행."""
        T = self.frame.matrix_odom_from_start()
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.g('odom_frame')
        tf.child_frame_id = self.g('map_frame')
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, T[:3, 3])
        tf.transform.rotation.z = math.sin(self.frame.yaw0 / 2)
        tf.transform.rotation.w = math.cos(self.frame.yaw0 / 2)
        self.static_br.sendTransform(tf)
        self.origin_sent = True
        self.get_logger().info(f'start 좌표 확정: +x = odom 방위 {math.degrees(self.frame.yaw0):.1f}°')

    def static_tf(self, frame: str):
        """base_link ← frame 고정 변환 (R, t). 센서 장착은 고정이라 한 번만 조회해 캐시."""
        if frame not in self.static_cache or self.static_cache[frame] is None:
            try:
                tf = self.tf_buffer.lookup_transform(self.g('base_frame'), frame, Time())
                tr = tf.transform.translation
                self.static_cache[frame] = (_quat_to_matrix(tf.transform.rotation), np.array([tr.x, tr.y, tr.z]))
            except TransformException:
                self.static_cache[frame] = None
        return self.static_cache[frame]

    def pose_start(self, t: float):
        """시각 t 의 base_link 자세를 start 좌표 (R 3×3, p 3) 로. 없으면 None."""
        pose = self.track.at(t)
        if pose is None or not self.frame.ready:
            return None
        x, y, z, yaw = pose
        p = self.frame.to_start(np.array([[x, y, z]]))[0]
        return _rot_z(self.frame.yaw_to_start(yaw)), p

    # ------------------------------------------------------------ 복셀 맵
    def on_cloud(self, msg: PointCloud2):
        """스캔을 start 좌표로 옮겨 복셀 맵에 누적 (integrate_period 마다)."""
        t = self.stamp(msg)
        if t - self.last_integrate < self.g('integrate_period'):
            return
        pose = self.pose_start(t)
        st = self.static_tf(msg.header.frame_id)
        if pose is None or st is None:
            return
        arr = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if arr.size == 0:
            return
        pts = np.stack([arr['x'], arr['y'], arr['z']], axis=-1).astype(np.float64)
        r = np.linalg.norm(pts, axis=1)
        pts = pts[np.isfinite(r) & (r > self.g('min_range')) & (r < self.g('max_range'))]
        R_bl, t_bl = st                                   # 센서 → base_link
        R_sb, p_sb = pose                                 # base_link → start
        pts_start = (pts @ R_bl.T + t_bl) @ R_sb.T + p_sb
        self.vm.integrate(pts_start)
        self.last_integrate = t
        self.dirty = True

    def refresh_locator(self):
        """복셀 맵이 바뀌었으면 줄기 탐색용 '줄기 높이 띠' 복셀을 다시 뽑는다 (1 초마다)."""
        if not self.dirty:
            return
        c, _h = self.vm.occupied(int(self.g('display_min_hits')))
        self.loc.set_voxels(c, self.vm.height_above_ground(c))
        self.dirty = False

    # ------------------------------------------------------------ 줄기
    def on_info(self, msg: CameraInfo):
        """카메라 내부행렬·광학 프레임 저장."""
        self.k = np.array(msg.k, float).reshape(3, 3)
        self.cam_frame = msg.header.frame_id

    def on_detections(self, msg: Detection2DArray):
        """YOLO 박스 → 방향 → 복셀 맵에서 줄기 확인 → 매칭."""
        if self.g('detection_source') == 'lidar' or self.k is None or not msg.detections:
            return
        self.last_yolo = time.monotonic()
        t = self.stamp(msg)
        pose = self.pose_start(t)
        st = self.static_tf(self.cam_frame)
        if pose is None or st is None:
            return
        R_sb, p_sb = pose
        R_bc, t_bc = st                                   # 카메라 광학 → base_link
        R_sc = R_sb @ R_bc                                # 카메라 광학 → start
        cam = R_sb @ t_bc + p_sb                          # 카메라 위치 (start)
        for det in msg.detections:
            conf = det.results[0].hypothesis.score if det.results else 1.0
            if conf < self.g('min_conf'):
                continue
            b = det.bbox
            bearing, half = bbox_bearing(b.center.position.x, b.center.position.y, b.size_x, self.k, R_sc)
            hit = self.loc.locate(cam[:2], bearing, half)
            if hit is not None:
                self.reg.update(hit.x, hit.y, hit.z, float(conf), t, 'yolo')

    def on_trees(self, msg: TreeArray):
        """(YOLO 가 없을 때) LiDAR 줄기 → 복셀 맵에서 확인 → 매칭."""
        src = self.g('detection_source')
        if src == 'yolo' or (src == 'auto' and time.monotonic() - self.last_yolo < 5.0):
            return
        pose = self.pose_start(self.stamp(msg))
        if pose is None:
            return
        R_sb, p_sb = pose
        for tr in msg.trees:
            if tr.confidence < 0.3 or math.hypot(tr.position.x, tr.position.y) > 8.0:
                continue
            q = R_sb @ np.array([tr.position.x, tr.position.y, 0.0]) + p_sb
            hit = self.loc.locate_near(q[:2])
            if hit is not None:
                self.reg.update(hit.x, hit.y, hit.z, float(tr.confidence), self.stamp(msg), 'lidar')

    # ------------------------------------------------------------ 표시
    def publish(self):
        """복셀(높이 색), 확정 줄기, 궤적을 RViz 로 발행."""
        if not self.frame.ready:
            return
        stamp = self.get_clock().now().to_msg()
        frame = self.g('map_frame')
        c, h = self.vm.occupied(int(self.g('display_min_hits')))
        if len(c):
            n_max = int(self.g('display_max_voxels'))
            if len(c) > n_max:                             # 너무 많으면 관측 많은 칸 위주로 줄여 표시
                keep = np.argsort(-h)[:n_max]
                c = c[keep]
            hag = self.vm.height_above_ground(c)
            rgb = height_colors(np.nan_to_num(hag, nan=0.0))
            m = Marker(type=Marker.CUBE_LIST, action=Marker.ADD, ns='voxels', id=0)
            m.header.frame_id, m.header.stamp = frame, stamp
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = float(self.vm.voxel)
            m.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in c]
            a = float(self.g('display_alpha'))
            m.colors = [ColorRGBA(r=float(r), g=float(gg), b=float(b), a=a) for r, gg, b in rgb]
            self.pub_vox.publish(MarkerArray(markers=[m]))
        arr = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for tr in self.trunks_on_rows()[0]:
            cyl = Marker(type=Marker.CYLINDER, action=Marker.ADD, ns='trunks', id=tr.id)
            cyl.header.frame_id, cyl.header.stamp = frame, stamp
            cyl.pose.position = Point(x=tr.x, y=tr.y, z=tr.z + 0.6)
            cyl.pose.orientation.w = 1.0
            cyl.scale.x = cyl.scale.y = 0.2
            cyl.scale.z = 1.2
            if tr.source == 'yolo':
                cyl.color = ColorRGBA(r=1.0, g=0.55, b=0.1, a=0.9)
            else:
                cyl.color = ColorRGBA(r=0.7, g=0.7, b=0.7, a=0.9)
            txt = Marker(type=Marker.TEXT_VIEW_FACING, action=Marker.ADD, ns='trunk_ids', id=tr.id)
            txt.header = cyl.header
            txt.pose.position = Point(x=tr.x, y=tr.y, z=tr.z + 1.45)
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.3
            txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            txt.text = f'T{tr.id}'
            arr.markers += [cyl, txt]
        self.pub_trunks.publish(arr)
        path = Path()
        path.header.frame_id, path.header.stamp = frame, stamp
        for _t, x, y, z, yaw in self.path:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position = Point(x=x, y=y, z=z)
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            path.poses.append(ps)
        self.pub_path.publish(path)

    # ------------------------------------------------------------ 저장
    def trunks_on_rows(self):
        """(열 위 줄기, 열 밖 줄기). row_filter_tol 0 이면 거르지 않음."""
        tol = float(self.g('row_filter_tol'))
        if tol <= 0:
            return self.reg.confirmed(), []
        return self.reg.on_rows(float(self.g('row_spacing')), tol)

    def autosave(self):
        """주기 저장 (갑자기 꺼져도 남도록)."""
        if self.frame.ready and len(self.vm):
            self.save(quiet=True)

    def srv_save(self, _req, resp):
        """저장 서비스."""
        out = self.save()
        resp.success, resp.message = out is not None, out or '아직 원점이 정해지지 않았습니다'
        return resp

    def save(self, quiet: bool = False):
        """복셀 맵·줄기·궤적·메타데이터 저장. 저장 폴더 반환."""
        if not self.frame.ready:
            return None
        os.makedirs(self.out_dir, exist_ok=True)
        n_vox = self.vm.save_ply(os.path.join(self.out_dir, 'voxels.ply'), int(self.g('display_min_hits')))
        self.vm.save_npz(os.path.join(self.out_dir, 'voxel_map.npz'))
        keep, rejected = self.trunks_on_rows()
        n_tr = self.reg.save_csv(os.path.join(self.out_dir, 'trunks.csv'), keep)
        self.reg.save_csv(os.path.join(self.out_dir, 'trunks_rejected.csv'), rejected)   # 열 밖 (확인용)
        # 열별 표 (trunks_by_row.csv, trunks_table.html/md) — 보고서·한글 문서에 붙이기 쉬운 형태
        table = write_tables(self.out_dir, [dict(id=t.id, x=t.x, y=t.y, z=t.z, hits=t.hits, mean_conf=t.mean_conf,
                                                 source=t.source) for t in keep], float(self.g('row_spacing')))
        with open(os.path.join(self.out_dir, 'trajectory.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'z', 'yaw'])
            w.writerows([[f'{v:.3f}' for v in row] for row in self.path])
        meta = {
            'frame': 'start — 원점 = 미션 시작 위치, +x = 처음 곧게 달린 방향, +z = 위 (단위 m)',
            'origin_odom_xyz': [round(float(v), 3) for v in self.frame.origin],
            'x_axis_odom_heading_deg': round(math.degrees(self.frame.yaw0), 2),
            'voxel_size_m': self.vm.voxel, 'voxels_saved': n_vox, 'scans': self.vm.scans,
            'trunks_confirmed': n_tr, 'min_hits': self.reg.min_hits,
            'detection_source': self.g('detection_source'),
            'odom_drift_corrected': bool(self.g('use_odom_drift')),
            'last_odom_drift_xy': [round(v, 3) for v in self.drift],
            'trunk_sources': {s: sum(1 for t in keep if t.source == s) for s in ('yolo', 'lidar')},
            'trunks_rejected_off_row': len(rejected),
            'rows': [{'row': r.row, 'y': round(r.y, 2), 'trees': r.n, 'spacing_median': round(r.gap_med, 2)
                      if r.gap_med == r.gap_med else None, 'missing_suspected': r.missing} for r in table],
        }
        with open(os.path.join(self.out_dir, 'meta.json'), 'w', encoding='utf-8') as f:
            json.dump(meta, f, ensure_ascii=False, indent=1)
        log = self.get_logger().debug if quiet else self.get_logger().info
        log(f'복셀 맵 저장: {self.out_dir} (복셀 {n_vox}, 줄기 {n_tr}: {meta["trunk_sources"]})')
        return self.out_dir


def main(args=None):
    """노드 실행 진입점. 종료할 때 한 번 저장한다."""
    rclpy.init(args=args)
    node = VoxelMapNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.save()
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
