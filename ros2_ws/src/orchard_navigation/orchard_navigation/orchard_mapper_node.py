"""과수원 나무 지도 노드: /orchard/trees(base_link) → odom 좌표로 누적 → 열/나무 번호·결주 → 저장.

파이프라인에서의 역할
  주행(행 추종 → 행 끝 → U턴 → 다음 통로)과 나란히 돌며 나무 지도를 만든다. 주행 명령에는 관여하지 않는다.
  실제 계산(누적·열 정리·저장·보정 추정)은 treemap.py, 이 노드는 ROS 입출력과 보정 시점 관리만 한다.

구독: /orchard/trees (TreeArray), /orchard/mission/state (DONE 이면 자동 저장), /gps/fix + /odom (위경도 변환용)
발행: /orchard/map/markers (RViz: 나무 = 초록 원기둥 + 'R열-T번호', 결주 후보 = 빨간 공)
서비스: /orchard/map/save (std_srvs/Trigger)
저장: <output_dir>/orchard_map_<시작 날짜시각>/trees.csv, summary.json
      실행 하나에 폴더 하나. autosave_period 마다 덮어써서, 전원이 갑자기 꺼져도 마지막 지도가 남는다
자세: /odom 기록을 검출 시각에 맞춰 보간 (TF 최신값으로 대신하면 회전 중 지도가 번짐)
시각 보정(estimate_time_offset): odom 과 LiDAR 사이 지연이 있으면 달릴 때와 설 때 같은 나무가 v·지연 만큼
      어긋나 두 번 찍힌다 (시뮬에서 약 0.7 m 확인). 지도가 가장 뾰족해지는 시각 보정을 찾아 적용한다.
요 보정(estimate_yaw_bias): 나침반(자력계) 방위 오차는 과수원 철제 지주 등으로 흔하고, 위치는 GPS 라 맞아도
      나무가 로버 둘레로 돌아간 채 찍혀 U턴 뒤 같은 열이 두 줄로 겹친다. 곧게 전진할 때
      이동 방향(위치 차) − 자세 요 = 방위 오차로 추정해 보정하고, 추정이 바뀌면 원자료로 지도를 다시 만든다.
      출발 전 정지 중에는 PX4 방위가 아직 GPS 로 맞춰지지 않아(시뮬에서 약 15° 틀어짐 확인) 첫 직진 구간 이전 검출은 버린다.

파라미터 (ros2_ws/src/orchard_bringup/config/sim.yaml, robot.yaml 의 orchard_mapper_node 항목)
  row_spacing [m] (열 나누기 기준), max_range [m], assoc_radius [m] (같은 나무로 합칠 거리), min_hits,
  output_dir, autosave_period [s], estimate_yaw_bias, estimate_time_offset, time_offset [s].
  같은 나무가 두 개로 찍히면 assoc_radius 를 키우거나 time_offset/estimate_* 를 확인한다.
"""
from __future__ import annotations

import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from orchard_msgs.msg import TreeArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .treemap import PoseHistory, TreeMap, YawBiasEstimator, best_time_offset, project


def _yaw(q) -> float:
    """쿼터니언 (geometry_msgs/Quaternion) → 요 [rad] (z 축 회전, ZYX 오일러)."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class OrchardMapperNode(Node):
    """나무 지도 ROS 노드 (orchard_mapper_node)."""

    def __init__(self):
        """파라미터 선언, 구독/발행/서비스/타이머 생성."""
        super().__init__('orchard_mapper_node')
        d = self.declare_parameter
        d('map_frame', 'odom')
        d('base_frame', 'base_link')
        d('row_spacing', 3.8)
        d('assoc_radius', 0.35)          # [m] 기존 나무와 이 거리 안이면 같은 나무
        d('min_hits', 3)                 # 이만큼 관측돼야 확정 나무
        d('max_range', 8.0)             # 이 거리 안의 줄기만 지도에 넣는다 (먼 검출은 부정확)
        d('min_confidence', 0.3)         # 줄기 검출 신뢰도가 이보다 낮으면 버림
        d('publish_period', 2.0)         # [s] RViz 마커 발행 주기
        d('output_dir', os.path.expanduser('~/orchard_maps'))
        d('autosave_period', 30.0)       # [s] 0 이면 끔 (DONE·종료·서비스 때만 저장)
        d('estimate_yaw_bias', True)
        d('bias_segment', 1.0)           # [m] 이 거리를 곧게 갈 때마다 방위 오차 표본 하나
        d('bias_min_samples', 3)         # 표본이 이만큼 모이기 전 검출은 보관만 (나중에 보정해 반영)
        d('time_offset', 0.0)            # [s] 검출 시각에 더해 자세를 찾음 (odom 이 앞서면 음수)
        d('estimate_time_offset', True)  # 자동 추정 (지도 다시 만들 때·자동 저장 때)
        self.map = TreeMap(self.g('assoc_radius'), int(self.g('min_hits')))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.geo = None                  # odom 원점의 (lat, lon)
        self.last_odom = None
        self.hist = PoseHistory()        # odom (t, x, y, yaw)
        self.raw = []                    # (t, pts(base_link Nx2), radii) 지도 재구성용 원자료
        self.time_offset = float(self.g('time_offset'))
        self.bias_est = YawBiasEstimator(segment=self.g('bias_segment'))
        self.yaw_bias = 0.0
        self.applied_bias = 0.0
        self.saved_done = False
        self.pub = self.create_publisher(MarkerArray, '/orchard/map/markers', 1)
        self.create_subscription(TreeArray, '/orchard/trees', self.on_trees, 10)
        self.create_subscription(String, '/orchard/mission/state', self.on_state, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.on_fix, 10)
        self.create_service(Trigger, '/orchard/map/save', self.srv_save)
        self.create_timer(self.g('publish_period'), self.publish)
        self.out_dir = os.path.join(self.g('output_dir'), time.strftime('orchard_map_%Y%m%d_%H%M%S'))
        self.saved_n = 0                 # 마지막 저장 때 나무 수 (같으면 자동 저장 생략)
        if self.g('autosave_period') > 0:
            self.create_timer(self.g('autosave_period'), self.autosave)
        self.get_logger().info(f"나무 지도 누적 ({self.g('map_frame')}), 저장 위치 {self.g('output_dir')}")

    def g(self, name):
        """파라미터 값 읽기 (짧게 쓰려는 도우미)."""
        return self.get_parameter(name).value

    def on_trees(self, msg: TreeArray):
        """줄기 검출 수신: base_link 로 옮겨 신뢰도·거리로 거르고 원자료(self.raw)에 보관.

        방위 보정이 준비됐으면 바로 지도에 반영, 아니면 보관만 했다가 rebuild() 때 반영한다.
        """
        if not msg.trees:
            return
        t_msg = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        static = self.static_tf(msg.header.frame_id)
        if static is None:
            return
        pts, radii = [], []
        for tr in msg.trees:
            p = tr.position
            if tr.confidence < self.g('min_confidence'):
                continue
            # 검출 프레임 → base_link 2D 변환 (회전 후 평행이동)
            x, y = static[0] + static[2] * p.x - static[3] * p.y, static[1] + static[3] * p.x + static[2] * p.y
            if math.hypot(x, y) > self.g('max_range'):
                continue
            pts.append((x, y))
            radii.append(tr.radius)
        if not pts:
            return
        rec = (t_msg, np.array(pts), np.array(radii))
        self.raw.append(rec)
        if self.bias_ready():
            self.integrate([rec])

    def bias_ready(self):
        """방위 오차 추정이 꺼져 있거나, 표본이 bias_min_samples 개 이상 모였으면 True."""
        return not self.g('estimate_yaw_bias') or len(self.bias_est.samples) >= int(self.g('bias_min_samples'))

    def integrate(self, recs):
        """원자료 [(t, base_link 점, 반경)] 를 현재 시각·방위 보정으로 odom 에 투영해 지도에 누적."""
        for t, w, radii in project(recs, self.hist, self.time_offset, self.applied_bias):
            self.map.update(w, radii, t)

    def calibrate_time(self):
        """자세 시각 보정 자동 추정. 0.02 s 넘게 바뀌면 True."""
        if not self.g('estimate_time_offset') or len(self.raw) < 20:
            return False
        off, sc, sc0 = best_time_offset(self.raw, self.hist, self.applied_bias)
        # 0.02 s 이하 변화는 무시, 보정 0 보다 선명도가 5 % 이상 좋아질 때만 적용
        if abs(off - self.time_offset) <= 0.02 or sc < 1.05 * sc0:     # 뚜렷이 나아질 때만
            return False
        self.get_logger().info(f'자세 시각 보정 {off:+.2f} s (지도 선명도 {sc / max(sc0, 1e-9):.2f}배)')
        self.time_offset = off
        return True

    def rebuild(self, calibrate=True):
        """현재 방위 보정(과 필요하면 새 시각 보정)으로 원자료에서 지도를 처음부터 다시 만든다."""
        self.applied_bias = self.yaw_bias
        t0 = self.bias_est.valid_since if self.g('estimate_yaw_bias') else None
        if t0 is not None:                          # 방위 확인 전(출발 전 정지 등) 검출은 버림
            self.raw = [r for r in self.raw if r[0] >= t0]
        if calibrate:
            self.calibrate_time()
        self.map = TreeMap(self.g('assoc_radius'), int(self.g('min_hits')))
        self.integrate(self.raw)

    def static_tf(self, frame):
        """검출 프레임 → base_link (고정 장착). (tx, ty, cos, sin)"""
        if frame in ('', self.g('base_frame')):
            return (0.0, 0.0, 1.0, 0.0)
        try:
            tf = self.tf_buffer.lookup_transform(self.g('base_frame'), frame, Time())
        except TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        a = _yaw(q)
        return (t.x, t.y, math.cos(a), math.sin(a))

    def on_odom(self, msg: Odometry):
        """odom 수신: 자세 기록(검출 시각 보간용)에 넣고 방위 오차 추정을 갱신."""
        self.last_odom = msg
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        p = msg.pose.pose.position
        yaw = _yaw(msg.pose.pose.orientation)
        self.hist.add(t, p.x, p.y, yaw)
        if self.g('estimate_yaw_bias'):
            self.update_bias(p.x, p.y, yaw, msg.twist.twist.angular.z, t)

    def update_bias(self, x, y, yaw, wz, t):
        """방위 오차 표본 갱신. 처음 bias_min_samples 개가 모였을 때, 또는 추정이 1° 넘게 바뀌면 지도 재구성.

        x, y [m], yaw [rad], wz: 요레이트 [rad/s], t [s].
        """
        if not self.bias_est.update(x, y, yaw, wz, t):
            return
        self.yaw_bias = self.bias_est.bias
        n = len(self.bias_est.samples)
        if n == int(self.g('bias_min_samples')) or \
                (self.bias_ready() and abs(self.yaw_bias - self.applied_bias) > math.radians(1.0)):
            self.get_logger().info(f'방위 오차 추정 {math.degrees(self.yaw_bias):+.1f}° (표본 {n}) → 지도 다시 만듦')
            self.rebuild()

    def on_fix(self, msg: NavSatFix):
        """첫 GPS fix 로 odom 원점의 위경도를 한 번 계산 (odom 이 ENU: x 동 / y 북 이라고 가정)."""
        if self.geo is not None or self.last_odom is None or not math.isfinite(msg.latitude):
            return
        p = self.last_odom.pose.pose.position          # ENU [m] → 원점 위경도로 되돌림
        R = 6378137.0                                  # [m] WGS84 적도 반경 (구면 근사)
        lat0 = msg.latitude - math.degrees(p.y / R)
        lon0 = msg.longitude - math.degrees(p.x / (R * math.cos(math.radians(msg.latitude))))
        self.geo = (lat0, lon0)

    def on_state(self, msg: String):
        """미션 상태 수신: 'DONE...' 이 처음 오면 한 번 저장."""
        if msg.data.startswith('DONE') and not self.saved_done:
            self.saved_done = True
            self.save()

    def srv_save(self, _req, resp):
        """/orchard/map/save 서비스: 저장하고 폴더 경로(또는 실패 이유)를 돌려준다."""
        path = self.save()
        resp.success, resp.message = path is not None, path or '저장할 나무가 없습니다'
        return resp

    def autosave(self):
        """주기 자동 저장: 필요하면 시각 보정을 다시 추정하고, 확정 나무 수가 바뀌었을 때만 저장."""
        if self.bias_ready() and self.calibrate_time():
            self.rebuild(calibrate=False)
        n = len(self.map.confirmed())
        if n and n != self.saved_n:
            self.save(quiet=True)

    def save(self, quiet=False):
        """지도를 정리(organize)해 trees.csv / summary.json 으로 저장. 저장 폴더 경로, 나무가 없으면 None.

        quiet=True (자동 저장): 로그를 debug 로, 방위 보정이 준비됐으면 재구성을 생략.
        """
        if self.raw and (not self.bias_ready() or not quiet):  # 최종 저장 때는 시각 보정도 다시
            self.rebuild()
        summary = self.map.organize(self.g('row_spacing'))
        if not summary['trees']:
            return None
        summary['yaw_bias_deg'] = round(math.degrees(self.applied_bias), 2)
        summary['time_offset_s'] = round(self.time_offset, 3)
        out = self.out_dir
        os.makedirs(out, exist_ok=True)
        self.map.save(summary, os.path.join(out, 'trees.csv'), os.path.join(out, 'summary.json'), self.geo)
        self.saved_n = len(summary['trees'])
        rows = ', '.join(f"{r['row']}열 {r['num_trees']}그루(결주 {len(r['missing'])})" for r in summary['rows'])
        rows += (f", 이상점 {summary.get('outliers', 0)}, 방위 보정 {summary['yaw_bias_deg']:+.1f}°, "
                 f"시각 보정 {self.time_offset:+.2f} s")
        log = self.get_logger().debug if quiet else self.get_logger().info
        log(f'나무 지도 저장: {out}  [{rows}]')
        return out

    def publish(self):
        """RViz 마커 발행: 나무 = 초록 원기둥 + '열-번호' 글자, 결주 후보 = 빨간 공 (map_frame 기준)."""
        summary = self.map.organize(self.g('row_spacing'))
        arr = MarkerArray()
        arr.markers.append(Marker(action=Marker.DELETEALL))
        stamp = self.get_clock().now().to_msg()
        frame = self.g('map_frame')
        for i, t in enumerate(summary['trees']):
            m = Marker(type=Marker.CYLINDER, action=Marker.ADD, ns='trees', id=i)
            m.header.frame_id, m.header.stamp = frame, stamp
            m.pose.position = Point(x=float(t['x']), y=float(t['y']), z=0.4)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = max(0.1, 2 * t['radius'])
            m.scale.z = 0.8
            m.color.g, m.color.r, m.color.a = 0.8, 0.2, 0.8
            arr.markers.append(m)
            lab = Marker(type=Marker.TEXT_VIEW_FACING, action=Marker.ADD, ns='labels', id=i)
            lab.header = m.header
            lab.pose.position = Point(x=float(t['x']), y=float(t['y']), z=1.1)
            lab.pose.orientation.w = 1.0
            lab.scale.z = 0.18
            lab.color.r = lab.color.g = lab.color.b = lab.color.a = 1.0
            lab.text = f"{t['row']}-{t['tree']}"
            arr.markers.append(lab)
        k = 0
        for r in summary['rows']:
            for miss in r['missing']:
                m = Marker(type=Marker.SPHERE, action=Marker.ADD, ns='missing', id=k)
                m.header.frame_id, m.header.stamp = frame, stamp
                m.pose.position = Point(x=float(miss['x']), y=float(miss['y']), z=0.3)
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = 0.3
                m.color.r, m.color.a = 1.0, 0.9
                arr.markers.append(m)
                k += 1
        self.pub.publish(arr)


def main(args=None):
    """노드 실행 진입점 (ros2 run orchard_navigation orchard_mapper_node). 종료할 때도 지도를 저장한다."""
    rclpy.init(args=args)
    node = OrchardMapperNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.save()                 # 종료할 때도 한 번 저장
        except Exception:               # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
