"""지역 경로계획 노드 — DWA 로 전역 경로를 따라가며 LiDAR 장애물을 피하는 /cmd_vel 생성.

실제 계산은 dwa.DWAPlanner 와 obstacles.obstacle_points (ROS 없음). 이 노드는 토픽을 모으고 결과·표시를 낸다.
(nav_mode:=planner 일 때 row_navigator_node 대신 뜬다 — orchard_bringup/launch/autonomy.launch.py)

입력 (ROS topic)
  /orchard/global_path    nav_msgs/Path          전역 경로 (global_planner_node)
  /orchard/mission/state  std_msgs/String        APPROACH / FOLLOW_ROW / TURN 일 때만 달림 (그 밖은 정지 명령)
  /orchard/odom_drift     geometry_msgs/PointStamped  z = odom 요 오차 [rad] (전역 계획 추정) → 요에 더해 씀
  /odom                   nav_msgs/Odometry      로버 자세
  /livox/lidar            sensor_msgs/PointCloud2  장애물 추출 (지면 위 obstacle.h_min ~ h_max)
출력
  /cmd_vel                geometry_msgs/Twist    → orchard_px4_bridge (PX4 Offboard)
  /orchard/local_path     nav_msgs/Path          고른 궤적 (odom)
  /orchard/local/markers  visualization_msgs/MarkerArray
       candidates  후보 궤적 (초록 = 통과, 진할수록 비용 낮음 / 빨강 = 장애물로 탈락)
       obstacles   장애물 점 (주황 상자)
       footprint   차체 근사 원 3개 (base_link, 파랑)
       status      상태 글자 (ok/escape/blocked, 속도, 요레이트, 장애물 여유, 속도 한계)
  /orchard/local/status   std_msgs/String        'ok|v=..|w=..|clear=..' (로그·녹화 스크립트용)

파라미터 (config/sim.yaml · robot.yaml 의 local_planner_node)
  rate_hz, cloud_topic, base_frame, odom_frame, path_timeout, obstacle_rate_hz,
  obstacle.h_min / h_max / max_range / nominal_ground_z, dwa.* (dwa.DWAParams 참고)
"""
from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .dwa import DWAParams, DWAPlanner
from .global_planner_node import color, path_msg, yaw_of
from .obstacles import obstacle_points

_DWA_KEYS = ('max_speed', 'max_accel', 'max_decel', 'min_turn_radius', 'n_curvatures', 'n_curvatures2', 'arc1',
             'arc2', 'footprint_radius', 'safety', 'obstacle_influence', 'max_lat_accel', 'goal_lookahead',
             'w_path', 'w_goal', 'w_heading', 'w_obstacle', 'w_smooth', 'escape_speed', 'escape_margin')


def _quat_to_matrix(q) -> np.ndarray:
    """쿼터니언 → 3×3 회전행렬."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class LocalPlannerNode(Node):
    """지역 계획 ROS 노드 (local_planner_node)."""

    def __init__(self):
        """파라미터 선언 → DWA → 구독·발행·타이머."""
        super().__init__('local_planner_node')
        d = self.declare_parameter
        d('rate_hz', 10.0)                # 제어 주기 [Hz]
        d('obstacle_rate_hz', 5.0)        # 장애물 추출 주기 [Hz] (점군 처리 부하 조절)
        d('cloud_topic', '/livox/lidar')
        d('base_frame', 'base_link')
        d('odom_frame', 'odom')
        d('path_timeout', 2.0)            # [s] 전역 경로가 이보다 오래되면 정지
        d('obstacle_timeout', 1.0)        # [s] 장애물 정보가 이보다 오래되면 비운다
        d('obstacle.h_min', 0.2)          # 지면 위 이 높이부터 장애물 (잔디 제외) [m]
        d('obstacle.h_max', 0.75)         # 이 높이까지 (차체 높이 + 여유, 처진 가지는 통과) [m]
        d('obstacle.max_range', 7.0)
        d('obstacle.nominal_ground_z', -0.1)   # 평지에서 base_link 기준 지면 z (lidar_tree_node ground.nominal_z 와 같게)
        d('debug_dir', '')                # 비우지 않으면 'blocked' 가 이어질 때 30 s 마다 자세·경로·장애물 점을 npz 로 저장 (원인 분석용)
        dp = DWAParams()
        for k in _DWA_KEYS:
            d('dwa.' + k, getattr(dp, k))
        p = DWAParams(**{k: self.g('dwa.' + k) for k in _DWA_KEYS})
        p.n_curvatures, p.n_curvatures2 = int(p.n_curvatures), int(p.n_curvatures2)
        self.dwa = DWAPlanner(p)
        self.p = p
        self.pose = None
        self.path, self.path_time = None, None
        self.state = 'IDLE'
        self.obs, self.obs_time = np.zeros((0, 2)), None
        self.last_cloud = 0.0
        self.v_cmd = 0.0
        self.last_dump = -1e9
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_cache = {}

        self.create_subscription(Path, '/orchard/global_path', self.on_path, 2)
        self.create_subscription(String, '/orchard/mission/state', self.on_state, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.yaw_bias = 0.0
        self.create_subscription(PointStamped, '/orchard/odom_drift', self.on_drift, 10)
        self.create_subscription(PointCloud2, self.g('cloud_topic'), self.on_cloud, qos_profile_sensor_data)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_path = self.create_publisher(Path, '/orchard/local_path', 2)
        self.pub_mk = self.create_publisher(MarkerArray, '/orchard/local/markers', 2)
        self.pub_status = self.create_publisher(String, '/orchard/local/status', 10)
        self.create_timer(1.0 / self.g('rate_hz'), self.tick)
        self.get_logger().info('지역 계획(DWA) 준비')

    def g(self, name):
        """파라미터 값."""
        return self.get_parameter(name).value

    def now(self) -> float:
        """ROS 시각 [s]."""
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------ 입력
    def on_drift(self, msg: PointStamped):
        """전역 계획이 추정한 odom 요 오차 [rad] (z). 전역 경로가 이 보정을 한 좌표라 같이 맞춘다."""
        self.yaw_bias = float(msg.point.z)

    def on_odom(self, msg: Odometry):
        """odom → (x, y, 보정한 yaw)."""
        q = msg.pose.pose
        self.pose = (q.position.x, q.position.y, yaw_of(q.orientation) + self.yaw_bias)

    def on_state(self, msg: String):
        """미션 상태 (첫 필드)."""
        self.state = msg.data.split('|', 1)[0]

    def on_path(self, msg: Path):
        """전역 경로 → (N×3)."""
        self.path = np.array([[ps.pose.position.x, ps.pose.position.y, yaw_of(ps.pose.orientation)]
                              for ps in msg.poses]).reshape(-1, 3)
        self.path_time = self.now()

    def static_tf(self, frame: str):
        """base_link ← frame 고정 변환 (R, t), 캐시."""
        if self.static_cache.get(frame) is None:
            try:
                tf = self.tf_buffer.lookup_transform(self.g('base_frame'), frame, Time())
                tr = tf.transform.translation
                self.static_cache[frame] = (_quat_to_matrix(tf.transform.rotation), np.array([tr.x, tr.y, tr.z]))
            except TransformException:
                return None
        return self.static_cache[frame]

    def on_cloud(self, msg: PointCloud2):
        """점군 → base_link → 장애물 xy → odom (obstacle_rate_hz 로 제한)."""
        t0 = time.monotonic()
        if t0 - self.last_cloud < 1.0 / self.g('obstacle_rate_hz') or self.pose is None:
            return
        st = self.static_tf(msg.header.frame_id)
        if st is None:
            return
        self.last_cloud = t0
        arr = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if arr.size == 0:
            return
        P = np.stack([arr['x'], arr['y'], arr['z']], axis=-1).astype(np.float64)
        R, t = st
        P = P @ R.T + t
        ob = obstacle_points(P, self.g('obstacle.h_min'), self.g('obstacle.h_max'), self.g('obstacle.max_range'),
                             nominal_ground_z=self.g('obstacle.nominal_ground_z'))
        x, y, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        self.obs = np.c_[x + c * ob[:, 0] - s * ob[:, 1], y + s * ob[:, 0] + c * ob[:, 1]] if len(ob) else ob
        self.obs_time = self.now()

    # ------------------------------------------------------------ 주기
    def tick(self):
        """DWA → /cmd_vel, 고른 궤적, 표시."""
        now = self.now()
        cmd = Twist()
        res = None
        active = self.state in ('APPROACH', 'FOLLOW_ROW', 'TURN')
        fresh = self.path_time is not None and now - self.path_time < self.g('path_timeout')
        if self.obs_time is not None and now - self.obs_time > self.g('obstacle_timeout'):
            self.obs = np.zeros((0, 2))
        if active and fresh and self.pose is not None and self.path is not None and len(self.path) > 1:
            res = self.dwa.plan(*self.pose, self.v_cmd, self.path, self.obs)
            cmd.linear.x, cmd.angular.z = float(res.v), float(res.w)
            if res.status == 'blocked':
                self.get_logger().warn('지역 계획: 모든 후보가 장애물에 막힘 → 정지', throttle_duration_sec=5.0)
                self.dump_blocked()
            elif res.status == 'escape':
                self.get_logger().warn('지역 계획: 장애물에 너무 붙음 → 멀어지는 궤적으로 천천히 빠져나옴', throttle_duration_sec=5.0)
        self.v_cmd = cmd.linear.x
        self.pub_cmd.publish(cmd)
        stamp = self.get_clock().now().to_msg()
        status = res.status if res is not None else ('idle' if not active else 'no_path')
        clear = res.clearance if res is not None else math.inf
        self.pub_status.publish(String(data=f'{status}|v={cmd.linear.x:.2f}|w={cmd.angular.z:.2f}|clear={clear:.2f}'))
        if res is not None and len(res.best):
            self.pub_path.publish(path_msg(res.best, self.g('odom_frame'), stamp))
        self.publish_markers(res, status, cmd, stamp)

    # ------------------------------------------------------------ 표시
    def dump_blocked(self):
        """막힘 원인 분석용: 자세·전역 경로·장애물 점(odom)을 debug_dir/dwa_blocked_<시각>.npz 로 (30 s 에 한 번)."""
        out_dir = self.g('debug_dir')
        now = time.time()
        if not out_dir or now - self.last_dump < 30.0:
            return
        self.last_dump = now
        try:
            import os
            os.makedirs(out_dir, exist_ok=True)
            fn = os.path.join(out_dir, time.strftime('dwa_blocked_%Y%m%d_%H%M%S.npz'))
            np.savez(fn, pose=np.asarray(self.pose, float), path=np.asarray(self.path, float), obstacles=self.obs,
                     v_cmd=self.v_cmd)
            self.get_logger().info(f'막힘 자료 저장: {fn}')
        except OSError as e:
            self.get_logger().warn(f'막힘 자료 저장 실패: {e}')

    def publish_markers(self, res, status, cmd, stamp):
        """후보 궤적·장애물·차체·상태 글자."""
        odom, base = self.g('odom_frame'), self.g('base_frame')
        arr = MarkerArray()

        def mk(ns, frame, typ, scale):
            m = Marker()
            m.header.frame_id, m.header.stamp = frame, stamp
            m.ns, m.id, m.type, m.action = ns, 0, typ, Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = float(scale)
            return m
        cand = mk('candidates', odom, Marker.LINE_LIST, 0.02)
        if res is not None and res.candidates:
            best_by_k = {}                                      # 첫 곡률마다 가장 좋은 것 하나만 (표시 가볍게)
            for traj, cost, ok in res.candidates:
                key = round(float(math.atan2(traj[1, 1] - traj[0, 1], traj[1, 0] - traj[0, 0])), 3)
                if key not in best_by_k or (ok, -cost) > (best_by_k[key][2], -best_by_k[key][1]):
                    best_by_k[key] = (traj, cost, ok)
            costs = [c for _t, c, ok in best_by_k.values() if ok]
            lo, hi = (min(costs), max(costs)) if costs else (0.0, 1.0)
            for traj, cost, ok in best_by_k.values():
                if ok:
                    f = (cost - lo) / max(hi - lo, 1e-6)             # 0 = 가장 좋음
                    col = color(0.1 + 0.6 * f, 1.0 - 0.5 * f, 0.1, 0.9 - 0.5 * f)
                else:
                    col = color(1.0, 0.1, 0.1, 0.5)
                pts = traj[::3]
                for a, b in zip(pts[:-1], pts[1:]):
                    cand.points += [Point(x=float(a[0]), y=float(a[1]), z=0.1),
                                    Point(x=float(b[0]), y=float(b[1]), z=0.1)]
                    cand.colors += [col, col]
        arr.markers.append(cand)
        best = mk('best', odom, Marker.LINE_STRIP, 0.08)
        best.color = color(1.0, 0.9, 0.0)
        if res is not None and len(res.best):
            best.points = [Point(x=float(x), y=float(y), z=0.12) for x, y, _ in res.best[::2]]
        arr.markers.append(best)
        obs = mk('obstacles', odom, Marker.CUBE_LIST, 0.15)
        obs.color = color(1.0, 0.5, 0.0, 0.9)
        obs.points = [Point(x=float(x), y=float(y), z=0.3) for x, y in self.obs]
        arr.markers.append(obs)
        fp = mk('footprint', base, Marker.LINE_LIST, 0.02)
        fp.color = color(0.2, 0.5, 1.0)
        r = self.p.footprint_radius
        ang = np.linspace(0, 2 * math.pi, 25)
        for off in self.p.footprint_offsets:
            xs, ys = off + r * np.cos(ang), r * np.sin(ang)
            for i in range(len(ang) - 1):
                fp.points += [Point(x=float(xs[i]), y=float(ys[i]), z=0.2),
                              Point(x=float(xs[i + 1]), y=float(ys[i + 1]), z=0.2)]
        arr.markers.append(fp)
        txt = mk('status', odom, Marker.TEXT_VIEW_FACING, 0.35)
        if self.pose is not None:                                # 로버 오른쪽 아래 (전역 상태 글자와 안 겹치게)
            txt.pose.position.x, txt.pose.position.y = self.pose[0], self.pose[1] - 2.6
        txt.pose.position.z = 1.0
        txt.color = {'blocked': color(1.0, 0.3, 0.3), 'escape': color(1.0, 0.6, 0.2)}.get(status, color(1.0, 1.0, 0.6))
        lim = res.speed_limit if res is not None else 0.0
        clr = res.clearance if res is not None else math.inf
        txt.text = (f'DWA {status}  v {cmd.linear.x:.2f} m/s  w {cmd.angular.z:+.2f} rad/s\n'
                    f'clearance {clr:.2f} m  speed limit {lim:.2f}')
        arr.markers.append(txt)
        self.pub_mk.publish(arr)


def main(args=None):
    """노드 실행 진입점 (ros2 run orchard_planning local_planner_node)."""
    rclpy.init(args=args)
    node = LocalPlannerNode()
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
