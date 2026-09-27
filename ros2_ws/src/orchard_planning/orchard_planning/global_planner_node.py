"""전역 경로계획 노드 — 과수원 통로 순회 + 헤드랜드 U턴(Hybrid A*), 진행 상태 관리.

실제 계산은 global_planner.OrchardPlanner (ROS 없음). 이 노드는 토픽을 모아 주기적으로 update() 를 부르고,
결과를 경로·상태·RViz 표시로 내보낸다. 주행 명령(/cmd_vel)은 local_planner_node 가 낸다.
(nav_mode:=planner 일 때 row_navigator_node 대신 뜬다 — orchard_bringup/launch/autonomy.launch.py)

입력 (ROS topic)
  /odom            nav_msgs/Odometry           로버 자세 (PX4 → orchard_px4_bridge)
  /orchard/trees   orchard_msgs/TreeArray      이번 스캔의 줄기 (base_link) → 열·통로 모델
  /orchard/row     orchard_msgs/RowCenterline  LiDAR 통로 중심선 → odom 흐름 보정, U턴 넘겨받기, 자동 출발 조건
출력
  /orchard/global_path       nav_msgs/Path              남은 전역 경로 (odom) — 지역 계획이 따라감
  /orchard/mission/state     std_msgs/String            '상태|lanes_done=N|이유|row=planner' (지도 노드와 호환)
  /orchard/plan/markers      visualization_msgs/MarkerArray  열 선분(갈색), 통로 중심선(하늘색)+번호, 누적 줄기(초록 점),
                                                            U턴 목표(주황 화살표), 상태 글자, odom 흐름 보정 화살표(빨강)
  /orchard/plan/turn_grid    nav_msgs/OccupancyGrid     U턴 탐색에 쓴 점유 격자 (나무를 로버 반경만큼 부풀림)
  /orchard/odom_drift        geometry_msgs/PointStamped x, y = 추정한 odom 위치 흐름 [m] (참 위치 ≈ odom − 흐름),
                                                        z = odom 요 오차 [rad] (참 요 ≈ odom 요 + z)
                                                        → local_planner_node, voxel_map_node
  /orchard/row_frame         geometry_msgs/PoseStamped  통로 모델 원점·열 방향 (odom) → voxel_map_node 의 start 좌표 +x 축
서비스
  /orchard/mission/start, /orchard/mission/stop (std_srvs/Trigger)

파라미터 (config/sim.yaml · robot.yaml 의 global_planner_node)
  rate_hz, auto_start, auto_start_delay, row_timeout, odom_frame,
  plan.lanes, plan.first_turn, plan.row_spacing, plan.min_turn_radius, plan.exit_margin, plan.entry_margin,
  plan.end_confirm_dist, plan.ahead, plan.tree_radius, plan.robot_radius, plan.drift_gain, plan.handover_angle,
  plan.yaw_gain, plan.yaw_step_max, plan.track_heading_tol, plan.track_lateral_tol, plan.heading_baseline,
  plan.heading_max_change, plan.approach (과수원 밖 출발 → 입구 찾기), plan.approach_side/pre/radius/memory/max_dist
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid as GridMsg
from nav_msgs.msg import Odometry, Path
from orchard_msgs.msg import RowCenterline, TreeArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from .global_planner import IDLE, OrchardPlanner, PlannerParams, RowSeen


def yaw_of(q) -> float:
    """쿼터니언 → 요 [rad]."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def path_msg(points: np.ndarray, frame: str, stamp, every: int = 1) -> Path:
    """(N×3) [x, y, yaw] → nav_msgs/Path."""
    msg = Path()
    msg.header.frame_id, msg.header.stamp = frame, stamp
    for x, y, yaw in points[::every]:
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose.position.x, ps.pose.position.y = float(x), float(y)
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        msg.poses.append(ps)
    return msg


def color(r, g, b, a=1.0) -> ColorRGBA:
    """ColorRGBA 도우미."""
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


_PLAN_KEYS = ('lanes', 'max_lanes', 'first_turn', 'row_spacing', 'min_turn_radius', 'exit_margin', 'entry_margin',
              'end_confirm_dist', 'ahead', 'min_row_trees', 'tree_radius', 'robot_radius', 'drift_gain',
              'handover_angle', 'reach_tol', 'max_turn_deviation', 'yaw_gain', 'yaw_step_max', 'track_heading_tol',
              'track_lateral_tol', 'heading_baseline', 'heading_max_change', 'approach', 'approach_side',
              'approach_pre', 'approach_radius', 'approach_memory', 'approach_max_dist')


class GlobalPlannerNode(Node):
    """전역 계획 ROS 노드 (global_planner_node)."""

    def __init__(self):
        """파라미터 선언 → OrchardPlanner → 구독·발행·서비스·타이머."""
        super().__init__('global_planner_node')
        d = self.declare_parameter
        d('rate_hz', 5.0)                 # 전역 계획 갱신 주기 [Hz]
        d('auto_start', False)            # 시뮬: odom + 통로 인식이 auto_start_delay 동안 계속되면 자동 시작
        d('auto_start_delay', 5.0)
        d('row_timeout', 1.5)             # [s] /orchard/row 가 이보다 오래되면 무효
        d('trees_timeout', 0.5)           # [s] /orchard/trees 가 이보다 오래되면 누적하지 않음
        d('odom_frame', 'odom')
        dp = PlannerParams()
        for k in _PLAN_KEYS:
            d('plan.' + k, getattr(dp, k))          # 기본값 = PlannerParams (global_planner.py) 와 같음
        p = PlannerParams(**{k: self.g('plan.' + k) for k in _PLAN_KEYS})
        p.lanes, p.max_lanes, p.min_row_trees = int(p.lanes), int(p.max_lanes), int(p.min_row_trees)
        self.planner = OrchardPlanner(p)
        self.pose = None
        self.row, self.row_time = None, None
        self.trees, self.trees_time = np.zeros((0, 2)), None
        self.ready_since = None
        self.last_state = None
        self.out = None

        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(RowCenterline, '/orchard/row', self.on_row, 10)
        self.create_subscription(TreeArray, '/orchard/trees', self.on_trees, 10)
        self.pub_path = self.create_publisher(Path, '/orchard/global_path', 2)
        self.pub_state = self.create_publisher(String, '/orchard/mission/state', 10)
        self.pub_mk = self.create_publisher(MarkerArray, '/orchard/plan/markers', 2)
        self.pub_grid = self.create_publisher(GridMsg, '/orchard/plan/turn_grid', 1)
        self.pub_drift = self.create_publisher(PointStamped, '/orchard/odom_drift', 10)
        self.pub_rowframe = self.create_publisher(PoseStamped, '/orchard/row_frame', 10)
        self.create_service(Trigger, '/orchard/mission/start', self.srv_start)
        self.create_service(Trigger, '/orchard/mission/stop', self.srv_stop)
        self.create_timer(1.0 / self.g('rate_hz'), self.tick)
        self.last_grid, self.last_grid_pub = None, -1e9
        self.get_logger().info(f'전역 계획 준비 (통로 {p.lanes if p.lanes > 0 else "탐사"}, 첫 U턴 {p.first_turn}). '
                               '시작: ros2 service call /orchard/mission/start std_srvs/srv/Trigger')

    def g(self, name):
        """파라미터 값."""
        return self.get_parameter(name).value

    def now(self) -> float:
        """ROS 시각 [s]."""
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------ 입력
    def on_odom(self, msg: Odometry):
        """odom → (x, y, yaw)."""
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_row(self, msg: RowCenterline):
        """LiDAR 통로 중심선."""
        self.row = RowSeen(msg.valid, msg.lateral_offset, msg.heading_error, msg.confidence,
                           int(msg.left_count), int(msg.right_count))
        self.row_time = self.now()

    def on_trees(self, msg: TreeArray):
        """이번 스캔 줄기 (base_link, 8 m 안)."""
        self.trees = np.array([[t.position.x, t.position.y] for t in msg.trees]).reshape(-1, 2)
        self.trees_time = self.now()

    def srv_start(self, _req, resp):
        """미션 시작."""
        if self.pose is None:
            resp.success, resp.message = False, 'odom 미수신'
            return resp
        self.planner.start(self.now(), *self.pose)
        resp.success, resp.message = True, '전역 계획 시작'
        return resp

    def srv_stop(self, _req, resp):
        """정지."""
        self.planner.stop()
        resp.success, resp.message = True, '정지'
        return resp

    # ------------------------------------------------------------ 주기
    def tick(self):
        """자동 출발 → update → 경로·상태·표시 발행."""
        now = self.now()
        row = self.row if (self.row_time is not None and now - self.row_time < self.g('row_timeout')) else None
        if self.g('auto_start') and self.planner.state == IDLE:
            # 보통은 LiDAR 통로 중심선이 보일 때 출발. 과수원 밖 출발(approach)은 열이 아직 안 보이므로 자세만 있으면 출발
            ready = self.pose is not None and (self.planner.p.approach or (row is not None and row.valid))
            if ready:
                self.ready_since = self.ready_since or now
                if now - self.ready_since > self.g('auto_start_delay'):
                    self.planner.start(now, *self.pose)
            else:
                self.ready_since = None
        fresh = self.trees_time is not None and now - self.trees_time < self.g('trees_timeout')
        trees = self.trees if fresh else None
        self.trees_time = None if trees is not None else self.trees_time       # 같은 스캔을 두 번 넣지 않게
        if self.pose is not None:
            self.out = self.planner.update(now, *self.pose, trees, row)
        stamp = self.get_clock().now().to_msg()
        frame = self.g('odom_frame')
        out = self.out
        path = out.path if out is not None else np.zeros((0, 3))
        self.pub_path.publish(path_msg(path, frame, stamp, every=2))
        st = self.planner.state
        self.pub_state.publish(String(
            data=f'{st}|lanes_done={self.planner.lanes_done}|{self.planner.reason}|row=planner'))
        if st != self.last_state:
            self.get_logger().info(f'상태 → {st} ({self.planner.reason})')
            self.last_state = st
        if out is not None:
            dm = PointStamped()
            dm.header.frame_id, dm.header.stamp = frame, stamp
            dm.point.x, dm.point.y = float(out.drift[0]), float(out.drift[1])
            dm.point.z = float(out.yaw_bias)                  # z 칸에 요 오차 [rad] 를 실어 보냄
            self.pub_drift.publish(dm)                         # 복셀 맵 노드가 빼고 쌓는다
            m = self.planner.model
            if m is not None:                                  # 통로 모델 원점·열 방향 → 복셀 맵 start 좌표 +x 축
                rf = PoseStamped()
                rf.header.frame_id, rf.header.stamp = frame, stamp
                rf.pose.position.x, rf.pose.position.y = float(m.o[0] + out.drift[0]), float(m.o[1] + out.drift[1])
                rf.pose.orientation.z, rf.pose.orientation.w = math.sin(m.theta / 2), math.cos(m.theta / 2)
                self.pub_rowframe.publish(rf)
            self.publish_markers(out, frame, stamp)
            if out.turn_grid is not None and (out.turn_grid is not self.last_grid or now - self.last_grid_pub > 2.0):
                self.publish_grid(out.turn_grid, out.drift, frame, stamp)      # 새 격자이거나 2 s 마다 다시
                self.last_grid, self.last_grid_pub = out.turn_grid, now

    # ------------------------------------------------------------ 표시
    def publish_markers(self, out, frame, stamp):
        """열·통로·줄기·U턴 목표·상태·흐름 보정을 MarkerArray 로."""
        arr = MarkerArray()

        def mk(ns, i, typ, scale, col):
            m = Marker()
            m.header.frame_id, m.header.stamp = frame, stamp
            m.ns, m.id, m.type, m.action = ns, i, typ, Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = float(scale)
            m.color = col
            return m
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        rows = mk('rows', 0, Marker.LINE_LIST, 0.08, color(0.55, 0.35, 0.15))
        for a, b in out.rows:
            rows.points += [Point(x=float(a[0]), y=float(a[1]), z=0.3), Point(x=float(b[0]), y=float(b[1]), z=0.3)]
        arr.markers.append(rows)
        lanes = mk('lanes', 0, Marker.LINE_LIST, 0.05, color(0.3, 0.8, 1.0, 0.8))
        for i, (a, b, k) in enumerate(out.lane_lines):
            lanes.points += [Point(x=float(a[0]), y=float(a[1]), z=0.05), Point(x=float(b[0]), y=float(b[1]), z=0.05)]
            t = mk('lane_ids', i, Marker.TEXT_VIEW_FACING, 0.5, color(0.3, 0.8, 1.0))
            t.pose.position.x, t.pose.position.y, t.pose.position.z = float(a[0]), float(a[1]), 0.8
            t.text = f'lane {k}' + (' <' if k == out.lane else '')     # RViz 글꼴은 한글이 안 나와 영어로
            arr.markers.append(t)
        arr.markers.append(lanes)
        if len(out.trees):
            tr = mk('trees', 0, Marker.POINTS, 0.18, color(0.2, 0.9, 0.2))
            tr.points = [Point(x=float(x), y=float(y), z=0.4) for x, y in out.trees]
            arr.markers.append(tr)
        goal = self.planner.turn_goal
        if goal is not None and self.planner.state in ('FOLLOW_ROW', 'TURN') and self.planner.turn_path is not None:
            ga = mk('turn_goal', 0, Marker.ARROW, 1.0, color(1.0, 0.55, 0.0))
            gx, gy = goal[0] + out.drift[0], goal[1] + out.drift[1]
            ga.points = [Point(x=gx, y=gy, z=0.3),
                         Point(x=gx + 1.2 * math.cos(goal[2]), y=gy + 1.2 * math.sin(goal[2]), z=0.3)]
            ga.scale.x, ga.scale.y, ga.scale.z = 0.12, 0.25, 0.3
            arr.markers.append(ga)
        ent = getattr(self.planner, 'app_ent', None)
        if self.planner.state == 'APPROACH' and ent is not None:   # 과수원 진입: 정렬점 화살표(초록) + 입구 글자
            ea = mk('entry', 0, Marker.ARROW, 1.0, color(0.1, 1.0, 0.3))
            px, py = float(ent.pre[0]), float(ent.pre[1])
            ea.points = [Point(x=px, y=py, z=0.3),
                         Point(x=px + 1.5 * math.cos(ent.heading), y=py + 1.5 * math.sin(ent.heading), z=0.3)]
            ea.scale.x, ea.scale.y, ea.scale.z = 0.15, 0.3, 0.3
            arr.markers.append(ea)
            et = mk('entry_txt', 0, Marker.TEXT_VIEW_FACING, 0.45, color(0.1, 1.0, 0.3))
            et.pose.position.x, et.pose.position.y, et.pose.position.z = float(ent.entry[0]), float(ent.entry[1]), 1.2
            et.text = f'entrance ({ent.side} end lane, {ent.n_rows} rows seen)'
            arr.markers.append(et)
        if self.pose is not None:
            x, y, _ = self.pose
            txt = mk('state', 0, Marker.TEXT_VIEW_FACING, 0.45, color(1, 1, 1))
            txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = x, y + 2.6, 1.0   # 로버 왼쪽 위
            txt.text = (f'{self.planner.state}  lane {out.lane}  done {out.lanes_done}\n'
                        f'odom drift ({out.drift[0]:+.2f}, {out.drift[1]:+.2f}) m  '
                        f'yaw {math.degrees(out.yaw_bias):+.1f} deg')
            arr.markers.append(txt)
            if math.hypot(*out.drift) > 0.02:                   # odom 흐름 보정: 모델 → odom 방향 (빨간 화살표)
                da = mk('drift', 0, Marker.ARROW, 1.0, color(1.0, 0.2, 0.2))
                da.points = [Point(x=x - out.drift[0], y=y - out.drift[1], z=1.2), Point(x=x, y=y, z=1.2)]
                da.scale.x, da.scale.y, da.scale.z = 0.06, 0.15, 0.2
                arr.markers.append(da)
        self.pub_mk.publish(arr)

    def publish_grid(self, grid, drift, frame, stamp):
        """U턴 점유 격자 (model → odom 이동) 를 OccupancyGrid 로."""
        msg = GridMsg()
        msg.header.frame_id, msg.header.stamp = frame, stamp
        msg.info.resolution = float(grid.res)
        msg.info.width, msg.info.height = int(grid.nx), int(grid.ny)
        msg.info.origin.position.x = float(grid.x0 + drift[0])
        msg.info.origin.position.y = float(grid.y0 + drift[1])
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.to_int8().tolist()
        self.pub_grid.publish(msg)


def main(args=None):
    """노드 실행 진입점 (ros2 run orchard_planning global_planner_node)."""
    rclpy.init(args=args)
    node = GlobalPlannerNode()
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
