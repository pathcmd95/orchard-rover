"""과수원 행 순회 자율주행 노드.

파이프라인에서의 역할
  인식(orchard_perception: LiDAR 줄기·행 중심선, 카메라 세그멘테이션) → [이 노드] → PX4 브리지.
  행 추종 → 행 끝 → U턴 → 다음 통로 진입 전 과정을 mission.OrchardMission 상태기계로 돌린다.
  이 노드는 ROS 입출력(구독·발행·서비스·파라미터)만 맡고, 판단·제어 계산은 ROS 없는 모듈
  (mission.py, controller.py, headland.py, fusion.py) 에 있어 pytest 로 따로 시험한다.

입력: /orchard/row (RowCenterline), /odom (nav_msgs/Odometry, PX4 EKF 기반),
      /orchard/trees (TreeArray, 행 끝 U턴 계획에 쓰는 줄기 위치)
출력: /cmd_vel (geometry_msgs/Twist) → orchard_px4_bridge 가 PX4 Offboard 로 전달
      /orchard/turn_path (nav_msgs/Path, 계획한 U턴 경로 — RViz)
서비스: /orchard/mission/start, /orchard/mission/stop (std_srvs/Trigger)
상태: /orchard/mission/state (std_msgs/String, '상태|lanes_done=N|이유|row=융합상태')
선택 입력: /orchard/seg/row (RowCenterline, 카메라 중심선 — camera_row.enable 일 때 fusion.fuse_rows 로 융합)

파라미터 (ros2_ws/src/orchard_bringup/config/sim.yaml = 시뮬, robot.yaml = 실차 의 row_navigator_node 항목)
  - mission.* : 통로 수(lanes, 0 = 탐사), 첫 U턴 방향, 열 간격, U턴 방식·속도·반경 등 → MissionParams
  - follow.*  : 순항 속도, 전방 주시거리, 최소 회전반경, 장애물 정지/감속 거리 → controller.FollowParams
  - rate_hz (제어 주기), auto_start (시뮬 자동 출발), row_timeout (인식 끊김 판정 [s]), camera_row.*
  여기서 declare 하지 않은 MissionParams 항목은 mission.py 의 기본값을 고쳐야 바뀐다.
"""
from __future__ import annotations

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from orchard_msgs.msg import RowCenterline, TreeArray
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .controller import FollowParams
from .fusion import fuse_rows
from .mission import IDLE, MissionParams, OrchardMission, Pose2D, RowObs


def yaw_of(q) -> float:
    """쿼터니언 (geometry_msgs/Quaternion) → 요 [rad] (z 축 회전, ZYX 오일러)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RowNavigatorNode(Node):
    """행 순회 자율주행 ROS 노드 (row_navigator_node)."""

    def __init__(self):
        """파라미터 선언 → MissionParams/FollowParams 구성 → 구독·발행·서비스·제어 타이머 생성."""
        super().__init__('row_navigator_node')
        d = self.declare_parameter
        d('rate_hz', 20.0)               # [Hz] 제어 주기 (mission 의 *_confirm_frames 는 이 주기 기준 프레임 수)
        d('auto_start', False)
        d('auto_start_delay', 5.0)      # 시뮬: 행 인식 + odom 수신 후 이 시간 뒤 자동 시작
        d('row_timeout', 0.5)            # [s] /orchard/row 가 이보다 오래 안 오면 인식 무효로 처리
        d('camera_row.enable', True)     # /orchard/seg/row (카메라 세그멘테이션 중심선) 이 있으면 융합
        d('camera_row.weight', 0.5)      # 카메라 중심선 가중치 배율 (fusion.fuse_rows)
        d('camera_row.timeout', 0.6)     # [s] 카메라 중심선이 이보다 오래되면 안 씀
        d('mission.lanes', 3)             # 0 = 탐사 모드 (통로 수를 모르고 끝 열까지 순회)
        d('mission.max_lanes', 40)
        d('mission.first_turn', 'left')
        d('mission.row_spacing', 3.8)
        d('mission.turn_radius', 0.0)
        d('mission.turn_speed', 0.5)
        d('mission.exit_distance', 2.2)
        d('mission.end_trigger_x', 0.3)
        d('mission.min_row_travel', 3.0)
        d('mission.enter_speed', 0.4)
        d('mission.max_enter_distance', 6.0)
        d('mission.turn_mode', 'planned')   # planned: LiDAR 입구 + Dubins 경로 / arc: 반원(예전)
        d('mission.entry_margin', 0.8)
        d('odom_frame', 'odom')           # /orchard/turn_path 의 frame_id
        d('follow.cruise_speed', 0.8)
        d('follow.min_speed', 0.2)
        d('follow.lookahead', 2.5)
        d('follow.max_yaw_rate', 0.8)
        d('follow.min_turn_radius', 0.9)
        d('follow.stop_distance', 1.2)
        d('follow.slow_distance', 3.0)

        g = self.g
        follow = FollowParams(
            cruise_speed=g('follow.cruise_speed'), min_speed=g('follow.min_speed'),
            lookahead=g('follow.lookahead'), max_yaw_rate=g('follow.max_yaw_rate'),
            min_turn_radius=g('follow.min_turn_radius'), stop_distance=g('follow.stop_distance'),
            slow_distance=g('follow.slow_distance'))
        mp = MissionParams(
            lanes=int(g('mission.lanes')), first_turn=str(g('mission.first_turn')),
            row_spacing=g('mission.row_spacing'), turn_radius=g('mission.turn_radius'),
            turn_speed=g('mission.turn_speed'), exit_distance=g('mission.exit_distance'),
            end_trigger_x=g('mission.end_trigger_x'), min_row_travel=g('mission.min_row_travel'),
            enter_speed=g('mission.enter_speed'), max_enter_distance=g('mission.max_enter_distance'),
            max_lanes=int(g('mission.max_lanes')), turn_mode=str(g('mission.turn_mode')),
            entry_margin=g('mission.entry_margin'), follow=follow)
        self.mission = OrchardMission(mp)
        if self.mission.radius < follow.min_turn_radius:   # radius 는 OrchardMission 의 속성
            self.get_logger().warn(
                f'U턴 반경 {self.mission.radius:.2f} m 가 최소 회전반경 {follow.min_turn_radius:.2f} m 보다 작습니다. '
                '행 간격을 넓히거나 K-턴 로직이 필요합니다.')

        self.row = RowObs()
        self.row_time = None
        self.pose = None
        self.ready_since = None
        self.last_state = None

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_state = self.create_publisher(String, '/orchard/mission/state', 10)
        self.create_subscription(RowCenterline, '/orchard/row', self.on_row, 10)
        self.cam_row, self.cam_time, self.fuse_state = None, None, ''
        if self.g('camera_row.enable'):
            self.create_subscription(RowCenterline, '/orchard/seg/row', self.on_cam_row, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(TreeArray, '/orchard/trees', self.on_trees, 10)
        self.pub_path = self.create_publisher(Path, '/orchard/turn_path', 1)
        self.last_plan = None
        self.create_service(Trigger, '/orchard/mission/start', self.srv_start)
        self.create_service(Trigger, '/orchard/mission/stop', self.srv_stop)
        self.create_timer(1.0 / g('rate_hz'), self.tick)
        self.get_logger().info('행 순회 노드 준비. 시작: ros2 service call /orchard/mission/start std_srvs/srv/Trigger')

    def g(self, name):
        """파라미터 값 읽기 (짧게 쓰려는 도우미)."""
        return self.get_parameter(name).value

    def now(self) -> float:
        """현재 ROS 시각 [s] (use_sim_time 이면 시뮬 시각)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def on_row(self, msg: RowCenterline):
        """LiDAR 행 인식 (/orchard/row) 수신 → RowObs 로 저장하고 수신 시각 기록."""
        self.row = RowObs(msg.valid, msg.lateral_offset, msg.heading_error, msg.confidence,
                          msg.last_tree_ahead, msg.obstacle_distance, int(msg.left_count), int(msg.right_count))
        self.row_time = self.now()

    def on_odom(self, msg: Odometry):
        """odom 수신 → 2D 자세 (x, y [m], yaw [rad]) 저장."""
        p = msg.pose.pose
        self.pose = Pose2D(p.position.x, p.position.y, yaw_of(p.orientation))

    def on_trees(self, msg: TreeArray):
        """줄기 검출 (/orchard/trees, base_link) → 미션에 넘겨 U턴 계획용으로 odom 좌표에 모음."""
        if self.pose is None or not msg.trees:
            return
        self.mission.add_trees(self.now(), self.pose, [(t.position.x, t.position.y) for t in msg.trees])

    def publish_plan(self):
        """새 U턴 계획이 생겼으면 경로를 /orchard/turn_path (nav_msgs/Path) 로 한 번 발행하고 로그."""
        plan = self.mission.plan
        if plan is None or plan is self.last_plan:
            return
        self.last_plan = plan
        msg = Path()
        msg.header.frame_id = self.g('odom_frame')
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw in plan['path'][::4]:        # 4 점마다 하나 (0.05 m × 4 = 0.2 m 간격)
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            # 요만 있는 쿼터니언: (0, 0, sin(yaw/2), cos(yaw/2))
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            msg.poses.append(ps)
        self.pub_path.publish(msg)
        nl = plan['next_lane']
        self.get_logger().info(f'U턴 계획: 다음 통로 {nl.source} (줄기 관측 가까운 열 {nl.near_count}, 너머 열 {nl.far_count}회), '
                               f'간격 {nl.spacing:.2f} m, 반경 {plan["radius"]:.2f} m, 경로 {plan["length"]:.1f} m')

    def on_cam_row(self, msg: RowCenterline):
        """카메라 중심선 (/orchard/seg/row) 수신 → offset/heading/신뢰도만 저장 (행 끝 정보는 LiDAR 만 씀)."""
        self.cam_row = RowObs(msg.valid, msg.lateral_offset, msg.heading_error, msg.confidence)
        self.cam_time = self.now()

    def srv_start(self, _req, resp):
        """/orchard/mission/start 서비스: odom 이 있으면 미션 시작 (현재 자세에서 FOLLOW_ROW)."""
        if self.pose is None:
            resp.success, resp.message = False, 'odom 미수신 (PX4 브리지 확인)'
            return resp
        self.mission.start(self.now(), self.pose)
        resp.success, resp.message = True, '미션 시작'
        return resp

    def srv_stop(self, _req, resp):
        """/orchard/mission/stop 서비스: 즉시 STOPPED (속도 0)."""
        self.mission.stop()
        resp.success, resp.message = True, '정지'
        return resp

    def tick(self):
        """제어 주기 (rate_hz): 인식 신선도 확인 → 카메라 융합 → (자동 출발) → mission.step → /cmd_vel·상태 발행."""
        now = self.now()
        row = self.row
        if self.row_time is None or now - self.row_time > self.g('row_timeout'):
            row = RowObs()      # 인식 데이터가 끊기면 invalid 로 처리
        cam = self.cam_row if (self.cam_time is not None
                               and now - self.cam_time < self.g('camera_row.timeout')) else None
        row, self.fuse_state = fuse_rows(row, cam, self.g('camera_row.weight'))

        # 자동 출발 (시뮬용): odom 과 유효한 행 인식이 auto_start_delay 동안 계속되면 start()
        if self.g('auto_start') and self.mission.state == IDLE:
            if self.pose is not None and row.valid:
                self.ready_since = self.ready_since or now
                if now - self.ready_since > self.g('auto_start_delay'):
                    self.mission.start(now, self.pose)
            else:
                self.ready_since = None
                why = 'odom(/odom) 미수신' if self.pose is None else (
                    '행 인식 없음' if self.row_time is None else
                    f'행 인식 끊김/무효 (마지막 수신 {now - self.row_time:.2f}s 전, valid={self.row.valid})')
                self.get_logger().info(f'자동 출발 대기: {why}', throttle_duration_sec=10.0)

        v, w = (0.0, 0.0) if self.pose is None else self.mission.step(now, self.pose, row)
        self.publish_plan()
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.pub_cmd.publish(cmd)

        state = (f'{self.mission.state}|lanes_done={self.mission.lanes_done}|{self.mission.reason}'
                 f'|row={self.fuse_state}')
        self.pub_state.publish(String(data=state))
        if self.mission.state != self.last_state:
            self.get_logger().info(f'상태 → {self.mission.state} ({self.mission.reason})')
            self.last_state = self.mission.state


def main(args=None):
    """노드 실행 진입점 (ros2 run orchard_navigation row_navigator_node)."""
    rclpy.init(args=args)
    node = RowNavigatorNode()
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
