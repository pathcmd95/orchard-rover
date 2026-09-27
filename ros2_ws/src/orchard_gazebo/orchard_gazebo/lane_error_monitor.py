"""시뮬레이션 주행 성능 평가: 정답 위치(/ground_truth/odom)와 통로 중심선 사이 횡방향 오차.

행 안(x 가 0 ~ row_length)에 있을 때만 가장 가까운 통로 중심선과의 거리를 잰다.
출력: /orchard/eval/lateral_error (std_msgs/Float32), 10초마다 RMS/최대값 로그
정량 지표 '통로 중심선 횡오차(centerline lateral error)'를 자동 계산하는 도구.

입력
  파라미터 meta_file: world_gen 이 만든 orchard_meta.json (sim.launch.py 가 자동으로 넘김)
    - lane_centers_y: 통로 중심선 y [m] 목록, row_x_range: 나무 열의 x 범위 [m]
  구독 /ground_truth/odom (nav_msgs/Odometry): Gazebo OdometryPublisher 의 정답 위치
    (Gazebo 월드 좌표 = ENU: 원점 x = 나무 열 시작, y = 통로 0 중심. PX4 추정치가 아닌 시뮬 정답)
출력
  /orchard/eval/lateral_error (std_msgs/Float32) [m], 부호: + = 통로 중심보다 +y(왼쪽, +x 로 달릴 때) 쪽
  종료 시와 10초마다 RMS·최대 절댓값 로그 [cm]
바꿀 곳
  측정 구간(행 양 끝 1 m 제외)은 on_odom 의 1.0 [m], 로그 주기는 create_timer 의 10.0 [s].
"""
from __future__ import annotations

import json
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32


class LaneErrorMonitor(Node):
    """정답 위치에서 가장 가까운 통로 중심선까지의 횡오차를 발행·누적하는 평가 노드."""

    def __init__(self):
        """meta_file 을 읽어 통로 중심선·열 범위를 얻고 구독·타이머를 만든다."""
        super().__init__('lane_error_monitor')
        self.declare_parameter('meta_file', '')
        meta_file = self.get_parameter('meta_file').value
        with open(meta_file, encoding='utf-8') as f:
            meta = json.load(f)
        self.lanes = meta['lane_centers_y']
        self.x0, self.x1 = meta['row_x_range']
        self.errors = []                       # 누적 횡오차 샘플 [m] (RMS/최대 계산용)
        self.pub = self.create_publisher(Float32, '/orchard/eval/lateral_error', 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_odom, 10)
        self.create_timer(10.0, self.report)

    def on_odom(self, msg: Odometry):
        """행 안(양 끝 1 m 제외)에 있을 때만 가장 가까운 통로 중심선과의 y 차이 [m] 를 기록·발행."""
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if not (self.x0 + 1.0 < x < self.x1 - 1.0):   # 행 끝 U턴·진입 구간(1 m) 은 평가에서 뺀다
            return
        err = min((y - c for c in self.lanes), key=abs)   # 절댓값이 가장 작은 (부호 있는) 오차
        self.errors.append(err)
        self.pub.publish(Float32(data=float(err)))

    def report(self):
        """지금까지의 RMS·최대 횡오차를 cm 단위로 로그 (샘플이 없으면 생략)."""
        if not self.errors:
            return
        rms = math.sqrt(sum(e * e for e in self.errors) / len(self.errors))
        mx = max(abs(e) for e in self.errors)
        self.get_logger().info(f'통로 중심선 횡오차: RMS {rms * 100:.1f} cm, 최대 {mx * 100:.1f} cm '
                               f'(샘플 {len(self.errors)})')


def main(args=None):
    """ros2 run orchard_gazebo lane_error_monitor 진입점 (종료 시 마지막 요약을 한 번 더 출력)."""
    rclpy.init(args=args)
    node = LaneErrorMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
