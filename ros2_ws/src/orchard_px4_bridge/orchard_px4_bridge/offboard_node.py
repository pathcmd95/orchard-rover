"""/cmd_vel → PX4 로버 Offboard 제어 브리지.

PX4 v1.17 Ackermann 로버의 Offboard(velocity) 는 TrajectorySetpoint.velocity(NED)를 받아
  속도 = |v_NED|, 목표 요 = atan2(v_E, v_N)
로 바꾼 뒤 자체 속도·요 제어기로 돈다 (src/modules/rover_ackermann/.../AckermannOffboardMode.cpp).
RoverSpeedSetpoint 를 직접 보내면 PX4 가 같은 토픽을 매 주기 덮어써서 속도가 0 이 된다 (Gazebo 에서 확인).
그래서 cmd_vel(v, ω) → 목표 요 = 현재 요 + (−ω)·horizon, v_NED = v·(cos, sin) 로 보낸다 (frames.cmd_vel_to_ned_velocity).
horizon = 1 / RO_YAW_P (에어프레임 51010 에서 3) 이면 요청한 요레이트가 그대로 나온다.

토픽 (노드 이름 px4_offboard_bridge)
  구독: /cmd_vel (geometry_msgs/Twist, FLU: linear.x [m/s] 전진, angular.z [rad/s] + = 좌회전)
        /fmu/out/vehicle_odometry (px4_msgs/VehicleOdometry: 현재 NED 요를 얻는 데만 사용)
        /fmu/out/vehicle_status_v1 (px4_msgs/VehicleStatus: 시동·비행모드 확인, 파라미터로 토픽 변경)
  발행: /fmu/in/offboard_control_mode (OffboardControlMode: velocity 제어만 켬, 20 Hz 하트비트)
        /fmu/in/trajectory_setpoint (TrajectorySetpoint: velocity(NED) 만 채우고 나머지는 NaN = 사용 안 함)
        /fmu/in/vehicle_command (VehicleCommand: 시동/해제, Offboard 전환)
  서비스: /orchard/px4/arm, /orchard/px4/disarm, /orchard/px4/offboard (std_srvs/Trigger, 수동 명령용)

학생이 바꿀 만한 파라미터 (orchard_bringup/config/sim.yaml, robot.yaml 의 px4_offboard_bridge 절)
  rate_hz, cmd_timeout [s], max_speed [m/s], max_yaw_rate [rad/s], auto_offboard/auto_arm (시뮬 true, 실차 false),
  yaw_horizon [s] (= 1 / RO_YAW_P, PX4 에어프레임 px4/airframes/51010_gz_orchard_rover 의 RO_YAW_P 와 짝).

참고
  - PX4 User Guide "Offboard Mode" (setpoint 스트림 > 2 Hz 유지, 전환 전 스트림 필요)
  - PX4 User Guide "ROS 2 User Guide" (QoS, NED/FRD ↔ ENU/FLU frame conventions)
  - REP-103 (ROS 좌표·단위 규약)
"""
from __future__ import annotations

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry, VehicleStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from .frames import cmd_vel_to_ned_velocity, yaw_from_quat_wxyz

# PX4 uXRCE-DDS 토픽 QoS: PX4 가 BEST_EFFORT 로 내보내므로 구독 쪽도 맞춰야 메시지가 들어온다
# (PX4 ROS 2 User Guide 의 권장 QoS). depth 1 = 최신 값만 필요.
PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)

PX4_MODE_OFFBOARD = 6.0   # PX4_CUSTOM_MAIN_MODE_OFFBOARD (px4_custom_mode.h 의 main mode 번호)


class OffboardBridge(Node):
    """/cmd_vel 을 PX4 Offboard velocity setpoint 로 바꿔 rate_hz(기본 20 Hz)로 흘려보내고, 필요하면 시동·Offboard 전환까지 한다."""

    def __init__(self):
        """파라미터 선언, PX4/ROS 토픽·서비스 연결, 주기 타이머(tick) 등록."""
        super().__init__('px4_offboard_bridge')
        d = self.declare_parameter
        d('rate_hz', 20.0)                 # PX4 는 Offboard 유지에 2 Hz 이상 필요
        d('cmd_timeout', 0.5)              # cmd_vel 이 끊기면 정지 명령
        d('max_speed', 1.5)                # [m/s] cmd_vel 전진 속도 제한
        d('max_yaw_rate', 1.0)             # [rad/s] cmd_vel 요레이트 제한
        d('auto_offboard', False)          # 시뮬: True / 실차: False (조종기로 전환)
        d('auto_arm', False)               # 시뮬: True / 실차: False
        d('vehicle_status_topic', '/fmu/out/vehicle_status_v1')   # v1.16+ 메시지 버전 접미사
        d('target_system', 1)              # MAVLink system id (PX4 SITL 인스턴스 0 → 1)
        d('yaw_horizon', 1.0 / 3.0)        # = 1 / RO_YAW_P (에어프레임 51010: RO_YAW_P 3)

        self.pub_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self.pub_traj = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_QOS)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.on_odom, PX4_QOS)
        self.yaw_ned: float | None = None      # 현재 NED 요 [rad]; odometry 를 받기 전에는 None
        self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)
        self.create_subscription(VehicleStatus, self.g('vehicle_status_topic'), self.on_status, PX4_QOS)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_service(Trigger, '/orchard/px4/arm', lambda q, r: self.srv(r, self.arm, '시동 명령'))
        self.create_service(Trigger, '/orchard/px4/disarm', lambda q, r: self.srv(r, self.disarm, '시동 해제 명령'))
        self.create_service(Trigger, '/orchard/px4/offboard',
                            lambda q, r: self.srv(r, self.set_offboard, 'Offboard 전환 명령'))

        self.cmd = Twist()                     # 마지막으로 받은 /cmd_vel
        self.cmd_time = None                   # 그 수신 시각 [s] (cmd_timeout 판정)
        self.status: VehicleStatus | None = None
        self.stream_count = 0                  # 지금까지 보낸 setpoint 수 (Offboard 전환 전 예열 확인)
        self.last_request = -10.0              # 마지막 시동/모드 명령 시각 [s] (1초에 한 번만 보내도록)
        self.create_timer(1.0 / self.g('rate_hz'), self.tick)
        self.get_logger().info(
            f"auto_offboard={self.g('auto_offboard')} auto_arm={self.g('auto_arm')} "
            f"status_topic={self.g('vehicle_status_topic')}")

    def g(self, name):
        """파라미터 값 읽기 (짧게 쓰려는 도우미)."""
        return self.get_parameter(name).value

    def now_s(self) -> float:
        """노드 시계(use_sim_time 이면 시뮬 시간) 현재 시각 [s]."""
        return self.get_clock().now().nanoseconds * 1e-9

    def now_us(self) -> int:
        """현재 시각 [µs]. PX4 메시지의 timestamp 단위가 마이크로초."""
        return int(self.get_clock().now().nanoseconds / 1000)

    def srv(self, resp, fn, text):
        """Trigger 서비스 공통 처리: fn() 으로 명령을 보내고 성공 응답을 채운다 (PX4 수락 여부는 확인하지 않음)."""
        fn()
        resp.success, resp.message = True, text
        return resp

    def on_status(self, msg: VehicleStatus):
        """PX4 상태 저장. 시동/비행모드가 바뀔 때만 로그를 남긴다."""
        prev = self.status
        self.status = msg
        if prev is None or prev.nav_state != msg.nav_state or prev.arming_state != msg.arming_state:
            armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
            offb = msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            self.get_logger().info(f'PX4 상태: armed={armed} offboard={offb} (nav_state={msg.nav_state})')

    def on_odom(self, msg: VehicleOdometry):
        """PX4 자세 쿼터니언(FRD→NED, w,x,y,z)에서 현재 NED 요를 뽑아 둔다. NaN 이면 무시."""
        if all(math.isfinite(v) for v in msg.q):
            self.yaw_ned = yaw_from_quat_wxyz(msg.q)

    def on_cmd(self, msg: Twist):
        """/cmd_vel 수신: 값과 수신 시각만 저장 (실제 전송은 tick 에서 일정 주기로)."""
        self.cmd = msg
        self.cmd_time = self.now_s()

    def send_command(self, command: int, p1: float = 0.0, p2: float = 0.0):
        """VehicleCommand(MAVLink 명령과 같은 번호 체계) 발행. p1/p2 = param1/param2."""
        m = VehicleCommand()
        m.timestamp = self.now_us()
        m.command = command
        m.param1 = float(p1)
        m.param2 = float(p2)
        m.target_system = int(self.g('target_system'))
        m.target_component = 1                 # 1 = 오토파일럿 컴포넌트
        m.source_system = 1
        m.source_component = 1
        m.from_external = True                 # 외부(ROS)에서 온 명령임을 표시
        self.pub_cmd.publish(m)

    def arm(self):
        """시동 명령 (COMPONENT_ARM_DISARM, param1 = 1)."""
        self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def disarm(self):
        """시동 해제 명령 (param1 = 0)."""
        self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)

    def set_offboard(self):
        """Offboard 모드 전환 (DO_SET_MODE: param1 = 1 사용자 정의 모드 사용, param2 = main mode 6 Offboard)."""
        self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, PX4_MODE_OFFBOARD)

    def tick(self):
        """주기(rate_hz) 처리: Offboard 하트비트 + velocity setpoint 발행, 필요하면 자동 Offboard 전환·시동.

        cmd_vel 이 cmd_timeout 보다 오래되면 0 속도를 보내 정지한다 (통신 끊김 안전장치).
        """
        ts = self.now_us()
        mode = OffboardControlMode()
        mode.timestamp = ts
        mode.velocity = True                   # 위치·가속도·자세 제어는 끄고 속도 setpoint 만 사용
        self.pub_mode.publish(mode)

        # cmd_vel 이 최근(cmd_timeout 이내)에 왔을 때만 따르고, 속도·요레이트는 max_* 로 자른다
        fresh = self.cmd_time is not None and self.now_s() - self.cmd_time < self.g('cmd_timeout')
        v = self.cmd.linear.x if fresh else 0.0
        w = self.cmd.angular.z if fresh else 0.0
        v = max(-self.g('max_speed'), min(self.g('max_speed'), v))
        w = max(-self.g('max_yaw_rate'), min(self.g('max_yaw_rate'), w))

        tr = TrajectorySetpoint()
        tr.timestamp = ts
        # PX4 는 NaN 인 필드를 '지정 안 함' 으로 본다 → velocity(북, 동) 만 유효, 수직 속도·요는 NaN
        tr.position = [math.nan] * 3
        tr.acceleration = [math.nan] * 3
        tr.jerk = [math.nan] * 3
        tr.yaw = math.nan
        tr.yawspeed = math.nan
        if self.yaw_ned is None:
            tr.velocity = [0.0, 0.0, math.nan]           # 자세를 모르면 정지
        else:
            # (v, ω) → 목표 요 = 현재 요 − ω·horizon 방향의 NED 수평 속도 (frames.py 참고)
            vn, ve = cmd_vel_to_ned_velocity(v, w, self.yaw_ned, self.g('yaw_horizon'))
            tr.velocity = [float(vn), float(ve), math.nan]
        self.pub_traj.publish(tr)

        self.stream_count += 1
        if self.stream_count < 20 or self.status is None:
            return      # Offboard 전환 전 setpoint 를 먼저 1초 정도 흘려보내야 한다 (20개 / 20 Hz = 1 s)
        now = self.now_s()
        if now - self.last_request < 1.0:     # 명령은 1초에 한 번만 (PX4 에 명령이 쌓이지 않게)
            return
        if self.g('auto_offboard') and self.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.set_offboard()
            self.last_request = now
        elif self.g('auto_arm') and self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.arm()
            self.last_request = now


def main(args=None):
    """ros2 run orchard_px4_bridge offboard_node 진입점."""
    rclpy.init(args=args)
    node = OffboardBridge()
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
