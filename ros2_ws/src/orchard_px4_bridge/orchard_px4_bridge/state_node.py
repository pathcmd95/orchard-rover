"""PX4 상태 → 표준 ROS 메시지 변환 노드.

/fmu/out/vehicle_odometry     → /odom (nav_msgs/Odometry, ENU/FLU) + TF odom→base_link
/fmu/out/vehicle_gps_position → /gps/fix (sensor_msgs/NavSatFix)
/fmu/out/sensor_combined      → /imu/data_raw (sensor_msgs/Imu, FLU, 자세 없음)

역할
  PX4 EKF2 가 추정한 위치·자세·속도(NED/FRD)를 ROS 규약(REP-103: ENU/FLU)의 표준 메시지로 바꿔
  주행·인식 노드가 PX4 를 몰라도 되게 한다. 시뮬과 실차에서 같은 코드가 돈다.
  - /odom: pose 는 odom 프레임(ENU, PX4 로컬 원점 기준) [m], 자세 FLU→ENU 쿼터니언,
           twist 는 기체 프레임(FLU) 선속도 [m/s]·각속도 [rad/s] (nav_msgs/Odometry 규약: twist 는 child_frame_id 기준)
  - TF: odom → base_link (REP-105). publish_tf=false 로 끄면 다른 노드(예: robot_localization)가 TF 를 낼 수 있다.
  - /gps/fix: 위도·경도 [deg], 고도(MSL) [m], 공분산은 eph/epv 로 만든 대각 [m²].
  - /imu/data_raw: 자이로 [rad/s], 가속도계 [m/s²] (FLU). 자세는 없음(orientation_covariance[0] = -1).

파라미터 (orchard_bringup/config/sim.yaml, robot.yaml 의 px4_state_node 절)
  odom_frame ('odom'), base_frame ('base_link'), publish_tf (True)

참고
  - REP-103 "Standard Units of Measure and Coordinate Conventions", REP-105 "Coordinate Frames for Mobile Platforms"
  - PX4 User Guide "ROS 2 User Guide" (frame conventions), px4_msgs/VehicleOdometry 메시지 정의
  - sensor_msgs/Imu 메시지 주석 (공분산 -1 = 해당 값 없음)
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import SensorCombined, SensorGps, VehicleOdometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from tf2_ros import TransformBroadcaster

from .frames import attitude_ned_frd_to_enu_flu, frd_to_flu, ned_to_enu

# PX4 uXRCE-DDS 가 내보내는 토픽과 호환되는 QoS (BEST_EFFORT). offboard_node 와 같다.
PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)


class Px4StateNode(Node):
    """PX4 odometry·GPS·IMU 를 ROS 표준 메시지(ENU/FLU)와 TF 로 다시 발행하는 노드."""

    def __init__(self):
        """파라미터(odom_frame, base_frame, publish_tf) 읽기, 발행자·구독자 생성."""
        super().__init__('px4_state_node')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self.pub_odom = self.create_publisher(Odometry, '/odom', 20)
        self.pub_fix = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu/data_raw', 50)
        self.tf_pub = TransformBroadcaster(self)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.on_odom, PX4_QOS)
        self.create_subscription(SensorGps, '/fmu/out/vehicle_gps_position', self.on_gps, PX4_QOS)
        self.create_subscription(SensorCombined, '/fmu/out/sensor_combined', self.on_imu, PX4_QOS)
        self.warned_frame = False              # pose_frame 경고는 한 번만

    def on_odom(self, msg: VehicleOdometry):
        """VehicleOdometry(NED/FRD) → /odom(ENU/FLU) + TF odom→base_link.

        위치: NED → ENU. 자세: FRD→NED 쿼터니언 → FLU→ENU 쿼터니언.
        속도: 기체 FLU 로 표현 (velocity_frame 이 월드(NED)면 R_enu_flu.T 로 기체 좌표로 돌린다).
        값이 NaN(추정 전)이면 발행하지 않는다.
        """
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED and not self.warned_frame:
            self.get_logger().warn(f'pose_frame={msg.pose_frame} (NED 아님). GPS/나침반 상태를 확인하세요.')
            self.warned_frame = True
        pos = np.array(msg.position, dtype=float)
        q = np.array(msg.q, dtype=float)
        if not np.all(np.isfinite(pos)) or not np.all(np.isfinite(q)):
            return
        stamp = self.get_clock().now().to_msg()
        p_enu = ned_to_enu(pos)                                  # (N, E, D) → (E, N, U) [m]
        q_xyzw, r_enu_flu = attitude_ned_frd_to_enu_flu(q)      # r_enu_flu: FLU 벡터 → ENU 벡터

        vel = np.array(msg.velocity, dtype=float)
        if msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_BODY_FRD:
            v_body = frd_to_flu(vel)
        else:   # NED 또는 FRD(월드) → ENU → 기체좌표
            v_body = r_enu_flu.T @ ned_to_enu(vel)   # 회전행렬의 역 = 전치: ENU → FLU
        w_body = frd_to_flu(np.array(msg.angular_velocity, dtype=float))

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = map(float, p_enu)
        (od.pose.pose.orientation.x, od.pose.pose.orientation.y,
         od.pose.pose.orientation.z, od.pose.pose.orientation.w) = q_xyzw
        if np.all(np.isfinite(v_body)):
            od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.linear.z = map(float, v_body)
        if np.all(np.isfinite(w_body)):
            od.twist.twist.angular.x, od.twist.twist.angular.y, od.twist.twist.angular.z = map(float, w_body)
        # 위치 분산(대각)만 옮긴다. 분산은 축 순서만 바뀌면 되므로 NED→ENU 후 부호(−D)를 abs 로 되돌린다.
        # pose.covariance 는 6×6 (x, y, z, 롤, 피치, 요) 행 우선 배열 → 대각 원소 인덱스 = i * 7
        pv = np.array(msg.position_variance, dtype=float)
        if np.all(np.isfinite(pv)):
            pv_enu = np.abs(ned_to_enu(pv))
            for i in range(3):
                od.pose.covariance[i * 7] = float(pv_enu[i])
        self.pub_odom.publish(od)

        if self.publish_tf:     # TF odom → base_link (odometry 와 같은 값, 같은 시각)
            t = TransformStamped()
            t.header = od.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x = od.pose.pose.position.x
            t.transform.translation.y = od.pose.pose.position.y
            t.transform.translation.z = od.pose.pose.position.z
            t.transform.rotation = od.pose.pose.orientation
            self.tf_pub.sendTransform(t)

    def on_gps(self, msg: SensorGps):
        """PX4 GPS(SensorGps) → sensor_msgs/NavSatFix. fix_type 을 NavSatStatus 로, eph/epv [m] 를 분산으로."""
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'gps_link'
        fix.latitude = float(msg.latitude_deg)
        fix.longitude = float(msg.longitude_deg)
        fix.altitude = float(msg.altitude_msl_m)
        if msg.fix_type >= SensorGps.FIX_TYPE_RTK_FLOAT:       # RTK(float/fixed) → 지상 보정(GBAS) fix 로 표시
            fix.status.status = NavSatStatus.STATUS_GBAS_FIX
        elif msg.fix_type >= SensorGps.FIX_TYPE_2D:
            fix.status.status = NavSatStatus.STATUS_FIX
        else:
            fix.status.status = NavSatStatus.STATUS_NO_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        # 공분산 3×3 (ENU 순서 행 우선): 수평 표준편차 eph, 수직 epv [m] 를 제곱해 대각에 [m²]
        fix.position_covariance[0] = float(msg.eph) ** 2
        fix.position_covariance[4] = float(msg.eph) ** 2
        fix.position_covariance[8] = float(msg.epv) ** 2
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub_fix.publish(fix)

    def on_imu(self, msg: SensorCombined):
        """PX4 SensorCombined(FRD 자이로 [rad/s]·가속도 [m/s²]) → sensor_msgs/Imu(FLU). 자세는 채우지 않는다."""
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.base_frame
        imu.orientation_covariance[0] = -1.0   # 자세 정보 없음
        g = frd_to_flu(msg.gyro_rad)
        a = frd_to_flu(msg.accelerometer_m_s2)
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = map(float, g)
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = map(float, a)
        self.pub_imu.publish(imu)


def main(args=None):
    """ros2 run orchard_px4_bridge state_node 진입점."""
    rclpy.init(args=args)
    node = Px4StateNode()
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
