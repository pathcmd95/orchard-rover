"""robot_state_publisher 로 base_link 아래 센서 TF 를 발행한다.

역할
  urdf/orchard_rover.urdf.xacro 를 xacro 로 URDF 문자열로 변환해 robot_state_publisher 에 넘긴다.
  그러면 base_link → livox_frame / camera_link / gps_link 고정 TF 가 /tf_static 으로 발행된다.
  센서 장착 위치(치수)는 config/rover.yaml 에서 읽으므로, 위치를 바꾸려면 그 YAML 만 고치면 된다.
  (odom → base_link TF 는 이 파일이 아니라 px4_state_node 가 발행한다.)

언제 쓰나
  보통 sim.launch.py / robot.launch.py 가 include 한다. TF 만 확인하고 싶을 때 단독 실행:
    ros2 launch orchard_description description.launch.py sim:=true
    ros2 run tf2_tools view_frames      # TF 트리 PDF 생성

launch 인자
  sim           'true' 면 카메라 광학 프레임(camera_color_optical_frame) TF 도 발행 (시뮬 전용).
                실차에서는 realsense2_camera 드라이버가 직접 발행하므로 'false'.
  use_sim_time  'true' 면 Gazebo /clock 시간 사용 (시뮬).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """xacro → robot_description 파라미터를 만들고 robot_state_publisher 노드를 띄운다."""
    xacro_file = os.path.join(get_package_share_directory('orchard_description'), 'urdf',
                              'orchard_rover.urdf.xacro')
    sim = LaunchConfiguration('sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    # 실행 시점에 'xacro <파일> sim:=<true|false>' 명령을 돌려 나온 URDF 문자열을 파라미터로 쓴다.
    # value_type=str: URDF 문자열을 YAML 로 해석하려 하지 않도록 문자열로 고정
    robot_description = ParameterValue(Command(['xacro ', xacro_file, ' sim:=', sim]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]),
    ])
