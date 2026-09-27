"""시뮬 세그멘테이션 학습 데이터 자동 수집: 카메라 영상 + Gazebo 정답 라벨을 짝지어 저장.

sim.launch.py record_seg:=true 로 켜진다 (세그멘테이션 카메라도 함께 켜짐).
저장: <dataset_dir>/images/000123.jpg, labels/000123.png (픽셀값 = 클래스 번호, seg_classes.py)
학습: python3 tools/training/train_segmentation.py --data <dataset_dir>

입력 (ROS topic)
  image_topic (기본 /camera/color/image_raw)             sensor_msgs/Image  컬러 영상
  label_topic (기본 /camera/segmentation/labels_map)     sensor_msgs/Image  Gazebo 세그멘테이션 카메라의 정답 라벨
출력: ROS topic 없음, 파일만 저장 (+ <dataset_dir>/classes.json 에 클래스 이름 목록)
주요 파라미터 (config/sim.yaml 의 seg_dataset_node)
  dataset_dir  저장 위치, period  저장 간격 [s, 시뮬 시간], max_images  최대 저장 장수
이미 저장된 영상이 있으면 그 다음 번호부터 이어서 저장한다 (여러 번 실행해 데이터 누적 가능).
"""
from __future__ import annotations

import json
import os

import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image

from .seg_classes import CLASSES


class SegDatasetNode(Node):
    """컬러 영상과 정답 라벨 영상을 시각 동기화해 일정 간격으로 파일로 저장하는 노드."""

    def __init__(self):
        """파라미터 선언, 저장 폴더·classes.json 생성, 두 토픽의 근사 시간 동기화 구독 설정."""
        super().__init__('seg_dataset_node')
        p = self.declare_parameter
        p('image_topic', '/camera/color/image_raw')
        p('label_topic', '/camera/segmentation/labels_map')
        p('dataset_dir', os.path.expanduser('~/seg_dataset_sim'))
        p('period', 1.0)                 # 저장 간격 [s, 시뮬 시간]
        p('max_images', 5000)            # 이 장수에 도달하면 저장 중지 (디스크 보호)
        self.bridge = CvBridge()
        self.dir = self.get_parameter('dataset_dir').value
        for sub in ('images', 'labels'):
            os.makedirs(os.path.join(self.dir, sub), exist_ok=True)
        with open(os.path.join(self.dir, 'classes.json'), 'w', encoding='utf-8') as f:
            json.dump(CLASSES, f)
        self.count = len(os.listdir(os.path.join(self.dir, 'images')))     # 기존 파일 수 → 다음 파일 번호
        self.last_t = -1e9                                                  # 첫 프레임은 무조건 저장되도록
        subs = [Subscriber(self, Image, self.get_parameter('image_topic').value, qos_profile=qos_profile_sensor_data),
                Subscriber(self, Image, self.get_parameter('label_topic').value, qos_profile=qos_profile_sensor_data)]
        # 두 영상의 stamp 차이가 slop 0.05 s 이내면 한 쌍으로 묶음 (같은 시뮬 프레임의 컬러·라벨)
        self.sync = ApproximateTimeSynchronizer(subs, queue_size=10, slop=0.05)
        self.sync.registerCallback(self.on_pair)
        self.get_logger().info(f'세그멘테이션 데이터 저장: {self.dir} (이미 {self.count}장)')

    def on_pair(self, img: Image, lab: Image):
        """동기화된 (컬러, 라벨) 한 쌍 콜백: period 간격·max_images 제한을 지키며 jpg + png 로 저장."""
        import cv2
        t = Time.from_msg(img.header.stamp).nanoseconds * 1e-9
        if t - self.last_t < self.get_parameter('period').value or \
                self.count >= self.get_parameter('max_images').value:
            return
        self.last_t = t
        bgr = self.bridge.imgmsg_to_cv2(img, 'bgr8')
        labels = self.bridge.imgmsg_to_cv2(lab)
        labels = labels[..., 0] if labels.ndim == 3 else labels         # 다채널이면 첫 채널 (채널마다 같은 라벨 값)
        if labels.shape[:2] != bgr.shape[:2]:
            # 라벨은 최근접 보간만 사용 (선형 보간하면 클래스 번호 사이의 엉뚱한 값이 생김)
            labels = cv2.resize(labels, (bgr.shape[1], bgr.shape[0]), interpolation=cv2.INTER_NEAREST)
        name = f'{self.count:06d}'                  # 000123 형식, 영상·라벨이 같은 이름
        # 영상은 JPEG 품질 92 (용량·화질 절충), 라벨은 무손실 PNG 필수 (JPEG 은 픽셀값이 변해 라벨이 깨짐)
        cv2.imwrite(os.path.join(self.dir, 'images', name + '.jpg'), bgr, [cv2.IMWRITE_JPEG_QUALITY, 92])
        cv2.imwrite(os.path.join(self.dir, 'labels', name + '.png'), labels.astype(np.uint8))
        self.count += 1
        if self.count % 50 == 0:
            self.get_logger().info(f'{self.count}장 저장')


def main(args=None):
    """노드 실행 진입점 (ros2 run orchard_perception seg_dataset_node)."""
    rclpy.init(args=args)
    node = SegDatasetNode()
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
