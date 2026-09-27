"""카메라 세그멘테이션 → 주행가능 영역 → 통로 중심선 노드.

입력: /camera/color/image_raw + camera_info
      (sim 에서 use_gt_labels:=true 면 모델 대신 Gazebo 정답 라벨 /camera/segmentation/labels_map 사용)
출력: /orchard/seg/row      (orchard_msgs/RowCenterline, LiDAR 중심선과 같은 규약 → 주행 노드가 융합)
      /orchard/seg/labels   (mono8 라벨 영상)
      /orchard/debug/seg    (라벨 색을 입힌 영상 + 중심선, RViz 확인용)
모델: tools/training/train_segmentation.py 로 학습한 ONNX (models/orchard_seg.onnx)

메시지 형식
  입력: image_topic sensor_msgs/Image, camera_info_topic sensor_msgs/CameraInfo (K, frame_id)
  /orchard/seg/row 는 lateral_offset·heading_error·row_width·confidence 만 채움
  (left/right_offset = NaN, last_tree_ahead = -inf, obstacle_distance = inf: 카메라로는 알 수 없는 값)
필요 TF: 카메라 광학 frame (CameraInfo.header.frame_id) → base_link
처리 흐름: 라벨 영상(모델 또는 정답) → DRIVABLE 마스크 → seg_path.ground_points(지면 역투영)
  → seg_path.centerline(중심선 적합) → 발행
주요 파라미터 (config/sim.yaml · robot.yaml 의 seg_path_node)
  model_path  ONNX 경로 (없으면 비활성), use_cuda  GPU 추론, ground_z  base_link 기준 지면 높이 [m] ★실측
  min_period  처리 최소 간격 [s] (Orin 부하 조절), stride / x_min / x_max  seg_path.py 참고
"""
from __future__ import annotations

import math
import os
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from orchard_msgs.msg import RowCenterline
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from .geometry import make_transform
from .seg_classes import DRIVABLE, NUM_CLASSES, PALETTE_BGR
from .seg_path import centerline, ground_points


class SegPathNode(Node):
    """카메라 세그멘테이션 라벨에서 주행가능 영역의 중심선을 구해 RowCenterline 으로 발행하는 노드."""

    def __init__(self):
        """파라미터 선언, 발행자·구독자 생성. use_gt_labels 이면 정답 라벨, 아니면 ONNX 모델을 구독·로드."""
        super().__init__('seg_path_node')
        p = self.declare_parameter
        p('image_topic', '/camera/color/image_raw')
        p('camera_info_topic', '/camera/color/camera_info')
        p('model_path', '')
        p('use_gt_labels', False)                 # 시뮬: Gazebo 정답 라벨로 기하 파이프라인만 시험
        p('gt_topic', '/camera/segmentation/labels_map')
        p('use_cuda', False)
        p('base_frame', 'base_link')
        p('ground_z', -0.1)                       # base_link 기준 지면 높이 (sim.yaml ground.nominal_z 와 같게)
        p('stride', 3)                            # 역투영 픽셀 간격 (라벨 영상 기준, 클수록 빠름)
        p('x_min', 1.0)                           # [m] 중심선 적합 전방 거리 범위
        p('x_max', 8.0)
        p('min_period', 0.2)                      # 처리 최소 간격 [s, 벽시계] — Orin 부하 조절
        p('publish_debug', True)                  # /orchard/debug/seg 디버그 영상 발행
        self.bridge = CvBridge()
        self.k = None
        self.cam_frame = None
        self.t_base_cam = None           # 카메라 광학 → base_link 4x4 (정적 TF, 한 번 찾으면 캐시)
        self.model = None
        self.last = 0.0                  # 마지막 처리 시각 (time.monotonic, 벽시계)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub_row = self.create_publisher(RowCenterline, '/orchard/seg/row', 10)
        self.pub_lab = self.create_publisher(Image, '/orchard/seg/labels', 2)
        self.pub_dbg = self.create_publisher(Image, '/orchard/debug/seg', 2)
        self.create_subscription(CameraInfo, self.g('camera_info_topic'), self.on_info, qos_profile_sensor_data)
        path = self.g('model_path')
        if self.g('use_gt_labels'):
            self.create_subscription(Image, self.g('gt_topic'), self.on_gt, qos_profile_sensor_data)
            self.get_logger().info(f"정답 라벨 사용: {self.g('gt_topic')}")
        elif path and os.path.isfile(path):
            from .seg_model import SegModel      # 지연 import: 정답 라벨 모드에서는 onnxruntime 불필요
            self.model = SegModel(path, use_cuda=self.g('use_cuda'))
            self.create_subscription(Image, self.g('image_topic'), self.on_image, qos_profile_sensor_data)
            self.get_logger().info(f'세그멘테이션 모델: {path} ({self.model.backend}, 입력 {self.model.size})')
        else:
            self.get_logger().warn('model_path 가 비어 있거나 파일이 없습니다 → 카메라 중심선 비활성 '
                                   '(LiDAR 중심선만 사용). 학습: tools/training/train_segmentation.py')

    def g(self, name):
        """파라미터 값 읽기 단축 함수."""
        return self.get_parameter(name).value

    def on_info(self, msg: CameraInfo):
        """CameraInfo 콜백: 내부행렬 K(3x3), 카메라 frame_id, 원본 영상 크기 (너비, 높이) [px] 저장."""
        self.k = np.array(msg.k, float).reshape(3, 3)
        self.cam_frame = msg.header.frame_id
        self.img_size = (msg.width, msg.height)

    def transform(self):
        """카메라 광학 → base_link 4x4 (T_base_cam). 아직 TF 가 없으면 None.

        lookup_transform(target=base_link, source=카메라) 이므로 카메라 좌표 점을 base_link 로 바꾸는 행렬.
        """
        if self.t_base_cam is None and self.cam_frame:
            try:
                tf = self.tf_buffer.lookup_transform(self.g('base_frame'), self.cam_frame, Time(),
                                                     Duration(seconds=0.2))
            except TransformException as exc:
                self.get_logger().warn(f'카메라 TF 없음: {exc}', throttle_duration_sec=5.0)
                return None
            t, q = tf.transform.translation, tf.transform.rotation
            self.t_base_cam = make_transform((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        return self.t_base_cam

    def ready(self):
        """처리 가능 여부: CameraInfo 수신 + min_period 경과 + 카메라 TF 확보.

        주의: min_period 검사를 통과하면 TF 가 없어도 self.last 를 갱신한다 → TF 조회 시도도 min_period 마다 한 번.
        """
        now = time.monotonic()
        if self.k is None or now - self.last < self.g('min_period'):
            return False
        self.last = now
        return self.transform() is not None

    def on_image(self, msg: Image):
        """컬러 영상 콜백 (모델 모드): ONNX 추론으로 라벨 영상을 만들어 process 로 넘김."""
        if not self.ready():
            return
        bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        labels = self.model.predict(bgr)
        self.process(labels, msg.header, bgr)

    def on_gt(self, msg: Image):
        """정답 라벨 콜백 (시뮬, use_gt_labels:=true): 모델 없이 기하 파이프라인만 시험할 때 사용."""
        if not self.ready():
            return
        img = self.bridge.imgmsg_to_cv2(msg)
        labels = img[..., 0] if img.ndim == 3 else img     # semantic: 채널마다 같은 라벨 값
        self.process(labels.astype(np.uint8), msg.header, None)

    def process(self, labels: np.ndarray, header, bgr):
        """라벨 영상 → 중심선 추정 → /orchard/seg/row, /orchard/seg/labels (+ 디버그 영상) 발행.

        labels: H×W uint8 클래스 번호 (모델 입력 크기일 수 있음), header: 원본 영상 header,
        bgr: 원본 컬러 영상 (정답 라벨 모드에서는 None).
        """
        h, w = labels.shape
        # 라벨 영상은 모델 입력 크기(예: 320×240)라 원본 CameraInfo 크기와 다를 수 있음
        # → 내부행렬을 같은 비율로 축소 (1행: fx, cx 에 sx / 2행: fy, cy 에 sy)
        sx, sy = w / self.img_size[0], h / self.img_size[1]
        k = self.k.copy()
        k[0] *= sx
        k[1] *= sy
        pts, border = ground_points(labels == DRIVABLE, k, self.t_base_cam, self.g('ground_z'),
                                    stride=int(self.g('stride')))
        est = centerline(pts, border, self.g('x_min'), self.g('x_max'))
        m = RowCenterline()
        m.header = header
        m.header.frame_id = self.g('base_frame')
        m.valid = bool(est.valid)
        m.lateral_offset = float(est.offset)
        m.heading_error = float(est.heading)
        m.row_width = float(est.width)
        m.confidence = float(est.confidence)
        # 카메라 중심선은 열 위치·행 끝·장애물 정보를 주지 않으므로 '모름' 값을 채움
        m.left_offset = m.right_offset = float('nan')
        m.last_tree_ahead = float('-inf')
        m.obstacle_distance = float('inf')
        self.pub_row.publish(m)
        lab_msg = self.bridge.cv2_to_imgmsg(labels, 'mono8')
        lab_msg.header = header
        self.pub_lab.publish(lab_msg)
        if self.g('publish_debug'):
            self.publish_debug(labels, bgr, header, est, k)

    def publish_debug(self, labels, bgr, header, est, k):
        """라벨 색 영상(+원본 영상 반투명 합성) 위에 추정 중심선을 그려 /orchard/debug/seg 로 발행.

        k: 라벨 영상 크기에 맞춘 내부행렬.
        """
        import cv2
        # 256 색 조회표: 정의되지 않은 라벨 값(7~255)은 자홍색 → 잘못된 라벨이 눈에 띄고 인덱스 오류도 없음
        pal = np.array(PALETTE_BGR + [(255, 0, 255)] * max(0, 256 - NUM_CLASSES), np.uint8)
        color = pal[labels]                  # H×W 라벨 → H×W×3 BGR
        if bgr is not None:
            # 원본 55% + 라벨색 45% 반투명 합성
            color = cv2.addWeighted(cv2.resize(bgr, (labels.shape[1], labels.shape[0])), 0.55, color, 0.45, 0)
        if est.valid:           # 중심선을 지면에서 영상으로 다시 투영해 그림
            t_cam_base = np.linalg.inv(self.t_base_cam)
            xs = np.linspace(self.g('x_min'), self.g('x_max'), 15)          # 중심선 위 15 점
            ys = est.offset + math.tan(est.heading) * xs
            P = np.c_[xs, ys, np.full_like(xs, self.g('ground_z')), np.ones_like(xs)]   # 동차좌표 Nx4
            C = (t_cam_base @ P.T).T[:, :3]      # base_link → 카메라 광학좌표
            ok = C[:, 2] > 0.1                   # 카메라 앞 10 cm 이상인 점만 투영 (0 나누기 방지)
            # 핀홀 투영 u = fx·X/Z + cx, v = fy·Y/Z + cy
            uv = np.c_[k[0, 0] * C[ok, 0] / C[ok, 2] + k[0, 2], k[1, 1] * C[ok, 1] / C[ok, 2] + k[1, 2]]
            cv2.polylines(color, [uv.astype(np.int32)], False, (0, 230, 255), 2)
        msg = self.bridge.cv2_to_imgmsg(color, 'bgr8')
        msg.header = header
        self.pub_dbg.publish(msg)


def main(args=None):
    """노드 실행 진입점 (ros2 run orchard_perception seg_path_node)."""
    rclpy.init(args=args)
    node = SegPathNode()
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
