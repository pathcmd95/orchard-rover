"""LiDAR 줄기 + 카메라 검출 융합, 그리고 자동 라벨링(데이터셋 생성) 노드.

1) /orchard/trees(base_link) 의 줄기를 카메라 영상에 투영해 박스를 만든다.
2) YOLO 검출(/orchard/detections)과 겹치면 camera_confirmed=True 로 표시한다.
3) record_dataset=True 이면 영상 + YOLO 라벨(txt)을 저장한다.
   → LiDAR 가 카메라 학습 데이터를 자동으로 만들어 주는 구조 (시뮬/실차 공통)

입력 (ROS topic)
  /orchard/trees        orchard_msgs/TreeArray         lidar_tree_node 의 줄기 (base_link)
  /orchard/detections   vision_msgs/Detection2DArray   camera_tree_node 의 YOLO 검출 (없어도 동작)
  image_topic           sensor_msgs/Image              컬러 영상 (이 콜백이 처리 주기를 결정)
  camera_info_topic     sensor_msgs/CameraInfo         내부행렬 K, 카메라 frame_id
출력 (ROS topic)
  /orchard/trees_fused  orchard_msgs/TreeArray  camera_confirmed·confidence 를 갱신한 줄기
  /orchard/debug/fusion sensor_msgs/Image       투영 박스(초록=매칭, 주황=미매칭)·YOLO 박스(파랑) 겹친 영상
  (record_dataset=true) <dataset_dir>/images/*.jpg + labels/*.txt  YOLO 학습 데이터
필요 TF: base_link → 카메라 광학 frame (CameraInfo.header.frame_id). 카메라-LiDAR 외부 캘리브레이션이 틀리면
  투영 박스가 줄기에서 어긋나므로 /orchard/debug/fusion 으로 먼저 확인할 것.
주요 파라미터 (config/sim.yaml · robot.yaml 의 tree_fusion_node, 기본값은 __init__ 참고)
  record_dataset / dataset_dir / record_every_n  자동 라벨링 on/off, 저장 위치, 저장 간격(영상 N 장마다)
  trunk_label_height  라벨 박스 높이 [m], max_label_distance  라벨 최대 거리 [m]
  max_sync_dt  영상-LiDAR 시간차 허용 [s], iou_threshold  매칭 기준
기하 계산은 projection.py 참고. 학습 절차는 docs/04_perception_training.md.
"""
from __future__ import annotations

import copy
import math
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from cv_bridge import CvBridge
from orchard_msgs.msg import TreeArray
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray

from .geometry import make_transform
from .projection import Box, greedy_match, trunk_box, yolo_label


class TreeFusionNode(Node):
    """LiDAR 줄기를 영상에 투영해 카메라 검출과 융합하고, 선택적으로 YOLO 학습 데이터를 저장하는 노드."""

    def __init__(self):
        """파라미터 선언, TF 리스너·발행자·구독자 생성, (record_dataset 이면) 저장 폴더 생성."""
        super().__init__('tree_fusion_node')
        p = self.declare_parameter
        p('image_topic', '/camera/color/image_raw')
        p('camera_info_topic', '/camera/color/camera_info')
        p('base_frame', 'base_link')
        p('trunk_label_height', 0.8)     # 라벨 박스 높이 = 지면~수관 시작
        p('max_label_distance', 9.0)     # 너무 먼 줄기는 라벨에서 제외
        p('max_sync_dt', 0.25)           # 영상-LiDAR 시간차 허용 [s]
        p('iou_threshold', 0.2)          # 투영 박스-YOLO 박스 매칭 IoU 기준
        p('record_dataset', False)       # true: 영상 + 자동 라벨 저장
        p('dataset_dir', os.path.expanduser('~/orchard_dataset'))
        p('record_every_n', 5)           # 영상 N 장마다 1 장 저장 (연속 프레임은 거의 같아 중복만 늘어남)
        p('publish_overlay', True)       # /orchard/debug/fusion 디버그 영상 발행

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.k = None
        self.cam_frame = None
        self.t_cam_base = None           # base_link → 카메라 광학 4x4 (한 번 찾으면 캐시)
        self.trees: TreeArray | None = None          # 가장 최근 LiDAR 줄기
        self.dets: Detection2DArray | None = None    # 가장 최근 YOLO 검출
        self.frame_count = 0
        self.saved = 0

        self.pub_fused = self.create_publisher(TreeArray, '/orchard/trees_fused', 10)
        self.pub_overlay = self.create_publisher(Image, '/orchard/debug/fusion', 2)
        self.create_subscription(CameraInfo, self.g('camera_info_topic'), self.on_info, qos_profile_sensor_data)
        self.create_subscription(TreeArray, '/orchard/trees', self.on_trees, 10)
        self.create_subscription(Detection2DArray, '/orchard/detections', self.on_dets, 10)
        self.create_subscription(Image, self.g('image_topic'), self.on_image, qos_profile_sensor_data)

        if self.g('record_dataset'):
            for sub in ('images', 'labels'):
                os.makedirs(os.path.join(self.g('dataset_dir'), sub), exist_ok=True)
            self.get_logger().info(f"자동 라벨링 저장 위치: {self.g('dataset_dir')}")

    def g(self, name):
        """파라미터 값 읽기 단축 함수."""
        return self.get_parameter(name).value

    def on_info(self, msg: CameraInfo):
        """CameraInfo 콜백: 내부행렬 K(3x3)와 카메라 광학 frame_id 저장."""
        self.k = np.array(msg.k, dtype=float).reshape(3, 3)
        self.cam_frame = msg.header.frame_id

    def on_trees(self, msg: TreeArray):
        """LiDAR 줄기 콜백: 최신 메시지만 보관 (처리는 영상 콜백에서)."""
        self.trees = msg

    def on_dets(self, msg: Detection2DArray):
        """YOLO 검출 콜백: 최신 메시지만 보관."""
        self.dets = msg

    def camera_transform(self):
        """base_link → 카메라 광학좌표 4x4 (T_cam_base). TF 가 없으면 None.

        lookup_transform(target=카메라, source=base_link) 이므로 base_link 점을 카메라 좌표로 바꾸는 행렬.
        카메라는 차체에 고정(정적 TF)이라 한 번 찾으면 캐시한다.
        """
        if self.t_cam_base is not None:
            return self.t_cam_base
        try:
            tf = self.tf_buffer.lookup_transform(self.cam_frame, self.g('base_frame'), Time(),
                                                 Duration(seconds=0.2))
        except TransformException as exc:
            self.get_logger().warn(f'카메라 TF 없음: {exc}', throttle_duration_sec=2.0)
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        self.t_cam_base = make_transform((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        return self.t_cam_base

    @staticmethod
    def stamp_sec(stamp):
        """builtin_interfaces/Time → 초 [s] (float)."""
        return Time.from_msg(stamp).nanoseconds * 1e-9

    def on_image(self, msg: Image):
        """영상 콜백: 줄기 투영 → YOLO 매칭 → /orchard/trees_fused 발행 → (옵션) 데이터 저장·디버그 영상."""
        if self.k is None or self.trees is None:
            return
        t_cam_base = self.camera_transform()
        if t_cam_base is None:
            return
        img_t = self.stamp_sec(msg.header.stamp)
        # 로봇이 움직이는 중이므로 LiDAR 줄기와 영상 시각이 max_sync_dt 이상 차이 나면 투영이 어긋남 → 건너뜀
        if abs(img_t - self.stamp_sec(self.trees.header.stamp)) > self.g('max_sync_dt'):
            return
        size = (msg.width, msg.height)                    # (너비, 높이) [px]

        # 1) LiDAR 줄기 → 영상 박스 (proj_idx: 박스가 몇 번째 줄기에서 왔는지)

        proj, proj_idx = [], []
        for i, tr in enumerate(self.trees.trees):
            dist = math.hypot(tr.position.x, tr.position.y)
            if dist > self.g('max_label_distance'):
                continue
            # 반경 최소 5 cm: LiDAR 가 한쪽 면만 봐서 반경이 작게 나와도 박스가 너무 가늘지 않게
            box = trunk_box((tr.position.x, tr.position.y), tr.position.z, max(tr.radius, 0.05),
                            self.g('trunk_label_height'), t_cam_base, self.k, size)
            if box is not None:
                proj.append(box)
                proj_idx.append(i)

        # 2) 시간이 맞는 YOLO 검출을 Box(x1, y1, x2, y2) 로 변환 (vision_msgs 는 중심 + 크기)
        det_boxes = []
        if self.dets is not None and abs(img_t - self.stamp_sec(self.dets.header.stamp)) < self.g('max_sync_dt'):
            for d in self.dets.detections:
                cx, cy = d.bbox.center.position.x, d.bbox.center.position.y
                score = d.results[0].hypothesis.score if d.results else 1.0
                det_boxes.append(Box(cx - d.bbox.size_x / 2, cy - d.bbox.size_y / 2,
                                     cx + d.bbox.size_x / 2, cy + d.bbox.size_y / 2, score))
        # 3) 매칭: 짝지어진 투영 박스의 원래 줄기 번호 집합
        matches = greedy_match(proj, det_boxes, self.g('iou_threshold'))
        confirmed = {proj_idx[i] for i, _ in matches}

        fused = TreeArray()
        fused.header = self.trees.header
        for i, src in enumerate(self.trees.trees):
            tr = copy.deepcopy(src)          # 원본은 다음 영상에서도 쓰이므로 복사본만 수정
            tr.camera_confirmed = i in confirmed
            if tr.camera_confirmed:
                # 카메라로도 확인된 줄기는 신뢰도 +0.3 (두 센서가 독립적으로 봤다는 근거, 경험적 값)
                tr.confidence = float(min(1.0, tr.confidence + 0.3))
            fused.trees.append(tr)
        self.pub_fused.publish(fused)

        need_bgr = self.g('publish_overlay') or self.g('record_dataset')
        if not need_bgr:
            return
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.frame_count += 1
        # 4) 자동 라벨링: 투영된 줄기가 하나 이상일 때만 N 장마다 저장 (빈 라벨 영상은 저장 안 함)
        if self.g('record_dataset') and proj and self.frame_count % int(self.g('record_every_n')) == 0:
            self.save_sample(bgr, proj, size)
        if self.g('publish_overlay'):
            over = bgr.copy()
            for k, b in enumerate(proj):
                ok = proj_idx[k] in confirmed
                color = (0, 220, 0) if ok else (0, 140, 255)      # BGR: 초록=카메라 확인, 주황=미확인
                cv2.rectangle(over, (int(b.x1), int(b.y1)), (int(b.x2), int(b.y2)), color, 2)
            for b in det_boxes:
                cv2.rectangle(over, (int(b.x1), int(b.y1)), (int(b.x2), int(b.y2)), (255, 80, 0), 1)  # 파랑=YOLO
            cv2.putText(over, f'LiDAR trunks {len(proj)}  camera {len(det_boxes)}  matched {len(matches)}',
                        (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            out = self.bridge.cv2_to_imgmsg(over, encoding='bgr8')
            out.header = msg.header
            self.pub_overlay.publish(out)

    def save_sample(self, bgr, boxes, size):
        """영상과 YOLO 라벨(txt, 클래스 0=trunk) 을 dataset_dir/images, labels 에 같은 이름으로 저장.

        파일명: <벽시계 ms>_<저장 순번> → 여러 번 실행해도 이름이 겹치지 않음.
        주의: 라벨은 LiDAR 투영 박스이므로 캘리브레이션 오차·가림이 그대로 들어간다 (학습 전 샘플 검수 권장).
        """
        root = self.g('dataset_dir')
        name = f'{int(time.time() * 1000)}_{self.saved:06d}'
        cv2.imwrite(os.path.join(root, 'images', name + '.jpg'), bgr)
        with open(os.path.join(root, 'labels', name + '.txt'), 'w') as f:
            f.write('\n'.join(yolo_label(b, size, 0) for b in boxes) + '\n')
        self.saved += 1
        if self.saved % 50 == 0:
            self.get_logger().info(f'자동 라벨 {self.saved}장 저장')


def main(args=None):
    """노드 실행 진입점 (ros2 run orchard_perception tree_fusion_node)."""
    rclpy.init(args=args)
    node = TreeFusionNode()
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
