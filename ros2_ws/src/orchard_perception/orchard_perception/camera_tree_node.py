"""RealSense D455 컬러 영상 → YOLO(ONNX) 과수 줄기 검출 노드.

model_path 가 비어 있으면 추론 없이 대기한다 (모델 학습 전 단계).
학습용 데이터는 tree_fusion_node 의 자동 라벨링 기능으로 모은다 (docs/04 참고).

파이프라인 내 역할
  카메라 → [camera_tree_node: YOLO 줄기 검출] → tree_fusion_node (LiDAR 줄기와 매칭해 camera_confirmed 표시)
입력 (ROS topic)
  image_topic (기본 /camera/color/image_raw)  sensor_msgs/Image, BGR/RGB 컬러
출력 (ROS topic)
  /orchard/detections        vision_msgs/Detection2DArray  줄기 박스 (픽셀 좌표, 중심+크기)
  /orchard/debug/detections  sensor_msgs/Image             박스를 그린 디버그 영상 (publish_debug_image=true 일 때)
주요 파라미터 (config/sim.yaml · robot.yaml 의 camera_tree_node)
  model_path   학습한 ONNX 경로 (tools/training/train_yolo.py 로 학습). 비어 있으면 검출 비활성
  conf_threshold / nms_threshold  검출 점수 / NMS IoU 기준, input_size  학습 imgsz 와 같게
  use_cuda / backend  추론 장치·백엔드, max_rate_hz  최대 처리 주기 [Hz] (Orin 부하 조절)
추론 세부 구현은 yolo.py 참고.
"""
from __future__ import annotations

import os

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from .yolo import YoloOnnxDetector


class CameraTreeNode(Node):
    """컬러 영상을 받아 YOLO 로 줄기를 검출하고 Detection2DArray 로 발행하는 노드."""

    def __init__(self):
        """파라미터 선언, 모델 로드(있을 때만), 발행자·구독자 생성."""
        super().__init__('camera_tree_node')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('model_path', '')
        self.declare_parameter('class_names', ['trunk'])       # 클래스 번호 → 이름 (학습 데이터셋 순서와 같게)
        self.declare_parameter('input_size', 640)              # 네트워크 입력 한 변 [px]
        self.declare_parameter('conf_threshold', 0.35)         # 검출 점수 기준 (0~1)
        self.declare_parameter('nms_threshold', 0.45)          # NMS 에서 겹침으로 볼 IoU
        self.declare_parameter('use_cuda', False)
        self.declare_parameter('backend', 'auto')       # auto | onnxruntime | opencv
        self.declare_parameter('yolov5_format', False)          # YOLOv5 출력(objectness 포함) 이면 true
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('max_rate_hz', 10.0)             # 최대 처리 주기 [Hz]

        self.bridge = CvBridge()
        self.names = list(self.get_parameter('class_names').value)
        self.detector = None
        model = self.get_parameter('model_path').value
        if model and os.path.isfile(model):
            self.detector = YoloOnnxDetector(
                model,
                int(self.get_parameter('input_size').value),
                float(self.get_parameter('conf_threshold').value),
                float(self.get_parameter('nms_threshold').value),
                bool(self.get_parameter('use_cuda').value),
                bool(self.get_parameter('yolov5_format').value),
                str(self.get_parameter('backend').value))
            self.get_logger().info(f'YOLO 모델 로드: {model} (backend={self.detector.backend})')
        else:
            self.get_logger().warn(
                'model_path 가 비어 있거나 파일이 없습니다 → 카메라 검출 비활성. '
                'LiDAR 줄기 검출과 자동 라벨링은 계속 동작합니다.')

        self.pub_det = self.create_publisher(Detection2DArray, '/orchard/detections', 10)
        self.pub_dbg = self.create_publisher(Image, '/orchard/debug/detections', 2)
        # 처리 최소 간격 [s]. max_rate_hz 하한 0.1 Hz 로 0 나누기 방지
        self.min_period = 1.0 / max(0.1, float(self.get_parameter('max_rate_hz').value))
        self.last_t = 0.0
        # 센서 QoS(best effort): 카메라 드라이버와 QoS 가 맞아야 메시지가 들어온다
        self.create_subscription(Image, self.get_parameter('image_topic').value, self.on_image,
                                 qos_profile_sensor_data)

    def on_image(self, msg: Image):
        """영상 콜백: 주기 제한 → 검출 → Detection2DArray 발행 → (옵션) 디버그 영상 발행."""
        if self.detector is None:
            return
        # 노드 시계 기준 (시뮬에서는 use_sim_time 이면 시뮬 시간) 으로 max_rate_hz 초과 프레임은 건너뜀
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_t < self.min_period:
            return
        self.last_t = now
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        boxes = self.detector.detect(bgr)

        out = Detection2DArray()
        out.header = msg.header          # 영상 시각·frame_id 유지 → tree_fusion_node 가 시간 동기화에 사용
        for b in boxes:
            d = Detection2D()
            d.header = msg.header
            # vision_msgs 박스는 중심 + 크기 형식 [px]
            d.bbox.center.position.x = (b.x1 + b.x2) / 2.0
            d.bbox.center.position.y = (b.y1 + b.y2) / 2.0
            d.bbox.size_x = b.w
            d.bbox.size_y = b.h
            hyp = ObjectHypothesisWithPose()
            # class_names 에 없는 번호는 숫자 문자열로
            hyp.hypothesis.class_id = self.names[b.cls] if b.cls < len(self.names) else str(b.cls)
            hyp.hypothesis.score = float(b.score)
            d.results.append(hyp)
            out.detections.append(d)
        self.pub_det.publish(out)

        if self.get_parameter('publish_debug_image').value:
            for b in boxes:
                # (0, 200, 255) = BGR 주황색, 선 두께 2 px
                cv2.rectangle(bgr, (int(b.x1), int(b.y1)), (int(b.x2), int(b.y2)), (0, 200, 255), 2)
                label = self.names[b.cls] if b.cls < len(self.names) else str(b.cls)
                # 글자는 박스 위 4 px, 화면 위로 잘리지 않게 y ≥ 12 px
                cv2.putText(bgr, f'{label} {b.score:.2f}', (int(b.x1), max(12, int(b.y1) - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
            dbg = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
            dbg.header = msg.header
            self.pub_dbg.publish(dbg)


def main(args=None):
    """노드 실행 진입점 (ros2 run orchard_perception camera_tree_node)."""
    rclpy.init(args=args)
    node = CameraTreeNode()
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
