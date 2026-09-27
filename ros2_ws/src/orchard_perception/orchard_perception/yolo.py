"""YOLO(ONNX) 추론기 — onnxruntime(기본) 또는 OpenCV DNN.

PyTorch 없이 동작하므로 x86 PC 와 Jetson Orin 모두에서 같은 코드로 돌아간다.
지원 출력 형식
- YOLOv8/v11 (ultralytics export format=onnx): (1, 4+nc, N)
- YOLOv5: (1, N, 5+nc)  (objectness 포함)

파이프라인 내 역할
  camera_tree_node 가 사용. BGR 영상 → [letterbox 전처리 → ONNX 추론 → 출력 해석 → NMS] → Box 리스트
입력 / 출력
  입력: BGR uint8 영상 H×W×3 (cv_bridge 로 변환한 카메라 영상)
  출력: projection.Box 리스트 (원본 영상 픽셀 좌표 x1,y1,x2,y2 [px], score 0~1, cls 클래스 번호)
주요 튜닝 파라미터 (camera_tree_node, config/sim.yaml · robot.yaml)
  model_path, input_size(학습 시 imgsz 와 같게, 기본 640), conf_threshold(0.35), nms_threshold(0.45),
  backend(auto|onnxruntime|opencv), use_cuda, yolov5_format
  모델 학습: tools/training/train_yolo.py (docs/04_perception_training.md)

References
  J. Redmon, S. Divvala, R. Girshick, A. Farhadi, "You Only Look Once: Unified, Real-Time Object Detection",
  CVPR 2016. (이 파일은 YOLO 계열 모델의 추론 후처리만 구현; v5/v8/v11 은 Ultralytics 공개 구현)
  NMS 는 OpenCV cv2.dnn.NMSBoxes 사용.
"""
from __future__ import annotations

import numpy as np

from .projection import Box


def letterbox_params(img_w: int, img_h: int, size: int):
    """letterbox(비율 유지 축소 + 회색 여백) 파라미터 계산.

    img_w, img_h: 원본 영상 크기 [px], size: 정사각 네트워크 입력 한 변 [px].
    반환: (scale, pad_x, pad_y, new_w, new_h). 네트워크 좌표 = 원본 좌표 × scale + pad.
    예) 640×480 → 640: scale 1.0, 위아래 80 px 여백.
    """
    scale = min(size / img_w, size / img_h)          # 긴 변이 size 에 맞도록 (가로세로 비율 유지)
    new_w, new_h = int(round(img_w * scale)), int(round(img_h * scale))
    pad_x = (size - new_w) / 2.0                      # 좌우 여백 (양쪽 같게 가운데 정렬)
    pad_y = (size - new_h) / 2.0                      # 위아래 여백
    return scale, pad_x, pad_y, new_w, new_h


def decode_output(out: np.ndarray, conf_thr: float, scale: float, pad_x: float, pad_y: float,
                  img_w: int, img_h: int, has_objectness: bool = False):
    """네트워크 출력 -> (boxes[x,y,w,h], scores, class_ids) (원본 영상 좌표).

    has_objectness=True 이면 YOLOv5 형식(cx,cy,w,h,obj,cls...), 아니면 v8/v11 형식(cx,cy,w,h,cls...).
    conf_thr: 이 점수 미만 후보는 버림. scale/pad_x/pad_y: letterbox_params 결과 (역변환에 사용).
    img_w, img_h: 원본 영상 크기 [px] (박스를 화면 안으로 자름).
    반환 boxes: Mx4 (왼쪽 위 x, y, 너비, 높이) [px] — cv2.dnn.NMSBoxes 입력 형식.
    """
    out = np.squeeze(np.asarray(out, dtype=np.float32), axis=0) if np.ndim(out) == 3 else np.asarray(out)
    if out.ndim != 2:
        raise ValueError(f'unexpected YOLO output shape {out.shape}')
    # v8/v11 은 (4+nc, N) 로 나오므로 행이 적으면 전치한다 (N 은 보통 수천).
    if out.shape[0] < out.shape[1]:
        out = out.T
    boxes_c = out[:, :4]                              # (cx, cy, w, h) 네트워크 입력 좌표 [px]
    if has_objectness:
        cls_scores = out[:, 5:] * out[:, 4:5]         # v5: 클래스 점수 × objectness
    else:
        cls_scores = out[:, 4:]
    if cls_scores.shape[1] == 0:
        raise ValueError('YOLO output has no class columns')
    class_ids = np.argmax(cls_scores, axis=1)         # 후보마다 가장 점수 높은 클래스
    scores = cls_scores[np.arange(cls_scores.shape[0]), class_ids]
    keep = scores >= conf_thr
    boxes_c, scores, class_ids = boxes_c[keep], scores[keep], class_ids[keep]
    # letterbox 역변환: 여백을 빼고 scale 로 나눠 원본 영상 좌표로
    cx = (boxes_c[:, 0] - pad_x) / scale
    cy = (boxes_c[:, 1] - pad_y) / scale
    w = boxes_c[:, 2] / scale
    h = boxes_c[:, 3] / scale
    # 중심·크기 → 왼쪽 위 + 크기, 화면 밖으로 나간 부분은 잘라냄
    x = np.clip(cx - w / 2, 0, img_w - 1)
    y = np.clip(cy - h / 2, 0, img_h - 1)
    w = np.clip(w, 0, img_w - x)
    h = np.clip(h, 0, img_h - y)
    return np.c_[x, y, w, h], scores, class_ids


class YoloOnnxDetector:
    """backend: 'onnxruntime'(권장) | 'opencv' | 'auto'(onnxruntime 이 있으면 사용).

    Ubuntu 24.04 의 python3-opencv(4.6)는 최신 YOLO ONNX 를 못 읽는 경우가 있어
    onnxruntime 을 기본으로 쓰고, 없을 때만 OpenCV DNN 을 쓴다.
    """

    def __init__(self, model_path: str, input_size: int = 640, conf_thr: float = 0.35,
                 nms_thr: float = 0.45, use_cuda: bool = False, yolov5: bool = False,
                 backend: str = 'auto'):
        """ONNX 모델을 불러온다.

        model_path: .onnx 경로, input_size: 정사각 입력 한 변 [px] (학습 imgsz 와 같게)
        conf_thr: 검출 점수 기준 (낮추면 더 많이 검출·오검출 증가), nms_thr: NMS IoU 기준 (겹친 박스 제거)
        use_cuda: GPU 사용 (onnxruntime-gpu 또는 CUDA 빌드 OpenCV 필요), yolov5: v5 출력 형식이면 True
        backend: 'auto' | 'onnxruntime' | 'opencv'. 'onnxruntime' 을 지정했는데 미설치면 ImportError.
        """
        import cv2  # 지연 import (테스트 환경에서 cv2 없이도 decode 테스트 가능)
        self.cv2 = cv2
        self.size = input_size
        self.conf_thr = conf_thr
        self.nms_thr = nms_thr
        self.yolov5 = yolov5
        self.session = None
        self.net = None
        if backend in ('auto', 'onnxruntime'):
            try:
                import onnxruntime as ort
                providers = ['CPUExecutionProvider']
                # CUDA 제공자가 있으면 맨 앞에 넣어 우선 사용 (없으면 CPU 로 자동 대체)
                if use_cuda and 'CUDAExecutionProvider' in ort.get_available_providers():
                    providers.insert(0, 'CUDAExecutionProvider')
                self.session = ort.InferenceSession(model_path, providers=providers)
                self.input_name = self.session.get_inputs()[0].name
            except ImportError:
                if backend == 'onnxruntime':
                    raise
        if self.session is None:                    # onnxruntime 을 못 쓰면 OpenCV DNN 으로
            self.net = cv2.dnn.readNetFromONNX(model_path)
            if use_cuda:
                self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
                self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        self.backend = 'onnxruntime' if self.session is not None else 'opencv'

    def preprocess(self, bgr: np.ndarray):
        """BGR 영상 → 네트워크 입력 blob (1×3×size×size, RGB, 0~1).

        반환: (blob, scale, left, top). left/top 은 실제로 붙인 정수 여백 [px] (좌표 역변환에 사용).
        """
        cv2 = self.cv2
        h, w = bgr.shape[:2]
        scale, pad_x, pad_y, nw, nh = letterbox_params(w, h, self.size)
        resized = cv2.resize(bgr, (nw, nh))
        # 114 = Ultralytics letterbox 기본 여백 회색값 (학습 때와 같은 값을 써야 경계 오검출이 적음)
        canvas = np.full((self.size, self.size, 3), 114, dtype=np.uint8)
        # 여백이 x.5 일 때 Ultralytics 와 같은 쪽으로 반올림하려고 0.1 을 뺌
        top, left = int(round(pad_y - 0.1)), int(round(pad_x - 0.1))
        canvas[top:top + nh, left:left + nw] = resized
        # 1/255 로 0~1 정규화, swapRB 로 BGR → RGB, HWC → NCHW
        blob = cv2.dnn.blobFromImage(canvas, 1 / 255.0, (self.size, self.size), swapRB=True, crop=False)
        return blob, scale, left, top

    def detect(self, bgr: np.ndarray) -> list[Box]:
        """BGR 영상 한 장에서 검출. 반환: NMS 후 Box 리스트 (원본 영상 좌표 [px])."""
        h, w = bgr.shape[:2]
        blob, scale, left, top = self.preprocess(bgr)
        if self.session is not None:
            out = self.session.run(None, {self.input_name: blob.astype(np.float32)})[0]
        else:
            self.net.setInput(blob)
            out = self.net.forward()
        boxes, scores, cls = decode_output(out, self.conf_thr, scale, left, top, w, h, self.yolov5)
        if len(scores) == 0:
            return []
        # NMS: 같은 물체에 겹쳐 나온 박스 중 점수 최고만 남김 (클래스 구분 없이 적용)
        idx = self.cv2.dnn.NMSBoxes(boxes.tolist(), scores.tolist(), self.conf_thr, self.nms_thr)
        idx = np.array(idx).reshape(-1)          # OpenCV 버전에 따라 (K,1) 또는 (K,) → 1차원으로
        # (x, y, w, h) → Box(x1, y1, x2, y2)
        return [Box(float(boxes[i, 0]), float(boxes[i, 1]), float(boxes[i, 0] + boxes[i, 2]),
                    float(boxes[i, 1] + boxes[i, 3]), float(scores[i]), int(cls[i])) for i in idx]
