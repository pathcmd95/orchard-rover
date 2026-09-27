"""세그멘테이션 ONNX 추론 (onnxruntime 우선, 없으면 OpenCV DNN).

모델 규약 (tools/training/train_segmentation.py 가 내보내는 형식)
  입력  'images' : 1×3×H×W float32, RGB, 0~1 에서 ImageNet 평균/표준편차로 정규화
  출력  'logits' : 1×C×H×W (클래스 점수)
입력 크기는 ONNX 에서 읽는다 (기본 320×240).

파이프라인 내 역할
  seg_path_node 가 사용. BGR 영상 → [전처리 → ONNX 추론 → argmax] → 픽셀별 클래스 라벨 영상 (seg_classes.py)
입력 / 출력
  입력: BGR uint8 영상 (크기 무관, 모델 입력 크기로 축소됨)
  출력: 라벨 영상 H×W uint8 (모델 입력 크기, 예: 240×320). 원본 크기로 되돌리지 않으므로
        seg_path_node 가 CameraInfo 내부행렬을 같은 비율로 축소해 쓴다.
설정: seg_path_node 의 model_path / use_cuda (config/sim.yaml · robot.yaml). 입력 해상도는 학습 시 --size 로 정해짐.

References
  학습 스크립트의 모델 구조 (LR-ASPP + MobileNetV3-Small, torchvision):
  A. Howard et al., "Searching for MobileNetV3", ICCV 2019.
"""
from __future__ import annotations

import numpy as np

# ImageNet RGB 채널별 평균/표준편차 (torchvision 사전학습 backbone 의 정규화 값, 학습 때와 반드시 같아야 함)
MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)


def preprocess(bgr: np.ndarray, size_wh: tuple[int, int], cv2) -> np.ndarray:
    """BGR 영상 → 모델 입력 텐서 1×3×H×W float32.

    size_wh: 모델 입력 (너비, 높이) [px]. cv2: 모듈을 인자로 받아 이 파일이 cv2 없이도 import 되게 함.
    단계: 축소(INTER_AREA, 축소 시 앨리어싱 적음) → BGR→RGB → 0~1 → ImageNet 정규화 → HWC→NCHW.
    """
    img = cv2.resize(bgr, size_wh, interpolation=cv2.INTER_AREA)[:, :, ::-1].astype(np.float32) / 255.0
    img = (img - MEAN) / STD
    return np.ascontiguousarray(img.transpose(2, 0, 1)[None])


class SegModel:
    """세그멘테이션 ONNX 모델 래퍼. backend 속성: 실제 사용 중인 'onnxruntime' | 'opencv'."""

    def __init__(self, model_path: str, use_cuda: bool = False, backend: str = 'auto',
                 default_size: tuple[int, int] = (320, 240)):
        """ONNX 모델을 불러오고 입력 크기를 읽는다.

        model_path: .onnx 경로, use_cuda: onnxruntime CUDA 제공자 사용 (있을 때만)
        backend: 'auto'(onnxruntime 있으면 사용) | 'onnxruntime' | 'opencv'
        default_size: ONNX 입력 크기가 동적(문자열 축)일 때 쓸 (너비, 높이) [px]
        """
        import cv2
        self.cv2 = cv2
        self.session = None
        self.net = None
        self.size = default_size
        if backend in ('auto', 'onnxruntime'):
            try:
                import onnxruntime as ort
                providers = ['CPUExecutionProvider']
                if use_cuda and 'CUDAExecutionProvider' in ort.get_available_providers():
                    providers.insert(0, 'CUDAExecutionProvider')
                self.session = ort.InferenceSession(model_path, providers=providers)
                inp = self.session.get_inputs()[0]
                self.input_name = inp.name
                shape = inp.shape                    # [N, C, H, W]; 동적 축이면 int 가 아닌 문자열
                if isinstance(shape[2], int) and isinstance(shape[3], int):
                    self.size = (shape[3], shape[2])     # (W, H) 순서로 저장 (cv2.resize 규약)
            except ImportError:
                if backend == 'onnxruntime':
                    raise
        if self.session is None:
            # OpenCV DNN 은 입력 크기를 읽지 않으므로 default_size 를 그대로 사용
            self.net = cv2.dnn.readNetFromONNX(model_path)
        self.backend = 'onnxruntime' if self.session is not None else 'opencv'

    def logits(self, bgr: np.ndarray) -> np.ndarray:
        """클래스 점수 C×H×W (배치 차원 제거)."""
        x = preprocess(bgr, self.size, self.cv2)
        if self.session is not None:
            return self.session.run(None, {self.input_name: x})[0][0]
        self.net.setInput(x)
        return self.net.forward()[0]

    def predict(self, bgr: np.ndarray) -> np.ndarray:
        """라벨 영상 (모델 입력 크기, uint8)."""
        # 픽셀마다 점수가 가장 큰 클래스 번호 (softmax 는 순서를 바꾸지 않으므로 생략)
        return np.argmax(self.logits(bgr), axis=0).astype(np.uint8)
