"""onnxruntime / OpenCV 백엔드로 실제 ONNX 파일을 읽어 검출까지 되는지 확인.
YOLOv8/11 출력 형태 (1, 4+nc, N) 를 흉내 낸 작은 ONNX 그래프를 만들어 쓴다.

주의: 모듈 최상단에서 pytest.importorskip 를 쓰면 ROS 2 의 launch_testing pytest 플러그인이
모듈을 미리 import 하는 과정에서 skip 이 테스트 폴더 전체로 번진다 (CI 에서 확인).
그래서 설치 여부는 find_spec 으로만 확인하고 import 는 함수 안에서 한다.
"""
import importlib.util

import numpy as np
import pytest

from orchard_perception.yolo import YoloOnnxDetector

HAS_ONNX = importlib.util.find_spec('onnx') is not None
HAS_CV2 = importlib.util.find_spec('cv2') is not None
HAS_ORT = importlib.util.find_spec('onnxruntime') is not None

pytestmark = pytest.mark.skipif(not (HAS_ONNX and HAS_CV2), reason='onnx 또는 cv2 미설치')


def _fake_yolo(path, box_xywh, score=0.9, n=8400):
    """입력과 무관하게 후보 1개(123번, box_xywh·score)만 있는 YOLOv8 형식 ONNX 를 path 에 저장.

    n=8400: 640×640 입력 YOLOv8 의 실제 후보 수 (80² + 40² + 20²).
    """
    import onnx
    from onnx import TensorProto, helper, numpy_helper

    out = np.zeros((1, 5, n), dtype=np.float32)
    out[0, :4, 123] = box_xywh
    out[0, 4, 123] = score
    const = numpy_helper.from_array(out, name='det')
    # 입력을 실제로 사용하도록 0 * ReduceMean(images) 를 더한다
    zero = numpy_helper.from_array(np.zeros((1,), dtype=np.float32), name='zero')
    nodes = [
        helper.make_node('ReduceMean', ['images'], ['m'], keepdims=0),
        helper.make_node('Mul', ['m', 'zero'], ['z']),
        helper.make_node('Add', ['det', 'z'], ['output0']),
    ]
    graph = helper.make_graph(
        nodes, 'fake_yolo',
        [helper.make_tensor_value_info('images', TensorProto.FLOAT, [1, 3, 640, 640])],
        [helper.make_tensor_value_info('output0', TensorProto.FLOAT, [1, 5, n])],
        initializer=[const, zero])
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid('', 12)])
    model.ir_version = 8
    onnx.save(model, path)


@pytest.mark.parametrize('backend', [
    pytest.param('onnxruntime', marks=pytest.mark.skipif(not HAS_ORT, reason='onnxruntime 미설치')),
    'opencv',
])
def test_detect_with_real_onnx_file(tmp_path, backend):
    """640×480 영상 → letterbox → 가짜 모델 → 역변환: 박스 1개, 중심 (320, 320), 높이 160 px 로 복원."""
    path = str(tmp_path / 'fake.onnx')
    _fake_yolo(path, [320, 400, 40, 160])           # 입력(640x640) 좌표
    det = YoloOnnxDetector(path, backend=backend)
    assert det.backend == backend
    img = np.zeros((480, 640, 3), dtype=np.uint8)    # 640x480 → 위아래 80px 패딩
    boxes = det.detect(img)
    assert len(boxes) == 1
    b = boxes[0]
    assert abs((b.x1 + b.x2) / 2 - 320) < 1.0
    assert abs((b.y1 + b.y2) / 2 - 320) < 1.0        # 400 - 80(패딩)
    assert abs(b.h - 160) < 1.0 and b.score > 0.85
