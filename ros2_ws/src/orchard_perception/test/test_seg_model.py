"""작은 ONNX(입력 채널 평균으로 클래스 점수를 만드는 그래프)로 SegModel 입출력 규약 확인.

확인 항목: ONNX 에서 입력 크기(320×240) 읽기, BGR→RGB 변환, 모델 크기의 uint8 라벨 영상 출력.
onnx / cv2 / onnxruntime 중 하나라도 없으면 건너뛴다.
"""
import importlib.util

import numpy as np
import pytest

HAS = all(importlib.util.find_spec(m) is not None for m in ('onnx', 'cv2', 'onnxruntime'))


@pytest.mark.skipif(not HAS, reason='onnx/cv2/onnxruntime 미설치')
def test_seg_model_io(tmp_path):
    """오른쪽 절반만 빨간 영상: 라벨이 왼쪽 0, 오른쪽 1 로 나와야 한다 (R 채널이 제대로 전달됨)."""
    import onnx
    from onnx import TensorProto, helper, numpy_helper

    from orchard_perception.seg_model import SegModel

    # logits[c] = w_c * R + b_c (R 채널 크기에 따라 클래스 결정) — 1x1 Conv
    w = np.zeros((7, 3, 1, 1), np.float32)
    w[1, 0] = 5.0          # R 이 크면 클래스 1
    b = np.zeros(7, np.float32)
    b[0] = 1.0             # 기본은 클래스 0
    nodes = [helper.make_node('Conv', ['images', 'W', 'B'], ['logits'])]
    graph = helper.make_graph(nodes, "seg",
                              [helper.make_tensor_value_info("images", TensorProto.FLOAT, [1, 3, 240, 320])],
                              [helper.make_tensor_value_info('logits', TensorProto.FLOAT, [1, 7, 240, 320])],
                              [numpy_helper.from_array(w, 'W'), numpy_helper.from_array(b, 'B')])
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid('', 13)])
    model.ir_version = 8
    path = tmp_path / 'seg.onnx'
    onnx.save(model, str(path))
    m = SegModel(str(path))
    assert m.size == (320, 240) and m.backend == 'onnxruntime'
    img = np.zeros((480, 640, 3), np.uint8)
    img[:, 320:, 2] = 255              # 오른쪽 절반 빨강(BGR 의 R)
    lab = m.predict(img)
    assert lab.shape == (240, 320)
    assert lab[:, :150].max() == 0 and lab[:, 170:].min() == 1
