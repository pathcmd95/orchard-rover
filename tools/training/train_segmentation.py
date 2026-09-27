#!/usr/bin/env python3
"""과수원 세그멘테이션(주행가능 영역) 경량 모델 학습 → ONNX 내보내기.

모델: LR-ASPP + MobileNetV3-Small (torchvision). Orin 에서 TensorRT/onnxruntime 으로 실시간.
데이터: <data>/images/*.jpg + <data>/labels/*.png (픽셀값 = 클래스 번호, orchard_perception/seg_classes.py)
  - 시뮬: sim.launch.py record_seg:=true 로 자동 수집 (data/seg_dataset_sim)
  - 현장: 같은 형식으로 라벨링(CVAT 등)해 --data 를 여러 개 주면 섞어서 학습

  pip install torch torchvision onnx          # 개발 PC (GPU 있으면 CUDA 버전)
  python3 tools/training/train_segmentation.py --data data/seg_dataset_sim --out models/orchard_seg.onnx
  python3 tools/training/train_segmentation.py --data data/seg_dataset_sim data/seg_field --init models/orchard_seg.pt \
      --epochs 20 --lr 3e-4                    # 현장 데이터로 미세조정
출력: models/orchard_seg.onnx (추론 노드용), models/orchard_seg.pt (이어 학습용), 검증 IoU 표

언제 쓰나: 시뮬/현장에서 세그멘테이션 데이터를 모은 뒤, GPU 가 있는 PC 에서 실행한다.
  학습한 ONNX 는 seg_path_node(model_path, sim.launch.py seg_model:=...)가 읽는다.
주요 옵션: --size(입력 해상도 WxH, 추론 노드와 같아야 함) --epochs --batch --lr --val(검증 비율)
          --init(이어 학습할 .pt) --no-pretrained(ImageNet 가중치 없이) --threads(CPU 스레드 수)
평가 지표: IoU(Intersection over Union) = 예측∩정답 / 예측∪정답 (클래스별). mIoU 는 그 평균.

References
  - A. Howard et al., "Searching for MobileNetV3", ICCV 2019.
    (LR-ASPP 세그멘테이션 헤드와 MobileNetV3 백본 모두 이 논문에서 제안)
  - torchvision (MobileNetV3 / LRASPP 구현, ImageNet 사전학습 가중치): https://github.com/pytorch/vision
"""
from __future__ import annotations

import argparse
import glob
import os
import random
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # 저장소 최상위 (세 단계 위)
sys.path.insert(0, os.path.join(ROOT, 'ros2_ws', 'src', 'orchard_perception'))
from orchard_perception.seg_classes import CLASSES, NUM_CLASSES  # noqa: E402

# ImageNet 정규화 평균/표준편차 (RGB). 사전학습 백본이 이 분포로 학습됐으므로 입력도 똑같이 정규화한다.
# 추론 쪽 전처리(orchard_perception/seg_model.py)도 같은 값을 써야 한다.
MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)


def build_model(num_classes: int = NUM_CLASSES, pretrained: bool = True):
    """LR-ASPP + MobileNetV3-Small 세그멘테이션 모델을 만든다.

    Args:
        num_classes: 출력 클래스 수 (seg_classes.CLASSES 길이).
        pretrained: True 면 ImageNet 사전학습 백본 가중치를 시도 (실패하면 무작위 초기화).
    Returns:
        입력 (N, 3, H, W) → 출력 logits (N, num_classes, H, W) 텐서 하나를 내는 nn.Module.
    """
    import torch.nn as nn
    from torchvision.models import mobilenet_v3_small
    from torchvision.models._utils import IntermediateLayerGetter
    from torchvision.models.segmentation.lraspp import LRASPP
    weights = None
    if pretrained:
        try:
            from torchvision.models import MobileNet_V3_Small_Weights
            weights = MobileNet_V3_Small_Weights.IMAGENET1K_V1
        except Exception:  # noqa: BLE001
            weights = None
    # dilated=True: 마지막 단계의 stride 대신 dilation 을 써 출력 해상도를 1/16 로 유지 (세그멘테이션용)
    try:
        backbone = mobilenet_v3_small(weights=weights, dilated=True).features
    except Exception as exc:  # noqa: BLE001  (오프라인 등으로 가중치 내려받기 실패)
        print(f'ImageNet 가중치 없이 시작: {exc}')
        backbone = mobilenet_v3_small(weights=None, dilated=True).features
    # torchvision 의 lraspp_mobilenet_v3_large 와 같은 방식으로 특징맵 두 개를 고른다.
    #   _is_cn 표시가 있는 블록 = 해상도가 줄어드는 단계. low = 고해상도(1/8) 특징, high = 마지막(1/16) 특징
    stage = [0] + [i for i, b in enumerate(backbone) if getattr(b, '_is_cn', False)] + [len(backbone) - 1]
    low, high = stage[-4], stage[-1]
    low_ch, high_ch = backbone[low].out_channels, backbone[high].out_channels
    # 백본에서 두 층의 출력을 {'low': ..., 'high': ...} dict 로 꺼내도록 감싼다
    backbone = IntermediateLayerGetter(backbone, return_layers={str(low): 'low', str(high): 'high'})
    # LR-ASPP 헤드: high 특징에 전역 풀링 기반 어텐션을 곱하고 low 특징과 더해 클래스별 점수를 낸다
    model = LRASPP(backbone, low_ch, high_ch, num_classes, inter_channels=128)
    for m in model.modules():          # torchvision MobileNetV3 의 BN momentum 0.01 은 처음부터 학습할 때
        if isinstance(m, nn.BatchNorm2d):   # 추론용 통계가 너무 늦게 따라와 검증 결과가 한 클래스로 뭉개진다
            m.momentum = 0.1

    class Wrap(nn.Module):             # ONNX 출력은 텐서 하나 ('logits')
        """torchvision 세그멘테이션 모델의 dict 출력({'out': ...})을 텐서 하나로 바꾸는 래퍼."""

        def __init__(self, m):
            """감쌀 모델 m 을 저장한다."""
            super().__init__()
            self.m = m

        def forward(self, x):
            """입력 이미지 배치 → 클래스별 logits (N, C, H, W)."""
            return self.m(x)['out']
    return Wrap(model)


def list_pairs(dirs):
    """여러 데이터 폴더에서 (이미지 경로, 라벨 PNG 경로) 쌍 목록을 만든다. 라벨이 없는 이미지는 건너뛴다."""
    pairs = []
    for d in dirs:
        imgs = glob.glob(os.path.join(d, 'images', '*.jpg')) + glob.glob(os.path.join(d, 'images', '*.png'))
        for img in sorted(imgs):
            lab = os.path.join(d, 'labels', os.path.splitext(os.path.basename(img))[0] + '.png')
            if os.path.isfile(lab):
                pairs.append((img, lab))
    return pairs


class SegDataset:
    """(이미지, 라벨) 쌍을 읽어 크기 조정·데이터 증강·정규화한 텐서를 돌려주는 PyTorch Dataset.

    Args:
        pairs: list_pairs() 결과.
        size: (W, H) 학습 입력 크기.
        augment: True 면 학습용 무작위 증강(반전·확대·밝기/색·흐림·잡음)을 적용 (검증은 False).
    """

    def __init__(self, pairs, size, augment):
        """데이터 쌍과 설정을 저장한다 (cv2 는 DataLoader 워커에서도 쓰도록 속성으로 보관)."""
        import cv2
        self.cv2, self.pairs, self.size, self.augment = cv2, pairs, size, augment

    def __len__(self):
        """데이터 쌍 개수."""
        return len(self.pairs)

    def __getitem__(self, i):
        """i 번째 샘플 → (이미지 텐서 (3, H, W) float32 정규화, 라벨 텐서 (H, W) int64)."""
        import torch
        cv2 = self.cv2
        img = cv2.imread(self.pairs[i][0], cv2.IMREAD_COLOR)
        lab = cv2.imread(self.pairs[i][1], cv2.IMREAD_UNCHANGED)
        if lab.ndim == 3:                                 # 라벨이 3채널로 저장된 경우 첫 채널만 사용
            lab = lab[..., 0]
        w, h = self.size
        if self.augment:
            if random.random() < 0.5:                     # 좌우 반전 (통로는 좌우 대칭)
                img, lab = img[:, ::-1], lab[:, ::-1]
            s = random.uniform(1.0, 1.25)                 # 약간 확대 후 자르기
            H, W = img.shape[:2]
            ch, cw = int(H / s), int(W / s)
            y0, x0 = random.randint(0, H - ch), random.randint(0, W - cw)
            img, lab = img[y0:y0 + ch, x0:x0 + cw], lab[y0:y0 + ch, x0:x0 + cw]
        # 라벨은 클래스 번호이므로 반드시 최근접(INTER_NEAREST) 보간 — 평균을 내면 없는 번호가 생긴다
        img = cv2.resize(np.ascontiguousarray(img), (w, h), interpolation=cv2.INTER_AREA)
        lab = cv2.resize(np.ascontiguousarray(lab), (w, h), interpolation=cv2.INTER_NEAREST)
        x = img[:, :, ::-1].astype(np.float32) / 255.0   # OpenCV BGR → RGB, 0~1 범위
        if self.augment:                                  # 밝기·대비·색조·흐림 (시뮬↔현장 차이 줄이기)
            x = x * random.uniform(0.6, 1.4) + random.uniform(-0.12, 0.12)
            x = x * np.random.uniform(0.85, 1.15, 3).astype(np.float32)
            gray = x.mean(axis=2, keepdims=True)
            x = gray + (x - gray) * random.uniform(0.6, 1.4)
            if random.random() < 0.3:
                x = cv2.GaussianBlur(x, (3, 3), 0)
            x = x + np.random.normal(0, 0.02, x.shape).astype(np.float32)
            x = np.clip(x, 0, 1)
        x = (x - MEAN) / STD
        # HWC → CHW (PyTorch 입력 순서)
        return torch.from_numpy(x.transpose(2, 0, 1).copy()), torch.from_numpy(lab.astype(np.int64))


def evaluate(model, loader, device):
    """검증 데이터 전체에 대해 클래스별 IoU 배열을 계산한다 (정답·예측 모두 없는 클래스는 NaN)."""
    import torch
    inter = np.zeros(NUM_CLASSES)
    union = np.zeros(NUM_CLASSES)
    model.eval()
    with torch.no_grad():
        for x, y in loader:
            p = model(x.to(device)).argmax(1).cpu().numpy()
            y = y.numpy()
            for c in range(NUM_CLASSES):
                inter[c] += np.logical_and(p == c, y == c).sum()
                union[c] += np.logical_or(p == c, y == c).sum()
    iou = np.where(union > 0, inter / np.maximum(union, 1), np.nan)
    return iou


def main():
    """명령행 인자 해석 → 데이터 분할 → 학습(최고 mIoU 모델 저장) → 최종 평가 → ONNX 내보내기."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--data', nargs='+', required=True)
    ap.add_argument('--out', default=os.path.join(ROOT, 'models', 'orchard_seg.onnx'))
    ap.add_argument('--size', default='320x240', help='학습·추론 입력 크기 WxH')
    ap.add_argument('--epochs', type=int, default=30)
    ap.add_argument('--batch', type=int, default=16)
    ap.add_argument('--lr', type=float, default=1e-3)
    ap.add_argument('--val', type=float, default=0.1, help='검증 비율')
    ap.add_argument('--init', default='', help='이어 학습할 .pt')
    ap.add_argument('--workers', type=int, default=2)
    ap.add_argument('--no-pretrained', action='store_true')
    ap.add_argument('--threads', type=int, default=0)
    a = ap.parse_args()

    import torch
    from torch.utils.data import DataLoader
    if a.threads:
        torch.set_num_threads(a.threads)
    # 난수 시드 고정 → 같은 데이터면 같은 학습/검증 분할 (재현성)
    random.seed(0)
    torch.manual_seed(0)
    size = tuple(int(v) for v in a.size.lower().split('x'))
    pairs = list_pairs(a.data)
    if len(pairs) < 10:
        sys.exit(f'데이터가 너무 적습니다: {len(pairs)}쌍 ({a.data})')
    random.shuffle(pairs)
    n_val = max(1, int(len(pairs) * a.val))
    train, val = pairs[n_val:], pairs[:n_val]
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f'학습 {len(train)} / 검증 {len(val)}, 입력 {size}, 장치 {device}')

    # 클래스 불균형 보정 가중치 (빈도의 -0.5 제곱)
    #   드문 클래스(장애물·구조물)의 손실을 키워 모델이 흔한 클래스만 맞히지 않게 한다. 속도를 위해 300장만 샘플링.
    import cv2
    freq = np.zeros(NUM_CLASSES)
    for _, lab in train[:300]:
        l_ = cv2.imread(lab, cv2.IMREAD_UNCHANGED)
        l_ = l_[..., 0] if l_.ndim == 3 else l_
        freq += np.bincount(l_.ravel(), minlength=NUM_CLASSES)[:NUM_CLASSES]
    wts = 1.0 / np.sqrt(np.maximum(freq / freq.sum(), 1e-4))
    wts = wts / wts.mean()
    print('클래스 가중치:', ' '.join(f'{c}={w:.2f}' for c, w in zip(CLASSES, wts)))

    model = build_model(pretrained=not a.no_pretrained).to(device)
    if a.init:                                         # 미세조정: 이전에 학습한 가중치에서 시작
        model.load_state_dict(torch.load(a.init, map_location=device))
    # 학습 로더는 섞고(shuffle) 증강, 검증 로더는 순서대로·증강 없이
    tl = DataLoader(SegDataset(train, size, True), batch_size=a.batch, shuffle=True, num_workers=a.workers,
                    drop_last=True)
    vl = DataLoader(SegDataset(val, size, False), batch_size=a.batch, num_workers=a.workers)
    # AdamW + OneCycle 학습률 (처음 올렸다가 끝으로 갈수록 낮춤), 가중 교차엔트로피 손실
    opt = torch.optim.AdamW(model.parameters(), lr=a.lr, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, a.lr, total_steps=a.epochs * len(tl))
    lossf = torch.nn.CrossEntropyLoss(weight=torch.tensor(wts, dtype=torch.float32, device=device))
    best, pt = -1.0, os.path.splitext(a.out)[0] + '.pt'   # 최고 성능 PyTorch 가중치는 ONNX 옆에 .pt 로 저장
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    for ep in range(a.epochs):
        model.train()
        t0, tot = time.time(), 0.0
        for x, y in tl:
            x, y = x.to(device), y.to(device)
            loss = lossf(model(x), y)
            opt.zero_grad()
            loss.backward()
            opt.step()
            sched.step()
            tot += loss.item()
        iou = evaluate(model, vl, device)
        miou = float(np.nanmean(iou[1:]))            # 미분류/하늘 제외
        print(f'[{ep + 1}/{a.epochs}] loss {tot / len(tl):.3f}  mIoU {miou:.3f}  '
              f'주행가능 IoU {iou[1]:.3f}  ({time.time() - t0:.0f}s)', flush=True)
        if miou > best:                                # 검증 mIoU 가 가장 좋은 에폭의 가중치만 남긴다
            best = miou
            torch.save(model.state_dict(), pt)
    # 마지막 에폭이 아니라 최고 에폭 가중치로 최종 평가·내보내기
    model.load_state_dict(torch.load(pt, map_location=device))
    iou = evaluate(model, vl, device)
    print('\n최종 검증 IoU')
    for c, v in zip(CLASSES, iou):
        print(f'  {c:12s} {v:.3f}')
    # ONNX 내보내기: 고정 입력 크기 (1, 3, H, W), 입력 이름 'images', 출력 이름 'logits', opset 17
    model = model.cpu().eval()
    dummy = torch.zeros(1, 3, size[1], size[0])
    kw = dict(input_names=['images'], output_names=['logits'], opset_version=17)
    try:
        torch.onnx.export(model, dummy, a.out, dynamo=False, **kw)
    except TypeError:                                  # 옛 PyTorch 에는 dynamo 인자가 없다
        torch.onnx.export(model, dummy, a.out, **kw)
    print(f'ONNX 저장: {a.out}  (PyTorch: {pt})')


if __name__ == '__main__':
    main()
