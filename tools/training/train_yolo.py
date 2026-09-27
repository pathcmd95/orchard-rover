#!/usr/bin/env python3
"""자동 라벨링 데이터로 과수 줄기 YOLO 모델을 학습하고 ONNX 로 내보낸다.

GPU 가 있는 PC(예: RTX 3080)에서 컨테이너 밖 가상환경으로 실행하는 것을 권장한다.
    python3 -m venv ~/yolo && source ~/yolo/bin/activate
    pip install ultralytics
    python3 tools/training/train_yolo.py --data data/orchard_dataset_sim --epochs 80
결과: models/trunk_yolo.onnx  → sim.launch.py camera_model:=/workspace/models/trunk_yolo.onnx

데이터 폴더 구조 (tree_fusion_node record_dataset:=true 가 만든 형태)
    <data>/images/*.jpg
    <data>/labels/*.txt   (YOLO 형식, 클래스 0 = trunk)

처리 순서
  1) --data 폴더를 학습/검증으로 무작위 분할해 --work 폴더에 복사하고 Ultralytics 용 data.yaml 생성
  2) --model(사전학습 가중치, 기본 yolo11n.pt)에서 시작해 --epochs 만큼 학습 (결과: runs/trunk/)
  3) 검증 mAP 출력 → ONNX(opset 12, 고정 입력 크기)로 내보내 --out 에 복사

주요 옵션
  --data       자동 라벨링 데이터 폴더 (필수)
  --work       분할 데이터를 만들 작업 폴더 (기본 data/yolo_split. 이전 실행 파일이 남으므로 데이터가 바뀌면 지우고 실행)
  --model      시작 가중치. n(nano)이 가장 가볍다 → Orin 실시간용. 정확도가 부족하면 s 등 큰 모델
  --epochs     학습 반복 횟수
  --imgsz      학습·추론 입력 크기 [px]. camera_tree_node 의 input_size(기본 640)와 같게
  --val-ratio  검증 데이터 비율
  --out        ONNX 저장 경로
지표: mAP50 = IoU 0.5 기준 평균 정밀도, mAP50-95 = IoU 0.5~0.95 평균 (높을수록 좋음)

References
  - Ultralytics YOLO (학습·검증·ONNX 내보내기 라이브러리): https://github.com/ultralytics/ultralytics
"""
from __future__ import annotations

import argparse
import os
import random
import shutil


def split(data: str, out: str, val_ratio: float, seed: int) -> str:
    """이미지/라벨을 학습·검증 폴더로 나눠 복사하고 Ultralytics data.yaml 경로를 돌려준다.

    Args:
        data: 원본 데이터 폴더 (images/, labels/ 포함).
        out: 분할 결과를 만들 폴더 (out/train, out/val, out/data.yaml).
        val_ratio: 검증 비율 (0~1). 최소 1장은 검증으로 보낸다.
        seed: 섞기 난수 시드 (같으면 같은 분할 → 재현성).
    Returns:
        생성한 data.yaml 경로.
    """
    images = sorted(f for f in os.listdir(os.path.join(data, 'images')) if f.endswith('.jpg'))
    if not images:
        raise SystemExit(f'{data}/images 에 이미지가 없습니다')
    random.Random(seed).shuffle(images)
    n_val = max(1, int(len(images) * val_ratio))
    for part, names in (('val', images[:n_val]), ('train', images[n_val:])):
        for sub in ('images', 'labels'):
            os.makedirs(os.path.join(out, part, sub), exist_ok=True)
        for name in names:
            stem = os.path.splitext(name)[0]
            shutil.copy(os.path.join(data, 'images', name), os.path.join(out, part, 'images', name))
            # 라벨 파일이 없는 이미지는 '줄기 없음(배경)' 이미지로 학습에 쓰인다
            lab = os.path.join(data, 'labels', stem + '.txt')
            if os.path.exists(lab):
                shutil.copy(lab, os.path.join(out, part, 'labels', stem + '.txt'))
    # Ultralytics 데이터셋 설정: 경로 + 클래스 이름 (클래스 0 = trunk 하나)
    yaml_path = os.path.join(out, 'data.yaml')
    with open(yaml_path, 'w', encoding='utf-8') as f:
        f.write(f'path: {os.path.abspath(out)}\ntrain: train/images\nval: val/images\nnames:\n  0: trunk\n')
    print(f'학습 {len(images) - n_val}장 / 검증 {n_val}장 → {yaml_path}')
    return yaml_path


def main():
    """명령행 인자 해석 → 데이터 분할 → YOLO 학습·검증 → ONNX 내보내기."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', required=True, help='자동 라벨링 데이터 폴더')
    ap.add_argument('--work', default='data/yolo_split')
    ap.add_argument('--model', default='yolo11n.pt', help='yolov8n.pt 등도 가능')
    ap.add_argument('--epochs', type=int, default=80)
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--val-ratio', type=float, default=0.15)
    ap.add_argument('--out', default='models/trunk_yolo.onnx')
    a = ap.parse_args()

    # ultralytics 는 무거운 선택 의존성이라 실제로 학습할 때만 import
    from ultralytics import YOLO   # pip install ultralytics

    data_yaml = split(a.data, a.work, a.val_ratio, seed=0)
    # 사전학습 가중치에서 시작 (처음 실행 시 인터넷에서 자동 다운로드)
    model = YOLO(a.model)
    # 학습 결과(가중치·그래프)는 runs/trunk/ 에 저장. exist_ok=True: 같은 폴더를 덮어씀
    model.train(data=data_yaml, epochs=a.epochs, imgsz=a.imgsz, project='runs', name='trunk', exist_ok=True)
    metrics = model.val()
    print(f'mAP50={metrics.box.map50:.3f}  mAP50-95={metrics.box.map:.3f}')
    # ONNX: opset 12 (OpenCV DNN·onnxruntime 호환), simplify 로 그래프 정리, dynamic=False → 고정 입력 크기
    onnx = model.export(format='onnx', imgsz=a.imgsz, opset=12, simplify=True, dynamic=False)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    shutil.copy(onnx, a.out)
    print(f'ONNX 저장: {a.out}')


if __name__ == '__main__':
    main()
