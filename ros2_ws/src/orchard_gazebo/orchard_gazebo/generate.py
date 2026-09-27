"""월드 + 로버 모델을 한 번에 생성하는 CLI.

ros2 run orchard_gazebo generate_orchard --rows 6 --obstacles 1

역할
  world_gen(과수원 월드 SDF + 메타 JSON)와 model_gen(센서를 붙인 로버 모델 SDF)을 차례로 부른다.
  sim.launch.py 는 이 모듈의 generate_all() 을 직접 호출한다 (launch 인자 rows, row_spacing, row_length,
  obstacles, seed, relief, bump_amp, spawn_*, lite, seg_camera, camera_lite 가 여기로 넘어온다).
생성 파일 (기본 --out-dir ~/.cache/orchard_rover/worlds)
  <out-dir>/orchard.sdf, <out-dir>/orchard_meta.json, <out-dir>/assets_<해시>/ (메시·텍스처 캐시)
  <out-dir>/../models/orchard_rover/model.sdf, model.config  → 이 models 폴더를 GZ_SIM_RESOURCE_PATH 에 넣는다.
CLI 옵션 전체는 world_gen.config_from_args 참고 (--help).
"""
from __future__ import annotations

import os
import sys

from .model_gen import generate_model
from .world_gen import config_from_args, generate_world


def default_px4_model(px4_dir: str | None = None) -> str:
    """PX4 rover_ackermann 모델 SDF 경로 (px4_dir 없으면 환경변수 PX4_DIR, 그것도 없으면 /opt/PX4-Autopilot)."""
    px4_dir = px4_dir or os.environ.get('PX4_DIR', '/opt/PX4-Autopilot')
    return os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'models', 'rover_ackermann', 'model.sdf')


def default_rover_yaml() -> str:
    """센서 장착 위치의 단일 출처 orchard_description/config/rover.yaml 경로 (설치본 우선, 없으면 소스 트리)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('orchard_description'), 'config', 'rover.yaml')
    except Exception:   # 소스 트리에서 직접 실행하는 경우
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.normpath(os.path.join(here, '..', '..', 'orchard_description', 'config', 'rover.yaml'))


def generate_all(argv=None, px4_model: str | None = None, rover_yaml: str | None = None,
                 seg_camera: bool = False, camera_lite: bool = False):
    """월드와 로버 모델을 생성하고 (world.sdf, meta.json, model.sdf, models 폴더) 경로를 반환.

    argv: world_gen CLI 인자 리스트 (None 이면 기본값). seg_camera: 세그멘테이션 정답 카메라 추가.
    camera_lite: 카메라 320×240·10 Hz 로 낮춤 (느린 PC).
    """
    cfg, out_dir = config_from_args(argv)
    world, meta = generate_world(cfg, out_dir)
    models_dir = os.path.join(os.path.dirname(out_dir.rstrip('/')), 'models')   # worlds 폴더의 형제 models/
    model = generate_model(px4_model or default_px4_model(), rover_yaml or default_rover_yaml(),
                           models_dir, cfg.rover_name, segmentation=seg_camera, camera_lite=camera_lite)
    return world, meta, model, models_dir


def main(argv=None):
    """generate_orchard 콘솔 진입점: 생성 후 경로를 출력한다."""
    argv = sys.argv[1:] if argv is None else argv
    world, meta, model, models_dir = generate_all(argv)
    print(f'world : {world}\nmeta  : {meta}\nmodel : {model}\nGZ_SIM_RESOURCE_PATH 에 추가: {models_dir}')


if __name__ == '__main__':
    main()
