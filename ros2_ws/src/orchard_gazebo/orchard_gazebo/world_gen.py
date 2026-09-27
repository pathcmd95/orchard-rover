"""울산 애플팜(M9 왜성 사과 밀식 과원)을 흉내 낸 Gazebo 과수원 월드 생성기.

기본값은 울산 애플팜 현장 조건을 따른다: 행간거리 3.8 m, 주간(줄기) 간격 1.2 m.
좌표: x = 행 방향(동쪽), y = 행과 수직(북쪽). 통로(lane) k 의 중심 y = k * row_spacing.
로봇은 통로 0 입구(x = -spawn_back, 자갈 농로 위)에서 +x 방향을 보고 시작한다.

사실감 요소 (terrain.py, vegetation.py)
  지형: 완만한 기복 + 1~3 m 요철 + 나무 열 두둑 + 통로 바퀴자국 + 바깥 언덕 (충돌 메시)
  지면: 불규칙한 잔디(깎은 곳/웃자란 곳/마른 곳/잡초), 맨흙, 제초 띠, 자갈 농로·자갈 패치, 바위, 낙과
  나무: 절차적 사과나무(접목부, 처진 곁가지, 잎 수백 장, 사과), 개별 지주, 콘크리트 기둥 + 철선, 점적 호스
무거운 메시·텍스처는 설정값 해시별로 캐시(assets_<해시>/)해 두 번째 실행부터는 바로 뜬다.

생성 파일 (out_dir, 기본 ~/.cache/orchard_rover/worlds)
  orchard.sdf       Gazebo 월드 (지형, 나무 열, 지면 덮개, 장애물, 로버 <include>, GPS 원점). 메시 경로는 절대 경로
  orchard_meta.json 평가·지도 정답: 통로 중심 y, 열 y, 나무·바위·장애물 위치, 출발 위치 (lane_error_monitor 등이 읽음)
  assets_<해시>/    OBJ 메시, 텍스처, scene.json (캐시)
설정 바꾸는 곳
  OrchardConfig 기본값 (아래) / CLI 옵션 (config_from_args, --help) / sim.launch.py 인자 (rows:=, row_spacing:=,
  row_length:=, obstacles:=, seed:=, relief:=, bump_amp:=, spawn_x:=, spawn_lane:=, spawn_yaw:=, lite:=).
  세그멘테이션 라벨 번호 LABELS 는 orchard_perception/seg_classes.py 와 같이 바꿔야 한다.
주의: 캐시 해시에는 이 파일과 terrain.py, vegetation.py, meshes.py 의 소스 내용이 들어가므로
  (주석 포함) 어느 파일이든 고치면 다음 실행 때 자산을 새로 만든다 (수십 초).
참고: SDFormat 1.9 명세 (sdformat.org), Gazebo Harmonic 시스템 플러그인 (Physics, Sensors, NavSat, Label 등).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import random
import time
from dataclasses import asdict, dataclass

WORLD_NAME = 'orchard'


@dataclass
class OrchardConfig:
    """과수원 월드 생성 설정. 길이 [m], 각도 [rad]. CLI(config_from_args)는 이 중 일부만 노출한다."""

    rows: int = 6                  # 나무 열 수 (통로 수 = rows - 1)
    row_spacing: float = 3.8       # 열 간격 [m]
    tree_spacing: float = 1.2      # 주간 간격 [m]
    row_length: float = 30.0       # 열 길이 [m]
    headland: float = 8.0          # 열 끝 회전 공간 [m]
    missing_tree_prob: float = 0.03     # 결주(빠진 나무) 비율
    position_jitter: float = 0.05       # 나무 위치 무작위 흔들림 ± [m]
    trellis_every: int = 8              # 콘크리트 기둥 간격(나무 수), 0 이면 없음
    # 지형 [m]
    relief: float = 0.30                # 완만한 기복 진폭 (수십 m 파장)
    bump_amp: float = 0.045             # 요철 진폭 (1~3 m 파장) → 차체 롤/피치 흔들림
    micro_amp: float = 0.01             # 미세 거칠기 진폭 (파장 약 0.5 m)
    slope_x: float = 0.012              # 배수 경사 (x 방향 1.2 %)
    slope_y: float = 0.006              # (y 방향 0.6 %)
    ridge_height: float = 0.10          # 나무 열 두둑
    rut_depth: float = 0.035            # 트랙터 바퀴자국
    rut_offset: float = 0.72            # 통로 중심에서 바퀴자국까지
    rocks: int = 14                     # 충돌 있는 바위 개수
    # 표현 밀도 (lite 는 줄인 값)
    leaves_per_tree: int = 1400         # 낱장 잎 (안쪽 잎 뭉치와 별도)
    leaf_clump_every: int = 2           # 단과지 몇 개마다 안쪽 잎 뭉치 1개
    apples_per_tree: int = 18
    tree_variants: int = 12             # 서로 다른 나무 모양 수 (돌려 쓰며 크기·방향만 바꿈)
    grass_density: float = 8.0          # 풀 뭉치 시도 횟수 / m² (구역별 확률로 걸러짐)
    stone_density: float = 12.0         # 자갈 시도 횟수 / m²
    fallen_per_tree: float = 1.0        # 낙과 평균 개수
    forest_trees: int = 260             # 배경 숲 나무 수
    terrain_grid: float = 0.2           # 지형 메시 격자 간격 [m] (작을수록 요철이 정확, 삼각형 증가)
    texture_px_per_m: float = 64.0      # 지면 텍스처 해상도 [픽셀/m] (최대 4096 px 로 제한)
    obstacles: int = 0                  # 통로 안 장애물(사람/운반차) 개수
    spawn_back: float = 3.0             # 열 시작(x = 0)에서 출발 지점까지 뒤로 [m]
    spawn_x: float | None = None        # 구간 시험용 출발 위치 (None 이면 통로 0 입구 x = -spawn_back)
    spawn_lane: int = 0                 # 출발 통로 번호 (0 ~ lanes-1 로 잘림)
    spawn_y: float | None = None        # 출발 y 를 직접 지정 [m] (과수원 밖 진입 시험용, 주면 spawn_lane 무시)
    spawn_yaw: float = 0.0              # 출발 방향 [rad] (0 = +x, 행 방향)
    latitude: float = 35.5727           # 울산(UNIST 인근), GPS 원점
    longitude: float = 129.1904
    elevation: float = 40.0             # GPS 원점 고도 [m]
    seed: int = 7                       # 난수 seed (같은 seed = 같은 과수원)
    rover_name: str = 'orchard_rover'   # Gazebo 모델 이름 (model_gen, PX4_GZ_MODEL_NAME 과 같아야 함)

    @property
    def lanes(self) -> int:
        """통로 수 = 열 수 − 1 (나무 열 사이마다 통로 하나)."""
        return self.rows - 1

    def row_y(self, j: int) -> float:
        """j 번째 나무 열의 y [m]. 열은 통로 사이에 있으므로 반 칸 어긋남: 열 0 = −spacing/2, 열 1 = +spacing/2."""
        return (j - 0.5) * self.row_spacing

    def lane_y(self, k: int) -> float:
        """k 번째 통로 중심선의 y [m] (통로 0 = y 0)."""
        return k * self.row_spacing

    def apply_lite(self):
        """저사양 PC(맥 Docker 등 소프트웨어 렌더링): 사실감은 유지하되 삼각형·텍스처를 줄인다."""
        self.leaves_per_tree = min(self.leaves_per_tree, 800)
        self.leaf_clump_every = max(self.leaf_clump_every, 3)
        self.apples_per_tree = min(self.apples_per_tree, 12)
        self.tree_variants = min(self.tree_variants, 8)
        self.grass_density = min(self.grass_density, 5.0)
        self.stone_density = min(self.stone_density, 6.0)
        self.forest_trees = min(self.forest_trees, 140)
        self.terrain_grid = max(self.terrain_grid, 0.25)
        self.texture_px_per_m = min(self.texture_px_per_m, 48.0)
        return self


# ---------------------------------------------------------------- SDF 조각
def _color(rgb, spec=0.03, double=False, texture=None, rough=0.9):
    """SDF <material> 문자열. rgb 0~1, spec 반사광 세기, double 양면 렌더링, texture 알베도 맵 경로(PBR), rough 거칠기."""
    r, g, b = rgb
    tex = ''
    if texture:
        tex = (f'<pbr><metal><albedo_map>{texture}</albedo_map><roughness>{rough}</roughness>'
               f'<metalness>0</metalness></metal></pbr>')
    ds = '<double_sided>true</double_sided>' if double else ''
    return (f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>'
            f'<specular>{spec} {spec} {spec} 1</specular>{ds}{tex}</material>')


def _material(r, g, b):
    """단색 재질 (장애물 등 단순 도형용)."""
    return _color((r, g, b), spec=0.05)


# 세그멘테이션 라벨 (Gazebo segmentation 카메라 → 학습 정답). orchard_perception/seg_classes.py 와 같아야 한다
LABELS = {'unlabeled': 0, 'drivable': 1, 'tree_strip': 2, 'tree': 3, 'structure': 4, 'obstacle': 5, 'background': 6}
LABEL_OF = {
    'bark': 'tree', 'leaf_a': 'tree', 'leaf_b': 'tree', 'apple': 'tree',
    'stake': 'structure', 'post': 'structure', 'wire': 'structure', 'hose': 'structure',
    'grass_a': 'drivable', 'grass_b': 'drivable', 'grass_dry': 'drivable', 'stone': 'drivable',
    'rock': 'obstacle', 'fallen': 'tree_strip', 'forest': 'background',
    'terrain_drive': 'drivable', 'terrain_strip': 'tree_strip', 'terrain_outer': 'drivable',
}


def _label(key: str) -> str:
    """재질 키 → gz-sim-label-system 플러그인 (세그멘테이션 카메라가 이 visual 을 해당 라벨 번호로 칠한다)."""
    return (f'<plugin filename="gz-sim-label-system" name="gz::sim::systems::Label">'
            f'<label>{LABELS[LABEL_OF[key]]}</label></plugin>')


# 재질: (색, 그림자 드리움)
MATERIALS = {
    'bark': ((0.30, 0.22, 0.15), True), 'leaf_a': ((0.12, 0.26, 0.07), True),
    'leaf_b': ((0.22, 0.42, 0.11), True), 'apple': ((0.70, 0.08, 0.06), False),
    'stake': ((0.55, 0.56, 0.58), True), 'post': ((0.62, 0.61, 0.58), True),
    'wire': ((0.45, 0.45, 0.47), False), 'hose': ((0.05, 0.05, 0.05), False),
    'grass_a': ((0.26, 0.48, 0.12), False), 'grass_b': ((0.38, 0.56, 0.16), False),
    'grass_dry': ((0.55, 0.50, 0.28), False), 'stone': ((0.52, 0.50, 0.47), False),
    'rock': ((0.46, 0.44, 0.41), True), 'fallen': ((0.55, 0.12, 0.07), False),
    'forest': ((0.13, 0.24, 0.10), True),
}


def _visual(name, pose, geom, mat, shadows=True, label=''):
    """SDF <visual> 문자열 (보이기 전용, 충돌 없음). shadows=False 면 그림자를 드리우지 않는다 (렌더링 부하↓)."""
    cs = '' if shadows else '<cast_shadows>false</cast_shadows>'
    return f'<visual name="{name}"><pose>{pose}</pose>{cs}<geometry>{geom}</geometry>{mat}{label}</visual>'


def _mesh_visual(name, path, key):
    """OBJ 메시 visual: MATERIALS[key] 의 색·그림자와 LABEL_OF[key] 라벨을 붙인다."""
    rgb, shadow = MATERIALS[key]
    return _visual(name, _p(0, 0, 0), f'<mesh><uri>{path}</uri></mesh>', _color(rgb), shadows=shadow,
                   label=_label(key))


def _collision(name, pose, geom):
    """SDF <collision> 문자열 (물리·LiDAR 충돌용 단순 도형)."""
    return f'<collision name="{name}"><pose>{pose}</pose><geometry>{geom}</geometry></collision>'


def _cyl(r, length):
    """원기둥 geometry (반지름, 길이 [m]; 축은 z)."""
    return f'<cylinder><radius>{r:.4f}</radius><length>{length:.4f}</length></cylinder>'


def _box(x, y, z):
    """상자 geometry (크기 [m])."""
    return f'<box><size>{x:.3f} {y:.3f} {z:.3f}</size></box>'


def _p(x, y, z, r=0.0, p=0.0, yaw=0.0):
    """SDF <pose> 문자열 'x y z roll pitch yaw' ([m], [rad], mm 단위까지)."""
    return f'{x:.3f} {y:.3f} {z:.3f} {r:.3f} {p:.3f} {yaw:.3f}'


# ---------------------------------------------------------------- 자산 생성 (캐시)
def _config_hash(cfg: OrchardConfig) -> str:
    """자산 캐시 폴더 이름용 해시 (SHA-1 앞 10자리): 메시에 영향을 주는 설정 + 생성 코드 내용."""
    d = asdict(cfg)
    # 장애물·GPS 원점·로버 이름·출발 위치는 SDF 에만 들어가고 메시에는 영향이 없으므로 해시에서 뺀다
    for k in ('obstacles', 'latitude', 'longitude', 'elevation', 'rover_name', 'spawn_x', 'spawn_lane', 'spawn_yaw',
              'spawn_y'):
        d.pop(k)
    h = hashlib.sha1(json.dumps(d, sort_keys=True).encode())
    here = os.path.dirname(os.path.abspath(__file__))
    for fn in ('world_gen.py', 'terrain.py', 'vegetation.py', 'meshes.py'):   # 생성 코드가 바뀌면 캐시 무효
        with open(os.path.join(here, fn), 'rb') as f:
            h.update(f.read())
    return h.hexdigest()[:10]


def build_assets(cfg: OrchardConfig, assets_dir: str, log=print) -> dict:
    """지형·나무·지면 메시를 만들고 scene dict(상대 경로, 충돌체, 나무 목록) 반환."""
    import numpy as np

    from .meshes import write_obj
    from .terrain import OrchardGround, write_ground_assets
    from .vegetation import RowBuilder, apple_tree, distant_forest, ground_cover

    t0 = time.time()
    os.makedirs(assets_dir, exist_ok=True)
    rng = np.random.default_rng(cfg.seed)
    ground = OrchardGround(cfg, rng)
    g = write_ground_assets(ground, assets_dir, cfg.terrain_grid, cfg.texture_px_per_m, rng)
    tris = g['tris']
    variants = [apple_tree(rng, cfg.leaves_per_tree, cfg.apples_per_tree, cfg.leaf_clump_every)
                for _ in range(cfg.tree_variants)]
    builder = RowBuilder(ground, variants, rng)
    scene = {'terrain_core': 'terrain_core.obj', 'terrain_outer': 'terrain_outer.obj',
             'terrain_drive': 'terrain_core_drive.obj', 'terrain_strip': 'terrain_core_strip.obj',
             'ground_tex': os.path.basename(g['ground_tex']), 'meadow_tex': os.path.basename(g['meadow_tex']),
             'rows': [], 'trees': []}
    for j in range(cfg.rows):
        meshes, cols = builder.build(cfg, j, scene['trees'])
        files = {}
        for k, m in meshes.items():
            if m.empty():
                continue
            fn = f'row{j}_{k}.obj'
            write_obj(os.path.join(assets_dir, fn), m, uv=False)
            files[k] = fn
            tris += m.n_tris
        scene['rows'].append({'files': files, 'collisions': cols})
    cover, rocks, rock_meta = ground_cover(ground, cfg, rng, cfg.grass_density, cfg.stone_density,
                                           scene['trees'])
    scene['cover'] = {}
    for k, m in cover.items():
        if m.empty():
            continue
        fn = f'cover_{k}.obj'
        write_obj(os.path.join(assets_dir, fn), m, uv=False)
        scene['cover'][k] = fn
        tris += m.n_tris
    scene['rocks'], scene['rock_meta'] = rocks, rock_meta
    forest = distant_forest(ground, rng, cfg.forest_trees)
    write_obj(os.path.join(assets_dir, 'forest.obj'), forest, uv=False)
    tris += forest.n_tris
    scene['forest'] = 'forest.obj'
    spawn_xy = (-cfg.spawn_back, cfg.lane_y(0))
    scene['spawn_z'] = float(ground.height(*spawn_xy))
    scene['triangles'] = int(tris)
    scene['core_extent'] = [list(ground.core_x), list(ground.core_y)]
    scene['roads_x'] = [list(r) for r in ground.roads]
    with open(os.path.join(assets_dir, 'scene.json'), 'w', encoding='utf-8') as f:
        json.dump(scene, f)
    log(f'[orchard] 월드 자산 생성 {time.time() - t0:.1f}s, 삼각형 {tris / 1e6:.2f}M → {assets_dir}')
    return scene


def load_or_build_assets(cfg: OrchardConfig, out_dir: str, log=print) -> tuple[dict, str]:
    """캐시(assets_<해시>/scene.json)가 있으면 읽고, 없으면 build_assets 로 만든다. (scene, assets_dir) 반환."""
    assets_dir = os.path.join(out_dir, f'assets_{_config_hash(cfg)}')
    path = os.path.join(assets_dir, 'scene.json')
    if os.path.isfile(path):
        with open(path, encoding='utf-8') as f:
            return json.load(f), assets_dir
    return build_assets(cfg, assets_dir, log), assets_dir


def build_terrain_sdf(scene: dict, adir: str) -> str:
    """충돌은 전체 지형 메시 하나, 보이는 면은 주행가능/나무 밑 띠/바깥으로 나눠 라벨을 다르게."""
    core = os.path.join(adir, scene['terrain_core'])
    mat = _color((1, 1, 1), spec=0.02, texture=os.path.join(adir, scene['ground_tex']))
    meadow = _color((1, 1, 1), spec=0.02, texture=os.path.join(adir, scene['meadow_tex']))
    vis = ''
    for key, fn, m in (('terrain_drive', scene['terrain_drive'], mat), ('terrain_strip', scene['terrain_strip'], mat),
                       ('terrain_outer', scene['terrain_outer'], meadow)):
        vis += _visual(key, _p(0, 0, 0), f'<mesh><uri>{os.path.join(adir, fn)}</uri></mesh>', m,
                       shadows=False, label=_label(key))
    return (
        '<model name="terrain"><static>true</static><link name="link">'
        f'<collision name="collision"><geometry><mesh><uri>{core}</uri></mesh></geometry>'
        '<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>'   # 마찰계수 1
        f'{vis}</link></model>')


def build_row_sdf(j: int, row: dict, adir: str) -> str:
    """나무 열 j 의 정적 모델: 재질별 메시 visual + 줄기 원기둥('cyl')·기둥 상자('box') 충돌체."""
    parts = [_mesh_visual(k, os.path.join(adir, fn), k) for k, fn in row['files'].items()]
    for n, c in enumerate(row['collisions']):
        if c[0] == 'cyl':
            _, x, y, z, r, length = c
            parts.append(_collision(f't{n}', _p(x, y, z), _cyl(r, length)))
        else:
            _, x, y, z, sx, sy, sz = c
            parts.append(_collision(f'p{n}', _p(x, y, z), _box(sx, sy, sz)))
    return f'<model name="tree_row_{j}"><static>true</static><link name="link">{"".join(parts)}</link></model>'


def build_cover_sdf(scene: dict, adir: str) -> str:
    """풀·자갈·낙과·숲 visual 과 바위 충돌 구를 담은 정적 모델 ground_cover."""
    parts = [_mesh_visual(k, os.path.join(adir, fn), k) for k, fn in scene['cover'].items()]
    for n, (x, y, z, r) in enumerate(scene['rocks']):
        parts.append(_collision(f'rock{n}', _p(x, y, z), f'<sphere><radius>{r:.3f}</radius></sphere>'))
    parts.append(_mesh_visual('forest', os.path.join(adir, scene['forest']), 'forest'))
    return f'<model name="ground_cover"><static>true</static><link name="link">{"".join(parts)}</link></model>'


def ground_for(cfg: OrchardConfig):
    """자산 생성 때와 같은 지형 함수 (같은 seed 로 다시 만든다)."""
    import numpy as np

    from .terrain import OrchardGround
    return OrchardGround(cfg, np.random.default_rng(cfg.seed))


def build_obstacles(cfg: OrchardConfig, rnd: random.Random, ground, meta: list) -> str:
    """통로 안 사람/운반차 (정지·OOD 시험용). 지면 높이에 맞춰 세운다.

    짝수 번째는 사람(키 약 1.7 m), 홀수 번째는 운반차. x 는 열 시작 6 m ~ 끝 4 m 전, y 는 통로 중심 ±0.3 m.
    meta 에 {'type', 'x', 'y', 'lane', 'yaw', 'size'(충돌 상자 길이·폭 [m])} 을 덧붙인다. 라벨은 'obstacle'.
    """
    out = []
    for n in range(cfg.obstacles):
        k = rnd.randrange(max(1, cfg.lanes))
        x = rnd.uniform(6.0, cfg.row_length - 4.0)
        y = cfg.lane_y(k) + rnd.uniform(-0.3, 0.3)
        z = float(ground.height(x, y))
        if n % 2 == 0:
            kind = 'person'
            body = (_visual('legs', _p(0, 0, 0.42), _box(0.3, 0.2, 0.84), _material(0.15, 0.17, 0.25))
                    + _visual('body', _p(0, 0, 1.12), _box(0.42, 0.26, 0.58), _material(0.2, 0.35, 0.7))
                    + _visual('head', _p(0, 0, 1.55), '<sphere><radius>0.12</radius></sphere>',
                              _material(0.85, 0.68, 0.55))
                    + _visual('hat', _p(0, 0, 1.64), _cyl(0.2, 0.03), _material(0.9, 0.85, 0.6))
                    + _collision('c', _p(0, 0, 0.8), _box(0.45, 0.3, 1.6)))
        else:
            kind = 'cart'
            body = (_visual('box', _p(0, 0, 0.5), _box(1.0, 0.6, 0.45), _material(0.1, 0.4, 0.8))
                    + _visual('crate', _p(0.1, 0, 0.82), _box(0.5, 0.4, 0.22), _material(0.6, 0.45, 0.25))
                    + _collision('c', _p(0, 0, 0.45), _box(1.0, 0.6, 0.6)))
        yaw = rnd.uniform(-0.5, 0.5)                    # (난수 순서는 예전과 같음: k, x, y, yaw)
        size = (0.45, 0.3) if kind == 'person' else (1.0, 0.6)
        meta.append({'type': kind, 'x': round(x, 2), 'y': round(y, 2), 'lane': k, 'yaw': round(yaw, 3),
                     'size': list(size)})
        lab = (f'<plugin filename="gz-sim-label-system" name="gz::sim::systems::Label">'
               f'<label>{LABELS["obstacle"]}</label></plugin>')
        out.append(f'<model name="obstacle_{n}_{kind}"><static>true</static>'
                   f'<pose>{_p(x, y, z, yaw=yaw)}</pose><link name="link">{body}</link>'
                   f'{lab}</model>')
    return ''.join(out)


# 월드 머리말: 물리 4 ms 스텝 × 250 Hz = 실시간 1배(목표), PX4 SITL 이 쓰는 센서 시스템(IMU, 기압, NavSat, 지자기),
# 렌더링 센서(LiDAR·카메라)용 Sensors(ogre2). 빛은 오전 햇살 + 하늘 보조광.
WORLD_HEADER = """<?xml version="1.0" ?>
<!-- 자동 생성 파일: orchard_gazebo/world_gen.py 로 다시 만드세요 -->
<sdf version="1.9">
  <world name="{name}">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.42 0.42 0.45 1</ambient>
      <background>0.66 0.78 0.90 1</background>
      <sky><clouds><speed>2</speed><humidity>0.35</humidity></clouds></sky>
      <shadows>true</shadows>
    </scene>
    <!-- 오전 햇살(남동쪽, 고도 약 40°) -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 40 0 0 0</pose>
      <diffuse>1.0 0.95 0.86 1</diffuse>
      <specular>0.15 0.15 0.15 1</specular>
      <direction>-0.55 0.45 -0.70</direction>
    </light>
    <light type="directional" name="sky_fill">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 40 0 0 0</pose>
      <diffuse>0.22 0.25 0.30 1</diffuse>
      <specular>0 0 0 1</specular>
      <direction>0.4 -0.3 -0.85</direction>
    </light>
"""

# 월드 꼬리말: 로버 모델 include(출발 pose) + GPS 원점(spherical_coordinates, 월드 좌표 = ENU)
WORLD_FOOTER = """    <include>
      <uri>model://{rover}</uri>
      <name>{rover}</name>
      <pose>{spawn}</pose>
    </include>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{lat}</latitude_deg>
      <longitude_deg>{lon}</longitude_deg>
      <elevation>{elev}</elevation>
    </spherical_coordinates>
  </world>
</sdf>
"""


def generate_world(cfg: OrchardConfig, out_dir: str, log=print) -> tuple[str, str]:
    """(world.sdf 경로, meta.json 경로) 반환. SDF 안의 메시·텍스처 경로는 절대 경로."""
    out_dir = os.path.abspath(os.path.expanduser(out_dir))
    os.makedirs(out_dir, exist_ok=True)
    scene, adir = load_or_build_assets(cfg, out_dir, log)
    ground = ground_for(cfg)
    rnd = random.Random(cfg.seed + 1000)          # 장애물 배치용 (자산 캐시와 무관하게 따로)
    obstacles = []
    body = [WORLD_HEADER.format(name=WORLD_NAME), build_terrain_sdf(scene, adir)]
    for j, row in enumerate(scene['rows']):
        body.append(build_row_sdf(j, row, adir))
    body.append(build_cover_sdf(scene, adir))
    body.append(build_obstacles(cfg, rnd, ground, obstacles))
    sx = -cfg.spawn_back if cfg.spawn_x is None else cfg.spawn_x
    sy = cfg.lane_y(min(max(cfg.spawn_lane, 0), cfg.lanes - 1))   # 통로 번호를 유효 범위로 자름
    if cfg.spawn_y is not None:
        sy = cfg.spawn_y
    sz = float(ground.height(sx, sy))
    spawn = _p(sx, sy, sz + 0.15, yaw=cfg.spawn_yaw)   # 지면 15 cm 위에서 떨어뜨려 바퀴가 땅에 박히지 않게
    body.append(WORLD_FOOTER.format(rover=cfg.rover_name, spawn=spawn, lat=cfg.latitude,
                                    lon=cfg.longitude, elev=cfg.elevation))
    world_path = os.path.join(out_dir, f'{WORLD_NAME}.sdf')
    with open(world_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(body))
    meta = {
        'world': WORLD_NAME,
        'config': asdict(cfg),
        'lane_centers_y': [cfg.lane_y(k) for k in range(cfg.lanes)],
        'row_y': [cfg.row_y(j) for j in range(cfg.rows)],
        'row_x_range': [0.0, cfg.row_length],
        'spawn': [sx, sy, sz],
        'trees': scene['trees'],
        'rocks': scene['rock_meta'],
        'roads_x': scene['roads_x'],
        'obstacles': obstacles,
        'assets_dir': adir,
        'triangles': scene['triangles'],
    }
    meta_path = os.path.join(out_dir, f'{WORLD_NAME}_meta.json')
    with open(meta_path, 'w', encoding='utf-8') as f:
        json.dump(meta, f, ensure_ascii=False, indent=1)
    return world_path, meta_path


def config_from_args(argv=None) -> tuple[OrchardConfig, str]:
    """CLI 인자 → (OrchardConfig, out_dir). argv None 이면 sys.argv. 기본값은 OrchardConfig 에서 가져온다."""
    ap = argparse.ArgumentParser(description='과수원 Gazebo 월드 생성')
    d = OrchardConfig()
    ap.add_argument('--out-dir', default=os.path.expanduser('~/.cache/orchard_rover/worlds'))
    ap.add_argument('--rows', type=int, default=d.rows)
    ap.add_argument('--row-spacing', type=float, default=d.row_spacing)
    ap.add_argument('--tree-spacing', type=float, default=d.tree_spacing)
    ap.add_argument('--row-length', type=float, default=d.row_length)
    ap.add_argument('--relief', type=float, default=d.relief, help='지형 기복 진폭 [m] (0 이면 평탄에 가깝게)')
    ap.add_argument('--bump-amp', type=float, default=d.bump_amp, help='요철 진폭 [m]')
    ap.add_argument('--rocks', type=int, default=d.rocks)
    ap.add_argument('--grass-density', type=float, default=d.grass_density)
    ap.add_argument('--leaves-per-tree', type=int, default=d.leaves_per_tree)
    ap.add_argument('--obstacles', type=int, default=d.obstacles)
    ap.add_argument('--missing-tree-prob', type=float, default=d.missing_tree_prob)
    ap.add_argument('--seed', type=int, default=d.seed)
    ap.add_argument('--spawn-x', type=float, default=None, help='구간 시험: 출발 x [m] (기본 통로 입구)')
    ap.add_argument('--spawn-lane', type=int, default=0)
    ap.add_argument('--spawn-y', type=float, default=None, help='구간 시험: 출발 y [m] (주면 spawn-lane 무시)')
    ap.add_argument('--spawn-yaw', type=float, default=0.0, help='[rad] 0 = +x, 3.1416 = -x')
    ap.add_argument('--lite', action='store_true', help='저사양 PC용: 잎·풀·자갈 수와 텍스처 해상도를 줄임')
    a = ap.parse_args(argv)
    cfg = OrchardConfig(rows=a.rows, row_spacing=a.row_spacing, tree_spacing=a.tree_spacing,
                        row_length=a.row_length, relief=a.relief, bump_amp=a.bump_amp, rocks=a.rocks,
                        grass_density=a.grass_density, leaves_per_tree=a.leaves_per_tree,
                        obstacles=a.obstacles, missing_tree_prob=a.missing_tree_prob, seed=a.seed,
                        spawn_x=a.spawn_x, spawn_lane=a.spawn_lane, spawn_yaw=a.spawn_yaw,
                        spawn_y=a.spawn_y)
    if a.lite:
        cfg.apply_lite()
    return cfg, a.out_dir


def main(argv=None):
    """월드만 생성 (로버 모델까지는 generate.py 의 generate_orchard)."""
    cfg, out_dir = config_from_args(argv)
    world, meta = generate_world(cfg, out_dir)
    print(f'world: {world}\nmeta : {meta}')


if __name__ == '__main__':
    main()
