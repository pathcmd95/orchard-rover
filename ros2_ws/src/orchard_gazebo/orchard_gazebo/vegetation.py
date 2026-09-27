"""사과나무(밀식 세장방추형), 지주·철선·점적관수 호스, 풀·자갈·바위·낙과, 먼 숲을 절차적으로 만든다.

울산 애플팜처럼 M9 왜성대목 사과 밀식 과원을 기준으로 했다.
  - 나무: 높이 3.0~3.5 m, 줄기 지름 9~14 cm, 접목부 혹, 첫 가지 0.8~1.0 m (그 아래는 줄기만)
  - 나무마다 쇠파이프 지주, 8그루마다 콘크리트 기둥 + 철선 3줄, 나무 밑 점적관수 호스
  - 통로: 불규칙한 잔디(깎은 곳/웃자란 곳/마른 곳), 트랙터 바퀴자국, 맨흙, 자갈 패치
좌표: 나무 한 그루는 원점(줄기 밑동)에서 +z 로 선다.

출력 (모두 meshes.Mesh, 월드 좌표 [m])
  - apple_tree(): 재질별 메시 dict {'bark', 'leaf_a', 'leaf_b', 'apple'} + 정보 dict (나무 모양 변형 1개)
  - RowBuilder.build(): 열 하나의 재질별 메시 + 충돌체 목록 ('cyl' 줄기, 'box' 기둥)
  - ground_cover(): 풀·자갈·바위·낙과 메시 + 바위 충돌 구 목록
  - distant_forest(): 배경 숲 메시 (충돌 없음)
  재질 키 이름은 world_gen.MATERIALS / LABEL_OF (색·세그멘테이션 라벨) 과 맞아야 한다.
조정할 곳
  잎·사과 수, 나무 변형 수, 풀·자갈 밀도, 바위 수, 결주율, 숲 나무 수 → world_gen.OrchardConfig (--lite 로 줄임).
  나무 치수(높이, 줄기 반지름, 첫 가지 높이 등)는 apple_tree() 안의 rng.uniform 범위.
  LiDAR 줄기 검출(orchard_perception lidar_tree_node 의 sim.yaml trunk.band_min/max 0.25~0.7 m)이 줄기만 보도록
  첫 가지·잎은 z_first(0.8 m 이상) 위에만 둔다 — 이 값을 낮추면 인식 성능이 달라진다.
"""
from __future__ import annotations

import math

import numpy as np

from .meshes import Mesh, blob, box, grass_blade, leaf_shape, rot_from_to, rot_z, tube

GOLDEN = math.pi * (3 - 5 ** 0.5)          # 황금각 ≈ 2.400 rad (137.5°): 곁가지 방위가 겹치지 않게 돌린다 (잎차례처럼)


def apple_tree(rng: np.random.Generator, leaves: int = 400, apples: int = 18,
               clump_every: float = 1.0) -> tuple[dict, dict]:
    """나무 한 그루 메시 (재질별 Mesh dict) 와 정보 dict(줄기 반지름 등) 반환.

    leaves: 낱장 잎 수 (대략), apples: 사과 수, clump_every: 가지 위치 몇 개마다 안쪽 잎 뭉치 1개.
    정보 dict: height [m], trunk_radius (z = 0.5 m 에서의 줄기 반지름 [m], 충돌 원기둥·지도 정답에 사용),
    first_branch [m]. 원점 = 줄기 밑동, +z = 위.
    """
    H = rng.uniform(3.0, 3.5)
    r0 = rng.uniform(0.045, 0.07)                # 지면 위 30 cm 줄기 반지름
    z_graft = rng.uniform(0.18, 0.30)             # 접목부(대목과 품종 이음매) 높이 [m]
    z_first = rng.uniform(0.80, 0.98)             # 첫 가지 높이 (LiDAR 줄기 밴드 0.25~0.7 m 위)
    # 중심 줄기: 0.9 m 까지는 거의 곧게, 위로는 조금씩 휘어짐
    zs = np.concatenate([np.linspace(-0.05, 0.9, 9), np.linspace(1.05, H, 12)])   # 중심선 높이 샘플 [m] (땅속 5 cm 부터)
    lean = rng.normal(0, 0.012, 2)                # 줄기 전체 기울기 (x, y 방향, 높이 1 m 당 [m])
    # 0.9 m 위로만 무작위 걸음(random walk)으로 휘게: 아래 줄기는 곧아야 LiDAR 원 맞춤이 잘 된다
    walk = np.cumsum(rng.normal(0, 0.025, (len(zs), 2)), axis=0) * (zs[:, None] > 0.9)
    pts = np.column_stack([lean[0] * zs + walk[:, 0], lean[1] * zs + walk[:, 1], zs])

    def trunk_r(z):
        """높이 z [m] 에서의 줄기 반지름 [m] = r0 × 밑동 퍼짐 × 접목부 혹 × 위로 가늘어짐."""
        z = np.asarray(z, float)
        flare = 1 + 0.45 * np.exp(-np.maximum(z, 0) / 0.07)        # 지면 부근 최대 +45 %, 7 cm 척도로 감소
        graft = 1 + 0.14 * np.exp(-((z - z_graft) / 0.04) ** 2)   # 접목부 혹 +14 % (폭 척도 4 cm)
        taper = np.clip(1 - 0.78 * (z - 0.3) / (H - 0.3), 0.2, 1.08)   # 0.3 m 에서 1, 꼭대기에서 0.22
        return r0 * flare * graft * taper

    bark = Mesh()
    bark.add(*tube(pts, trunk_r(zs), sides=8))
    leaf_a, leaf_b, fruit = Mesh(), Mesh(), Mesh()
    shoots = []                                   # 잎이 붙을 가지 위치 (점, 방향, 세기)

    # 곁가지: 아래는 길고(1.0~1.2 m) 위로 갈수록 짧은 원뿔형. 열 방향(x)으로 조금 더 길게 (통로 쪽은 전정)
    n_br = int(rng.integers(15, 22))
    az0 = rng.uniform(0, 2 * math.pi)
    L_max = rng.uniform(0.95, 1.2)
    for k in range(n_br):
        # 가지 높이: 첫 가지 ~ 꼭대기 35 cm 아래까지 거의 고르게 (지수 0.9 로 아래쪽을 약간 촘촘히)
        z = z_first + (H - 0.35 - z_first) * (k / (n_br - 1)) ** 0.9 + rng.normal(0, 0.03)
        az = az0 + k * GOLDEN + rng.normal(0, 0.25)
        frac = (z - z_first) / (H - z_first)
        length = (L_max * (1 - frac) ** 0.85 + 0.18) * rng.uniform(0.8, 1.15)
        # 통로 쪽 짧게: 반축 (1.0, 0.78) 타원의 반지름 → y(통로) 방향 가지는 최대 78 % 길이
        length *= 1.0 / math.sqrt((math.cos(az) / 1.0) ** 2 + (math.sin(az) / 0.78) ** 2)   # 통로 쪽 짧게
        up = rng.uniform(0.15, 0.55)             # 처음 올라가는 각도 [rad]
        droop = rng.uniform(0.25, 0.55) * (1 - 0.6 * frac)   # 열매 무게로 처짐
        base = np.array([np.interp(z, zs, pts[:, 0]), np.interp(z, zs, pts[:, 1]), z])
        t = np.linspace(0, 1, 6)
        d = np.array([math.cos(az), math.sin(az), 0.0])
        # 가지 중심선: 수평으로 t·length, 높이는 포물선 length·(sin(up)·t − droop·t²) → 올라갔다 끝이 처진다
        bp = base + np.outer(t * length, d) + np.outer(length * (math.sin(up) * t - droop * t ** 2), [0, 0, 1])
        bp[:, 2] = np.maximum(bp[:, 2], z_first - 0.06)   # 처진 가지도 첫 가지 높이 −6 cm 아래로는 안 내려가게
        rb = max(0.012, float(trunk_r(z)) * 0.42)         # 가지 밑동 반지름 = 줄기의 42 % (최소 1.2 cm)
        bark.add(*tube(bp, rb * (1 - 0.8 * t), sides=5))
        for tt in np.linspace(0.25, 1.0, max(3, int(length * 9))):   # 가지 1 m 당 약 9 곳에 잎 자리
            i = min(int(tt * 5), 4)                        # 6 점 중심선의 몇 번째 구간인지 (5 구간)
            p = bp[i] + (bp[i + 1] - bp[i]) * (tt * 5 - i)   # 그 구간 안 선형 보간
            shoots.append((p, d, 1.0 - 0.4 * tt))
    for tt in np.linspace(0.05, 1.0, 10):         # 중심 줄기 윗부분에도 잎
        z = 1.0 + (H - 1.0) * tt
        p = np.array([np.interp(z, zs, pts[:, 0]), np.interp(z, zs, pts[:, 1]), z])
        a = rng.uniform(0, 2 * math.pi)
        shoots.append((p, np.array([math.cos(a), math.sin(a), 0.3]), 0.8))

    # 잎: 안쪽의 짙은 잎 뭉치(속이 비쳐 보이지 않게) + 바깥의 낱장 잎 수백~천여 장
    step = max(1, int(round(clump_every)))
    for n, (p, d, w) in enumerate(shoots):
        if n % step:
            continue
        r = rng.uniform(0.07, 0.12) * (0.7 + 0.3 * w)
        c = p + d * r * 0.3 + rng.normal(0, 0.03, 3)
        c[2] = max(c[2], z_first + 0.6 * r)          # 수관 아래는 줄기만 보이게
        v, f = blob(rng, r, squash=(1.0, rng.uniform(0.75, 1.0), rng.uniform(0.7, 0.9)), roughness=0.35,
                    subdiv=0)
        leaf_a.add(v @ rot_z(rng.uniform(0, 6.3)).T + c, f)
    per = max(1, leaves // max(1, len(shoots)))   # 잎 자리 하나당 낱장 잎 수
    for p, d, w in shoots:
        for _ in range(per):
            L = rng.uniform(0.065, 0.10)
            v, f = leaf_shape(L, L * rng.uniform(0.5, 0.65))
            # 잎 방향: 가지 방향 + 큰 무작위 + 약간 위로. 잎(+x)을 그 방향으로 돌리고 잎맥 축으로 ±1.2 rad 비튼다
            dir_ = d * rng.uniform(0.2, 0.8) + rng.normal(0, 0.55, 3) + np.array([0, 0, 0.25])
            R = rot_from_to([1, 0, 0], dir_) @ _rot_x(rng.uniform(-1.2, 1.2))
            off = rng.normal(0, 0.10 * w + 0.05, 3)
            off[2] = max(off[2], z_first - p[2])     # 잎도 첫 가지 높이 아래로는 두지 않는다 (줄기 밴드 비우기)
            (leaf_a if rng.random() < 0.35 else leaf_b).add(v @ R.T + p + off, f)
    # 사과: 반지름 3.5~4.5 cm, 가지 아래 3~8 cm 에 매달림
    low = [s for s in shoots if s[0][2] < H - 0.6]   # 꼭대기 60 cm 에는 사과를 달지 않는다
    for _ in range(apples if low else 0):
        p, d, w = low[int(rng.integers(len(low)))]
        r = rng.uniform(0.035, 0.045)
        v, f = blob(rng, r, squash=(1.0, 1.0, 0.88), roughness=0.04, subdiv=0)
        fruit.add(v + p + rng.normal(0, 0.05, 3) - [0, 0, r + rng.uniform(0.03, 0.08)], f)

    info = {'height': round(H, 3), 'trunk_radius': round(float(trunk_r(0.5)), 4),
            'first_branch': round(z_first, 3)}
    return {'bark': bark, 'leaf_a': leaf_a, 'leaf_b': leaf_b, 'apple': fruit}, info


def _rot_x(a):
    """x 축 둘레 a [rad] 회전행렬."""
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


class RowBuilder:
    """열 하나(나무 + 지주 + 철선 + 호스)를 재질별 메시로 합친다. 나무 모양은 variants 를 돌려 쓴다."""

    def __init__(self, ground, variants: list, rng: np.random.Generator):
        """ground: terrain.OrchardGround (높이), variants: apple_tree() 결과 목록, rng: 배치용 난수."""
        self.ground = ground
        self.variants = variants
        self.rng = rng

    def build(self, cfg, j: int, trees_out: list) -> tuple[dict, list]:
        """j 번째 나무 열을 만든다.

        나무는 x = 0, tree_spacing, ... 에 (결주 확률 missing_tree_prob 로 빠짐, 위치 ±position_jitter [m]) 지면 높이에 세운다.
        trees_out 에 나무 정답 {'row', 'x', 'y', 'z', 'radius', 'variant'} 을 덧붙인다 (meta.json 의 trees).
        반환: (재질별 Mesh dict, 충돌체 목록 [('cyl', x, y, z, r, 길이) | ('box', x, y, z, sx, sy, sz)]).
        """
        rng, g = self.rng, self.ground
        y0 = cfg.row_y(j)
        m = {k: Mesh() for k in ('bark', 'leaf_a', 'leaf_b', 'apple', 'stake', 'post', 'wire', 'hose')}
        collisions = []                            # (종류, x, y, z, 크기...)
        n = int(cfg.row_length / cfg.tree_spacing) + 1   # 열 하나의 나무 자리 수 (양 끝 포함)
        post_x = []
        for i in range(n):
            x = i * cfg.tree_spacing
            if cfg.trellis_every and i % cfg.trellis_every == 0:
                post_x.append(x - cfg.tree_spacing / 2)   # 기둥은 나무와 나무 사이
            if rng.random() < cfg.missing_tree_prob:
                continue
            tx = x + rng.uniform(-cfg.position_jitter, cfg.position_jitter)
            ty = y0 + rng.uniform(-cfg.position_jitter, cfg.position_jitter)
            tz = float(g.height(tx, ty))
            vi = int(rng.integers(len(self.variants)))
            parts, info = self.variants[vi]
            s = rng.uniform(0.92, 1.08)                  # 크기 ±8 % 와 무작위 회전으로 같은 변형도 달라 보이게
            R = rot_z(rng.uniform(0, 2 * math.pi))
            for k in ('bark', 'leaf_a', 'leaf_b', 'apple'):
                m[k].add_mesh(parts[k], rot=R, trans=(tx, ty, tz - 0.03), scale=s)   # 3 cm 묻어 뜬 틈 방지
            r = info['trunk_radius'] * s
            trees_out.append({'row': j, 'x': round(tx, 3), 'y': round(ty, 3), 'z': round(tz, 3),
                              'radius': round(r, 3), 'variant': vi})
            collisions.append(('cyl', tx, ty, tz + 0.45, r, 0.9))   # 줄기 충돌: 지면~0.9 m 원기둥 (중심 높이 0.45)
            # 개별 지주(쇠파이프): 줄기 표면에서 2.5~4.5 cm 떨어져 열 방향으로
            sx = tx + rng.choice([-1, 1]) * (r * 1.2 + rng.uniform(0.025, 0.045))
            sz = float(g.height(sx, ty))
            # 지주: 땅속 0.2 m ~ 지상 2.6 m, 반지름 1.2 cm (충돌체 없음, 보이기·LiDAR 반사용)
            v, f, _ = tube(np.array([[sx, ty, sz - 0.2], [sx, ty, sz + 2.6]]), [0.012, 0.012], sides=6)
            m['stake'].add(v, f)
        # 콘크리트 기둥 (+ 열 끝은 바깥으로 기운 끝기둥)
        post_x.append(n * cfg.tree_spacing - cfg.tree_spacing / 2)
        tops = []
        for k, px in enumerate(post_x):
            pz = float(g.height(px, y0))
            tilt = 0.0
            if k == 0:                                  # 끝기둥은 철선 장력을 받게 바깥으로 약 10° (0.18 rad) 기울임
                tilt = -0.18
            elif k == len(post_x) - 1:
                tilt = 0.18
            v, f = box((0.1, 0.1, 3.4))                   # 10 cm 각, 길이 3.4 m 중 0.4 m 는 땅속
            R = _rot_y(tilt)
            v = v @ R.T + np.array([px, y0, pz + 1.7 - 0.4])   # 상자 중심 = 지면 + 3.4/2 − 0.4
            m['post'].add(v, f)
            tops.append(np.array([px + math.sin(tilt) * 1.3, y0, pz]))   # 철선 고정점 x (기울기 반영, 높이 z 는 지면)
            collisions.append(('box', px, y0, pz + 1.3, 0.1, 0.1, 2.6))   # 충돌은 기울기 없이 지상 2.6 m 상자로 근사
        # 철선 3줄 (기둥 사이를 곧게), 점적 호스 (지면을 따라)
        for hz in (1.1, 1.9, 2.7):                          # 철선 높이 [m], 반지름 3 mm
            for a, b in zip(tops[:-1], tops[1:]):
                p = np.array([a + [0, 0, hz], b + [0, 0, hz]])
                v, f, _ = tube(p, [0.003, 0.003], sides=4, cap_top=False)
                m['wire'].add(v, f)
        hx = np.arange(post_x[0], post_x[-1], 0.5)          # 호스: 0.5 m 간격 점으로 지면을 따라감
        hy = np.full_like(hx, y0 + 0.12)                    # 열 중심선에서 12 cm 옆
        hz = g.height(hx, hy) + 0.01                        # 지면 위 1 cm (반지름 8 mm)
        v, f, _ = tube(np.column_stack([hx, hy, hz]), np.full(len(hx), 0.008), sides=5, cap_top=False)
        m['hose'].add(v, f)
        return m, collisions


def _rot_y(a):
    """y 축 둘레 a [rad] 회전행렬."""
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def ground_cover(ground, cfg, rng: np.random.Generator, grass_density: float, stone_density: float,
                 trees: list) -> tuple[dict, list, list]:
    """풀(두 색), 자갈, 바위(충돌 있음), 낙과 메시. (meshes, rock_collisions, rock_meta) 반환.

    grass_density, stone_density: 시도 횟수 / m² (terrain.zones 확률로 걸러짐), trees: 낙과를 둘 나무 목록.
    rock_collisions: [(x, y, z, 반지름)] 충돌 구 [m], rock_meta: meta.json 에 넣을 바위 정답.
    메시 키 이름: grass_a, grass_b, grass_dry, stone, rock, fallen.
    """
    m = {k: Mesh() for k in ('grass_a', 'grass_b', 'grass_dry', 'stone', 'rock', 'fallen')}
    (x0, x1), (y0, y1) = ground.core_x, ground.core_y
    x0, x1, y0, y1 = x0 + 1.5, x1 - 1.5, y0 + 1.5, y1 - 1.5   # 정밀 영역 가장자리 1.5 m 는 비움
    area = (x1 - x0) * (y1 - y0)

    # --- 풀 뭉치: 구역별 확률로 걸러 불규칙하게
    n_try = int(area * grass_density)
    px = rng.uniform(x0, x1, n_try)
    py = rng.uniform(y0, y1, n_try)
    z = ground.zones(px, py)
    keep = rng.random(n_try) < np.clip(z['grass'], 0, 1) ** 1.5   # 지수 1.5: 풀이 드문 곳은 더 성기게
    px, py = px[keep], py[keep]
    mow, dry = z['mow'][keep], z['dry'][keep]
    rut = z['rut'][keep]
    pz = ground.height(px, py)
    dlane = ground.d_lane(py)
    for i in range(len(px)):
        # 풀 길이: 바퀴자국 3~6 cm, 깎은 곳 6~12 cm, 웃자란 곳 12~22 cm,
        # 가끔 잡초 25~35 cm (예초기가 덜 닿는 나무 쪽 가장자리에만. 통로 가운데는 트랙터가 눌러 낮음)
        if rng.random() < 0.03 and dlane[i] > 0.95:
            hmax = rng.uniform(0.25, 0.35)
        else:
            hmax = (0.05 + 0.15 * mow[i]) * (1 - 0.6 * rut[i]) + 0.02
        key = 'grass_dry' if dry[i] > 0.72 and rng.random() < 0.7 else ('grass_a' if rng.random() < 0.5 else 'grass_b')
        for _ in range(int(rng.integers(6, 14))):          # 뭉치 하나 = 풀잎 6~13장 (σ 3 cm 로 흩뿌림)
            v, f = grass_blade(rng, hmax * rng.uniform(0.5, 1.0), rng.uniform(0.006, 0.012))
            m[key].add(v + [px[i] + rng.normal(0, 0.03), py[i] + rng.normal(0, 0.03), pz[i] - 0.005], f)

    # --- 자갈: 자갈 구역에 촘촘히, 통로 곳곳에 드문드문
    n_try = int(area * stone_density)
    sx = rng.uniform(x0, x1, n_try)
    sy = rng.uniform(y0, y1, n_try)
    z = ground.zones(sx, sy)
    p = np.clip(z['road'] + z['gravel'], 0, 1) * 0.9 + 0.02   # 자갈 구역 92 %, 그 밖 2 % 만 남김
    keep = rng.random(n_try) < p
    sx, sy = sx[keep], sy[keep]
    sz = ground.height(sx, sy)
    for i in range(len(sx)):
        r = rng.uniform(0.012, 0.035) if rng.random() < 0.85 else rng.uniform(0.035, 0.06)   # 반지름 [m]: 85 % 잔자갈
        v, f = blob(rng, r, squash=(1.0, rng.uniform(0.6, 1.0), rng.uniform(0.4, 0.7)), roughness=0.3, subdiv=0)
        m['stone'].add(v @ rot_z(rng.uniform(0, 6.3)).T + [sx[i], sy[i], sz[i] - 0.3 * r], f)   # 중심을 0.3 r 아래로 (살짝 묻힘)

    # --- 바위 (차체를 흔드는 요철, 충돌 있음): 통로와 농로에 드문드문
    rocks, rock_meta = [], []
    for _ in range(cfg.rocks):
        k = int(rng.integers(max(1, cfg.lanes)))
        if rng.random() < 0.3:
            rx = rng.uniform(*ground.roads[int(rng.integers(2))])
        else:
            rx = rng.uniform(1.0, cfg.row_length - 1.0)
        ry = cfg.lane_y(k) + rng.uniform(-1.0, 1.0)
        rr = rng.uniform(0.07, 0.13)                        # 바위 반지름 [m]
        rz = float(ground.height(rx, ry)) - 0.55 * rr       # 중심을 0.55 r 아래로 묻어 낮은 턱이 되게
        v, f = blob(rng, rr, squash=(1.2, 1.0, 0.8), roughness=0.22, subdiv=1)
        m['rock'].add(v + [rx, ry, rz], f)
        rocks.append((rx, ry, rz, rr * 0.9))                # 충돌 구는 보이는 메시보다 10 % 작게 (울퉁불퉁한 면 안쪽)
        rock_meta.append({'x': round(rx, 2), 'y': round(ry, 2), 'r': round(rr, 3), 'lane': k})

    # --- 낙과: 나무 밑 제초 띠에 몇 개
    for t in trees:
        for _ in range(int(rng.poisson(cfg.fallen_per_tree))):
            fx = t['x'] + rng.normal(0, 0.35)
            fy = t['y'] + rng.normal(0, 0.25)
            r = rng.uniform(0.033, 0.042)
            v, f = blob(rng, r, squash=(1, 1, 0.85), roughness=0.08, subdiv=0)
            m['fallen'].add(v + [fx, fy, float(ground.height(fx, fy)) + r * 0.6], f)   # 살짝 묻힌 사과
    return m, rocks, rock_meta


def distant_forest(ground, rng: np.random.Generator, count: int) -> Mesh:
    """과수원 바깥 30~90 m 에 둘러선 숲 (배경).

    count: 나무 수. 정밀 영역에서 26~60 m 떨어진 곳에만 둔다 (거절 샘플링). 충돌 없음(보이기용).
    """
    m = Mesh()
    (x0, x1), (y0, y1) = ground.core_x, ground.core_y
    placed = 0
    while placed < count:
        x = rng.uniform(x0 - 90, x1 + 90)
        y = rng.uniform(y0 - 90, y1 + 90)
        d = float(ground.outside_dist(x, y))
        if d < 26 or d > 60:
            continue
        z = float(ground.height(x, y))
        h = rng.uniform(6, 13)                            # 나무 높이 [m]
        for _ in range(int(rng.integers(2, 4))):         # 수관 2~3 덩어리
            off = rng.normal(0, h * 0.12, 3)
            v, f = blob(rng, h * rng.uniform(0.22, 0.32), squash=(1, 1, 1.25), roughness=0.3, subdiv=1)
            m.add(v + [x + off[0], y + off[1], z + h * 0.62 + off[2]], f)
        v, f, _ = tube(np.array([[x, y, z - 0.5], [x, y, z + h * 0.4]]), [0.18, 0.12], sides=5)
        m.add(v, f)
        placed += 1
    return m
