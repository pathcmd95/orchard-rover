"""과수원 지형: 높이 함수, 지면 구역(풀/맨흙/바퀴자국/자갈길), 지면 텍스처, 지형 메시.

높이 = 전체 경사 + 완만한 기복(수십 m) + 중간 요철(1~3 m) + 미세 거칠기
       + 나무 열 아래 두둑(배수용 높은 이랑) − 통로 바퀴자국 + 과수원 밖 언덕
모든 함수는 numpy 배열을 받아 벡터 계산하므로 텍스처·풀·자갈 배치·나무 높이가 같은 값을 쓴다.

좌표·단위
  월드 좌표 (x = 행 방향, y = 행과 수직, z = 위) [m]. 입력 x, y 는 스칼라 또는 같은 모양의 배열,
  출력도 같은 모양 (height [m], zones 의 각 값 0~1, normal 은 마지막 축 3).
생성 파일 (write_ground_assets)
  terrain_core.obj (정밀 영역, 충돌용), terrain_outer.obj (바깥 거친 메시),
  terrain_core_drive.obj / terrain_core_strip.obj (보이는 면을 세그멘테이션 라벨별로 나눈 것),
  ground_core.jpg (정밀 영역 텍스처 1장), meadow.jpg (바깥 풀밭 반복 텍스처) — Pillow 없으면 .png
조정할 곳
  진폭·깊이 [m]: world_gen.OrchardConfig 의 relief, bump_amp, micro_amp, slope_x/y, ridge_height, rut_depth,
  rut_offset (relief, bump_amp 는 CLI --relief/--bump-amp, sim.launch.py relief:=, bump_amp:= 로도 바꿈).
  메시 간격 terrain_grid [m], 텍스처 해상도 texture_px_per_m. 노이즈 격자 크기(파장)는 OrchardGround.__init__.
노이즈
  ValueNoise 는 격자점에 난수 값을 두고 smoothstep 으로 보간하는 value noise 이다 (Perlin 의 gradient noise 와
  다른 방식). 여러 격자 크기의 노이즈를 가중합해 큰 기복과 작은 요철을 함께 만든다.
"""
from __future__ import annotations


import numpy as np

from .meshes import Mesh, write_image, write_obj


class ValueNoise:
    """격자 난수 + 부드러운 보간 (0~1). 영역 밖은 가장자리 값으로 고정."""

    def __init__(self, rng: np.random.Generator, cell: float, xr, yr):
        """cell: 격자 간격 [m] (≈ 특징 파장), xr/yr: 쓸 영역 (min, max) [m]. 격자 값은 rng 로 0~1 균등 난수."""
        self.cell = cell
        self.x0, self.y0 = xr[0] - cell, yr[0] - cell       # 한 칸 여유를 두고 시작
        nx = int((xr[1] - xr[0]) / cell) + 4                # 양쪽 여유 + 보간용 이웃 칸
        ny = int((yr[1] - yr[0]) / cell) + 4
        self.g = rng.random((nx, ny)).astype(np.float32)

    def __call__(self, x, y):
        """위치 (x, y) [m] 의 노이즈 값 0~1 (x, y 와 같은 모양, float32)."""
        gx = (np.asarray(x, np.float32) - self.x0) / self.cell
        gy = (np.asarray(y, np.float32) - self.y0) / self.cell
        nx, ny = self.g.shape
        gx = np.clip(gx, 0, nx - 1.001)                     # i+1 이 배열 안에 있도록 (영역 밖은 가장자리 값)
        gy = np.clip(gy, 0, ny - 1.001)
        i, j = gx.astype(np.int32), gy.astype(np.int32)
        fx, fy = gx - i, gy - j
        fx = fx * fx * (3 - 2 * fx)                         # smoothstep 3t²−2t³: 격자선에서 기울기가 끊기지 않게
        fy = fy * fy * (3 - 2 * fy)
        g = self.g
        # 네 격자점 값의 쌍선형 보간 (가중치는 smoothstep 으로 휜 fx, fy)
        a = g[i, j] * (1 - fx) + g[i + 1, j] * fx
        b = g[i, j + 1] * (1 - fx) + g[i + 1, j + 1] * fx
        return a * (1 - fy) + b * fy


def _smoothstep(e0, e1, x):
    """x 가 e0 → e1 로 갈 때 0 → 1 로 부드럽게 (e0 > e1 이면 거꾸로 1 → 0). 구역 경계를 흐리게 하는 데 쓴다."""
    t = np.clip((x - e0) / (e1 - e0), 0.0, 1.0)
    return t * t * (3 - 2 * t)


class OrchardGround:
    """지형 높이와 지면 구역. cfg 는 world_gen.OrchardConfig."""

    def __init__(self, cfg, rng: np.random.Generator):
        """정밀 영역 범위, 노이즈(격자 크기 [m] 별), 자갈 농로 x 구간을 정한다.

        rng 로 노이즈 격자를 만들므로 같은 seed → 같은 지형 (world_gen.ground_for 가 이 성질을 쓴다).
        """
        self.cfg = cfg
        L = cfg.row_length
        self.rows_y = np.array([cfg.row_y(j) for j in range(cfg.rows)])
        self.lanes_y = np.array([cfg.lane_y(k) for k in range(cfg.lanes)])
        # 정밀 영역(core): 나무·통로·양쪽 회전 공간
        self.core_x = (-cfg.headland - 2.0, L + cfg.headland + 2.0)
        self.core_y = (self.rows_y[0] - 5.0, self.rows_y[-1] + 5.0)
        # 노이즈 영역: 큰 기복·언덕은 바깥 메시(~100 m)까지, 세밀한 요철은 정밀 영역 +3 m 만
        big = (self.core_x[0] - 120, self.core_x[1] + 120), (self.core_y[0] - 120, self.core_y[1] + 120)
        near = (self.core_x[0] - 3, self.core_x[1] + 3), (self.core_y[0] - 3, self.core_y[1] + 3)
        n = lambda cell, ext: ValueNoise(rng, cell, *ext)   # noqa: E731
        # 인자 = 격자 크기 [m] (클수록 완만)
        self.n_roll = [n(30.0, big), n(14.0, big)]          # 완만한 기복
        self.n_hill = n(45.0, big)                          # 과수원 밖 언덕 높이 변화
        self.n_bump = [n(2.4, near), n(1.1, near)]          # 1~3 m 요철 (차체 흔들림)
        self.n_micro = n(0.45, near)                        # 미세 거칠기
        self.n_ridge = n(5.0, near)                         # 두둑 높이의 열 방향 변화
        self.n_rut = n(3.0, near)                           # 바퀴자국 깊이 변화
        self.n_edge = n(1.2, near)          # 구역 경계 들쭉날쭉
        self.n_patch = n(2.2, near)         # 맨흙 패치
        self.n_gravel = n(3.5, near)        # 통로 속 자갈 패치
        self.n_mow = n(4.0, near)           # 풀 길이 (깎은 곳/안 깎은 곳)
        self.n_dry = n(6.0, near)           # 마른 풀
        self.n_soil = n(0.8, near)                          # 흙 색 얼룩
        # 자갈 농로(회전 공간): 먼 쪽은 U턴 경로가 지나가도록, 가까운 쪽은 출발 지점 포함
        self.roads = [(-cfg.spawn_back - 2.6, -cfg.spawn_back + 1.0), (L + 2.2, L + 5.6)]   # (x 시작, x 끝) [m]

    # ------------------------------------------------------------ 거리
    def d_row(self, y):
        """가장 가까운 나무 열까지의 |Δy| [m]."""
        y = np.asarray(y)
        return np.min(np.abs(y[..., None] - self.rows_y), axis=-1)

    def d_lane(self, y):
        """가장 가까운 통로 중심선까지의 |Δy| [m]."""
        y = np.asarray(y)
        return np.min(np.abs(y[..., None] - self.lanes_y), axis=-1)

    def core_fade(self, x, y, margin: float = 2.5):
        """정밀 영역 가장자리로 갈수록 0 (바깥 거친 메시와 틈 없이 이어지게)."""
        fx = np.minimum(_smoothstep(self.core_x[0], self.core_x[0] + margin, x),
                        1 - _smoothstep(self.core_x[1] - margin, self.core_x[1], x))
        fy = np.minimum(_smoothstep(self.core_y[0], self.core_y[0] + margin, y),
                        1 - _smoothstep(self.core_y[1] - margin, self.core_y[1], y))
        return np.minimum(fx, fy)

    def outside_dist(self, x, y):
        """정밀 영역 사각형 밖으로 떨어진 거리 [m] (안이면 0)."""
        dx = np.maximum(np.maximum(self.core_x[0] - x, x - self.core_x[1]), 0)
        dy = np.maximum(np.maximum(self.core_y[0] - y, y - self.core_y[1]), 0)
        return np.hypot(dx, dy)

    def field_x(self, x, pad=0.6):
        """나무 열의 x 범위(양 끝 pad [m] 여유) 안이면 1, 밖이면 0 (0.8 m 에 걸쳐 부드럽게)."""
        L = self.cfg.row_length
        return _smoothstep(-pad - 0.8, -pad, x) * (1 - _smoothstep(L + pad, L + pad + 0.8, x))

    # ------------------------------------------------------------ 높이
    def height(self, x, y):
        """지면 높이 z [m] (x, y 와 같은 모양, float64). 충돌 메시·나무·풀·로봇 출발 높이가 모두 이 값을 쓴다."""
        c = self.cfg
        x = np.asarray(x, np.float64)
        y = np.asarray(y, np.float64)
        h = c.slope_x * x + c.slope_y * y                  # 배수 경사 (기울기, 무차원)
        # 노이즈 가중합(가중치 합 1) − 0.5 → −0.5~0.5, ×2×진폭 → ±relief [m]
        roll = 0.65 * self.n_roll[0](x, y) + 0.35 * self.n_roll[1](x, y) - 0.5
        h = h + 2.0 * c.relief * roll
        fade = self.core_fade(x, y)                        # 요철·두둑·바퀴자국은 정밀 영역 안에서만
        bump = 0.6 * self.n_bump[0](x, y) + 0.4 * self.n_bump[1](x, y) - 0.5
        h = h + fade * (2.0 * c.bump_amp * bump + 2.0 * c.micro_amp * (self.n_micro(x, y) - 0.5))
        fx = self.field_x(x, pad=0.9)
        dr = self.d_row(y)
        # 두둑: 열 중심에서 가우시안 단면 (폭 척도 0.55 m), 높이는 ridge_height × 0.75~1.25
        ridge = c.ridge_height * (0.75 + 0.5 * self.n_ridge(x, y)) * np.exp(-(dr / 0.55) ** 2)
        h = h + fade * fx * ridge
        dl = self.d_lane(y)
        # 바퀴자국: 통로 중심 ±rut_offset 에 폭 척도 0.12 m 가우시안 홈, 깊이 rut_depth × 0.5~1.5.
        # 회전 공간까지 이어지도록 열 끝에서 2.5 m 더 연장
        rut = c.rut_depth * (0.5 + self.n_rut(x, y)) * np.exp(-((dl - c.rut_offset) / 0.12) ** 2)
        h = h - fade * self.field_x(x, pad=2.5) * rut
        d_out = self.outside_dist(x, y)
        # 바깥 언덕: 정밀 영역에서 4 m → 60 m 멀어지는 동안 서서히 1~8 m 까지 솟는다 (배경, 로봇은 안 감)
        hill = _smoothstep(4.0, 60.0, d_out) * (1.0 + 7.0 * self.n_hill(x, y))
        return (h + hill).astype(np.float64)

    def normal(self, x, y, eps=0.05):
        """지면 단위 법선 (..., 3). 높이의 중앙 차분(간격 eps [m])으로 기울기를 구해 (−∂h/∂x, −∂h/∂y, 1) 정규화."""
        hx = (self.height(x + eps, y) - self.height(x - eps, y)) / (2 * eps)
        hy = (self.height(x, y + eps) - self.height(x, y - eps)) / (2 * eps)
        n = np.stack([-hx, -hy, np.ones_like(hx)], axis=-1)
        return n / np.linalg.norm(n, axis=-1, keepdims=True)

    # ------------------------------------------------------------ 구역
    def zones(self, x, y) -> dict:
        """각 점의 지면 구역 비율(0~1).

        반환 dict (모두 x, y 와 같은 모양): road 자갈 농로, strip 나무 밑 제초 띠, rut 바퀴자국, gravel 통로 속 자갈 패치,
        bare 맨흙 패치, grass 풀 덮임, mow 풀 길이, dry 마른 정도, soil 흙 색 (마지막 셋은 원래 노이즈 값).
        구역 경계에 n_edge 노이즈를 더해 직선이 아니라 들쭉날쭉하게 만든다.
        """
        c = self.cfg
        x = np.asarray(x, np.float32)
        y = np.asarray(y, np.float32)
        edge = self.n_edge(x, y) - 0.5
        # 과수원 y 범위(바깥 열에서 1.4~2.2 m 까지) 안이면 1
        in_rows_y = _smoothstep(self.rows_y[0] - 2.2, self.rows_y[0] - 1.4, y) * \
            (1 - _smoothstep(self.rows_y[-1] + 1.4, self.rows_y[-1] + 2.2, y))
        road = np.zeros_like(x)
        for a, b in self.roads:
            road = np.maximum(road, _smoothstep(a - 0.25, a + 0.25, x + 0.6 * edge)
                              * (1 - _smoothstep(b - 0.25, b + 0.25, x + 0.6 * edge)))
        road = road * _smoothstep(self.rows_y[0] - 4.0, self.rows_y[0] - 3.2, y) * \
            (1 - _smoothstep(self.rows_y[-1] + 3.2, self.rows_y[-1] + 4.0, y))
        fx = self.field_x(x, pad=0.5)
        dr = self.d_row(y)
        strip = fx * in_rows_y * (1 - _smoothstep(0.42, 0.62, dr + 0.35 * edge))    # 제초 띠(나무 아래 맨흙)
        dl = self.d_lane(y)
        rut = self.field_x(x, pad=2.5) * in_rows_y * \
            (1 - _smoothstep(0.08, 0.16, np.abs(dl - c.rut_offset) + 0.05 * edge))
        gravel = fx * in_rows_y * _smoothstep(0.80, 0.86, self.n_gravel(x, y)) * (1 - strip)
        bare = _smoothstep(0.22, 0.15, self.n_patch(x, y))       # e0 > e1: 노이즈 ≤ 0.15 면 1, ≥ 0.22 면 0 (작은 곳이 맨흙)
        road = np.clip(road * (1 - strip), 0, 1)
        grass = np.clip(1 - strip * 0.9 - road - gravel, 0, 1) * (1 - 0.7 * rut) * (1 - 0.8 * bare)
        return {'road': road, 'strip': strip, 'rut': rut, 'gravel': gravel, 'bare': bare,
                'grass': grass, 'mow': self.n_mow(x, y), 'dry': self.n_dry(x, y), 'soil': self.n_soil(x, y)}

    # ------------------------------------------------------------ 텍스처
    def paint_core(self, px_per_m: float, rng: np.random.Generator) -> np.ndarray:
        """정밀 영역 지면 텍스처 (HxWx3, 0~1). 첫 행 = y 최대(텍스처 위쪽).

        px_per_m: 해상도 [픽셀/m] (크기는 4096 px 로 제한, GPU 텍스처 한계·메모리 고려).
        zones() 로 구한 구역 비율로 흙·풀·자갈 색을 섞는다. 색 값은 sRGB 0~1.
        """
        (x0, x1), (y0, y1) = self.core_x, self.core_y
        W = min(4096, int((x1 - x0) * px_per_m))
        H = min(4096, int((y1 - y0) * px_per_m))
        xs = x0 + (np.arange(W, dtype=np.float32) + 0.5) * (x1 - x0) / W   # 픽셀 중심의 월드 x
        ys = y1 - (np.arange(H, dtype=np.float32) + 0.5) * (y1 - y0) / H   # 위(y 최대)에서 아래로
        X, Y = np.meshgrid(xs, ys)
        z = self.zones(X, Y)
        speck = rng.random((H, W), dtype=np.float32)       # 픽셀 단위 얼룩 (알갱이·잎 결)
        speck2 = rng.random((H, W), dtype=np.float32)
        col = lambda *c: np.array(c, np.float32)  # noqa: E731
        # 흙: 밝은 황토 ~ 짙은 갈색, 미세 알갱이
        soil = (col(0.42, 0.33, 0.23) * (1 - z['soil'])[..., None] + col(0.30, 0.23, 0.16) * z['soil'][..., None])
        soil = soil * (0.88 + 0.24 * speck)[..., None]
        # 바퀴자국: 다져진 짙은 흙
        soil = soil * (1 - 0.18 * z['rut'])[..., None]
        # 제초 띠 위 낙엽
        litter = (speck2 > 0.93) & (z['strip'] > 0.5)
        soil[litter] = soil[litter] * 0.6 + col(0.45, 0.30, 0.12) * 0.4
        # 풀: 초록 ~ 연두 ~ 마른 풀, 잎 결을 흉내 낸 고주파 얼룩
        g1, g2, gd = col(0.20, 0.38, 0.10), col(0.33, 0.50, 0.15), col(0.56, 0.52, 0.30)
        mix = np.clip(speck2 * 0.6 + z['mow'] * 0.4, 0, 1)[..., None]
        grass = g1 * (1 - mix) + g2 * mix
        dry = _smoothstep(0.62, 0.85, z['dry'])[..., None]
        grass = grass * (1 - dry * 0.7) + gd * dry * 0.7
        grass = grass * (0.72 + 0.5 * speck)[..., None]
        # 풀 덮임: 픽셀 단위로 흙이 비치게 (듬성듬성한 잔디)
        cover = np.clip(z['grass'] * 1.25 - 0.1 + 0.35 * (speck2 - 0.5), 0, 1)[..., None]
        img = soil * (1 - cover) + grass * cover
        # 자갈: 회색·갈색 알갱이와 어두운 틈
        pebble = self._pebble_field(X, Y, rng)
        gcol = col(0.56, 0.54, 0.50) * (0.7 + 0.45 * pebble)[..., None]
        gcol = gcol + (col(0.08, 0.04, 0.0) * (speck > 0.7)[..., None])
        gmask = np.clip(z['road'] + z['gravel'], 0, 1)[..., None]
        img = img * (1 - gmask) + gcol * gmask
        # 가장자리는 바깥 풀밭 색으로 자연스럽게 이어지게
        edge = (1 - self.core_fade(X, Y, margin=5.0))[..., None]
        meadow = (col(0.24, 0.40, 0.11) * 0.5 + col(0.34, 0.49, 0.15) * 0.5) * (0.75 + 0.45 * speck)[..., None]
        img = img * (1 - edge) + meadow * edge
        return np.clip(img, 0, 1)

    @staticmethod
    def _pebble_field(X, Y, rng):
        """1~4 cm 조약돌 무늬 (서로 다른 크기의 격자 난수를 겹쳐 둥근 알갱이처럼).

        격자 칸마다 공간 해시로 밝기를 정하므로 큰 난수 배열 없이 4096² 픽셀에도 빠르다.
        해시 소수 73856093, 19349663 은 Teschner et al., "Optimized Spatial Hashing for Collision Detection of
        Deformable Objects", VMV 2003 의 공간 해시에서 쓰는 값.
        """
        out = np.zeros_like(X)
        for cell, w in ((0.035, 0.5), (0.018, 0.35), (0.009, 0.15)):     # (알갱이 크기 [m], 가중치)
            gx = np.floor(X / cell).astype(np.int64)
            gy = np.floor(Y / cell).astype(np.int64)
            h = (gx * 73856093 ^ gy * 19349663 ^ int(rng.integers(1 << 30))) & 0xffff   # 칸 번호 → 16비트 해시
            v = (h / 65535.0).astype(np.float32)                # 칸별 밝기 0~1
            fx = X / cell - gx - 0.5                             # 칸 중심 기준 위치 (−0.5~0.5 칸)
            fy = Y / cell - gy - 0.5
            rim = np.clip(1 - 2.2 * np.hypot(fx, fy), 0, 1)       # 알갱이 가장자리 어둡게
            out += w * (0.35 + 0.65 * v) * (0.55 + 0.45 * rim)
        return out

    @staticmethod
    def paint_meadow(size: int, rng: np.random.Generator) -> np.ndarray:
        """과수원 바깥 풀밭 반복 텍스처 (이음매 없이 반복되도록 고주파 얼룩만).

        size: 한 변 픽셀 수 (8 의 배수). 반환 (size, size, 3), 0~1. 메시 UV 로 4 m 마다 반복된다.
        """
        s = rng.random((size, size), dtype=np.float32)
        s2 = rng.random((size // 8, size // 8), dtype=np.float32)
        s2 = np.kron(s2, np.ones((8, 8), np.float32))                    # 작은 덩어리 얼룩
        g1 = np.array([0.24, 0.40, 0.11], np.float32)
        g2 = np.array([0.34, 0.49, 0.15], np.float32)
        img = g1 * (1 - s2[..., None]) + g2 * s2[..., None]
        return np.clip(img * (0.75 + 0.45 * s)[..., None], 0, 1)

    # ------------------------------------------------------------ 메시
    def build_meshes(self, grid: float) -> tuple[Mesh, Mesh]:
        """(정밀 영역 메시, 바깥 메시). 경계 정점을 공유해 틈이 없다.

        grid: 정밀 영역 격자 간격 [m] (OrchardConfig.terrain_grid). 하나의 직사각 격자(높이장, heightfield)를
        만들고 각 칸을 삼각형 2개로 나눈 뒤, 칸이 정밀 영역 안인지로 두 메시에 나눠 담는다.
        정밀 영역 UV 는 0~1 (ground_core 텍스처 1장), 바깥은 월드 좌표 / 4 m (meadow 반복).
        """
        (x0, x1), (y0, y1) = self.core_x, self.core_y
        xs_core = np.linspace(x0, x1, int(round((x1 - x0) / grid)) + 1)
        ys_core = np.linspace(y0, y1, int(round((y1 - y0) / grid)) + 1)
        steps = np.cumsum(np.linspace(2.0, 12.0, 14))            # 바깥으로 갈수록 성긴 격자 (~100 m)
        xs = np.concatenate([x0 - steps[::-1], xs_core, x1 + steps])
        ys = np.concatenate([y0 - steps[::-1], ys_core, y1 + steps])
        X, Y = np.meshgrid(xs, ys, indexing='ij')           # X[i, j] = xs[i], Y[i, j] = ys[j]
        Z = self.height(X, Y)
        V = np.stack([X, Y, Z], axis=-1).reshape(-1, 3)
        nx, ny = len(xs), len(ys)
        idx = np.arange(nx * ny).reshape(nx, ny)            # 격자점 (i, j) 의 정점 번호
        # 칸 (i, j) 의 네 모서리: a(i, j) → b(i+1, j) → c(i+1, j+1) → d(i, j+1) 는 위(+z)에서 보면 반시계
        a, b = idx[:-1, :-1].ravel(), idx[1:, :-1].ravel()
        c, d = idx[1:, 1:].ravel(), idx[:-1, 1:].ravel()
        # 칸 전체가 정밀 영역 사각형 안에 있는지 (1e-6 은 부동소수 오차 여유)
        cx = ((X[:-1, :-1] >= x0 - 1e-6) & (X[1:, :-1] <= x1 + 1e-6)).ravel()
        cy = ((Y[:-1, :-1] >= y0 - 1e-6) & (Y[:-1, 1:] <= y1 + 1e-6)).ravel()
        in_core = cx & cy
        core, outer = Mesh(), Mesh()
        uv_core = np.stack([(V[:, 0] - x0) / (x1 - x0), (V[:, 1] - y0) / (y1 - y0)], axis=1)
        uv_outer = V[:, :2] / 4.0                                   # 4 m 마다 반복
        for m, sel, uv in ((core, in_core, uv_core), (outer, ~in_core, uv_outer)):
            # 사각형 abcd → 삼각형 (a, b, c), (a, c, d). 법선이 +z 를 향한다
            f = np.concatenate([np.stack([a[sel], b[sel], c[sel]], 1), np.stack([a[sel], c[sel], d[sel]], 1)])
            used = np.unique(f)
            remap = np.full(len(V), -1)
            remap[used] = np.arange(len(used))
            m.add(V[used], remap[f], uv[used])
        return core, outer


def write_ground_assets(ground: OrchardGround, out_dir: str, grid: float, px_per_m: float,
                        rng: np.random.Generator) -> dict:
    """지형 OBJ(충돌용 전체, 보이는 면: 주행가능/나무 밑 띠/바깥)와 텍스처 2장을 쓰고 경로 dict 반환.
    보이는 면을 나눠 두면 세그멘테이션 카메라가 '주행가능 지면'과 '나무 밑 띠'를 다른 라벨로 본다."""
    import os
    core, outer = ground.build_meshes(grid)
    paths = {'terrain_core': os.path.join(out_dir, 'terrain_core.obj'),
             'terrain_outer': os.path.join(out_dir, 'terrain_outer.obj'),
             'terrain_drive': os.path.join(out_dir, 'terrain_core_drive.obj'),
             'terrain_strip': os.path.join(out_dir, 'terrain_core_strip.obj')}
    write_obj(paths['terrain_core'], core)
    write_obj(paths['terrain_outer'], outer)
    v, f, _ = core.arrays()
    c = v[f].mean(axis=1)                                   # 삼각형 무게중심 (M, 3)
    strip = ground.zones(c[:, 0], c[:, 1])['strip'] > 0.5
    drive_m, strip_m = core.split(~strip)
    write_obj(paths['terrain_drive'], drive_m)
    write_obj(paths['terrain_strip'], strip_m)
    paths['ground_tex'] = write_image(os.path.join(out_dir, 'ground_core.jpg'),
                                      ground.paint_core(px_per_m, rng))
    paths['meadow_tex'] = write_image(os.path.join(out_dir, 'meadow.jpg'), ground.paint_meadow(512, rng))
    paths['tris'] = core.n_tris + outer.n_tris
    return paths
