"""좌/우 과수열 직선 추정과 행간중심선 계산.

모델 (base_link 기준, x 전방 / y 왼쪽):
    왼쪽 열:  y = a_L + b x
    오른쪽 열: y = a_R + b x   (두 열은 평행하다고 가정 → 기울기 b 공유)
    행간중심선: y = (a_L + a_R)/2 + b x
공유 기울기 최소제곱은 선형이므로 닫힌 해로 풀린다.

파이프라인 내 역할
  lidar_tree_node: trunks.py(줄기 위치) → [rows.py 열·중심선 추정 + 프레임간 평활화 + 통로 장애물 거리]
  → orchard_msgs/RowCenterline (/orchard/row) → orchard_navigation 의 row_navigator_node 가 추종
입력 / 출력 (ROS 의존성 없음)
  fit_rows: 줄기 xy (Nx2, base_link, m) + 가중치 → RowEstimate (offset [m], heading [rad], width [m] ...)
  RowTracker.update: 매 프레임 RowEstimate → 평활화된 RowEstimate
  corridor_obstacle_distance: 점군 Nx3 + 지면높이 + 중심선 → 통로 안 최근접 장애물 x 거리 [m]
주요 튜닝 파라미터 (lidar_tree_node 의 row.* / obstacle.* 파라미터, config/sim.yaml · robot.yaml)
  row.expected_width        과수원 행간거리 [m] (울산 애플팜 3.8). ★ 과수원마다 반드시 수정
  row.fit_x_min / fit_x_max 행 적합에 쓰는 줄기의 x 범위 [m]
  row.max_residual          열 직선에서 이보다 멀면 이상점으로 제외 [m]
  row.smoothing_alpha / row.hold_time  RowTracker 평활 계수 / 검출 실패 시 유지 시간 [s]
  obstacle.half_width / h_min / h_max ...  통로 장애물 검사 범위 [m]
  (min_trees_per_side, max_heading 은 ROS 파라미터로 노출되지 않음 → 이 파일의 기본값 수정)
알고리즘: 가중 최소제곱(WLS) 직선 적합 + 잔차 기반 반복 이상점 제거 + 지수이동평균(EMA) 평활. 일반 기법이라 별도 논문 인용 없음.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


@dataclass
class RowEstimate:
    """한 프레임의 행 추정 결과 (base_link 기준). 필드 의미는 orchard_msgs/RowCenterline 과 같다."""

    valid: bool = False
    offset: float = 0.0          # 중심선의 x=0 에서 y 값 (+: 중심선이 왼쪽)
    heading: float = 0.0         # atan(b), 중심선 방향 (+: 왼쪽으로 기울어짐)
    width: float = 0.0           # 두 열 사이 수직거리
    left_offset: float = float('nan')     # 왼쪽 열 직선의 a_L [m] (x=0 에서 y)
    right_offset: float = float('nan')    # 오른쪽 열 직선의 a_R [m]
    left_count: int = 0                   # 적합에 쓰인 왼쪽 줄기 수
    right_count: int = 0                  # 적합에 쓰인 오른쪽 줄기 수
    rms: float = 0.0                      # 적합 잔차 RMS [m]
    confidence: float = 0.0               # 0~1
    last_tree_ahead: float = float('-inf')  # 적합에 쓰인 줄기 중 가장 앞쪽 x [m] → 행 끝 판단용
    # 입력 줄기마다 +1(왼쪽 열) / -1(오른쪽 열) / 0(미사용). /orchard/trees 의 row_side 로 나감
    sides: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=int))

    @property
    def slope(self) -> float:
        """중심선 기울기 b = tan(heading) [m/m]."""
        return float(np.tan(self.heading))

    def centerline_y(self, x):
        """전방 거리 x [m] (스칼라 또는 배열) 에서 중심선의 y [m]."""
        return self.offset + self.slope * np.asarray(x)


def _solve_shared_slope(x, y, side, w):
    """side: +1(왼쪽) / -1(오른쪽). 반환 (a_L, a_R, b).

    미지수 [a_L, a_R, b] 에 대한 선형 모델 y_i = a_L·[왼쪽] + a_R·[오른쪽] + b·x_i 를
    가중 최소제곱으로 푼다. x, y: 줄기 좌표 [m], w: 줄기별 가중치(신뢰도).
    """
    left = (side > 0).astype(float)      # 왼쪽 열 지시변수 (0/1)
    right = (side < 0).astype(float)     # 오른쪽 열 지시변수 (0/1)
    a_mat = np.c_[left, right, x]
    # 가중 최소제곱: 각 행에 sqrt(w) 를 곱하면 Σ w_i·r_i² 를 최소화하는 일반 최소제곱과 같아진다
    sw = np.sqrt(w)
    sol, *_ = np.linalg.lstsq(a_mat * sw[:, None], y * sw, rcond=None)
    return float(sol[0]), float(sol[1]), float(sol[2])


def _solve_single(x, y, w):
    """한쪽 열만 보일 때 y = a + b x 를 가중 최소제곱으로 적합. 반환 (a, b)."""
    a_mat = np.c_[np.ones_like(x), x]
    sw = np.sqrt(w)
    sol, *_ = np.linalg.lstsq(a_mat * sw[:, None], y * sw, rcond=None)
    return float(sol[0]), float(sol[1])


def estimate_heading(xy: np.ndarray, weights: np.ndarray, expected_width: float,
                     prior_heading: float = 0.0, max_heading: float = math.radians(40.0),
                     step: float = math.radians(1.0)) -> float:
    """열 방향 탐색: 후보 각도마다 줄기들을 열 수직축에 투영했을 때
    좌/우 두 줄로 가장 잘 모이는 각도를 고른다 (간이 Hough).

    Hough 변환처럼 파라미터(각도) 공간을 전수 탐색한다는 발상만 빌렸고, 투표 대신 비용 최소화를 쓴다.
    xy: 줄기 위치 Nx2 [m, base_link], weights: 길이 N 가중치, expected_width: 행간거리 [m]
    prior_heading: 이전 프레임 방향 [rad], max_heading: 탐색 범위 ±[rad], step: 탐색 간격 [rad]
    반환: 가장 비용이 작은 열 방향 [rad]. 줄기가 3개 미만이면 prior 를 그대로 돌려준다.
    """
    if xy.shape[0] < 3:
        return float(np.clip(prior_heading, -max_heading, max_heading))
    half = 0.5 * expected_width
    best, best_cost = 0.0, np.inf
    prior = float(np.clip(prior_heading, -max_heading, max_heading))
    for th in np.arange(-max_heading, max_heading + 1e-9, step):   # +1e-9: 끝값(+max_heading) 포함
        u = -xy[:, 0] * np.sin(th) + xy[:, 1] * np.cos(th)     # 로봇 원점 기준 수직 좌표
        cost, used = 0.0, 0.0
        for sgn in (1, -1):              # +1: 왼쪽 열, -1: 오른쪽 열
            # 로봇이 통로 안에 있다고 보고, 행간 절반의 0.15 ~ 1.6 배 거리에 있는 줄기만 그 쪽 열 후보로 본다
            # (너무 가까운 것은 통로 안 물체, 너무 먼 것은 옆 통로의 열)
            m = (sgn * u > 0.15 * half) & (sgn * u < 1.6 * half)
            if np.count_nonzero(m) < 2:
                continue
            med = np.median(u[m])
            # 중앙값에서의 편차. 0.5 m 로 상한을 둬서 이상점 하나가 비용을 지배하지 않게 함
            dev = np.minimum(np.abs(u[m] - med), 0.5)
            cost += float(np.sum(weights[m] * dev))
            used += float(np.sum(weights[m]))
        if used < 1e-6:
            continue
        # 더 많은 줄기를 설명할수록 유리, 이전 방향과 가까울수록 약간 유리
        # 어느 열에도 속하지 않은 줄기는 최대 편차(0.5 m)만큼 벌점. 0.02/rad 는 동점일 때만 영향 주는 작은 사전항
        c = (cost + 0.5 * (np.sum(weights) - used)) / np.sum(weights) + 0.02 * abs(th - prior)
        if c < best_cost:
            best, best_cost = float(th), c
    return best


def fit_rows(
    xy: np.ndarray,
    weights: np.ndarray | None = None,
    expected_width: float = 3.8,
    prior_offset: float = 0.0,
    prior_heading: float = 0.0,
    x_min: float = -3.0,
    x_max: float = 12.0,
    max_residual: float = 0.35,
    min_trees_per_side: int = 2,
    max_heading: float = math.radians(40.0),
) -> RowEstimate:
    """줄기 위치(Nx2)로 좌/우 열과 중심선을 추정한다.

    xy: 줄기 위치 Nx2 [m, base_link], weights: 줄기 신뢰도 (None 이면 모두 1)
    expected_width: 행간거리 [m], prior_offset/prior_heading: 이전 추정 (offset 은 현재 사용 안 함)
    x_min, x_max: 적합에 쓸 줄기의 x 범위 [m], max_residual: 이상점 판정 잔차 [m]
    min_trees_per_side: 한 열로 인정할 최소 줄기 수, max_heading: 허용 최대 방향 오차 [rad]
    반환: RowEstimate. 조건(줄기 수, 방향, 폭)을 못 맞추면 valid=False.

    절차: (1) estimate_heading 으로 대략의 방향을 구해 줄기를 좌/우로 분류
    → (2) 공유 기울기 WLS 적합 → 잔차 큰 줄기 제거를 최대 3회 반복 → (3) 방향·폭 검사 및 신뢰도 계산.
    """
    est = RowEstimate(sides=np.zeros(xy.shape[0], dtype=int))
    if xy.shape[0] == 0:
        return est
    # 가중치 하한 1e-3: 0 가중치로 행렬이 특이해지는 것을 방지
    w_all = np.ones(xy.shape[0]) if weights is None else np.clip(np.asarray(weights, float), 1e-3, None)

    # 좌/우 열은 '로봇을 사이에 둔 두 열'로 정의한다 → 측면 분류 기준선은 로봇 원점을 지난다.
    # 기준선의 방향은 이전 추정에만 의존하지 않고 후보 각도 탐색으로 다시 구한다
    # (회전 직후 이전 추정이 옆 통로를 가리키는 문제 방지).
    del prior_offset  # 오프셋 사전정보는 옆 통로로 잘못 잠기는 원인이 되어 사용하지 않는다
    in_range = (xy[:, 0] >= x_min) & (xy[:, 0] <= x_max)
    half = 0.5 * expected_width
    heading0 = estimate_heading(xy[in_range], w_all[in_range], expected_width, prior_heading,
                                max_heading)
    b0 = np.tan(heading0)
    rel = xy[:, 1] - b0 * xy[:, 0]            # 로봇 원점을 지나는 기준선에서의 횡방향 거리
    rel_perp = rel * np.cos(heading0)         # y 방향 거리 → 기준선에 수직인 거리로 변환
    # estimate_heading 과 같은 기준 (행간 절반의 0.15 ~ 1.6 배) 으로 바로 옆 두 열의 줄기만 사용
    near_row = (np.abs(rel_perp) > 0.15 * half) & (np.abs(rel_perp) < 1.6 * half)
    use = in_range & near_row
    side = np.where(rel > 0, 1, -1)           # 기준선 왼쪽(+y) → 왼쪽 열 (+1)

    # 적합 → 이상점 제거 반복 (최대 3회). 이상점이 없으면 조기 종료
    for _ in range(3):
        idx = np.nonzero(use)[0]
        n_left = int(np.count_nonzero(side[idx] > 0))
        n_right = int(np.count_nonzero(side[idx] < 0))
        x, y, s, w = xy[idx, 0], xy[idx, 1], side[idx], w_all[idx]

        if n_left >= min_trees_per_side and n_right >= min_trees_per_side:
            a_l, a_r, b = _solve_shared_slope(x, y, s, w)
        elif n_left >= min_trees_per_side or n_right >= min_trees_per_side:
            # 한쪽 열만 보이면 그 열을 적합하고, 반대 열은 expected_width 만큼 평행이동해 가정한다
            sel = s > 0 if n_left >= min_trees_per_side else s < 0
            a_s, b = _solve_single(x[sel], y[sel], w[sel])
            # 수직거리 W 인 두 평행선의 y 절편 차이는 W / cos(방향)
            shift = expected_width / np.cos(np.arctan(b))
            if n_left >= min_trees_per_side:
                a_l, a_r = a_s, a_s - shift
            else:
                a_l, a_r = a_s + shift, a_s
        else:
            return est

        pred = np.where(s > 0, a_l, a_r) + b * x
        resid = np.abs(y - pred)              # y 방향 잔차 [m]
        bad = resid > max_residual
        if not np.any(bad):
            break
        use[idx[bad]] = False

    heading = float(np.arctan(b))
    if abs(heading) > max_heading:
        return est
    width = float((a_l - a_r) * np.cos(heading))   # y 절편 차이 → 두 열 사이 수직거리
    # 행간거리가 기대값의 0.5 ~ 1.5 배를 벗어나면 잘못된 열 조합(옆 통로 등)으로 보고 버림
    if not (0.5 * expected_width <= width <= 1.5 * expected_width):
        return est

    idx = np.nonzero(use)[0]
    pred = np.where(side[idx] > 0, a_l, a_r) + b * xy[idx, 0]
    resid = np.abs(xy[idx, 1] - pred)
    rms = float(np.sqrt(np.mean(resid ** 2))) if resid.size else 0.0
    n_left = int(np.count_nonzero(side[idx] > 0))
    n_right = int(np.count_nonzero(side[idx] < 0))
    # 신뢰도 = 줄기 수 점수 × 폭 점수 × 잔차 점수 (각 0~1)
    # 줄기 수: 한쪽 최대 4그루까지 셈 (4+4=8 이면 1.0), 양쪽 모두 2그루 이상이면 +0.25 보너스
    count_score = min(1.0, (min(n_left, 4) + min(n_right, 4)) / 8.0 + 0.25 * (min(n_left, n_right) >= 2))
    # 폭: 기대값과의 차이에 대한 가우시안 (표준편차 역할 0.35 × 기대 폭)
    width_score = float(np.exp(-((width - expected_width) / (0.35 * expected_width)) ** 2))
    # 잔차: RMS 가 max_residual 과 같으면 e^-1 ≈ 0.37
    rms_score = float(np.exp(-(rms / max_residual) ** 2))
    ahead = xy[idx, 0]
    est = RowEstimate(
        valid=True,
        offset=0.5 * (a_l + a_r),
        heading=heading,
        width=width,
        left_offset=a_l,
        right_offset=a_r,
        left_count=n_left,
        right_count=n_right,
        rms=rms,
        confidence=float(np.clip(count_score * width_score * rms_score, 0.0, 1.0)),
        last_tree_ahead=float(ahead.max()) if ahead.size else float('-inf'),
        sides=np.where(use, side, 0),
    )
    return est


class RowTracker:
    """프레임 간 중심선 평활화 + 일시적 검출 실패 시 유지.

    alpha: 지수이동평균 계수 (0~1, 클수록 새 측정을 많이 반영 → 빠르지만 흔들림)
    hold_time: 검출이 끊겨도 마지막 추정을 유지하는 시간 [s] (나무 한두 그루 빠진 구간 대비)
    max_jump: 새 offset 이 이전과 이만큼 [m] 이상 다르면 평활하지 않고 바로 교체 (다른 통로로 이동 등)
    """

    def __init__(self, alpha: float = 0.4, hold_time: float = 0.6, max_jump: float = 0.8):
        """평활 파라미터를 저장하고 상태를 비운다 (lidar_tree_node 의 row.smoothing_alpha / row.hold_time)."""
        self.alpha = alpha
        self.hold_time = hold_time
        self.max_jump = max_jump
        self.state: RowEstimate | None = None
        self.last_valid_time: float | None = None

    def prior(self) -> tuple[float, float]:
        """다음 fit_rows 에 넘길 사전정보 (offset [m], heading [rad]). 상태가 없으면 (0, 0)."""
        if self.state is None:
            return 0.0, 0.0
        return self.state.offset, self.state.heading

    def reset(self):
        """추적 상태 초기화 (예: 행 끝에서 회전한 뒤 새 통로 진입 시)."""
        self.state = None
        self.last_valid_time = None

    def update(self, est: RowEstimate, now: float) -> RowEstimate:
        """새 추정 est 를 반영한 평활 결과를 반환. now: 메시지 시각 [s].

        - est 가 유효: 이전 상태와 EMA 로 섞음 (offset 이 max_jump 이상 튀면 그대로 교체)
        - est 가 무효: hold_time 이내면 이전 상태 사본을 신뢰도 절반으로 반환, 지나면 무효 반환
        """
        if est.valid:
            if self.state is None or abs(est.offset - self.state.offset) > self.max_jump:
                self.state = est
            else:
                a = self.alpha
                s = self.state
                # 지수이동평균: 새값 = (1-α)·이전 + α·측정
                est.offset = (1 - a) * s.offset + a * est.offset
                est.heading = (1 - a) * s.heading + a * est.heading
                est.width = (1 - a) * s.width + a * est.width
                self.state = est
            self.last_valid_time = now
            return self.state
        if self.state is not None and self.last_valid_time is not None \
                and now - self.last_valid_time < self.hold_time:
            held = RowEstimate(**{**self.state.__dict__})    # 상태를 건드리지 않도록 얕은 복사
            held.confidence *= 0.5                           # 유지 중임을 신뢰도로 알림
            held.last_tree_ahead = est.last_tree_ahead       # 행 끝 판단은 현재 프레임 값을 따름
            return held
        self.state = None
        return est


def corridor_obstacle_distance(
    points: np.ndarray,
    heights: np.ndarray,
    offset: float,
    heading: float,
    half_width: float,
    x_min: float = 0.3,
    x_max: float = 6.0,
    h_min: float = 0.2,
    h_max: float = 1.6,
    min_points: int = 5,
    bin_size: float = 0.15,
) -> float:
    """주행 통로(중심선 ± half_width) 안에서 가장 가까운 장애물까지 x 거리. 없으면 inf.

    points: 점군 Nx3 [m, base_link], heights: 각 점의 지면 높이 [m]
    offset, heading: 중심선 [m, rad] (행 추정 실패 시 노드가 0, 0 = 로봇 정면을 넘김)
    half_width: 통로 반폭 [m] (차체 반폭 + 여유), x_min/x_max: 검사 전방 거리 범위 [m]
    h_min/h_max: 장애물로 볼 높이 범위 [m] (h_min 아래는 잔디, h_max 위는 차체보다 높은 처진 가지)
    min_points: 장애물 판정 최소 점 수 (잡음 한두 점 무시), bin_size: x 방향 히스토그램 칸 [m]
    반환: 장애물 x 거리 [m] 또는 inf.
    """
    if points.shape[0] == 0:
        return float('inf')
    b = np.tan(heading)
    # 중심선까지의 수직거리 (y 방향 차이 × cos(heading))
    lateral = (points[:, 1] - (offset + b * points[:, 0])) * np.cos(heading)
    mask = ((points[:, 0] > x_min) & (points[:, 0] < x_max) & (np.abs(lateral) < half_width)
            & (heights > h_min) & (heights < h_max))
    xs = points[mask, 0]
    if xs.size < min_points:
        return float('inf')
    # x 방향 히스토그램으로 점이 모인 곳을 찾는다 (흩어진 잡음 점은 한 칸에 min_points 이상 모이기 어려움)
    bins = np.floor((xs - x_min) / bin_size).astype(int)
    counts = np.bincount(bins)
    # 인접 2개 bin 합으로 판정 (얇은 물체 보정)
    smooth = counts + np.r_[counts[1:], 0]
    hit = np.nonzero(smooth >= min_points)[0]
    if hit.size == 0:
        return float('inf')
    first = hit[0]                                    # 가장 가까운 장애물 칸
    in_bin = xs[(bins == first) | (bins == first + 1)]
    return float(in_bin.min())
