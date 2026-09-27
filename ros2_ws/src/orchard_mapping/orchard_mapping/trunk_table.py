"""줄기 위치를 열별 표로 정리 — ROS 의존성 없음.

확정 줄기(trunks.csv 의 행들)를 열(row)로 묶고, 열 안에서는 x(앞으로 간 거리) 순서로 번호를 매겨 표로 저장한다.
  열 번호: start 좌표 y 가 작은 쪽(출발 방향 기준 오른쪽)부터 1, 2, 3 … (열 y 위치를 함께 적어 헷갈리지 않게)
  열 안 순번: x 가 작은 쪽(출발점 쪽)부터 1, 2, 3 …
  간격: 같은 열 앞 줄기와의 거리 [m]. 그 열 간격 중앙값의 1.6 배를 넘으면 '결주 의심' (나무가 빠졌거나 못 본 곳),
        0.6 배보다 짧으면 '기둥·중복 의심' (나무 사이 기둥·지주를 줄기로 잡았거나 한 나무가 두 번 등록)

저장 파일 (write_tables)
  trunks_by_row.csv  엑셀용 (UTF-8 BOM — 엑셀에서 한글이 안 깨짐)
  trunks_table.html  한글(HWP)·워드에 복사해 붙이기 좋은 표 (브라우저로 열고 전체 선택 → 복사)
  trunks_table.md    마크다운 표 (GitHub·노션)
정답(시뮬 월드 나무 위치, start 좌표)을 주면 '정답 오차' 열을 붙인다.
"""
from __future__ import annotations

import csv
import html
import os
from dataclasses import dataclass, field

import numpy as np
from orchard_navigation.treemap import _row_peaks

GAP_FACTOR = 1.6          # 열 간격 중앙값의 이 배수를 넘는 간격 = 결주 의심
SHORT_FACTOR = 0.6        # 이 배수보다 짧은 간격 = 기둥(나무 사이 콘크리트 기둥·지주)이나 같은 나무 중복 등록 의심
MIN_ROW_TREES = 3         # 이보다 적은 줄기로 된 '열' 은 오검출 의심으로 표시 (과수원 밖 숲, 통로 물체 등)


@dataclass
class TableRow:
    """표 한 줄 (줄기 하나)."""
    row: int                 # 열 번호 (1부터)
    order: int               # 열 안 순번 (1부터)
    id: int
    x: float
    y: float
    z: float
    hits: int
    conf: float
    source: str
    gap: float = float('nan')        # 같은 열 앞 줄기와 거리 [m]
    err: float = float('nan')        # 정답까지 거리 [m] (시뮬)
    note: str = ''


@dataclass
class RowSummary:
    """열 하나 요약."""
    row: int
    y: float                 # 열 y 평균 [m]
    n: int                   # 줄기 수
    x_min: float
    x_max: float
    gap_med: float           # 주간 거리 중앙값 [m]
    missing: int             # 결주 의심 개수
    items: list = field(default_factory=list)
    note: str = ''
    short: int = 0           # 기둥·중복 의심 개수


def _f(v, nd=2) -> str:
    """숫자 → 글자 (nan 은 빈칸)."""
    return '' if v is None or not np.isfinite(v) else f'{v:.{nd}f}'


def build(trunks: list[dict], row_spacing: float = 3.8, truth: np.ndarray | None = None) -> list[RowSummary]:
    """trunks: trunks.csv 행 (dict: id,x,y,z,hits,mean_conf,source) → 열별 요약 목록 (y 오름차순)."""
    if not trunks:
        return []
    P = np.array([[float(t['x']), float(t['y'])] for t in trunks])
    peaks = np.sort(_row_peaks(P[:, 1], row_spacing)) if len(P) >= 4 else np.array([np.median(P[:, 1])])
    lab = np.argmin(np.abs(P[:, 1][:, None] - peaks[None, :]), axis=1)
    out = []
    for r in range(len(peaks)):
        idx = [i for i in np.argsort(P[:, 0]) if lab[i] == r]
        if not idx:
            continue
        items = []
        for k, i in enumerate(idx):
            t = trunks[i]
            tr = TableRow(r + 1, k + 1, int(t['id']), float(t['x']), float(t['y']), float(t.get('z', 0) or 0),
                          int(t.get('hits', 0) or 0), float(t.get('mean_conf', 0) or 0), t.get('source', ''))
            if k:
                tr.gap = float(np.hypot(*(P[i] - P[idx[k - 1]])))
            if truth is not None and len(truth):
                tr.err = float(np.min(np.hypot(truth[:, 0] - tr.x, truth[:, 1] - tr.y)))
            items.append(tr)
        gaps = np.array([it.gap for it in items[1:]])
        med = float(np.median(gaps)) if len(gaps) else float('nan')
        miss = short = 0
        prev_short = False
        for it in items[1:]:
            if np.isfinite(med) and it.gap > GAP_FACTOR * med:
                n_skip = int(round(it.gap / med)) - 1
                it.note = f'앞에 결주 의심 {max(n_skip, 1)}'
                miss += max(n_skip, 1)
                prev_short = False
            elif np.isfinite(med) and it.gap < SHORT_FACTOR * med and not prev_short:
                it.note = '간격 짧음 — 기둥·중복 의심'     # 나무 사이 기둥이면 짧은 간격이 두 번 이어짐 → 앞 것만 표시
                short += 1
                prev_short = True
            else:
                prev_short = False
        ys = np.array([it.y for it in items])
        out.append(RowSummary(len(out) + 1, float(ys.mean()), len(items), items[0].x, items[-1].x, med, miss, items,
                              short=short))
        if len(items) < MIN_ROW_TREES:
            out[-1].note = f'줄기 {len(items)}개뿐 — 오검출 의심'
        for it in items:
            it.row = out[-1].row
            if out[-1].note and not it.note:
                it.note = '오검출 의심'
    return out


def write_tables(folder: str, trunks: list[dict], row_spacing: float = 3.8, truth: np.ndarray | None = None,
                 title: str = '과수 줄기 위치') -> list[RowSummary]:
    """열별 표를 csv·html·md 로 저장하고 요약을 돌려준다."""
    rows = build(trunks, row_spacing, truth)
    has_err = truth is not None and len(truth) > 0
    head = ['열', '순번', 'ID', 'x [m]', 'y [m]', '앞 나무와 간격 [m]', '관측 수', '인식 경로', '비고'] \
        + (['정답 오차 [m]'] if has_err else [])
    body = []
    for r in rows:
        for it in r.items:
            body.append([str(it.row), str(it.order), f'T{it.id}', _f(it.x), _f(it.y), _f(it.gap), str(it.hits),
                         'YOLO' if it.source == 'yolo' else 'LiDAR', it.note] + ([_f(it.err)] if has_err else []))
    s_head = ['열', '열 y [m]', '그루 수', 'x 범위 [m]', '주간 거리 중앙값 [m]', '결주 의심', '기둥·중복 의심', '비고']
    s_body = [[str(r.row), _f(r.y), str(r.n), f'{_f(r.x_min, 1)} ~ {_f(r.x_max, 1)}', _f(r.gap_med), str(r.missing),
               str(r.short), r.note] for r in rows]
    n_all = sum(r.n for r in rows)
    note = ('좌표: start 프레임 — 출발점 (0, 0), +x 처음 곧게 달린 방향, +y 왼쪽 [m]. '
            '열 번호는 y 가 작은 쪽(출발 방향 오른쪽)부터, 순번은 출발점 쪽부터.')
    if has_err:
        errs = np.array([it.err for r in rows for it in r.items])
        note += f' 정답 오차 중앙값 {np.median(errs):.2f} m, 최대 {errs.max():.2f} m.'

    os.makedirs(folder, exist_ok=True)
    with open(os.path.join(folder, 'trunks_by_row.csv'), 'w', newline='', encoding='utf-8-sig') as f:
        w = csv.writer(f)
        w.writerow(head)
        w.writerows(body)

    def md(h, b):
        rows_md = ['| ' + ' | '.join(x) + ' |' for x in b]
        return '\n'.join(['| ' + ' | '.join(h) + ' |', '|' + '---|' * len(h)] + rows_md)
    with open(os.path.join(folder, 'trunks_table.md'), 'w', encoding='utf-8') as f:
        f.write(f'# {title} (열 {len(rows)}개, 줄기 {n_all}개)\n\n{note}\n\n## 열 요약\n\n{md(s_head, s_body)}\n\n'
                f'## 줄기 목록\n\n{md(head, body)}\n')

    def tbl(h, b):
        th = ''.join(f'<th>{html.escape(x)}</th>' for x in h)
        tr = ''.join('<tr>' + ''.join(f'<td>{html.escape(x)}</td>' for x in row) + '</tr>' for row in b)
        return f'<table><thead><tr>{th}</tr></thead><tbody>{tr}</tbody></table>'
    css = ('body{font-family:"Malgun Gothic","Apple SD Gothic Neo",sans-serif;font-size:10pt;margin:16px}'
           'table{border-collapse:collapse;margin:6px 0 18px}th,td{border:1px solid #444;padding:3px 8px;'
           'text-align:center}th{background:#e8e8e8}p{color:#333}')
    with open(os.path.join(folder, 'trunks_table.html'), 'w', encoding='utf-8') as f:
        f.write(f'<!doctype html><html lang="ko"><head><meta charset="utf-8"><title>{html.escape(title)}</title>'
                f'<style>{css}</style></head><body><h2>{html.escape(title)} (열 {len(rows)}개, 줄기 {n_all}개)</h2>'
                f'<p>{html.escape(note)}</p><h3>열 요약</h3>{tbl(s_head, s_body)}<h3>줄기 목록</h3>{tbl(head, body)}'
                f'</body></html>\n')
    return rows
