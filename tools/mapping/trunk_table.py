"""저장된 줄기 지도(trunks.csv)를 열별 표로 정리 — ROS 없이. 예전 결과 폴더에도 쓸 수 있다.

입력: voxel_map_node 가 저장한 폴더 (data/voxel_maps/voxel_map_<날짜시각>/ 의 trunks.csv, meta.json)
출력: 같은 폴더에 trunks_by_row.csv (엑셀), trunks_table.html (한글·워드에 복사), trunks_table.md, 화면에 열 요약
  (voxel_map_node 는 저장할 때 같은 파일을 자동으로 만든다 — orchard_mapping/trunk_table.py)

사용 예
  python3 tools/mapping/trunk_table.py data/voxel_maps/voxel_map_<날짜시각>
  python3 tools/mapping/trunk_table.py data/voxel_maps/voxel_map_<날짜시각> \
      --truth ~/.cache/orchard_rover/worlds/orchard_meta.json
"""
from __future__ import annotations

import argparse
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
for pkg in ('orchard_mapping', 'orchard_navigation'):
    sys.path.insert(0, os.path.join(ROOT, 'ros2_ws', 'src', pkg))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from orchard_mapping.trunk_table import write_tables  # noqa: E402
from view_voxel_map import read_csv, truth_in_start  # noqa: E402


def main() -> None:
    """폴더의 trunks.csv → 표 파일 저장, 열 요약 출력."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('folder', help='voxel_map_<날짜시각> 폴더')
    ap.add_argument('--row-spacing', type=float, default=3.8, help='열 간격 [m] (열 나누기 기준)')
    ap.add_argument('--truth', default='', help='(시뮬) Gazebo orchard_meta.json — 정답 오차 열 추가')
    ap.add_argument('--title', default='과수 줄기 위치')
    a = ap.parse_args()
    trunks = read_csv(os.path.join(a.folder, 'trunks.csv'))
    truth = None
    meta_path = os.path.join(a.folder, 'meta.json')
    if a.truth and os.path.isfile(meta_path):
        truth = truth_in_start(a.truth, json.load(open(meta_path, encoding='utf-8')))
    rows = write_tables(a.folder, trunks, a.row_spacing, truth, a.title)
    print(f'열 {len(rows)}개, 줄기 {sum(r.n for r in rows)}개 → {a.folder}/trunks_table.html (.md, trunks_by_row.csv)')
    for r in rows:
        print(f'  열 {r.row}: y {r.y:+.2f} m, {r.n}그루, x {r.x_min:.1f}~{r.x_max:.1f} m, '
              f'주간 {r.gap_med:.2f} m, 결주 의심 {r.missing}')


if __name__ == '__main__':
    main()
