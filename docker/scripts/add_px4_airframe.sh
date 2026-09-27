#!/usr/bin/env bash
# PX4 소스트리에 과수원 로버 SITL 에어프레임을 추가한다.
# 사용법: add_px4_airframe.sh <PX4-Autopilot 경로> <airframe 파일>...
# 예:     add_px4_airframe.sh /opt/PX4-Autopilot /opt/orchard/px4/airframes/*
# 언제: Docker 이미지 빌드 중 install_sim.sh 가 호출한다 (직접 실행할 일은 거의 없음).
# 왜: PX4 SITL 은 ROMFS/.../airframes 폴더에 있고 같은 폴더 CMakeLists.txt 목록에 등록된 에어프레임만
#     빌드 결과에 포함한다. 그래서 파일 복사와 CMakeLists 등록을 함께 해야 PX4_SYS_AUTOSTART=51010 이 동작한다.
set -euo pipefail
PX4_DIR=${1:?PX4 경로}
shift
# SITL(posix) 에어프레임 폴더
AF_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
# 인자로 받은 에어프레임 파일마다
for f in "$@"; do
  name=$(basename "$f")
  # 파일 복사 (권한 0644)
  install -m 0644 "$f" "$AF_DIR/$name"
  # 이미 등록돼 있지 않을 때만 CMakeLists.txt 에 파일명을 추가 (여러 번 실행해도 안전)
  if ! grep -q "^\s*$name\s*$" "$AF_DIR/CMakeLists.txt"; then
    # 51000_gz_rover_ackermann 다음 줄에 추가
    sed -i "s/^\(\s*\)51000_gz_rover_ackermann\s*$/&\n\1$name/" "$AF_DIR/CMakeLists.txt"
  fi
  # 등록 확인 (실패하면 PX4 버전이 바뀌어 기준 줄 51000_gz_rover_ackermann 이 없을 수 있음)
  grep -q "$name" "$AF_DIR/CMakeLists.txt" || { echo "CMakeLists 추가 실패: $name" >&2; exit 1; }
  echo "airframe 추가: $name"
done
