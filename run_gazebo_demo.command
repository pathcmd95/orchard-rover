#!/bin/bash
# Finder 에서 더블클릭하면 Gazebo 데모 녹화를 백그라운드로 시작합니다 (창은 닫아도 됨).
# macOS 전용 .command 파일: Finder 에서 더블클릭하면 터미널에서 실행된다.
# 실제 작업은 scripts/gazebo_demo_job.sh 가 한다 (Docker Desktop 이 실행 중이어야 함).
# 녹화 시간 등은 scripts/demo.env 에서 바꾼다. 진행 상황은 결과 폴더의 job.log 로 확인.
# 이 파일이 있는 폴더(저장소 최상위)로 이동
cd "$(dirname "$0")" || exit 1
# nohup + &: 터미널 창을 닫아도 작업이 계속되도록 백그라운드 실행 ($! = 방금 시작한 작업의 PID)
nohup ./scripts/gazebo_demo_job.sh > /dev/null 2>&1 &
echo "Gazebo 데모 시작 (PID $!). 결과: $(pwd)/data/media/ 아래 최신 gazebo_* 폴더"
echo "약 15분 걸립니다. 이 창은 닫아도 됩니다."
