#!/usr/bin/env bash
# 컨테이너 안에서 실행: 가상 화면(Xvfb)에 Gazebo GUI 와 RViz 를 띄우고 녹화한다.
# 직접 쓰지 말고 scripts/gazebo_demo_job.sh 를 사용할 것.
#
# 하는 일
#   1) 워크스페이스 빌드 → 2) 가상 화면 두 개(:91 Gazebo GUI 1600x900, :92 RViz 1280x800) 실행
#   3) sim.launch.py 실행 → 4) WARMUP 동안 대기 (중간에 Gazebo GUI 카메라를 로버 추적으로 설정)
#   5) 두 화면을 ffmpeg 로 녹화 + 주기적 스크린샷. 미션이 DONE/STOPPED 가 되면 일찍 종료
#   6) 프로세스 정리 후 로그에서 횡오차·오류 요약 추출
# 환경변수 (gazebo_demo_job.sh 가 docker run -e 로 넘기거나 scripts/demo.env 에 적음)
#   OUT (필수) 결과 폴더 / DURATION 최대 녹화 [s] / WARMUP 준비 대기 [s] / LITE true|false
#   HEADLESS_RENDERING true|false / FPS 녹화 프레임률 (기본 15) / TAIL 미션 종료 뒤 추가 녹화 [s] (기본 15)
#   LAUNCH_ARGS sim.launch.py 에 덧붙일 인자 / STUCK_LIMIT 미션 중 로버가 안 움직이면 이 시간 [s] 뒤 종료 (기본 180)
# 결과 파일 ($OUT): gazebo.mp4, rviz.mp4, gazebo_shot_*.png, rviz_shot_*.png, timeline.txt,
#   build.log, launch.log, rviz.log, mission_state_start.txt, mission_state_end.txt,
#   gz_stats.txt, lane_error.txt, errors.txt
set -eo pipefail
# Docker Desktop 에서 컨테이너를 '다시 시작'할 때 녹화 시간 등을 바꾸려면 scripts/demo.env 에 DURATION=1200 처럼 적는다
[ -f /workspace/scripts/demo.env ] && . /workspace/scripts/demo.env
# ':' 는 아무 일도 하지 않는 명령. 인자 확장만 일어나 OUT 이 없으면 오류로 끝나고, 나머지는 비어 있으면 기본값을 넣는다
: "${OUT:?OUT 경로 필요}" "${DURATION:=240}" "${WARMUP:=60}" "${LITE:=true}" "${HEADLESS_RENDERING:=false}"
mkdir -p "$OUT"

# ROS 2 Jazzy + px4_msgs 환경
source /opt/ros/jazzy/setup.bash
source /opt/px4_ws/install/setup.bash
cd /workspace
echo "[inner] 워크스페이스 빌드"
./scripts/build_ws.sh > "$OUT/build.log" 2>&1
source /workspace/ros2_ws/install/setup.bash

# 컨테이너를 '다시 시작'하면 이전 가상 화면의 잠금 파일이 남아 Xvfb 가 뜨지 않는다 → 지우고 시작
rm -f /tmp/.X91-lock /tmp/.X92-lock /tmp/.X11-unix/X91 /tmp/.X11-unix/X92
# 가상 X 화면 두 개 (:91 Gazebo, :92 RViz). '& XG=$!' = 백그라운드로 실행하고 PID 를 저장 (나중에 종료용)
Xvfb :91 -screen 0 1600x900x24 -nolisten tcp > /dev/null 2>&1 & XG=$!
Xvfb :92 -screen 0 1280x800x24 -nolisten tcp > /dev/null 2>&1 & XR=$!
sleep 2

echo "[inner] 시뮬레이션 시작 (lite=$LITE, headless_rendering=$HEADLESS_RENDERING)"
# RViz 는 launch 안이 아니라 아래에서 별도 가상 화면(:92)에 띄우므로 rviz:=false
# LAUNCH_ARGS: 추가 launch 인자 (예: scripts/demo.env 에 LAUNCH_ARGS="explore:=true camera_lite:=true")
# shellcheck disable=SC2086
DISPLAY=:91 ros2 launch orchard_bringup sim.launch.py lite:="$LITE" rviz:=false \
  headless_rendering:="$HEADLESS_RENDERING" ${LAUNCH_ARGS:-} > "$OUT/launch.log" 2>&1 & LP=$!
sleep 5
# RViz 를 두 번째 가상 화면에 띄움 (설치된 orchard.rviz 설정 사용)
DISPLAY=:92 rviz2 -d "$(ros2 pkg prefix orchard_bringup)/share/orchard_bringup/rviz/orchard.rviz" \
  --ros-args -p use_sim_time:=true > "$OUT/rviz.log" 2>&1 & RP=$!

# 진단용 타임라인: 시뮬 시각·정답 위치·임무 상태를 15초마다 기록 (로버가 명령 없이 움직이는지 등 원인 추적용)
timeline() {
  set +e
  # 진단용 ros2 CLI 는 짧게 여러 번 뜨는 프로세스라 Fast DDS 공유메모리(SHM) 포트를 다 써서
  # 'open_and_lock_file failed' 뒤로 토픽을 못 받을 수 있다 → CLI 만 UDP 로
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  while true; do
    {
      # timeout -k 2 8: 8초 안에 안 끝나면 종료 신호, 그래도 2초 더 버티면 강제 종료 (--once 가 무한 대기하지 않게)
      echo "=== $(date '+%T')"
      timeout -k 2 8 ros2 topic echo --once /clock --field clock 2>/dev/null | head -2 | tr '\n' ' '; echo
      timeout -k 2 8 ros2 topic echo --once /ground_truth/odom --field pose.pose.position 2>/dev/null | head -3 | tr '\n' ' '; echo
      timeout -k 2 8 ros2 topic echo --once /orchard/mission/state --field data 2>/dev/null | head -1
      echo -n "odom: "; timeout -k 2 8 ros2 topic echo --once /odom --field pose.pose.position 2>/dev/null | head -3 | tr '\n' ' '; echo
      echo -n "row.valid x10: "; timeout -k 2 8 ros2 topic echo /orchard/row --field valid 2>/dev/null | grep -v -- --- | head -10 | tr '\n' ' '; echo
      echo -n "cmd_vel: "; timeout -k 2 8 ros2 topic echo --once /cmd_vel --field linear.x 2>/dev/null | head -1
      echo -n "seg_row: "; timeout -k 2 8 ros2 topic echo --once /orchard/seg/row 2>/dev/null | grep -E "valid|lateral_offset|confidence" | tr -s ' \n' ' '; echo
      echo -n "wheel cmd: "; timeout -k 2 5 gz topic -e -t /model/orchard_rover/command/motor_speed -n 1 2>/dev/null | tr '\n' ' '; echo
      echo -n "joints(name/vel): "; timeout -k 2 5 gz topic -e -t /world/orchard/model/orchard_rover/joint_state -n 1 2>/dev/null \
        | grep -E '^ *name:|velocity:' | tr -s ' \n' ' ' | cut -c1-600; echo
    } >> "$OUT/timeline.txt"
    sleep 15
  done
}
# 타임라인 기록을 백그라운드로 시작 (PID = TL)
timeline & TL=$!

echo "[inner] 준비 대기 ${WARMUP}s"
# 준비 시간의 절반을 기다린 뒤 GUI 카메라 추적을 설정하고, 나머지 절반을 더 기다린다
sleep "$(( WARMUP / 2 ))"
# Gazebo GUI 카메라가 로버를 뒤에서 따라가도록 (녹화 영상에 로버가 보이게)
timeout 10 gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
  --timeout 5000 --req 'data: "orchard_rover"' > "$OUT/gui_follow.txt" 2>&1 || true
timeout 10 gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean \
  --timeout 5000 --req 'x: -5.0, y: 0.0, z: 3.0' >> "$OUT/gui_follow.txt" 2>&1 || true
sleep "$(( WARMUP - WARMUP / 2 ))"
# 녹화 시작 시점의 미션 상태 기록
timeout 10 ros2 topic echo /orchard/mission/state --once > "$OUT/mission_state_start.txt" 2>&1 || true

echo "[inner] 녹화 최대 ${DURATION}s (미션이 DONE/STOPPED 가 되면 ${TAIL:-15}s 뒤 일찍 끝냄)"
# 두 가상 화면을 동시에 녹화 (x11grab = X 화면 캡처, -t = 최대 길이, H.264, crf 28 = 적당한 압축)
ffmpeg -loglevel error -y -f x11grab -framerate "${FPS:-15}" -video_size 1600x900 -i :91 -t "$DURATION" \
  -c:v libx264 -pix_fmt yuv420p -crf 28 "$OUT/gazebo.mp4" & F1=$!
ffmpeg -loglevel error -y -f x11grab -framerate "${FPS:-15}" -video_size 1280x800 -i :92 -t "$DURATION" \
  -c:v libx264 -pix_fmt yuv420p -crf 28 "$OUT/rviz.mp4" & F2=$!
# 녹화 중 감시 루프: 10초마다
#   (1) 일정 간격 스크린샷 (최대 9장, 간격 = min(DURATION/4, 60) 초)
#   (2) 미션 상태 확인 → DONE/STOPPED 면 TAIL 초 더 녹화하고 종료
#   (3) 미션이 진행 중인데 로버(odom)가 STUCK_LIMIT 초 동안 0.1 m 도 안 움직이면 종료
#       (PX4 가 바퀴 명령을 안 내보내 로버가 서 있는 채로 녹화만 계속되는 경우 대비)
T0=$(date +%s); SHOT=1; SHOT_EVERY=$(( DURATION / 4 > 60 ? 60 : DURATION / 4 ))
STUCK_LIMIT=${STUCK_LIMIT:-180}; LAST_XY=""; STILL_SINCE=""
while kill -0 "$F1" 2>/dev/null; do
  sleep 10
  EL=$(( $(date +%s) - T0 ))
  if [ "$EL" -ge $(( SHOT * SHOT_EVERY )) ] && [ "$SHOT" -le 9 ]; then
    ffmpeg -loglevel error -y -f x11grab -video_size 1600x900 -i :91 -frames:v 1 "$OUT/gazebo_shot_$SHOT.png" || true
    ffmpeg -loglevel error -y -f x11grab -video_size 1280x800 -i :92 -frames:v 1 "$OUT/rviz_shot_$SHOT.png" || true
    SHOT=$(( SHOT + 1 ))
  fi
  # 6초 안에 메시지가 없으면 timeout 이 124 를 돌려주는데, set -e + pipefail 이면 스크립트(=컨테이너) 전체가 끝나
  # 녹화가 중간에 끊긴다 (컨테이너 Exited (124)). 반드시 || true
  ST=$(FASTDDS_BUILTIN_TRANSPORTS=UDPv4 timeout -k 2 6 ros2 topic echo --once /orchard/mission/state --field data 2>/dev/null | head -1) || true
  case "$ST" in
    DONE*|STOPPED*)
      echo "[inner] 미션 종료 감지: $ST"
      sleep "${TAIL:-15}"
      ffmpeg -loglevel error -y -f x11grab -video_size 1600x900 -i :91 -frames:v 1 "$OUT/gazebo_shot_end.png" || true
      ffmpeg -loglevel error -y -f x11grab -video_size 1280x800 -i :92 -frames:v 1 "$OUT/rviz_shot_end.png" || true
      kill -INT "$F1" "$F2" 2>/dev/null || true       # ffmpeg 는 SIGINT 에 파일을 정상 마무리한다
      break ;;
    APPROACH*|FOLLOW_ROW*|TURN*|ENTER_ROW*|EXIT_ROW*)
      # odom 위치를 0.1 m 로 반올림해 지난번과 같으면 '서 있음'
      XY=$(FASTDDS_BUILTIN_TRANSPORTS=UDPv4 timeout -k 2 6 ros2 topic echo --once /odom --field pose.pose.position 2>/dev/null \
           | awk '/^x:/{x=$2} /^y:/{y=$2} END{if (x != "") printf "%.1f %.1f", x, y}') || true
      if [ -n "$XY" ] && [ "$XY" = "$LAST_XY" ]; then
        STILL_SINCE=${STILL_SINCE:-$EL}
        if [ $(( EL - STILL_SINCE )) -ge "$STUCK_LIMIT" ]; then
          echo "[inner] 미션 진행 중인데 로버가 ${STUCK_LIMIT}s 동안 안 움직임 → 녹화 종료 (timeline.txt 의 wheel cmd, PX4 경고 확인)"
          kill -INT "$F1" "$F2" 2>/dev/null || true
          break
        fi
      elif [ -n "$XY" ]; then
        STILL_SINCE=""; LAST_XY=$XY
      fi ;;
  esac
done
# 두 ffmpeg 가 끝날 때까지 기다린 뒤 마무리 기록
wait "$F1" "$F2" || true
timeout 10 ros2 topic echo /orchard/mission/state --once > "$OUT/mission_state_end.txt" 2>&1 || true
timeout 10 gz topic -e -t /world/orchard/stats -n 1 > "$OUT/gz_stats.txt" 2>&1 || true   # real_time_factor = 실시간 대비 속도

echo "[inner] 종료"
# --- 정리: 타임라인·launch·RViz 종료 → 남은 Gazebo/PX4/Agent 프로세스 강제 종료 → 가상 화면 종료 ---
# pkill 패턴의 [g] 같은 대괄호: 이 명령을 실행한 셸의 명령줄이 패턴에 걸려 자기 자신을 죽이지 않게 하는 관용구
kill "$TL" 2>/dev/null || true
kill -INT "$LP" "$RP" 2>/dev/null || true
sleep 8
pkill -f "[g]z sim" || true
pkill -f "[p]x4_sitl_default/bin/px4" || true
pkill -f "[M]icroXRCEAgent" || true
kill "$XG" "$XR" 2>/dev/null || true
# 로그 요약: 마지막 통로 중심선 횡오차 한 줄, 오류 줄 목록
grep -h "통로 중심선 횡오차" "$OUT/launch.log" | tail -1 > "$OUT/lane_error.txt" || true
grep -E "Traceback|Error|process has died" "$OUT/launch.log" > "$OUT/errors.txt" || true   # 비어 있어야 정상
echo "[inner] 완료"
