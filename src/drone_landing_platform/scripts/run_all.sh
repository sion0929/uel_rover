#!/usr/bin/env bash
# 4개 시나리오를 헤드리스로 차례대로 실행하고 결과(CSV + JSON)를 모은다.
#
# 사용법 (워크스페이스를 source 한 뒤):
#   bash run_all.sh                      # 결과: ~/landing_results
#   bash run_all.sh /path/to/out         # 결과 폴더 지정
#   MODE=sw_openloop bash run_all.sh     # SolidWorks 추력 스케줄 모드
#   TIMEOUT=180 EXTRA="damping:=30.0" bash run_all.sh
set -u

OUT="${1:-$HOME/landing_results}"
MODE="${MODE:-closed_loop}"
TIMEOUT="${TIMEOUT:-120}"
EXTRA="${EXTRA:-}"
SCENARIOS="${SCENARIOS:-center x30 x60 yaw30}"

mkdir -p "$OUT"
if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 명령이 없습니다. 먼저 source /opt/ros/jazzy/setup.bash && source install/setup.bash" >&2
  exit 1
fi

cleanup_gz() {
  pkill -INT -f "gz sim" >/dev/null 2>&1 || true
  sleep 1
  pkill -KILL -f "gz sim" >/dev/null 2>&1 || true
  pkill -f parameter_bridge >/dev/null 2>&1 || true
}

for s in $SCENARIOS; do
  echo "================= scenario: $s (mode: $MODE) ================="
  cleanup_gz
  # shellcheck disable=SC2086
  timeout --signal=INT --kill-after=15 "$TIMEOUT" \
    ros2 launch drone_landing_platform landing_sim.launch.py \
      scenario:="$s" mode:="$MODE" gui:=false csv_dir:="$OUT" shutdown_on_done:=true $EXTRA \
    2>&1 | tee "$OUT/${s}_${MODE}_launch.log"
  cleanup_gz
  sleep 2
done

echo
echo "================= 결과 요약 ($OUT) ================="
for s in $SCENARIOS; do
  if [ "$MODE" = "closed_loop" ]; then base="$s"; else base="${s}_${MODE}"; fi
  f="$OUT/${base}_result.json"
  if [ -f "$f" ]; then
    echo "--- $s"
    cat "$f"
    echo
  else
    echo "--- $s: 결과 파일 없음 ($f) - ${s}_${MODE}_launch.log 확인"
  fi
done
