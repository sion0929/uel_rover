#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────
#  개발PC — 네이티브 rviz2 실행
#  젯슨의 zenoh_bridge(tcp/192.168.0.112:7447)로 접속해 젯슨의 ROS2
#  토픽을 로컬 DDS 그래프(ROS_DOMAIN_ID=0)로 재주입한 뒤, rviz2는 그
#  로컬 그래프를 그냥 구독한다.
#
#  전제: setup.sh를 먼저 1회 실행해 둘 것.
# ─────────────────────────────────────────────────────────────────
set -e
cd "$(dirname "$0")"

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
# 개발PC 호스트 기본 RMW가 rmw_zenoh_cpp로 설정되어 있을 수 있음. 그대로 두면
# rviz2가 로컬 DDS 그래프 대신 자체 Zenoh 세션으로 붙어 zenoh_bridge가
# 재주입한 토픽을 못 볼 수 있으므로 fastrtps로 명시 고정한다.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

PIDS=()
cleanup() {
    echo "[rviz.sh] 종료 중 — zenoh_bridge 정리"
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
}
trap cleanup EXIT INT TERM

echo "[rviz.sh] zenoh-bridge-ros2dds 시작 (-> tcp/192.168.0.112:7447)"
zenoh-bridge-ros2dds -e tcp/192.168.0.112:7447 &
PIDS+=($!)

sleep 2

echo "[rviz.sh] rviz2 시작"
rviz2 -d uel_rover.rviz
