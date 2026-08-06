#!/bin/bash
# ─────────────────────────────────────────────────────────────────
#  UEL Rover 수동/포그라운드 실행 (개발·디버그용)
#  전제: setup.sh를 먼저 1회 실행해 둘 것.
# ─────────────────────────────────────────────────────────────────
set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/jazzy/setup.bash
source "$REPO_DIR/install/setup.bash"
export ROS_DOMAIN_ID=0

PIDS=()
cleanup() {
    echo "[run.sh] 종료 중 — 백그라운드 프로세스 정리"
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
}
trap cleanup EXIT INT TERM

echo "[run.sh] zenoh-bridge-ros2dds 시작 (tcp/0.0.0.0:7447)"
ROS_DOMAIN_ID=0 zenoh-bridge-ros2dds -l tcp/0.0.0.0:7447 &
PIDS+=($!)

echo "[run.sh] micro_ros_agent 시작 (/dev/ttyACM0)"
(
    source "$REPO_DIR/microros_ws/install/setup.bash"
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp MICROROS_DISABLE_SHM=1 ROS_DOMAIN_ID=0 \
        exec ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
) &
PIDS+=($!)

sleep 2

echo "[run.sh] uel_rover_control mapping.launch.py 시작 (포그라운드)"
ros2 launch uel_rover_control mapping.launch.py
