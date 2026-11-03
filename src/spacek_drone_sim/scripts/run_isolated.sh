#!/bin/bash
# 명령을 루프백 전용 네트워크 네임스페이스 안에서 실행한다 (root 불필요: user + net 네임스페이스).
# 같은 LAN 에 실기체·다른 사람 시뮬이 있을 때 PX4 SITL·XRCE Agent·Gazebo·ROS 2 가 서로 섞이지 않게 한다.
# 안에는 "lo" 만 있다. 시뮬과 그 시뮬을 쓰는 노드는 모두 같은 격리 셸 안에서 띄워야 서로 보인다:
#   run_isolated.sh bash -c 'source install/setup.bash; ros2 launch spacek_drone_sim px4_rover_landing.launch.py & ...; wait'
#   run_isolated.sh bash        # 격리 셸 하나를 열고 그 안에서 여러 명령 (tmux 를 그 안에서 켜면 편함)
# 이미 떠 있는 격리 시뮬에 다른 터미널로 들어가기: nsenter -t <px4 또는 gz PID> -U -n --preserve-credentials bash
set -e
if [ $# -eq 0 ]; then echo "usage: run_isolated.sh <command> [args...]" >&2; exit 2; fi
exec unshare --map-root-user --net bash -c '
  ip link set lo up
  ip link set lo multicast on
  ip route add 224.0.0.0/4 dev lo 2>/dev/null || true
  export GZ_IP=127.0.0.1
  exec "$@"' isolated "$@"
