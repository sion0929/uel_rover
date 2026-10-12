"""rmw_zenoh_cpp 이면 launch 가 Zenoh 라우터를 노드보다 먼저 띄운다.

rmw_zenoh_cpp 노드는 라우터를 거쳐야 서로 찾는다. 라우터 없이 뜬 노드는 경고만 남기고 혼자 돌며,
나중에 라우터가 떠도 브리지 구독이 살아나지 않는다 (2026-10-12: plate_coupler 명령이 가제보에 닿지 않아
바닥만 내려가고 접이판 끝 핀이 슬롯에서 빠짐). 그래서 라우터를 먼저 띄우고, 노드는 라우터에 붙을 때까지
ZENOH_ROUTER_CHECK_ATTEMPTS 번(1 s 간격) 기다리게 한다. 다른 RMW 이면 아무것도 하지 않는다.
같은 파일이 drone_landing_platform/zenoh_router.py 에 있다 (고정 플랫폼은 spacek_rover 없이도 뜸,
test_zenoh_router_copy 가 같은지 검사).
"""

import os
import socket

from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

ROUTER_PORT = 7447               # rmw_zenohd 기본 포트 (rmw_zenoh_cpp 노드의 기본 접속 주소)
CHECK_ATTEMPTS = '30'            # 노드가 라우터를 기다리는 횟수 (1 s 간격)


def router_running(host='127.0.0.1', port=ROUTER_PORT, timeout=0.5):
    """이 PC 에 이미 라우터(포트 7447 대기)가 있으면 True."""
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def router_plan(env, router_up):
    """(라우터를 띄울지, 노드에 줄 환경변수). 사용자가 정한 ZENOH_ROUTER_CHECK_ATTEMPTS 는 그대로 둔다."""
    if env.get('RMW_IMPLEMENTATION', '') != 'rmw_zenoh_cpp':
        return False, {}
    extra = {}
    if 'ZENOH_ROUTER_CHECK_ATTEMPTS' not in env:
        extra['ZENOH_ROUTER_CHECK_ATTEMPTS'] = CHECK_ATTEMPTS
    return not router_up, extra


def router_actions():
    """[라우터 프로세스] + 노드용 환경변수. launch 액션 목록 맨 앞(노드보다 먼저)에 둔다."""
    start, extra = router_plan(os.environ, router_running())
    # ros2 run 을 거치면 launch 종료 신호가 래퍼에서 끊겨 라우터가 남으므로 실행 파일을 직접 띄운다
    actions = [Node(package='rmw_zenoh_cpp', executable='rmw_zenohd', name='zenoh_router',
                    output='screen')] if start else []
    return actions + [SetEnvironmentVariable(k, v) for k, v in extra.items()]
