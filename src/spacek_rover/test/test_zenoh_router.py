"""zenoh_router 시험: rmw_zenoh_cpp 일 때만 라우터를 띄우고, 노드는 라우터를 기다린다."""

import socket

from spacek_rover.zenoh_router import CHECK_ATTEMPTS, router_plan, router_running

ZENOH = {'RMW_IMPLEMENTATION': 'rmw_zenoh_cpp'}


def test_other_rmw_needs_nothing():
    assert router_plan({}, router_up=False) == (False, {})
    assert router_plan({'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp'}, router_up=False) == (False, {})


def test_zenoh_without_router_starts_one_and_nodes_wait():
    assert router_plan(ZENOH, router_up=False) == (
        True, {'ZENOH_ROUTER_CHECK_ATTEMPTS': CHECK_ATTEMPTS})


def test_zenoh_with_router_running_starts_no_second_one():
    assert router_plan(ZENOH, router_up=True) == (
        False, {'ZENOH_ROUTER_CHECK_ATTEMPTS': CHECK_ATTEMPTS})


def test_user_check_attempts_is_kept():
    env = dict(ZENOH, ZENOH_ROUTER_CHECK_ATTEMPTS='-1')
    assert router_plan(env, router_up=False) == (True, {})


def test_router_running_sees_listening_port():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        s.listen(1)
        port = s.getsockname()[1]
        assert router_running(port=port)
    assert not router_running(port=port)
