"""launch 계산 시험 (ROS 없이 실행)."""

import ast
import os
import xml.etree.ElementTree as ET

import pytest
from spacek_drone_sim.launch_tools import (drone_spawn_pose, floor_top_local, home_env,
                                           join_paths, px4_env, static_tf_args, WORLD_NAME)

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH = os.path.join(PKG_DIR, 'launch', 'px4_rover_landing.launch.py')
PLATFORM_SDF = os.path.join(os.path.dirname(PKG_DIR), 'drone_landing_platform', 'models',
                            'landing_platform', 'model.sdf')


def test_home_env_requires_all_or_none():
    assert home_env('', '', '') == {}
    assert home_env('36.6', '127.4', '50') == {'PX4_HOME_LAT': '36.6', 'PX4_HOME_LON': '127.4',
                                               'PX4_HOME_ALT': '50'}
    with pytest.raises(ValueError):
        home_env('36.6', '', '')
    with pytest.raises(ValueError):
        home_env('north', '127.4', '50')


def test_px4_env_attaches_to_existing_model():
    env = px4_env({'PX4_HOME_LAT': '1', 'PX4_HOME_LON': '2', 'PX4_HOME_ALT': '3'})
    assert env['PX4_GZ_STANDALONE'] == '1'
    assert env['PX4_SYS_AUTOSTART'] == '4901'
    assert env['PX4_GZ_MODEL_NAME'] == 'spacek_drone'
    assert env['PX4_GZ_WORLD'] == WORLD_NAME == 'rover_landing_px4'
    assert env['PX4_HOME_ALT'] == '3'
    assert 'PX4_SIM_MODEL' not in env


def test_floor_top_from_platform_sdf():
    if not os.path.isfile(PLATFORM_SDF):
        pytest.skip('drone_landing_platform 없음')
    assert floor_top_local(ET.parse(PLATFORM_SDF).getroot()) == pytest.approx(0.1136 + 0.0075)


def test_drone_spawn_on_ground_behind_rover():
    x, y, z = drone_spawn_pose('ground', 0.09325, (0.0, 0.0, 0.21954, 3.14159), 0.1211, -0.1644)
    assert (x, y) == (-1.5, 0.0)
    assert z + (-0.1644) == pytest.approx(0.005)


def test_drone_spawn_on_platform_clears_floor():
    rover_z, origin, floor_top, gear = 0.09325, (0.0, 0.0, 0.21954, 3.14159), 0.1211, -0.1644
    x, y, z = drone_spawn_pose('platform', rover_z, origin, floor_top, gear)
    assert (x, y) == pytest.approx((0.0, 0.0))
    assert z + gear == pytest.approx(rover_z + origin[2] + floor_top + 0.005)
    with pytest.raises(ValueError):
        drone_spawn_pose('air', rover_z, origin, floor_top, gear)


def test_static_tf_args():
    args = static_tf_args((0.1, 0.0, -0.04), (0.1, 0.2, 0.3, 0.9), 'base_link', 'cam_optical')
    assert args == ['--x', '0.100000', '--y', '0.000000', '--z', '-0.040000',
                    '--qx', '0.100000', '--qy', '0.200000', '--qz', '0.300000', '--qw', '0.900000',
                    '--frame-id', 'base_link', '--child-frame-id', 'cam_optical']


def test_join_paths_keeps_order_and_old_value():
    assert join_paths(['a', 'b'], '') == os.pathsep.join(['a', 'b'])
    assert join_paths(['a'], 'old') == os.pathsep.join(['a', 'old'])


def _exit_handlers(tree):
    out = {}
    for n in ast.walk(tree):
        if isinstance(n, ast.Call) and getattr(n.func, 'id', '') == 'OnProcessExit':
            kw = {k.arg: k.value for k in n.keywords}
            target = getattr(kw.get('target_action'), 'id', None)
            out[target] = {x.id for x in ast.walk(kw['on_exit']) if isinstance(x, ast.Name)}
    return out


def test_launch_spawns_drone_after_rover_and_px4_after_drone():
    # 가제보가 늦게 떠도 드론이 로버보다 먼저 스폰되면 안 된다 (drone_start:=platform 에서 겹침)
    handlers = _exit_handlers(ast.parse(open(LAUNCH, encoding='utf-8').read()))
    assert 'spawn_drone' in handlers.get('spawn_rover', set())
    assert {'TimerAction', 'px4_proc'} <= handlers.get('spawn_drone', set())


def test_launch_declares_expected_arguments():
    tree = ast.parse(open(LAUNCH, encoding='utf-8').read())
    names = {n.args[0].value for n in ast.walk(tree)
             if isinstance(n, ast.Call) and getattr(n.func, 'id', '') == 'DeclareLaunchArgument'}
    assert names == {'gui', 'px4', 'px4_dir', 'px4_shell', 'px4_delay', 'drone_start', 'lat',
                     'lon', 'alt', 'visual', 'gz_verbosity', 'spawn_delay', 'xrce_agent',
                     'marker_probe', 'markers_config'}
