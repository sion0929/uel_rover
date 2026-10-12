"""px4_rover_landing.launch.py 의 계산 부분 (ROS 비의존, 단위시험 대상)."""

import os

from spacek_drone_sim.drone_params import AIRFRAME_ID, MODEL_NAME

WORLD_NAME = 'rover_landing_px4'
GROUND_START = (-1.5, 0.0)       # 로버 뒤 지면
SPAWN_GAP = 0.005                # 발 바닥을 이만큼 띄워 스폰


def home_env(lat, lon, alt):
    """PX4_HOME_LAT/LON/ALT. 셋 다 비면 {} (월드 기본 위치), 일부만 주면 오류."""
    vals = [str(v).strip() for v in (lat, lon, alt)]
    if not any(vals):
        return {}
    if not all(vals):
        raise ValueError('lat, lon, alt 는 셋 다 주거나 셋 다 비워야 합니다 (받은 값: %s)' % vals)
    for v in vals:
        float(v)          # 숫자가 아니면 ValueError
    return dict(zip(('PX4_HOME_LAT', 'PX4_HOME_LON', 'PX4_HOME_ALT'), vals))


def px4_env(home):
    env = {'PX4_GZ_STANDALONE': '1',
           'PX4_SYS_AUTOSTART': str(AIRFRAME_ID),
           'PX4_GZ_MODEL_NAME': MODEL_NAME,
           'PX4_GZ_WORLD': WORLD_NAME}
    env.update(home)
    return env


def floor_top_local(platform_root):
    """플랫폼 원점 기준 바닥판 윗면 높이 (floor_col 상자)."""
    col = platform_root.find("model/link[@name='floor']/collision[@name='floor_col']")
    if col is None:
        raise ValueError('플랫폼 SDF 에 floor_col 이 없습니다')
    z = float((col.findtext('pose') or '0 0 0').split()[2])
    return z + float(col.findtext('geometry/box/size').split()[2]) / 2.0


def drone_spawn_pose(mode, rover_z, origin, floor_top, gear_bottom_z):
    """드론 base_link 스폰 위치. origin = 로버 base_link 기준 플랫폼 원점 (x, y, z, yaw), 로버 yaw 0."""
    if mode == 'ground':
        return GROUND_START[0], GROUND_START[1], SPAWN_GAP - gear_bottom_z
    if mode == 'platform':
        top = rover_z + origin[2] + floor_top
        return origin[0], origin[1], top + SPAWN_GAP - gear_bottom_z
    raise ValueError('drone_start 는 ground 또는 platform 입니다 (받은 값: %s)' % mode)


def static_tf_args(xyz, q, parent, child):
    vals = list(xyz) + list(q)
    keys = ('--x', '--y', '--z', '--qx', '--qy', '--qz', '--qw')
    args = []
    for k, v in zip(keys, vals):
        args += [k, '%.6f' % float(v)]
    return args + ['--frame-id', parent, '--child-frame-id', child]


def join_paths(paths, old):
    return os.pathsep.join(list(paths) + ([old] if old else []))
