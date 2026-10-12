"""rover_params 계산 시험 (ROS/Gazebo 없이 실행 가능)."""

import math
import os

import pytest
from spacek_rover.rover_params import (box_inertia, cylinder_inertia, derive, load_config,
                                       spawn_height, wheel_positions)

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG_PATH = os.path.join(PKG_DIR, 'config', 'rover.yaml')
G = 9.80665
_W = cylinder_inertia(1.0, 0.0877, 0.0875844)


@pytest.fixture
def cfg():
    return load_config(CFG_PATH)


def test_mass_budget_is_20kg(cfg):
    p = derive(cfg)
    assert p['base_mass'] == pytest.approx(16.0)
    assert p['wheel_mass'] == pytest.approx(1.0)
    assert p['base_mass'] + 4 * p['wheel_mass'] == pytest.approx(20.0)


def test_base_inertia_matches_spec(cfg):
    ixx, iyy, izz = derive(cfg)['base_inertia']
    assert ixx == pytest.approx(0.0735053, rel=1e-5)
    assert iyy == pytest.approx(0.2498053, rel=1e-5)
    assert izz == pytest.approx(0.2829667, rel=1e-5)


def test_wheel_inertia_matches_spec(cfg):
    i_axis, i_perp = derive(cfg)['wheel_inertia']
    assert i_axis == pytest.approx(0.00384565, rel=1e-5)
    assert i_perp == pytest.approx(0.00256208, rel=1e-5)


@pytest.mark.parametrize('inertia', [
    box_inertia(16.0, 0.415, 0.2, 0.123),
    box_inertia(3.0, 1.0, 0.01, 0.01),
    (_W[1], _W[0], _W[1]),
])
def test_inertia_is_physical(inertia):
    a, b, c = inertia
    assert min(a, b, c) > 0.0
    assert a + b >= c and b + c >= a and a + c >= b


def test_base_com_default(cfg):
    assert derive(cfg)['base_com'] == pytest.approx((-0.00028, 0.0, 0.12578))


def test_com_offset_is_added(cfg):
    cfg['base']['com_offset_m'] = [0.01, 0.0, -0.05]
    assert derive(cfg)['base_com'] == pytest.approx((0.00972, 0.0, 0.07578))


def test_real_rover_wheel_radius_and_firmware_speed_limit(cfg):
    """실기 로버: 바퀴 반지름 0.0877 m (1 m 직진 오도메트리 0.997 m), 펌웨어 MAX_RPM 50."""
    p = derive(cfg)
    assert p['wheel_radius'] == pytest.approx(0.0877)
    assert p['wheel_velocity'] == pytest.approx(50.0 * 2.0 * math.pi / 60.0)    # 5.236 rad/s
    assert p['max_linear_velocity'] == pytest.approx(0.4591961, rel=1e-6)      # 약 0.46 m/s
    assert p['max_angular_velocity'] == pytest.approx(2.4194679, rel=1e-6)


def test_motor_torque_limit(cfg):
    assert derive(cfg)['wheel_effort'] == pytest.approx(5.3)


def test_gear_ratio_scales_torque_only(cfg):
    """MAX_RPM 은 바퀴 회전수이므로 기어비는 토크 상한에만 들어간다."""
    cfg['motor']['gear_ratio'] = 2.0
    p = derive(cfg)
    assert p['wheel_effort'] == pytest.approx(10.6)
    assert p['wheel_velocity'] == pytest.approx(50.0 * 2.0 * math.pi / 60.0)


def test_wheel_positions_match_spec(cfg):
    pos = wheel_positions(cfg)
    assert pos['front_left'] == pytest.approx((0.235, 0.1897922, 0.0))
    assert pos['front_right'] == pytest.approx((0.235, -0.1897922, 0.0))
    assert pos['rear_left'] == pytest.approx((-0.235, 0.1897922, 0.0))
    assert pos['rear_right'] == pytest.approx((-0.235, -0.1897922, 0.0))


def test_spawn_height_above_ground(cfg):
    assert spawn_height(cfg) == pytest.approx(0.0927)


def test_spin_in_place_torque_is_feasible(cfg):
    p = derive(cfg)
    normal = (p['base_mass'] + 4 * p['wheel_mass']) * G / 4.0
    needed = p['mu_long'] * normal * p['wheel_radius']
    assert needed == pytest.approx(3.9, abs=0.05)
    assert needed < p['wheel_effort']


def test_skid_turn_is_friction_feasible(cfg):
    """DART 마찰 피라미드: 전후 마찰 x 반트랙 > 측면 마찰 x 반축간 이어야 제자리 선회가 된다."""
    p = derive(cfg)
    half_track = p['wheel_separation'] / 2.0
    half_base = cfg['wheel']['wheelbase_m'] / 2.0
    margin = p['mu_long'] * half_track / (p['mu_lat'] * half_base)
    assert p['turn_margin'] == pytest.approx(margin)
    assert margin > 1.2


def test_equal_friction_cannot_turn(cfg):
    cfg['wheel']['mu_lateral'] = cfg['wheel']['mu_longitudinal']
    assert derive(cfg)['turn_margin'] < 1.0


def _setter(path, value):
    def mutate(c):
        node = c
        for key in path[:-1]:
            node = node[key]
        node[path[-1]] = value
    return mutate


@pytest.mark.parametrize('mutate', [
    _setter(('mass', 'wheel_kg'), 5.0),
    _setter(('mass', 'total_kg'), -1.0),
    _setter(('motor', 'gear_ratio'), 0.0),
    _setter(('motor', 'max_wheel_rpm'), 0.0),
    _setter(('motor', 'stall_torque_nm'), -10.6),
    _setter(('motor', 'wheels_per_motor'), 0),
    _setter(('wheel', 'radius_m'), 0.0),
    _setter(('wheel', 'mu_lateral'), 0.0),
    _setter(('base', 'inertia_box_m'), [0.415, 0.2]),
])
def test_invalid_config_raises(cfg, mutate):
    mutate(cfg)
    with pytest.raises(ValueError):
        derive(cfg)
