"""drone_params 유도값 시험 (ROS 없이 실행)."""

import math
import os

import pytest
from spacek_drone_sim.drone_params import derive, load_config
from spacek_drone_sim.pose_math import quat_rotate

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = os.path.join(PKG_DIR, 'config', 'drone.yaml')


def _cfg():
    return load_config(CFG)


def test_hover_matches_real_flight_and_motor_range():
    """실비행 hover_thrust_estimate 0.35 (09-21 로그) 를 맞추도록 최소 회전수를 정한다."""
    p = derive(_cfg())
    hover = math.sqrt(1.9 * 9.80665 / (4 * 8.54858e-06))
    assert p['hover_rot_velocity'] == pytest.approx(738.18, abs=0.05)
    w_max = hover * math.sqrt(5.0)
    assert p['max_rot_velocity'] == pytest.approx(w_max)
    assert p['hover_actuator'] == pytest.approx(0.35)
    assert p['min_rot_velocity'] == pytest.approx((hover - 0.35 * w_max) / 0.65)
    assert (p['hover_rot_velocity'] - p['min_rot_velocity']) / (
        p['max_rot_velocity'] - p['min_rot_velocity']) == pytest.approx(0.35)
    assert 0.0 < p['min_rot_velocity'] < p['hover_rot_velocity']


@pytest.mark.parametrize('hover_actuator', [0.0, 1.0, 0.9])
def test_impossible_hover_actuator_is_rejected(hover_actuator):
    cfg = _cfg()
    cfg['motor']['hover_actuator'] = hover_actuator      # 0.9 -> 최소 회전수가 음수
    with pytest.raises(ValueError):
        derive(cfg)


def test_camera_fov_from_real_focal_length():
    """실측 초점거리 (AVISDrone cam_*_info.yaml, 1280x720): 전방 727.00, 하방 594.01 px."""
    fov = derive(_cfg())['camera_hfov']
    assert fov['front_camera'] == pytest.approx(2.0 * math.atan(640.0 / 727.00))
    assert fov['down_camera'] == pytest.approx(2.0 * math.atan(640.0 / 594.01))


def test_total_mass_is_measured_value():
    p = derive(_cfg())
    assert p['base_mass'] + 4 * p['rotor_mass'] + 2 * 0.001 == pytest.approx(1.9)


def test_rotor_order_is_px4_quad_x():
    r = derive(_cfg())['rotors']
    a = 0.1329
    flu = [(a, -a), (-a, a), (a, a), (-a, -a)]
    frd = [(a, a), (-a, -a), (a, -a), (-a, a)]
    assert [x['xyz'][:2] for x in r] == [pytest.approx(v) for v in flu]
    assert [x['direction'] for x in r] == ['ccw', 'ccw', 'cw', 'cw']
    assert [x['px4_xy'] for x in r] == [pytest.approx(v) for v in frd]
    assert [x['px4_km'] for x in r] == pytest.approx([0.05, 0.05, -0.05, -0.05])
    assert [x['joint'] for x in r] == ['rotor_%d_joint' % i for i in range(4)]


def test_down_camera_looks_down_and_image_x_is_right():
    _xyz, q = derive(_cfg())['cameras']['down_camera']
    assert quat_rotate(q, (0.0, 0.0, 1.0)) == pytest.approx((0.0, 0.0, -1.0), abs=1e-6)
    assert quat_rotate(q, (1.0, 0.0, 0.0)) == pytest.approx((0.0, -1.0, 0.0), abs=1e-6)


def test_front_camera_optical_axis_from_mount_pitch():
    cfg = _cfg()
    cfg['sensors']['front_camera']['rpy_rad'] = [0.0, math.radians(30.0), 0.0]
    _xyz, q = derive(cfg)['cameras']['front_camera']
    assert quat_rotate(q, (0.0, 0.0, 1.0)) == pytest.approx(
        (math.cos(math.radians(30.0)), 0.0, -0.5), abs=1e-6)
    assert quat_rotate(q, (1.0, 0.0, 0.0)) == pytest.approx((0.0, -1.0, 0.0), abs=1e-6)


def test_weak_motor_is_rejected():
    cfg = _cfg()
    cfg['motor']['thrust_to_weight'] = 1.0
    with pytest.raises(ValueError):
        derive(cfg)


def test_mass_below_parts_is_rejected():
    cfg = _cfg()
    cfg['mass_kg'] = 0.05
    with pytest.raises(ValueError):
        derive(cfg)
