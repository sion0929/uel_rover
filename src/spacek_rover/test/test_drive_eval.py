"""drive_eval 판정 로직 시험 (ROS/Gazebo 없이 실행 가능)."""

import math

import pytest
from spacek_rover.drive_eval import (displacement_in_start_frame, evaluate_phase, make_phases,
                                     quat_to_rpy, ramp_distance, summarize, unwrapped_change,
                                     wrap_pi)

P = {
    'settle_duration_s': 2.0, 'forward_speed_mps': 0.2, 'forward_duration_s': 5.0,
    'spin_rate_radps': 0.8, 'spin_duration_s': 4.0, 'arc_speed_mps': 0.15,
    'arc_rate_radps': 0.3, 'arc_duration_s': 4.0, 'stop_duration_s': 2.0,
    'ride_height_m': 0.08825, 'settle_z_tol_m': 0.005, 'settle_tilt_tol_deg': 1.0,
    'settle_wheel_speed_tol': 0.05, 'forward_distance_tol': 0.10,
    'forward_heading_tol_deg': 2.0, 'forward_lateral_tol_m': 0.03, 'spin_drift_tol_m': 0.10,
    'stop_speed_tol_mps': 0.01, 'stop_window_s': 0.5, 'linear_accel_mps2': 0.5,
    'angular_accel_radps2': 2.0, 'min_motion_fraction': 0.3,
}
DT = 0.02
Z0 = 0.08825


def _phase(name):
    return [ph for ph in make_phases(P) if ph['name'] == name][0]


def _straight(speed, duration, accel, yaw0=0.0, z=Z0, scale=1.0):
    out = []
    for i in range(int(round(duration / DT)) + 1):
        t = i * DT
        d = scale * ramp_distance(speed, t, accel)
        out.append((t, d * math.cos(yaw0), d * math.sin(yaw0), z, 0.0, 0.0, yaw0))
    return out


def _spin(rate, duration, yaw0=0.0):
    out = []
    for i in range(int(round(duration / DT)) + 1):
        t = i * DT
        out.append((t, 0.0, 0.0, Z0, 0.0, 0.0, wrap_pi(yaw0 + rate * t)))
    return out


def test_wrap_pi():
    assert wrap_pi(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)
    assert wrap_pi(-0.5) == pytest.approx(-0.5)


def test_quat_to_rpy_yaw_90():
    s = math.sin(math.pi / 4)
    roll, pitch, yaw = quat_to_rpy(0.0, 0.0, s, s)
    assert (roll, pitch) == pytest.approx((0.0, 0.0), abs=1e-9)
    assert yaw == pytest.approx(math.pi / 2)


def test_unwrapped_change_crosses_pi():
    assert unwrapped_change([3.0, -3.0]) == pytest.approx(2 * math.pi - 6.0)
    assert unwrapped_change([s[6] for s in _spin(0.8, 4.0)]) == pytest.approx(3.2, abs=1e-6)


def test_displacement_in_start_frame_uses_start_heading():
    samples = [(0.0, 1.0, 1.0, 0.0, 0.0, 0.0, math.pi / 2),
               (1.0, 1.0, 2.0, 0.0, 0.0, 0.0, math.pi / 2)]
    assert displacement_in_start_frame(samples) == pytest.approx((1.0, 0.0), abs=1e-9)


def test_ramp_distance():
    assert ramp_distance(0.2, 5.0, 0.5) == pytest.approx(0.96)
    assert ramp_distance(0.2, 0.2, 0.5) == pytest.approx(0.01)
    assert ramp_distance(0.0, 2.0, 0.5) == pytest.approx(0.0)


def test_forward_passes_with_ideal_motion():
    res = evaluate_phase(_phase('forward'), _straight(0.2, 5.0, 0.5), P)
    assert res['passed'], res
    assert res['measured']['expected_forward_m'] == pytest.approx(0.96)


def test_forward_fails_when_short():
    res = evaluate_phase(_phase('forward'), _straight(0.2, 5.0, 0.5, scale=0.85), P)
    assert not res['checks']['distance']


def test_forward_with_spawn_yaw():
    res = evaluate_phase(_phase('forward'), _straight(0.2, 5.0, 0.5, yaw0=2.5), P)
    assert res['passed'], res


def test_forward_detects_lateral_drift():
    samples = _straight(0.2, 5.0, 0.5)
    t, x, y, z, r, p, yaw = samples[-1]
    samples[-1] = (t, x, y + 0.05, z, r, p, yaw)
    assert not evaluate_phase(_phase('forward'), samples, P)['checks']['lateral']


def test_spin_unwraps_past_pi():
    odom = [s[6] for s in _spin(1.0, 4.0, yaw0=3.0)]
    res = evaluate_phase(_phase('spin'), _spin(0.8, 4.0, yaw0=3.0), P, odom_yaws=odom)
    assert res['passed'], res
    assert res['measured']['yaw_change_deg'] == pytest.approx(math.degrees(3.2), abs=1e-3)
    assert res['measured']['yaw_ratio_gt_over_odom'] == pytest.approx(0.8)


def test_spin_clockwise_fails():
    res = evaluate_phase(_phase('spin'), _spin(-0.8, 4.0), P)
    assert not res['checks']['turns_ccw']


def test_settle_checks_ride_height_and_wheels():
    still = _straight(0.0, 2.0, 0.5)
    ok = evaluate_phase(_phase('settle'), still, P, wheel_speeds=[0.0] * 4)
    assert ok['passed'], ok
    low = evaluate_phase(_phase('settle'), _straight(0.0, 2.0, 0.5, z=0.07), P)
    assert not low['checks']['ride_height']
    turning = evaluate_phase(_phase('settle'), still, P, wheel_speeds=[0.3, 0.0, 0.0, 0.0])
    assert not turning['checks']['wheels_still']


def test_stop_detects_motion():
    assert evaluate_phase(_phase('stop'), _straight(0.0, 2.0, 0.5), P)['passed']
    moving = evaluate_phase(_phase('stop'), _straight(0.05, 2.0, 100.0), P)
    assert not moving['checks']['stopped']


def test_arc_needs_forward_and_ccw():
    samples = []
    for i in range(201):
        t = i * DT
        yaw = 0.3 * t
        samples.append((t, 0.5 * math.sin(yaw), 0.5 * (1 - math.cos(yaw)), Z0, 0.0, 0.0, yaw))
    assert evaluate_phase(_phase('arc'), samples, P)['passed']


def test_spin_stuck_rover_with_noise_fails():
    samples = [(i * DT, 0.0, 0.0, Z0, 0.0, 0.0, 1e-4 * (-1) ** i) for i in range(200)]
    samples.append((200 * DT, 0.0, 0.0, Z0, 0.0, 0.0, 2e-4))
    res = evaluate_phase(_phase('spin'), samples, P)
    assert not res['checks']['turns_ccw']


def test_arc_driving_straight_with_creep_fails():
    samples = [s[:6] + (0.001 * s[0] / 4.0,) for s in _straight(0.15, 4.0, 0.5)]
    res = evaluate_phase(_phase('arc'), samples, P)
    assert not res['checks']['turns_ccw']


def test_arc_barely_moving_fails():
    samples = []
    for i in range(201):
        t = i * DT
        yaw = 0.3 * t
        samples.append((t, 0.01 * math.sin(yaw), 0.01 * (1 - math.cos(yaw)), Z0, 0.0, 0.0, yaw))
    assert not evaluate_phase(_phase('arc'), samples, P)['checks']['moves_forward']


def test_too_few_samples_fails():
    res = evaluate_phase(_phase('forward'), [(0.0, 0.0, 0.0, Z0, 0.0, 0.0, 0.0)], P)
    assert not res['passed']


def test_unknown_phase_raises():
    with pytest.raises(ValueError):
        evaluate_phase({'name': 'jump', 'v': 0.0, 'w': 0.0, 'duration': 1.0}, _spin(0.0, 1.0), P)


def test_summarize():
    ok = {'name': 'stop', 'passed': True, 'checks': {}, 'measured': {}}
    bad = {'name': 'spin', 'passed': False, 'checks': {}, 'measured': {}}
    assert summarize([])['success'] is False
    assert summarize([ok])['success'] is True
    assert summarize([ok, bad])['success'] is False
