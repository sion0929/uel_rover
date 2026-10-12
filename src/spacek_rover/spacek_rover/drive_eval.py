"""자동 주행 시험의 단계 정의와 판정 (ROS 비의존, 단위시험 대상).

샘플 = (t, x, y, z, roll, pitch, yaw): 실제 위치(/rover/odom_gt), 월드 좌표, m / rad.
"""

import math


def wrap_pi(a):
    """각도를 [-pi, pi) 로 정규화한다."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def quat_to_rpy(x, y, z, w):
    """쿼터니언 (x, y, z, w) 을 (roll, pitch, yaw) [rad] 로 바꾼다."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def unwrapped_change(angles):
    """연속 각도 샘플의 누적 변화 [rad] 를 구한다 (±pi 경계를 넘어도 이어서 더함)."""
    total = 0.0
    for a, b in zip(angles, angles[1:]):
        total += wrap_pi(b - a)
    return total


def displacement_in_start_frame(samples):
    """첫 샘플의 자세 기준으로 마지막 샘플까지의 (전진, 왼쪽) 변위 [m] 를 구한다."""
    x0, y0, yaw0 = samples[0][1], samples[0][2], samples[0][6]
    dx = samples[-1][1] - x0
    dy = samples[-1][2] - y0
    c, s = math.cos(yaw0), math.sin(yaw0)
    return c * dx + s * dy, -s * dx + c * dy


def ramp_distance(speed, duration, accel):
    """정지에서 accel 로 speed 까지 가속한 뒤 유지할 때 duration 동안의 이동 거리 [m]."""
    sign = math.copysign(1.0, speed)
    t_ramp = abs(speed) / accel
    if duration <= t_ramp:
        return sign * 0.5 * accel * duration ** 2
    return sign * (0.5 * accel * t_ramp ** 2 + abs(speed) * (duration - t_ramp))


def make_phases(p):
    """시험 단계 목록 [{'name', 'v', 'w', 'duration'}] 을 만든다 (p: drive_test 파라미터)."""
    return [
        {'name': 'settle', 'v': 0.0, 'w': 0.0, 'duration': p['settle_duration_s']},
        {'name': 'forward', 'v': p['forward_speed_mps'], 'w': 0.0,
         'duration': p['forward_duration_s']},
        {'name': 'spin', 'v': 0.0, 'w': p['spin_rate_radps'], 'duration': p['spin_duration_s']},
        {'name': 'arc', 'v': p['arc_speed_mps'], 'w': p['arc_rate_radps'],
         'duration': p['arc_duration_s']},
        {'name': 'stop', 'v': 0.0, 'w': 0.0, 'duration': p['stop_duration_s']},
    ]


def _tail(samples, window):
    t_end = samples[-1][0]
    return [s for s in samples if s[0] >= t_end - window]


def _check_settle(samples, p, wheel_speeds, checks, measured):
    last = samples[-1]
    tilt = max(max(abs(s[4]), abs(s[5])) for s in _tail(samples, p['stop_window_s']))
    measured['max_tilt_deg'] = math.degrees(tilt)
    checks['ride_height'] = abs(last[3] - p['ride_height_m']) <= p['settle_z_tol_m']
    checks['level'] = math.degrees(tilt) < p['settle_tilt_tol_deg']
    if wheel_speeds is not None:
        fastest = max((abs(v) for v in wheel_speeds), default=0.0)
        measured['max_wheel_speed'] = fastest
        checks['wheels_still'] = fastest < p['settle_wheel_speed_tol']


def _check_turn(phase, p, dyaw, checks, measured):
    # 명령 yaw(정지에서 각가속 한계로 램프)의 min_motion_fraction 이상 반시계로 돌아야 통과.
    # 잡음/미세 크립(dyaw > 0 만으로는 통과)과 "바퀴만 헛돌고 안 도는" 경우를 걸러낸다.
    expected = ramp_distance(phase['w'], phase['duration'], p['angular_accel_radps2'])
    measured['expected_yaw_deg'] = math.degrees(expected)
    checks['turns_ccw'] = dyaw > 0.0 and dyaw >= p['min_motion_fraction'] * expected


def _check_stop(samples, p, checks, measured):
    tail = _tail(samples, p['stop_window_s'])
    dt = tail[-1][0] - tail[0][0]
    dist = math.hypot(tail[-1][1] - tail[0][1], tail[-1][2] - tail[0][2])
    speed = dist / dt if dt > 0.0 else 0.0
    measured['end_speed_mps'] = speed
    checks['stopped'] = speed < p['stop_speed_tol_mps']


def evaluate_phase(phase, samples, p, odom_yaws=None, wheel_speeds=None):
    """단계 하나를 판정해 {'name', 'passed', 'checks', 'measured'} 를 돌려준다."""
    name = phase['name']
    if name not in ('settle', 'forward', 'spin', 'arc', 'stop'):
        raise ValueError('알 수 없는 단계: %s' % name)
    if len(samples) < 2:
        return {'name': name, 'passed': False, 'checks': {'enough_samples': False},
                'measured': {}}
    fwd, left = displacement_in_start_frame(samples)
    dyaw = unwrapped_change([s[6] for s in samples])
    measured = {'forward_m': fwd, 'left_m': left, 'yaw_change_deg': math.degrees(dyaw),
                'z_end_m': samples[-1][3], 'duration_s': samples[-1][0] - samples[0][0]}
    checks = {}
    if name == 'settle':
        _check_settle(samples, p, wheel_speeds, checks, measured)
    elif name == 'forward':
        expected = ramp_distance(phase['v'], phase['duration'], p['linear_accel_mps2'])
        measured['expected_forward_m'] = expected
        checks['distance'] = abs(fwd - expected) <= p['forward_distance_tol'] * abs(expected)
        checks['heading'] = abs(math.degrees(dyaw)) < p['forward_heading_tol_deg']
        checks['lateral'] = abs(left) < p['forward_lateral_tol_m']
    elif name == 'spin':
        _check_turn(phase, p, dyaw, checks, measured)
        checks['in_place'] = math.hypot(fwd, left) < p['spin_drift_tol_m']
        if odom_yaws and len(odom_yaws) >= 2:
            d_odom = unwrapped_change(odom_yaws)
            if abs(d_odom) > 1e-6:
                measured['yaw_ratio_gt_over_odom'] = dyaw / d_odom
    elif name == 'arc':
        _check_turn(phase, p, dyaw, checks, measured)
        expected_fwd = ramp_distance(phase['v'], phase['duration'], p['linear_accel_mps2'])
        measured['expected_forward_m'] = expected_fwd
        checks['moves_forward'] = fwd > 0.0 and fwd >= p['min_motion_fraction'] * expected_fwd
    else:
        _check_stop(samples, p, checks, measured)
    return {'name': name, 'passed': all(checks.values()), 'checks': checks,
            'measured': measured}


def summarize(results):
    """단계 결과 목록을 {'success': bool, 'phases': results} 로 묶는다."""
    return {'success': bool(results) and all(r['passed'] for r in results), 'phases': results}
