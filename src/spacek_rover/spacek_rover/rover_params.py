"""로버 파라미터 계산 (ROS 비의존, 단위시험 대상).

config/rover.yaml 의 질량 예산, 바퀴, 모터 값에서 SDF 에 넣을 질량/관성/조인트 한계와
DiffDrive 속도 한계를 계산한다. 단위: m, kg, s, rad, N m.
"""

import math

import yaml

WHEELS = ('front_left', 'front_right', 'rear_left', 'rear_right')


def load_config(path):
    """YAML 파일을 읽어 dict 로 돌려준다."""
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def _positive(name, value):
    v = float(value)
    if not v > 0.0:
        raise ValueError('%s 는 양수여야 합니다: %r' % (name, value))
    return v


def _vec3(name, values):
    vals = [float(v) for v in values]
    if len(vals) != 3:
        raise ValueError('%s 는 값 3개여야 합니다: %r' % (name, values))
    return vals


def box_inertia(mass, lx, ly, lz):
    """균일 직육면체의 주관성 (ixx, iyy, izz) [kg m^2] 을 돌려준다."""
    k = float(mass) / 12.0
    return (k * (ly * ly + lz * lz), k * (lx * lx + lz * lz), k * (lx * lx + ly * ly))


def cylinder_inertia(mass, radius, width):
    """축이 링크 y 인 균일 원통의 (i_axis, i_perp) [kg m^2] 를 돌려준다."""
    r2 = float(radius) ** 2
    return (0.5 * mass * r2, mass * (3.0 * r2 + float(width) ** 2) / 12.0)


def wheel_positions(cfg):
    """바퀴 조인트 위치 {이름: (x, y, z)} [m] 를 돌려준다 (base_link 기준, x 전방, y 왼쪽)."""
    w = cfg['wheel']
    hx = _positive('wheel.wheelbase_m', w['wheelbase_m']) / 2.0
    hy = _positive('wheel.separation_m', w['separation_m']) / 2.0
    return {
        'front_left': (hx, hy, 0.0),
        'front_right': (hx, -hy, 0.0),
        'rear_left': (-hx, hy, 0.0),
        'rear_right': (-hx, -hy, 0.0),
    }


def spawn_height(cfg, margin=0.005):
    """스폰 시 base_link 높이 [m] = 바퀴 반경 + margin (지면 z = 0)."""
    return _positive('wheel.radius_m', cfg['wheel']['radius_m']) + float(margin)


def derive(cfg):
    """rover.yaml dict 에서 SDF/DiffDrive 에 넣을 값을 계산한다.

    잘못된 값(0 이하, 바퀴 질량 합 >= 총질량, 값 개수 틀림)은 ValueError 로 알린다.
    """
    mass = cfg['mass']
    total = _positive('mass.total_kg', mass['total_kg'])
    wheel_mass = _positive('mass.wheel_kg', mass['wheel_kg'])
    base_mass = total - len(WHEELS) * wheel_mass
    if base_mass <= 0.0:
        raise ValueError('mass.total_kg (%g) 는 바퀴 %d 개 질량 합 (%g) 보다 커야 합니다'
                         % (total, len(WHEELS), len(WHEELS) * wheel_mass))
    base = cfg['base']
    com = [a + b for a, b in zip(_vec3('base.com_m', base['com_m']),
                                 _vec3('base.com_offset_m', base['com_offset_m']))]
    lx, ly, lz = (_positive('base.inertia_box_m', v)
                  for v in _vec3('base.inertia_box_m', base['inertia_box_m']))
    wheel = cfg['wheel']
    radius = _positive('wheel.radius_m', wheel['radius_m'])
    width = _positive('wheel.width_m', wheel['width_m'])
    separation = _positive('wheel.separation_m', wheel['separation_m'])
    motor = cfg['motor']
    stall = _positive('motor.stall_torque_nm', motor['stall_torque_nm'])
    rpm = _positive('motor.max_wheel_rpm', motor['max_wheel_rpm'])
    gear = _positive('motor.gear_ratio', motor['gear_ratio'])
    per_motor = int(_positive('motor.wheels_per_motor', motor['wheels_per_motor']))
    drive = cfg['drive']
    accel_lin = _positive('drive.max_linear_accel_mps2', drive['max_linear_accel_mps2'])
    accel_ang = _positive('drive.max_angular_accel_radps2', drive['max_angular_accel_radps2'])
    wheel_velocity = rpm * 2.0 * math.pi / 60.0      # 펌웨어 MAX_RPM 은 바퀴 회전수
    v_max = wheel_velocity * radius
    mu_long = _positive('wheel.mu_longitudinal', wheel['mu_longitudinal'])
    mu_lat = _positive('wheel.mu_lateral', wheel['mu_lateral'])
    half_base = _positive('wheel.wheelbase_m', wheel['wheelbase_m']) / 2.0
    return {
        'base_mass': base_mass,
        'wheel_mass': wheel_mass,
        'base_com': tuple(com),
        'base_inertia': box_inertia(base_mass, lx, ly, lz),
        'wheel_inertia': cylinder_inertia(wheel_mass, radius, width),
        'wheel_radius': radius,
        'wheel_width': width,
        'wheel_separation': separation,
        'wheel_positions': wheel_positions(cfg),
        'wheel_effort': stall * gear / per_motor,
        'wheel_velocity': wheel_velocity,
        'max_linear_velocity': v_max,
        'max_angular_velocity': 2.0 * v_max / separation,
        'max_linear_accel': accel_lin,
        'max_angular_accel': accel_ang,
        'odom_rate_hz': _positive('drive.odom_rate_hz', drive['odom_rate_hz']),
        'mu_long': mu_long,
        'mu_lat': mu_lat,
        # > 1 이어야 제자리 선회 가능 (DART 마찰 피라미드: 전후 마찰 모멘트 / 측면 마찰 모멘트)
        'turn_margin': mu_long * (separation / 2.0) / (mu_lat * half_base),
    }
