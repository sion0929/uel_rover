"""drone.yaml -> 드론 모델·PX4 기체 설정 유도값 (ROS 비의존, 단위시험 대상).

드론 좌표 = 기체 중심, +X 앞, Y 왼쪽, Z 위 (가제보 FLU). PX4 기체 설정은 FRD (y, z 부호 반대).
"""

import math

from spacek_drone_sim.pose_math import quat_from_rpy, quat_mul
import yaml

GRAVITY = 9.80665
MODEL_NAME = 'spacek_drone'
AIRFRAME_ID = 4901
# PX4 쿼드 X 순서: (링크, 앞 +1 / 뒤 -1, 왼쪽 +1 / 오른쪽 -1, 회전 방향)
ROTORS = (('rotor_0', 1, -1, 'ccw'),
          ('rotor_1', -1, 1, 'ccw'),
          ('rotor_2', 1, 1, 'cw'),
          ('rotor_3', -1, -1, 'cw'))
# 센서 좌표 (x 앞, y 왼쪽, z 위) -> 광학 좌표 (z 앞, x 오른쪽, y 아래)
OPTICAL_FROM_SENSOR = quat_from_rpy(-math.pi / 2, 0.0, -math.pi / 2)
CAMERAS = ('front_camera', 'down_camera')
SENSOR_LINK_MASS = 0.001      # flow_link, lidar_sensor_link 각각 (x500_flow 의 거리센서 링크와 같음)


def load_config(path):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def frd(xyz):
    """FLU 좌표 -> PX4 FRD 좌표 (0 은 -0 이 아닌 0)."""
    return float(xyz[0]), 0.0 - float(xyz[1]), 0.0 - float(xyz[2])


def camera_optical(cam):
    """카메라 설정 -> (xyz, 광학 좌표 쿼터니언) in base_link."""
    q_mount = quat_from_rpy(*[float(v) for v in cam['rpy_rad']])
    return tuple(float(v) for v in cam['xyz_m']), quat_mul(q_mount, OPTICAL_FROM_SENSOR)


def camera_hfov(cam):
    """실측 초점거리 [px] -> 수평 화각 [rad] (주점은 영상 가운데로 둔다)."""
    return 2.0 * math.atan(float(cam['width']) / 2.0 / float(cam['focal_px']))


def derive(cfg):
    mass = float(cfg['mass_kg'])
    rot = cfg['rotors']
    mot = cfg['motor']
    rotor_mass = float(rot['mass_kg'])
    base_mass = mass - 4 * rotor_mass - 2 * SENSOR_LINK_MASS
    if base_mass <= 0.0:
        raise ValueError('mass_kg (%.3f) 가 로터·센서 링크 질량 합보다 작습니다' % mass)
    kf = float(mot['motor_constant'])
    hover = math.sqrt(mass * GRAVITY / (4.0 * kf))
    max_rot = hover * math.sqrt(float(mot['thrust_to_weight']))
    # PX4 gz 브리지: 회전수 = MIN + 출력 x (MAX - MIN). 실비행 호버 출력이 나오도록 MIN 을 정한다.
    hover_u = float(mot['hover_actuator'])
    if not 0.0 < hover_u < 1.0:
        raise ValueError('hover_actuator (%.3f) 는 0 과 1 사이여야 합니다' % hover_u)
    min_rot = (hover - hover_u * max_rot) / (1.0 - hover_u)
    if not 0.0 <= min_rot < hover < max_rot:
        raise ValueError('0 <= 최소 회전수 (%.0f) < 호버링 회전수 (%.0f) < 최대 회전수 (%.0f) 이어야 합니다'
                         ' (thrust_to_weight / hover_actuator 확인)' % (min_rot, hover, max_rot))
    arm = float(rot['arm_xy_m'])
    z = float(rot['z_m'])
    km = float(mot['px4_km'])
    rotors = []
    for i, (name, fwd, left, direction) in enumerate(ROTORS):
        xyz = (fwd * arm, left * arm, z)
        rotors.append({'index': i, 'link': name, 'joint': name + '_joint', 'xyz': xyz,
                       'direction': direction, 'px4_xy': frd(xyz)[:2],
                       'px4_km': km if direction == 'ccw' else -km})
    sens = cfg['sensors']
    return {
        'mass': mass,
        'base_mass': base_mass,
        'base_com': tuple(float(v) for v in cfg['com_m']),
        'base_inertia': tuple(float(v) for v in cfg['inertia_kgm2']),
        'rotor_mass': rotor_mass,
        'rotor_inertia': tuple(float(v) for v in rot['inertia_kgm2']),
        'rotors': rotors,
        'hover_rot_velocity': hover,
        'max_rot_velocity': max_rot,
        'min_rot_velocity': min_rot,
        'hover_actuator': (hover - min_rot) / (max_rot - min_rot),
        'cameras': {name: camera_optical(sens[name]) for name in CAMERAS},
        'camera_hfov': {name: camera_hfov(sens[name]) for name in CAMERAS},
        'flow_xyz': tuple(float(v) for v in sens['flow']['xyz_m']),
        'lidar_xyz': tuple(float(v) for v in sens['lidar']['xyz_m']),
        'gear_bottom_z': float(cfg['gear_bottom_z_m']),
        'blades': int(rot['blades']),
        'px4_params': dict(cfg.get('px4_params') or {}),
    }
