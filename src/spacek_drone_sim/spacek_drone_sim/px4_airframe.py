"""PX4 기체 설정 파일 (4901_gz_spacek_drone) 생성·스테이징과 PX4 SITL 경로 (ROS 비의존).

기본 3a launch는 drone.yaml로 생성해 PX4 빌드 트리에 설치한다. 실험용
px4_log_dir 또는 px4_overrides_file이 있으면 실행별 작업 폴더의 etc 사본에 넣는다.
패키지 안의 px4/4901_gz_spacek_drone 은 같은 생성기의 출력 (참고·수동 설치용):
  python3 -m spacek_drone_sim.px4_airframe config/drone.yaml px4/4901_gz_spacek_drone
"""

import math
import os
import re
import shutil
import sys

from spacek_drone_sim.drone_params import AIRFRAME_ID, derive, frd, load_config, MODEL_NAME

AIRFRAME_NAME = '%d_gz_%s' % (AIRFRAME_ID, MODEL_NAME)
PX4_TAG = 'v1.17.0'
SITL_BUILD = os.path.join('build', 'px4_sitl_default')
PARAM_NAME = re.compile(r'^[A-Z][A-Z0-9_]*$')


def normalize_overrides(raw):
    """Validate values before placing them in the PX4 /bin/sh airframe script."""
    if not isinstance(raw, dict):
        raise ValueError('px4 overrides must be a mapping')
    result = {}
    for name, value in raw.items():
        if not isinstance(name, str) or not PARAM_NAME.fullmatch(name):
            raise ValueError('invalid PX4 parameter name: %r' % name)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError('%s must have a numeric value' % name)
        if not math.isfinite(value):
            raise ValueError('%s must have a finite value' % name)
        result[name] = value
    return result


def _value(v):
    """PX4 param set 값 표기: 정수는 정수로, 실수는 짧게."""
    v = float(v)
    return '%d' % v if v.is_integer() else '%g' % v


def airframe_text(p, overrides=None):
    overrides = normalize_overrides(overrides if overrides is not None else {})
    out = ['#!/bin/sh',
           '#',
           '# @name Gazebo spacek_drone (대회 기체: Pixhawk 6X, H-Flow, 전방 60도 + 하방 카메라)',
           '#',
           '# @type Quadrotor',
           '#',
           '# spacek_drone_sim 이 drone.yaml 로 만든 파일 (PX4 %s). 직접 고치지 말고 drone.yaml 을 고칠 것.'
           % PX4_TAG,
           '',
           'PX4_SIM_MODEL=${PX4_SIM_MODEL:=%s}' % MODEL_NAME,
           '',
           '. ${R}etc/init.d-posix/airframes/4001_gz_x500',
           '']
    for r in p['rotors']:
        i = r['index']
        out += ['param set-default CA_ROTOR%d_PX %.4f' % (i, r['px4_xy'][0]),
                'param set-default CA_ROTOR%d_PY %.4f' % (i, r['px4_xy'][1]),
                'param set-default CA_ROTOR%d_KM %.3f' % (i, r['px4_km'])]
    out.append('')
    for i in range(1, 5):
        out += ['param set-default SIM_GZ_EC_MIN%d %d' % (i, round(p['min_rot_velocity'])),
                'param set-default SIM_GZ_EC_MAX%d %d' % (i, round(p['max_rot_velocity']))]
    out += ['',
            '# 실비행 호버 출력 (09-21 로그 hover_thrust_estimate)',
            'param set-default MPC_THR_HOVER %.2f' % p['hover_actuator'],
            '',
            '# 가제보 센서 (실기 프로필 GPS 융합 없음; 실험 덮어쓰기일 때 활성화)',
            'param set-default SIM_GZ_EN_GPS %d' % bool(overrides.get('EKF2_GPS_CTRL', 0)),
            'param set-default SIM_GZ_EN_BARO 1',
            'param set-default SIM_GZ_EN_FLOW 1',
            'param set-default SIM_GZ_EN_LIDAR 1',
            '',
            '# 실기체 파라미터 (drone.yaml px4_params, 09-21 비행 로그)']
    out += ['param set-default %s %s' % (name, _value(v)) for name, v in p['px4_params'].items()]
    out += ['',
            '# 시뮬에 조종기 없음: 오프보드가 조종기 상실 페일세이프로 끊기지 않게 (설계 6장)',
            'param set-default COM_RCL_EXCEPT 4']
    for prefix, xyz in (('EKF2_OF_POS', p['flow_xyz']), ('EKF2_RNG_POS', p['lidar_xyz'])):
        for axis, v in zip('XYZ', frd(xyz)):
            out.append('param set-default %s_%s %.3f' % (prefix, axis, v))
    if overrides:
        out += ['', '# air_ground_loc experiment.yaml: 부팅 시 적용할 실험 전용 값']
        out += ['param set %s %s' % (name, _value(value))
                for name, value in sorted(overrides.items())]
    return '\n'.join(out) + '\n'


def parse_airframe(text):
    params = {}
    for line in text.splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[:2] == ['param', 'set-default']:
            params[parts[2]] = float(parts[3])
    return params


def px4_paths(px4_dir):
    root = os.path.abspath(os.path.expanduser(px4_dir))
    build = os.path.join(root, SITL_BUILD)
    paths = {'root': root,
             'binary': os.path.join(build, 'bin', 'px4'),
             'airframes': os.path.join(build, 'etc', 'init.d-posix', 'airframes'),
             'gz_plugins': os.path.join(build, 'src', 'modules', 'simulation', 'gz_plugins')}
    missing = [paths[k] for k in ('binary', 'airframes') if not os.path.exists(paths[k])]
    if missing:
        raise FileNotFoundError(
            'PX4 SITL 빌드를 찾을 수 없습니다 (px4_dir=%s): %s\n'
            '먼저 share/spacek_drone_sim/scripts/setup_px4.sh 를 실행하거나 px4_dir:=<경로> 를 주세요.'
            % (root, ', '.join(missing)))
    return paths


def install_airframe(px4_dir, text):
    path = os.path.join(px4_paths(px4_dir)['airframes'], AIRFRAME_NAME)
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write(text)
    os.chmod(path, 0o755)
    return path


def stage_airframe(px4_dir, runtime_root, text):
    """Copy the small PX4 etc tree and add this airframe without editing PX4."""
    source = os.path.join(px4_paths(px4_dir)['root'], SITL_BUILD, 'etc')
    destination = os.path.join(runtime_root, 'etc')
    shutil.copytree(source, destination)
    path = os.path.join(destination, 'init.d-posix', 'airframes', AIRFRAME_NAME)
    with open(path, 'w', encoding='utf-8', newline='\n') as stream:
        stream.write(text)
    os.chmod(path, 0o755)
    return path


def main(argv=None):
    args = sys.argv[1:] if argv is None else argv
    if len(args) != 2:
        print('사용: python3 -m spacek_drone_sim.px4_airframe <drone.yaml> <출력 파일>')
        return 2
    with open(args[1], 'w', encoding='utf-8', newline='\n') as f:
        f.write(airframe_text(derive(load_config(args[0]))))
    print('작성:', args[1])
    return 0


if __name__ == '__main__':
    sys.exit(main())
