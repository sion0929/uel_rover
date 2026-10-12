"""PX4 기체 설정 파일 시험: drone.yaml 과 값 일치, PX4 경로 안내, 설치 스크립트."""

import os
import shutil
import subprocess

import pytest
from spacek_drone_sim.drone_params import derive, load_config
from spacek_drone_sim.px4_airframe import (AIRFRAME_NAME, airframe_text, install_airframe,
                                           parse_airframe, px4_paths)

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = os.path.join(PKG_DIR, 'config', 'drone.yaml')
COMMITTED = os.path.join(PKG_DIR, 'px4', AIRFRAME_NAME)
SETUP = os.path.join(PKG_DIR, 'scripts', 'setup_px4.sh')


def _params(cfg=None):
    return parse_airframe(airframe_text(derive(cfg or load_config(CFG))))


def test_committed_file_matches_generator():
    with open(COMMITTED, encoding='utf-8') as f:
        assert f.read() == airframe_text(derive(load_config(CFG)))


def test_rotor_geometry_in_px4_frd():
    p = _params()
    a = 0.1329
    expected = [(a, a, 0.05), (-a, -a, 0.05), (a, -a, -0.05), (-a, a, -0.05)]
    for i, (px, py, km) in enumerate(expected):
        assert p['CA_ROTOR%d_PX' % i] == pytest.approx(px)
        assert p['CA_ROTOR%d_PY' % i] == pytest.approx(py)
        assert p['CA_ROTOR%d_KM' % i] == pytest.approx(km)


def test_motor_range_and_hover_follow_drone_yaml():
    d = derive(load_config(CFG))
    p = _params()
    for i in range(1, 5):
        assert p['SIM_GZ_EC_MAX%d' % i] == round(d['max_rot_velocity'])
        assert p['SIM_GZ_EC_MIN%d' % i] == round(d['min_rot_velocity'])
    assert p['MPC_THR_HOVER'] == pytest.approx(d['hover_actuator'], abs=0.005)


def test_ekf_is_gps_denied_like_real_drone_at_sensor_positions():
    """실기체: GPS 없음, 광류 + 거리센서, 높이 기준 = 거리센서 (항상 사용)."""
    cfg = load_config(CFG)
    p = _params(cfg)
    assert (p['EKF2_GPS_CTRL'], p['EKF2_OF_CTRL'], p['EKF2_RNG_CTRL']) == (0, 1, 2)
    assert (p['SYS_HAS_GPS'], p['SIM_GZ_EN_GPS'], p['EKF2_HGT_REF']) == (0, 0, 2)
    fx, fy, fz = cfg['sensors']['flow']['xyz_m']
    assert (p['EKF2_OF_POS_X'], p['EKF2_OF_POS_Y'], p['EKF2_OF_POS_Z']) == pytest.approx(
        (fx, -fy, -fz), abs=5e-4)
    lx, ly, lz = cfg['sensors']['lidar']['xyz_m']
    assert (p['EKF2_RNG_POS_X'], p['EKF2_RNG_POS_Y'], p['EKF2_RNG_POS_Z']) == pytest.approx(
        (lx, -ly, -lz), abs=5e-4)


def test_real_drone_params_are_applied():
    """drone.yaml px4_params (실비행 로그 값) 가 모두 기체 설정에 들어간다."""
    cfg = load_config(CFG)
    p = _params(cfg)
    assert cfg['px4_params']['MC_ROLLRATE_P'] == pytest.approx(0.14)
    assert cfg['px4_params']['IMU_GYRO_CUTOFF'] == pytest.approx(30)
    for name, value in cfg['px4_params'].items():
        assert p[name] == pytest.approx(value), name


def test_offboard_is_excepted_from_rc_loss_failsafe():
    # 설계 6장: 시뮬에 조종기 없음 -> 오프보드 시험이 RC 상실 페일세이프로 끊기지 않게
    assert _params()['COM_RCL_EXCEPT'] == 4


def test_heavier_drone_changes_motor_range():
    cfg = load_config(CFG)
    cfg['mass_kg'] = 2.5
    assert _params(cfg)['SIM_GZ_EC_MAX1'] > _params()['SIM_GZ_EC_MAX1']


def test_airframe_extends_x500():
    text = airframe_text(derive(load_config(CFG)))
    assert '. ${R}etc/init.d-posix/airframes/4001_gz_x500' in text
    assert 'PX4_SIM_MODEL=${PX4_SIM_MODEL:=spacek_drone}' in text


def test_px4_paths_explains_missing_build(tmp_path):
    with pytest.raises(FileNotFoundError) as e:
        px4_paths(str(tmp_path))
    assert 'setup_px4.sh' in str(e.value)


def test_install_airframe_into_px4_build(tmp_path):
    build = tmp_path / 'build' / 'px4_sitl_default'
    (build / 'bin').mkdir(parents=True)
    (build / 'bin' / 'px4').write_text('')
    (build / 'etc' / 'init.d-posix' / 'airframes').mkdir(parents=True)
    path = install_airframe(str(tmp_path), 'x\n')
    assert os.path.basename(path) == AIRFRAME_NAME
    assert open(path, encoding='utf-8').read() == 'x\n'


def test_setup_script_stops_on_other_px4_version(tmp_path):
    bash, git = shutil.which('bash'), shutil.which('git')
    if bash is None or git is None:
        pytest.skip('bash/git 없음')
    repo = tmp_path / 'PX4-Autopilot'
    repo.mkdir()
    ident = ['-c', 'user.email=t@t', '-c', 'user.name=t']
    for cmd in (['init', '-q'], ident + ['commit', '-q', '--allow-empty', '-m', 'x'],
                ['tag', 'v1.16.0']):
        subprocess.run([git] + cmd, cwd=str(repo), check=True)
    env = dict(os.environ, PX4_DIR=repo.as_posix(), WS_DIR=(tmp_path / 'ws').as_posix())
    r = subprocess.run([bash, SETUP], env=env, capture_output=True, text=True,
                       encoding='utf-8', errors='replace')
    out = r.stdout + r.stderr
    assert r.returncode != 0
    assert 'v1.16.0' in out and 'v1.17.0' in out
    assert 'ubuntu.sh' not in out            # 다른 버전이면 의존성 설치·빌드 전에 멈춘다


def test_setup_script_pins_versions_and_parses():
    text = open(SETUP, encoding='utf-8').read()
    for pin in ('v1.17.0', 'release/1.17', 'v2.4.3'):
        assert pin in text
    bash = shutil.which('bash')
    if bash is None:
        pytest.skip('bash 없음')
    assert subprocess.run([bash, '-n', SETUP]).returncode == 0
