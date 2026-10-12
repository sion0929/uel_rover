"""SDF/XML/YAML 정적 검사와 런치 시 SDF 수정 도구 시험 (Gazebo 없이 실행 가능)."""

import glob
import math
import os
import xml.etree.ElementTree as ET

from drone_landing_platform.sdf_tools import (patch_drone, patch_platform,
                                              prepare_runtime_models, spring_reference_m)
import pytest
import yaml

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
XML_FILES = (glob.glob(os.path.join(PKG_DIR, 'models', '*', '*.sdf'))
             + glob.glob(os.path.join(PKG_DIR, 'models', '*', '*.config'))
             + glob.glob(os.path.join(PKG_DIR, 'worlds', '*.sdf'))
             + [os.path.join(PKG_DIR, 'package.xml')])
YAML_FILES = glob.glob(os.path.join(PKG_DIR, 'config', '*.yaml'))


def _load(path):
    return ET.parse(path).getroot()


def _platform_cfg():
    with open(os.path.join(PKG_DIR, 'config', 'platform.yaml'), encoding='utf-8') as f:
        return yaml.safe_load(f)


@pytest.mark.parametrize('path', XML_FILES)
def test_xml_wellformed(path):
    _load(path)


@pytest.mark.parametrize('path', YAML_FILES)
def test_yaml_wellformed(path):
    with open(path, encoding='utf-8') as f:
        assert yaml.safe_load(f) is not None


def test_platform_structure():
    root = _load(os.path.join(PKG_DIR, 'models', 'landing_platform', 'model.sdf'))
    joints = {j.get('name'): j for j in root.iter('joint')}
    for name in ('frame_fixed', 'floor_joint', 'plate_px_joint', 'plate_mx_joint',
                 'plate_py_joint', 'plate_my_joint'):
        assert name in joints
    assert joints['floor_joint'].get('type') == 'prismatic'
    plugins = [p.get('name') for p in root.iter('plugin')]
    assert plugins.count('gz::sim::systems::JointPositionController') == 4
    assert 'gz::sim::systems::JointStatePublisher' in plugins
    for col in root.iter('collision'):
        assert col.find('geometry/mesh') is None, '메시 충돌은 사용하지 않음'


def test_drone_structure():
    root = _load(os.path.join(PKG_DIR, 'models', 'simple_drone', 'model.sdf'))
    names = [j.get('name') for j in root.iter('joint')]
    for name in ('drone_x', 'drone_y', 'drone_z', 'drone_yaw', 'drone_pitch', 'drone_roll'):
        assert name in names
    plugins = [p.get('name') for p in root.iter('plugin')]
    assert plugins.count('gz::sim::systems::ApplyJointForce') == 6


def test_spring_reference_sign():
    # d = 0 에서 위로 8.1 + 3.3 N -> q_ref = -0.0456 m
    q_ref = spring_reference_m(250.0, 8.1, 3.3)
    assert q_ref == pytest.approx(-0.0456)
    # 축 방향 스프링 힘 = -k (q - q_ref); 축이 -Z 이므로 위쪽 힘 = +k (q - q_ref) 의 부호 반대
    force_along_axis = -250.0 * (0.0 - q_ref)
    assert force_along_axis == pytest.approx(-11.4)


def test_static_sink_matches_solidworks():
    cfg = _platform_cfg()
    sus = cfg['suspension']
    k = sus['stiffness_n_per_m']
    g = 9.80665
    hanging = (0.486 + 4 * 0.176) * g          # Gazebo 에서 바닥 좌표에 걸리는 무게
    effective = hanging - sus['plate_guide_support_n']
    assert effective == pytest.approx(8.4, abs=0.1)
    up0 = sus['preload_n'] + sus['plate_guide_support_n']
    d_free = (hanging - up0) / k               # 드론 없음: 약 1 mm 처짐
    assert 0.0 < d_free < 0.002
    d_drone = (hanging + cfg['drone']['mass_kg'] * g - up0) / k
    assert d_drone == pytest.approx(0.075, abs=0.002)
    assert d_drone > sus['hard_stop_m']        # -> 하단 스톱(60.6 mm)에 닿는다


def test_patch_platform_and_drone():
    root = _load(os.path.join(PKG_DIR, 'models', 'landing_platform', 'model.sdf'))
    cfg = _platform_cfg()
    cfg['suspension']['damping_ns_per_m'] = 12.5
    patch_platform(root, cfg)
    fj = [j for j in root.iter('joint') if j.get('name') == 'floor_joint'][0]
    assert float(fj.find('axis/dynamics/damping').text) == pytest.approx(12.5)
    assert float(fj.find('axis/limit/upper').text) == pytest.approx(0.0606)
    droot = _load(os.path.join(PKG_DIR, 'models', 'simple_drone', 'model.sdf'))
    ratio = patch_drone(droot, 3.8)
    assert ratio == pytest.approx(2.0)


def test_prepare_runtime_models_box_fallback(tmp_path):
    dst = str(tmp_path / 'models')
    info = prepare_runtime_models(os.path.join(PKG_DIR, 'models'), dst, _platform_cfg(), 'box')
    assert info['platform_visual'] == 'box'
    root = _load(os.path.join(dst, 'landing_platform', 'model.sdf'))
    names = [v.get('name') for v in root.iter('visual')]
    assert not any(n.endswith('_mesh_visual') for n in names)
    assert any(n.endswith('_box_visual') for n in names)


def test_plate_geometry_consistency():
    # 플레이트 판 중심/각도가 힌지(107.2, 115.7) - 핀(212.7, 135.0) 선 위에 있는지
    root = _load(os.path.join(PKG_DIR, 'models', 'landing_platform', 'model.sdf'))
    link = [lk for lk in root.iter('link') if lk.get('name') == 'plate_px'][0]
    pose = [float(v) for v in link.find('collision/pose').text.split()]
    theta0 = math.asin(19.3 / 107.25)
    assert -pose[4] == pytest.approx(theta0, abs=1e-4)
    hx, hz = 0.1072, 0.1157
    assert pose[0] == pytest.approx(hx + 0.107250 / 2 * math.cos(theta0), abs=1e-4)
    assert pose[2] == pytest.approx(hz + 0.107250 / 2 * math.sin(theta0), abs=1e-4)


def test_patch_platform_sets_frame_inertial():
    root = _load(os.path.join(PKG_DIR, 'models', 'landing_platform', 'model.sdf'))
    patch_platform(root, _platform_cfg())
    frame = [lk for lk in root.iter('link') if lk.get('name') == 'frame'][0]
    inertial = frame.find('inertial')
    assert float(inertial.find('mass').text) == pytest.approx(1.15)
    com = [float(v) for v in inertial.find('pose').text.split()][:3]
    assert com == pytest.approx([0.0, 0.0, 0.0446])
    got = [float(inertial.find('inertia/' + t).text) for t in ('ixx', 'iyy', 'izz')]
    assert got == pytest.approx([0.0188, 0.0188, 0.0323])


def test_platform_total_mass_estimate():
    total = _platform_cfg()['frame']['mass_kg'] + 0.486 + 4 * 0.176
    assert total == pytest.approx(2.34, abs=0.01)
