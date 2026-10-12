"""SDF/XML/YAML 정적 검사와 런치 시 SDF 수정 도구 시험 (Gazebo 없이 실행 가능)."""

import glob
import math
import os
import struct
import xml.etree.ElementTree as ET

import pytest
from spacek_rover.rover_params import derive, load_config
from spacek_rover.sdf_tools import find_link, patch_rover, prepare_runtime_model
import yaml

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_DIR = os.path.join(PKG_DIR, 'models', 'spacek_rover')
SDF_PATH = os.path.join(MODEL_DIR, 'model.sdf')
BASE_STL = os.path.join(MODEL_DIR, 'meshes', 'base.stl')
WHEEL_STL = os.path.join(MODEL_DIR, 'meshes', 'wheel.stl')
WORLD_PATH = os.path.join(PKG_DIR, 'worlds', 'rover_test.sdf')
XML_FILES = (glob.glob(os.path.join(PKG_DIR, 'models', '*', '*.sdf'))
             + glob.glob(os.path.join(PKG_DIR, 'models', '*', '*.config'))
             + glob.glob(os.path.join(PKG_DIR, 'worlds', '*.sdf'))
             + [os.path.join(PKG_DIR, 'package.xml')])
YAML_FILES = glob.glob(os.path.join(PKG_DIR, 'config', '*.yaml'))
WHEEL_NAMES = ('front_left', 'front_right', 'rear_left', 'rear_right')
DIFF_DRIVE = 'gz::sim::systems::DiffDrive'
ODOM_PUB = 'gz::sim::systems::OdometryPublisher'


def _root():
    return ET.parse(SDF_PATH).getroot()


def _cfg():
    return load_config(os.path.join(PKG_DIR, 'config', 'rover.yaml'))


def _floats(text):
    return [float(v) for v in text.split()]


def _plugin(root, name):
    found = [p for p in root.iter('plugin') if p.get('name') == name]
    assert len(found) == 1, name
    return found[0]


@pytest.mark.parametrize('path', XML_FILES)
def test_xml_wellformed(path):
    ET.parse(path)


@pytest.mark.parametrize('path', YAML_FILES)
def test_yaml_wellformed(path):
    with open(path, encoding='utf-8') as f:
        assert yaml.safe_load(f) is not None


def test_links_and_joints():
    root = _root()
    links = {lk.get('name') for lk in root.iter('link')}
    assert links == {'base_link'} | {w + '_wheel' for w in WHEEL_NAMES}
    joints = {j.get('name'): j for j in root.iter('joint')}
    assert set(joints) == {w + '_wheel_joint' for w in WHEEL_NAMES}
    for w in WHEEL_NAMES:
        j = joints[w + '_wheel_joint']
        assert j.get('type') == 'revolute'
        assert j.find('parent').text == 'base_link'
        assert j.find('child').text == w + '_wheel'
        assert _floats(j.find('axis/xyz').text) == [0.0, 1.0, 0.0]


def test_collisions_are_primitives():
    for col in _root().iter('collision'):
        assert col.find('geometry/mesh') is None, col.get('name')


def test_wheel_collision_is_cylinder_along_y():
    root = _root()
    for w in WHEEL_NAMES:
        cols = find_link(root, w + '_wheel').findall('collision')
        assert len(cols) == 1
        assert cols[0].find('geometry/cylinder') is not None
        pose = _floats(cols[0].find('pose').text)
        assert pose[3] == pytest.approx(math.pi / 2, abs=1e-6)
        assert pose[4:] == pytest.approx([0.0, 0.0])


def test_wheel_friction_aligned_with_axle():
    """fdir1 = 충돌 좌표 z(바퀴 축) -> mu = 측면, mu2 = 전후. DART 는 두 면 중 작은 mu 를 쓴다."""
    root = _root()
    p = derive(_cfg())
    for w in WHEEL_NAMES:
        ode = find_link(root, w + '_wheel').find('collision/surface/friction/ode')
        assert _floats(ode.find('fdir1').text) == [0.0, 0.0, 1.0]
        assert float(ode.find('mu').text) == pytest.approx(p['mu_lat'])
        assert float(ode.find('mu2').text) == pytest.approx(p['mu_long'])


def test_plugins_present_and_wired():
    root = _root()
    dd = _plugin(root, DIFF_DRIVE)
    left = [e.text for e in dd.findall('left_joint')]
    right = [e.text for e in dd.findall('right_joint')]
    assert left == ['front_left_wheel_joint', 'rear_left_wheel_joint']
    assert right == ['front_right_wheel_joint', 'rear_right_wheel_joint']
    jsp = _plugin(root, 'gz::sim::systems::JointStatePublisher')
    assert {e.text for e in jsp.findall('joint_name')} == {w + '_wheel_joint' for w in WHEEL_NAMES}
    _plugin(root, ODOM_PUB)


def test_odometry_topics_distinct():
    root = _root()
    dd = _plugin(root, DIFF_DRIVE)
    op = _plugin(root, ODOM_PUB)
    assert dd.find('odom_topic').text != op.find('odom_topic').text
    assert dd.find('tf_topic').text != op.find('tf_topic').text
    assert op.find('dimensions').text.strip() == '3'


def test_platform_mount_frame():
    frames = {f.get('name'): f for f in _root().iter('frame')}
    mount = frames['platform_mount']
    assert mount.get('attached_to') == 'base_link'
    assert _floats(mount.find('pose').text) == pytest.approx([0.034659, 0, 0.214540, 0, 0, 0])


def test_default_sdf_matches_yaml():
    """model.sdf 기본값이 rover.yaml 계산값과 같은지 본다 (launch 없이 열어도 같은 로버)."""
    root = _root()
    p = derive(_cfg())
    base = find_link(root, 'base_link')
    assert float(base.find('inertial/mass').text) == pytest.approx(p['base_mass'])
    inertia = base.find('inertial/inertia')
    got = [float(inertia.find(t).text) for t in ('ixx', 'iyy', 'izz')]
    assert got == pytest.approx(list(p['base_inertia']), rel=1e-5)
    assert _floats(base.find('inertial/pose').text)[:3] == pytest.approx(list(p['base_com']))
    joints = {j.get('name'): j for j in root.iter('joint')}
    for w in WHEEL_NAMES:
        link = find_link(root, w + '_wheel')
        assert _floats(link.find('pose').text)[:3] == pytest.approx(list(p['wheel_positions'][w]))
        assert float(link.find('inertial/mass').text) == pytest.approx(p['wheel_mass'])
        lim = joints[w + '_wheel_joint'].find('axis/limit')
        assert float(lim.find('effort').text) == pytest.approx(p['wheel_effort'])
        assert float(lim.find('velocity').text) == pytest.approx(p['wheel_velocity'], rel=1e-6)
    dd = _plugin(root, DIFF_DRIVE)
    for key in ('wheel_separation', 'wheel_radius', 'max_linear_velocity',
                'max_angular_velocity'):
        assert float(dd.find(key).text) == pytest.approx(p[key], rel=1e-5)


def test_patch_rover_applies_yaml_changes():
    cfg = _cfg()
    cfg['mass']['total_kg'] = 24.0
    cfg['wheel']['mu_lateral'] = 0.6
    cfg['wheel']['mu_longitudinal'] = 0.8
    cfg['motor']['gear_ratio'] = 2.0
    p = derive(cfg)
    root = _root()
    patch_rover(root, p)
    assert float(find_link(root, 'base_link').find('inertial/mass').text) == pytest.approx(20.0)
    col = find_link(root, 'front_left_wheel').find('collision')
    assert float(col.find('surface/friction/ode/mu').text) == pytest.approx(0.6)
    assert float(col.find('surface/friction/ode/mu2').text) == pytest.approx(0.8)
    joint = [j for j in root.iter('joint') if j.get('name') == 'rear_right_wheel_joint'][0]
    assert float(joint.find('axis/limit/effort').text) == pytest.approx(10.6)
    dd = _plugin(root, DIFF_DRIVE)
    assert float(dd.find('min_linear_velocity').text) == pytest.approx(-p['max_linear_velocity'])


def test_prepare_runtime_model_box_fallback(tmp_path):
    info = prepare_runtime_model(os.path.join(PKG_DIR, 'models'), str(tmp_path / 'models'),
                                 derive(_cfg()), 'box')
    assert info['visual'] == 'box'
    names = [v.get('name') for v in ET.parse(info['sdf']).getroot().iter('visual')]
    assert names and not any(n.endswith('_mesh_visual') for n in names)


def test_prepare_runtime_model_mesh(tmp_path):
    info = prepare_runtime_model(os.path.join(PKG_DIR, 'models'), str(tmp_path / 'models'),
                                 derive(_cfg()), 'mesh')
    assert info['visual'] == 'mesh'
    names = [v.get('name') for v in ET.parse(info['sdf']).getroot().iter('visual')]
    assert names and not any(n.endswith('_box_visual') for n in names)


def test_prepare_runtime_model_rejects_unknown_mode(tmp_path):
    with pytest.raises(ValueError):
        prepare_runtime_model(os.path.join(PKG_DIR, 'models'), str(tmp_path / 'models'),
                              derive(_cfg()), 'stl')


def _stl_points(path):
    with open(path, 'rb') as f:
        data = f.read()
    if len(data) >= 84:
        n = struct.unpack_from('<I', data, 80)[0]
        if 84 + 50 * n == len(data):
            pts = []
            for i in range(n):
                vals = struct.unpack_from('<9f', data, 84 + 50 * i + 12)
                pts.extend((vals[0:3], vals[3:6], vals[6:9]))
            return pts
    pts = []
    for line in data.decode('utf-8', 'ignore').splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[0] == 'vertex':
            pts.append(tuple(float(v) for v in parts[1:]))
    return pts


def _bbox(points):
    return ([min(p[i] for p in points) for i in range(3)],
            [max(p[i] for p in points) for i in range(3)])


def _rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def _mesh_bbox_in_link(link, visual_name, points):
    vis = [v for v in link.findall('visual') if v.get('name') == visual_name][0]
    pose = _floats(vis.find('pose').text)
    scale = _floats(vis.find('geometry/mesh/scale').text)
    m = _rpy_matrix(*pose[3:])
    lo, hi = _bbox(points)
    corners = [(a, b, c) for a in (lo[0], hi[0]) for b in (lo[1], hi[1]) for c in (lo[2], hi[2])]
    moved = []
    for c in corners:
        v = [c[i] * scale[i] for i in range(3)]
        moved.append([sum(m[i][j] * v[j] for j in range(3)) + pose[i] for i in range(3)])
    return _bbox(moved)


@pytest.mark.skipif(not os.path.isfile(BASE_STL), reason='base.stl 없음 (Task 1 미실행)')
def test_base_mesh_aligned_with_link():
    lo, hi = _mesh_bbox_in_link(find_link(_root(), 'base_link'), 'base_mesh_visual',
                                _stl_points(BASE_STL))
    assert lo == pytest.approx([-0.2638862, -0.146, -0.0241372], abs=5e-4)
    assert hi == pytest.approx([0.2583838, 0.146, 0.2195403], abs=5e-4)


@pytest.mark.skipif(not os.path.isfile(WHEEL_STL), reason='wheel.stl 없음 (Task 1 미실행)')
@pytest.mark.parametrize('wheel', WHEEL_NAMES)
def test_wheel_mesh_centered_on_axle(wheel):
    link = find_link(_root(), wheel + '_wheel')
    lo, hi = _mesh_bbox_in_link(link, wheel + '_wheel_mesh_visual', _stl_points(WHEEL_STL))
    assert lo == pytest.approx([-0.08825, -0.0437922, -0.08825], abs=5e-4)
    assert hi == pytest.approx([0.08825, 0.0437922, 0.08825], abs=5e-4)


def test_bridge_topics_match_sdf():
    with open(os.path.join(PKG_DIR, 'config', 'bridge.yaml'), encoding='utf-8') as f:
        bridge = {e['ros_topic_name']: e for e in yaml.safe_load(f)}
    root = _root()
    dd = _plugin(root, DIFF_DRIVE)
    op = _plugin(root, ODOM_PUB)
    assert bridge['/rover/cmd_vel']['gz_topic_name'] == dd.find('topic').text
    assert bridge['/rover/cmd_vel']['direction'] == 'ROS_TO_GZ'
    assert bridge['/rover/odom']['gz_topic_name'] == dd.find('odom_topic').text
    assert bridge['/rover/odom_gt']['gz_topic_name'] == op.find('odom_topic').text
    assert bridge['/tf']['gz_topic_name'] == dd.find('tf_topic').text
    world = ET.parse(WORLD_PATH).getroot().find('world').get('name')
    model = root.find('model').get('name')
    expected = '/world/%s/model/%s/joint_state' % (world, model)
    assert bridge['/rover/joint_states']['gz_topic_name'] == expected


def test_world_physics_matches_landing_world():
    world = ET.parse(WORLD_PATH).getroot().find('world')
    physics = world.find('physics')
    assert physics.get('type') == 'dart'
    assert float(physics.find('max_step_size').text) == pytest.approx(0.001)
    plugins = {p.get('name') for p in world.iter('plugin')}
    assert 'gz::sim::systems::Physics' in plugins
    ground = [m for m in world.iter('model') if m.get('name') == 'ground_plane'][0]
    mu = ground.find('link/collision/surface/friction/ode/mu')
    assert float(mu.text) == pytest.approx(1.0)


def test_mesh_uris_match_files_case_sensitively():
    """리눅스(대소문자 구분)에서도 model:// 메시 URI 가 실제 파일 이름과 같아야 한다."""
    files = set(os.listdir(os.path.join(MODEL_DIR, 'meshes')))
    for uri in _root().iter('uri'):
        name = uri.text.rsplit('/', 1)[-1]
        assert name in files, '%s 가 meshes/ 에 정확한 이름으로 없음 (있는 파일: %s)' % (name, files)
