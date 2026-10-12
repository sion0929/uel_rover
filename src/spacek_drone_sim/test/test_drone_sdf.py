"""드론 SDF 시험: PX4 이름 규칙, 모터, 질량, drone.yaml 과 템플릿 일치."""

import copy
import math
import os

import pytest
from spacek_drone_sim.drone_params import derive, load_config
from spacek_drone_sim.drone_sdf import GZ_NS, parse_sdf, patch_drone, write_drone_sdf

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = os.path.join(PKG_DIR, 'config', 'drone.yaml')
MODEL_DIR = os.path.join(PKG_DIR, 'models', 'spacek_drone')
SDF = os.path.join(MODEL_DIR, 'model.sdf')
PX4_BASE_SENSORS = {'imu_sensor': 'imu', 'air_pressure_sensor': 'air_pressure',
                    'magnetometer_sensor': 'magnetometer', 'navsat_sensor': 'navsat',
                    'front_camera': 'camera', 'down_camera': 'camera'}


def _model():
    return parse_sdf(SDF).getroot().find('model')


def _link(model, name):
    return model.find("link[@name='%s']" % name)


def _num(text):
    try:
        return [float(t) for t in text.split()]
    except (AttributeError, ValueError):
        return None


def _same(a, b, path='sdf'):
    assert a.tag == b.tag, path
    assert a.attrib == b.attrib, path
    na, nb = _num(a.text), _num(b.text)
    if na and nb:
        assert na == pytest.approx(nb, rel=1e-6, abs=1e-9), '%s: %s != %s' % (path, a.text, b.text)
    else:
        assert (a.text or '').strip() == (b.text or '').strip(), path
    ca, cb = list(a), list(b)
    assert len(ca) == len(cb), path
    for x, y in zip(ca, cb):
        _same(x, y, '%s/%s[%s]' % (path, x.tag, x.get('name', '')))


def test_template_matches_drone_yaml():
    tree = parse_sdf(SDF)
    cfg = load_config(CFG)
    patched = patch_drone(copy.deepcopy(tree.getroot()), derive(cfg), cfg)
    _same(tree.getroot(), patched)


def test_px4_sensor_names():
    model = _model()
    base = _link(model, 'base_link')
    assert {s.get('name'): s.get('type') for s in base.findall('sensor')} == PX4_BASE_SENSORS
    flow = _link(model, 'flow_link')
    of = flow.find("sensor[@name='optical_flow']")
    assert of.get('type') == 'custom' and of.get('{%s}type' % GZ_NS) == 'optical_flow'
    assert flow.find("sensor[@name='flow_camera']").get('type') == 'camera'
    lidar = _link(model, 'lidar_sensor_link').find("sensor[@name='lidar']")
    assert lidar.get('type') == 'gpu_lidar'
    assert lidar.findtext('pose').split() == ['0', '0', '0', '3.14', '0', '0']


def test_props_have_three_blades():
    """실기 3엽 프롭 (사용자 2026-10-12): 날개 3개를 120° 간격, 길이 = 프롭 반지름."""
    model = _model()
    cfg = load_config(CFG)
    radius = cfg['rotors']['prop_diameter_m'] / 2.0
    for i in range(4):
        link = _link(model, 'rotor_%d' % i)
        blades = [v for v in link.findall('visual')
                  if v.get('name').startswith('rotor_%d_blade' % i)]
        assert len(blades) == 3
        yaws = sorted(round(math.degrees(_num(b.findtext('pose'))[5])) % 360 for b in blades)
        assert yaws == [0, 120, 240]
        for b in blades:
            assert _num(b.findtext('geometry/box/size'))[0] == pytest.approx(radius)
        assert not [v for v in link.findall('visual') if v.get('name').endswith('_prop_visual')]


def test_cameras_and_range_sensor_follow_real_hardware():
    """카메라 화각 = 실측 초점거리, H-Flow 거리 0.08~30 m (실기 UAVCAN_RNG_MIN/MAX)."""
    model = _model()
    p = derive(load_config(CFG))
    base = _link(model, 'base_link')
    for name in ('front_camera', 'down_camera'):
        cam = base.find("sensor[@name='%s']" % name)
        assert float(cam.findtext('camera/horizontal_fov')) == pytest.approx(
            p['camera_hfov'][name], rel=1e-6)
        assert cam.findtext('camera/image/width') == '1280'
        assert cam.findtext('camera/image/height') == '720'
        assert float(cam.findtext('update_rate')) == pytest.approx(30.0)
    lidar = _link(model, 'lidar_sensor_link').find("sensor[@name='lidar']")
    assert float(lidar.findtext('ray/range/min')) == pytest.approx(0.08)
    assert float(lidar.findtext('ray/range/max')) == pytest.approx(30.0)


def test_motor_plugins_follow_px4_order():
    model = _model()
    p = derive(load_config(CFG))
    plugins = [pl for pl in model.findall('plugin')
               if pl.get('name') == 'gz::sim::systems::MulticopterMotorModel']
    assert len(plugins) == 4
    for r, pl in zip(p['rotors'], plugins):
        assert pl.findtext('motorNumber') == str(r['index'])
        assert pl.findtext('jointName') == r['joint']
        assert pl.findtext('linkName') == r['link']
        assert pl.findtext('turningDirection') == r['direction']
        assert pl.findtext('commandSubTopic') == 'command/motor_speed'
        assert pl.findtext('motorType') == 'velocity'
        assert float(pl.findtext('maxRotVelocity')) == pytest.approx(p['max_rot_velocity'])
        joint = model.find("joint[@name='%s']" % r['joint'])
        assert joint.get('type') == 'revolute'
        assert joint.findtext('parent') == 'base_link' and joint.findtext('axis/xyz') == '0 0 1'


def test_total_mass_is_measured_value():
    total = sum(float(link.findtext('inertial/mass')) for link in _model().findall('link'))
    assert total == pytest.approx(1.9)


def test_written_sdf_keeps_gz_namespace(tmp_path):
    dst = write_drone_sdf(SDF, str(tmp_path / 'spacek_drone.sdf'), load_config(CFG))
    text = open(dst, encoding='utf-8').read()
    assert 'xmlns:gz="%s"' % GZ_NS in text
    assert 'gz:type="optical_flow"' in text


def test_patch_follows_config_changes():
    cfg = load_config(CFG)
    cfg['mass_kg'] = 2.2
    cfg['sensors']['front_camera']['width'] = 640
    root = patch_drone(copy.deepcopy(parse_sdf(SDF).getroot()), derive(cfg), cfg)
    model = root.find('model')
    assert float(_link(model, 'base_link').findtext('inertial/mass')) == pytest.approx(
        derive(cfg)['base_mass'])
    width = "sensor[@name='front_camera']/camera/image/width"
    assert _link(model, 'base_link').findtext(width) == '640'


def test_missing_element_is_reported():
    root = copy.deepcopy(parse_sdf(SDF).getroot())
    base = root.find("model/link[@name='base_link']")
    base.remove(base.find('inertial'))
    with pytest.raises(ValueError):
        patch_drone(root, derive(load_config(CFG)), load_config(CFG))


def test_mesh_uris_exist_case_sensitively():
    names = os.listdir(os.path.join(MODEL_DIR, 'meshes'))
    for uri in _model().iter('uri'):
        assert uri.text.startswith('model://spacek_drone/meshes/')
        assert uri.text.rsplit('/', 1)[1] in names, uri.text
