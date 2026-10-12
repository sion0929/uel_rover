"""마커 시험: 배치, 면 안에 들어감, 메시 위에 보임, OBJ 방향, 플랫폼 SDF 비주얼, 그림·검출(OpenCV)."""

import copy
import math
import os
import shutil
import struct
import xml.etree.ElementTree as ET

import pytest
from spacek_drone_sim.markers import (add_marker_visuals, image_to_gray, layout,
                                      load_markers_config, marker_total_m, MARKER_URI, obj_text,
                                      prepare_platform_markers, write_marker_meshes)
from spacek_drone_sim.pose_math import mat_vec, rpy_to_matrix

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = os.path.join(PKG_DIR, 'config', 'markers.yaml')


def _platform_dir():
    sibling = os.path.join(os.path.dirname(PKG_DIR), 'drone_landing_platform', 'models',
                           'landing_platform')
    if os.path.isdir(sibling):
        return sibling
    ament = pytest.importorskip('ament_index_python.packages')
    return os.path.join(ament.get_package_share_directory('drone_landing_platform'), 'models',
                        'landing_platform')


def _platform_root():
    return ET.parse(os.path.join(_platform_dir(), 'model.sdf')).getroot()


def _cfg():
    return load_markers_config(CFG)


def _by_id(items):
    return {m['id']: m for m in items}


def test_total_size_includes_white_border():
    assert marker_total_m('tag36h11', 0.160) == pytest.approx(0.200)
    assert marker_total_m('aruco_4x4_50', 0.060) == pytest.approx(0.080)
    with pytest.raises(ValueError):
        marker_total_m('qr', 0.1)


def test_floor_marker_sits_on_floor_top():
    m = _by_id(layout(_platform_root(), _cfg()))[0]
    assert m['link'] == 'floor'
    assert m['pose'][0] == pytest.approx((0.0, 0.0, 0.1136 + 0.0075 + 0.0005))
    assert m['pose'][1] == pytest.approx((0.0, 0.0, 0.0))
    assert m['total_m'] == pytest.approx(0.200)


def test_plate_marker_follows_plate_tilt():
    m = _by_id(layout(_platform_root(), _cfg()))[1]
    h = 0.003 + 0.0057 + 0.0005
    assert m['pose'][0] == pytest.approx((0.15995 - h * math.sin(0.180938), 0.0,
                                          0.12535 + h * math.cos(0.180938)), abs=1e-9)
    assert m['pose'][1] == pytest.approx((0.0, -0.180938, 0.0), abs=1e-9)


def test_bad_configs_are_rejected():
    root = _platform_root()
    for change in ({'id': 1}, {'size_m': 0.170}, {'anchor': 'nope'}, {'link': 'nope'}):
        cfg = _cfg()
        cfg['markers'][0].update(change)
        with pytest.raises(ValueError):
            layout(root, cfg)


def _stl_vertices(path):
    data = open(path, 'rb').read()
    n = struct.unpack('<I', data[80:84])[0]
    for i in range(n):
        v = struct.unpack('<12f', data[84 + 50 * i:84 + 50 * i + 48])
        for k in range(3):
            yield tuple(c * 0.001 for c in v[3 + 3 * k:6 + 3 * k])


def test_markers_sit_above_visible_mesh():
    mesh_dir = os.path.join(_platform_dir(), 'meshes')
    for m in layout(_platform_root(), _cfg()):
        path = os.path.join(mesh_dir, m['link'] + '.stl')
        if not os.path.isfile(path):
            pytest.skip('메시 없음: %s' % path)
        (t, rpy) = m['pose']
        r = rpy_to_matrix(*rpy)
        rt = tuple(tuple(r[j][i] for j in range(3)) for i in range(3))
        top = max(mat_vec(rt, tuple(p[i] - t[i] for i in range(3)))[2]
                  for p in _stl_vertices(path))
        assert top <= 1e-6, (m['id'], top)


def test_obj_uv_orientation_is_not_mirrored():
    # 위(+z)에서 볼 때 그림의 왼쪽 위(uv 0,1) 가 (-h, +h) 에 와야 거울상이 아니다
    lines = obj_text('marker_0', 0.2).splitlines()
    v = [tuple(float(x) for x in ln.split()[1:]) for ln in lines if ln.startswith('v ')]
    vt = [tuple(float(x) for x in ln.split()[1:]) for ln in lines if ln.startswith('vt ')]
    face = [ln for ln in lines if ln.startswith('f ')][0].split()[1:]
    pairs = [(v[int(f.split('/')[0]) - 1], vt[int(f.split('/')[1]) - 1]) for f in face]
    assert ((-0.1, 0.1, 0.0), (0.0, 1.0)) in pairs
    assert ((0.1, -0.1, 0.0), (1.0, 0.0)) in pairs
    a, b, c = pairs[0][0], pairs[1][0], pairs[2][0]
    normal_z = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    assert normal_z > 0


def test_flip_v_swaps_texture_rows():
    lines = obj_text('m', 0.1, flip_v=True).splitlines()
    assert [ln for ln in lines if ln.startswith('vt ')][0] == 'vt 0 1'


def test_add_marker_visuals_to_platform_links():
    root = _platform_root()
    items = layout(root, _cfg())
    add_marker_visuals(root, items)
    model = root.find('model')
    for m in items:
        path = "link[@name='%s']/visual[@name='marker_%d_visual']" % (m['link'], m['id'])
        vis = model.find(path)
        assert vis is not None
        assert vis.findtext('geometry/mesh/uri') == '%s/marker_%d.obj' % (MARKER_URI, m['id'])
        assert vis.findtext('cast_shadows') == 'false'


def test_write_marker_meshes(tmp_path):
    items = layout(_platform_root(), _cfg())
    write_marker_meshes(items, str(tmp_path))
    for m in items:
        obj = (tmp_path / ('marker_%d.obj' % m['id'])).read_text(encoding='utf-8')
        mtl = (tmp_path / ('marker_%d.mtl' % m['id'])).read_text(encoding='utf-8')
        assert 'mtllib marker_%d.mtl' % m['id'] in obj
        assert 'map_Kd marker_%d.png' % m['id'] in mtl


def test_image_to_gray_handles_row_padding():
    np = pytest.importorskip('numpy')
    data = bytes([255, 0, 0, 9, 0, 0, 255, 9])          # 1x1 rgb8, step 4 (패딩 1바이트) x 2줄
    gray = image_to_gray('rgb8', 2, 1, 4, data)
    assert gray.shape == (2, 1)
    assert gray[0, 0] == int(0.299 * 255) and gray[1, 0] == int(0.114 * 255)
    with pytest.raises(ValueError):
        image_to_gray('yuv422', 1, 1, 2, b'\x00\x00')
    assert np is not None


def test_prepare_platform_markers_end_to_end(tmp_path):
    pytest.importorskip('cv2')
    dst = tmp_path / 'landing_platform'
    shutil.copytree(_platform_dir(), str(dst))
    items = prepare_platform_markers(str(dst / 'model.sdf'), _cfg())
    assert len(items) == 5
    for m in items:
        for ext in ('png', 'obj', 'mtl'):
            assert (dst / 'markers' / ('marker_%d.%s' % (m['id'], ext))).is_file()
    model = ET.parse(str(dst / 'model.sdf')).getroot().find('model')
    assert len([v for v in model.iter('visual') if v.get('name', '').startswith('marker_')]) == 5


def test_marker_textures_are_three_channel(tmp_path):
    # 회색 1채널 PNG 는 Ogre2 에서 빨강(R8)으로 보일 수 있어 BGR 3채널로 쓴다
    cv2 = pytest.importorskip('cv2')
    from spacek_drone_sim.markers import write_marker_textures
    items = layout(_platform_root(), _cfg())
    write_marker_textures(_cfg(), items[:1], str(tmp_path))
    img = cv2.imread(str(tmp_path / 'marker_0.png'), cv2.IMREAD_UNCHANGED)
    assert img.ndim == 3 and img.shape[2] == 3
    assert (img[:, :, 0] == img[:, :, 2]).all()


def test_generated_marker_is_detected_and_mirror_is_not():
    cv2 = pytest.importorskip('cv2')
    np = pytest.importorskip('numpy')
    from spacek_drone_sim.markers import detect_ids, marker_image
    if not hasattr(cv2, 'aruco'):
        pytest.skip('cv2.aruco 없음')
    for marker_id in (0, 3):
        img = marker_image('tag36h11', marker_id, 20)
        assert img.shape == (200, 200)
        canvas = np.full((400, 400), 255, dtype=np.uint8)
        canvas[100:300, 100:300] = img
        assert detect_ids(canvas, 'tag36h11') == [marker_id]
        assert detect_ids(np.ascontiguousarray(np.fliplr(canvas)), 'tag36h11') != [marker_id]


def test_layout_does_not_modify_input():
    root = _platform_root()
    before = ET.tostring(root)
    layout(copy.deepcopy(root), _cfg())
    layout(root, _cfg())
    assert ET.tostring(root) == before
