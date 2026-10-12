"""플랫폼 마커: 배치 계산, OBJ/PNG 파일, 플랫폼 SDF 비주얼, 영상 검출 (ROS 비의존).

배치·파일·SDF 는 표준 라이브러리만, 그림 생성·검출은 OpenCV(aruco) + numpy (함수 안에서 import).
마커 사각형 OBJ: z = 0 면, 법선 +z. 위에서 볼 때 x 오른쪽, y 위 = 그림의 오른쪽, 위 (거울상 아님).
"""

import os
import xml.etree.ElementTree as ET

from spacek_drone_sim.pose_math import compose, parse_pose, pose_text
import yaml

# 이름 -> (OpenCV 사전, 데이터 칸 수 n: 마커 = n x n 데이터 + 검은 테두리 1칸)
FAMILIES = {
    'tag36h11': ('DICT_APRILTAG_36h11', 6),
    'tag25h9': ('DICT_APRILTAG_25h9', 5),
    'tag16h5': ('DICT_APRILTAG_16h5', 4),
    'aruco_4x4_50': ('DICT_4X4_50', 4),
    'aruco_5x5_100': ('DICT_5X5_100', 5),
    'aruco_6x6_250': ('DICT_6X6_250', 6),
}
MARKER_DIR = 'markers'
MARKER_URI = 'model://landing_platform/' + MARKER_DIR
GRAY_CHANNELS = {'mono8': 1, 'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}


def load_markers_config(path):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def _family(family):
    if family not in FAMILIES:
        raise ValueError('알 수 없는 마커 종류: %s (가능: %s)' % (family, ', '.join(sorted(FAMILIES))))
    return FAMILIES[family]


def marker_total_m(family, size_m):
    """size_m (검은 테두리 한 변) -> 흰 여백 1칸 포함 한 변."""
    black = _family(family)[1] + 2
    return float(size_m) * (black + 2) / black


def _named(parent, tag, name):
    el = parent.find("%s[@name='%s']" % (tag, name))
    if el is None:
        raise ValueError('플랫폼 SDF 에 %s %s 가 없습니다' % (tag, name))
    return el


def layout(platform_root, cfg):
    """마커마다 {id, link, pose(링크 기준), size_m, total_m}. 입력 트리는 바꾸지 않는다."""
    model = platform_root.find('model')
    ids = [int(m['id']) for m in cfg['markers']]
    if len(set(ids)) != len(ids):
        raise ValueError('마커 번호가 겹칩니다: %s' % ids)
    lift = float(cfg.get('lift_m', 0.0005))
    out = []
    for m in cfg['markers']:
        link = _named(model, 'link', m['link'])
        col = _named(link, 'collision', m['anchor'])
        size = col.find('geometry/box/size')
        if size is None:
            raise ValueError('%s 는 상자 충돌이 아닙니다' % m['anchor'])
        sx, sy, sz = (float(v) for v in size.text.split())
        total = marker_total_m(cfg['family'], m['size_m'])
        if total > min(sx, sy) + 1e-9:
            raise ValueError('마커 %d (여백 포함 %.4f m) 가 %s 윗면 (%.4f x %.4f m) 보다 큽니다'
                             % (int(m['id']), total, m['anchor'], sx, sy))
        h = sz / 2.0 + float(m.get('surface_offset_m', 0.0)) + lift
        pose = compose(parse_pose(col.findtext('pose')), ((0.0, 0.0, h), (0.0, 0.0, 0.0)))
        out.append({'id': int(m['id']), 'link': m['link'], 'pose': pose,
                    'size_m': float(m['size_m']), 'total_m': total})
    return out


def obj_text(name, total_m, flip_v=False):
    h = total_m / 2.0
    vt = ('vt 0 1\nvt 1 1\nvt 1 0\nvt 0 0\n' if flip_v else 'vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\n')
    return ('# %s: 마커 사각형 (z = 0, 법선 +z), 위에서 보면 그림과 같은 방향\n'
            'mtllib %s.mtl\n'
            'o %s\n'
            'v %.6f %.6f 0\nv %.6f %.6f 0\nv %.6f %.6f 0\nv %.6f %.6f 0\n'
            '%s'
            'vn 0 0 1\n'
            'usemtl %s\n'
            'f 1/1/1 2/2/1 3/3/1 4/4/1\n'
            % (name, name, name, -h, -h, h, -h, h, h, -h, h, vt, name))


def mtl_text(name):
    return ('newmtl %s\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nd 1\nillum 1\nmap_Kd %s.png\n'
            % (name, name))


def _write(path, text):
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write(text)


def write_marker_meshes(items, out_dir, flip_v=False):
    os.makedirs(out_dir, exist_ok=True)
    for m in items:
        name = 'marker_%d' % m['id']
        _write(os.path.join(out_dir, name + '.obj'), obj_text(name, m['total_m'], flip_v))
        _write(os.path.join(out_dir, name + '.mtl'), mtl_text(name))


def marker_image(family, marker_id, px_per_cell=40):
    """흰 여백 1칸을 포함한 마커 그림 (uint8, 정사각형)."""
    import cv2
    import numpy as np
    dict_name, bits = _family(family)
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
    side = (bits + 2) * px_per_cell
    if hasattr(cv2.aruco, 'generateImageMarker'):
        img = cv2.aruco.generateImageMarker(dictionary, int(marker_id), side, borderBits=1)
    else:  # OpenCV 4.6 (Ubuntu 24.04 python3-opencv)
        img = cv2.aruco.drawMarker(dictionary, int(marker_id), side, borderBits=1)
    return np.pad(img, px_per_cell, mode='constant', constant_values=255)


def write_marker_textures(cfg, items, out_dir):
    import cv2
    os.makedirs(out_dir, exist_ok=True)
    for m in items:
        img = marker_image(cfg['family'], m['id'], int(cfg.get('pixels_per_cell', 40)))
        # 1채널 PNG 는 Ogre2 에서 빨강(R8)으로 보일 수 있어 3채널로 쓴다
        cv2.imwrite(os.path.join(out_dir, 'marker_%d.png' % m['id']),
                    cv2.cvtColor(img, cv2.COLOR_GRAY2BGR))


def add_marker_visuals(platform_root, items, uri_dir=MARKER_URI):
    model = platform_root.find('model')
    for m in items:
        link = _named(model, 'link', m['link'])
        vis = ET.SubElement(link, 'visual', name='marker_%d_visual' % m['id'])
        ET.SubElement(vis, 'pose').text = pose_text(*m['pose'])
        ET.SubElement(vis, 'cast_shadows').text = 'false'
        mesh = ET.SubElement(ET.SubElement(vis, 'geometry'), 'mesh')
        ET.SubElement(mesh, 'uri').text = '%s/marker_%d.obj' % (uri_dir, m['id'])
    return platform_root


def prepare_platform_markers(platform_sdf, cfg):
    """런타임 플랫폼 SDF 사본에 마커를 붙인다 (그림·메시는 같은 폴더의 markers/). items 반환."""
    tree = ET.parse(platform_sdf)
    items = layout(tree.getroot(), cfg)
    out_dir = os.path.join(os.path.dirname(platform_sdf), MARKER_DIR)
    write_marker_meshes(items, out_dir, bool(cfg.get('flip_v', False)))
    write_marker_textures(cfg, items, out_dir)
    add_marker_visuals(tree.getroot(), items)
    tree.write(platform_sdf, encoding='utf-8', xml_declaration=True)
    return items


def image_to_gray(encoding, height, width, step, data):
    """sensor_msgs/Image 필드 -> 회색 numpy 배열 (높이 x 너비)."""
    import numpy as np
    if encoding not in GRAY_CHANNELS:
        raise ValueError('지원하지 않는 영상 형식: %s' % encoding)
    c = GRAY_CHANNELS[encoding]
    rows = np.frombuffer(bytes(data), dtype=np.uint8).reshape(height, step)[:, :width * c]
    img = rows.reshape(height, width, c).astype(np.float32)
    if c == 1:
        return img[:, :, 0].astype(np.uint8)
    if encoding.startswith('rgb'):
        r, g, b = img[:, :, 0], img[:, :, 1], img[:, :, 2]
    else:
        b, g, r = img[:, :, 0], img[:, :, 1], img[:, :, 2]
    return (0.299 * r + 0.587 * g + 0.114 * b).astype(np.uint8)


def detect_ids(gray, family):
    import cv2
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, _family(family)[0]))
    if hasattr(cv2.aruco, 'ArucoDetector'):
        _corners, ids, _rej = cv2.aruco.ArucoDetector(
            dictionary, cv2.aruco.DetectorParameters()).detectMarkers(gray)
    else:  # OpenCV 4.6
        _corners, ids, _rej = cv2.aruco.detectMarkers(
            gray, dictionary, parameters=cv2.aruco.DetectorParameters_create())
    return sorted(int(i) for i in ids.flatten()) if ids is not None else []
