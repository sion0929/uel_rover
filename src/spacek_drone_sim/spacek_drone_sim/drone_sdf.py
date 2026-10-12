"""드론 SDF (models/spacek_drone/model.sdf) 를 drone.yaml 값으로 맞춘다 (ROS 비의존).

패키지 폴더에서 템플릿 갱신:
  python3 -m spacek_drone_sim.drone_sdf config/drone.yaml models/spacek_drone/model.sdf
"""

import math
import sys
import xml.etree.ElementTree as ET

from spacek_drone_sim.drone_params import CAMERAS, derive, load_config
from spacek_drone_sim.pose_math import pose_text

GZ_NS = 'http://gazebosim.org/schema'
ET.register_namespace('gz', GZ_NS)     # 다시 쓸 때 gz:type 속성 이름을 유지
MOTOR_PLUGIN = 'gz::sim::systems::MulticopterMotorModel'
LIDAR_RPY = (0.0, 1.57, 0.0)           # x500_flow 와 같음 (PX4 가 아래 방향 거리센서로 인식)
INERTIA_TAGS = ('ixx', 'iyy', 'izz')
BLADE_CHORD_M, BLADE_THICK_M = 0.015, 0.002
BLADE_MATERIAL = (('ambient', '0.1 0.1 0.1 1'), ('diffuse', '0.2 0.2 0.2 1'))


def _fmt(v):
    return '%.9g' % float(v)


def _set(parent, path, text):
    el = parent.find(path)
    if el is None:
        raise ValueError('드론 SDF 에 %s/%s 가 없습니다' % (parent.get('name', parent.tag), path))
    el.text = text
    return el


def _named(parent, tag, name):
    el = parent.find("%s[@name='%s']" % (tag, name))
    if el is None:
        raise ValueError('드론 SDF 에 %s %s 가 없습니다' % (tag, name))
    return el


def parse_sdf(path):
    """주석을 보존해서 읽는다 (템플릿을 다시 쓸 때 설명이 남도록)."""
    return ET.parse(path, parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))


def _patch_inertial(link, mass, diag, com=None):
    if com is not None:
        _set(link, 'inertial/pose', pose_text(com))
    _set(link, 'inertial/mass', _fmt(mass))
    for tag, v in zip(INERTIA_TAGS, diag):
        _set(link, 'inertial/inertia/' + tag, _fmt(v))


def _patch_camera(base, name, cam, hfov):
    s = _named(base, 'sensor', name)
    _set(s, 'gz_frame_id', cam['frame_id'])
    _set(s, 'pose', pose_text(cam['xyz_m'], cam['rpy_rad']))
    _set(s, 'topic', cam['gz_topic'])
    _set(s, 'update_rate', _fmt(cam['rate_hz']))
    _set(s, 'camera/horizontal_fov', _fmt(hfov))
    _set(s, 'camera/image/width', str(int(cam['width'])))
    _set(s, 'camera/image/height', str(int(cam['height'])))


def _set_blades(link, n, diameter):
    """프롭 비주얼 = 날개 n 개 (상자, 길이 = 반지름, 360/n 도 간격). 이전 프롭 비주얼은 지운다."""
    name = link.get('name')
    for v in list(link.findall('visual')):
        if v.get('name', '').startswith((name + '_blade', name + '_prop_visual')):
            link.remove(v)
    r = float(diameter) / 2.0
    for k in range(n):
        yaw = 2.0 * math.pi * k / n
        v = ET.SubElement(link, 'visual', {'name': '%s_blade_%d' % (name, k)})
        center = (r / 2.0 * math.cos(yaw), r / 2.0 * math.sin(yaw), 0.0)
        ET.SubElement(v, 'pose').text = pose_text(center, (0.0, 0.0, yaw))
        box = ET.SubElement(ET.SubElement(v, 'geometry'), 'box')
        size = (r, BLADE_CHORD_M, BLADE_THICK_M)
        ET.SubElement(box, 'size').text = ' '.join(_fmt(v) for v in size)
        mat = ET.SubElement(v, 'material')
        for tag, text in BLADE_MATERIAL:
            ET.SubElement(mat, tag).text = text


def patch_drone(root, p, cfg):
    """derive(cfg) 결과 p 와 cfg 로 드론 SDF 트리를 고친다 (제자리 수정, root 반환)."""
    model = root.find('model')
    if model is None:
        raise ValueError('드론 SDF 에 model 이 없습니다')
    base = _named(model, 'link', 'base_link')
    _patch_inertial(base, p['base_mass'], p['base_inertia'], p['base_com'])
    col = cfg['collision']
    c = _named(base, 'collision', 'lower_frame_col')
    _set(c, 'pose', pose_text(col['center_m']))
    _set(c, 'geometry/box/size', ' '.join(_fmt(v) for v in col['size_m']))
    _set(c, 'surface/friction/ode/mu', _fmt(col['mu']))
    _set(c, 'surface/friction/ode/mu2', _fmt(col['mu']))
    _set(_named(base, 'visual', 'drone_mesh_visual'), 'pose',
         pose_text((0.0, 0.0, 0.0), (0.0, 0.0, cfg['visual']['drone_mesh_yaw_rad'])))
    fc = cfg['sensors']['front_camera']
    case = _named(base, 'visual', 'front_case_visual')
    _set(case, 'pose', pose_text(fc['case_xyz_m'], fc['case_rpy_rad']))
    _set(case, 'geometry/mesh/uri', 'model://spacek_drone/meshes/' + fc['case_mesh'])
    _set(_named(base, 'visual', 'down_camera_visual'), 'pose',
         pose_text(cfg['sensors']['down_camera']['xyz_m']))
    for name in CAMERAS:
        _patch_camera(base, name, cfg['sensors'][name], p['camera_hfov'][name])

    mot = cfg['motor']
    plugins = [pl for pl in model.findall('plugin') if pl.get('name') == MOTOR_PLUGIN]
    if len(plugins) != len(p['rotors']):
        raise ValueError('모터 플러그인이 %d 개입니다 (4 개여야 함)' % len(plugins))
    for r, pl in zip(p['rotors'], plugins):
        link = _named(model, 'link', r['link'])
        _set(link, 'pose', pose_text(r['xyz']))
        _patch_inertial(link, p['rotor_mass'], p['rotor_inertia'])
        _set_blades(link, p['blades'], cfg['rotors']['prop_diameter_m'])
        _set(pl, 'jointName', r['joint'])
        _set(pl, 'linkName', r['link'])
        _set(pl, 'turningDirection', r['direction'])
        _set(pl, 'motorNumber', str(r['index']))
        _set(pl, 'maxRotVelocity', _fmt(p['max_rot_velocity']))
        _set(pl, 'motorConstant', _fmt(mot['motor_constant']))
        _set(pl, 'momentConstant', _fmt(mot['moment_constant']))
        _set(pl, 'timeConstantUp', _fmt(mot['time_constant_up']))
        _set(pl, 'timeConstantDown', _fmt(mot['time_constant_down']))
        _set(pl, 'rotorDragCoefficient', _fmt(mot['rotor_drag_coefficient']))
        _set(pl, 'rollingMomentCoefficient', _fmt(mot['rolling_moment_coefficient']))

    _set(_named(model, 'link', 'flow_link'), 'pose', pose_text(p['flow_xyz']))
    lidar_link = _named(model, 'link', 'lidar_sensor_link')
    _set(lidar_link, 'pose', pose_text(p['lidar_xyz'], LIDAR_RPY))
    lc = cfg['sensors']['lidar']
    lidar = _named(lidar_link, 'sensor', 'lidar')
    _set(lidar, 'update_rate', _fmt(lc['rate_hz']))
    _set(lidar, 'ray/range/min', _fmt(lc['min_m']))
    _set(lidar, 'ray/range/max', _fmt(lc['max_m']))
    return root


def write_drone_sdf(src_sdf, dst_sdf, cfg):
    tree = parse_sdf(src_sdf)
    patch_drone(tree.getroot(), derive(cfg), cfg)
    tree.write(dst_sdf, encoding='utf-8', xml_declaration=True)
    return dst_sdf


def main(argv=None):
    args = sys.argv[1:] if argv is None else argv
    if len(args) != 2:
        print('사용: python3 -m spacek_drone_sim.drone_sdf <drone.yaml> <model.sdf>')
        return 2
    write_drone_sdf(args[1], args[1], load_config(args[0]))
    print('갱신:', args[1])
    return 0


if __name__ == '__main__':
    sys.exit(main())
