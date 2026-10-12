"""로버 위 착륙 플랫폼 장착: 두 모델 SDF 를 하나로 합친다 (ROS 의존성 없음, 단위시험 대상).

* 로버(spacek_rover) 모델은 그대로 두고, 플랫폼(landing_platform) 링크를 frame platform_origin 기준으로 붙인다.
* 플랫폼의 world 고정 조인트(frame_fixed)는 base_link 에 붙는 fixed 조인트 platform_fixed 로 바꾼다.
* JointStatePublisher 토픽을 명시해 두 모델의 기본 토픽 충돌을 없앤다.
좌표: 로버 base_link (x 전방, y 왼쪽, z 위), m / rad.
"""

import copy
import math
import xml.etree.ElementTree as ET

import yaml

MERGED_NAME = 'rover_platform'
ORIGIN_FRAME = 'platform_origin'
JSP = 'gz::sim::systems::JointStatePublisher'
PLATFORM_JSP_TOPIC = '/model/landing_platform/joint_state'
ROVER_JSP_TOPIC = '/model/spacek_rover/joint_state'


def load_mount_config(path):
    """rover_mount.yaml 을 읽어 dict 로 돌려준다."""
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def _model(root):
    model = root.find('model')
    if model is None:
        raise ValueError('<model> 이 없는 SDF 입니다')
    return model


def platform_mount_z(rover_root):
    """로버 SDF 의 platform_mount 프레임 z [m] (base_link 기준) 를 돌려준다."""
    for frame in _model(rover_root).findall('frame'):
        if frame.get('name') == 'platform_mount':
            return float(frame.find('pose').text.split()[2])
    raise ValueError('로버 SDF 에 platform_mount 프레임이 없습니다')


def platform_origin(rover_root, cfg):
    """플랫폼 원점 (x, y, z, yaw) 을 base_link 기준으로 돌려준다."""
    m = cfg['mount']
    z = platform_mount_z(rover_root) + float(m['base_plate_thickness_m'])
    return float(m['x_m']), float(m['y_m']), z, float(m['yaw_rad'])


def ground_z(origin, wheel_radius):
    """월드 원점 = 플랫폼 원점일 때의 지면 z [m] 를 돌려준다."""
    return -(float(wheel_radius) + origin[2])


def rover_spawn_pose(origin, clearance):
    """플랫폼 원점 = 월드 원점, 플랫폼 축 = 월드 축이 되는 로버 스폰 자세 (x, y, z, yaw)."""
    ox, oy, oz, oyaw = origin
    yaw = math.atan2(math.sin(-oyaw), math.cos(-oyaw))
    c, s = math.cos(yaw), math.sin(yaw)
    return -(c * ox - s * oy), -(s * ox + c * oy), -oz + float(clearance), yaw


def _names(model, tag):
    return {e.get('name') for e in model.findall(tag)}


def _set_topic(plugin, topic):
    el = plugin.find('topic')
    if el is None:
        el = ET.SubElement(plugin, 'topic')
    el.text = topic


def merge_platform_onto_rover(rover_root, platform_root, cfg, model_name=MERGED_NAME):
    """로버 SDF 와 플랫폼 SDF 를 모델 하나로 합친 새 <sdf> 루트를 돌려준다."""
    root = copy.deepcopy(rover_root)
    model = _model(root)
    platform = _model(platform_root)
    for tag in ('link', 'joint', 'frame'):
        clash = _names(model, tag) & _names(platform, tag)
        if clash:
            raise ValueError('이름 충돌 (%s): %s' % (tag, ', '.join(sorted(clash))))
    if ORIGIN_FRAME in _names(model, 'frame') | _names(platform, 'frame'):
        raise ValueError('%s 프레임이 이미 있습니다' % ORIGIN_FRAME)
    ox, oy, oz, oyaw = platform_origin(rover_root, cfg)
    model.set('name', model_name)
    for plugin in model.findall('plugin'):
        if plugin.get('name') == JSP and plugin.find('topic') is None:
            _set_topic(plugin, ROVER_JSP_TOPIC)
    frame = ET.SubElement(model, 'frame', {'name': ORIGIN_FRAME, 'attached_to': 'base_link'})
    pose = ET.SubElement(frame, 'pose', {'relative_to': 'base_link'})
    pose.text = '%.9g %.9g %.9g 0 0 %.9g' % (ox, oy, oz, oyaw)
    for link in platform.findall('link'):
        new = copy.deepcopy(link)
        lpose = new.find('pose')
        if lpose is None:
            lpose = ET.SubElement(new, 'pose')
            lpose.text = '0 0 0 0 0 0'
        lpose.set('relative_to', ORIGIN_FRAME)
        model.append(new)
    for joint in platform.findall('joint'):
        new = copy.deepcopy(joint)
        if new.find('parent').text.strip() == 'world':
            new.set('name', 'platform_fixed')
            new.set('type', 'fixed')
            new.find('parent').text = 'base_link'
        model.append(new)
    for plugin in platform.findall('plugin'):
        new = copy.deepcopy(plugin)
        if new.get('name') == JSP:
            _set_topic(new, PLATFORM_JSP_TOPIC)
        model.append(new)
    return root


def write_merged_model(rover_sdf, platform_sdf, cfg, dst_sdf, model_name=MERGED_NAME):
    """두 SDF 파일을 합쳐 dst_sdf 에 쓰고 {'sdf': dst_sdf, 'origin': 원점} 을 돌려준다."""
    rover_root = ET.parse(rover_sdf).getroot()
    platform_root = ET.parse(platform_sdf).getroot()
    merged = merge_platform_onto_rover(rover_root, platform_root, cfg, model_name)
    ET.ElementTree(merged).write(dst_sdf, encoding='utf-8', xml_declaration=True)
    return {'sdf': dst_sdf, 'origin': platform_origin(rover_root, cfg)}
