"""런치 시점에 spacek_rover 모델 SDF 를 복사/수정하는 도구 (ROS 의존성 없음).

* rover_params.derive() 결과로 질량/관성/무게중심, 바퀴 위치/원통 치수/마찰,
  바퀴 조인트 effort/velocity 한계, DiffDrive 치수와 속도/가속 한계를 덮어쓴다.
* 메시(STL)가 모두 있으면 *_box_visual 을, 없으면 *_mesh_visual 을 제거한다.
"""

import os
import shutil
import xml.etree.ElementTree as ET

from spacek_rover.rover_params import WHEELS

MODEL_NAME = 'spacek_rover'
MESHES = ['base.stl', 'wheel.stl']
DIFF_DRIVE = 'gz::sim::systems::DiffDrive'


def _set(parent, tag, value):
    child = parent.find(tag)
    if child is None:
        child = ET.SubElement(parent, tag)
    child.text = '%.9g' % float(value)
    return child


def _pose_text(xyz, rpy=(0.0, 0.0, 0.0)):
    return ' '.join('%.9g' % float(v) for v in tuple(xyz) + tuple(rpy))


def find_link(root, name):
    """이름이 name 인 <link> 를 찾는다 (없으면 RuntimeError)."""
    for link in root.iter('link'):
        if link.get('name') == name:
            return link
    raise RuntimeError('링크 %s 를 SDF 에서 찾지 못했습니다' % name)


def _set_inertial(link, mass, diag, com=None):
    inertial = link.find('inertial')
    _set(inertial, 'mass', mass)
    inertia = inertial.find('inertia')
    for tag, value in zip(('ixx', 'iyy', 'izz'), diag):
        _set(inertia, tag, value)
    for tag in ('ixy', 'ixz', 'iyz'):
        _set(inertia, tag, 0.0)
    if com is not None:
        pose = inertial.find('pose')
        if pose is None:
            pose = ET.SubElement(inertial, 'pose')
        pose.text = _pose_text(com)


def _set_cylinder(element, radius, width):
    cyl = element.find('geometry/cylinder')
    if cyl is not None:
        _set(cyl, 'radius', radius)
        _set(cyl, 'length', width)


def strip_visuals(root, suffix):
    """이름이 suffix 로 끝나는 <visual> 을 모두 제거하고 제거 개수를 반환한다."""
    removed = 0
    for link in root.iter('link'):
        for vis in list(link.findall('visual')):
            if vis.get('name', '').endswith(suffix):
                link.remove(vis)
                removed += 1
    return removed


def patch_rover(root, p):
    """spacek_rover 모델 SDF(root)에 rover_params.derive() 결과 p 를 반영한다."""
    _set_inertial(find_link(root, 'base_link'), p['base_mass'], p['base_inertia'], p['base_com'])
    i_axis, i_perp = p['wheel_inertia']
    joints = {j.get('name'): j for j in root.iter('joint')}
    for wheel in WHEELS:
        link = find_link(root, wheel + '_wheel')
        _set_inertial(link, p['wheel_mass'], (i_perp, i_axis, i_perp))
        link.find('pose').text = _pose_text(p['wheel_positions'][wheel])
        for col in link.findall('collision'):
            _set_cylinder(col, p['wheel_radius'], p['wheel_width'])
            ode = col.find('surface/friction/ode')
            if ode is not None:
                # fdir1 = 충돌 좌표 z = 바퀴 축 -> mu 는 측면, mu2 는 굴림 방향
                _set(ode, 'mu', p['mu_lat'])
                _set(ode, 'mu2', p['mu_long'])
                fdir1 = ode.find('fdir1')
                if fdir1 is None:
                    fdir1 = ET.SubElement(ode, 'fdir1')
                fdir1.text = '0 0 1'
        for vis in link.findall('visual'):
            _set_cylinder(vis, p['wheel_radius'], p['wheel_width'])
        joint = joints.get(wheel + '_wheel_joint')
        if joint is None:
            raise RuntimeError('조인트 %s_wheel_joint 를 SDF 에서 찾지 못했습니다' % wheel)
        limit = joint.find('axis/limit')
        _set(limit, 'effort', p['wheel_effort'])
        _set(limit, 'velocity', p['wheel_velocity'])
    plugins = [pl for pl in root.iter('plugin') if pl.get('name') == DIFF_DRIVE]
    if len(plugins) != 1:
        raise RuntimeError('DiffDrive 플러그인이 1개여야 합니다 (현재 %d 개)' % len(plugins))
    dd = plugins[0]
    _set(dd, 'wheel_separation', p['wheel_separation'])
    _set(dd, 'wheel_radius', p['wheel_radius'])
    _set(dd, 'odom_publish_frequency', p['odom_rate_hz'])
    limits = {
        'linear_velocity': p['max_linear_velocity'],
        'angular_velocity': p['max_angular_velocity'],
        'linear_acceleration': p['max_linear_accel'],
        'angular_acceleration': p['max_angular_accel'],
    }
    for key, value in limits.items():
        _set(dd, 'max_' + key, value)
        _set(dd, 'min_' + key, -value)


def meshes_present(model_dir):
    """model_dir/meshes 에 base.stl, wheel.stl 이 모두 있으면 True 를 돌려준다."""
    return all(os.path.isfile(os.path.join(model_dir, 'meshes', n)) for n in MESHES)


def prepare_runtime_model(src_models_dir, dst_models_dir, params, visual_mode='auto'):
    """설치된 models/spacek_rover 를 dst 로 복사하고 파라미터/비주얼을 반영한다.

    visual_mode: 'auto' (메시가 모두 있으면 mesh, 아니면 box) | 'mesh' | 'box'
    반환값: {'visual': 'mesh'|'box', 'sdf': 수정된 model.sdf 경로}
    """
    if visual_mode not in ('auto', 'mesh', 'box'):
        raise ValueError('visual_mode 는 auto|mesh|box 중 하나여야 합니다: %r' % visual_mode)
    if os.path.isdir(dst_models_dir):
        shutil.rmtree(dst_models_dir)
    dst = os.path.join(dst_models_dir, MODEL_NAME)
    shutil.copytree(os.path.join(src_models_dir, MODEL_NAME), dst)
    use_mesh = visual_mode == 'mesh' or (visual_mode == 'auto' and meshes_present(dst))
    sdf_path = os.path.join(dst, 'model.sdf')
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    patch_rover(root, params)
    strip_visuals(root, '_box_visual' if use_mesh else '_mesh_visual')
    tree.write(sdf_path, encoding='utf-8', xml_declaration=True)
    return {'visual': 'mesh' if use_mesh else 'box', 'sdf': sdf_path}
