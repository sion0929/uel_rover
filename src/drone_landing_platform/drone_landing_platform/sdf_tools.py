"""런치 시점에 모델 SDF 를 복사/수정하는 도구 (ROS 의존성 없음).

* 스프링 강성/예압/감쇠, 하단 스톱, 플레이트 PID 게인, 드론 질량을 YAML 값으로 덮어쓴다.
* 메시(STL)가 있으면 박스 대체 비주얼(*_box_visual)을, 없으면 메시 비주얼(*_mesh_visual)을 제거한다.
  -> 메시가 없어도 박스 형상으로 정상 동작하고, 메시가 있으면 겹쳐 그려지지 않는다.
"""

import os
import shutil
import xml.etree.ElementTree as ET

PLATFORM_MESHES = ['frame.stl', 'floor.stl', 'plate_px.stl', 'plate_mx.stl',
                   'plate_py.stl', 'plate_my.stl']
DRONE_MESHES = ['drone.stl']
GRAVITY = 9.80665


def spring_reference_m(stiffness_n_per_m, preload_n, guide_support_n=0.0):
    """대기 위치(d = 0)에서 위로 (preload + guide_support) 를 내는 spring_reference[m].

    floor_joint 축은 (0, 0, -1) 이므로 조인트 위치 q = d (아래로 +).
    SDF/DART 스프링 힘(축 방향) = -k (q - q_ref).
    d = 0 에서 위쪽(= 축의 - 방향) 힘 F 를 원하면 -k (0 - q_ref) = -F  ->  q_ref = -F / k.
    """
    k = float(stiffness_n_per_m)
    if k <= 0.0:
        raise ValueError('stiffness 는 양수여야 합니다')
    return -(float(preload_n) + float(guide_support_n)) / k


def _set_child_text(parent, tag, value):
    child = parent.find(tag)
    if child is None:
        child = ET.SubElement(parent, tag)
    child.text = '%.9g' % float(value)
    return child


def strip_visuals(root, suffix):
    """이름이 suffix 로 끝나는 <visual> 을 모두 제거하고 제거 개수를 반환한다."""
    removed = 0
    for link in root.iter('link'):
        for vis in list(link.findall('visual')):
            if vis.get('name', '').endswith(suffix):
                link.remove(vis)
                removed += 1
    return removed


def patch_platform(root, cfg):
    """landing_platform 모델 SDF(root)에 서스펜션/플레이트 제어 파라미터를 반영한다."""
    sus = cfg.get('suspension', {})
    pc = cfg.get('plate_controller', {})
    k = float(sus.get('stiffness_n_per_m', 250.0))
    preload = float(sus.get('preload_n', 8.1))
    guide = float(sus.get('plate_guide_support_n', 0.0))
    damping = float(sus.get('damping_ns_per_m', 40.0))
    stop = float(sus.get('hard_stop_m', 0.0606))
    found = False
    for joint in root.iter('joint'):
        if joint.get('name') == 'floor_joint':
            axis = joint.find('axis')
            dyn = axis.find('dynamics')
            _set_child_text(dyn, 'damping', damping)
            _set_child_text(dyn, 'spring_stiffness', k)
            _set_child_text(dyn, 'spring_reference', spring_reference_m(k, preload, guide))
            lim = axis.find('limit')
            _set_child_text(lim, 'lower', 0.0)
            _set_child_text(lim, 'upper', stop)
            found = True
    if not found:
        raise RuntimeError('floor_joint 를 SDF 에서 찾지 못했습니다')
    keys = {'p_gain': 'p_gain', 'i_gain': 'i_gain', 'd_gain': 'd_gain',
            'i_max': 'i_max', 'cmd_max': 'cmd_max'}
    for plugin in root.iter('plugin'):
        if plugin.get('name') != 'gz::sim::systems::JointPositionController':
            continue
        for yaml_key, sdf_tag in keys.items():
            if yaml_key in pc:
                _set_child_text(plugin, sdf_tag, pc[yaml_key])
        if 'i_max' in pc:
            _set_child_text(plugin, 'i_min', -float(pc['i_max']))
        if 'cmd_max' in pc:
            _set_child_text(plugin, 'cmd_min', -float(pc['cmd_max']))
    if cfg.get('frame'):
        _patch_frame_inertial(root, cfg['frame'])


def _patch_frame_inertial(root, fr):
    """고정 프레임 링크의 질량/무게중심/관성을 platform.yaml frame 값으로 바꾼다."""
    for link in root.iter('link'):
        if link.get('name') != 'frame':
            continue
        inertial = link.find('inertial')
        _set_child_text(inertial, 'mass', fr['mass_kg'])
        pose = inertial.find('pose')
        if pose is None:
            pose = ET.SubElement(inertial, 'pose')
        pose.text = '%.9g %.9g %.9g 0 0 0' % tuple(float(v) for v in fr['com_m'])
        inertia = inertial.find('inertia')
        for tag, value in zip(('ixx', 'iyy', 'izz'), fr['inertia_kgm2']):
            _set_child_text(inertia, tag, value)
        return
    raise RuntimeError('frame 링크를 플랫폼 SDF 에서 찾지 못했습니다')


def patch_drone(root, mass_kg):
    """simple_drone 의 body 링크 질량을 바꾸고 관성 텐서를 같은 비율로 스케일한다."""
    for link in root.iter('link'):
        if link.get('name') != 'body':
            continue
        inertial = link.find('inertial')
        mass_el = inertial.find('mass')
        old = float(mass_el.text)
        ratio = float(mass_kg) / old
        mass_el.text = '%.9g' % float(mass_kg)
        inertia = inertial.find('inertia')
        for tag in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'):
            el = inertia.find(tag)
            if el is not None:
                el.text = '%.9g' % (float(el.text) * ratio)
        return ratio
    raise RuntimeError('body 링크를 드론 SDF 에서 찾지 못했습니다')


def _meshes_present(model_dir, names):
    return all(os.path.isfile(os.path.join(model_dir, 'meshes', n)) for n in names)


def _process_model(sdf_path, use_mesh, patch_fn=None):
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    if patch_fn is not None:
        patch_fn(root)
    strip_visuals(root, '_box_visual' if use_mesh else '_mesh_visual')
    tree.write(sdf_path, encoding='utf-8', xml_declaration=True)


def prepare_runtime_models(src_models_dir, dst_models_dir, platform_cfg,
                           visual_mode='auto', drone_mass_kg=None):
    """설치된 models/ 를 dst 로 복사하고 파라미터/비주얼을 반영한다.

    visual_mode: 'auto' (메시가 모두 있으면 mesh, 아니면 box) | 'mesh' | 'box'
    반환값: {'platform_visual': 'mesh'|'box', 'drone_visual': ..., 'dst': dst_models_dir}
    """
    if os.path.isdir(dst_models_dir):
        shutil.rmtree(dst_models_dir)
    shutil.copytree(src_models_dir, dst_models_dir)
    plat_dir = os.path.join(dst_models_dir, 'landing_platform')
    drone_dir = os.path.join(dst_models_dir, 'simple_drone')

    def decide(model_dir, names):
        if visual_mode == 'mesh':
            return True
        if visual_mode == 'box':
            return False
        return _meshes_present(model_dir, names)

    plat_mesh = decide(plat_dir, PLATFORM_MESHES)
    drone_mesh = decide(drone_dir, DRONE_MESHES)
    _process_model(os.path.join(plat_dir, 'model.sdf'), plat_mesh,
                   lambda r: patch_platform(r, platform_cfg))
    mass = drone_mass_kg
    if mass is None:
        mass = platform_cfg.get('drone', {}).get('mass_kg', None)
    _process_model(os.path.join(drone_dir, 'model.sdf'), drone_mesh,
                   (lambda r: patch_drone(r, mass)) if mass else None)
    return {'platform_visual': 'mesh' if plat_mesh else 'box',
            'drone_visual': 'mesh' if drone_mesh else 'box',
            'dst': dst_models_dir}
