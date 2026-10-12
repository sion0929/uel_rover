"""rover_mount 시험: 로버 + 플랫폼 SDF 합치기 (ROS/Gazebo 없이 실행 가능)."""

import ast
import math
import os
import xml.etree.ElementTree as ET

from drone_landing_platform.rover_mount import (ground_z, load_mount_config,
                                                merge_platform_onto_rover, platform_origin,
                                                rover_spawn_pose, write_merged_model)
from drone_landing_platform.sdf_tools import patch_platform
import pytest
import yaml

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PLATFORM_SDF = os.path.join(PKG_DIR, 'models', 'landing_platform', 'model.sdf')
MOUNT_CFG = os.path.join(PKG_DIR, 'config', 'rover_mount.yaml')
ROVER_REL = ('models', 'spacek_rover', 'model.sdf')
PLATFORM_LINKS = ('frame', 'floor', 'plate_px', 'plate_mx', 'plate_py', 'plate_my')


def _rover_sdf():
    path = os.path.join(os.path.dirname(PKG_DIR), 'spacek_rover', *ROVER_REL)
    if os.path.isfile(path):
        return path
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('spacek_rover'), *ROVER_REL)
    except (ImportError, LookupError):
        pytest.skip('spacek_rover 모델을 찾지 못함')


def _cfg():
    return load_mount_config(MOUNT_CFG)


def _wheel_radius():
    """로버 바퀴 반지름의 원본 = spacek_rover/config/rover.yaml."""
    path = os.path.join(os.path.dirname(_rover_sdf()), '..', '..', 'config', 'rover.yaml')
    with open(path, encoding='utf-8') as f:
        return float(yaml.safe_load(f)['wheel']['radius_m'])


def _rover_root():
    return ET.parse(_rover_sdf()).getroot()


def _platform_root():
    root = ET.parse(PLATFORM_SDF).getroot()
    with open(os.path.join(PKG_DIR, 'config', 'platform.yaml'), encoding='utf-8') as f:
        patch_platform(root, yaml.safe_load(f))
    return root


def _merged_model():
    return merge_platform_onto_rover(_rover_root(), _platform_root(), _cfg()).find('model')


def _link(model, name):
    return [lk for lk in model.findall('link') if lk.get('name') == name][0]


def test_platform_origin_above_wheel_center():
    origin = platform_origin(_rover_root(), _cfg())
    assert origin == pytest.approx((0.0, 0.0, 0.21954, math.pi), abs=1e-6)


def test_merged_links_and_joints():
    model = _merged_model()
    assert model.get('name') == 'rover_platform'
    links = {lk.get('name') for lk in model.findall('link')}
    rover_links = {'base_link', 'front_left_wheel', 'front_right_wheel', 'rear_left_wheel',
                   'rear_right_wheel'}
    assert links == rover_links | set(PLATFORM_LINKS)
    joints = {j.get('name'): j for j in model.findall('joint')}
    assert len(joints) == 10
    assert 'frame_fixed' not in joints
    assert all(j.find('parent').text.strip() != 'world' for j in joints.values())
    fixed = joints['platform_fixed']
    assert fixed.get('type') == 'fixed'
    assert (fixed.find('parent').text, fixed.find('child').text) == ('base_link', 'frame')


def test_platform_links_are_placed_on_platform_origin():
    model = _merged_model()
    origin = [f for f in model.findall('frame') if f.get('name') == 'platform_origin'][0]
    assert origin.get('attached_to') == 'base_link'
    assert origin.find('pose').get('relative_to') == 'base_link'
    pose = [float(v) for v in origin.find('pose').text.split()]
    assert pose == pytest.approx([0.0, 0.0, 0.21954, 0.0, 0.0, math.pi], abs=1e-6)
    for name in PLATFORM_LINKS:
        assert _link(model, name).find('pose').get('relative_to') == 'platform_origin'
    assert _link(model, 'base_link').find('pose').get('relative_to') is None


def test_plugin_topics_are_explicit_and_unique():
    model = _merged_model()
    topics = []
    for plugin in model.findall('plugin'):
        found = [plugin.find(t) for t in ('topic', 'odom_topic', 'tf_topic',
                                          'odom_covariance_topic')]
        found = [e.text for e in found if e is not None]
        assert found, plugin.get('name')
        topics += found
    assert len(topics) == len(set(topics))
    jsp = {p.find('topic').text for p in model.findall('plugin')
           if p.get('name') == 'gz::sim::systems::JointStatePublisher'}
    assert jsp == {'/model/landing_platform/joint_state', '/model/spacek_rover/joint_state'}


def test_merged_total_mass():
    model = _merged_model()
    total = sum(float(lk.find('inertial/mass').text) for lk in model.findall('link'))
    assert total == pytest.approx(20.0 + 1.15 + 0.486 + 4 * 0.176)


def test_name_clash_raises():
    platform = _platform_root()
    platform.find('model').find('link').set('name', 'base_link')
    with pytest.raises(ValueError):
        merge_platform_onto_rover(_rover_root(), platform, _cfg())


@pytest.mark.parametrize('origin', [
    (0.0, 0.0, 0.21954, math.pi),
    (0.05, -0.02, 0.21954, math.pi),
    (0.03, 0.0, 0.21954, 0.0),
    (0.0, 0.04, 0.21954, 1.0),
])
def test_spawn_pose_puts_platform_at_world_origin(origin):
    x, y, z, yaw = rover_spawn_pose(origin, 0.005)
    ox, oy, oz, oyaw = origin
    c, s = math.cos(yaw), math.sin(yaw)
    world = (x + c * ox - s * oy, y + s * ox + c * oy, z + oz)
    assert world == pytest.approx((0.0, 0.0, 0.005), abs=1e-9)
    assert math.cos(yaw + oyaw) == pytest.approx(1.0)
    assert z - ground_z(origin, _wheel_radius()) == pytest.approx(_wheel_radius() + 0.005)


def test_write_merged_model(tmp_path):
    platform_path = str(tmp_path / 'platform.sdf')
    ET.ElementTree(_platform_root()).write(platform_path, encoding='utf-8',
                                           xml_declaration=True)
    dst = str(tmp_path / 'rover_platform.sdf')
    info = write_merged_model(_rover_sdf(), platform_path, _cfg(), dst)
    assert info['sdf'] == dst
    assert info['origin'] == pytest.approx((0.0, 0.0, 0.21954, math.pi), abs=1e-6)
    assert ET.parse(dst).getroot().find('model').get('name') == 'rover_platform'


WORLD = os.path.join(PKG_DIR, 'worlds', 'rover_landing.sdf')
BRIDGE = os.path.join(PKG_DIR, 'config', 'bridge_rover.yaml')
LAUNCH = os.path.join(PKG_DIR, 'launch', 'landing_sim.launch.py')


def test_world_ground_matches_mount():
    world = ET.parse(WORLD).getroot().find('world')
    assert world.get('name') == 'rover_landing'
    ground = [m for m in world.findall('model') if m.get('name') == 'ground_plane'][0]
    z = float(ground.find('pose').text.split()[2])
    origin = platform_origin(_rover_root(), _cfg())
    assert z == pytest.approx(ground_z(origin, _wheel_radius()), abs=1e-6)
    assert not world.findall('include')


def test_bridge_rover_topics_match_models():
    with open(BRIDGE, encoding='utf-8') as f:
        bridge = {e['ros_topic_name']: e for e in yaml.safe_load(f)}
    gz_topics = set()
    for plugin in _merged_model().findall('plugin'):
        for tag in ('topic', 'odom_topic', 'tf_topic'):
            if plugin.find(tag) is not None:
                gz_topics.add(plugin.find(tag).text)
    for ros in ('/joint_states', '/rover/joint_states', '/rover/cmd_vel', '/rover/odom',
                '/rover/odom_gt', '/tf'):
        assert bridge[ros]['gz_topic_name'] in gz_topics, ros
    for j in ('plate_px_joint', 'plate_mx_joint', 'plate_py_joint', 'plate_my_joint'):
        entry = bridge['/platform/%s/cmd_pos' % j]
        assert entry['gz_topic_name'] in gz_topics
        assert entry['direction'] == 'ROS_TO_GZ'
    assert bridge['/joint_states']['gz_topic_name'] == '/model/landing_platform/joint_state'
    drone = '/world/%s/model/simple_drone/joint_state' % ET.parse(WORLD).getroot().find(
        'world').get('name')
    assert bridge['/drone/joint_states']['gz_topic_name'] == drone


def test_rover_runtime_dir_is_separate():
    with open(LAUNCH, encoding='utf-8') as f:
        src = f.read()
    assert "os.path.join(runtime_root, 'rover_models')" in src


def _launch_tree():
    with open(LAUNCH, encoding='utf-8') as f:
        return ast.parse(f.read())


def test_rover_watch_start_topic_is_bridged():
    # rover_watch 는 드론 스폰(첫 조인트 상태) 시점의 로버 자세를 기준으로 삼는다
    starts = [n.values[i].value for n in ast.walk(_launch_tree()) if isinstance(n, ast.Dict)
              for i, k in enumerate(n.keys)
              if isinstance(k, ast.Constant) and k.value == 'start_topic']
    assert starts == ['/drone/joint_states']
    with open(BRIDGE, encoding='utf-8') as f:
        bridge = {e['ros_topic_name']: e for e in yaml.safe_load(f)}
    assert bridge[starts[0]]['direction'] == 'GZ_TO_ROS'


def test_fixed_platform_launch_does_not_need_spacek_rover():
    # rover:=false 는 spacek_rover 가 없어도 떠야 한다 -> 모듈 최상위에서 import 금지
    for node in _launch_tree().body:
        if isinstance(node, ast.ImportFrom):
            assert not (node.module or '').startswith('spacek_rover'), node.module
        elif isinstance(node, ast.Import):
            assert not any(a.name.startswith('spacek_rover') for a in node.names)
