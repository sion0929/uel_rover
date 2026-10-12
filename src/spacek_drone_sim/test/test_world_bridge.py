"""월드·브리지 시험: PX4 시스템 목록, 지면, 카메라·로버·플랫폼 토픽 일치."""

import os
import xml.etree.ElementTree as ET

from spacek_drone_sim.drone_params import CAMERAS, load_config
import yaml

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD = os.path.join(PKG_DIR, 'worlds', 'rover_landing_px4.sdf')
BRIDGE = os.path.join(PKG_DIR, 'config', 'bridge.yaml')
CFG = os.path.join(PKG_DIR, 'config', 'drone.yaml')
ROVER_BRIDGE = os.path.join(os.path.dirname(PKG_DIR), 'drone_landing_platform', 'config',
                            'bridge_rover.yaml')
# PX4 v1.17.0 src/modules/simulation/gz_bridge/server.config (AirSpeed, GstCamera 제외)
PX4_SYSTEMS = {'gz::sim::systems::Physics', 'gz::sim::systems::UserCommands',
               'gz::sim::systems::SceneBroadcaster', 'gz::sim::systems::Contact',
               'gz::sim::systems::Imu', 'gz::sim::systems::AirPressure',
               'gz::sim::systems::ApplyLinkWrench', 'gz::sim::systems::NavSat',
               'gz::sim::systems::Magnetometer', 'gz::sim::systems::Sensors',
               'custom::OpticalFlowSystem'}


def _world():
    return ET.parse(WORLD).getroot().find('world')


def _bridge():
    with open(BRIDGE, encoding='utf-8') as f:
        return {e['ros_topic_name']: e for e in yaml.safe_load(f)}


def test_world_name_systems_and_ground():
    w = _world()
    assert w.get('name') == 'rover_landing_px4'
    assert {p.get('name') for p in w.findall('plugin')} == PX4_SYSTEMS
    assert w.find("plugin[@name='gz::sim::systems::Sensors']").findtext('render_engine') == 'ogre2'
    ground = w.find("model[@name='ground_plane']")
    assert (ground.findtext('pose') or '0 0 0').split()[2] in ('0', '0.0')
    assert w.find('spherical_coordinates/latitude_deg') is not None
    assert float(w.findtext('physics/max_step_size')) == 0.001


def test_camera_topics_match_drone_model():
    cfg = load_config(CFG)
    b = _bridge()
    for name in CAMERAS:
        gz_image = cfg['sensors'][name]['gz_topic']
        ns = gz_image.rsplit('/', 1)[0]
        assert b[ns + '/image_raw']['gz_topic_name'] == gz_image
        assert b[ns + '/image_raw']['ros_type_name'] == 'sensor_msgs/msg/Image'
        assert b[ns + '/camera_info']['gz_topic_name'] == ns + '/camera_info'
        assert b[ns + '/camera_info']['ros_type_name'] == 'sensor_msgs/msg/CameraInfo'
        assert b[ns + '/image_raw']['direction'] == 'GZ_TO_ROS'


def test_rover_and_platform_topics_same_as_stage2():
    b = _bridge()
    with open(ROVER_BRIDGE, encoding='utf-8') as f:
        stage2 = {e['ros_topic_name']: e for e in yaml.safe_load(f)}
    for ros, entry in stage2.items():
        if ros.startswith('/drone') or ros.startswith('/model/simple_drone') or ros == '/tf':
            continue
        assert b[ros] == entry, ros
    assert not any(t.startswith('/model/simple_drone') for t in b)


def test_rover_tf_does_not_share_base_link_with_drone():
    # 드론 카메라 정적 TF 가 base_link 를 부모로 쓰므로 로버 TF(odom -> base_link) 는 /tf 에 내지 않는다
    b = _bridge()
    assert '/tf' not in b
    assert b['/rover/tf']['gz_topic_name'] == '/model/spacek_rover/tf'
    assert b['/rover/tf']['direction'] == 'GZ_TO_ROS'
