"""3a단계: PX4 SITL 드론(spacek_drone) + 마커 붙은 로버 플랫폼 (ROS 2 Jazzy + Gazebo Harmonic).

예)
  ros2 launch spacek_drone_sim px4_rover_landing.launch.py
  ros2 launch spacek_drone_sim px4_rover_landing.launch.py drone_start:=platform gui:=false
  ros2 launch spacek_drone_sim px4_rover_landing.launch.py px4:=false    # PX4 없이 모델만
  ros2 launch spacek_drone_sim px4_rover_landing.launch.py environment:=mars   # 화성 지형·바위
  ros2 launch spacek_drone_sim px4_rover_landing.launch.py px4_dir:=/opt/PX4-Autopilot \
      lat:=36.6 lon:=127.4 alt:=50

순서: 런타임 모델 준비 (플랫폼 + 마커, 로버와 합친 rover_platform, 드론 SDF, PX4 기체 파일 4901)
     -> gz sim -> 브리지, plate_coupler, 카메라 정적 TF -> 로버+플랫폼 스폰 -> 드론 스폰
     -> PX4 (standalone, 이미 있는 spacek_drone 에 붙음) -> Micro XRCE-DDS Agent.
px4:=false 이면 PX4 확인·기체 파일 설치·PX4·Agent 를 건너뛴다 (드론은 지면에 놓여 있고 카메라 토픽은 나옴).
비행 명령: 다른 터미널에서 <px4_dir>/build/px4_sitl_default/bin/px4-commander takeoff / land
          (또는 px4_shell:=true 로 PX4 콘솔 창을 띄움, QGroundControl 도 자동 연결).
"""

from datetime import datetime
import os
import re
import shutil
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from drone_landing_platform import rover_mount
from drone_landing_platform.sdf_tools import prepare_runtime_models
from drone_landing_platform.zenoh_router import router_actions
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable,
                            TimerAction)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from spacek_drone_sim import launch_tools, markers, px4_airframe
from spacek_drone_sim.drone_params import derive, load_config, MODEL_NAME
from spacek_drone_sim.drone_sdf import write_drone_sdf
from spacek_rover.rover_params import derive as derive_rover, load_config as load_rover_config
from spacek_rover.sdf_tools import prepare_runtime_model as prepare_rover_model
import yaml

PKG = 'spacek_drone_sim'
# environment 인자 -> worlds/ 의 월드 파일. 둘 다 월드 이름은 launch_tools.WORLD_NAME (PX4·브리지 토픽 공통)
WORLD_FILES = {'earth': 'rover_landing_px4.sdf', 'mars': 'rover_landing_mars.sdf'}


def _truthy(s):
    return str(s).strip().lower() in ('1', 'true', 'yes', 'on')


def _load_yaml(path):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    share = get_package_share_directory(PKG)
    plat_share = get_package_share_directory('drone_landing_platform')
    rover_share = get_package_share_directory('spacek_rover')
    gui = _truthy(arg('gui'))
    use_px4 = _truthy(arg('px4'))
    # PX4 빌드가 없으면 안내와 함께 중단 (px4:=false 이면 PX4 를 찾지 않음)
    px4 = px4_airframe.px4_paths(arg('px4_dir')) if use_px4 else None
    home = launch_tools.home_env(arg('lat'), arg('lon'), arg('alt'))

    runtime_root = tempfile.mkdtemp(prefix='spacek_drone_sim_')
    override_file = arg('px4_overrides_file')
    log_root_arg = arg('px4_log_dir')
    run_name = arg('px4_run_name')
    if run_name and (not log_root_arg or not re.fullmatch(r'px4_[A-Za-z0-9_-]+', run_name)):
        raise ValueError('px4_run_name requires px4_log_dir and a safe px4_* name')
    if use_px4:
        if log_root_arg or override_file:
            px4_log_root = os.path.abspath(os.path.expanduser(
                log_root_arg or '~/.ros/spacek_px4'))
            os.makedirs(px4_log_root, exist_ok=True)
            if run_name:
                px4_workdir = os.path.join(px4_log_root, run_name)
                os.makedirs(px4_workdir, exist_ok=True)
                if os.listdir(px4_workdir):
                    raise ValueError('PX4 run folder is not empty: %s' % px4_workdir)
            else:
                px4_workdir = tempfile.mkdtemp(
                    prefix='px4_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '_',
                    dir=px4_log_root)
        else:
            px4_workdir = None  # Original 3a build/rootfs behavior.
    else:
        px4_workdir = None
    runtime_models = os.path.join(runtime_root, 'models')

    # 플랫폼 + 마커 (런타임 사본에만)
    plat = _load_yaml(os.path.join(plat_share, 'config', 'platform.yaml'))
    prepare_runtime_models(os.path.join(plat_share, 'models'), runtime_models, plat,
                           arg('visual'), float(plat['drone']['mass_kg']))
    platform_sdf = os.path.join(runtime_models, 'landing_platform', 'model.sdf')
    marker_cfg_path = arg('markers_config') or os.path.join(share, 'config', 'markers.yaml')
    marker_items = markers.prepare_platform_markers(
        platform_sdf, markers.load_markers_config(marker_cfg_path))
    floor_top = launch_tools.floor_top_local(ET.parse(platform_sdf).getroot())

    # 로버 + 플랫폼 합치기 (2단계)
    rparams = derive_rover(load_rover_config(os.path.join(rover_share, 'config', 'rover.yaml')))
    rover_models = os.path.join(runtime_root, 'rover_models')
    rinfo = prepare_rover_model(os.path.join(rover_share, 'models'), rover_models, rparams,
                                arg('visual'))
    mount_cfg = rover_mount.load_mount_config(
        os.path.join(plat_share, 'config', 'rover_mount.yaml'))
    merged = rover_mount.write_merged_model(
        rinfo['sdf'], platform_sdf, mount_cfg,
        os.path.join(runtime_root, rover_mount.MERGED_NAME + '.sdf'))
    rover_z = float(rparams['wheel_radius']) + float(mount_cfg['spawn']['clearance_m'])
    if _truthy(arg('truth_links')):
        tree = ET.parse(merged['sdf'])
        plugin = ET.SubElement(tree.getroot().find('model'), 'plugin', {
            'filename': 'gz-sim-pose-publisher-system',
            'name': 'gz::sim::systems::PosePublisher'})
        for key, value in [('publish_model_pose', 'true'), ('publish_link_pose', 'true'),
                           ('use_pose_vector_msg', 'true'), ('update_frequency', '50')]:
            ET.SubElement(plugin, key).text = value
        tree.write(merged['sdf'], encoding='unicode')

    # 드론 SDF + PX4 기체 파일 (drone.yaml 이 원본)
    dcfg = load_config(os.path.join(share, 'config', 'drone.yaml'))
    dp = derive(dcfg)
    drone_sdf = write_drone_sdf(os.path.join(share, 'models', MODEL_NAME, 'model.sdf'),
                                os.path.join(runtime_root, MODEL_NAME + '.sdf'), dcfg)
    overrides = (_load_yaml(override_file) if override_file else {})
    if not isinstance(overrides, dict):
        raise ValueError('px4_overrides_file 은 파라미터 이름→숫자 YAML 사전이어야 합니다')
    if use_px4:
        text = px4_airframe.airframe_text(dp, overrides)
        airframe = (px4_airframe.stage_airframe(arg('px4_dir'), px4_workdir, text)
                    if px4_workdir else px4_airframe.install_airframe(arg('px4_dir'), text))
    else:
        airframe = None
    dx, dy, dz = launch_tools.drone_spawn_pose(arg('drone_start'), rover_z, merged['origin'],
                                               floor_top, dp['gear_bottom_z'])

    resource_path = launch_tools.join_paths(
        [runtime_models, rover_models, os.path.join(share, 'models'),
         os.path.join(plat_share, 'models')], os.environ.get('GZ_SIM_RESOURCE_PATH', ''))
    plugin_path = launch_tools.join_paths(
        [px4['gz_plugins']] if use_px4 else [], os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''))
    world = launch_tools.WORLD_NAME

    gz_cmd = ['gz', 'sim', '-r', '-v', arg('gz_verbosity')]
    if not gui:
        gz_cmd += ['-s', '--headless-rendering']
    gz_cmd.append(arg('world_file')
                  or os.path.join(share, 'worlds', WORLD_FILES[arg('environment')]))
    gz = ExecuteProcess(cmd=gz_cmd, output='screen',
                        additional_env={'GZ_SIM_RESOURCE_PATH': resource_path,
                                        'GZ_SIM_SYSTEM_PLUGIN_PATH': plugin_path})
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
                  output='screen',
                  parameters=[{'config_file': os.path.join(share, 'config', 'bridge.yaml')}])
    truth_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                        name='air_ground_truth_bridge', output='screen',
                        arguments=['/model/rover_platform/pose@tf2_msgs/msg/TFMessage'
                                   '[gz.msgs.Pose_V'],
                        remappings=[('/model/rover_platform/pose',
                                     '/air_ground/rover_truth_pose')])
    coupler = Node(package='drone_landing_platform', executable='plate_coupler',
                   name='plate_coupler', output='screen',
                   parameters=[os.path.join(plat_share, 'config', 'coupling.yaml'),
                               {'use_sim_time': True}])
    tfs = [Node(package='tf2_ros', executable='static_transform_publisher', name=name + '_tf',
                output='log',
                arguments=launch_tools.static_tf_args(xyz, q, 'base_link',
                                                      dcfg['sensors'][name]['frame_id']))
           for name, (xyz, q) in sorted(dp['cameras'].items())]
    spawn_rover = Node(package='ros_gz_sim', executable='create', output='screen',
                       arguments=['-world', world, '-name', rover_mount.MERGED_NAME,
                                  '-file', merged['sdf'], '-x', '0', '-y', '0',
                                  '-z', '%.6f' % rover_z])
    spawn_drone = Node(package='ros_gz_sim', executable='create', output='screen',
                       arguments=['-world', world, '-name', MODEL_NAME, '-file', drone_sdf,
                                  '-x', '%.6f' % dx, '-y', '%.6f' % dy, '-z', '%.6f' % dz])
    probe = Node(package=PKG, executable='marker_probe', name='marker_probe', output='screen',
                 parameters=[{'markers_config': marker_cfg_path, 'use_sim_time': True}])

    def _cleanup(_context):
        shutil.rmtree(runtime_root, ignore_errors=True)
        return []

    actions = [
        *router_actions(),
        LogInfo(msg='[px4_rover_landing] PX4=%s airframe=%s drone_start=%s (%.3f, %.3f, %.3f) '
                    '마커 %d개 (%s) gui=%s px4_logs=%s'
                    % (px4['root'] if use_px4 else '사용 안 함 (px4:=false)', airframe,
                       arg('drone_start'), dx, dy, dz, len(marker_items), marker_cfg_path,
                       gui, px4_workdir)),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_path),
        gz, bridge, *([truth_bridge] if _truthy(arg('truth_links')) else []), coupler, *tfs,
        # 순서 보장: 로버 스폰(create 종료) -> 드론 스폰 -> px4_delay 뒤 PX4
        TimerAction(period=float(arg('spawn_delay')), actions=[spawn_rover]),
        RegisterEventHandler(OnProcessExit(target_action=spawn_rover, on_exit=[spawn_drone])),
        RegisterEventHandler(OnProcessExit(
            target_action=gz, on_exit=[EmitEvent(event=Shutdown(reason='gz sim 종료'))])),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=_cleanup)])),
    ]
    if use_px4:
        px4_cmd = [px4['binary']]
        if _truthy(arg('px4_shell')):
            px4_cmd = ['xterm', '-hold', '-e'] + px4_cmd     # PX4 콘솔 창 (xterm 필요)
        else:
            px4_cmd.append('-d')                             # 콘솔 없이 실행
        if px4_workdir:
            px4_cmd += ['-w', px4_workdir, px4_workdir]
        if arg('px4_record_file'):
            px4_cmd = ['python3', '-m', 'air_ground_loc.px4_guard',
                       arg('px4_record_file'), arg('px4_active_file'), '--'] + px4_cmd
        px4_proc = ExecuteProcess(cmd=px4_cmd, cwd=px4['root'], output='screen',
                                  additional_env=launch_tools.px4_env(home))
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=spawn_drone,
            on_exit=[TimerAction(period=float(arg('px4_delay')), actions=[px4_proc])])))
        if _truthy(arg('xrce_agent')):
            # Fast DDS 2.x (the Agent links libfastrtps 2.14) reads FASTRTPS_*, 3.x reads FASTDDS_*
            # The PX4 client creates its participant from XML, so this default profile is not
            # applied to it (supervisor stage-2 run 6: sockets on LAN/tailscale) and SHM-only
            # transports cut the localhost-limited ROS nodes off (run 9). Isolation comes from
            # running the experiment in a loopback-only network namespace (run_isolated.sh).
            agent_env = ({'FASTRTPS_DEFAULT_PROFILES_FILE': arg('agent_profile_file'),
                          'FASTDDS_DEFAULT_PROFILES_FILE': arg('agent_profile_file')}
                         if arg('agent_profile_file') else None)
            actions.append(ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
                                          output='screen', additional_env=agent_env))
    if _truthy(arg('marker_probe')):
        actions.append(probe)
    return actions


def generate_launch_description():
    args = [
        DeclareLaunchArgument('gui', default_value='true', description='false 이면 헤드리스'),
        DeclareLaunchArgument('px4', default_value='true', choices=['true', 'false'],
                              description='false 이면 PX4 없이 모델만 (드론은 날지 않음)'),
        DeclareLaunchArgument('px4_dir', default_value='~/PX4-Autopilot',
                              description='PX4-Autopilot 폴더 (make px4_sitl 완료)'),
        DeclareLaunchArgument('px4_shell', default_value='false',
                              description='true 이면 PX4 콘솔을 xterm 창으로'),
        DeclareLaunchArgument('px4_delay', default_value='3.0',
                              description='드론 스폰 후 PX4 시작까지 [s]'),
        DeclareLaunchArgument('drone_start', default_value='ground',
                              choices=['ground', 'platform'],
                              description='ground: 로버 뒤 1.5 m 지면 / platform: 바닥판 위'),
        DeclareLaunchArgument('lat', default_value='', description='GPS 기준 위도 (lon, alt 와 함께)'),
        DeclareLaunchArgument('lon', default_value='', description='GPS 기준 경도'),
        DeclareLaunchArgument('alt', default_value='', description='GPS 기준 고도 [m]'),
        DeclareLaunchArgument('visual', default_value='auto', choices=['auto', 'mesh', 'box'],
                              description='플랫폼·로버 비주얼'),
        DeclareLaunchArgument('gz_verbosity', default_value='3'),
        DeclareLaunchArgument('spawn_delay', default_value='3.0',
                              description='gz 기동 후 로버 스폰까지 [s]'),
        DeclareLaunchArgument('xrce_agent', default_value='true',
                              description='Micro XRCE-DDS Agent (PX4 <-> ROS 2) 실행'),
        DeclareLaunchArgument('marker_probe', default_value='false',
                              description='true 이면 카메라 마커 검출 결과를 1 Hz 로 출력'),
        DeclareLaunchArgument('markers_config', default_value='',
                              description='마커 설정 파일 (빈칸 = config/markers.yaml)'),
        DeclareLaunchArgument('px4_overrides_file', default_value='',
                              description='실험용 PX4 파라미터 YAML 사전 (빈칸 = 기존 3a 동작)'),
        DeclareLaunchArgument('px4_log_dir', default_value='',
                              description='실험 PX4 실행 폴더의 상위 경로 (매번 px4_* 생성)'),
        DeclareLaunchArgument('px4_run_name', default_value='',
                              description='wrapper가 예약한 px4_* 하위 폴더 이름'),
        DeclareLaunchArgument('environment', default_value='earth', choices=sorted(WORLD_FILES),
                              description='earth: 초록 평지 / mars: 화성 지형 + 바위 (충돌 바닥은 평면)'),
        DeclareLaunchArgument('world_file', default_value='',
                              description='월드 파일 직접 지정 (environment 보다 우선, 월드 이름은 '
                                          'rover_landing_px4 이어야 함)'),
        DeclareLaunchArgument('truth_links', default_value='false'),
        DeclareLaunchArgument('px4_record_file', default_value=''),
        DeclareLaunchArgument('px4_active_file', default_value=''),
        DeclareLaunchArgument('agent_profile_file', default_value=''),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
