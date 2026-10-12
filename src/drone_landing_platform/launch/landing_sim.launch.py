"""드론 착륙 포획 시뮬레이션 런치 (ROS 2 Jazzy + Gazebo Harmonic).

예)
  ros2 launch drone_landing_platform landing_sim.launch.py scenario:=x60
  ros2 launch drone_landing_platform landing_sim.launch.py scenario:=yaw30 gui:=false
  ros2 launch drone_landing_platform landing_sim.launch.py mode:=sw_openloop damping:=30.0
  ros2 launch drone_landing_platform landing_sim.launch.py rover:=true scenario:=x30

동작:
  1) config/platform.yaml (+ launch 인자 덮어쓰기) 로 모델 SDF 를 임시 폴더에 복사/수정
     (스프링/감쇠/스톱/플레이트 PID/프레임 질량/드론 질량, 메시 유무에 따른 비주얼 선택)
  2) GZ_SIM_RESOURCE_PATH 를 그 임시 폴더 + 설치된 models 로 설정 -> model:// 해석
  3) gz sim (gui:=false 이면 -s 헤드리스), ros_gz_bridge, 드론 스폰, plate_coupler, landing_controller
     rover:=true 이면 월드 rover_landing, 로버+플랫폼 합친 모델 스폰(rover_mount), rover_watch 추가.
     월드 원점 = 플랫폼 원점을 유지하므로 제어기/커플러 파라미터는 같다.
  4) landing_controller 가 종료되면(shutdown_on_done:=true) 전체 종료
"""

import os
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from drone_landing_platform import rover_mount
from drone_landing_platform.sdf_tools import prepare_runtime_models, spring_reference_m
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable,
                            TimerAction)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

PKG = 'drone_landing_platform'
WORLD_NAME = 'landing_test'
ROVER_WORLD_NAME = 'rover_landing'


def _truthy(s):
    return str(s).strip().lower() in ('1', 'true', 'yes', 'on')


def _load_yaml(path):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def _rover_setup(arg, runtime_root, runtime_models, result_name):
    """로버 + 플랫폼을 합친 모델을 만들고 스폰/감시 노드와 리소스 폴더를 돌려준다."""
    # rover:=false 는 spacek_rover 없이도 떠야 하므로 여기서만 import 한다.
    from spacek_rover.rover_params import derive as derive_rover
    from spacek_rover.rover_params import load_config as load_rover_config
    from spacek_rover.sdf_tools import prepare_runtime_model as prepare_rover_model

    share = get_package_share_directory(PKG)
    rover_share = get_package_share_directory('spacek_rover')
    rparams = derive_rover(load_rover_config(os.path.join(rover_share, 'config', 'rover.yaml')))
    # prepare_rover_model 은 대상 폴더를 지우므로 플랫폼 runtime 폴더와 반드시 분리한다.
    rover_models = os.path.join(runtime_root, 'rover_models')
    rinfo = prepare_rover_model(os.path.join(rover_share, 'models'), rover_models, rparams,
                                arg('visual'))
    cfg = rover_mount.load_mount_config(os.path.join(share, 'config', 'rover_mount.yaml'))
    merged = rover_mount.write_merged_model(
        rinfo['sdf'], os.path.join(runtime_models, 'landing_platform', 'model.sdf'), cfg,
        os.path.join(runtime_root, rover_mount.MERGED_NAME + '.sdf'))
    x, y, z, yaw = rover_mount.rover_spawn_pose(merged['origin'], cfg['spawn']['clearance_m'])
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', ROVER_WORLD_NAME, '-name', rover_mount.MERGED_NAME,
                            '-file', merged['sdf'], '-x', '%.6f' % x, '-y', '%.6f' % y,
                            '-z', '%.6f' % z, '-Y', '%.6f' % yaw])
    watch = Node(package='spacek_rover', executable='rover_watch', name='rover_watch',
                 output='screen',
                 parameters=[{'use_sim_time': True, 'csv_dir': arg('csv_dir'),
                              'name': result_name,
                              # 드론이 스폰되는 순간(첫 조인트 상태)을 기준 자세로 삼는다
                              'start_topic': '/drone/joint_states'}])
    log = LogInfo(msg='[landing_sim] rover:=true platform origin (base_link) = '
                      '(%.4f, %.4f, %.4f) yaw %.3f, rover spawn (%.4f, %.4f, %.4f) yaw %.3f, '
                      'rover base %.2f kg' % (merged['origin'] + (x, y, z, yaw)
                                              + (rparams['base_mass'],)))
    return {'models_dir': rover_models, 'spawn': spawn, 'watch': watch, 'log': log}


def launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    share = get_package_share_directory(PKG)
    cfg_dir = os.path.join(share, 'config')
    scenario = arg('scenario')
    mode = arg('mode')
    gui = _truthy(arg('gui'))
    rover = _truthy(arg('rover'))
    world_name = ROVER_WORLD_NAME if rover else WORLD_NAME

    scenarios = _load_yaml(os.path.join(cfg_dir, 'scenarios.yaml'))
    if scenario not in scenarios:
        raise RuntimeError('알 수 없는 scenario: %s (가능: %s)'
                           % (scenario, ', '.join(sorted(scenarios))))
    sc = scenarios[scenario]

    plat = _load_yaml(os.path.join(cfg_dir, 'platform.yaml'))
    overrides = {
        'damping': ('suspension', 'damping_ns_per_m'),
        'spring_k': ('suspension', 'stiffness_n_per_m'),
        'preload': ('suspension', 'preload_n'),
        'drone_mass': ('drone', 'mass_kg'),
    }
    for launch_arg, (section, key) in overrides.items():
        value = arg(launch_arg)
        if value.strip() != '':
            plat[section][key] = float(value)
    sus = plat['suspension']
    geo = plat['geometry']
    mass = float(plat['drone']['mass_kg'])
    if mode == 'sw_openloop':
        gap = float(plat['drone']['spawn_gap_openloop_m'])
    else:
        gap = float(plat['drone']['spawn_gap_closed_loop_m'])
    spawn_x = float(sc['x_mm']) / 1000.0
    spawn_y = float(sc['y_mm']) / 1000.0
    spawn_z = float(geo['frame_top_z']) + gap + float(geo['gear_bottom_to_origin_m'])
    result_name = scenario if mode == 'closed_loop' else scenario + '_' + mode

    # 모델 SDF 를 임시 폴더에 준비 (파라미터 반영 + 비주얼 선택)
    runtime_root = tempfile.mkdtemp(prefix='drone_landing_platform_')
    runtime_models = os.path.join(runtime_root, 'models')
    info = prepare_runtime_models(os.path.join(share, 'models'), runtime_models, plat,
                                  arg('visual'), mass)
    paths = [runtime_models, os.path.join(share, 'models')]
    rover_parts = None
    if rover:
        rover_parts = _rover_setup(arg, runtime_root, runtime_models, result_name)
        paths.insert(1, rover_parts['models_dir'])
    old = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if old:
        paths.append(old)
    resource_path = os.pathsep.join(paths)
    q_ref = spring_reference_m(sus['stiffness_n_per_m'], sus['preload_n'],
                               sus.get('plate_guide_support_n', 0.0))

    gz_cmd = ['gz', 'sim', '-r', '-v', arg('gz_verbosity')]
    if not gui:
        gz_cmd.insert(2, '-s')
    gz_cmd.append(os.path.join(share, 'worlds', world_name + '.sdf'))
    gz = ExecuteProcess(cmd=gz_cmd, output='screen',
                        additional_env={'GZ_SIM_RESOURCE_PATH': resource_path})

    bridge_file = 'bridge_rover.yaml' if rover else 'bridge.yaml'
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
                  output='screen',
                  parameters=[{'config_file': os.path.join(cfg_dir, bridge_file)}])

    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', world_name, '-name', 'simple_drone',
                            '-file', os.path.join(runtime_models, 'simple_drone', 'model.sdf'),
                            '-x', '%.6f' % spawn_x, '-y', '%.6f' % spawn_y,
                            '-z', '%.6f' % spawn_z])

    coupler = Node(package=PKG, executable='plate_coupler', name='plate_coupler',
                   output='screen',
                   parameters=[os.path.join(cfg_dir, 'coupling.yaml'), {'use_sim_time': True}])

    controller = Node(
        package=PKG, executable='landing_controller', name='landing_controller',
        output='screen',
        parameters=[os.path.join(cfg_dir, 'controller.yaml'), {
            'use_sim_time': True,
            'mode': mode,
            'scenario': scenario,
            'target_x': spawn_x,
            'target_y': spawn_y,
            'target_yaw_deg': float(sc['yaw_deg']),
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
            'spawn_z': spawn_z,
            'drone_mass': mass,
            'approach_height': float(plat['drone']['spawn_gap_closed_loop_m']),
            'frame_top_z': float(geo['frame_top_z']),
            'floor_top_standby_z': float(geo['floor_top_standby_z']),
            'gear_bottom_to_origin': float(geo['gear_bottom_to_origin_m']),
            'floor_hard_stop': float(sus['hard_stop_m']),
            'csv_dir': arg('csv_dir'),
            'shutdown_on_done': _truthy(arg('shutdown_on_done')),
        }])

    def _cleanup(_context):
        shutil.rmtree(runtime_root, ignore_errors=True)
        return []

    spawn_delay = float(arg('spawn_delay'))
    actions = [
        LogInfo(msg='[landing_sim] scenario=%s mode=%s gui=%s rover=%s '
                    'spawn=(%.3f, %.3f, %.4f) m'
                    % (scenario, mode, gui, rover, spawn_x, spawn_y, spawn_z)),
        LogInfo(msg='[landing_sim] k=%.1f N/m preload=%.2f N (+guide %.2f N) c=%.1f N s/m '
                    'stop=%.1f mm spring_reference=%.5f m, drone=%.3f kg, visual=%s/%s'
                    % (sus['stiffness_n_per_m'], sus['preload_n'],
                       sus.get('plate_guide_support_n', 0.0), sus['damping_ns_per_m'],
                       sus['hard_stop_m'] * 1000.0, q_ref, mass,
                       info['platform_visual'], info['drone_visual'])),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        gz,
        bridge,
        coupler,
        controller,
    ]
    if rover_parts is None:
        actions.append(TimerAction(period=spawn_delay, actions=[spawn]))
    else:
        # 로버+플랫폼을 먼저 스폰해 안착시킨 뒤 드론을 스폰한다.
        drone_delay = spawn_delay + float(arg('rover_settle_s'))
        actions += [rover_parts['log'], rover_parts['watch'],
                    TimerAction(period=spawn_delay, actions=[rover_parts['spawn']]),
                    TimerAction(period=drone_delay, actions=[spawn])]
    actions += [
        RegisterEventHandler(OnProcessExit(
            target_action=controller,
            on_exit=[EmitEvent(event=Shutdown(reason='landing_controller 종료'))])),
        RegisterEventHandler(OnProcessExit(
            target_action=gz,
            on_exit=[EmitEvent(event=Shutdown(reason='gz sim 종료'))])),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=_cleanup)])),
    ]
    return actions


def generate_launch_description():
    args = [
        DeclareLaunchArgument('scenario', default_value='center',
                              choices=['center', 'x30', 'x60', 'yaw30'],
                              description='착륙 시나리오'),
        DeclareLaunchArgument('mode', default_value='closed_loop',
                              choices=['closed_loop', 'sw_openloop'],
                              description='closed_loop: 상태기계 / sw_openloop: SolidWorks 추력 스케줄'),
        DeclareLaunchArgument('gui', default_value='true', description='false 이면 헤드리스(-s)'),
        DeclareLaunchArgument('visual', default_value='auto', choices=['auto', 'mesh', 'box'],
                              description='auto: STL 이 모두 있으면 메시, 없으면 박스'),
        DeclareLaunchArgument('damping', default_value='',
                              description='서스펜션 감쇠 c [N s/m] (빈칸 = platform.yaml)'),
        DeclareLaunchArgument('spring_k', default_value='',
                              description='서스펜션 강성 k [N/m] (빈칸 = platform.yaml)'),
        DeclareLaunchArgument('preload', default_value='',
                              description='대기 예압 [N] (빈칸 = platform.yaml)'),
        DeclareLaunchArgument('drone_mass', default_value='',
                              description='드론 질량 [kg] (빈칸 = platform.yaml)'),
        DeclareLaunchArgument('csv_dir', default_value='~/landing_results',
                              description="CSV/JSON 결과 폴더 ('' = 기록 안 함)"),
        DeclareLaunchArgument('shutdown_on_done', default_value='false',
                              description='DONE 후 launch 전체 종료'),
        DeclareLaunchArgument('gz_verbosity', default_value='3'),
        DeclareLaunchArgument('spawn_delay', default_value='3.0',
                              description='gz 기동 후 스폰까지 대기 [s]'),
        DeclareLaunchArgument('rover', default_value='false',
                              description='true 이면 로버(spacek_rover) 위 플랫폼 (월드 rover_landing)'),
        DeclareLaunchArgument('rover_settle_s', default_value='2.0',
                              description='rover:=true 에서 로버 스폰 후 드론 스폰까지 [s]'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
