"""spacek 로버 단독 주행 시뮬레이션 런치 (ROS 2 Jazzy + Gazebo Harmonic).

예)
  ros2 launch spacek_rover rover_sim.launch.py
  ros2 launch spacek_rover rover_sim.launch.py drive_test:=true gui:=false shutdown_on_done:=true
  ros2 launch spacek_rover rover_sim.launch.py x:=1.0 y:=0.5 yaw:=1.57

동작:
  1) config/rover.yaml -> rover_params.derive() -> 모델 SDF 를 임시 폴더에 복사/수정
  2) GZ_SIM_RESOURCE_PATH 를 그 임시 폴더 + 설치된 models 로 설정 -> model:// 해석
  3) gz sim (gui:=false 이면 -s 헤드리스), ros_gz_bridge, 로버 스폰,
     drive_test:=true 이면 자동 주행 시험 노드 (끝나면 shutdown_on_done:=true 일 때 전체 종료)
"""

import os
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable,
                            TimerAction)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from spacek_rover.rover_params import derive, load_config, spawn_height
from spacek_rover.sdf_tools import MODEL_NAME, prepare_runtime_model

PKG = 'spacek_rover'
WORLD_NAME = 'rover_test'


def _truthy(s):
    return str(s).strip().lower() in ('1', 'true', 'yes', 'on')


def launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    share = get_package_share_directory(PKG)
    cfg_dir = os.path.join(share, 'config')
    gui = _truthy(arg('gui'))
    cfg = load_config(os.path.join(cfg_dir, 'rover.yaml'))
    params = derive(cfg)

    runtime_root = tempfile.mkdtemp(prefix='spacek_rover_')
    runtime_models = os.path.join(runtime_root, 'models')
    info = prepare_runtime_model(os.path.join(share, 'models'), runtime_models, params,
                                 arg('visual'))
    paths = [runtime_models, os.path.join(share, 'models')]
    old = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if old:
        paths.append(old)
    resource_path = os.pathsep.join(paths)

    gz_cmd = ['gz', 'sim', '-r', '-v', arg('gz_verbosity')]
    if not gui:
        gz_cmd.insert(2, '-s')
    gz_cmd.append(os.path.join(share, 'worlds', WORLD_NAME + '.sdf'))
    gz = ExecuteProcess(cmd=gz_cmd, output='screen',
                        additional_env={'GZ_SIM_RESOURCE_PATH': resource_path})

    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
                  output='screen',
                  parameters=[{'config_file': os.path.join(cfg_dir, 'bridge.yaml')}])

    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', WORLD_NAME, '-name', MODEL_NAME, '-file', info['sdf'],
                            '-x', arg('x'), '-y', arg('y'), '-z', '%.6f' % spawn_height(cfg),
                            '-Y', arg('yaw')])

    def _cleanup(_context):
        shutil.rmtree(runtime_root, ignore_errors=True)
        return []

    turn_note = '' if params['turn_margin'] > 1.0 else ' (경고: 1 이하 -> 제자리 선회 불가)'
    actions = [
        LogInfo(msg='[rover_sim] base %.2f kg + 4 x %.2f kg, wheel effort %.2f N m, '
                    'v_max %.3f m/s, w_max %.3f rad/s, mu long/lat %.2f/%.2f, '
                    'turn_margin %.2f%s, visual=%s'
                    % (params['base_mass'], params['wheel_mass'], params['wheel_effort'],
                       params['max_linear_velocity'], params['max_angular_velocity'],
                       params['mu_long'], params['mu_lat'], params['turn_margin'],
                       turn_note, info['visual'])),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        gz,
        bridge,
        TimerAction(period=float(arg('spawn_delay')), actions=[spawn]),
        RegisterEventHandler(OnProcessExit(
            target_action=gz,
            on_exit=[EmitEvent(event=Shutdown(reason='gz sim 종료'))])),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=_cleanup)])),
    ]
    if _truthy(arg('drive_test')):
        test = Node(package=PKG, executable='drive_test', name='drive_test', output='screen',
                    parameters=[os.path.join(cfg_dir, 'drive_test.yaml'), {
                        'use_sim_time': True,
                        'csv_dir': arg('csv_dir'),
                        'shutdown_on_done': _truthy(arg('shutdown_on_done')),
                        'ride_height_m': params['wheel_radius'],
                        'linear_accel_mps2': params['max_linear_accel'],
                        'angular_accel_radps2': params['max_angular_accel'],
                    }])
        actions.append(test)
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=test,
            on_exit=[EmitEvent(event=Shutdown(reason='drive_test 종료'))])))
    return actions


def generate_launch_description():
    args = [
        DeclareLaunchArgument('gui', default_value='true', description='false 이면 헤드리스(-s)'),
        DeclareLaunchArgument('visual', default_value='auto', choices=['auto', 'mesh', 'box'],
                              description='auto: STL 이 모두 있으면 메시, 없으면 박스/원통'),
        DeclareLaunchArgument('x', default_value='0.0', description='스폰 x [m]'),
        DeclareLaunchArgument('y', default_value='0.0', description='스폰 y [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='스폰 yaw [rad]'),
        DeclareLaunchArgument('drive_test', default_value='false',
                              description='true 이면 자동 주행 시험 노드 실행'),
        DeclareLaunchArgument('shutdown_on_done', default_value='false',
                              description='drive_test 가 끝나면 launch 전체 종료'),
        DeclareLaunchArgument('csv_dir', default_value='~/rover_results',
                              description="drive_test CSV/JSON 폴더 ('' = 기록 안 함)"),
        DeclareLaunchArgument('gz_verbosity', default_value='3'),
        DeclareLaunchArgument('spawn_delay', default_value='3.0',
                              description='gz 기동 후 로버 스폰까지 대기 [s]'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
