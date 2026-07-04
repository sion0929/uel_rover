import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# 위치추정 모드: 공통 브링업 + map_server + AMCL (저장맵 주행)
def generate_launch_description():
    pkg = get_package_share_directory('uel_rover_control')
    loc_params = os.path.join(pkg, 'config', 'localization.yaml')

    map_yaml     = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    lifecycle_nodes = ['map_server', 'amcl']

    return LaunchDescription([
        DeclareLaunchArgument('map', description='저장맵 .yaml 경로 (예: /maps/office.yaml)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'bringup_common.launch.py'))),

        # map_server: localization 오버레이 로드 + map 경로 주입
        Node(package='nav2_map_server', executable='map_server',
             name='map_server', output='screen',
             parameters=[loc_params, {'yaml_filename': map_yaml,
                                      'use_sim_time': use_sim_time}]),

        Node(package='nav2_amcl', executable='amcl',
             name='amcl', output='screen',
             parameters=[loc_params, {'use_sim_time': use_sim_time}]),

        # map_server/amcl 전용 lifecycle 매니저
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'autostart': True, 'node_names': lifecycle_nodes}]),
    ])
