import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# 매핑 모드: 공통 브링업 + slam_toolbox(온라인 async 매핑)
def generate_launch_description():
    pkg = get_package_share_directory('uel_rover_control')
    slam_params = os.path.join(pkg, 'config', 'slam_toolbox.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'bringup_common.launch.py'))),

        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', output='screen',
             parameters=[slam_params, {'use_sim_time': use_sim_time}]),

        # slam_toolbox는 lifecycle 노드라 configure/activate를 걸어줄
        # lifecycle_manager가 없으면 unconfigured 상태로 멈춰있는다
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_slam', output='screen',
             parameters=[{'autostart': True, 'node_names': ['slam_toolbox']}]),
    ])
