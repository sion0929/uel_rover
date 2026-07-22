import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# 공통 브링업: SLAM/localization을 제외한 모든 노드
# (rover 노드, TF, 카메라, scan 변환, Nav2 서버, lifecycle_manager)
def generate_launch_description():
    pkg = get_package_share_directory('uel_rover_control')

    urdf_path   = os.path.join(pkg, 'urdf',   'uel_rover.urdf')
    params      = os.path.join(pkg, 'config', 'rover_params.yaml')
    p2l_params  = os.path.join(pkg, 'config', 'pointcloud_to_laserscan.yaml')
    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    nav2_lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # ── 로버 기본 노드 ────────────────────────────────────────
        Node(package='uel_rover_control', executable='odom_publisher',
             name='odom_publisher', output='screen', parameters=[params]),

        # ── URDF → TF ─────────────────────────────────────────────
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_desc,
                          'use_sim_time': use_sim_time}]),

        # ── RealSense D455 ────────────────────────────────────────
        Node(package='realsense2_camera', executable='realsense2_camera_node',
             name='camera', output='screen',
             parameters=[{
                 'initial_reset': True,
                 'enable_depth': True,
                 'enable_color': True,
                 'enable_infra1': False,
                 'enable_infra2': False,
                 'pointcloud__neon_.enable': True,   # realsense-ros __neon_ 파라미터 버그
                                                       # (IntelRealSense/realsense-ros #3397)
                 'enable_sync': True,
                 'depth_module.depth_profile': '640x480x30',
                 'rgb_camera.color_profile': '640x480x30',
                 'enable_gyro': False,
                 'enable_accel': False,
             }]),

        # ── PointCloud2 → LaserScan (/scan) ───────────────────────
        Node(package='pointcloud_to_laserscan',
             executable='pointcloud_to_laserscan_node',
             name='pointcloud_to_laserscan', output='screen',
             parameters=[p2l_params],
             remappings=[('cloud_in', '/camera/camera/depth/color/points'),
                         ('scan', '/scan')]),

        # ── Nav2 핵심 서버 ────────────────────────────────────────
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen', parameters=[nav2_params],
             remappings=[('cmd_vel', '/cmd_vel_out')]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=[nav2_params]),
        # 복구 동작(spin/backup/drive_on_heading)도 controller_server와 같은
        # 경로로 나가야 한다. remap이 없으면 /cmd_vel로 발행되는데 구독자가
        # 없어(MCU는 /cmd_vel_out만 구독) 복구 명령이 버려진다.
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen', parameters=[nav2_params],
             remappings=[('cmd_vel', '/cmd_vel_out')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen', parameters=[nav2_params]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen', parameters=[nav2_params]),

        # ── Nav2 라이프사이클 매니저 (Nav2 서버만 관리) ───────────
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'autostart': True, 'node_names': nav2_lifecycle_nodes}]),
    ])
