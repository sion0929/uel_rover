import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# ═══════════════════════════════════════════════════════════════
#  UEL Rover — Nav2 통합 브링업 (RealSense D455 기반)
#
#  TF:  map ─(slam_toolbox)→ odom ─(odom_publisher)→ base_link
#                                                      └(URDF)→ camera_link
#  데이터: D455 depth → pointcloud_to_laserscan → /scan
#          /scan → slam_toolbox(지도+위치) + Nav2 costmap
#          Nav2 controller → /cmd_vel → mode_manager(AUTO) → OpenRB-150
#
#  ※ nav2_bringup의 navigation_launch.py를 쓰지 않고 핵심 서버만 직접 구동
#    (collision_monitor/velocity_smoother/route_server/docking 제외 —
#     불필요한 노드가 lifecycle 브링업을 막는 것 방지)
# ═══════════════════════════════════════════════════════════════
def generate_launch_description():
    pkg = get_package_share_directory('uel_rover_control')

    urdf_path   = os.path.join(pkg, 'urdf',   'uel_rover.urdf')
    p2l_params  = os.path.join(pkg, 'config', 'pointcloud_to_laserscan.yaml')
    slam_params = os.path.join(pkg, 'config', 'slam_toolbox.yaml')
    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    # Nav2 라이프사이클로 관리할 핵심 노드 목록
    nav2_lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # ── 기존 로버 노드 (odom_publisher, mode_manager) ────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'rover.launch.py')),
        ),

        # ── URDF → TF (base_link, 바퀴, camera_link) ─────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc,
                         'use_sim_time': use_sim_time}],
        ),

        # ── RealSense D455 드라이버 ──────────────────────────────
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'initial_reset': True,              # 시작 시 카메라 리셋(HW 에러 방지)
                'enable_depth': True,
                'enable_color': True,
                'enable_infra1': False,
                'enable_infra2': False,
                'pointcloud.enable': True,          # depth → PointCloud2
                'enable_sync': True,
                'depth_module.depth_profile': '640x480x30',
                'rgb_camera.color_profile': '640x480x30',
                'enable_gyro': False,               # IMU(HID) 미인식 → 비활성
                'enable_accel': False,
            }],
        ),

        # ── PointCloud2 → LaserScan (/scan) ──────────────────────
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[p2l_params],
            remappings=[('cloud_in', '/camera/depth/color/points'),
                        ('scan', '/scan')],
        ),

        # ── SLAM (map + map→odom TF) ─────────────────────────────
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params, {'use_sim_time': use_sim_time}],
        ),

        # ── Nav2 핵심 서버 ───────────────────────────────────────
        Node(
            package='nav2_controller', executable='controller_server',
            name='controller_server', output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_planner', executable='planner_server',
            name='planner_server', output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_behaviors', executable='behavior_server',
            name='behavior_server', output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_bt_navigator', executable='bt_navigator',
            name='bt_navigator', output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_waypoint_follower', executable='waypoint_follower',
            name='waypoint_follower', output='screen',
            parameters=[nav2_params],
        ),

        # ── Nav2 라이프사이클 매니저 ─────────────────────────────
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[{
                'autostart': True,
                'node_names': nav2_lifecycle_nodes,
            }],
        ),
    ])
