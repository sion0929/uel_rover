import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg    = get_package_share_directory('uel_rover_control')
    params = os.path.join(pkg, 'config', 'rover_params.yaml')

    uros_port = LaunchConfiguration('uros_port', default='/dev/ttyACM0')
    uros_baud = LaunchConfiguration('uros_baud', default='115200')

    return LaunchDescription([
        DeclareLaunchArgument('uros_port', default_value='/dev/ttyACM0',
                              description='OpenRB-150 USB 포트'),
        DeclareLaunchArgument('uros_baud', default_value='115200',
                              description='OpenRB-150 통신 속도'),

        # ── 오도메트리 (OpenRB-150 바퀴 RPM → /odom) ────────────
        Node(
            package='uel_rover_control',
            executable='odom_publisher',
            name='odom_publisher',
            output='screen',
            parameters=[params],
        ),

        # ── 모드 관리 (RC ↔ 자율주행 전환) ─────────────────────
        Node(
            package='uel_rover_control',
            executable='mode_manager',
            name='mode_manager',
            output='screen',
            parameters=[params],
        ),
    ])
