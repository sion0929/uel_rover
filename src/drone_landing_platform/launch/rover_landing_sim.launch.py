"""로버 위 착륙 플랫폼 시뮬레이션 (= landing_sim.launch.py rover:=true).

예)
  ros2 launch drone_landing_platform rover_landing_sim.launch.py scenario:=x60 \
      gui:=false shutdown_on_done:=true
그 밖의 인자는 landing_sim.launch.py 와 같다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = get_package_share_directory('drone_landing_platform')
    landing = PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'landing_sim.launch.py'))
    return LaunchDescription([
        IncludeLaunchDescription(landing, launch_arguments={'rover': 'true'}.items()),
    ])
