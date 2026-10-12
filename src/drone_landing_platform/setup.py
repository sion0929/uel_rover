from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'drone_landing_platform'


def tree_data_files(src_dir):
    """src_dir 하위의 모든 파일을 share/<pkg>/<src_dir>/... 로 설치하는 목록을 만든다."""
    out = []
    for dirpath, _dirnames, filenames in os.walk(src_dir):
        files = [os.path.join(dirpath, f) for f in filenames]
        if files:
            out.append((os.path.join('share', package_name, dirpath), files))
    return out


data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml', 'README.md']),
    ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ('share/' + package_name + '/config', glob('config/*.yaml')),
    ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
    ('share/' + package_name + '/scripts', glob('scripts/*.sh')),
]
# models/ 는 meshes/*.stl 을 나중에 추가할 수 있으므로 디렉터리 전체를 설치한다.
# (STL 을 추가한 뒤에는 colcon build 를 다시 실행해야 share 에 복사된다.)
data_files += tree_data_files('models')

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sion0',
    maintainer_email='202042036@cbnu.ac.kr',
    description='Drone landing capture platform simulation (ROS 2 Jazzy + Gazebo Harmonic)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plate_coupler = drone_landing_platform.plate_coupler:main',
            'landing_controller = drone_landing_platform.landing_controller:main',
        ],
    },
)
