from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hello_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # 기본 ROS2 패키지 설정
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # ➕ launch 폴더 설치 추가
        ('share/' + package_name + '/launch', glob('launch/*.launch.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pw',
    maintainer_email='pw@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'turtle_circle = hello_ros2.turtle_circle:main',
            'turtle_pose_subscriber = hello_ros2.turtle_pose_subscriber:main',
            'turtle_pose_live_monitor = hello_ros2.turtle_pose_live_monitor:main',
            'multi_spawning_turtle = hello_ros2.multi_spawning_turtle:main',
        ],
    },
)
