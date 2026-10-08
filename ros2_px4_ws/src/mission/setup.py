import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hassetorvalds',
    maintainer_email='hassetorvalds@users.noreply.github.com',
    description='任务层：航点状态机（起飞、航点巡航、降落）',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'waypoint_mission = mission.waypoint_mission:main',
            'approach_supervisor = mission.approach_supervisor:main',
        ],
    },
)
