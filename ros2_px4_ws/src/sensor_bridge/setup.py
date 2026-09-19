from setuptools import find_packages, setup

package_name = 'sensor_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hassetorvalds',
    maintainer_email='hassetorvalds@users.noreply.github.com',
    description='传感器接入：把 AirSim 相机图像/深度以 ROS 2 话题发布',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'airsim_camera = sensor_bridge.airsim_camera:main',
        ],
    },
)
