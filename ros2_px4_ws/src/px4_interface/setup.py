from setuptools import find_packages, setup

package_name = 'px4_interface'

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
    description='PX4 接口层：QoS、坐标系转换、状态跟踪、Offboard 设定点流与指令处理',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'offboard_bridge = px4_interface.offboard_bridge:main',
        ],
    },
)
