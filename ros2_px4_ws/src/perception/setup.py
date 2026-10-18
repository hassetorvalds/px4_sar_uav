from setuptools import find_packages, setup

package_name = 'perception'

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
    description='人员/目标检测：RGB(+深度) → 检测框 + 世界系位置（ENU）',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'victim_detector = perception.victim_detector:main',
        ],
    },
)
