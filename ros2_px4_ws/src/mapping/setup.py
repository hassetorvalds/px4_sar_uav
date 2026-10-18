from setuptools import find_packages, setup

package_name = 'mapping'

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
    description='局部地图：深度点云 → 体素障碍地图 + 前方最近障碍距离',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'local_map = mapping.local_map:main',
            'coverage_monitor = mapping.coverage_monitor:main',
        ],
    },
)
