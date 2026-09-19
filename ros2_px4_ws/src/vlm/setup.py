from setuptools import find_packages, setup

package_name = 'vlm'

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
    description='视觉语言模型导航层：图像指向解析、几何反投影、机体速度指令生成',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'vlm_navigator = vlm.vlm_navigator:main',
        ],
    },
)
