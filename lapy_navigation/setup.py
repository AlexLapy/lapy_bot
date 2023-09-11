from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lapy_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alexlapy',
    maintainer_email='alexlapyy@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'clicked_point_sub = lapy_navigation.clicked_point_sub:main',
            'initial_pose_pub = lapy_navigation.initial_pose_pub:main',
            'set_param = lapy_navigation.set_param:main',
            'go_to_pose = lapy_navigation.go_to_pose:main',
            'spot_recorder_sub = lapy_navigation.spot_recorder_sub:main',
            'spot_recorder_srv = lapy_navigation.spot_recorder_srv:main',
        ],
    },
)
