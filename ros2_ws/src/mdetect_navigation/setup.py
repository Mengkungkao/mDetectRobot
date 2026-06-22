from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'mdetect_navigation'
setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='Meng Kung Kao',
    maintainer_email='meng@example.com',
    description='Remote workstation SLAM, Nav2, RViz and waypoint tools for mDetect.',
    license='MIT',
    entry_points={'console_scripts': ['waypoint_cli = mdetect_navigation.waypoint_cli:main']},
)
