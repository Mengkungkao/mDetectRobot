from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'mdetect_bringup'
setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Meng Kung Kao',
    maintainer_email='meng@example.com',
    description='TurtleBot3-style onboard bringup for mDetect.',
    license='MIT',
)
