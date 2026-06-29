from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'arduino_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'param'), glob('param/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='M.K.',
    maintainer_email='mengkungkao@gmail.com',
    description='ROS2 bridge between Raspberry Pi and Arduino motor controller via UART serial.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'arduino_bridge_node = arduino_bridge.arduino_bridge_node:main',
        ],
    },
)
