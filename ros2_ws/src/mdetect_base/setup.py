from setuptools import find_packages, setup

package_name = 'mdetect_base'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/base.launch.py']),
        ('share/' + package_name + '/config', ['config/base.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Meng Kung Kao',
    maintainer_email='meng@example.com',
    description='Low-level ROS 2 bridge and safety nodes for the mDetect robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'serial_bridge = mdetect_base.serial_bridge:main',
            'safety_supervisor = mdetect_base.safety_supervisor:main',
            'timed_waypoint_executor = mdetect_base.timed_waypoint_executor:main',
            'send_waypoints = mdetect_base.waypoint_sender:main',
        ],
    },
)
