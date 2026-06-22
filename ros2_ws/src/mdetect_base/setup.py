from setuptools import find_packages, setup

package_name = 'mdetect_base'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Meng Kung Kao',
    maintainer_email='meng@example.com',
    description='Arduino serial base driver, command mux, safety gate, and startup monitor for mDetect.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'serial_bridge = mdetect_base.serial_bridge:main',
            'cmd_mux = mdetect_base.cmd_mux:main',
            'robot_initializer = mdetect_base.robot_initializer:main',
            'actuator_self_test = mdetect_base.actuator_self_test:main',
        ],
    },
)
