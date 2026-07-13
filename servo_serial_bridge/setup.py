from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'servo_serial_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=[
        'setuptools',
        'pyserial',
    ],
    zip_safe=True,
    maintainer='amsr',
    maintainer_email='user@example.com',
    description='ROS2 serial bridge for ESP32 servo mission device controller.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_bridge_node = servo_serial_bridge.servo_bridge_node:main',
        ],
    },
)