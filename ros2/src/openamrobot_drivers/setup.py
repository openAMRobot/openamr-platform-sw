from glob import glob
import os

from setuptools import setup

package_name = 'openamrobot_drivers'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenAMRobot Maintainers',
    maintainer_email='botshare.ai@gmail.com',
    description=(
        'Host-side hardware drivers for the real OpenAMRobot: micro-ROS agent '
        '(Teensy bridge) and LiDAR driver launch.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
