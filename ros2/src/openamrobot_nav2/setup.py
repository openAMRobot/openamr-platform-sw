from glob import glob
import os

from setuptools import setup

package_name = 'openamrobot_nav2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenAMRobot Maintainers',
    maintainer_email='botshare.ai@gmail.com',
    description=(
        'Nav2 bringup, configuration, maps, SLAM, localization, and '
        'navigation stack integration for OpenAMRobot.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
