# SPDX-License-Identifier: MIT
from glob import glob
import os

from setuptools import setup

package_name = 'openamrobot_station_alignment'


def tree(directory):
    """(install dir, files) for every directory below `directory`."""
    out = []
    for root, _, files in os.walk(directory):
        if files:
            out.append((os.path.join('share', package_name, root), [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ] + tree('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenAMRobot Maintainers',
    maintainer_email='botshare.ai@gmail.com',
    description='Station-relative final alignment for OpenAMRobot (Proposed).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'station_marker_detector = openamrobot_station_alignment.station_marker_detector:main',
            'station_alignment = openamrobot_station_alignment.station_alignment_node:main',
            'station_alignment_trials = openamrobot_station_alignment.trials:main',
        ],
    },
)
