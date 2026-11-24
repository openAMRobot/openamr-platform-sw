# SPDX-License-Identifier: MIT
from glob import glob
import os

from setuptools import setup

package_name = 'openamrobot_clearance_trials'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenAMRobot Maintainers',
    maintainer_email='botshare.ai@gmail.com',
    description='Obstacle clearance trials in simulation for OpenAMRobot (test only, Proposed).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'clearance_trials = openamrobot_clearance_trials.trials:main',
            'clearance_scene = openamrobot_clearance_trials.scene:main',
            'clearance_overrides = openamrobot_clearance_trials.overrides:main',
        ],
    },
)
