from glob import glob
import os

from setuptools import setup

package_name = 'openamrobot_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenAMRobot Maintainers',
    maintainer_email='botshare.ai@gmail.com',
    description=(
        'Top-level launch composition for OpenAMRobot: real-robot data-source '
        'bring-up (drivers, perception, EKF, static TFs).'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
