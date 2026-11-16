from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'openamrobot_sil'
vendor = os.path.join(package_name, 'vendor', 'can_sil')

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name + '.vendor.can_sil': ['LICENSE', 'PROVENANCE.md', 'SHA256SUMS', 'tests/*.py'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'models'), glob('config/models/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='OpenAMRobot Maintainers',
    maintainer_email='botshare.ai@gmail.com',
    description=(
        'Base and lift software-in-the-loop layer on Linux virtual CAN, bridged to Gazebo.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zlac8015d_emulator = openamrobot_sil.nodes.zlac8015d_node:main',
            'ctl017_emulator = openamrobot_sil.nodes.ctl017_node:main',
            'daly_bms_emulator = openamrobot_sil.nodes.daly_bms_node:main',
            'stm32_host_client = openamrobot_sil.nodes.stm32_host_client_node:main',
        ],
    },
)
