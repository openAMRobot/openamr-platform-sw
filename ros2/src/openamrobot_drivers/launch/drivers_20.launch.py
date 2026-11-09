"""
OpenAMRobot 2.0 base drivers: micro-ROS agent for the NUCLEO-H723ZG, RPLIDAR S3, status monitor.

Next to drivers.launch.py (legacy Teensy, unchanged). The base controller is the STM32H723ZG
on the NUCLEO-H723ZG (P-03 item 14). The first bench uses serial or USB CDC; micro-ROS over
Ethernet UDP has an open hard-fault report on this board, so Ethernet is a later step
(roadmap rev 2, decision D5). The agent is run with the ping and reconnect behaviour of the
micro-ROS agent defaults; the base stops on its own 200 ms command watchdog when the agent
or the link is lost, and the status monitor holds the command lock until an explicit Resume.

  ros2 launch openamrobot_drivers drivers_20.launch.py
  ros2 launch openamrobot_drivers drivers_20.launch.py base_port:=/dev/ttyACM0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Name made by the install udev rules for the NUCLEO ST-Link virtual COM port, when they are
# installed (openamr-platform-sw install/udev). On a bench without the rules, pass base_port.
BASE_PORT_DEFAULT = '/dev/openamr/nucleo_stlink_vcp'
# TODO(Evans): the firmware serial baud rate. 115200 is the legacy Teensy agent value and
# is only a placeholder until the STM32 transport is written.
BASE_BAUD_DEFAULT = '115200'
LIDAR_DEFAULT = ('/dev/serial/by-id/'
                 'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0')
# RPLIDAR S3 values, same as the s3 entry of drivers.launch.py (Slamtec sllidar_ros2).
LIDAR_S3 = {'serial_baudrate': 1000000, 'scan_mode': 'DenseBoost'}


def generate_launch_description():
    share = get_package_share_directory('openamrobot_drivers')
    monitor_params = os.path.join(share, 'config', 'base_status_monitor.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'base_port', default_value=BASE_PORT_DEFAULT,
            description='Serial or USB CDC device of the base controller (NUCLEO-H723ZG).'),
        DeclareLaunchArgument(
            'base_baud', default_value=BASE_BAUD_DEFAULT,
            description='Serial baud rate of the micro-ROS transport. TODO(Evans).'),
        DeclareLaunchArgument(
            'lidar_port', default_value=LIDAR_DEFAULT,
            description='Serial device of the RPLIDAR S3 USB to UART adapter.'),
        ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial',
                 '-b', LaunchConfiguration('base_baud'),
                 '-D', LaunchConfiguration('base_port')],
            name='micro_ros_agent_base', output='screen',
            respawn=True, respawn_delay=2.0),
        Node(
            package='sllidar_ros2', executable='sllidar_node', name='sllidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': LaunchConfiguration('lidar_port'),
                'serial_baudrate': LIDAR_S3['serial_baudrate'],
                'frame_id': 'lidar_link',
                'angle_compensate': True,
                'scan_mode': LIDAR_S3['scan_mode'],
            }],
            respawn=True, respawn_delay=3.0, output='screen'),
        Node(
            package='openamrobot_drivers', executable='base_status_monitor',
            name='base_status_monitor', parameters=[monitor_params], output='screen'),
    ])
