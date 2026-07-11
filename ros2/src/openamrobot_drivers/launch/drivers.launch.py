"""
Real-robot hardware drivers: micro-ROS agent (Teensy) + RPLIDAR.

This is the host-side "thin" driver layer. The real control loop (per-wheel PID,
encoder reading, odometry, IMU) runs in the Teensy firmware; here we only bridge
it to ROS via the micro-ROS agent, and start the LiDAR driver.

Ports default to THIS unit's by-id device paths; override on another robot:
  ros2 launch openamrobot_drivers drivers.launch.py teensy_port:=/dev/ttyACM0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TEENSY_DEFAULT = '/dev/serial/by-id/usb-Teensyduino_USB_Serial_16778200-if00'
LIDAR_DEFAULT = ('/dev/serial/by-id/'
                 'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0')


def generate_launch_description():
    teensy = LaunchConfiguration('teensy_port')
    lidar = LaunchConfiguration('lidar_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='teensy_port', default_value=TEENSY_DEFAULT,
            description='Serial device of the Teensy (micro-ROS). Unit-specific by-id path.'),
        DeclareLaunchArgument(
            name='lidar_port', default_value=LIDAR_DEFAULT,
            description='Serial device of the RPLIDAR (CP2102). Unit-specific by-id path.'),
        # micro-ROS agent: bridges the Teensy (/cmd_vel, /odom/unfiltered, /imu/data).
        ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
                 'serial', '-b', '115200', '-D', teensy],
            output='screen'),
        # RPLIDAR A1 driver -> /scan in frame lidar_link.
        Node(
            package='rplidar_ros', executable='rplidar_composition', name='rplidar',
            parameters=[{
                'serial_port': lidar,
                'serial_baudrate': 115200,
                'frame_id': 'lidar_link',
                'angle_compensate': True,   # correct rplidar_ros param name (was angle_compensation — Raj review PR2)
                'scan_mode': 'Standard',
            }],
            respawn=True, respawn_delay=3.0,
            output='screen'),
    ])
