"""
Real-robot hardware drivers: micro-ROS agent (Teensy) + navigation LiDAR (sllidar_ros2).

This is the host-side "thin" driver layer. The real control loop (per-wheel PID,
encoder reading, odometry, IMU) runs in the Teensy firmware; here we only bridge
it to ROS via the micro-ROS agent, and start the LiDAR driver.

LiDAR: the OpenAMRobot 2.0 navigation LiDAR is the SLAMTEC RPLIDAR S3 (lidar_model:=s3,
default). The RPLIDAR A1 of the existing robot is kept as legacy (existing robot):
lidar_model:=a1. Both run the Slamtec sllidar_ros2 driver and publish /scan in frame
lidar_link. The LiDAR is functional sensing only, not a safety device.

Ports default to THIS unit's by-id device paths; override on another robot:
  ros2 launch openamrobot_drivers drivers.launch.py teensy_port:=/dev/ttyACM0
  ros2 launch openamrobot_drivers drivers.launch.py lidar_model:=a1   # legacy (existing robot)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node

TEENSY_DEFAULT = '/dev/serial/by-id/usb-Teensyduino_USB_Serial_16778200-if00'
# By-id path of the existing unit's CP2102 USB to UART adapter. To confirm on the S3 unit.
LIDAR_DEFAULT = ('/dev/serial/by-id/'
                 'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0')

# Per-model sllidar_ros2 settings. Upstream reference: Slamtec/sllidar_ros2 at commit
# 34300099fadfc772965962dec837bf436706188f (launch/sllidar_s3_launch.py and
# launch/sllidar_a1_launch.py).
LIDAR_MODELS = {
    # RPLIDAR S3 (OpenAMRobot 2.0): values from upstream sllidar_s3_launch.py.
    's3': {
        'serial_baudrate': 1000000,
        'scan_mode': 'DenseBoost',
    },
    # RPLIDAR A1, legacy (existing robot). Upstream sllidar_a1_launch.py defaults to
    # scan_mode 'Sensitivity'; we keep 'Standard', the mode this robot used so far.
    'a1': {
        'serial_baudrate': 115200,
        'scan_mode': 'Standard',
    },
}


def _lidar_node(model, serial_port):
    # sllidar_node publishes 'scan' (relative name, no namespace) -> /scan; no remap needed.
    return Node(
        package='sllidar_ros2', executable='sllidar_node', name='sllidar_node',
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('lidar_model'), model)),
        parameters=[{
            'channel_type': 'serial',
            'serial_port': serial_port,
            'serial_baudrate': LIDAR_MODELS[model]['serial_baudrate'],
            'frame_id': 'lidar_link',
            'angle_compensate': True,
            'scan_mode': LIDAR_MODELS[model]['scan_mode'],
        }],
        respawn=True, respawn_delay=3.0,
        output='screen')


def generate_launch_description():
    teensy = LaunchConfiguration('teensy_port')
    lidar = LaunchConfiguration('lidar_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='teensy_port', default_value=TEENSY_DEFAULT,
            description='Serial device of the Teensy (micro-ROS). Unit-specific by-id path.'),
        DeclareLaunchArgument(
            name='lidar_port', default_value=LIDAR_DEFAULT,
            description=('Serial device of the LiDAR USB to UART adapter. Unit-specific '
                         "by-id path; the default is the existing unit's CP2102 "
                         '(to confirm on the S3 unit).')),
        # choices= makes launch reject any other value before starting anything.
        DeclareLaunchArgument(
            name='lidar_model', default_value='s3', choices=sorted(LIDAR_MODELS),
            description=('Navigation LiDAR model: s3 = RPLIDAR S3 (OpenAMRobot 2.0), '
                         'a1 = RPLIDAR A1, legacy (existing robot).')),
        # micro-ROS agent: bridges the Teensy (/cmd_vel, /odom/unfiltered, /imu/data).
        ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
                 'serial', '-b', '115200', '-D', teensy],
            output='screen'),
        # LiDAR driver (sllidar_ros2) -> /scan in frame lidar_link; one node per model,
        # only the one matching lidar_model starts.
        *[_lidar_node(model, lidar) for model in sorted(LIDAR_MODELS)],
    ])
