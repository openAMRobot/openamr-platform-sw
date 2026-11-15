"""Nav2 in front of the simref bridge (sil.launch.py stm32:=simref nav2:=true).

The parameters are openamrobot_nav2/config/nav2_params.yaml, read at launch time, with the
few changes this SIL needs (simref_nav2_params below): no map and no localization, so the
global frame is odom; no lidar in the SIL model, so the costmaps hold only the inflation layer
and the collision monitor's scan source has no publisher and no timeout. The velocity chain is
the one of openamrobot_nav2/launch/navigation_launch.py and is not changed:
controller_server and behavior_server -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed
-> collision_monitor -> cmd_vel -> simref bridge -> reference process -> CAN1.
The collision monitor sees no obstacle in this mode. Development evidence only.
"""
import copy
import os
import tempfile

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

LIFECYCLE_NODES = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator',
                   'velocity_smoother', 'collision_monitor']
NO_SCAN_TOPIC = '/sil/simref/no_scan'


def simref_nav2_params(src):
    """Copy of the nav2_params.yaml dictionary with the SIL changes. Pure function, tested."""
    p = copy.deepcopy(src)

    def rp(node):
        return p[node]['ros__parameters']

    rp('bt_navigator')['global_frame'] = 'odom'
    for key in ('default_nav_to_pose_bt_xml', 'default_nav_through_poses_bt_xml'):
        rp('bt_navigator').pop(key, None)
    rp('behavior_server')['global_frame'] = 'odom'
    g = p['global_costmap']['global_costmap']['ros__parameters']
    g.update({'global_frame': 'odom', 'rolling_window': True, 'width': 12, 'height': 12,
              'plugins': ['inflation_layer']})
    loc = p['local_costmap']['local_costmap']['ros__parameters']
    loc['plugins'] = ['inflation_layer']
    cm = rp('collision_monitor')
    cm['source_timeout'] = 0.0
    for s in cm['observation_sources']:
        cm[s]['topic'] = NO_SCAN_TOPIC
        cm[s]['source_timeout'] = 0.0
    for node in LIFECYCLE_NODES:
        rp(node)['use_sim_time'] = True
    return p


def _setup(context):
    try:
        nav2_dir = get_package_share_directory('openamrobot_nav2')
    except PackageNotFoundError:
        return [LogInfo(msg='BLOCKED: openamrobot_nav2 is not built'),
                EmitEvent(event=Shutdown(reason='BLOCKED: openamrobot_nav2 missing'))]
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    with open(os.path.join(nav2_dir, 'config', 'nav2_params.yaml'), 'r', encoding='utf-8') as f:
        params = simref_nav2_params(yaml.safe_load(f))
    fd, path = tempfile.mkstemp(prefix='nav2_simref_', suffix='.yaml')
    with os.fdopen(fd, 'w', encoding='utf-8') as f:
        yaml.safe_dump(params, f)
    common = dict(output='screen', parameters=[path, {'use_sim_time': use_sim_time}])
    tf = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    return [
        LogInfo(msg='simref Nav2 parameters: %s (from %s)' % (path, nav2_dir)),
        Node(package='nav2_controller', executable='controller_server',
             remappings=tf + [('cmd_vel', 'cmd_vel_nav')], **common),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             remappings=tf, **common),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             remappings=tf + [('cmd_vel', 'cmd_vel_nav')], **common),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             remappings=tf, **common),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', remappings=tf + [('cmd_vel', 'cmd_vel_nav')], **common),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor', remappings=tf, **common),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                          'node_names': LIFECYCLE_NODES, 'bond_timeout': 60.0}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
