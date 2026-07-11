"""
Top-level OpenAMRobot bring-up — pick simulation or real with ``sim:=``.

ONE command brings up the WHOLE stack (data source + localization + navigation),
for both profiles. The same Nav2 stack and the SAME ``nav2_params.yaml`` run in both
cases; only the data source and the clock differ.

* ``sim:=false`` (default) — real hardware: openamrobot_drivers + openamrobot_perception
  + EKF + measured static TFs, then localization + Nav2 with ``use_sim_time:=false`` and a
  ``/goal_pose -> /goal_pose_nav`` relay (so the RViz "2D Goal Pose" reaches Nav2). No Gazebo
  window; RViz only if ``use_rviz:=true``. Needs the real hardware and a real map.
* ``sim:=true`` — simulation: openamrobot_gazebo (Gazebo + gz_bridge + robot_state_publisher)
  publishes the same topics, then localization + Nav2 with ``use_sim_time:=true``, plus the docking
  layer (``use_docking:=true`` by default). Opens the Gazebo window; no hardware needed. This makes
  ``sim:=true`` the single command for the whole simulation (it replaces the legacy
  ``openamrobot_docking/bringup_sim.launch.py``, which still works for backwards compatibility).

Docking is ON by default in BOTH profiles. ``dock_trigger`` owns ``/goal_pose`` and forwards it to
``/goal_pose_nav`` (it IS the goal forwarder). With ``use_docking:=false`` a plain relay takes over
that forwarding for nav-only debugging. Exactly one forwarder runs — never both.

  ros2 launch openamrobot_bringup bringup.launch.py                              # real, everything
  ros2 launch openamrobot_bringup bringup.launch.py sim:=true use_rviz:=true     # sim, everything + RViz
  ros2 launch openamrobot_bringup bringup.launch.py map:=/path/to/real_map.yaml  # real, your map
  ros2 launch openamrobot_bringup bringup.launch.py sim:=false use_camera:=false # real, lighter (no camera)

For step-by-step / per-terminal launching (debugging), see openamrobot_bringup/README.md
"Individual commands"; each include below can be launched on its own.

The map is injected by writing a temporary copy of ``nav2_params.yaml`` with ``map_server``'s
``yaml_filename`` already set, then launching localization with ``map:=''`` (its simple,
no-condition path). This is deterministic — the stock ``localization_launch.py`` map-argument
mechanism (RewrittenYaml + ``map==''`` conditions) does not propagate reliably through an
include, leaving ``map_server`` with an empty ``yaml_filename`` and stalling the lifecycle.
"""
import atexit
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _truthy(value):
    return str(value).lower() in ('true', '1', 'yes')


def _params_with_map(nav2_share, map_file, is_sim):
    """Write a temp copy of nav2_params.yaml with map_server.yaml_filename = map_file.

    Returns the temp file path. Lets localization run its simple (no map-condition) node
    while map_server still gets the map — deterministic across includes.
    """
    src_yaml = os.path.join(nav2_share, 'config', 'nav2_params.yaml')
    with open(src_yaml) as f:
        data = yaml.safe_load(f)
    data.setdefault('map_server', {}).setdefault('ros__parameters', {})['yaml_filename'] = map_file
    if not is_sim:
        # The real robot is almost never at the map origin at startup. nav2_params.yaml has
        # set_initial_pose:true at (0,0) — correct for sim (robot spawns at origin), but it
        # mislocalizes the real robot. Disable it so the operator sets the pose with RViz
        # "2D Pose Estimate" (matches the old bringall.sh auto-pose step).
        data.setdefault('amcl', {}).setdefault('ros__parameters', {})['set_initial_pose'] = False
    fd, path = tempfile.mkstemp(prefix='nav2_params_map_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
    # Don't leak the temp file: remove it when the launch process exits (the nodes have read
    # their parameters by then). Avoids accumulating /tmp/nav2_params_map_* on the Pi.
    atexit.register(lambda p=path: os.path.exists(p) and os.remove(p))
    return path


def launch_setup(context, *args, **kwargs):
    nav2 = get_package_share_directory('openamrobot_nav2')
    bringup = get_package_share_directory('openamrobot_bringup')

    sim = LaunchConfiguration('sim').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    gui = LaunchConfiguration('gui').perform(context)
    use_camera = LaunchConfiguration('use_camera').perform(context)
    use_docking = LaunchConfiguration('use_docking').perform(context)
    map_file = LaunchConfiguration('map').perform(context)
    is_sim = _truthy(sim)

    # Map gating (Raj review PR2): the real robot must NOT silently fall back to the bundled sim map.
    if not map_file:
        if is_sim:
            map_file = os.path.join(nav2, 'maps', 'my_map.yaml')   # sim: the walled_world map
        else:
            raise RuntimeError(
                'sim:=false requires an explicit map: pass map:=/path/to/your_real_map.yaml '
                '(the bundled sim map is never used on the real robot).')
    if not os.path.exists(map_file):
        raise RuntimeError('map file not found: ' + map_file)

    def src(pkg, rel):
        return PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', rel))

    actions = []

    # --- data source: ONE of the two ---
    if is_sim:
        gazebo = get_package_share_directory('openamrobot_gazebo')
        actions.append(IncludeLaunchDescription(
            src(gazebo, 'gz_simulator.launch.py'),
            launch_arguments={'gui': gui}.items()))
        # Scan filtering is a DATA-SOURCE responsibility (Raj review PR1): the SIM profile owns its
        # laser_filters chain here (the REAL profile owns it via openamrobot_perception). This keeps
        # exactly ONE /scan_filtered publisher per profile; navigation_launch only consumes it.
        actions.append(Node(
            package='laser_filters', executable='scan_to_scan_filter_chain', name='scan_body_filter',
            parameters=[os.path.join(nav2, 'config', 'scan_body_filter.yaml'),
                        {'use_sim_time': True}],
            remappings=[('scan', '/scan'), ('scan_filtered', '/scan_filtered')]))
        # Goal routing (sim) — SYMMETRIC to the real branch below, gated by use_docking.
        # navigation_launch remaps bt_navigator's goal to goal_pose_nav (for the docking gate), so the
        # RViz "2D Goal Pose" (/goal_pose) must be forwarded or it never reaches Nav2. Exactly ONE
        # forwarder runs: dock_trigger (when docking) OR a plain relay (nav-only) — never both.
        if _truthy(use_docking):
            # Sim docking layer (gz camera_info bridge + sync + apriltag_sim + detected_dock_pose +
            # dock_trigger). dock_trigger OWNS /goal_pose and forwards it to /goal_pose_nav, so it IS
            # the forwarder (no relay). This folds the legacy openamrobot_docking/bringup_sim.launch.py
            # into the unified bringup: `sim:=true` now does Gazebo + Nav2 + docking in one command.
            docking = get_package_share_directory('openamrobot_docking')
            actions.append(IncludeLaunchDescription(
                src(docking, 'openamrobot_docking.launch.py'),
                launch_arguments={'use_sim_time': sim}.items()))
        else:
            # nav-only debug (no docking): the standalone relay forwarder (same brick you'd launch by
            # hand in the per-terminal workflow). Exactly one forwarder -> here the relay, no docking.
            actions.append(IncludeLaunchDescription(src(bringup, 'goal_relay.launch.py')))
    else:
        actions.append(IncludeLaunchDescription(
            src(bringup, 'real_bringup.launch.py'),
            launch_arguments={'use_camera': use_camera}.items()))
        # Goal routing (real only). navigation_launch remaps bt_navigator's goal to goal_pose_nav
        # so the docking node can gate /goal_pose (undock-before-navigate).
        if _truthy(use_docking):
            # Real docking: dock_trigger OWNS /goal_pose and forwards it to /goal_pose_nav (after
            # undocking if docked, immediately otherwise) -> replaces the placeholder relay AND
            # adds AprilTag docking. Needs the physical dock + tags (config/tags_36h11.yaml,
            # config/dock_trigger.yaml). Harmless if the dock is absent (just forwards goals).
            docking = get_package_share_directory('openamrobot_docking')
            actions.append(IncludeLaunchDescription(
                src(docking, 'docking_real.launch.py'),
                launch_arguments={'use_sim_time': sim}.items()))
        else:
            # nav-only debug (no docking): the standalone relay forwarder (same brick you'd launch by
            # hand in the per-terminal workflow). Exactly one forwarder -> here the relay, no docking.
            actions.append(IncludeLaunchDescription(src(bringup, 'goal_relay.launch.py')))

    # --- same Nav2 stack on top of either source ---
    # map injected via a temp params file (see module docstring); localization runs map:=''.
    params_with_map = _params_with_map(nav2, map_file, is_sim)
    actions.append(IncludeLaunchDescription(
        src(nav2, 'localization_launch.py'),
        launch_arguments={
            'map': '', 'params_file': params_with_map, 'use_sim_time': sim,
        }.items()))
    actions.append(IncludeLaunchDescription(
        src(nav2, 'navigation_launch.py'),
        # navigation_launch no longer runs a scan filter (it's owned by the data source above) —
        # it only consumes /scan_filtered. Exactly one /scan_filtered publisher per profile.
        launch_arguments={'use_sim_time': sim}.items()))

    if _truthy(use_rviz):
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', os.path.join(nav2, 'rviz', 'nav2_view.rviz')],
            parameters=[{'use_sim_time': is_sim}]))

    return actions


def generate_launch_description():
    nav2 = get_package_share_directory('openamrobot_nav2')
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim', default_value='false',
            description='true = Gazebo simulation; false = real hardware.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='Open RViz with the Nav2 view.'),
        DeclareLaunchArgument(
            'gui', default_value='false',
            description='Sim only: open the Gazebo 3D window (false = headless).'),
        DeclareLaunchArgument(
            'use_camera', default_value='true',
            description='Real only: launch the IMX708 camera (false = lighter load).'),
        DeclareLaunchArgument(
            'use_docking', default_value='true',
            description='Sim AND real: docking is ON by default (dock_trigger owns /goal_pose and IS '
                        'the goal forwarder). Set false only for nav-only debug -> a plain '
                        'topic_tools relay /goal_pose->/goal_pose_nav takes over as the forwarder. '
                        'Exactly one forwarder runs in either case.'),
        DeclareLaunchArgument(
            'map', default_value='',
            description='Map YAML for AMCL/localization. Empty -> sim uses the bundled walled_world '
                        'map; sim:=false REQUIRES an explicit map:= (no silent bundled-map fallback).'),
        OpaqueFunction(function=launch_setup),
    ])
