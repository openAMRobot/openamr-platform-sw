"""Checks of the 2.0 real-robot profile: command chain, limits, lock wiring, legacy intact."""
import importlib.util
import os

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import pytest
import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.dirname(PKG)
LAUNCH_20 = os.path.join(PKG, 'launch', 'real_bringup_20.launch.py')
DRIVERS_20 = os.path.join(SRC, 'openamrobot_drivers', 'launch', 'drivers_20.launch.py')
MONITOR_YAML = os.path.join(SRC, 'openamrobot_drivers', 'config', 'base_status_monitor.yaml')


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda pkg: os.path.join('/fake-share', pkg)
    return module


def _chain():
    with open(os.path.join(PKG, 'config', 'cmd_chain_20.yaml')) as handle:
        return yaml.safe_load(handle)


def _nodes(path, name):
    module = _load(path, name)
    return [e for e in module.generate_launch_description().entities if isinstance(e, Node)]


def _remaps(node):
    return {(_text(a), _text(b)) for a, b in node._Node__remappings or []}


def _by_exec(nodes, executable):
    found = [n for n in nodes if n.node_executable == executable]
    assert len(found) == 1, executable
    return found[0]


def _text(substitutions):
    return perform_substitutions(LaunchContext(), substitutions)


# --- limits -----------------------------------------------------------------
def test_speed_ceiling_is_half_a_metre_per_second_and_no_cap():
    smoother = _chain()['velocity_smoother']['ros__parameters']
    assert smoother['max_velocity'][0] == 0.5
    assert smoother['max_velocity'][0] != 0.3


def test_reverse_is_off_for_build_1():
    assert _chain()['velocity_smoother']['ros__parameters']['min_velocity'][0] == 0.0


def test_travel_deceleration_is_1_5_negative_sign():
    assert _chain()['velocity_smoother']['ros__parameters']['max_decel'][0] == -1.5


def test_smoother_feedback_and_timeout_stop_quickly_when_the_mux_goes_silent():
    params = _chain()['velocity_smoother']['ros__parameters']
    assert params['velocity_timeout'] <= 0.3


# --- collision monitor: one definition only (P-03 item 30 A2, decision D-16) -------------
# The stop zone is defined once, in openamrobot_nav2/config/nav2_params.yaml, which the
# stop-zone check of platform-sw #113 reads. The review of platform-sw #132 found the
# draft's second definition here in the wrong frame (base_footprint, D-16 says base_link)
# and in the wrong order (stopped, rotation, forward). It is removed, not repaired twice.
NAV2_PARAMS = os.path.join(SRC, 'openamrobot_nav2', 'config', 'nav2_params.yaml')


def test_cmd_chain_20_carries_no_second_collision_monitor_definition():
    chain = _chain()
    assert 'collision_monitor' not in chain
    text = open(os.path.join(PKG, 'config', 'cmd_chain_20.yaml')).read()
    for key in ('polygons:', 'velocity_polygons', 'observation_sources', 'StopZone'):
        assert key not in text, key


def test_collision_monitor_in_the_2_0_launch_reads_the_single_nav2_definition():
    nodes = _nodes(LAUNCH_20, 'real_bringup_20_cm')
    collision = _by_exec(nodes, 'collision_monitor')
    params = []
    for param in collision._Node__parameters:
        param = getattr(param, 'param_file', param)
        params.append(param if isinstance(param, str) else _text(param))
    expected = os.path.join('/fake-share', 'openamrobot_nav2', 'config', 'nav2_params.yaml')
    assert params == [expected]
    assert not _remaps(collision)


def test_the_single_definition_exists_and_writes_the_final_cmd_vel():
    with open(NAV2_PARAMS) as handle:
        cm = yaml.safe_load(handle)['collision_monitor']['ros__parameters']
    assert cm['cmd_vel_in_topic'] == 'cmd_vel_smoothed'
    assert cm['cmd_vel_out_topic'] == 'cmd_vel'
    assert cm['base_frame_id'] == 'base_link'      # D-16


# Test oracle for the D-16 sub-polygon rules (forward, rotation, stopped in that order;
# first match wins, as in nav2_collision_monitor VelocityPolygon). It is applied below to
# fixtures; the owner of platform-sw #113 can apply it to the real zone when it lands.
D16_ORDER = ['forward', 'rotation', 'stopped']


def select_sub_polygon(zone, linear_x, angular_z):
    """Return the first sub-polygon whose velocity window holds the command, else None."""
    for name in zone['velocity_polygons']:
        sub = zone[name]
        if (sub['linear_min'] <= linear_x <= sub['linear_max'] and
                sub['theta_min'] <= angular_z <= sub['theta_max']):
            return name
    return None


def d16_violations(zone):
    """
    List the D-16 ordering and selection problems of a velocity_polygon stop zone.

    Order: forward first, stopped (catch-all) last, rotation in between. nav2 gives each
    sub-polygon one velocity window, so rotation in both directions without containing the
    zero command needs two windows; names starting with 'rotation' count as rotation.
    """
    problems = []
    order = list(zone.get('velocity_polygons', []))
    middle = order[1:-1]
    if (len(order) < 3 or order[0] != 'forward' or order[-1] != 'stopped' or
            not all(n.startswith('rotation') for n in middle)):
        problems.append('order %r, D-16 wants forward, rotation, stopped' % (order,))
    cases = [((0.0, 0.0), 'stopped'), ((0.2, 0.0), 'forward'), ((0.5, 0.0), 'forward'),
             ((0.0, 0.3), 'rotation'), ((0.0, -0.3), 'rotation')]
    for (v, w), want in cases:
        got = select_sub_polygon(zone, v, w) or 'none'
        if not got.startswith(want):
            problems.append('command (%.2f, %.2f) selects %s, want %s' % (v, w, got, want))
    return problems


def _window(lmin, lmax, tmin, tmax):
    return {'linear_min': lmin, 'linear_max': lmax, 'theta_min': tmin, 'theta_max': tmax}


def test_oracle_rejects_the_draft_zone_order_and_overlap():
    draft = {'velocity_polygons': ['stopped', 'rotation', 'forward'],
             'forward': _window(0.01, 0.5, -0.5, 0.5),
             'rotation': _window(-0.01, 0.01, -0.5, 0.5),
             'stopped': _window(-0.01, 0.01, -0.01, 0.01)}
    assert d16_violations(draft)
    # the reviewer's point: in the D-16 order the zero command lands in rotation
    reordered = dict(draft, velocity_polygons=D16_ORDER)
    assert select_sub_polygon(reordered, 0.0, 0.0) == 'rotation'
    assert d16_violations(reordered)


def test_oracle_accepts_disjoint_windows_in_the_d16_order():
    split = {'velocity_polygons': ['forward', 'rotation_ccw', 'rotation_cw', 'stopped'],
             'forward': _window(0.02, 0.5, -0.5, 0.5),
             'rotation_ccw': _window(-0.01, 0.01, 0.02, 0.5),
             'rotation_cw': _window(-0.01, 0.01, -0.5, -0.02),
             'stopped': _window(-0.5, 0.5, -0.5, 0.5)}
    assert d16_violations(split) == []
    assert select_sub_polygon(split, 0.0, 0.0) == 'stopped'
    assert select_sub_polygon(split, 0.0, 0.3) == 'rotation_ccw'
    assert select_sub_polygon(split, 0.0, -0.3) == 'rotation_cw'


def test_chain_lifecycle_manager_bond_off_and_only_the_two_nodes():
    manager = _chain()['lifecycle_manager_cmd_chain']['ros__parameters']
    assert manager['bond_timeout'] == 0.0
    assert manager['node_names'] == ['velocity_smoother', 'collision_monitor']


# --- mux lock ---------------------------------------------------------------
def test_mux_lock_is_the_monitor_topic_and_fails_closed():
    lock = _chain()['twist_mux']['ros__parameters']['locks']['base_status']
    assert lock['topic'] == '/base/command_lock'
    assert 0.0 < lock['timeout'] <= 0.3     # timeout 0 would never expire: fail open
    assert lock['priority'] == 255
    with open(MONITOR_YAML) as handle:
        monitor = yaml.safe_load(handle)['base_status_monitor']['ros__parameters']
    assert monitor['lock_topic'] == lock['topic']
    assert 1.0 / monitor['lock_rate_hz'] < lock['timeout'] / 2


def test_mux_inputs_have_lower_priority_than_the_lock():
    params = _chain()['twist_mux']['ros__parameters']
    lock = params['locks']['base_status']['priority']
    assert all(t['priority'] < lock for t in params['topics'].values())


def test_monitor_stale_times_are_the_i8_values():
    with open(MONITOR_YAML) as handle:
        monitor = yaml.safe_load(handle)['base_status_monitor']['ros__parameters']
    assert monitor['status_stale_s'] == 0.5
    assert monitor['aux_stale_s'] == 0.6


# --- launch wiring ----------------------------------------------------------
def test_chain_nodes_are_wired_mux_to_smoother_to_monitor_to_cmd_vel():
    nodes = _nodes(LAUNCH_20, 'real_bringup_20')
    mux = _by_exec(nodes, 'twist_mux')
    smoother = _by_exec(nodes, 'velocity_smoother')
    collision = _by_exec(nodes, 'collision_monitor')
    assert ('cmd_vel_out', 'cmd_vel_mux') in _remaps(mux)
    assert ('cmd_vel', 'cmd_vel_mux') in _remaps(smoother)
    # collision_monitor reads cmd_vel_smoothed and writes cmd_vel by its own parameters,
    # from the single definition in nav2_params.yaml
    with open(NAV2_PARAMS) as handle:
        cm = yaml.safe_load(handle)['collision_monitor']['ros__parameters']
    assert cm['cmd_vel_in_topic'] == 'cmd_vel_smoothed' and cm['cmd_vel_out_topic'] == 'cmd_vel'
    assert not _remaps(collision)


def test_only_collision_monitor_writes_the_final_cmd_vel():
    nodes = _nodes(LAUNCH_20, 'real_bringup_20')
    mux = _by_exec(nodes, 'twist_mux')
    # mux output is moved off /cmd_vel; smoother output is cmd_vel_smoothed
    assert ('cmd_vel_out', 'cmd_vel_mux') in _remaps(mux)
    smoother_params = _chain()['velocity_smoother']['ros__parameters']
    assert 'cmd_vel_out_topic' not in smoother_params


def test_ekf_reads_imu_data_raw_and_odom_unfiltered():
    with open(os.path.join(PKG, 'config', 'ekf_20.yaml')) as handle:
        ekf = yaml.safe_load(handle)['ekf_filter_node']['ros__parameters']
    assert ekf['imu0'] == '/imu/data_raw'
    assert ekf['odom0'] == '/odom/unfiltered'
    assert ekf['base_link_frame'] == 'base_footprint'
    # yaw rate only from the IMU
    assert ekf['imu0_config'].count(True) == 1 and ekf['imu0_config'][11] is True


def test_launch_has_no_hand_written_static_transforms():
    nodes = _nodes(LAUNCH_20, 'real_bringup_20')
    assert not [n for n in nodes if n.node_executable == 'static_transform_publisher']
    assert _by_exec(nodes, 'robot_state_publisher')


def test_launch_includes_drivers_20_not_the_legacy_drivers():
    module = _load(LAUNCH_20, 'real_bringup_20_inc')
    includes = [e for e in module.generate_launch_description().entities
                if isinstance(e, IncludeLaunchDescription)]
    sources = [_text(i.launch_description_source._LaunchDescriptionSource__location)
               for i in includes]
    assert os.path.join('/fake-share', 'openamrobot_drivers', 'launch',
                        'drivers_20.launch.py') in sources
    assert not any(s.endswith('/drivers.launch.py') for s in sources)


def test_drivers_20_starts_the_nucleo_agent_the_lidar_and_the_monitor():
    module = _load(DRIVERS_20, 'drivers_20')
    entities = module.generate_launch_description().entities
    args = {e.name: e for e in entities if isinstance(e, DeclareLaunchArgument)}
    assert args['base_port'].default_value[0].text == '/dev/openamr/nucleo_stlink_vcp'
    execs = [e for e in entities if type(e).__name__ == 'ExecuteProcess']
    assert len(execs) == 1
    context = LaunchContext()
    for entity in entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    cmd = [perform_substitutions(context, part) for part in execs[0].cmd]
    assert cmd[cmd.index('-b') + 1] == '115200' and cmd[cmd.index('-D') + 1] == \
        '/dev/openamr/nucleo_stlink_vcp'
    assert cmd[:4] == ['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent']
    assert 'serial' in cmd and '-D' in cmd
    assert 'udp4' not in cmd                      # first bench is serial or USB CDC
    nodes = [e for e in entities if isinstance(e, Node)]
    assert {n.node_executable for n in nodes} == {'sllidar_node', 'base_status_monitor'}


@pytest.mark.parametrize('legacy', ['real_bringup.launch.py', 'bringup.launch.py'])
def test_legacy_launch_files_do_not_mention_the_2_0_profile(legacy):
    with open(os.path.join(PKG, 'launch', legacy)) as handle:
        text = handle.read()
    assert 'drivers_20' not in text and 'cmd_chain_20' not in text
    assert 'base_status_monitor' not in text
