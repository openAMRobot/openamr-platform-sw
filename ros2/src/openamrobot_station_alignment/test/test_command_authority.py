# SPDX-License-Identifier: MIT
"""Command authority: this package publishes no velocity and starts no second navigator or controller.

Motion stays in Nav2: the alignment node is an action client of navigate_to_pose with the station tree of
openamrobot_nav2 (platform-sw #164). Static checks of the sources; the trial runner checks the live graph.
"""
import ast
import glob
import os

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NAV2 = os.path.join(os.path.dirname(PKG), "openamrobot_nav2")
SOURCES = sorted(glob.glob(os.path.join(PKG, "openamrobot_station_alignment", "*.py"))
                 + glob.glob(os.path.join(PKG, "launch", "*.py")) + glob.glob(os.path.join(PKG, "scripts", "*.py")))
NODES = [p for p in SOURCES if os.path.basename(p) in (
    "station_alignment_node.py", "station_marker_detector.py", "image_delay_relay.py")]


def tree(path):
    with open(path, encoding="utf-8") as f:
        return ast.parse(f.read(), path)


def names(node):
    return {n.id for n in ast.walk(node) if isinstance(n, ast.Name)} | {
        n.attr for n in ast.walk(node) if isinstance(n, ast.Attribute)}


def test_sources_exist():
    assert len(SOURCES) >= 8 and len(NODES) == 3


def test_no_velocity_type_and_no_cmd_vel_in_the_nodes():
    assert len(NODES) == 3
    for path in NODES:
        t = tree(path)
        assert not names(t) & {"Twist", "TwistStamped"}, path
        strings = [n.value for n in ast.walk(t) if isinstance(n, ast.Constant) and isinstance(n.value, str)]
        assert not [s for s in strings if "cmd_vel" in s], path


def test_publishers_are_only_the_station_pose_and_the_report():
    published = []
    for path in NODES:
        for call in ast.walk(tree(path)):
            if (isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute)
                    and call.func.attr == "create_publisher"):
                published.append(ast.unparse(call.args[0]))
    assert sorted(published) == ["Image", "PoseWithCovarianceStamped", "String"]


def test_the_alignment_node_uses_nav2_actions_only():
    t = tree(os.path.join(PKG, "openamrobot_station_alignment", "station_alignment_node.py"))
    clients = sorted(ast.unparse(c.args[1]) for c in ast.walk(t)
                     if isinstance(c, ast.Call) and ast.unparse(c.func) in ("ActionClient", "ActionServer"))
    # Nav2's navigate_to_pose (the #164 approach and rotation-only goals; and this node's own server) and
    # behavior_server's BackUp and DriveOnHeading (odom-frame translations): all through Nav2's smoother and
    # collision monitor. No Spin: it stops on odometry, which misses part of the turn.
    assert clients == ["BackUp", "DriveOnHeading", "NavigateToPose", "NavigateToPose"]
    strings = {n.value for n in ast.walk(t) if isinstance(n, ast.Constant) and isinstance(n.value, str)}
    assert "navigate_to_station.xml" in strings and "navigate_to_pose" in strings


def _launch_nodes(path):
    out = []
    for call in ast.walk(tree(path)):
        if isinstance(call, ast.Call) and ast.unparse(call.func) == "Node":
            kw = {k.arg: ast.unparse(k.value).strip("'\"") for k in call.keywords}
            out.append((kw.get("package"), kw.get("executable")))
    return out


def test_robot_launch_starts_the_detector_the_alignment_and_apriltag_only():
    nodes = _launch_nodes(os.path.join(PKG, "launch", "station_alignment.launch.py"))
    # station_marker_detector and station_alignment run in one process, station_alignment_stack (stack.py)
    assert sorted(nodes) == sorted([("apriltag_ros", "apriltag_node"),
                                    ("openamrobot_station_alignment", "station_alignment_stack")])


def test_sim_launch_has_one_nav2_and_no_other_motion_source():
    path = os.path.join(PKG, "launch", "station_alignment_sim.launch.py")
    with open(path, encoding="utf-8") as f:
        text = f.read()
    assert text.count('"navigation_launch.py"') == 1 and text.count('"localization_launch.py"') == 1
    for package, executable in _launch_nodes(path):
        assert not (package or "").startswith(("nav2", "opennav")), (package, executable)
        assert executable not in ("dock_trigger.py", "controller_server", "bt_navigator", "docking_server")


def test_the_station_tree_of_platform_sw_164_is_present():
    path = os.path.join(NAV2, "behavior_trees", "navigate_to_station.xml")
    with open(path, encoding="utf-8") as f:
        xml = f.read()
    assert 'goal_checker_id="station_goal_checker"' in xml and 'controller_id="StationApproach"' in xml


def test_the_alignment_node_uses_no_wall_clock():
    """Every deadline on the node clock, simulation time with use_sim_time (Baseline 1 at a real-time factor of 0.2;
    353e3d6 bounded the sample window and Nav2 goals with time.monotonic as well)."""
    t = tree(os.path.join(PKG, "openamrobot_station_alignment", "station_alignment_node.py"))
    # time.sleep only paces the polling; every deadline is on the node clock (simulation time)
    assert "monotonic" not in {n.attr for n in ast.walk(t) if isinstance(n, ast.Attribute)}


def test_the_image_relay_publishes_reliable():
    """A best-effort publisher reaches no reliable subscriber (camera_info_sync): run 44168522141 got no image."""
    t = tree(os.path.join(PKG, "openamrobot_station_alignment", "image_delay_relay.py"))
    calls = [c for c in ast.walk(t) if isinstance(c, ast.Call) and getattr(c.func, "attr", "") == "create_publisher"]
    assert len(calls) == 1 and ast.unparse(calls[0].args[2]) != "qos_profile_sensor_data"


def test_corrections_are_not_map_frame_position_goals():
    """NavigateToPose is sent for the nominal #164 approach and for rotation-only goals at the current map position;
    a corrected position goal in map moved with every AMCL update, and #164's controller cannot reverse."""
    t = tree(os.path.join(PKG, "openamrobot_station_alignment", "station_alignment_node.py"))
    calls = [c for c in ast.walk(t) if isinstance(c, ast.Call) and getattr(c.func, "attr", "") == "_navigate"]
    assert len(calls) == 2
    rotate = next(f for f in ast.walk(t) if isinstance(f, ast.FunctionDef) and f.name == "_rotate")
    source = ast.unparse(rotate)
    assert "x, y, yaw = self._map_from_base()" in source and "(x, y, geometry.wrap(yaw + delta))" in source
    translations = [c for c in ast.walk(t) if isinstance(c, ast.Call) and getattr(c.func, "attr", "") == "_behavior"]
    assert len(translations) == 1


def test_station_approach_controller_cannot_reverse():
    """Why the correction does not reverse through StationApproach: nav2_graceful_controller turns allow_backward
    off when initial_rotation is on (log: "setting allow backward to false")."""
    import yaml
    with open(os.path.join(NAV2, "config", "nav2_params.yaml"), encoding="utf-8") as f:
        params = yaml.safe_load(f)["controller_server"]["ros__parameters"]["StationApproach"]
    assert params["initial_rotation"] is True and params["allow_backward"] is True
