# SPDX-License-Identifier: MIT
"""Command authority: this package publishes no velocity and starts no second navigator or controller.

Motion stays in Nav2: the alignment node is an action client of navigate_to_pose with the station tree of
openamrobot_nav2 (platform-sw #164). Static checks of the sources; the trial runner checks the live graph.
"""
import ast
import glob
import os
import shutil

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NAV2 = os.path.join(os.path.dirname(PKG), "openamrobot_nav2")
SOURCES = sorted(glob.glob(os.path.join(PKG, "openamrobot_station_alignment", "*.py"))
                 + glob.glob(os.path.join(PKG, "launch", "*.py")) + glob.glob(os.path.join(PKG, "scripts", "*.py")))
VELOCITY_TYPES = {"Twist", "TwistStamped"}
# Every publisher the package may create (Kunal's review of ea8ed13, point 6: the list of three node files missed
# stack.py and scripts/*.py). A new publisher anywhere in the package fails here until it is added deliberately.
ALLOWED_PUBLISHERS = ["Image", "PoseWithCovarianceStamped", "String"]


def python_files(root):
    """Every Python file of the package below `root`, tests excepted."""
    out = []
    for folder, dirs, files in os.walk(root):
        dirs[:] = sorted(d for d in dirs if d not in ("test", "__pycache__", ".pytest_cache"))
        out += [os.path.join(folder, f) for f in sorted(files) if f.endswith(".py")]
    return out


def tree(path):
    with open(path, encoding="utf-8") as f:
        return ast.parse(f.read(), path)


def names(node):
    return {n.id for n in ast.walk(node) if isinstance(n, ast.Name)} | {
        n.attr for n in ast.walk(node) if isinstance(n, ast.Attribute)}


def command_scan(paths):
    """(findings, publisher types): a velocity type named anywhere, a publisher whose topic is a velocity command,
    and the type of every create_publisher call."""
    findings, published = [], []
    for path in paths:
        t = tree(path)
        if names(t) & VELOCITY_TYPES:
            findings.append((os.path.basename(path), "velocity message type"))
        for call in ast.walk(t):
            if (isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute)
                    and call.func.attr == "create_publisher"):
                kind = ast.unparse(call.args[0]) if call.args else "?"
                published.append(kind)
                topic = call.args[1:2] + [k.value for k in call.keywords if k.arg == "topic"]
                strings = [n.value for a in topic for n in ast.walk(a)
                           if isinstance(n, ast.Constant) and isinstance(n.value, str)]
                if [x for x in strings if "cmd_vel" in x]:
                    findings.append((os.path.basename(path), f"publisher on a cmd_vel topic ({kind})"))
    return findings, sorted(published)


def test_sources_exist():
    files = {os.path.relpath(p, PKG) for p in python_files(PKG)}
    assert len(SOURCES) >= 8 and {os.path.relpath(p, PKG) for p in SOURCES} <= files
    for required in ("openamrobot_station_alignment/stack.py",
                     "openamrobot_station_alignment/station_alignment_node.py",
                     "scripts/apply_pr90_behaviour_set.py", "scripts/check_camera_pose.py", "setup.py"):
        assert required in files, required


def test_no_python_file_of_the_package_publishes_a_velocity():
    findings, published = command_scan(python_files(PKG))
    assert findings == [] and published == ALLOWED_PUBLISHERS


def test_a_new_velocity_publisher_anywhere_in_the_package_is_caught(tmp_path):
    """The package as it is passes the scan; the same package plus one new script that publishes cmd_vel fails it,
    in a file no list names (scripts/), and a new module with any other publisher changes the publisher list."""
    copy = tmp_path / "pkg"
    shutil.copytree(PKG, copy, ignore=shutil.ignore_patterns("test", "__pycache__", ".pytest_cache"))
    assert (copy / "openamrobot_station_alignment" / "stack.py").exists()
    assert command_scan(python_files(str(copy))) == ([], ALLOWED_PUBLISHERS)
    (copy / "scripts" / "nudge.py").write_text(
        "from geometry_msgs.msg import Twist\n"
        "def main(node):\n    node.create_publisher(Twist, '/cmd_vel', 10)\n", encoding="utf-8")
    findings, _ = command_scan(python_files(str(copy)))
    assert ("nudge.py", "velocity message type") in findings
    assert ("nudge.py", "publisher on a cmd_vel topic (Twist)") in findings
    (copy / "scripts" / "nudge.py").unlink()
    (copy / "openamrobot_station_alignment" / "relay.py").write_text(
        "def build(node, Float64MultiArray):\n    node.create_publisher(Float64MultiArray, 'wheel_command', 1)\n",
        encoding="utf-8")
    findings, published = command_scan(python_files(str(copy)))
    assert findings == [] and published != ALLOWED_PUBLISHERS


def test_the_alignment_node_uses_nav2_actions_only():
    t = tree(os.path.join(PKG, "openamrobot_station_alignment", "station_alignment_node.py"))
    clients = sorted(ast.unparse(c.args[1]) for c in ast.walk(t)
                     if isinstance(c, ast.Call) and ast.unparse(c.func) in ("ActionClient", "ActionServer"))
    # Nav2's navigate_to_pose only (the #164 approach, rotation-only goals and forward drives; and this node's own
    # server), through Nav2's smoother and collision monitor. No behavior_server action: platform-sw #90 removes
    # backup, drive_on_heading and assisted_teleop (D-19), and Spin stops on odometry, which misses part of a turn.
    assert clients == ["NavigateToPose", "NavigateToPose"]
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
    """NavigateToPose is sent for the nominal #164 approach, for rotation-only goals at the current map position and
    for forward drives on the current heading. A corrected station pose is never a map goal: it moved with every
    AMCL update. The position target stays in odom; each drive is planned from the current pose."""
    t = tree(os.path.join(PKG, "openamrobot_station_alignment", "station_alignment_node.py"))
    calls = [c for c in ast.walk(t) if isinstance(c, ast.Call) and getattr(c.func, "attr", "") == "_navigate"]
    assert len(calls) == 3
    rotate = next(f for f in ast.walk(t) if isinstance(f, ast.FunctionDef) and f.name == "_rotate")
    source = ast.unparse(rotate)
    assert "x, y, _ = self._map_from_base()" in source and "(x, y, geometry.wrap(target_yaw))" in source
    drive = ast.unparse(next(f for f in ast.walk(t) if isinstance(f, ast.FunctionDef) and f.name == "_drive"))
    assert "maneuver.forward_goal(self._map_from_base(), distance" in drive
    assert not [c for c in ast.walk(t) if isinstance(c, ast.Call) and getattr(c.func, "attr", "") == "_behavior"]


# nav2_graceful_controller (navigation2 1.3.14, 58e4019), the controller of #164's StationApproach, ported for the
# sign of the linear velocity only:
# - parameter_handler.cpp 107-112: initial_rotation and allow_backward both true at configuration -> allow_backward
#   is set to false ("setting allow backward to false"); 172-189: a later parameter change that would make both true
#   is rejected;
# - graceful_controller.cpp 224-230: reversing = allow_backward and the motion target lies behind (x < 0);
# - smooth_control_law.cpp 66-76: v = clamp(v_linear_max / (1 + beta |kappa|^lambda), v_linear_min, v_linear_max),
#   then v = -v when reversing; rotateToTarget (365-370) sends linear.x 0.
# A port of the source, not the controller running: the simulation legs of station-alignment-tests run the real one.
def graceful_effective(params):
    effective = dict(params)
    if effective["initial_rotation"] and effective["allow_backward"]:
        effective["allow_backward"] = False
    return effective


def graceful_set(effective, name, value):
    """Runtime parameter change: rejected when it would make initial_rotation and allow_backward both true."""
    other = {"initial_rotation": "allow_backward", "allow_backward": "initial_rotation"}[name]
    if value and effective[other]:
        return effective
    return {**effective, name: value}


def graceful_linear_x(effective, target_x, kappa=0.0):
    reversing = effective["allow_backward"] and target_x < 0.0
    v = effective["v_linear_max"] / (1.0 + effective["beta"] * abs(kappa) ** effective["lambda"])
    v = min(max(v, effective["v_linear_min"]), effective["v_linear_max"])
    return -v if reversing else v


def _station_approach():
    import yaml
    with open(os.path.join(NAV2, "config", "nav2_params.yaml"), encoding="utf-8") as f:
        return yaml.safe_load(f)["controller_server"]["ros__parameters"]["StationApproach"]


TARGETS = [(x / 10.0, k) for x in range(-5, 6) for k in (0.0, 1.0, 5.0, 20.0)]


def test_station_approach_controller_cannot_reverse():
    """Why the correction does not reverse through StationApproach: the effective configuration of
    nav2_graceful_controller (initial_rotation turns allow_backward off) gives no negative linear velocity for any
    motion target, behind the robot included, and a runtime change cannot turn reversing on (Kunal's review of
    ea8ed13, point 6: the YAML flags alone kept passing with initial_rotation false and allow_backward true)."""
    params = _station_approach()
    assert params["plugin"] == "nav2_graceful_controller::GracefulController"
    assert 0.0 <= params["v_linear_min"] <= params["v_linear_max"]
    effective = graceful_effective(params)
    assert effective["allow_backward"] is False
    assert min(graceful_linear_x(effective, x, k) for x, k in TARGETS) > 0.0
    for name in ("allow_backward", "initial_rotation"):
        changed = graceful_set(effective, name, True)
        assert min(graceful_linear_x(graceful_effective(changed), x, k) for x, k in TARGETS) > 0.0, name
    # not vacuous: with initial_rotation false and allow_backward true (open question 2 of #164) the same model
    # reverses towards a target behind the robot
    flipped = graceful_effective({**params, "initial_rotation": False, "allow_backward": True})
    assert graceful_linear_x(flipped, -0.2) < 0.0 and graceful_linear_x(flipped, 0.2) > 0.0
