#!/usr/bin/env python3
"""Regression: every launch profile that enables docking must start the docking server,
and the server must not publish on the final /cmd_vel.

Two things are asserted, for BOTH launch profiles:

1. platform-sw#123/D-13 — the profile starts the `opennav_docking` action server, so
   /dock_robot and /undock_robot actually exist. D-13 requires every ENABLED motion
   producer to exist; a profile that enables docking but starts no server leaves the
   gate with a producer it can never get idle evidence from.
2. platform-sw#125 review (P1) — the server is launched with
   remappings=[('cmd_vel', 'cmd_vel_dock')]. DockingServer creates its publisher on the
   relative topic "cmd_vel" (nav2_docking/opennav_docking/src/docking_server.cpp:64), so
   at the root namespace it would publish on the final /cmd_vel, alongside
   collision_monitor and bypassing it. twist_mux takes docking at priority 20 on
   cmd_vel_dock.

Asserted on the parsed launch source rather than through launch_ros, because Node keeps its
remap rules private until execution and this check should run without a ROS graph.

Fails if either profile stops starting the server, if the remapping is removed, or if it is
moved to another node.
"""
import ast
import os

LAUNCH_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "launch"
)

# Every profile that enables docking. Sim: composed into bringup.launch.py. Real: composed
# into the real bring-up. Both must start the producer they enable.
PROFILES = ["openamrobot_docking.launch.py", "docking_real.launch.py"]

DOCKING_PACKAGE = "opennav_docking"
DOCKING_EXECUTABLE = "opennav_docking"
EXPECTED_REMAP = ("cmd_vel", "cmd_vel_dock")


def _const_str(node):
    """Literal value of a keyword value, or None when it is a substitution."""
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    return None


def _nodes_calling(path, package, executable):
    """Find Node(...) calls for a given package/executable pair in one launch file."""
    tree = ast.parse(open(path, encoding="utf-8").read(), filename=path)
    found = []
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call):
            continue
        kwargs = {kw.arg: kw.value for kw in call.keywords if kw.arg}
        pkg = _const_str(kwargs["package"]) if "package" in kwargs else None
        exe = _const_str(kwargs["executable"]) if "executable" in kwargs else None
        if pkg == package and exe == executable:
            found.append(kwargs)
    return found


def _check_profile(name):
    path = os.path.join(LAUNCH_DIR, name)
    assert os.path.exists(path), f"launch profile missing: {path}"

    nodes = _nodes_calling(path, DOCKING_PACKAGE, DOCKING_EXECUTABLE)
    assert len(nodes) == 1, (
        f"{name}: expected exactly one {DOCKING_PACKAGE}/{DOCKING_EXECUTABLE} node, "
        f"found {len(nodes)}. D-13: an enabled motion producer must exist."
    )

    kwargs = nodes[0]
    assert "remappings" in kwargs, (
        f"{name}: the docking server has no remappings. DockingServer publishes on the "
        "relative topic 'cmd_vel', which at the root namespace is the final /cmd_vel — "
        "that bypasses collision_monitor (platform-sw#125 review, P1)."
    )

    rules = []
    if isinstance(kwargs["remappings"], ast.List):
        for element in kwargs["remappings"].elts:
            if isinstance(element, ast.Tuple) and len(element.elts) == 2:
                rules.append((_const_str(element.elts[0]), _const_str(element.elts[1])))

    assert EXPECTED_REMAP in rules, (
        f"{name}: expected remapping {EXPECTED_REMAP} on the docking server, found {rules}. "
        "Without it docking/undocking velocity commands reach /cmd_vel directly and "
        "twist_mux's cmd_vel_dock input (priority 20) stays unused."
    )


def test_sim_profile_starts_the_docking_server():
    _check_profile(PROFILES[0])


def test_real_profile_starts_the_docking_server():
    _check_profile(PROFILES[1])


def _declared_default(path, argument):
    """Literal default_value of a DeclareLaunchArgument in one launch file, or None."""
    tree = ast.parse(open(path, encoding="utf-8").read(), filename=path)
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if getattr(node.func, "id", None) != "DeclareLaunchArgument" or not node.args:
            continue
        first = node.args[0]
        if not (isinstance(first, ast.Constant) and first.value == argument):
            continue
        kwargs = {kw.arg: kw.value for kw in node.keywords if kw.arg}
        return _const_str(kwargs["default_value"]) if "default_value" in kwargs else None
    return None


def test_real_profile_defaults_the_server_off():
    """D-19 (Alex, 11 Nov): the standalone real launch must not start the server by default.

    The server exposes a direct /undock_robot that #90's dock_trigger constant cannot gate, and the
    real profile is the one that runs on hardware. The node stays declared (so an explicit
    launch_docking_server:=true still works once the three #124 conditions are met) but the default
    must be false.
    """
    path = os.path.join(LAUNCH_DIR, PROFILES[1])
    default = _declared_default(path, "launch_docking_server")
    assert default == "false", (
        f"docking_real.launch.py declares launch_docking_server default={default!r}; D-19 requires "
        "'false' on the real profile until the #124 conditions are met")


def test_sim_profile_keeps_the_server_on():
    """Positive control for the test above: simulation keeps docking on."""
    path = os.path.join(LAUNCH_DIR, PROFILES[0])
    default = _declared_default(path, "launch_docking_server")
    assert default == "true", (
        f"openamrobot_docking.launch.py declares launch_docking_server default={default!r}; "
        "simulation is where the docking stack is exercised")


BRINGUP_LAUNCH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    'openamrobot_bringup', 'launch', 'bringup.launch.py')


def test_real_profile_does_not_default_docking_on():
    """D-19 (Alex, 11 Nov): the real launch keeps docking off by default.

    Guarded at source level so a revert to a hard-coded default cannot pass quietly: the argument
    must be declared empty and resolved from `sim`, never declared 'true'.
    """
    source = open(BRINGUP_LAUNCH).read()
    tree = ast.parse(source)

    declared = None
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = getattr(node.func, 'id', None)
        if func != 'DeclareLaunchArgument' or not node.args:
            continue
        first = node.args[0]
        if not (isinstance(first, ast.Constant) and first.value == 'use_docking'):
            continue
        keywords = {kw.arg: kw.value for kw in node.keywords if kw.arg}
        declared = keywords.get('default_value')
    assert declared is not None, 'bringup.launch.py no longer declares use_docking'
    assert isinstance(declared, ast.Constant) and declared.value == '', (
        'use_docking must be declared with an empty default and resolved from sim; '
        'a hard-coded default re-enables docking on the real robot against D-19')

    assert "use_docking = 'true' if is_sim else 'false'" in source, (
        'the sim/real resolution of use_docking is missing from bringup.launch.py')
