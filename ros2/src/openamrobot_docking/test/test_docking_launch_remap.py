#!/usr/bin/env python3
"""Regression: the docking server must not publish on the final /cmd_vel.

platform-sw#125 review (P1): nav2's DockingServer creates its velocity publisher on the RELATIVE
topic "cmd_vel" (nav2_docking/opennav_docking/src/docking_server.cpp:64). Launched at the root
namespace it therefore publishes on the final /cmd_vel, alongside collision_monitor and bypassing
it. twist_mux takes docking at priority 20 on `cmd_vel_dock`, so the launch must remap it there.

Asserted on the parsed launch source rather than through launch_ros, because Node keeps its
remap rules private until execution and this check should run without a ROS graph.

Fails if the remapping is removed, renamed, or moved to another node.
"""
import ast
import os

LAUNCH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "launch",
    "openamrobot_docking.launch.py",
)

DOCKING_PACKAGE = "opennav_docking"
DOCKING_EXECUTABLE = "opennav_docking"
EXPECTED_REMAP = ("cmd_vel", "cmd_vel_dock")


def _const_str(node):
    """Literal value of a keyword value, or None when it is a substitution."""
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    return None


def _nodes_calling(package, executable):
    """Find Node(...) calls for a given package/executable pair."""
    tree = ast.parse(open(LAUNCH, encoding="utf-8").read(), filename=LAUNCH)
    found = []
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call):
            continue
        kwargs = {kw.arg: kw.value for kw in call.keywords if kw.arg}
        pkg = _const_str(kwargs.get("package")) if "package" in kwargs else None
        exe = _const_str(kwargs.get("executable")) if "executable" in kwargs else None
        if pkg == package and exe == executable:
            found.append(kwargs)
    return found


def test_docking_server_is_launched():
    assert len(_nodes_calling(DOCKING_PACKAGE, DOCKING_EXECUTABLE)) == 1, (
        "expected exactly one opennav_docking node in the docking launch"
    )


def test_docking_server_remaps_cmd_vel_to_cmd_vel_dock():
    kwargs = _nodes_calling(DOCKING_PACKAGE, DOCKING_EXECUTABLE)[0]
    assert "remappings" in kwargs, (
        "the docking server has no remappings: DockingServer publishes on the relative topic "
        "'cmd_vel', which at the root namespace is the final /cmd_vel — that bypasses "
        "collision_monitor (platform-sw#125 review, P1)"
    )

    rules = []
    for element in kwargs["remappings"].elts if isinstance(kwargs["remappings"], ast.List) else []:
        if isinstance(element, ast.Tuple) and len(element.elts) == 2:
            rules.append((_const_str(element.elts[0]), _const_str(element.elts[1])))

    assert EXPECTED_REMAP in rules, (
        f"expected remapping {EXPECTED_REMAP} on the docking server, found {rules}. "
        "Without it docking/undocking velocity commands reach /cmd_vel directly and "
        "twist_mux's cmd_vel_dock input (priority 20) stays unused."
    )
