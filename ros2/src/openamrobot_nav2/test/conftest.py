"""
Pytest setup for the openamrobot_nav2 tests.

The producer tests need openamr_nav_msgs from openamrobot-interfaces. On a
developer machine without it they skip. CI sets OPENAMR_REQUIRE_INTERFACES=1,
and then a missing package stops the run with an error instead.

The real-mux full-chain tests (test_twist_mux_startup_and_restart.py,
test_docking_full_chain_through_real_mux.py, test_sim_clock_readiness_full_
chain.py, test_stop_contract_full_chain.py, test_gate_crash_full_chain_cmd_
vel_bound.py) need the real twist_mux, nav2_velocity_smoother, nav2_
collision_monitor and tf2_ros packages and the ros2 CLI on PATH; each file
skips itself cleanly without them, which is correct on a developer machine
that genuinely does not have them. In CI they are explicitly apt-installed
(.github/workflows/nav-status-tests.yml), so a skip there would silently
hide a broken install step behind a passing job instead of failing it -
platform-sw#90, Alex's review. OPENAMR_REQUIRE_TWIST_MUX=1 (set by that same
workflow) turns a missing package into a hard error here, the same pattern
OPENAMR_REQUIRE_INTERFACES already uses above. nav2_collision_monitor and
tf2_ros were added to this required list later (panthera-momagdii's review
on d29dfc0): test_gate_crash_full_chain_cmd_vel_bound.py needs both and was
skipping silently behind a green job if either were missing, same failure
mode this variable exists to prevent for twist_mux/nav2_velocity_smoother.
"""

import importlib.util
import os
import shutil
import subprocess

if os.environ.get('OPENAMR_REQUIRE_INTERFACES') and \
        importlib.util.find_spec('openamr_nav_msgs') is None:
    raise RuntimeError(
        'openamr_nav_msgs is not installed but OPENAMR_REQUIRE_INTERFACES is set')


def _ros2_pkg_available(pkg_name: str) -> bool:
    if shutil.which('ros2') is None:
        return False
    try:
        result = subprocess.run(
            ['ros2', 'pkg', 'prefix', pkg_name],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10.0)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return result.returncode == 0


if os.environ.get('OPENAMR_REQUIRE_TWIST_MUX'):
    _missing = [
        pkg for pkg in (
            'twist_mux', 'nav2_velocity_smoother', 'nav2_collision_monitor', 'tf2_ros')
        if not _ros2_pkg_available(pkg)]
    if _missing:
        raise RuntimeError(
            f'{_missing} not installed/discoverable but OPENAMR_REQUIRE_TWIST_MUX is set - '
            f'the real-mux full-chain tests would otherwise silently skip instead of failing')
