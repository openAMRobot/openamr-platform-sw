"""Regression: the pre-start check must fail closed.

Review finding on #125 (11 Nov): the preflight turned a failed `ros2 action info` query into an
empty string, `${pre:-0}` read that as zero, and the harness started its launch anyway. A graph
that cannot be queried is not evidence that no docking server is running.

Each case supplies a stub `ros2` on PATH, so the decision is tested without ROS.
"""

import os
import shutil
import subprocess
from pathlib import Path

import pytest

SCRIPT = Path(__file__).parent / "docking_action_servers.sh"

CASES = [
    # name, stub script body, expected exit code, why, force_fallback
    ("queryable, no servers", 'echo "Action servers: 0"; exit 0', 0,
     "a healthy graph with no servers is the only case that may proceed", False),
    ("query fails", "exit 1", 20,
     "a failing query is UNKNOWN, not zero", False),
    ("prose without a count", 'echo "unknown action"; exit 0', 20,
     "output that cannot be parsed is UNKNOWN", False),
    ("foreign stack present", 'echo "Action servers: 2"; exit 0', 20,
     "an existing stack must be refused, not joined", False),
    ("empty output", "exit 0", 20,
     "silence is not a count", False),
    # HARNESS_FORCE_FALLBACK=1 makes this case skip the runner's coreutils `timeout` and exercise
    # the hand-rolled deadline. Without it the host `timeout` answers in 2 s and the fallback path
    # is never run, so a broken fallback would still pass (review P2, 12 Nov).
    ("hung query", "sleep 600", 20,
     "a query that never answers must be killed by the fallback deadline, not waited on", True),
]


def _run_preflight(tmp_path: Path, stub_body: str,
                   force_fallback: bool = False) -> subprocess.CompletedProcess:
    bindir = tmp_path / "bin"
    bindir.mkdir()
    stub = bindir / "ros2"
    stub.write_text(f"#!/bin/sh\n{stub_body}\n")
    stub.chmod(0o755)
    env = dict(os.environ)
    env["PATH"] = f"{bindir}:{env['PATH']}"
    # Without coreutils `timeout` the fallback deadline is the only bound; keep it short so the
    # hung-query case is a test rather than a hang.
    env["QUERY_TIMEOUT_S"] = "2"
    if force_fallback:
        # Force the hand-rolled deadline even though the host may have coreutils `timeout`, so the
        # fallback path is the code under test (review P2, 12 Nov).
        env["HARNESS_FORCE_FALLBACK"] = "1"
    return subprocess.run(
        ["bash", str(SCRIPT), "--preflight-only", "/dock_robot", "/undock_robot"],
        capture_output=True, text=True, env=env, timeout=120,
    )


@pytest.mark.skipif(shutil.which("bash") is None, reason="bash required")
@pytest.mark.parametrize("name,stub_body,expected,why,force_fallback", CASES,
                         ids=[c[0] for c in CASES])
def test_preflight_fails_closed(tmp_path, name, stub_body, expected, why, force_fallback):
    import time
    start = time.monotonic()
    result = _run_preflight(tmp_path, stub_body, force_fallback)
    elapsed = time.monotonic() - start
    assert result.returncode == expected, (
        f"{name}: expected exit {expected} ({why}), got {result.returncode}\n"
        f"stdout: {result.stdout}\nstderr: {result.stderr}"
    )
    if force_fallback:
        # The forced fallback deadline is QUERY_TIMEOUT_S=2; the process must end within a few
        # seconds, not wait out the stub's `sleep 600`. A hung fallback is the macOS defect here.
        assert elapsed < 20, f"{name}: fallback deadline did not fire (took {elapsed:.1f}s)"


@pytest.mark.skipif(shutil.which("bash") is None, reason="bash required")
def test_preflight_self_test_passes():
    """The script's own four cases must agree with the pytest table above."""
    result = subprocess.run(
        ["bash", str(SCRIPT), "--self-test-preflight"],
        capture_output=True, text=True, timeout=120,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert result.stdout.count("ok   ") == 5, result.stdout


@pytest.mark.skipif(shutil.which("bash") is None, reason="bash required")
def test_a_failed_launch_is_an_error_not_a_mismatch(tmp_path):
    """A launch that dies inside its supervisor must exit 20, never 10.

    Run on the hosted runner (12 Nov) where `openamrobot_nav2` was missing: the launch logged
    `Caught exception in launch ... not found, searching`, the supervisor stayed alive, and the
    harness reported a count mismatch. The negative control accepts exactly that code, so a crashed
    launch would have been counted as the expected negative result.
    """
    bindir = tmp_path / "bin"
    bindir.mkdir()
    stub = bindir / "ros2"
    stub.write_text(
        "#!/bin/sh\n"
        'case "$1" in\n'
        '  launch) echo "[ERROR] [launch]: Caught exception in launch: package not found, searching: [x]";'
        " sleep 60 ;;\n"
        '  action) echo "Action servers: 0" ;;\n'
        '  node) echo "/detected_dock_pose_publisher" ;;\n'
        "  *) exit 0 ;;\n"
        "esac\n"
    )
    stub.chmod(0o755)
    env = dict(os.environ)
    env["PATH"] = f"{bindir}:{env['PATH']}"
    env["HARNESS_TIMEOUT_S"] = "25"
    result = subprocess.run(
        ["bash", str(SCRIPT), "openamrobot_docking", "openamrobot_docking.launch.py", "1",
         "--control-node", "detected_dock_pose_publisher", "--", "use_sim_time:=false"],
        capture_output=True, text=True, env=env, timeout=120,
    )
    assert result.returncode == 20, (
        f"a failed launch must be EXIT_ERROR (20), got {result.returncode}\n"
        f"stdout: {result.stdout}\nstderr: {result.stderr}"
    )
    assert "launch itself failed" in result.stdout
