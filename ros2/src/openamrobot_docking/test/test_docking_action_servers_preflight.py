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
    # name, stub script body, expected exit code, why
    ("queryable, no servers", 'echo "Action servers: 0"; exit 0', 0,
     "a healthy graph with no servers is the only case that may proceed"),
    ("query fails", "exit 1", 20,
     "a failing query is UNKNOWN, not zero"),
    ("prose without a count", 'echo "unknown action"; exit 0', 20,
     "output that cannot be parsed is UNKNOWN"),
    ("foreign stack present", 'echo "Action servers: 2"; exit 0', 20,
     "an existing stack must be refused, not joined"),
    ("empty output", "exit 0", 20,
     "silence is not a count"),
]


def _run_preflight(tmp_path: Path, stub_body: str) -> subprocess.CompletedProcess:
    bindir = tmp_path / "bin"
    bindir.mkdir()
    stub = bindir / "ros2"
    stub.write_text(f"#!/bin/sh\n{stub_body}\n")
    stub.chmod(0o755)
    env = dict(os.environ)
    env["PATH"] = f"{bindir}:{env['PATH']}"
    return subprocess.run(
        ["bash", str(SCRIPT), "--preflight-only", "/dock_robot", "/undock_robot"],
        capture_output=True, text=True, env=env, timeout=120,
    )


@pytest.mark.skipif(shutil.which("bash") is None, reason="bash required")
@pytest.mark.parametrize("name,stub_body,expected,why", CASES, ids=[c[0] for c in CASES])
def test_preflight_fails_closed(tmp_path, name, stub_body, expected, why):
    result = _run_preflight(tmp_path, stub_body)
    assert result.returncode == expected, (
        f"{name}: expected exit {expected} ({why}), got {result.returncode}\n"
        f"stdout: {result.stdout}\nstderr: {result.stderr}"
    )


@pytest.mark.skipif(shutil.which("bash") is None, reason="bash required")
def test_preflight_self_test_passes():
    """The script's own four cases must agree with the pytest table above."""
    result = subprocess.run(
        ["bash", str(SCRIPT), "--self-test-preflight"],
        capture_output=True, text=True, timeout=120,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert result.stdout.count("ok   ") == 4, result.stdout
