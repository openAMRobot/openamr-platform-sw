"""Fast tests for deterministic docking geometry helpers."""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'dock_trigger.py'
SPEC = importlib.util.spec_from_file_location('dock_trigger_under_test', SCRIPT)
DOCK_TRIGGER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(DOCK_TRIGGER)


@pytest.mark.parametrize(('angle', 'expected'), [
    (0.0, 0.0),
    (math.pi, math.pi),
    (-math.pi, -math.pi),
    (3.0 * math.pi, math.pi),
    (-3.0 * math.pi, -math.pi),
    (10.0 * math.pi + 0.25, 0.25),
])
def test_normalize_angle(angle, expected):
    assert DOCK_TRIGGER.normalize_angle(angle) == pytest.approx(expected)


@pytest.mark.parametrize(('yaw', 'expected'), [
    (0.0, 0.0),
    (math.pi / 2.0, math.pi / 2.0),
    (-math.pi / 2.0, -math.pi / 2.0),
])
def test_quaternion_to_yaw(yaw, expected):
    quaternion = SimpleNamespace(
        x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
    assert DOCK_TRIGGER.quat_to_yaw(quaternion) == pytest.approx(expected)


def test_quaternion_rotates_tag_normal():
    assert DOCK_TRIGGER.quat_rotate_z(0.0, 0.0, 0.0, 1.0) == pytest.approx((0, 0, 1))
    half = math.sqrt(0.5)
    assert DOCK_TRIGGER.quat_rotate_z(0.0, half, 0.0, half) == pytest.approx((1, 0, 0))
