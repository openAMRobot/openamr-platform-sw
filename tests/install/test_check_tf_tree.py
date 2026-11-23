"""tools/check_tf_tree.py: unit tests on synthetic trees, plus the current repository state."""
import math
import os
import subprocess
import sys

import pytest

from conftest import REPO

import check_tf_tree as t  # noqa: E402  (tools/ is on sys.path via conftest)

try:
    import launch  # noqa: F401
    import xacro  # noqa: F401
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False


def _tree(*edges):
    return [t.Edge(*e) for e in edges]


def test_valid_tree_has_no_structure_problem():
    edges = _tree(('odom', 'base_link'), ('base_link', 'lidar_link'), ('base_link', 'imu_link'))
    assert t.check_structure(edges) == []


def test_two_parents_detected():
    edges = _tree(('odom', 'base_link'), ('base_link', 'lidar_link'), ('base_footprint', 'lidar_link'))
    problems = t.check_structure(edges)
    assert any('lidar_link has two parents' in p for p in problems)


def test_cycle_detected_once():
    edges = _tree(('base_footprint', 'base_link'), ('base_link', 'base_footprint'),
                  ('base_link', 'lidar_link'))
    problems = t.check_structure(edges)
    assert len([p for p in problems if p.startswith('cycle')]) == 1


def test_two_roots_detected():
    edges = _tree(('a', 'b'), ('c', 'd'))
    assert any('one root' in p for p in t.check_structure(edges))


def test_empty_tree_is_a_problem():
    assert t.check_structure([])


def test_pose_within_one_millimetre_passes_and_two_fails():
    ref = {'s': (0.330, 0.0, 0.124, 0.0, 0.0, math.pi)}
    ok = _tree(('base_link', 's', (0.3305, 0, 0.124), (0, 0, math.pi)))
    assert t.check_poses(ok, ref) == []
    bad = _tree(('base_link', 's', (0.332, 0, 0.124), (0, 0, math.pi)))
    assert len(t.check_poses(bad, ref)) == 1


def test_pose_orientation_is_checked():
    ref = {'s': (0.0, 0.0, 0.0, 0.0, 0.0, -math.pi / 2)}
    bad = _tree(('base_link', 's', (0, 0, 0), (0, 0, 0)))
    problems = t.check_poses(bad, ref)
    assert problems and '1.571' in problems[0]


def test_pose_through_a_parent_chain_and_inverted_root():
    # base_link is a child of odom, the sensor hangs off base_link through a mount frame
    edges = _tree(('odom', 'base_link', (5, 5, 0)), ('base_link', 'mount', (0.1, 0, 0.1)),
                  ('mount', 's', (0.2, 0, 0.0)))
    pose = t.pose_in_base_link(edges, 's')
    assert pose[0][3] == pytest.approx(0.3) and pose[2][3] == pytest.approx(0.1)


def test_missing_frame_reported():
    assert 'not published' in t.check_poses([], {'s': (0, 0, 0, 0, 0, 0)})[0]


def test_base_height():
    good = _tree(('base_footprint', 'base_link', (0, 0, 0.098)))
    assert t.check_base_height(good) == []
    bad = _tree(('base_link', 'base_footprint'))
    assert t.check_base_height(bad)


def test_reference_values_are_p03_rev20_6():
    ref = t.P03_REFERENCE
    assert ref['imu_link'][:3] == (0.0544, 0.0, 0.1210)
    assert ref['lidar_link'][:3] == (0.330, 0.0, 0.124)
    assert ref['base_camera_link'][:3] == (0.387, 0.0, 0.046)
    assert ref['tof_front'][4] == pytest.approx(math.radians(10))
    assert len(ref) == 9


@pytest.mark.skipif(not HAVE_ROS, reason='needs ROS 2 Jazzy (launch, xacro) on the runner')
class TestRepository:
    """What the repository produces now. Known violations are listed, not hidden."""

    def test_urdf_tree_is_a_single_tree_rooted_at_base_footprint(self):
        edges = t.build('urdf')
        assert t.check_structure(edges) == []
        assert t.check_base_height(edges) == []
        roots = {e.parent for e in edges} - {e.child for e in edges}
        assert roots == {'base_footprint'}

    def test_launch_profile_is_a_single_tree_rooted_at_odom(self):
        for profile in ('real_launch', 'hday_legacy'):
            edges = t.build(profile)
            assert t.check_structure(edges) == []
            roots = {e.parent for e in edges} - {e.child for e in edges}
            assert roots == {'odom'}

    def test_known_violation_combined_profile_is_not_a_tree(self):
        # Known mismatch: URDF base_footprint -> base_link against the real
        # odom -> base_link and base_link -> base_footprint, and duplicate sensor frames.
        problems = t.check_structure(t.build('combined'))
        assert any('lidar_link has two parents' in p for p in problems)
        assert any('cycle' in p for p in problems)

    def test_known_violation_real_launch_lidar_and_imu_differ_from_p03(self):
        problems = t.check_poses(t.build('real_launch'))
        assert any(p.startswith('lidar_link: off by 56.2 mm') for p in problems)
        assert any(p.startswith('imu_link:') for p in problems)

    def test_urdf_sensor_poses_match_p03_except_the_known_ones(self):
        # Allowed to fail today: tof pitch (20 deg on main, 10 deg in platform-sw #93) and the
        # P-03 text values 0.390 against CAD 0.399 / 0.3886. Anything else is a regression.
        problems = t.check_poses(t.build('urdf'))
        allowed = ('tof_front', 'tof_rear', 'us_rear_left', 'us_rear_right')
        unexpected = [p for p in problems if not p.startswith(allowed)]
        assert unexpected == []

    def test_cli_exit_codes(self):
        script = os.path.join(REPO, 'tools', 'check_tf_tree.py')
        ok = subprocess.run([sys.executable, script, 'hday_legacy'], capture_output=True, text=True)
        assert ok.returncode == 0, ok.stdout + ok.stderr
        bad = subprocess.run([sys.executable, script, 'combined'], capture_output=True, text=True)
        assert bad.returncode == 1
