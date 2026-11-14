"""
The shipped navigate_to_pose/navigate_through_poses trees must never command BackUp.

D-19 audit, platform-sw#124, Alex's finding on #90.

nav2_bt_navigator's own shipped default BT automatically commands a blind
0.30 m reverse (BackUp) as a controller/planner-failure recovery action -
no external client, no docking involved - the same LiDAR rear-blind-spot
hazard D-19 disables dock_trigger's own undock reverse for. Spin and Wait
(the other two RecoveryActions) do not reverse the chassis and must stay,
so this also guards against someone accidentally gutting the whole
RecoveryActions fallback instead of just the one unsafe entry.

Statically parses the XML, no ROS graph needed. See
test_navigation_launch_bt_xml_resolves_to_no_backup.py for the live proof
that bt_navigator actually resolves to these exact files.
"""
import os
import xml.etree.ElementTree as ET

_BT_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'behavior_trees')

_FILES = (
    'navigate_to_pose_no_backup.xml',
    'navigate_through_poses_no_backup.xml',
)


def _tree(filename):
    return ET.parse(os.path.join(_BT_DIR, filename))


def test_both_files_exist():
    for filename in _FILES:
        path = os.path.join(_BT_DIR, filename)
        assert os.path.isfile(path), f'missing behavior tree file: {path}'


def test_neither_file_contains_a_backup_node():
    for filename in _FILES:
        tree = _tree(filename)
        backups = tree.getroot().findall('.//BackUp')
        assert backups == [], (
            f'{filename} still contains a <BackUp .../> node - D-19 (platform-sw#124, '
            "Alex's finding on #90): this commands a blind 0.30m reverse automatically "
            'on ordinary controller/planner failure recovery, with the same LiDAR '
            "rear-blind-spot hazard as dock_trigger's own undock reverse.")


def test_recovery_actions_still_has_spin_and_wait():
    # Sanity check: confirm this removed ONLY BackUp, not gutted the whole
    # RecoveryActions fallback (Spin/Wait don't reverse the chassis).
    for filename in _FILES:
        tree = _tree(filename)
        recovery_actions = tree.getroot().find(".//RoundRobin[@name='RecoveryActions']")
        assert recovery_actions is not None, (
            f'{filename} has no RoundRobin name="RecoveryActions" - '
            'did the tree structure change upstream?')
        assert recovery_actions.find('Spin') is not None, f'{filename} lost its Spin recovery'
        assert recovery_actions.find('Wait') is not None, f'{filename} lost its Wait recovery'


if __name__ == '__main__':
    import pytest
    raise SystemExit(pytest.main([__file__, '-v']))
