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
    'navigate_to_pose.xml',
    'navigate_through_poses.xml',
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


def test_neither_file_contains_a_clear_entire_costmap_node():
    # Alex's 19/20 Nov decision: these files now carry #91/#92's bounded-
    # clear fix (adopted here instead of #90's own simpler BackUp-only
    # removal, which still wiped the WHOLE costmap on every recovery -
    # erasing real, currently-correct obstacle data anywhere else on the
    # map, not just near whatever triggered the failure).
    for filename in _FILES:
        tree = _tree(filename)
        unbounded = tree.getroot().findall('.//ClearEntireCostmap')
        assert unbounded == [], (
            f'{filename} still contains a <ClearEntireCostmap .../> node - '
            'this must be ClearCostmapAroundRobot (bounded) everywhere, '
            "per #91/#92's fix adopted into this file")


def test_every_clear_is_bounded_with_the_expected_reset_distance():
    expected = {
        'global_costmap/clear_around_global_costmap': '3.0',
        'local_costmap/clear_around_local_costmap': '1.5',
    }
    for filename in _FILES:
        tree = _tree(filename)
        clears = tree.getroot().findall('.//ClearCostmapAroundRobot')
        assert len(clears) == 4, (
            f'{filename} has {len(clears)} ClearCostmapAroundRobot nodes, expected 4 '
            '(global/local context clear + global/local subtree clear)')
        for clear in clears:
            service_name = clear.get('service_name')
            assert service_name in expected, (
                f'{filename}: unexpected service_name {service_name!r} on a '
                'ClearCostmapAroundRobot node')
            assert clear.get('reset_distance') == expected[service_name], (
                f'{filename}: {service_name} reset_distance is '
                f"{clear.get('reset_distance')!r}, expected {expected[service_name]!r}")


def test_each_inline_clear_is_preceded_by_a_named_wait():
    # RecoveryTracker matches the bare node name "Wait" to count a recovery
    # attempt (same name the outer round-robin's genuine Wait(5.0) uses) -
    # these inline waits must stay named so they are never mistaken for one.
    for filename in _FILES:
        tree = _tree(filename)
        waits = {w.get('name') for w in tree.getroot().findall('.//Wait') if w.get('name')}
        assert {'WaitBeforeClear-GlobalContext', 'WaitBeforeClear-LocalContext'} <= waits, (
            f'{filename} is missing one of the named inline waits - an unnamed inline '
            'Wait would be miscounted as a RecoveryTracker recovery action')


def test_follow_path_names_the_general_goal_checker():
    # Platform-sw#164 adds a second goal checker (station_goal_checker);
    # controller_server refuses an empty goal_checker_id once more than one
    # is configured. These are the default trees (nav2_params.yaml's
    # default_nav_to_pose_bt_xml/default_nav_through_poses_bt_xml), so they
    # must name the general one explicitly rather than leave it empty.
    for filename in _FILES:
        tree = _tree(filename)
        follow_path = tree.getroot().find('.//FollowPath')
        assert follow_path is not None, f'{filename} has no FollowPath node'
        assert follow_path.get('goal_checker_id') == 'general_goal_checker', (
            f'{filename}: FollowPath goal_checker_id is '
            f"{follow_path.get('goal_checker_id')!r}, expected 'general_goal_checker' - "
            "required once #164's station_goal_checker is also configured")


if __name__ == '__main__':
    import pytest
    raise SystemExit(pytest.main([__file__, '-v']))
