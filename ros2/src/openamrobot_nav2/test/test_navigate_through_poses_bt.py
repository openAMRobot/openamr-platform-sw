#!/usr/bin/env python3
"""
Regression test for #35/N4: navigate_through_poses.xml recovery policy.

Checks that planner/controller-failure recovery uses bounded costmap
clearing, not Nav2's stock blanket ClearEntireCostmap, and waits before
each inline clear instead of folding straight into it. Statically parses
the BT XML, same style as test_navigation_launch_cmd_vel_routing.py's AST
approach for the launch file - no ROS graph or BT.CPP parser needed, just
the tree structure.
"""
import os
import xml.etree.ElementTree as ET

BT_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'behavior_trees', 'navigate_through_poses.xml')

# the two inline (per-action) clears, each expected right after its own Wait
INLINE_CLEARS = {
    'ClearGlobalCostmap-Context': '3.0',
    'ClearLocalCostmap-Context': '1.5',
}
# the two outer round-robin clears - bounded too, but no Wait requirement
# (the round-robin already has its own Wait(5.0) as a separate fallback step)
SUBTREE_CLEARS = {
    'ClearLocalCostmap-Subtree': '1.5',
    'ClearGlobalCostmap-Subtree': '3.0',
}


def _parse():
    return ET.parse(BT_FILE)


def test_file_exists_and_is_well_formed():
    assert os.path.isfile(BT_FILE), f'{BT_FILE} does not exist.'
    _parse()  # raises ET.ParseError on malformed XML


def test_no_clear_entire_costmap_anywhere():
    """
    Enforce the plan's section 27 rule against universal first responses.

    ClearEntireCostmap wipes real, currently-correct obstacle data
    anywhere else on the map - it must not appear at all once bounded
    clearing has replaced it.
    """
    root = _parse().getroot()
    found = [el.get('name') for el in root.iter('ClearEntireCostmap')]
    assert found == [], (
        f'Found ClearEntireCostmap node(s) {found} - these must be '
        f'ClearCostmapAroundRobot (bounded) instead (#35/N4).')


def test_all_four_clears_are_bounded_with_expected_reset_distance():
    root = _parse().getroot()
    by_name = {el.get('name'): el for el in root.iter('ClearCostmapAroundRobot')}
    expected = dict(INLINE_CLEARS)
    expected.update(SUBTREE_CLEARS)
    assert set(by_name) == set(expected), (
        f'Expected exactly the bounded clears {sorted(expected)}, '
        f'found {sorted(by_name)}.')
    for name, reset_distance in expected.items():
        actual = by_name[name].get('reset_distance')
        assert actual == reset_distance, (
            f'{name} has reset_distance={actual!r}, expected {reset_distance!r}.')


def test_inline_clears_are_preceded_by_a_named_wait():
    """
    Require wait-for-transient-obstacle as its own, individually named step.

    #35/N4: each inline clear's immediately preceding sibling (within the
    same recovery <Sequence>) must be an explicit <Wait>, not the clear
    itself silently acting as the wait.

    That Wait must also be named, not left as bare <Wait> (BotshareAI's
    platform-sw#92 review): RecoveryTracker (status_trackers.py) matches
    the bare node name "Wait" to count a recovery attempt - the same name
    the outer round-robin's genuine recovery Wait(5.0) uses (see
    test_outer_round_robin_recovery_actions_are_unchanged_from_stock). An
    unnamed inline Wait would be indistinguishable from that and would
    count every ordinary inline clear toward the shared attempt_limit,
    reaching RECOVERY_LIMIT_REACHED around Nav2's 2nd outer retry while
    Nav2 itself is still legitimately recovering.

    checked_clears guards against this test silently passing vacuously if
    the loop below ever stopped actually finding both inline clears (the
    exact kind of positive control the earlier #90 review asked for).
    """
    root = _parse().getroot()
    checked_clears = set()
    for parent in root.iter():
        children = list(parent)
        for i, child in enumerate(children):
            if child.tag != 'ClearCostmapAroundRobot':
                continue
            name = child.get('name')
            if name not in INLINE_CLEARS:
                continue
            checked_clears.add(name)
            assert i > 0, f'{name} has no preceding sibling at all.'
            previous = children[i - 1]
            assert previous.tag == 'Wait', (
                f'{name} is preceded by <{previous.tag}>, not <Wait> - '
                f'the explicit wait-before-replan step is missing (#35/N4).')
            assert previous.get('wait_duration') == '2.0', (
                f'The Wait preceding {name} has wait_duration='
                f'{previous.get("wait_duration")!r}, expected 2.0.')
            assert previous.get('name'), (
                f'The Wait preceding {name} has no name attribute, so '
                f'RecoveryTracker would count it as a bare "Wait" recovery '
                f'action on every inline clear (platform-sw#92 review).')
            assert previous.get('name') != 'Wait', (
                f'The Wait preceding {name} is literally named "Wait", '
                f'which RecoveryTracker matches the same as an unnamed one.')
    assert checked_clears == set(INLINE_CLEARS), (
        f'Expected to check the Wait preceding both {sorted(INLINE_CLEARS)}, '
        f'actually checked {sorted(checked_clears)} - this test must not '
        f'pass vacuously by silently checking fewer than both.')


def test_outer_round_robin_recovery_actions_are_unchanged_from_stock():
    """
    Pin today's stock Spin/Wait(5.0)/BackUp values.

    Zone-gating them is explicitly out of scope here (blocked on #73/N2's
    zone data) - pinning these values makes a future change to them a
    deliberate, reviewed decision, not an accidental side effect of
    something else touching this file.
    """
    root = _parse().getroot()
    spin = root.find('.//Spin')
    assert spin is not None and spin.get('spin_dist') == '1.57'
    backup = root.find('.//BackUp')
    assert backup is not None and backup.get('backup_dist') == '0.30' \
        and backup.get('backup_speed') == '0.15'
    waits = [w.get('wait_duration') for w in root.iter('Wait')]
    assert '5.0' in waits, 'Outer round-robin Wait(5.0) is missing.'


if __name__ == '__main__':
    import pytest
    raise SystemExit(pytest.main([__file__, '-v']))
