"""
safety_stop_gate_node.py's _CMD_VEL_INPUT_TIMEOUTS must match twist_mux.yaml.

Alex's 19 Nov P1 review on #90: _CMD_VEL_INPUT_TIMEOUTS is a hand-maintained
copy of config/twist_mux.yaml's own per-topic timeout values (that
dictionary's own comment already says so - "NOT re-derived here; keep these
in sync with that file by hand"), and nothing failed if an input's timeout
changed, or an input was renamed or added, in one file but not the other.

This loads the shipped config/twist_mux.yaml directly (no standalone copy -
same convention as test_twist_mux_startup_and_restart.py's own
_load_shipped_lock_timeout) and cross-checks it against the dictionary in
both directions: every topic _CMD_VEL_INPUT_TIMEOUTS names must exist in the
YAML with the same timeout, and every non-lock YAML topic must be named in
_CMD_VEL_INPUT_TIMEOUTS - so a renamed/added/changed entry in either file
fails here instead of silently drifting.
"""
from pathlib import Path

import pytest

yaml = pytest.importorskip('yaml')

from openamrobot_nav2.safety_stop_gate_node import _CMD_VEL_INPUT_TIMEOUTS  # noqa: E402,I100

_SHIPPED_PARAMS_PATH = Path(__file__).resolve().parents[1] / 'config' / 'twist_mux.yaml'


def _load_shipped_topics():
    with open(_SHIPPED_PARAMS_PATH) as f:
        params = yaml.safe_load(f)
    return params['twist_mux']['ros__parameters']['topics']


def test_every_tracked_input_exists_in_the_yaml_with_the_same_timeout():
    topics = _load_shipped_topics()
    by_topic_name = {entry['topic']: entry for entry in topics.values()}
    for topic_name, timeout in _CMD_VEL_INPUT_TIMEOUTS.items():
        assert topic_name in by_topic_name, (
            f'_CMD_VEL_INPUT_TIMEOUTS names {topic_name!r}, which is not any '
            'topics.*.topic value in config/twist_mux.yaml - the gate is '
            'reading a timeout for an input twist_mux no longer has')
        assert by_topic_name[topic_name]['timeout'] == timeout, (
            f'_CMD_VEL_INPUT_TIMEOUTS[{topic_name!r}] is {timeout!r}, but '
            f"config/twist_mux.yaml's own timeout for that topic is "
            f"{by_topic_name[topic_name]['timeout']!r} - the hand-copied "
            'value in safety_stop_gate_node.py has drifted from the shipped '
            'mux config')


def test_every_non_lock_yaml_input_is_tracked():
    # software_stop (priority 255) is the gate's own zero-Twist output, not
    # a release condition (c) input the gate reads back from outside itself -
    # deliberately excluded from _CMD_VEL_INPUT_TIMEOUTS, same as today.
    topics = _load_shipped_topics()
    tracked = set(_CMD_VEL_INPUT_TIMEOUTS)
    shipped = {entry['topic'] for entry in topics.values()} - {'cmd_vel_stop'}
    assert shipped == tracked, (
        f'config/twist_mux.yaml names {shipped - tracked or "(nothing missing)"} '
        'that _CMD_VEL_INPUT_TIMEOUTS does not track, or _CMD_VEL_INPUT_TIMEOUTS '
        f'tracks {tracked - shipped or "(nothing extra)"} that is no longer in '
        'the shipped config - a renamed or added mux input must be added to '
        'safety_stop_gate_node.py by hand, and this is what catches a case '
        'where that was missed')


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__, '-v']))
