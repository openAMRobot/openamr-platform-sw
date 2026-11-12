"""
Pytest setup for the openamrobot_nav2 tests.

The producer tests need openamr_nav_msgs from openamrobot-interfaces. On a
developer machine without it they skip. CI sets OPENAMR_REQUIRE_INTERFACES=1,
and then a missing package stops the run with an error instead.
"""

import importlib.util
import os

if os.environ.get('OPENAMR_REQUIRE_INTERFACES') and \
        importlib.util.find_spec('openamr_nav_msgs') is None:
    raise RuntimeError(
        'openamr_nav_msgs is not installed but OPENAMR_REQUIRE_INTERFACES is set')
