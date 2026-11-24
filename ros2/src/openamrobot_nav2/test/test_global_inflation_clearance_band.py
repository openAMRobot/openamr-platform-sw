# SPDX-License-Identifier: MIT
"""The global plan needs a cost band beyond the footprint's inscribed radius (#188).

With an inflation radius of 0.35 m against an inscribed radius of 0.30 m the band was
0.05 m, and the robot passed a cart at 0.019 to 0.037 m in simulation
(openamrobot_clearance_trials). The clearance target of the trials is 0.10 m; the band
has to be wider than that or the planner cannot prefer it. This is a configuration
check, not evidence of clearance: the measurement is the clearance-trials workflow.
"""
import math
import os

import yaml

PARAMS = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "nav2_params.yaml")
CLEARANCE_TARGET = 0.10


def _inscribed_radius(points):
    best = math.inf
    n = len(points)
    for i in range(n):
        (ax, ay), (bx, by) = points[i], points[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        t = max(0.0, min(1.0, -(ax * dx + ay * dy) / (dx * dx + dy * dy)))
        best = min(best, math.hypot(ax + t * dx, ay + t * dy))
    return best


def _global():
    with open(PARAMS, encoding="utf-8") as f:
        return yaml.safe_load(f)["global_costmap"]["global_costmap"]["ros__parameters"]


def test_global_inflation_band_is_wider_than_the_clearance_target():
    params = _global()
    inscribed = _inscribed_radius(yaml.safe_load(params["footprint"]))
    band = params["inflation_layer"]["inflation_radius"] - inscribed
    assert band >= 2.0 * CLEARANCE_TARGET, (
        f"cost band {band:.3f} m beyond the inscribed radius {inscribed:.3f} m")


def test_cost_at_the_clearance_target_is_a_real_cost():
    """At 0.10 m beyond the inscribed radius the cost must still be well above free."""
    layer = _global()["inflation_layer"]
    cost = 252.0 * math.exp(-layer["cost_scaling_factor"] * CLEARANCE_TARGET)
    assert cost >= 150.0, cost
