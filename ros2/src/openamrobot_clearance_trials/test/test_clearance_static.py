# SPDX-License-Identifier: MIT
"""Static checks of the clearance trials: geometry, scene arithmetic, overrides. No ROS, no simulator."""
import math
import os
import re

import pytest
import yaml

from openamrobot_clearance_trials import geometry, overrides, scene

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.dirname(os.path.dirname(HERE))
NAV2_PARAMS = os.path.join(SRC, "openamrobot_nav2", "config", "nav2_params.yaml")
XACRO = os.path.join(SRC, "openamrobot_description", "urdf", "robo_urdf.urdf.xacro")


def test_polygon_distance_of_two_boxes():
    a = geometry.box(1.0, 1.0, (0.0, 0.0, 0.0))
    assert geometry.polygon_distance(a, geometry.box(1.0, 1.0, (1.5, 0.0, 0.0))) == pytest.approx(0.5)
    assert geometry.polygon_distance(a, geometry.box(1.0, 1.0, (0.9, 0.0, 0.0))) == 0.0
    assert geometry.polygon_distance(a, geometry.box(1.0, 1.0, (2.0, 2.0, 0.0))) == pytest.approx(math.sqrt(2.0))
    turned = geometry.box(1.0, 1.0, (2.0, 0.0, math.pi / 4.0))
    assert geometry.polygon_distance(a, turned) == pytest.approx(2.0 - 0.5 - math.sqrt(0.5))


def test_circle_distance():
    a = geometry.box(1.0, 1.0, (0.0, 0.0, 0.0))
    assert geometry.circle_distance(a, (1.0, 0.0), 0.2) == pytest.approx(0.3)
    assert geometry.circle_distance(a, (0.6, 0.0), 0.2) == 0.0
    assert geometry.circle_distance(a, (0.1, 0.1), 0.01) == 0.0


def test_chassis_is_the_collision_box_of_the_description():
    with open(XACRO, encoding="utf-8") as f:
        text = f.read()
    base = text[text.index('name="base_link"'):]
    collision = base[base.index("<collision>"):base.index("</collision>")]
    sx, sy, _ = (float(v) for v in re.search(r'<box size="([^"]+)"', collision).group(1).split())
    ox, oy, _ = (float(v) for v in re.search(r'<origin xyz="([^"]+)"', collision).group(1).split())
    xs = sorted({p[0] for p in geometry.CHASSIS})
    ys = sorted({p[1] for p in geometry.CHASSIS})
    assert xs == pytest.approx([ox - sx / 2.0, ox + sx / 2.0]) and ys == pytest.approx([oy - sy / 2.0, oy + sy / 2.0])
    assert scene.ROBOT_WIDTH == pytest.approx(sy)


def test_footprint_is_the_one_of_nav2_params():
    from openamrobot_clearance_trials import trials_constants
    with open(NAV2_PARAMS, encoding="utf-8") as f:
        params = yaml.safe_load(f)
    for costmap in ("local_costmap", "global_costmap"):
        footprint = yaml.safe_load(params[costmap][costmap]["ros__parameters"]["footprint"])
        assert [tuple(p) for p in footprint] == [tuple(p) for p in trials_constants.FOOTPRINT]


def test_gap_scenarios_leave_the_stated_gap():
    specs = scene.scenarios(1.5)
    left_wall = scene.shapes(scene.mapped(1.5))["aisle_left"]
    for name, gap in (("gap100", 1.00), ("gap080", 0.80), ("gap066", 0.66), ("gap050", 0.50)):
        cart = scene.shapes(specs[name]["obstacles"])["cart"]
        assert geometry.shape_distance(cart[1], left_wall) == pytest.approx(gap, abs=1e-9)
    right_wall = scene.shapes(scene.mapped(1.5))["aisle_right"]
    cart = scene.shapes(specs["gap100"]["obstacles"])["cart"]
    assert geometry.shape_distance(cart[1], right_wall) == pytest.approx(0.0, abs=1e-9)


def test_all_blocked_leaves_no_passage_wider_than_the_chassis():
    items = scene.scenarios(1.5)["all_blocked"]["obstacles"]
    spans = sorted((d[1][1] - d[0][1] / 2.0, d[1][1] + d[0][1] / 2.0) for _, _, d, _ in items)
    edges = [scene.ARENA_Y[0] + scene.WALL / 2.0] + [v for span in spans for v in span] + \
            [scene.ARENA_Y[1] - scene.WALL / 2.0]
    gaps = [edges[i + 1] - edges[i] for i in range(0, len(edges), 2)]
    assert max(gaps) < 0.30, gaps      # the aisle walls (0.10 m) stand in the two larger gaps


def test_overrides_change_only_the_named_value_and_refuse_a_wrong_path():
    with open(NAV2_PARAMS, encoding="utf-8") as f:
        tree = yaml.safe_load(f)
    path = "local_costmap/local_costmap/ros__parameters/inflation_layer/inflation_radius"
    changed = overrides.apply(tree, {path: 0.45})
    assert changed == [(path, 0.15, 0.45)]
    with open(NAV2_PARAMS, encoding="utf-8") as f:
        again = yaml.safe_load(f)
    again["local_costmap"]["local_costmap"]["ros__parameters"]["inflation_layer"]["inflation_radius"] = 0.45
    assert tree == again
    with pytest.raises(KeyError):
        overrides.apply(tree, {"local_costmap/local_costmap/ros__parameters/inflation_layr/inflation_radius": 0.5})


def test_runner_has_no_velocity_publisher():
    with open(os.path.join(os.path.dirname(HERE), "openamrobot_clearance_trials", "trials.py"), encoding="utf-8") as f:
        text = f.read()
    assert "create_publisher" not in text
