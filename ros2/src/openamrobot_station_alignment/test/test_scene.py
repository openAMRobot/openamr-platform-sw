# SPDX-License-Identifier: MIT
"""The station alignment world, map, marker model and configuration agree with each other (no ROS needed)."""
import itertools
import math
import os
import re
import xml.etree.ElementTree as ET

import pytest
import yaml

from openamrobot_station_alignment import evaluation, geometry, scene

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def read(relative, mode="r"):
    with open(os.path.join(PKG, relative), mode) as f:
        return f.read()


@pytest.mark.parametrize("relative", sorted(scene.outputs()))
def test_committed_world_and_map_are_generated_by_scene_py(relative):
    assert read(relative, "rb") == scene.outputs()[relative], "run python3 -m openamrobot_station_alignment.scene"


def test_station_is_the_baseline1_work_shelf_station():
    # openamr-upperbody-sw scene.py station("work_shelf") at d1c88be: front (0, -2.8), gap 0.30 + 0.415, facing -y.
    x, y, yaw = scene.station()
    assert (x, y) == pytest.approx((0.0, -2.085)) and yaw == pytest.approx(-math.pi / 2)


def test_detector_configuration_matches_the_scene():
    params = yaml.safe_load(read("config/station_alignment.yaml"))["station_marker_detector"]["ros__parameters"]
    assert params["tag_ids"] == list(scene.MARKER_IDS)
    assert dict(zip(params["tag_ids"], params["tag_offsets"])) == scene.MARKER_OFFSETS
    assert params["standoff"] == pytest.approx(scene.standoff())
    assert params["tag_frames"] == [f"station_marker_{i}" for i in scene.MARKER_IDS]
    tags = yaml.safe_load(read("config/tags_station_sim.yaml"))["/**"]["ros__parameters"]
    assert tags["tag"]["ids"] == params["tag_ids"] and tags["tag"]["frames"] == params["tag_frames"]
    assert tags["size"] == pytest.approx(scene.MARKER_TAG_SIZE) == pytest.approx(0.8 * scene.MARKER_PANEL)
    assert not set(tags["tag"]["ids"]) & {0, 1, 2}, "ids 0 to 2 are the charging dock's"


def test_alignment_acceptance_is_inside_the_reach_budget_and_not_below_the_goal_checker():
    params = yaml.safe_load(read("config/station_alignment.yaml"))["station_alignment"]["ros__parameters"]
    assert math.hypot(params["accept_along"], params["accept_lateral"]) < evaluation.REACH_XY
    assert params["accept_yaw"] < evaluation.REACH_YAW
    assert params["accept_along"] >= evaluation.CHECKER_XY and params["accept_yaw"] >= evaluation.CHECKER_YAW


def test_marker_model_places_the_tags_of_the_scene():
    root = ET.fromstring(read("models/station_marker/model.sdf"))
    visuals = {v.get("name"): v for v in root.iter("visual")}
    for tag, offset in scene.MARKER_OFFSETS.items():
        v = visuals[f"tag{tag}"]
        pose = [float(a) for a in v.find("pose").text.split()]
        assert pose[1] == pytest.approx(-offset), "model +y is minus the shelf axis"
        size = [float(a) for a in v.find("geometry/box/size").text.split()]
        assert size[1] == size[2] == pytest.approx(scene.MARKER_PANEL)
        texture = v.find(".//albedo_map").text.replace("model://station_marker/", "")
        assert os.path.isfile(os.path.join(PKG, "models/station_marker", texture))
    assert root.find(".//collision") is None, "the plate is visual only"


def test_plate_is_above_the_lidar_plane_and_not_in_the_map():
    assert scene.MARKER_HEIGHT - scene.MARKER_PANEL / 2.0 > scene.LIDAR_PLANE_Z + 0.01


def _camera():
    root = ET.fromstring(read("urdf/station_alignment_test_robot.urdf.xacro"))
    sensor = next(s for s in root.iter("sensor") if s.get("name") == "station_alignment_base_camera")
    pose = [float(a) for a in sensor.find("pose").text.split()]
    hfov = float(sensor.find("camera/horizontal_fov").text)
    width = int(sensor.find("camera/image/width").text)
    height = int(sensor.find("camera/image/height").text)
    return pose, hfov, width, height


def _project(robot, point, camera):
    """Pixel (u, v) of a world point, or None behind the camera. base_link is 0.098 m above base_footprint."""
    (cx, cy, cz, _, pitch, _), hfov, width, height = camera
    bx, by, _ = geometry.between(robot, (point[0], point[1], 0.0))
    dx, dy, dz = bx - cx, by - cy, point[2] - (0.098 + cz)
    # camera frame: x forward, y left, z up, pitched by `pitch` about y (negative = up)
    fx_ = math.cos(pitch) * dx - math.sin(pitch) * dz
    fz_ = math.sin(pitch) * dx + math.cos(pitch) * dz
    if fx_ <= 0.05:
        return None
    f = (width / 2.0) / math.tan(hfov / 2.0)
    return width / 2.0 - f * dy / fx_, height / 2.0 - f * fz_ / fx_


def _visible_tags(offset):
    camera = _camera()
    robot = geometry.compose(scene.station(), offset)
    half = scene.MARKER_TAG_SIZE / 2.0
    (_, _), (nx, ny) = scene.shelf_front()
    ux, uy = ny, -nx
    seen = set()
    for tag, (x, y, z) in scene.marker_centres().items():
        pixels = [_project(robot, (x + ux * su * half, y + uy * su * half, z + sz * half), camera)
                  for su, sz in itertools.product((-1, 1), (-1, 1))]
        if all(p is not None and 10 <= p[0] <= camera[2] - 10 and 10 <= p[1] <= camera[3] - 10 for p in pixels):
            seen.add(tag)
    return seen


@pytest.mark.parametrize("dx,dy,dyaw", list(itertools.product((-0.06, 0.0, 0.06), (-0.06, 0.0, 0.06), (-0.08, 0.08))))
def test_both_tags_are_in_the_image_where_station_approach_stops(dx, dy, dyaw):
    # 0.058 m: the largest AMCL error at the station in Baseline 1; 0.08 rad: the reach budget's yaw
    assert _visible_tags((dx, dy, dyaw)) == set(scene.MARKER_IDS)


def test_one_tag_drops_out_beyond_that_envelope():
    assert _visible_tags((0.06, 0.06, 0.1)) != set(scene.MARKER_IDS)


_OCCUPIED = []


def _occupied():
    if _OCCUPIED:
        return _OCCUPIED
    width, height, grid = scene.occupancy()
    cells = []
    for row, col in itertools.product(range(height), range(width)):
        if grid[row][col] == 0:
            cells.append((-scene.FLOOR[0] / 2 + (col + 0.5) * scene.MAP_RESOLUTION,
                          -scene.FLOOR[1] / 2 + (height - 1 - row + 0.5) * scene.MAP_RESOLUTION))
    _OCCUPIED.extend(cells)
    return cells


def test_map_has_the_shelf_posts_and_the_obstacles():
    cells = _occupied()
    for _, size, centre, _ in scene.static_boxes():
        if centre[2] - size[2] / 2 <= scene.LIDAR_PLANE_Z <= centre[2] + size[2] / 2:
            assert any(math.hypot(x - centre[0], y - centre[1]) < 0.06 for x, y in cells), centre
    assert re.search(r"origin: \[-7.0, -5.0, 0.0\]", read("maps/station_alignment_map.yaml"))


@pytest.mark.parametrize("pose", [scene.station()] + list(evaluation.START_POSES))
def test_station_and_trial_start_poses_are_clear(pose):
    # 0.47 m: the circumscribed radius of the nav2_params.yaml footprint plus a margin
    nearest = min(math.hypot(x - pose[0], y - pose[1]) for x, y in _occupied())
    assert nearest > 0.47 or pose == scene.station() and nearest > 0.40


def test_tag_textures_have_notice_and_licence():
    notice = os.path.join(PKG, "..", "..", "..", "NOTICE.md")
    with open(notice, encoding="utf-8") as f:
        text = f.read()
    textures = os.path.join(PKG, "models", "station_marker", "materials", "textures")
    for tag in scene.MARKER_IDS:
        assert f"apriltag_36h11_id{tag}.png" in text
    with open(os.path.join(textures, "LICENSE.apriltag-imgs"), encoding="utf-8") as f:
        licence = f.read()
    assert "BSD 2-Clause License" in licence and "The Regents of The University of Michigan" in licence
