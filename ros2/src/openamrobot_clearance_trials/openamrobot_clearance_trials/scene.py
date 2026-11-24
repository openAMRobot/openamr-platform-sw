# SPDX-License-Identifier: MIT
"""The clearance trial scene: a walled arena with one straight aisle, and the obstacle sets of the scenarios.

Map frame = Gazebo world frame. The robot starts at the origin facing +x (nav2_params.yaml's AMCL initial pose).
Everything in MAPPED is in the Nav2 map. The scenario obstacles are not: they are spawned for one trial and removed,
as a cart left in an aisle would be. Every value is a simulation input, not a site measurement.

  python3 -m openamrobot_clearance_trials.scene <output dir> [aisle width]   writes clearance.sdf and clearance_map.*
"""
import math
import os
import sys

from openamrobot_clearance_trials import geometry

WORLD_NAME = "clearance"
FLOOR = (40.0, 20.0)                     # Gazebo floor plane, centred on the origin
MAP_ORIGIN, MAP_SIZE = (-7.0, -6.0), (24.0, 12.0)
ARENA_X, ARENA_Y = (-3.0, 13.0), (-4.0, 4.0)
WALL, WALL_HEIGHT = 0.10, 1.00
AISLE_X = (3.0, 9.0)
AISLE_WIDTH = 1.50                       # Baseline 1's aisle width (openamrobot_station_alignment scene.py)
START, GOAL = (0.0, 0.0, 0.0), (11.0, 0.0, 0.0)
ROBOT_WIDTH = 0.580                      # chassis collision box, y
MAP_RESOLUTION = 0.05
LIDAR_PLANE_Z = 0.222
# Mapped features that are not symmetric, so that AMCL has more than two parallel walls to hold on to.
PILLARS = ((0.0, 2.5, 0.15), (11.0, -2.5, 0.15), (1.5, -3.0, 0.10))
BLOCKS = (((0.40, 0.40), (-1.5, -2.0, 0.0)), ((0.60, 0.30), (12.0, 2.8, 0.0)))


def mapped(aisle_width=AISLE_WIDTH):
    """[(name, kind, data, height)]: kind "box" -> ((sx, sy), (x, y, yaw)); "cyl" -> (x, y, r)."""
    (x0, x1), (y0, y1) = ARENA_X, ARENA_Y
    out = [("wall_south", "box", ((x1 - x0 + WALL, WALL), ((x0 + x1) / 2.0, y0, 0.0)), WALL_HEIGHT),
           ("wall_north", "box", ((x1 - x0 + WALL, WALL), ((x0 + x1) / 2.0, y1, 0.0)), WALL_HEIGHT),
           ("wall_west", "box", ((WALL, y1 - y0 + WALL), (x0, (y0 + y1) / 2.0, 0.0)), WALL_HEIGHT),
           ("wall_east", "box", ((WALL, y1 - y0 + WALL), (x1, (y0 + y1) / 2.0, 0.0)), WALL_HEIGHT)]
    length, centre = AISLE_X[1] - AISLE_X[0], (AISLE_X[0] + AISLE_X[1]) / 2.0
    for name, sign in (("aisle_left", 1.0), ("aisle_right", -1.0)):
        out.append((name, "box", ((length, WALL), (centre, sign * (aisle_width + WALL) / 2.0, 0.0)), WALL_HEIGHT))
    for i, (x, y, r) in enumerate(PILLARS):
        out.append((f"pillar_{i}", "cyl", (x, y, r), WALL_HEIGHT))
    for i, (size, pose) in enumerate(BLOCKS):
        out.append((f"block_{i}", "box", (size, pose), 0.50))
    return out


def shapes(items):
    """Plane shapes for geometry.shape_distance."""
    out = {}
    for name, kind, data, _ in items:
        out[name] = ("box", geometry.box(data[0][0], data[0][1], data[1])) if kind == "box" else ("cyl", data)
    return out


def _box_obstacle(name, size_x, size_y, x, y, yaw=0.0, height=1.0):
    return (name, "box", ((size_x, size_y), (x, y, yaw)), height)


def scenarios(aisle_width=AISLE_WIDTH):
    """{name: {"obstacles": [...], "expect": "pass" | "blocked", "note": str, "remove_after_stop_s": float|None}}.

    gap = free width between the cart and the opposite aisle wall. The chassis is 0.58 m wide, so the largest
    possible clearance on each side is (gap - 0.58) / 2.
    """
    half = aisle_width / 2.0
    mid = (AISLE_X[0] + AISLE_X[1]) / 2.0

    def cart_with_gap(gap, length=0.90):
        width = aisle_width - gap
        return _box_obstacle("cart", length, width, mid, -half + width / 2.0)

    bypass_len = ARENA_Y[1] - half - WALL
    closures = [_box_obstacle("closure_north", 0.30, bypass_len - 0.02, mid, half + WALL + bypass_len / 2.0),
                _box_obstacle("closure_south", 0.30, bypass_len - 0.02, mid, -(half + WALL + bypass_len / 2.0))]
    block = _box_obstacle("cart", 0.90, aisle_width - 0.02, mid, 0.0)
    out = {
        "free": {"obstacles": [], "expect": "pass", "note": "no obstacle: reference time and path"},
        "open_box": {"obstacles": [_box_obstacle("cart", 0.50, 0.90, 1.6, 0.0)], "expect": "pass",
                     "note": "cart 0.5 x 0.9 m on the path in the open area, free on both sides"},
        "gap100": {"obstacles": [cart_with_gap(1.00)], "expect": "pass",
                   "note": "cart in the aisle, 1.00 m free: up to 0.21 m per side"},
        "gap080": {"obstacles": [cart_with_gap(0.80)], "expect": "pass",
                   "note": "cart in the aisle, 0.80 m free: up to 0.11 m per side"},
        "gap100_only": {"obstacles": [cart_with_gap(1.00)] + closures, "expect": "pass",
                        "note": "cart in the aisle, 1.00 m free, and the bypass closed: the gap is the only route"},
        "gap066": {"obstacles": [cart_with_gap(0.66)], "expect": "either",
                   "note": "cart in the aisle, 0.66 m free: at most 0.04 m per side; passing is below the 0.10 m target,"
                           " the bypass outside the aisle is free"},
        "gap050": {"obstacles": [cart_with_gap(0.50)], "expect": "pass",
                   "note": "cart in the aisle, 0.50 m free: narrower than the chassis; the bypass outside is free"},
        "aisle_blocked": {"obstacles": [block], "expect": "pass",
                          "note": "aisle closed; the bypass outside the aisle is free"},
        "temporary": {"obstacles": [block] + closures, "expect": "pass", "remove_after_stop_s": 20.0,   # summed standstill time, not one stretch
                      "note": "every route closed; the aisle block is removed once the robot has stood still for 20 s in total"},
        "all_blocked": {"obstacles": [block] + closures, "expect": "blocked",
                        "note": "every route closed for the whole trial: the robot must stay stopped and report"},
    }
    out["head_on_block"] = dict(out["all_blocked"], note="every route closed: the approach to the block, straight on,"
                                " gives the stopping behaviour (smallest distance and the speed profile)")
    # Rotation next to an obstacle: the goal is in the aisle beside the cart, facing back. The chassis diagonal is
    # 0.972 m, the free width 1.00 m: a turn on the spot fits with 0.014 m per side only when centred in the gap.
    out["turn_beside_cart"] = {"obstacles": [cart_with_gap(1.00, length=2.0)], "expect": "either",
                               "target": (mid, half - 0.50, math.pi),
                               "note": "goal beside a 2 m cart in the aisle, heading reversed: a turn on the spot in a"
                                       " 1.00 m gap (chassis diagonal 0.972 m)"}
    # Moving obstacles (spawned when the robot has driven trigger_path_m). Their contacts are counted apart.
    out["crossing"] = {"obstacles": [], "expect": "pass",
                       "mover": {"size": (0.50, 0.50), "ahead_m": 1.6, "lateral_m": 1.9, "speed": 0.25,
                                 "trigger_path_m": 0.8},
                       "note": "a 0.5 m trolley starts 1.6 m ahead and 1.9 m to the left and crosses the robot's path at"
                               " 0.25 m/s: on an unchanged course the two meet"}
    out["oncoming"] = {"obstacles": [], "expect": "either",
                       "mover": {"size": (0.50, 0.50), "start": (8.5, 0.0), "velocity": (-0.20, 0.0),
                                 "trigger_path_m": 2.0},
                       "note": "a 0.5 m trolley comes towards the robot through the aisle at 0.20 m/s and does not"
                               " stop: what the robot does when it cannot get out of the way"}
    out["operator_stop"] = {"obstacles": [block] + closures, "expect": "stopped", "operator_stop_after_still_s": 10.0,
                            "note": "every route closed; the operator Stop (/safety/stop_lock_engage) is engaged while"
                                    " the robot waits, then the aisle block is removed: the robot must not move until"
                                    " the Stop is released, and the automatic recovery must not resume the goal"}
    for value in out.values():
        value.setdefault("operator_stop_after_still_s", None)
        value.setdefault("remove_after_stop_s", None)
        value.setdefault("target", None)
        value.setdefault("mover", None)
    return out


def model_sdf(name, kind, data, height, static=True):
    if kind == "box":
        (sx, sy), _ = data
        geom = f"<box><size>{sx:.4f} {sy:.4f} {height:.4f}</size></box>"
    else:
        geom = f"<cylinder><radius>{data[2]:.4f}</radius><length>{height:.4f}</length></cylinder>"
    return (f'<?xml version="1.0"?><sdf version="1.9"><model name="{name}"><static>{"true" if static else "false"}'
            f'</static><link name="link"><pose>0 0 {height / 2.0:.4f} 0 0 0</pose>'
            f'<collision name="collision"><geometry>{geom}</geometry></collision>'
            f'<visual name="visual"><geometry>{geom}</geometry><material><ambient>0.8 0.3 0.1 1</ambient>'
            f"<diffuse>0.8 0.3 0.1 1</diffuse></material></visual></link></model></sdf>")


def mover_sdf(name, size, velocity, height=1.0, mass=15.0):
    """A box that moves at a constant world velocity (VelocityControl) and reports its true pose (OdometryPublisher).
    Not static: it is a pushed trolley, and it pushes what stands in its way."""
    sx, sy = size
    ixx = mass * (sy * sy + height * height) / 12.0
    iyy = mass * (sx * sx + height * height) / 12.0
    izz = mass * (sx * sx + sy * sy) / 12.0
    geom = f"<box><size>{sx:.4f} {sy:.4f} {height:.4f}</size></box>"
    return (f'<?xml version="1.0"?><sdf version="1.9"><model name="{name}"><static>false</static>'
            f'<link name="link"><pose>0 0 {height / 2.0 + 0.005:.4f} 0 0 0</pose><inertial><mass>{mass}</mass>'
            f"<inertia><ixx>{ixx:.4f}</ixx><iyy>{iyy:.4f}</iyy><izz>{izz:.4f}</izz><ixy>0</ixy><ixz>0</ixz>"
            f"<iyz>0</iyz></inertia></inertial>"
            f'<collision name="collision"><geometry>{geom}</geometry><surface><friction><ode><mu>0.0</mu>'
            f"<mu2>0.0</mu2></ode></friction></surface></collision>"
            f'<visual name="visual"><geometry>{geom}</geometry><material><ambient>0.1 0.3 0.8 1</ambient>'
            f"<diffuse>0.1 0.3 0.8 1</diffuse></material></visual></link>"
            f'<plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">'
            f"<initial_linear>{velocity[0]:.3f} {velocity[1]:.3f} 0</initial_linear></plugin>"
            f'<plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">'
            f"<odom_frame>world</odom_frame><robot_base_frame>link</robot_base_frame>"
            f"<odom_topic>/clearance/gz/mover_odom</odom_topic><tf_topic>/clearance/gz/mover_tf</tf_topic>"
            f"<odom_publish_frequency>50</odom_publish_frequency><dimensions>3</dimensions></plugin>"
            f"</model></sdf>")


def model_pose(kind, data):
    """(x, y, yaw) of the model origin."""
    return data[1] if kind == "box" else (data[0], data[1], 0.0)


def world_sdf(aisle_width=AISLE_WIDTH):
    fx, fy = FLOOR
    out = ['<?xml version="1.0"?>', "<!-- SPDX-License-Identifier: MIT -->",
           "<!-- Generated by openamrobot_clearance_trials/scene.py. Simulation only. -->",
           '<sdf version="1.9">', f'<world name="{WORLD_NAME}">',
           '<physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size>'
           "<real_time_factor>1.0</real_time_factor></physics>",
           '<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>',
           '<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>',
           '<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>',
           '<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">'
           "<render_engine>ogre2</render_engine></plugin>",
           '<scene><ambient>0.6 0.6 0.6 1</ambient><background>0.8 0.8 0.8 1</background>'
           "<shadows>false</shadows></scene>",
           '<light type="directional" name="sun"><cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose>'
           "<diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.4 0.2 -0.9</direction>"
           "</light>",
           '<model name="floor"><static>true</static><link name="link">'
           '<collision name="collision"><geometry><plane><normal>0 0 1</normal>'
           f"<size>{fx} {fy}</size></plane></geometry><surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode>"
           "</friction></surface></collision>"
           f'<visual name="visual"><geometry><plane><normal>0 0 1</normal><size>{fx} {fy}</size></plane></geometry>'
           "<material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material></visual>"
           "</link></model>",
           '<model name="fixtures"><static>true</static><link name="link">']
    for name, kind, data, height in mapped(aisle_width):
        if kind == "box":
            (sx, sy), (x, y, yaw) = data
            geom = f"<geometry><box><size>{sx:.4f} {sy:.4f} {height:.4f}</size></box></geometry>"
            pose = f"<pose>{x:.5f} {y:.5f} {height / 2.0:.5f} 0 0 {yaw:.5f}</pose>"
        else:
            x, y, r = data
            geom = f"<geometry><cylinder><radius>{r:.4f}</radius><length>{height:.4f}</length></cylinder></geometry>"
            pose = f"<pose>{x:.5f} {y:.5f} {height / 2.0:.5f} 0 0 0</pose>"
        out.append(f'<collision name="{name}_collision">{pose}{geom}</collision>'
                   f'<visual name="{name}_visual">{pose}{geom}<material><ambient>0.55 0.45 0.3 1</ambient>'
                   "<diffuse>0.55 0.45 0.3 1</diffuse></material></visual>")
    out += ["</link></model>", "</world>", "</sdf>", ""]
    return "\n".join(out)


def occupancy(aisle_width=AISLE_WIDTH):
    width, height = int(round(MAP_SIZE[0] / MAP_RESOLUTION)), int(round(MAP_SIZE[1] / MAP_RESOLUTION))
    ox, oy = MAP_ORIGIN
    grid = [[254] * width for _ in range(height)]
    half = MAP_RESOLUTION / 2.0
    items = [(kind, data) for _, kind, data, h in mapped(aisle_width) if h >= LIDAR_PLANE_Z]
    polys = [geometry.box(d[0][0], d[0][1], d[1]) if k == "box" else None for k, d in items]
    for row in range(height):
        cy = oy + (height - 1 - row + 0.5) * MAP_RESOLUTION
        for col in range(width):
            cx = ox + (col + 0.5) * MAP_RESOLUTION
            cell = geometry.box(MAP_RESOLUTION, MAP_RESOLUTION, (cx, cy, 0.0))
            for (kind, data), poly in zip(items, polys):
                if kind == "box":
                    hit = geometry.polygon_distance(cell, poly) == 0.0
                else:
                    hit = math.hypot(max(abs(cx - data[0]) - half, 0.0), max(abs(cy - data[1]) - half, 0.0)) <= data[2]
                if hit:
                    grid[row][col] = 0
                    break
    return width, height, (ox, oy), grid


def write(directory, aisle_width=AISLE_WIDTH):
    os.makedirs(directory, exist_ok=True)
    width, height, (ox, oy), grid = occupancy(aisle_width)
    with open(os.path.join(directory, "clearance.sdf"), "w", encoding="utf-8") as f:
        f.write(world_sdf(aisle_width))
    with open(os.path.join(directory, "clearance_map.pgm"), "wb") as f:
        f.write(f"P5\n# clearance trials map, generated by scene.py\n{width} {height}\n255\n".encode("ascii"))
        f.write(bytes(v for row in grid for v in row))
    with open(os.path.join(directory, "clearance_map.yaml"), "w", encoding="utf-8") as f:
        f.write("# SPDX-License-Identifier: MIT\n# Generated by openamrobot_clearance_trials/scene.py.\n"
                f"image: clearance_map.pgm\nmode: trinary\nresolution: {MAP_RESOLUTION}\n"
                f"origin: [{ox}, {oy}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    return directory


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    directory = argv[0] if argv else "/tmp/clearance_scene"
    width = float(argv[1]) if len(argv) > 1 else AISLE_WIDTH
    print("wrote", write(directory, width), "aisle width", width)


if __name__ == "__main__":
    main()
