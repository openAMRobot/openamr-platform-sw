#!/usr/bin/env python3
"""
Check the TF tree of a robot profile and the sensor poses against P-03.

Profiles:
  urdf          the description (robo_urdf.urdf.xacro expanded with xacro), root base_footprint
  real_launch   the static transforms in real_bringup.launch.py plus the EKF odom -> base_link
  hday_legacy   same frames as real_launch (hday_legacy.launch.py includes real_bringup only)
  combined      urdf and real_launch together: what robot_state_publisher on the real path gives

Checks, per profile:
  structure   one parent per frame, no cycle, one root, every frame reaches the root
  poses       base_link -> sensor pose equals the P-03 value within 1 mm (and 0.001 rad),
              only for profiles that claim the 2.0 sensor layout (urdf, real_launch)

Exit code 0 when no violation, 1 otherwise, 2 on a usage or input error.

  python3 tools/check_tf_tree.py urdf real_launch combined
  python3 tools/check_tf_tree.py --json real_launch
"""
import argparse
import importlib.util
import json
import math
import os
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(REPO, 'ros2', 'src')
TOL_M = 0.001
TOL_RAD = 0.001

PI = math.pi

# P-03 rev20.6 values, in base_link, metres and radians (x, y, z, roll, pitch, yaw).
# imu_link: item 19 D5 update. lidar_link: item 19 (S3 and A1 mounted rotated 180 deg).
# base_camera_link: item 19 D3 update (pitch from base_tilt, default bt10 = -10 deg).
# us_*: item 19 D1. tof_*: item 19 D6, pitched 10 degrees down (rev19.7).
P03_REFERENCE = {
    'imu_link': (0.0544, 0.0, 0.1210, 0.0, 0.0, -PI / 2),
    'lidar_link': (0.330, 0.0, 0.124, 0.0, 0.0, PI),
    'base_camera_link': (0.387, 0.0, 0.046, 0.0, -math.radians(10), 0.0),
    'us_front_left': (0.390, 0.130, -0.0066, 0.0, 0.0, 0.0),
    'us_front_right': (0.390, -0.130, -0.0066, 0.0, 0.0, 0.0),
    'us_rear_left': (-0.390, 0.130, -0.0066, 0.0, 0.0, PI),
    'us_rear_right': (-0.390, -0.130, -0.0066, 0.0, 0.0, PI),
    'tof_front': (0.390, 0.0, -0.0066, 0.0, math.radians(10), 0.0),
    'tof_rear': (-0.390, 0.0, -0.0066, 0.0, math.radians(10), PI),
}
# base_footprint -> base_link: REP 105, base_link 98.0 mm above the floor.
BASE_LINK_HEIGHT = 0.098

POSE_PROFILES = ('urdf', 'real_launch')


# ---- small rigid-transform maths (4x4 as nested lists) ---------------------------------

def _mat(x, y, z, roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y],
        [-sp, cp * sr, cp * cr, z],
        [0.0, 0.0, 0.0, 1.0]]


def _mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _inv(m):
    rot = [[m[j][i] for j in range(3)] for i in range(3)]
    t = [-sum(rot[i][k] * m[k][3] for k in range(3)) for i in range(3)]
    return [rot[0] + [t[0]], rot[1] + [t[1]], rot[2] + [t[2]], [0.0, 0.0, 0.0, 1.0]]


def _rotation_angle(m):
    trace = m[0][0] + m[1][1] + m[2][2]
    return math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0)))


class Edge:
    """One transform: child pose in parent."""

    def __init__(self, parent, child, xyz=(0, 0, 0), rpy=(0, 0, 0), source=''):
        self.parent = parent
        self.child = child
        self.xyz = tuple(float(v) for v in xyz)
        self.rpy = tuple(float(v) for v in rpy)
        self.source = source

    def matrix(self):
        return _mat(*self.xyz, *self.rpy)


# ---- structure and pose checks (pure functions, unit tested) ------------------------------

def check_structure(edges):
    """Return a list of violation strings for a list of Edge."""
    problems = []
    parents = {}
    for edge in edges:
        if edge.child in parents:
            first = parents[edge.child]
            problems.append('%s has two parents: %s (%s) and %s (%s)' % (
                edge.child, first.parent, first.source, edge.parent, edge.source))
        else:
            parents[edge.child] = edge
    frames = set(parents) | {e.parent for e in edges}
    roots = set()
    cycles = set()
    for frame in sorted(frames):
        seen = [frame]
        while seen[-1] in parents:
            nxt = parents[seen[-1]].parent
            if nxt in seen:
                loop = seen[seen.index(nxt):] + [nxt]
                key = frozenset(loop)
                if key not in cycles:
                    cycles.add(key)
                    problems.append('cycle: ' + ' -> '.join(loop))
                break
            seen.append(nxt)
        else:
            roots.add(seen[-1])
    cyclic = any(p.startswith('cycle') for p in problems)
    if not cyclic and len(roots) != 1:
        problems.append('expected one root, found %d: %s' % (len(roots), ', '.join(sorted(roots))))
    if not frames:
        problems.append('no frames')
    return problems


def pose_in_base_link(edges, frame):
    """Pose of frame in base_link, walking through the common root. None if unreachable."""
    parents = {e.child: e for e in edges}

    def to_root(name):
        m = _mat(0, 0, 0, 0, 0, 0)
        seen = set()
        while name in parents:
            if name in seen:
                return None
            seen.add(name)
            m = _mul(parents[name].matrix(), m)
            name = parents[name].parent
        return name, m

    a, b = to_root(frame), to_root('base_link')
    if a is None or b is None or a[0] != b[0]:
        return None
    return _mul(_inv(b[1]), a[1])


def check_poses(edges, reference=None, tol_m=TOL_M, tol_rad=TOL_RAD):
    """Compare sensor poses in base_link with the reference. Returns violation strings."""
    reference = P03_REFERENCE if reference is None else reference
    problems = []
    for frame, (x, y, z, roll, pitch, yaw) in sorted(reference.items()):
        pose = pose_in_base_link(edges, frame)
        if pose is None:
            problems.append('%s: not published in this profile' % frame)
            continue
        want = _mat(x, y, z, roll, pitch, yaw)
        dist = math.sqrt(sum((pose[i][3] - want[i][3]) ** 2 for i in range(3)))
        angle = _rotation_angle(_mul(_inv(want), pose))
        if dist > tol_m or angle > tol_rad:
            problems.append(
                '%s: off by %.1f mm and %.3f rad (have xyz %.4f %.4f %.4f, want %.4f %.4f %.4f)' % (
                    frame, dist * 1000, angle, pose[0][3], pose[1][3], pose[2][3], x, y, z))
    return problems


def check_base_height(edges):
    """base_footprint -> base_link must be 98.0 mm up when both frames are in the tree."""
    pose = pose_in_base_link(edges, 'base_footprint')
    if pose is None:
        return ['base_footprint: not reachable from base_link']
    # pose is base_footprint in base_link: z should be -0.098 and x, y zero
    if abs(pose[2][3] + BASE_LINK_HEIGHT) > TOL_M or abs(pose[0][3]) > TOL_M \
            or abs(pose[1][3]) > TOL_M:
        return ['base_footprint is %.1f mm below base_link, want 98.0 mm' % (-pose[2][3] * 1000)]
    return []


# ---- profile builders -------------------------------------------------------------------

def urdf_edges(xacro_args=None):
    """Expand the URDF with xacro and return its joints as Edge objects."""
    import xml.etree.ElementTree as ET
    path = os.path.join(SRC, 'openamrobot_description', 'urdf', 'robo_urdf.urdf.xacro')
    try:
        import xacro
        doc = xacro.process_file(path, mappings=xacro_args or {})
        text = doc.toxml()
    except ImportError:
        result = subprocess.run(['xacro', path] + ['%s:=%s' % kv for kv in (xacro_args or {}).items()],
                                capture_output=True, text=True)
        if result.returncode != 0:
            raise RuntimeError('xacro failed: ' + result.stderr)
        text = result.stdout
    root = ET.fromstring(text)
    edges = []
    for joint in root.iter('joint'):
        origin = joint.find('origin')
        xyz = [float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None
                                  else '0 0 0').split()]
        rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None
                                  else '0 0 0').split()]
        edges.append(Edge(joint.find('parent').get('link'), joint.find('child').get('link'),
                          xyz, rpy, 'urdf joint ' + joint.get('name')))
    return edges


def _load_launch(name):
    spec = importlib.util.spec_from_file_location(
        name, os.path.join(SRC, 'openamrobot_bringup', 'launch', name + '.launch.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def launch_edges():
    """Static transforms of real_bringup.launch.py plus the EKF odom -> base_link."""
    import yaml
    from launch import LaunchContext
    from launch.utilities import perform_substitutions
    module = _load_launch('real_bringup')
    module.get_package_share_directory = lambda pkg: os.path.join('/fake-share', pkg)
    edges = []
    for entity in module.generate_launch_description().entities:
        args = getattr(entity, '_Node__arguments', None)
        if not args:
            continue
        flat = []
        for item in args:
            flat.append(item if isinstance(item, str)
                        else perform_substitutions(LaunchContext(), item))
        if '--frame-id' not in flat:
            continue

        def value(flag):
            return float(flat[flat.index(flag) + 1]) if flag in flat else 0.0
        edges.append(Edge(
            flat[flat.index('--frame-id') + 1], flat[flat.index('--child-frame-id') + 1],
            (value('--x'), value('--y'), value('--z')),
            (value('--roll'), value('--pitch'), value('--yaw')),
            'static_transform_publisher in real_bringup.launch.py'))
    with open(os.path.join(SRC, 'openamrobot_bringup', 'config', 'ekf.yaml'),
              encoding='utf-8') as f:
        cfg = yaml.safe_load(f)['ekf_filter_node']['ros__parameters']
    if cfg.get('publish_tf'):
        edges.append(Edge(cfg['odom_frame'], cfg['base_link_frame'], source='ekf.yaml publish_tf'))
    return edges


def build(profile):
    if profile == 'urdf':
        return urdf_edges()
    if profile in ('real_launch', 'hday_legacy'):
        return launch_edges()
    if profile == 'combined':
        return urdf_edges() + launch_edges()
    raise ValueError('unknown profile ' + profile)


def check_profile(profile):
    edges = build(profile)
    structure = check_structure(edges)
    poses = []
    height = []
    if profile in POSE_PROFILES:
        poses = check_poses(edges)
    if profile == 'urdf':
        height = check_base_height(edges)
    return {'profile': profile, 'frames': len({e.child for e in edges} | {e.parent for e in edges}),
            'structure': structure, 'poses': poses, 'base_height': height}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('profiles', nargs='*', default=['urdf', 'real_launch', 'hday_legacy'],
                        choices=['urdf', 'real_launch', 'hday_legacy', 'combined'])
    parser.add_argument('--json', action='store_true')
    args = parser.parse_args(argv)
    try:
        results = [check_profile(p) for p in args.profiles]
    except Exception as exc:  # input error: not a tree violation
        print('check_tf_tree: %s' % exc, file=sys.stderr)
        return 2
    bad = sum(len(r['structure']) + len(r['poses']) + len(r['base_height']) for r in results)
    if args.json:
        print(json.dumps(results, indent=2))
    else:
        for r in results:
            print('== %s (%d frames)' % (r['profile'], r['frames']))
            for kind in ('structure', 'poses', 'base_height'):
                for line in r[kind]:
                    print('  VIOLATION [%s] %s' % (kind, line))
            if not (r['structure'] or r['poses'] or r['base_height']):
                print('  ok')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
