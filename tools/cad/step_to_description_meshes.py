#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Generate the openamrobot_description base visual meshes from the v0.2 full-assembly STEP.

The chassis comes from the CAD (our MMP.* parts, exported as meshes); the components come
from the 2.0 bill of materials and are simplified envelopes built from the parameters in
tools/cad/base_mesh_map.yaml. Release 001 parts in the STEP are excluded. Every leaf instance
of the assembly must be classified by the map, or the run fails.

Outputs:
  ros2/src/openamrobot_description/meshes/visual/*.stl   binary STL, metres, per link and material
  ros2/src/openamrobot_description/meshes/PROVENANCE.yaml inputs, versions, parameters, classes
  build/base_visual.glb                                   same geometry in base_link, for the docs;
                                                          one node per part with glTF extras
  build/cad_checks.md, build/cad_checks.yaml              engineering checks (reported only)

Usage, from the repository root:
  python3 tools/cad/step_to_description_meshes.py --hw-repo ../openamr-platform-hw
"""
import argparse
import hashlib
import importlib.metadata
import json
import math
import os
import platform
import re
import struct
import subprocess
import sys
import tempfile
import zipfile

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cadlib  # noqa: E402

from OCP.BRepAdaptor import BRepAdaptor_Surface  # noqa: E402
from OCP.GeomAbs import GeomAbs_Cylinder  # noqa: E402
from OCP.gp import gp_Trsf  # noqa: E402
from OCP.TopAbs import TopAbs_FACE  # noqa: E402
from OCP.TopExp import TopExp_Explorer  # noqa: E402
from OCP.TopLoc import TopLoc_Location  # noqa: E402
from OCP.TopoDS import TopoDS  # noqa: E402

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
PKG = os.path.join(REPO, 'ros2', 'src', 'openamrobot_description')
STL_HEADER = b'openamr step_to_description_meshes binary STL, metres'.ljust(80, b' ')


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def git(repo, *args):
    try:
        return subprocess.check_output(['git', '-C', repo] + list(args), text=True,
                                       stderr=subprocess.DEVNULL).strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return None


def version(dist):
    try:
        return importlib.metadata.version(dist)
    except importlib.metadata.PackageNotFoundError:
        return 'not installed'


# ---------------------------------------------------------------- geometry helpers

def rpy_matrix(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def pose_matrix(xyz, rpy):
    m = np.eye(4)
    m[:3, :3] = rpy_matrix(rpy)
    m[:3, 3] = xyz
    return m


def to_loc(matrix):
    t = gp_Trsf()
    m = matrix
    t.SetValues(m[0, 0], m[0, 1], m[0, 2], m[0, 3], m[1, 0], m[1, 1], m[1, 2], m[1, 3],
                m[2, 0], m[2, 1], m[2, 2], m[2, 3])
    return TopLoc_Location(t)


def box_tris(lo, hi):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    c = np.array([[lo[0], lo[1], lo[2]], [hi[0], lo[1], lo[2]], [hi[0], hi[1], lo[2]], [lo[0], hi[1], lo[2]],
                  [lo[0], lo[1], hi[2]], [hi[0], lo[1], hi[2]], [hi[0], hi[1], hi[2]], [lo[0], hi[1], hi[2]]])
    f = [[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4],
         [1, 2, 6], [1, 6, 5], [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]]
    return c[np.array(f)]


def axis_frame(axis):
    """Rotation taking local z to the given axis (x, y or z)."""
    return {'x': rpy_matrix([0.0, math.pi / 2, 0.0]),
            'y': rpy_matrix([-math.pi / 2, 0.0, 0.0]),
            'z': np.eye(3)}[axis]


def revolve_tris(profile, sections):
    """Closed surface of revolution about local z; profile [[axial, radius], ...] in order."""
    pts = np.asarray(profile, float)
    ang = np.linspace(0.0, 2.0 * math.pi, sections, endpoint=False)
    ring = np.stack([np.cos(ang), np.sin(ang)], axis=1)
    tris = []
    for (a0, r0), (a1, r1) in zip(pts[:-1], pts[1:]):
        for i in range(sections):
            j = (i + 1) % sections
            p00 = [r0 * ring[i, 0], r0 * ring[i, 1], a0]
            p01 = [r0 * ring[j, 0], r0 * ring[j, 1], a0]
            p10 = [r1 * ring[i, 0], r1 * ring[i, 1], a1]
            p11 = [r1 * ring[j, 0], r1 * ring[j, 1], a1]
            if r0 > 0:
                tris.append([p00, p10, p01])
            if r1 > 0:
                tris.append([p01, p10, p11])
    return np.array(tris)


def cylinder_tris(radius, length, sections):
    h = length / 2.0
    return revolve_tris([[h, 0.0], [h, radius], [-h, radius], [-h, 0.0]], sections)


def part_tris(part, tess):
    """Triangles (mm) of one envelope part in its placement frame."""
    shape = part['shape']
    if shape == 'box':
        if 'box_mm' in part:
            lo, hi = part['box_mm']
            return box_tris(lo, hi)
        size = np.asarray(part['size_mm'], float)
        tris = box_tris(-size / 2, size / 2)
    elif shape == 'cylinder':
        tris = cylinder_tris(part['radius_mm'], part['length_mm'], part.get('sections', tess['cylinder_sections']))
        tris = tris @ axis_frame(part['axis']).T
    elif shape == 'revolve':
        tris = revolve_tris(part['profile_mm'], tess['revolve_sections'])
        tris = tris @ axis_frame(part['axis']).T
    else:
        raise ValueError('unknown envelope shape %s' % shape)
    m = pose_matrix(part.get('xyz_mm', [0, 0, 0]), part.get('rpy', [0, 0, 0]))
    return cadlib.transform(tris, m)


def expand_parts(parts):
    """Envelope parts with each repeat {count, step_mm} written out as separate parts.

    Separate parts keep small boxes out of the decimation (a part of 12 triangles is never reduced)."""
    out = []
    for part in parts:
        rep = part.get('repeat')
        if not rep:
            out.append(part)
            continue
        base = [float(v) for v in part.get('xyz_mm', part.get('xyz_rel_mm', [0.0, 0.0, 0.0]))]
        key = 'xyz_rel_mm' if 'xyz_rel_mm' in part else 'xyz_mm'
        for i in range(int(rep['count'])):
            p = {k: v for k, v in part.items() if k != 'repeat'}
            p[key] = [round(b + i * float(d), 6) for b, d in zip(base, rep['step_mm'])]
            p['index'] = i
            out.append(p)
    return out


def part_half_x(part):
    """Half extent along x of a box or a cylinder part (mm)."""
    if part['shape'] == 'box':
        return float(part['size_mm'][0]) / 2
    if part.get('axis') == 'x':
        return float(part['length_mm']) / 2
    return float(part['radius_mm'])


def part_rgba(part):
    return list(part['colour']) + [float(part.get('alpha', 1.0))]


def merge_vertices(tris):
    """Indexed mesh with exactly coincident vertices merged."""
    flat = np.round(tris.reshape(-1, 3), 7)
    verts, inverse = np.unique(flat, axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    keep = (faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])
    return verts, faces[keep]


def decimate(tris, max_faces):
    """Triangles reduced to at most max_faces with quadric decimation."""
    import fast_simplification
    if len(tris) <= max_faces:
        return tris
    verts, faces = merge_vertices(tris)
    reduction = 1.0 - max_faces / float(len(faces))
    v2, f2 = fast_simplification.simplify(verts.astype(np.float32), faces.astype(np.int32),
                                          target_reduction=max(0.0, min(0.999, reduction)))
    return np.asarray(v2, float)[np.asarray(f2)]


def decimate_parts(parts, max_faces):
    """Decimate each part to a common ratio so that the parts together fit max_faces.

    Thin sheet-metal parts stop reducing early (quadric collapse would flip faces); the ratio
    then drops for the other parts. Fails when the parts cannot reach max_faces."""
    total = sum(len(p) for p in parts)
    if total <= max_faces:
        return list(parts)
    ratio = max_faces / float(total)
    for _ in range(40):
        out = [decimate(p, max(12, int(len(p) * ratio))) for p in parts]
        if sum(len(p) for p in out) <= max_faces:
            return out
        ratio *= 0.9
    raise RuntimeError('parts cannot be decimated to %d faces' % max_faces)


def write_stl(path, tris):
    tris = np.asarray(tris, np.float64)
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    norm = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.divide(n, norm, out=np.zeros_like(n), where=norm > 0)
    rec = np.zeros(len(tris), dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
    rec['n'] = n
    rec['v'] = tris
    with open(path, 'wb') as stream:
        stream.write(STL_HEADER)
        stream.write(struct.pack('<I', len(tris)))
        stream.write(rec.tobytes())


def write_glb(path, nodes, extras):
    """Minimal glTF 2.0 binary: one node and mesh per part, one primitive per colour.

    nodes: [(extras dict with 'id', [(rgba, tris), ...]), ...]. Positions float32, indices
    uint16/32. Positions are base_link metres (x forward, y left, z up); the root node rotates
    them into the glTF y-up convention for generic viewers."""
    buf = bytearray()
    views, accessors, meshes, materials, out_nodes = [], [], [], [], []
    material_index = {}
    for meta, prims in nodes:
        gl_prims = []
        for rgba, tris in prims:
            verts, faces = merge_vertices(tris)
            pos = verts.astype('<f4').tobytes()
            itype, idt = (5123, '<u2') if len(verts) < 65536 else (5125, '<u4')
            idx = faces.astype(idt).tobytes()
            for data, target in ((pos, 34962), (idx, 34963)):
                while len(buf) % 4:
                    buf.append(0)
                views.append({'buffer': 0, 'byteOffset': len(buf), 'byteLength': len(data), 'target': target})
                buf.extend(data)
            accessors.append({'bufferView': len(views) - 2, 'componentType': 5126, 'count': len(verts),
                              'type': 'VEC3', 'min': [float(v) for v in verts.min(0)],
                              'max': [float(v) for v in verts.max(0)]})
            accessors.append({'bufferView': len(views) - 1, 'componentType': itype, 'count': int(faces.size),
                              'type': 'SCALAR'})
            key = tuple(round(float(c), 6) for c in rgba)
            if key not in material_index:
                material_index[key] = len(materials)
                materials.append({'name': ','.join('%.3f' % c for c in key), 'pbrMetallicRoughness': {
                    'baseColorFactor': list(key), 'metallicFactor': 0.0, 'roughnessFactor': 0.7}})
                if key[3] < 1.0:
                    materials[-1].update({'alphaMode': 'BLEND', 'doubleSided': True})
            gl_prims.append({'attributes': {'POSITION': len(accessors) - 2}, 'indices': len(accessors) - 1,
                             'material': material_index[key]})
        meshes.append({'name': meta['id'], 'primitives': gl_prims})
        out_nodes.append({'name': meta['id'], 'mesh': len(meshes) - 1, 'extras': meta})
    root = {'name': 'base_link', 'rotation': [-math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)],
            'children': list(range(len(out_nodes))), 'extras': extras}
    out_nodes.append(root)
    while len(buf) % 4:
        buf.append(0)
    doc = {'asset': {'version': '2.0', 'generator': 'openamr-platform-sw tools/cad/step_to_description_meshes.py'},
           'scene': 0, 'scenes': [{'nodes': [len(out_nodes) - 1]}], 'nodes': out_nodes, 'meshes': meshes,
           'materials': materials, 'accessors': accessors, 'bufferViews': views,
           'buffers': [{'byteLength': len(buf)}]}
    js = json.dumps(doc, separators=(',', ':'), sort_keys=True).encode()
    js += b' ' * ((4 - len(js) % 4) % 4)
    total = 12 + 8 + len(js) + 8 + len(buf)
    with open(path, 'wb') as stream:
        stream.write(struct.pack('<III', 0x46546C67, 2, total))
        stream.write(struct.pack('<II', len(js), 0x4E4F534A) + js)
        stream.write(struct.pack('<II', len(buf), 0x004E4942) + bytes(buf))


def glb_size(nodes):
    size = 0
    for meta, prims in nodes:
        size += len(json.dumps(meta)) + 200
        for _, tris in prims:
            verts, faces = merge_vertices(tris)
            size += 12 * len(verts) + faces.size * (2 if len(verts) < 65536 else 4) + 400
    return size


def region_points(tris, lo, hi, sample_edge=2.0):
    """Sample points (vertices and centroids of the subdivided triangles) inside the box lo, hi."""
    if not len(tris):
        return np.zeros((0, 3))
    s = cadlib.subdivide(tris, sample_edge, keep=(lo, hi))
    pts = np.concatenate([s.reshape(-1, 3), s.mean(1)])
    return pts[np.all((pts >= lo) & (pts <= hi), axis=1)]


def position_suffix(centres):
    """Suffixes (front/rear, left/right) that tell instances of one part apart by position."""
    c = np.asarray(centres, float)
    if len(c) == 1:
        return ['']
    words = [[] for _ in c]
    for axis, (pos, neg) in ((0, ('front', 'rear')), (1, ('left', 'right'))):
        if np.ptp(c[:, axis]) > 20.0:
            for w, v in zip(words, c[:, axis]):
                w.append(pos if v > 0 else neg)
    out = ['_' + '_'.join(w) if w else '' for w in words]
    if len(set(out)) != len(out):
        raise SystemExit('instances at the same position cannot get distinct node ids: %s' % out)
    return out


def points_box_clearance(lo, hi, pts):
    """Signed clearance (mm) from an axis-aligned box to sample points, and its axis.

    The smallest distance from the box to the points, or minus the deepest penetration when points lie
    inside the box, with the axis of that penetration; None without points."""
    if not len(pts):
        return None, None
    inside = np.all((pts > lo) & (pts < hi), axis=1)
    if inside.any():
        q = pts[inside]
        depth = np.minimum(q - lo, hi - q)       # distance to each face pair, per axis
        k = np.argmin(depth, axis=1)
        d = depth[np.arange(len(q)), k]
        i = int(np.argmax(d))
        return -round(float(d[i]), 2), 'xyz'[k[i]]
    gap = np.maximum(np.maximum(lo - pts, pts - hi), 0.0)
    dist = np.linalg.norm(gap, axis=1)
    i = int(np.argmin(dist))
    return round(float(dist[i]), 2), 'xyz'[int(np.argmax(gap[i]))]


# ---------------------------------------------------------------- converter

class Converter:

    def __init__(self, args):
        self.args = args
        with open(args.map, encoding='utf-8') as stream:
            self.map = yaml.safe_load(stream)
        self.tess = self.map['tessellation']
        for comp in self.map['components'].values():
            if 'parts' in comp:
                comp['parts'] = expand_parts(comp['parts'])
        self.inputs = []
        self.corrections = []
        self.findings = []

    def log(self, *msg):
        print(*msg, flush=True)

    # -- inputs
    def open_assembly(self):
        src = self.map['sources']['assembly']
        zpath = os.path.join(self.args.hw_repo, src['path'])
        scratch = self.args.scratch or tempfile.mkdtemp(prefix='openamr-step-')
        if os.path.abspath(scratch).startswith(REPO + os.sep):
            raise SystemExit('the scratch directory must be outside the repository')
        os.makedirs(scratch, exist_ok=True)
        with zipfile.ZipFile(zpath) as archive:
            archive.extract(src['member'], scratch)
        step = os.path.join(scratch, src['member'])
        self.add_input('assembly', src, zpath, extra={'member': src['member'], 'member_sha256': sha256(step)})
        self.log('reading', step)
        self.asm = cadlib.Assembly(step)
        self.log('leaf instances:', len(self.asm.leaves))

    def add_input(self, key, src, path, extra=None):
        rel = os.path.relpath(path, self.args.hw_repo)
        entry = {'name': key, 'repository': src['repository'], 'path': rel, 'sha256': sha256(path),
                 'checkout_commit': git(self.args.hw_repo, 'rev-parse', 'HEAD'),
                 'last_commit_for_file': git(self.args.hw_repo, 'log', '-1', '--format=%H', '--', rel) or 'not in the checkout history'}
        if key == 'hub_motor' and self.args.hub_step:
            entry['path'] = src['path']
            entry['git_ref'] = self.args.hub_ref
            entry['checkout_commit'] = self.args.hub_commit
            entry['last_commit_for_file'] = self.args.hub_commit
        if 'note' in src:
            entry['note'] = src['note']
        if extra:
            entry.update(extra)
        self.inputs.append(entry)

    # -- corrections and classification
    def apply_corrections(self):
        for corr in self.map['corrections']:
            rx = re.compile(corr['match'])
            hits = [lf for lf in self.asm.leaves if rx.search(lf.path)]
            if not hits:
                raise SystemExit('correction matches no instance: %s' % corr['match'])
            for lf in hits:
                rec = {'instance': lf.path, 'reason': ' '.join(corr['reason'].split())}
                if 'substitute' in corr:
                    src = self.map['sources'][corr['substitute']]
                    path = os.path.join(self.args.hw_repo, src['path'])
                    if not cadlib.is_void(lf.shape):
                        raise SystemExit('substitution target is not empty any more: %s' % lf.path)
                    lf.shape = cadlib.read_shape(path)
                    self.add_input(corr['substitute'], src, path)
                    rec['substituted_by'] = src['path']
                if 'set_origin_mm' in corr:
                    m = cadlib.trsf_matrix(lf.loc)
                    rec['origin_was_mm'] = [round(float(v), 3) for v in m[:3, 3]]
                    m[:3, 3] = corr['set_origin_mm']
                    lf.loc = to_loc(m)
                    rec['origin_set_mm'] = [float(v) for v in corr['set_origin_mm']]
                self.corrections.append(rec)

    def classify(self):
        rules = [(r, re.compile(r['match'])) for r in self.map['rules']]
        min_size = self.map['min_part_size_mm']
        unclassified = []
        for lf in self.asm.leaves:
            box = cadlib.bbox(lf.shape, lf.loc)
            lf.size = 0.0 if box is None else float(np.max(box[1] - box[0]))
            for rule, rx in rules:
                if rx.search(lf.path):
                    lf.rule, lf.cls = rule, rule['class']
                    break
            if lf.cls == 'chassis' and lf.size < min_size:
                lf.rule, lf.cls = {'name': 'chassis parts under %.0f mm' % min_size}, 'fastener_small'
            if lf.cls is None:
                if lf.size < min_size:
                    lf.rule, lf.cls = {'name': 'parts under %.0f mm' % min_size}, 'fastener_small'
                else:
                    unclassified.append(lf)
        if unclassified:
            for lf in unclassified:
                print('UNCLASSIFIED: %s (%.1f mm)' % (lf.path, lf.size))
            raise SystemExit('%d unclassified instances; add rules to %s' % (len(unclassified), self.args.map))
        counts = {}
        for lf in self.asm.leaves:
            counts[lf.cls] = counts.get(lf.cls, 0) + 1
        self.log('classes:', counts)

    # -- base_link frame
    def build_frame(self):
        fr = self.map['frame']
        wheels = self.asm.find(fr['axle_wheels'])
        if len(wheels) != 2:
            raise SystemExit('expected two release 001 axle wheels, found %d' % len(wheels))
        centres, axes = [], []
        for lf in wheels:
            box = cadlib.bbox(lf.shape, TopLoc_Location())
            dims = box[1] - box[0]
            k = int(np.argmin(dims))  # thin axis is the wheel axis
            m = cadlib.trsf_matrix(lf.loc)
            centres.append(m[:3, :3] @ ((box[0] + box[1]) / 2) + m[:3, 3])
            axes.append(m[:3, :3][:, k])
            self.release001_wheel_radius_mm = float(np.max(dims) / 2)
        origin = (centres[0] + centres[1]) / 2
        y = centres[0] - centres[1]
        track = float(np.linalg.norm(y))
        y /= track
        for a in axes:
            if abs(abs(float(a @ y)) - 1.0) > 1e-4:
                raise SystemExit('release 001 wheel axes are not on one line')
        front = cadlib.merge_boxes([cadlib.bbox(lf.shape, lf.loc) for lf in self.asm.find(fr['front'])])
        up = cadlib.merge_boxes([cadlib.bbox(lf.shape, lf.loc) for lf in self.asm.find(fr['up'])])
        # z: towards the cover, square to the axle; x = y cross z; y flips so that x points
        # to the front panel (y then points to the left wheel)
        u = (up[0] + up[1]) / 2 - origin
        z = u - (u @ y) * y
        z /= np.linalg.norm(z)
        x = np.cross(y, z)
        if x @ ((front[0] + front[1]) / 2 - origin) < 0:
            y, x = -y, -x
        rot = np.stack([x, y, z])
        floor_pts = [cadlib.bbox(lf.shape, lf.loc) for lf in self.asm.find(fr['floor'])]
        if len(floor_pts) != 4:
            raise SystemExit('expected four castor wheels for the floor, found %d' % len(floor_pts))
        lows = [float(np.min((np.array([[b[i][0], b[j][1], b[k][2]] for i in (0, 1) for j in (0, 1) for k in (0, 1)])
                              - origin) @ z)) for b in floor_pts]
        axle_height = -min(lows)
        self.T = np.eye(4)
        self.T[:3, :3] = rot / 1000.0
        self.T[:3, 3] = -(rot @ origin) / 1000.0
        self.Tmm = np.eye(4)
        self.Tmm[:3, :3] = rot
        self.Tmm[:3, 3] = -(rot @ origin)
        self.frame_report = {
            'release001_axle_midpoint_step_mm': [round(float(v), 3) for v in origin],
            'release001_track_mm': round(track, 3),
            'release001_wheel_radius_mm': round(self.release001_wheel_radius_mm, 3),
            'axle_height_above_castor_floor_mm': round(axle_height, 3),
            'design_base_link_height_mm': fr['base_link_height_mm'],
            'offset_mm': round(axle_height - fr['base_link_height_mm'], 3),
            'castor_contact_spread_mm': round(max(lows) - min(lows), 3),
        }
        self.log('base_link frame:', self.frame_report)
        if abs(axle_height - fr['base_link_height_mm']) > fr['tolerance_mm']:
            raise SystemExit('axle height %.2f mm differs from base_link %.1f mm' % (axle_height, fr['base_link_height_mm']))

    def to_base_link_mm(self, pts):
        return pts @ self.Tmm[:3, :3].T + self.Tmm[:3, 3]

    def unit_frames(self, pattern):
        """{unit path: (location of the unit, [leaves])} for the first path prefix matching pattern."""
        rx = re.compile(pattern)
        out = {}
        for lf in self.asm.leaves:
            for i in range(len(lf.names)):
                prefix = ' / '.join(lf.names[:i + 1])
                if rx.search(prefix):
                    out.setdefault(prefix, [lf.locs[i] if i < len(lf.names) - 1 else lf.loc, []])[1].append(lf)
                    break
        return out

    def unit_box_bl(self, leaves):
        """Axis-aligned box (mm) in base_link of leaves (corners of their STEP boxes)."""
        pts = []
        for lf in leaves:
            b = cadlib.bbox(lf.shape, lf.loc)
            if b is not None:
                pts += [[b[i][0], b[j][1], b[k][2]] for i in (0, 1) for j in (0, 1) for k in (0, 1)]
        p = self.to_base_link_mm(np.array(pts))
        return np.array([p.min(0), p.max(0)])

    # -- outputs
    def link_poses(self):
        poses = {k: pose_matrix(v['xyz'], v['rpy']) for k, v in self.map['links'].items()}
        with open(os.path.join(REPO, self.map['sensor_frames']), encoding='utf-8') as stream:
            frames = yaml.safe_load(stream)['frames']
        for k, v in frames.items():
            poses[k] = pose_matrix(v['xyz'], v['rpy'])
        return poses

    def chassis_node(self, lf):
        """Node id, name, CAD part and BOM row of a chassis instance, from its rule."""
        rule = lf.rule
        if 'node' in rule:
            return rule['node']
        for node in rule.get('nodes', []):
            if node['match'] in lf.path:
                return node
        raise SystemExit('chassis instance without a node id in %s: %s' % (self.args.map, lf.path))

    def build(self):
        self.groups = {name: [] for name in self.map['outputs']}  # link-frame or base_link tris (m) per output
        self.glb_parts = []                                       # (node id, tris in base_link m, rgba)
        self.glb_nodes = {}                                       # node id: glTF extras
        self.glb_fixed = set()                                    # node ids kept out of the GLB decimation
        chassis_colour_area = {}
        self.chassis_mm = []
        # chassis meshes
        chassis_nodes = {}
        self.chassis_ids = {}
        for lf in self.asm.leaves:
            if lf.cls != 'chassis':
                continue
            proto = cadlib.tessellate(lf.shape, self.tess['linear_deflection_mm'],
                                      self.tess['angular_deflection_rad'])
            tris = cadlib.transform(proto, self.T @ cadlib.trsf_matrix(lf.loc))
            out = lf.rule['output']
            self.groups[out].append(tris)
            self.chassis_mm.append((lf, tris * 1000.0))
            rgba = list(lf.colour or (0.6, 0.6, 0.6)) + [1.0]
            node = self.chassis_node(lf)
            chassis_nodes.setdefault(node['id'], []).append((len(self.glb_parts), lf, tris, node))
            self.glb_parts.append([node['id'], tris, rgba])
            area = float(np.sum(np.linalg.norm(np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]), axis=1)) / 2)
            chassis_colour_area.setdefault(out, {}).setdefault(tuple(rgba), 0.0)
            chassis_colour_area[out][tuple(rgba)] += area
        for base, items in chassis_nodes.items():
            centres = [(t.reshape(-1, 3).min(0) + t.reshape(-1, 3).max(0)) * 500.0 for _, _, t, _ in items]
            for (k, lf, _, node), suffix in zip(items, position_suffix(centres)):
                nid = base + suffix
                self.chassis_ids[lf.path] = nid
                self.glb_parts[k][0] = nid
                where = suffix.strip('_').replace('_', ' ')
                meta = {'id': nid, 'name': node['name'] + (' (%s)' % where if where else ''), 'class': 'chassis',
                        'source': 'CAD v0.2 part %s, instance %s (openamr-platform-hw %s)' % (
                            node['part'], lf.names[-1], self.map['sources']['assembly']['path']),
                        'output': lf.rule['output']}
                if node.get('bom_row'):
                    meta['bom_row'] = node['bom_row']
                self.glb_nodes[nid] = meta
        self.colours = {}
        for name, out in self.map['outputs'].items():
            if out['colour'] == 'from_cad':
                areas = chassis_colour_area[name]
                self.colours[name] = list(max(areas, key=areas.get))
            else:
                self.colours[name] = list(out['colour']) + [1.0]
        # panel buttons on the release 001 hole axes, battery against the chassis meshes, before the envelopes
        self.place_panel_parts()
        self.place_battery()
        # component envelopes
        poses = self.link_poses()
        self.component_report = {}
        for cid, comp in self.map['components'].items():
            rec = {'bom': comp['bom'], 'name': comp['name'], 'bom_row': comp['bom_row'], 'qty': comp['qty'],
                   'status': comp['status']}
            if comp['status'] != 'modelled':
                rec['reason'] = ' '.join(comp['reason'].split())
                self.component_report[cid] = rec
                continue
            rec.update({'envelope': True, 'output': comp['output'], 'file': self.map['outputs'][comp['output']]['file'],
                        'position': ' '.join(comp['position'].split()),
                        'position_decided': comp.get('position_decided', True),
                        'dimension_source': ' '.join(comp['dimension_source'].split()),
                        'class': comp.get('class', 'component')})
            if comp.get('provisional'):
                rec['provisional'] = True
            if comp.get('placeholder'):
                rec['placeholder'] = True
                rec['placeholder_note'] = ' '.join(comp['placeholder_note'].split())
            if cid in getattr(self, 'panel_place', {}):
                rec['position_source'] = ' '.join(comp['position_source'].split())
            if comp.get('brake_side'):
                rec['brake_side'] = comp['brake_side']
            meta = {'name': comp['name'], 'class': comp.get('class', 'component'),
                    'source': 'openamr-platform-sw meshes/PROVENANCE.yaml components.%s' % cid,
                    'bom_row': comp['bom_row'], 'output': comp['output']}
            if comp.get('provisional'):
                meta['provisional'] = True
            if comp.get('placeholder'):
                meta['placeholder'] = True
            if comp.get('glb_decimate') is False:
                self.glb_fixed.add(cid)
            local = np.concatenate([part_tris(p, self.tess) for p in comp['parts'] if p.get('stl', True)]) / 1000.0
            feats = {}
            for p in comp['parts']:
                if 'feature' in p:
                    feats[p['feature']] = feats.get(p['feature'], 0) + 1
            if feats:
                rec['features'] = feats
            place = comp['placement']
            if place.get('base_link'):
                self.groups[comp['output']].append(local)
                rec['frame'] = 'base_link'
                rec['node_ids'] = [cid]
                self.glb_nodes[cid] = dict(meta, id=cid)
                for p in comp['parts']:
                    self.glb_parts.append([cid, part_tris(p, self.tess) / 1000.0, part_rgba(p)])
            else:
                rec['links'] = place['links']
                rec['visual_rpy'] = place.get('visual_rpy', {})
                self.groups[comp['output']] = [local]
                rec['node_ids'] = [comp['node_ids'][link] for link in place['links']]
                for link in place['links']:
                    nid = comp['node_ids'][link]
                    where = ''
                    if len(place['links']) > 1:
                        where = ' (%s)' % nid[len(cid):].strip('_').replace('_', ' ') if nid.startswith(cid) else ''
                    self.glb_nodes[nid] = dict(meta, id=nid, name=comp['name'] + where, link=link)
                    vis = pose_matrix([0, 0, 0], place.get('visual_rpy', {}).get(link, [0, 0, 0]))
                    m = poses[link] @ vis
                    for p in comp['parts']:
                        self.glb_parts.append([nid, cadlib.transform(part_tris(p, self.tess) / 1000.0, m),
                                               part_rgba(p)])
            self.component_report[cid] = rec
        self.poses = poses

    def check_components(self):
        """Compare the envelope parameters with the STEP instances they were taken from."""
        rows = []
        for cid, comp in self.map['components'].items():
            chk = comp.get('check')
            if not chk:
                continue
            if chk.get('step') == 'hub_motor':
                got = self.hub_local_size
                want = chk['local_size_mm']
                kind = 'local size'
            else:
                units = self.unit_frames(chk['unit'])
                if not units:
                    raise SystemExit('check of %s matches no instance: %s' % (cid, chk['unit']))
                loc, leaves = next(iter(units.values()))
                if 'local_size_mm' in chk:
                    box = cadlib.merge_boxes([cadlib.bbox(lf.shape, cadlib.relative(loc, lf.loc)) for lf in leaves])
                    got, want, kind = box[1] - box[0], chk['local_size_mm'], 'local size'
                else:
                    box = self.unit_box_bl(leaves)
                    got, want, kind = box.reshape(-1), np.array(chk['base_link_box_mm']).reshape(-1), 'base_link box'
            delta = float(np.max(np.abs(np.asarray(got) - np.asarray(want))))
            rows.append({'component': cid, 'kind': kind, 'step': [round(float(v), 2) for v in np.ravel(got)],
                         'map': [float(v) for v in np.ravel(want)], 'max_delta_mm': round(delta, 3)})
            self.component_report[cid]['step_check'] = rows[-1]
            if delta > 0.5:
                raise SystemExit('%s: the STEP instance no longer matches the map (max delta %.2f mm)' % (cid, delta))
        self.component_checks = rows

    def write_meshes(self):
        b = self.map['budgets']
        per_file = (b['stl_max_bytes'] - 84) // 50
        before = {k: int(sum(len(p) for p in v)) for k, v in self.groups.items()}
        total_cap = (b['stl_total_max_bytes'] - 84 * len(before)) // 50
        counts = {k: min(v, per_file) for k, v in before.items()}
        while sum(counts.values()) > total_cap:
            big = max(counts, key=counts.get)
            counts[big] = int(counts[big] * 0.95)
        vis = os.path.join(self.args.out, 'visual')
        os.makedirs(vis, exist_ok=True)
        self.output_report = {}
        for name, out in self.map['outputs'].items():
            parts = decimate_parts(self.groups[name], counts[name])
            tris = np.concatenate(parts) if parts else np.zeros((0, 3, 3))
            path = os.path.join(vis, out['file'])
            write_stl(path, tris)
            self.output_report[name] = {
                'file': 'meshes/visual/' + out['file'], 'sha256': sha256(path), 'bytes': os.path.getsize(path),
                'triangles': int(len(tris)), 'triangles_before_decimation': before[name],
                'frame': out['frame'], 'material': out['material'],
                'rgba': [round(float(c), 3) for c in self.colours[name]]}
            self.log('wrote %s: %d triangles (%d before), %d bytes' % (out['file'], len(tris), before[name],
                                                                       os.path.getsize(path)))
        total = sum(r['bytes'] for r in self.output_report.values())
        if total > b['stl_total_max_bytes']:
            raise SystemExit('visual meshes total %d bytes, over the budget' % total)

    def write_glb(self):
        b = self.map['budgets']
        os.makedirs(self.args.build, exist_ok=True)
        # parts of components with glb_decimate: false keep their triangles and stay out of the size target,
        # so adding them leaves the decimation of every other node unchanged
        fixed = [e for e in self.glb_parts if e[0] in self.glb_fixed]
        dec_parts = [e for e in self.glb_parts if e[0] not in self.glb_fixed]
        cap = sum(len(t) for _, t, _ in dec_parts)
        while True:
            parts = decimate_parts([t for _, t, _ in dec_parts], cap)
            groups = {}
            for (nid, _, rgba), tris in zip(dec_parts, parts):
                groups.setdefault(nid, {}).setdefault(tuple(rgba), []).append(tris)
            dec = [(self.glb_nodes[nid], [(list(c), np.concatenate(v)) for c, v in prims.items()])
                   for nid, prims in sorted(groups.items())]
            size = glb_size(dec)
            if size <= b['glb_target_bytes']:
                break
            cap = int(sum(len(t) for t in parts) * 0.9 * b['glb_target_bytes'] / size)
        for nid, tris, rgba in fixed:
            groups.setdefault(nid, {}).setdefault(tuple(rgba), []).append(tris)
        dec = [(self.glb_nodes[nid], [(list(c), np.concatenate(v)) for c, v in prims.items()])
               for nid, prims in sorted(groups.items())]
        path = os.path.join(self.args.build, 'base_visual.glb')
        extras = {'frame': 'base_link, metres, x forward, y left, z up (the root node rotates to glTF y-up)',
                  'source': 'openamr-platform-sw meshes/PROVENANCE.yaml',
                  'nodes': 'one node per CAD part instance or component envelope; extras: id, name, class '
                           '(chassis, component, envelope), source, bom_row (BOM Issue 7.11), provisional'}
        write_glb(path, dec, extras)
        size = os.path.getsize(path)
        if size > b['glb_max_bytes']:
            raise SystemExit('GLB is %d bytes, over the budget' % size)
        node_rows = []
        for meta, prims in dec:
            row = {k: meta[k] for k in ('id', 'name', 'class', 'source', 'bom_row', 'provisional', 'placeholder') if k in meta}
            row['triangles'] = int(sum(len(t) for _, t in prims))
            node_rows.append(row)
        self.glb_report = {'file': 'build/base_visual.glb (not committed)', 'sha256': sha256(path), 'bytes': size,
                           'triangles': int(sum(r['triangles'] for r in node_rows)),
                           'primitives': int(sum(len(p) for _, p in dec)), 'node_count': len(dec), 'nodes': node_rows}
        self.log('wrote', path, {k: v for k, v in self.glb_report.items() if k != 'nodes'})

    # -- engineering checks
    def hub_motor(self):
        path = self.args.hub_step or os.path.join(self.args.hw_repo, self.map['sources']['hub_motor']['path'])
        if not os.path.exists(path):
            raise SystemExit('hub motor STEP not found: %s (it is on the platform-hw PR #28 branch; pass --hub-step)' % path)
        self.add_input('hub_motor', self.map['sources']['hub_motor'], path)
        hub = cadlib.Assembly(path)
        excl = re.compile(self.map['checks']['hub_motor_fit']['exclude_leaf'])
        # boxes before tessellation: the tight box otherwise follows the triangulation
        allbox = cadlib.merge_boxes([cadlib.bbox(lf.shape, lf.loc) for lf in hub.leaves])
        tris = []
        boxes = []
        for lf in hub.leaves:
            if excl.search(lf.path) or cadlib.is_void(lf.shape):
                continue
            boxes.append(cadlib.bbox(lf.shape, lf.loc))
            tris.append(cadlib.transform(cadlib.tessellate(lf.shape, self.tess['linear_deflection_mm'],
                                                           self.tess['angular_deflection_rad']),
                                         cadlib.trsf_matrix(lf.loc)))
        self.hub_tris = np.concatenate(tris)
        self.hub_local_size = allbox[1] - allbox[0]
        body = cadlib.merge_boxes(boxes)
        self.hub_radius_mm = float(max(body[1][1], -body[0][1], body[1][2], -body[0][2]))
        # tyre centre on the axis: the widest radius section (tyre) centre along X
        tyre = [b for b in boxes if b[1][1] - b[0][1] > 199.0]
        self.hub_tyre_x = float((tyre[0][0][0] + tyre[0][1][0]) / 2)
        self.hub_tyre_width = float(tyre[0][1][0] - tyre[0][0][0])

    def hub_placement_step(self, link, brake_inboard=True):
        """4 x 4 (mm) from the hub motor STEP frame to the assembly STEP frame."""
        local = np.eye(4)
        # hub X (axis, brake side +X) -> link -y; hub Y -> link x; hub Z -> link z
        local[:3, :3] = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], float)
        local[:3, 3] = [0.0, self.hub_tyre_x, 0.0]
        if not brake_inboard:
            local = pose_matrix([0, 0, 0], [0, 0, math.pi]) @ local
        pose = self.poses[link].copy()
        pose[:3, 3] *= 1000.0
        vis = self.map['components']['hub_motor_wheel']['placement'].get('visual_rpy', {}).get(link, [0, 0, 0])
        bl = pose @ pose_matrix([0, 0, 0], vis) @ local
        inv = np.linalg.inv(self.Tmm)
        return inv @ bl

    def run_checks(self):
        c = self.map['checks']
        res = []

        def row(group, item, measured, design, unit='mm', note=''):
            d = None if design is None or measured is None else round(float(measured) - float(design), 2)
            drift = d is not None and abs(d) > 2.0 and unit == 'mm' or (d is not None and unit == 'deg' and abs(d) > 0.5)
            res.append({'group': group, 'item': item, 'measured': None if measured is None else round(float(measured), 2),
                        'design': None if design is None else round(float(design), 2), 'delta': d, 'unit': unit, 'drift': bool(drift), 'note': note})

        # chassis
        fb = self.unit_box_bl(self.asm.find(c['panel_faces_mm']['front']))
        bb = self.unit_box_bl(self.asm.find(c['panel_faces_mm']['back']))
        row('chassis', 'front panel outer face x', fb[1][0], c['panel_faces_mm']['design'])
        row('chassis', 'back panel outer face x', bb[0][0], -c['panel_faces_mm']['design'])
        sb = self.unit_box_bl(self.asm.find(c['chassis_width_mm']['side']))
        row('chassis', 'width over the side panels', sb[1][1] - sb[0][1], c['chassis_width_mm']['design'])
        for corr in self.corrections:
            if 'origin_was_mm' in corr:
                was, now = np.array(corr['origin_was_mm']), np.array(corr['origin_set_mm'])
                row('chassis', 'base plate placement in the STEP (corrected), offset |d|', np.linalg.norm(was - now), 0.0,
                    note='origin was %s mm, set to %s mm (STEP axes)' % (corr['origin_was_mm'], corr['origin_set_mm']))
        row('chassis', 'release 001 axle height above the castor contact', self.frame_report['axle_height_above_castor_floor_mm'],
            self.map['frame']['base_link_height_mm'])
        row('chassis', 'release 001 track (not the 2.0 value)', self.frame_report['release001_track_mm'], 400.0,
            note='DRIVE-TRACK open; 0.400 m is the placeholder')

        # wheel radius
        row('hub motor', 'wheel radius (ZLLG80ASM250-L-B2 STEP)', self.hub_radius_mm, c['wheel_radius_mm']['design'])
        row('hub motor', 'tyre width (ZLLG80ASM250-L-B2 STEP)', self.hub_tyre_width, None)
        row('hub motor', 'tyre bottom below base_link at radius 100 (base_link 98 mm above the floor)',
            self.hub_radius_mm - self.map['frame']['base_link_height_mm'], 0.0,
            note='positive = the unloaded tyre reaches below the floor plane')

        # hub motor fit, and the base_link envelopes against the chassis; mesh distance in base_link mm
        fit = []
        min_clear = c['hub_motor_fit']['min_clearance_mm']

        def clearances(subject, tris, tag):
            self.log('fit check:', subject, tag)
            for lf, ptris in self.chassis_mm:
                d = cadlib.mesh_distance(tris, ptris)
                if d is not None:
                    touch = 'collision' if subject == 'hub motor wheel' else 'contact or overlap'
                    fit.append(dict(tag, subject=subject, part=lf.names[-1], clearance_mm=round(d, 2),
                                    verdict=touch if d < 0.5 else (
                                        'under %.0f mm' % min_clear if d < min_clear else 'ok')))

        for link in ('left_wheel_link', 'right_wheel_link'):
            for inboard in (True, False):
                m = self.Tmm @ self.hub_placement_step(link, inboard)
                clearances('hub motor wheel', cadlib.transform(self.hub_tris, m),
                           {'link': link, 'brake': 'inboard' if inboard else 'outboard'})
        for cid, comp in self.map['components'].items():
            if comp.get('status') != 'modelled' or not comp['placement'].get('base_link'):
                continue
            tris = np.concatenate([part_tris(p, self.tess) for p in comp['parts']])
            clearances(cid, tris, {'link': 'base_link', 'brake': ''})
        self.fit = fit
        bat = self.map['components']['battery']['parts'][0]
        top = bat['xyz_mm'][2] + bat['size_mm'][2] / 2 + self.map['frame']['base_link_height_mm']
        row('equipment', 'battery cell-group envelope top above the floor (provisional, 210 mm high on the base plate)', top, None,
            note='at most the steel deck top 294 mm (DATUM-HEIGHT-STACK); clearances in the battery section below')
        self.battery_checks()

        # castors
        for key, (loc, leaves) in sorted(self.unit_frames(c['castors']['unit']).items()):
            m = self.Tmm @ cadlib.trsf_matrix(loc)
            o = m[:3, 3]
            box = self.unit_box_bl(leaves)
            name = ('f' if o[0] > 0 else 'b') + ('l' if o[1] > 0 else 'r') + '_caster_link'
            urdf = np.array(self.map['links'][name]['xyz']) * 1000.0
            row('castors', '%s x (top plate centre)' % name, o[0], math.copysign(c['castors']['design_xy_mm'][0], o[0]),
                note='URDF joint x %.2f' % urdf[0])
            row('castors', '%s y' % name, o[1], math.copysign(c['castors']['design_xy_mm'][1], o[1]),
                note='URDF joint y %.2f' % urdf[1])
            row('castors', '%s contact z' % name, box[0][2], -self.map['frame']['base_link_height_mm'])

        # LiDAR
        sp = self.unit_box_bl(self.asm.find(c['lidar']['spacer']))
        plane = sp[1][2] + c['lidar']['s3_base_to_scan_plane_mm'] + self.map['frame']['base_link_height_mm']
        row('mounts', 'LiDAR scan plane above the floor (spacer top + 30.55)', plane, c['lidar']['design_scan_plane_mm'])
        s3 = next(iter(self.unit_frames(c['lidar']['s3']).values()))[1]
        s3b = self.unit_box_bl(s3)
        row('mounts', 'RPLIDAR S3 centre x in the STEP', (s3b[0][0] + s3b[1][0]) / 2, self.poses['lidar_link'][0, 3] * 1000,
            note='lidar_link x kept at 0.330 (NAV-LIDAR)')
        row('mounts', 'RPLIDAR S3 base above the spacer top', s3b[0][2] - sp[1][2], 0.0)

        # Gemini
        loc, leaves = next(iter(self.unit_frames(c['gemini']['unit']).values()))
        g = self.Tmm @ cadlib.trsf_matrix(loc)
        dz = g[:3, :3][:, 2]  # optical axis = model local z
        des = c['gemini']['design_xyz_mm']
        for i, ax in enumerate('xyz'):
            row('mounts', 'Gemini 336L model origin (front glass) %s' % ax, g[i, 3], des[i])
        row('mounts', 'Gemini 336L up-tilt', math.degrees(math.asin(dz[2])), c['gemini']['design_pitch_deg'], unit='deg')
        mb = self.unit_box_bl(self.asm.find(c['gemini']['mount']))
        mc = (mb[0] + mb[1]) / 2
        row('mounts', 'camera mount bounding-box centre y', mc[1], des[1])
        row('mounts', 'camera mount bounding-box centre z', mc[2], des[2],
            note='box of the whole printed mount; the housing centre is the register value')

        # panel holes
        def holes(pattern, dmin, dmax):
            return self.panel_holes(pattern, dmin, dmax)

        us = c['ultrasonic_holes']
        for lf, d, q in sorted(holes(us['panels'], *us['diameter_range_mm']), key=lambda h: (-h[2][0], -h[2][1])):
            pb = self.unit_box_bl([lf])
            face = pb[1][0] if pb[1][0] > 0 else pb[0][0]
            tag = ('front ' if face > 0 else 'rear ') + ('left' if q[1] > 0 else 'right')
            row('mounts', 'ultrasonic hole %s, centre x (panel face)' % tag, face, math.copysign(us['design_x_mm'], face))
            row('mounts', 'ultrasonic hole %s, centre y' % tag, q[1], math.copysign(us['design_y_mm'], q[1]))
            row('mounts', 'ultrasonic hole %s, centre z' % tag, q[2], us['design_z_mm'])
            row('mounts', 'ultrasonic hole %s, diameter' % tag, d, us['design_diameter_mm'])
        tf = c['tof']
        for side in (1, -1):
            hs = [h for h in holes(tf['panels'], *tf['hole_diameter_range_mm'])
                  if tf['hole_y_range_mm'][0] <= h[2][1] <= tf['hole_y_range_mm'][1]
                  and (self.unit_box_bl([h[0]])[1][0] > 0) == (side > 0)]
            tag = 'front' if side > 0 else 'rear'
            if not hs:
                row('mounts', 'ToF %s centre hole' % tag, None, tf['design_hole_diameter_mm'], note='no hole found')
                continue
            pb = self.unit_box_bl([hs[0][0]])
            face = pb[1][0] if side > 0 else pb[0][0]
            cy = float(np.mean([h[2][1] for h in hs]))
            cz = float(np.mean([h[2][2] for h in hs]))
            desc = ', '.join('%.1f mm at y %.1f z %.1f' % (h[1], h[2][1], h[2][2]) for h in hs)
            row('mounts', 'ToF %s opening centre x (panel face)' % tag, face, side * tf['design_hole_x_mm'])
            row('mounts', 'ToF %s opening centre y' % tag, cy, 0.0, note='holes: ' + desc)
            row('mounts', 'ToF %s opening centre z' % tag, cz, tf['design_z_mm'])
            row('mounts', 'ToF %s largest opening diameter' % tag, max(h[1] for h in hs), tf['design_hole_diameter_mm'])
        for key, (loc, leaves) in sorted(self.unit_frames(tf['unit']).items()):
            m = self.Tmm @ cadlib.trsf_matrix(loc)
            box = cadlib.merge_boxes([cadlib.bbox(lf.shape, cadlib.relative(loc, lf.loc)) for lf in leaves])
            out = math.copysign(1.0, m[0, 3])
            dots = [out * m[0, k] for k in range(3)]
            k = int(np.argmax(np.abs(dots)))
            sgn = math.copysign(1.0, dots[k])
            centre = (box[0] + box[1]) / 2
            centre[k] = box[1][k] if sgn > 0 else box[0][k]
            facept = m[:3, :3] @ centre + m[:3, 3]
            beam = m[:3, :3][:, k] * sgn
            tag = 'front' if out > 0 else 'rear'
            row('mounts', 'TF-Luna %s lens face centre x' % tag, facept[0], out * tf['design_lens_x_mm'])
            row('mounts', 'TF-Luna %s lens face centre y' % tag, facept[1], 0.0)
            row('mounts', 'TF-Luna %s lens face centre z' % tag, facept[2], tf['design_z_mm'])
            row('mounts', 'TF-Luna %s pitch down' % tag, math.degrees(math.asin(-beam[2])), tf['design_pitch_deg'], unit='deg')

        # IMU
        u = self.unit_box_bl(next(iter(self.unit_frames(c['imu']['unit']).values()))[1])
        uc = (u[0] + u[1]) / 2
        for i, ax in enumerate('xyz'):
            row('mounts', 'ICM-42688-P package centre %s (on the click board in the STEP)' % ax, uc[i],
                c['imu']['design_xyz_mm'][i])
        self.checks = res
        self.panel_button_checks()

    def panel_holes(self, pattern, dmin, dmax):
        """[(leaf, diameter mm, axis point in base_link mm)] of the cylindrical faces along x of the matching panels."""
        out = []
        for lf in self.asm.find(pattern):
            shape = lf.shape.Moved(lf.loc)
            exp = TopExp_Explorer(shape, TopAbs_FACE)
            seen = set()
            while exp.More():
                s = BRepAdaptor_Surface(TopoDS.Face(exp.Current()))
                exp.Next()
                if s.GetType() != GeomAbs_Cylinder:
                    continue
                cyl = s.Cylinder()
                d = 2 * cyl.Radius()
                ax = cyl.Axis().Direction()
                p = cyl.Axis().Location()
                if not (dmin <= d <= dmax) or abs(ax.X()) < 0.99:
                    continue
                q = self.to_base_link_mm(np.array([[p.X(), p.Y(), p.Z()]]))[0]
                k = (round(d, 2), round(q[1], 2), round(q[2], 2))
                if k not in seen:
                    seen.add(k)
                    out.append((lf, d, q))
        return out

    # -- panel components centred on the axis of a release 001 panel hole
    def place_panel_parts(self):
        """Centre each panel-hole component on the axis of the hole of the release 001 part it replaces.

        The hole is the cylindrical face of the panel (axis along x, diameter in range) nearest the
        centre of the release 001 instance; parts are given relative to the hole axis at the panel
        outer face (xyz_rel_mm, x outward). The release 001 instance itself stays excluded."""
        self.panel_place = {}
        for cid, comp in self.map['components'].items():
            rule = comp.get('placement_rule', {})
            if comp.get('status') != 'modelled' or rule.get('mode') != 'panel_hole':
                continue
            rx = re.compile(rule['instance'])
            inst = [lf for lf in self.asm.leaves if rx.search(lf.path)]
            if len(inst) != 1:
                raise SystemExit('%s: expected one release 001 instance, found %d (%s)' % (cid, len(inst), rule['instance']))
            ib = self.unit_box_bl(inst)
            ic = (ib[0] + ib[1]) / 2
            near = [(lf, d, q) for lf, d, q in self.panel_holes(rule['panel'], *rule['diameter_range_mm'])
                    if np.hypot(q[1] - ic[1], q[2] - ic[2]) <= rule['search_radius_mm']]
            if not near:
                raise SystemExit('%s: no panel hole within %.1f mm of the release 001 instance' % (cid, rule['search_radius_mm']))
            lf, d, q = min(near, key=lambda h: (np.hypot(h[2][1] - ic[1], h[2][2] - ic[2]), -h[1]))
            pb = self.unit_box_bl([lf])
            face = float(pb[1][0] if pb[1][0] > 0 else pb[0][0])
            out = 1.0 if face > 0 else -1.0
            axis = np.array([face, float(q[1]), float(q[2])])
            xs = []
            for p in comp['parts']:
                rel = np.array(p['xyz_rel_mm'], float)
                p['xyz_mm'] = [round(float(v), 3) for v in axis + np.array([out * rel[0], rel[1], rel[2]])]
                xs.append(p['xyz_mm'])
            self.panel_place[cid] = {
                'release001_instance': inst[0].names[-1], 'panel': lf.names[-1],
                'release001_instance_centre_mm': [round(float(v), 2) for v in ic],
                'hole_axis_mm': {'x_panel_outer_face': round(face, 2), 'y': round(float(q[1]), 2), 'z': round(float(q[2]), 2)},
                'hole_axis_z_floor_mm': round(float(q[2]) + self.map['frame']['base_link_height_mm'], 2),
                'old_hole_diameter_mm': round(float(d), 3),
                'parts_x_mm': [round(float(min(c[0] - part_half_x(p) for c, p in zip(xs, comp['parts']))), 2),
                               round(float(max(c[0] + part_half_x(p) for c, p in zip(xs, comp['parts']))), 2)]}
            self.log('%s: hole of %s on %s, axis y %.2f z %.2f, diameter %.3f mm' % (cid, inst[0].names[-1], lf.names[-1],
                                                                                   q[1], q[2], d))

    def panel_button_checks(self):
        """Old hole diameter against the required mounting hole, and the expected side of each button."""
        self.panel_report = {}
        for cid, rec in self.panel_place.items():
            rule = self.map['components'][cid]['placement_rule']
            req, (tmin, tplus) = rule['required_hole_mm'], rule['required_tolerance_mm']
            d = rec['old_hole_diameter_mm']
            y = rec['hole_axis_mm']['y']
            side = 1 if y > 0 else -1
            ok_side = side == rule['expected_side']
            in_band = req - tmin - 1e-9 <= d <= req + tplus + 1e-9
            r = dict(rec, required_mounting_hole_mm=req, required_tolerance_mm=[tmin, tplus],
                     required_source=' '.join(rule['required_source'].split()),
                     hole_change_mm=round(req - d, 3),
                     hole_within_required_band=bool(in_band),
                     hole_action=('keep: the existing %.2f mm hole is within %.1f +%.1f/-%.1f mm; confirm the tolerance '
                                  'against the Schneider datasheet' % (d, req, tplus, tmin)) if in_band else
                                 ('resize the hole from %.2f mm to %.1f mm (+%.1f/-%.1f)' % (d, req, tplus, tmin)),
                     expected_side='+y (robot left)' if rule['expected_side'] > 0 else '-y (robot right)',
                     measured_side='+y (robot left)' if side > 0 else '-y (robot right)',
                     side_ok=bool(ok_side))
            if not ok_side:
                msg = '%s: the release 001 hole is on %s, P-03 rev20.0 item 24 expects %s; not moved' % (
                    cid, r['measured_side'], r['expected_side'])
                self.findings.append(msg)
                r['finding'] = msg
            self.panel_report[cid] = r
            self.log('panel button check:', cid, {k: r[k] for k in ('old_hole_diameter_mm', 'required_mounting_hole_mm',
                                                                     'hole_action', 'side_ok')})

    # -- battery cell-group envelope: front face behind the centre bracket, clearances, free space
    def battery_points(self, sample, margin):
        """{GLB node id: sample points (mm, base_link)} of the chassis parts around the rear deck."""
        key = (sample, margin)
        self._bat_pts = getattr(self, '_bat_pts', {})
        if key in self._bat_pts:
            return self._bat_pts[key]
        comp = self.map['components']['battery']
        rule = comp['placement_rule']
        size = np.array(comp['parts'][0]['size_mm'], float)
        deck = float(rule['stand_z_mm'])
        region = (np.array([-420.0, -size[1] / 2 - margin, deck + 0.1]),
                  np.array([-60.0, size[1] / 2 + margin, deck + size[2] + margin]))
        pts = self._bat_pts[key] = {}
        for lf, tris in self.chassis_mm:
            q = region_points(tris, region[0], region[1], sample)
            if len(q):
                nid = self.chassis_ids[lf.path]
                pts[nid] = np.concatenate([pts[nid], q]) if nid in pts else q
        return pts

    def place_battery(self):
        """Front face of the cell-group envelope the clearance behind the rear face of the reference part.

        The reference face is the rear-most point of that part within the envelope's y and z extent.
        Writes the x into the battery part of the map; the module is not moved elsewhere."""
        comp = self.map['components']['battery']
        rule = comp['placement_rule']
        size = np.array(comp['parts'][0]['size_mm'], float)
        deck = float(rule['stand_z_mm'])
        pts = self.battery_points(rule['sample_mm'], 70.0)[rule['front_reference']]
        band = (np.abs(pts[:, 1]) <= size[1] / 2) & (pts[:, 2] > deck + 0.1) & (pts[:, 2] < deck + size[2])
        ref = float(pts[band][:, 0].min())
        x1 = round(float(ref - rule['front_clearance_mm']), 2)
        x0 = round(float(x1 - size[0]), 2)
        centre = [round((x0 + x1) / 2, 3), 0.0, round(deck + size[2] / 2, 3)]
        comp['parts'][0]['xyz_mm'] = centre
        # parts given relative to the envelope centre (cells and terminals) follow the envelope
        for p in comp['parts'][1:]:
            if 'xyz_rel_mm' in p:
                p['xyz_mm'] = [round(c + float(r), 3) for c, r in zip(centre, p['xyz_rel_mm'])]
        self.battery_cells = []
        for p in comp['parts']:
            if p.get('feature') == 'cell':
                half = np.array(p['size_mm'], float) / 2
                c = np.array(p['xyz_mm'], float)
                self.battery_cells.append({'x': [round(float(c[0] - half[0]), 2), round(float(c[0] + half[0]), 2)],
                                           'y': [round(float(c[1] - half[1]), 2), round(float(c[1] + half[1]), 2)],
                                           'z': [round(float(c[2] - half[2]), 2), round(float(c[2] + half[2]), 2)]})
        self.battery_place = {'reference_rear_face_x_mm': round(ref, 2), 'front_face_x_mm': x1, 'rear_face_x_mm': x0}
        self.log('battery cell-group envelope: %s rear face x %.2f, front face x %.2f, rear face x %.2f mm'
                 % (rule['front_reference'], ref, x1, x0))

    def battery_checks(self):
        """Clearances of the cell-group envelope and the free space behind and beside it."""
        comp = self.map['components']['battery']
        rule = comp['placement_rule']
        size = np.array(comp['parts'][0]['size_mm'], float)
        deck = float(rule['stand_z_mm'])
        g = rule['front_clearance_mm']
        x0, x1 = self.battery_place['rear_face_x_mm'], self.battery_place['front_face_x_mm']
        lo = np.array([x0, -size[1] / 2, deck])
        hi = np.array([x1, size[1] / 2, deck + size[2]])
        fine = self.battery_points(rule['sample_mm'], 70.0)
        rows = []
        for r in rule['clearance_report']:
            q = np.concatenate([fine[n] for n in r['nodes'] if n in fine]) if any(n in fine for n in r['nodes']) \
                else np.zeros((0, 3))
            if r.get('above_module'):    # the part of it right above the envelope footprint
                q = q[(q[:, 0] >= lo[0]) & (q[:, 0] <= hi[0]) & (np.abs(q[:, 1]) <= hi[1]) & (q[:, 2] >= hi[2])]
            if 'x_below_mm' in r:
                q = q[q[:, 0] < r['x_below_mm']]
            if r.get('above_deck'):      # the deck the envelope stands on is contact, not clearance
                q = q[q[:, 2] > deck + 0.1]
            d, axis = points_box_clearance(lo, hi, q)
            rows.append({'subject': r['name'], 'parts': r['nodes'], 'clearance_mm': d, 'axis': axis,
                         'verdict': 'no geometry' if d is None else (
                             'overlap' if d < 0 else ('under %.0f mm' % g if d < g else 'ok'))})
        by_part = []
        for nid, q in sorted(fine.items()):
            q = q[q[:, 2] > deck + 0.1]
            q = q[np.all((q > lo - 60.0) & (q < hi + 60.0), axis=1)]
            d, axis = points_box_clearance(lo, hi, q)
            if d is not None and d <= 60.0:
                by_part.append({'part': nid, 'clearance_mm': d, 'axis': axis})
        by_part.sort(key=lambda r: (r['clearance_mm'], r['part']))
        smallest = by_part[0] if by_part else None
        fits = all(r['clearance_mm'] is None or r['clearance_mm'] >= g for r in rows) and \
            (smallest is None or smallest['clearance_mm'] >= g)

        # free space behind (towards the rear panel) and beside the envelope; nothing is on top of the cells
        obst = list(self.battery_points(2.0, 240.0).values())
        for cid, c in self.map['components'].items():
            if cid != 'battery' and c.get('status') == 'modelled' and c['placement'].get('base_link'):
                tris = np.concatenate([part_tris(q, self.tess) for q in c['parts']])
                obst.append(region_points(tris, np.array([-1e4, -1e4, deck + 0.1]), np.array([1e4, 1e4, 1e4]), 2.0))
        allp = np.concatenate(obst)

        def first(box_lo, box_hi, axis, sign):
            q = allp[np.all((allp > box_lo) & (allp < box_hi), axis=1)]
            if not len(q):
                return None
            return float(q[:, axis].min() if sign > 0 else q[:, axis].max())

        def space(name, x_rng, y_rng, grow_axis, sign):
            """From the envelope face along one axis to the first obstacle, then up to the first obstacle."""
            blo = np.array([x_rng[0], y_rng[0], deck + 0.1])
            bhi = np.array([x_rng[1], y_rng[1], deck + size[2]])
            if sign > 0:
                bhi[grow_axis] = blo[grow_axis] + 1e3
            else:
                blo[grow_axis] = bhi[grow_axis] - 1e3
            f = first(blo, bhi, grow_axis, sign)
            if f is not None:
                if sign > 0:
                    bhi[grow_axis] = f - g
                else:
                    blo[grow_axis] = f + g
            if bhi[grow_axis] - blo[grow_axis] <= 0:
                return {'name': name, 'free': False, 'note': 'no free space with %.0f mm clearance' % g}
            top = first(np.array([blo[0], blo[1], deck + 0.1]), np.array([bhi[0], bhi[1], 1e4]), 2, 1)
            bhi[2] = (top - g) if top is not None else deck + size[2]
            r = lambda v: [round(float(a), 1) for a in v]  # noqa: E731
            return {'name': name, 'free': True, 'x_mm': r([blo[0], bhi[0]]), 'y_mm': r([blo[1], bhi[1]]),
                    'z_base_link_mm': r([deck, bhi[2]]),
                    'size_mm': {'x': round(float(bhi[0] - blo[0]), 1), 'y': round(float(bhi[1] - blo[1]), 1),
                                'z': round(float(bhi[2] - deck), 1)}}
        free = [space('behind the envelope (-x, towards the rear panel): BMS and power distribution zone (not modelled)',
                      (lo[0] - g - 1.0, lo[0] - g), (lo[1], hi[1]), 0, -1),
                space('left of the envelope (+y)', (lo[0], hi[0]), (hi[1] + g, hi[1] + g + 1.0), 1, 1),
                space('right of the envelope (-y)', (lo[0], hi[0]), (lo[1] - g - 1.0, lo[1] - g), 1, -1)]
        bl = self.map['frame']['base_link_height_mm']
        self.battery_report = {
            'envelope': 'cell-group envelope, provisional (not the complete battery assembly; dimensions not released)',
            'envelope_mm': {'x': float(size[0]), 'y': float(size[1]), 'z': float(size[2]),
                            'derivation': ' '.join(rule['derivation'].split())},
            'rule': 'front face %.1f mm behind the rear face of %s (its rear-most point within the envelope y and z '
                    'extent), centred in y, standing on the deck; %.1f mm surface sampling; the envelope is not moved '
                    'when a clearance is under %.1f mm' % (g, rule['front_reference'], rule['sample_mm'], g),
            'reference_rear_face_x_mm': self.battery_place['reference_rear_face_x_mm'],
            'front_face_x_mm': x1, 'rear_face_x_mm': x0, 'fits_with_clearance': fits,
            'box_mm': {'x': [float(lo[0]), float(hi[0])], 'y': [float(lo[1]), float(hi[1])],
                       'z_base_link': [float(lo[2]), float(hi[2])], 'z_floor': [float(lo[2]) + bl, float(hi[2]) + bl]},
            'cells_mm': {'note': 'the eight cell boxes drawn inside the envelope (base_link, mm); terminal positions '
                                 'are schematic', 'boxes': self.battery_cells},
            'smallest_clearance': smallest,
            'clearances': rows,
            'clearances_by_part': by_part,
            'free_space': free,
        }
        for r in rows:
            self.log('battery clearance:', r)
        self.log('battery smallest clearance:', smallest, 'fits:', fits)
        for f in free:
            self.log('battery free space:', f)

    # -- reports
    def write_reports(self):
        os.makedirs(self.args.build, exist_ok=True)
        lines = ['# CAD versus design checks', '',
                 'Generated by tools/cad/step_to_description_meshes.py. Reported only; no frame or register value '
                 'is changed. Lengths in mm in base_link (x forward, y left, z up) unless stated; drift = |delta| over '
                 '2 mm (0.5 deg for angles).', '',
                 '| Group | Item | Measured | Design | Delta | Drift | Note |', '|---|---|---|---|---|---|---|']
        for r in self.checks:
            fmt = lambda v: '' if v is None else ('%.2f' % v if isinstance(v, float) else str(v))  # noqa: E731
            lines.append('| %s | %s | %s | %s | %s | %s | %s |' % (
                r['group'], r['item'], fmt(r['measured']), fmt(r['design']),
                fmt(r['delta']) + (' deg' if r['unit'] == 'deg' and r['delta'] is not None else ''),
                'drift' if r['drift'] else '', r['note']))
        lines += ['', '## Base E-stop and RESET in the release 001 button holes (P-03 rev20.0 item 24)', '',
                  'Each 2.0 part is centred on the axis of the front-panel hole of the release 001 button it replaces; '
                  'the hole axis is read from the panel (cylindrical face along x nearest the release 001 instance). '
                  'Geometry is a placeholder from datasheet dimensions (Schneider STEP not read).', '',
                  '| Component | Release 001 button | Hole axis y, z mm (z above floor) | Old hole mm | Required mm | '
                  'Hole action | Side measured (expected) | Side ok | Parts x mm (panel outer face) |',
                  '|---|---|---|---|---|---|---|---|---|']
        for cid, r in self.panel_report.items():
            lines.append('| %s | %s | %.2f, %.2f (%.2f) | %.3f | %.1f +%.1f/-%.1f | %s | %s (%s) | %s | %.2f to %.2f (%.2f) |' % (
                cid, r['release001_instance'], r['hole_axis_mm']['y'], r['hole_axis_mm']['z'], r['hole_axis_z_floor_mm'],
                r['old_hole_diameter_mm'], r['required_mounting_hole_mm'], r['required_tolerance_mm'][1],
                r['required_tolerance_mm'][0], r['hole_action'], r['measured_side'], r['expected_side'],
                'yes' if r['side_ok'] else 'NO (finding)', r['parts_x_mm'][0], r['parts_x_mm'][1],
                r['hole_axis_mm']['x_panel_outer_face']))
        if self.findings:
            lines += ['', 'Findings:', ''] + ['- ' + f for f in self.findings]
        br = self.battery_report
        fmt2 = lambda v: '' if v is None else '%.2f' % v  # noqa: E731
        sm = br['smallest_clearance'] or {}
        lines += ['', '## Battery: %s' % br['envelope'], '',
                  'Envelope %.0f (x) x %.0f (y) x %.0f (z) mm: %s.' % (
                      br['envelope_mm']['x'], br['envelope_mm']['y'], br['envelope_mm']['z'], br['envelope_mm']['derivation']),
                  '', 'Placement: %s.' % br['rule'], '',
                  'Reference rear face x %.2f mm; front face x %.2f mm; rear face x %.2f mm; box y %s, z %s mm in base_link '
                  '(floor z %s mm). Fits with %.0f mm clearance: %s. Smallest clearance %s mm to %s (%s). '
                  'Negative clearance = overlap, depth along the axis given.'
                  % (br['reference_rear_face_x_mm'], br['front_face_x_mm'], br['rear_face_x_mm'], br['box_mm']['y'],
                     br['box_mm']['z_base_link'], br['box_mm']['z_floor'], 2.0, 'yes' if br['fits_with_clearance'] else 'no',
                     fmt2(sm.get('clearance_mm')), sm.get('part', ''), sm.get('axis', '')),
                  '', '| Subject | Parts | Clearance mm | Axis | Verdict |', '|---|---|---|---|---|']
        for r in br['clearances']:
            lines.append('| %s | %s | %s | %s | %s |' % (r['subject'], ', '.join(r['parts']), fmt2(r['clearance_mm']),
                                                        r['axis'] or '', r['verdict']))
        lines += ['', 'Every chassis part within 60 mm (GLB node id):', '', '| Part | Clearance mm | Axis |', '|---|---|---|']
        for r in br['clearances_by_part']:
            lines.append('| %s | %.2f | %s |' % (r['part'], r['clearance_mm'], r['axis']))
        lines += ['', 'Free space for the BMS, power distribution board, pack fuse, terminals and wiring (not modelled; '
                  'never on top of the cells), with the same clearance to the envelope and the parts around it; the '
                  'height is the first part above the box footprint:', '',
                  '| Space | x mm | y mm | z mm (base_link) | Size x, y, z mm |', '|---|---|---|---|---|']
        for f in br['free_space']:
            if f['free']:
                lines.append('| %s | %s | %s | %s | %.1f, %.1f, %.1f |' % (f['name'], f['x_mm'], f['y_mm'], f['z_base_link_mm'],
                                                                     f['size_mm']['x'], f['size_mm']['y'], f['size_mm']['z']))
            else:
                lines.append('| %s | | | | %s |' % (f['name'], f['note']))
        lines += ['', '## Fit against the chassis parts (hub motor from the ZLLG80ASM250-L-B2 STEP at track 0.400; '
                  'base_link envelopes)', '',
                  '| Subject | Link | Brake side | Chassis part | Clearance mm | Verdict |', '|---|---|---|---|---|---|']
        for f in sorted(self.fit, key=lambda f: (f['subject'] != 'hub motor wheel', f['brake'] != 'inboard',
                                                  f['link'], f['subject'], f['clearance_mm'])):
            lines.append('| %s | %s | %s | %s | %.2f | %s |' % (f['subject'], f['link'], f['brake'], f['part'],
                                                             f['clearance_mm'], f['verdict']))
        lines += ['', 'Mesh distance on the 0.5 mm tessellation: the hub motor or envelope surface sampled at 2 mm against the '
                  'chassis triangles (at most about 1 mm high); under 0.5 mm is a collision for the hub motor and '
                  '"contact or overlap" for the envelopes placed as in the CAD (they sit on their mounts). '
                  'Parts further than 10 mm are not listed.', '',
                  '## Envelope parameters against the STEP instances', '',
                  '| Component | Compared | Max delta mm |', '|---|---|---|']
        for r in self.component_checks:
            lines.append('| %s | %s | %.3f |' % (r['component'], r['kind'], r['max_delta_mm']))
        with open(os.path.join(self.args.build, 'cad_checks.md'), 'w', encoding='utf-8') as stream:
            stream.write('\n'.join(lines) + '\n')
        with open(os.path.join(self.args.build, 'cad_checks.yaml'), 'w', encoding='utf-8') as stream:
            yaml.safe_dump({'checks': self.checks, 'hub_motor_fit': self.fit, 'frame': self.frame_report,
                            'component_checks': self.component_checks, 'battery': self.battery_report,
                            'panel_buttons': self.panel_report, 'findings': self.findings},
                           stream, sort_keys=False, allow_unicode=True)

    def write_provenance(self):
        groups = []
        by_rule = {}
        for lf in self.asm.leaves:
            name = lf.rule['name']
            g = by_rule.get(name)
            if g is None:
                g = by_rule[name] = {'group': name, 'class': lf.cls, 'instances': []}
                if 'output' in lf.rule:
                    g['output'] = lf.rule['output']
                if 'component' in lf.rule:
                    g['replaced_by_envelope'] = lf.rule['component']
                if 'reason' in lf.rule:
                    g['reason'] = lf.rule['reason']
                groups.append(g)
            g['instances'].append(lf.path)
        for g in groups:
            g['count'] = len(g['instances'])
            if g['class'] == 'fastener_small':
                del g['instances']
        doc = {
            'generated_by': {
                'tool': 'tools/cad/step_to_description_meshes.py',
                'command': 'python3 tools/cad/step_to_description_meshes.py --hw-repo <openamr-platform-hw checkout> --hub-step <B2 STEP>',
                'map': {'file': 'tools/cad/base_mesh_map.yaml', 'sha256': sha256(self.args.map)},
                'converter_sha256': sha256(os.path.abspath(__file__)),
                'cadlib_sha256': sha256(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cadlib.py')),
                'versions': {'python': platform.python_version(), 'cadquery-ocp': version('cadquery-ocp'),
                             'trimesh': version('trimesh'), 'fast-simplification': version('fast-simplification'),
                             'numpy': version('numpy'), 'pyyaml': version('pyyaml')},
            },
            'licence': {
                'chassis_meshes': 'CERN-OHL-P-2.0, derived from openAMRobot/openamr-platform-hw CAD (MMP.* parts)',
                'envelopes': 'MIT, generated boxes, cylinders and revolved profiles; no vendor geometry is redistributed',
            },
            'inputs': self.inputs,
            'parameters': {'tessellation': self.tess, 'min_part_size_mm': self.map['min_part_size_mm'],
                           'budgets': self.map['budgets'],
                           'decimation': 'fast-simplification quadric decimation to the per-file and total budgets'},
            'base_link_frame': self.frame_report,
            'corrections': self.corrections,
            'outputs': self.output_report,
            'glb': self.glb_report,
            'components': dict(self.component_report, battery=dict(self.component_report['battery'],
                                                                     rear_deck_check=self.battery_report),
                               **{cid: dict(self.component_report[cid], panel_hole=r)
                                  for cid, r in self.panel_report.items()}),
            'groups': groups,
        }
        path = os.path.join(self.args.out, 'PROVENANCE.yaml')
        with open(path, 'w', encoding='utf-8') as stream:
            stream.write('# Generated by tools/cad/step_to_description_meshes.py; do not edit by hand.\n')
            yaml.safe_dump(doc, stream, sort_keys=False, allow_unicode=True, width=110)
        self.log('wrote', path)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--hw-repo', required=True, help='checkout of openAMRobot/openamr-platform-hw')
    parser.add_argument('--hub-step', help='ZLLG80ASM250-L-B2 double-shaft STEP, if not in the checkout')
    parser.add_argument('--hub-ref', default='refs/pull/28/head', help='git ref the hub STEP was read from')
    parser.add_argument('--hub-commit', help='commit the hub STEP was read from')
    parser.add_argument('--map', default=os.path.join(REPO, 'tools', 'cad', 'base_mesh_map.yaml'))
    parser.add_argument('--out', default=os.path.join(PKG, 'meshes'))
    parser.add_argument('--build', default=os.path.join(REPO, 'build'))
    parser.add_argument('--scratch', help='directory outside the repository for the unzipped STEP')
    args = parser.parse_args()
    if args.hub_step and not args.hub_commit:
        parser.error('--hub-commit is required with --hub-step')
    conv = Converter(args)
    conv.open_assembly()
    conv.apply_corrections()
    conv.classify()
    conv.build_frame()
    conv.hub_motor()
    conv.build()
    conv.check_components()
    conv.write_meshes()
    conv.write_glb()
    conv.run_checks()
    conv.write_reports()
    conv.write_provenance()


if __name__ == '__main__':
    main()
