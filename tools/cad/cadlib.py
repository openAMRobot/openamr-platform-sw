# SPDX-License-Identifier: MIT
"""STEP reading, tessellation and measurement helpers for the CAD mesh converter.

Uses OpenCascade through OCP (cadquery-ocp). All lengths here are in the STEP unit (mm).
"""
import os
import re
import tempfile

import numpy as np
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.collections import Sequence_TDF_Label
from OCP.IFSelect import IFSelect_RetDone
from OCP.Quantity import Quantity_Color
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.STEPControl import STEPControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label
from OCP.TDocStd import TDocStd_Document
from OCP.TopLoc import TopLoc_Location
from OCP.XCAFDoc import XCAFDoc_ColorType, XCAFDoc_DocumentTool


def _decode_token(data):
    best, best_score = data.decode('latin-1'), 0
    for enc in ('cp1251', 'cp866'):
        try:
            text = data.decode(enc)
        except UnicodeDecodeError:
            continue
        score = sum('А' <= ch <= 'я' or ch in 'Ёё' for ch in text)
        if score > best_score:
            best, best_score = text, score
    return best


def decode_name(raw):
    """Instance names in the assembly are cp1251 or cp866 bytes read as Latin-1.

    One name can mix both (a cp1251 part name with a cp866 configuration suffix), so each
    token between spaces and underscores is decoded on its own."""
    try:
        data = raw.encode('latin-1')
    except UnicodeEncodeError:
        return raw
    if not any(b >= 0x80 for b in data):
        return raw
    parts = re.split(rb'([ _])', data)
    return ''.join(_decode_token(p) if any(b >= 0x80 for b in p) else p.decode('latin-1') for p in parts)


def trsf_matrix(loc):
    """4 x 4 matrix of a TopLoc_Location."""
    t = loc.Transformation()
    m = np.eye(4)
    for i in range(3):
        for j in range(4):
            m[i, j] = t.Value(i + 1, j + 1)
    return m


class Leaf:
    """One leaf instance of the assembly tree."""

    def __init__(self, names, label, locs, colour):
        self.names = names          # decoded instance names from the root, e.g. 'Front panel:1'
        self.path = ' / '.join(names)
        self.label = label          # prototype label (shape definition)
        self.locs = locs            # accumulated location after each path component
        self.loc = locs[-1]
        self.colour = colour
        self.shape = None           # set by Assembly
        self.cls = None
        self.rule = None


class Assembly:
    """Leaf instances of a STEP assembly with names, placements and colours (XCAF)."""

    def __init__(self, path):
        self.doc = TDocStd_Document(TCollection_ExtendedString('doc'))
        reader = STEPCAFControl_Reader()
        reader.SetColorMode(True)
        reader.SetNameMode(True)
        reader.SetLayerMode(False)
        if reader.ReadFile(path) != IFSelect_RetDone:
            raise RuntimeError('cannot read STEP %s' % path)
        reader.Transfer(self.doc)
        self.shapes = XCAFDoc_DocumentTool.ShapeTool_s(self.doc.Main())
        self.colours = XCAFDoc_DocumentTool.ColorTool_s(self.doc.Main())
        self.leaves = []
        roots = Sequence_TDF_Label()
        self.shapes.GetFreeShapes(roots)
        for i in range(1, roots.Length() + 1):
            self._walk(roots.Value(i), TopLoc_Location(), [], [], None)

    def _name(self, lab):
        attr = TDataStd_Name()
        if lab.FindAttribute(TDataStd_Name.GetID_s(), attr):
            return decode_name(attr.Get().ToExtString())
        return '?'

    def _colour(self, lab):
        c = Quantity_Color()
        for kind in (XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen):
            if self.colours.GetColor_s(lab, kind, c):
                return (round(c.Red(), 3), round(c.Green(), 3), round(c.Blue(), 3))
        return None

    def _walk(self, lab, loc, names, locs, colour):
        st = self.shapes
        proto = lab
        if st.IsReference_s(lab):
            proto = TDF_Label()
            st.GetReferredShape_s(lab, proto)
            loc = loc.Multiplied(st.GetLocation_s(lab))
            colour = self._colour(lab) or self._colour(proto) or colour
        else:
            colour = self._colour(lab) or colour
        names = names + [self._name(lab)]
        locs = locs + [loc]
        if st.IsAssembly_s(proto):
            kids = Sequence_TDF_Label()
            st.GetComponents_s(proto, kids)
            for i in range(1, kids.Length() + 1):
                self._walk(kids.Value(i), loc, names, locs, colour)
            return
        leaf = Leaf(names, proto, locs, colour)
        leaf.shape = st.GetShape_s(proto)
        self.leaves.append(leaf)

    def find(self, pattern):
        import re
        rx = re.compile(pattern)
        return [lf for lf in self.leaves if rx.search(lf.path)]


def read_shape(path):
    """Single shape of a part STEP (no assembly structure needed)."""
    reader = STEPControl_Reader()
    if reader.ReadFile(path) != IFSelect_RetDone:
        raise RuntimeError('cannot read STEP %s' % path)
    reader.TransferRoots()
    return reader.OneShape()


def is_void(shape):
    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box, False)
    return box.IsVoid()


def bbox(shape, loc=None):
    """Tight axis-aligned bounding box (mm) of a shape, optionally moved by loc."""
    if loc is not None:
        shape = shape.Moved(loc)
    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, box, False, False)
    if box.IsVoid():
        return None
    p, q = box.CornerMin(), box.CornerMax()
    return np.array([[p.X(), p.Y(), p.Z()], [q.X(), q.Y(), q.Z()]])


def merge_boxes(boxes):
    boxes = [b for b in boxes if b is not None]
    if not boxes:
        return None
    return np.array([np.min([b[0] for b in boxes], axis=0), np.max([b[1] for b in boxes], axis=0)])


def relative(frame_loc, leaf_loc):
    """Location of leaf_loc expressed in frame_loc."""
    return frame_loc.Inverted().Multiplied(leaf_loc)


_MESH_CACHE = {}


def tessellate(shape, linear_mm, angular_rad, key=None):
    """Triangles (n, 3, 3) of a shape in its own coordinates, mm."""
    if key is not None and key in _MESH_CACHE:
        return _MESH_CACHE[key]
    BRepMesh_IncrementalMesh(shape, linear_mm, False, angular_rad, True)
    fd, tmp = tempfile.mkstemp(suffix='.stl')
    os.close(fd)
    try:
        writer = StlAPI_Writer()
        writer.ASCIIMode = False
        writer.Write(shape, tmp)
        with open(tmp, 'rb') as stream:
            data = stream.read()
    finally:
        os.unlink(tmp)
    if len(data) < 84:
        tris = np.zeros((0, 3, 3))
    else:
        count = int.from_bytes(data[80:84], 'little')
        rec = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
        tris = np.frombuffer(data, dtype=rec, count=count, offset=84)['v'].astype(np.float64)
    if key is not None:
        _MESH_CACHE[key] = tris
    return tris


def transform(tris, matrix):
    """Apply a 4 x 4 matrix to triangles (n, 3, 3)."""
    flat = tris.reshape(-1, 3)
    out = flat @ matrix[:3, :3].T + matrix[:3, 3]
    return out.reshape(-1, 3, 3)


def subdivide(tris, max_edge, keep=None):
    """Bisect the longest edge of each triangle until every edge is at most max_edge.

    keep: optional box [lo, hi]; triangles outside it are dropped at every step, so only the
    region of interest is refined."""
    out = []
    while len(tris):
        if keep is not None:
            tris = tris[np.all((tris.max(1) >= keep[0]) & (tris.min(1) <= keep[1]), axis=1)]
        edges = np.linalg.norm(tris - np.roll(tris, -1, axis=1), axis=2)  # edge k: vertex k to k+1
        k = np.argmax(edges, axis=1)
        done = edges[np.arange(len(tris)), k] <= max_edge
        out.append(tris[done])
        t, k = tris[~done], k[~done]
        idx = np.arange(len(t))
        a, b, c = t[idx, k], t[idx, (k + 1) % 3], t[idx, (k + 2) % 3]
        m = (a + b) / 2
        tris = np.concatenate([np.stack([a, m, c], axis=1), np.stack([m, b, c], axis=1)])
    return np.concatenate(out) if out else np.zeros((0, 3, 3))


def _point_triangle_distance(p, t):
    """Distances (n, m) from points p (n, 3) to triangles t (m, 3, 3) (Ericson, closest point)."""
    a, b, c = t[None, :, 0], t[None, :, 1], t[None, :, 2]
    p = p[:, None, :]
    ab, ac, ap = b - a, c - a, p - a
    d1, d2 = np.sum(ab * ap, -1), np.sum(ac * ap, -1)
    bp = p - b
    d3, d4 = np.sum(ab * bp, -1), np.sum(ac * bp, -1)
    cp = p - c
    d5, d6 = np.sum(ab * cp, -1), np.sum(ac * cp, -1)
    va = d3 * d6 - d5 * d4
    vb = d5 * d2 - d1 * d6
    vc = d1 * d4 - d3 * d2
    denom = va + vb + vc
    with np.errstate(divide='ignore', invalid='ignore'):
        v = vb / denom
        w = vc / denom
        closest = a + ab * v[..., None] + ac * w[..., None]
        # edge and vertex regions
        t_ab = np.clip(d1 / (d1 - d3), 0, 1)
        t_ac = np.clip(d2 / (d2 - d6), 0, 1)
        t_bc = np.clip((d4 - d3) / ((d4 - d3) + (d5 - d6)), 0, 1)
    e_ab = a + ab * np.nan_to_num(t_ab)[..., None]
    e_ac = a + ac * np.nan_to_num(t_ac)[..., None]
    e_bc = b + (c - b) * np.nan_to_num(t_bc)[..., None]
    inside = (va >= 0) & (vb >= 0) & (vc >= 0) & (denom > 0)
    dist = np.minimum.reduce([np.linalg.norm(p - e_ab, axis=-1), np.linalg.norm(p - e_ac, axis=-1),
                              np.linalg.norm(p - e_bc, axis=-1)])
    face = np.linalg.norm(p - np.nan_to_num(closest), axis=-1)
    return np.where(inside, np.minimum(face, dist), dist)


def mesh_distance(tris_a, tris_b, reach=10.0, sample_edge=2.0):
    """Approximate minimum distance from mesh a to mesh b (same units), or None beyond reach.

    Mesh a is sampled at sample_edge (vertices and centroids of the subdivided triangles), so
    the result overestimates the true distance by at most about sample_edge / 2; each sample is
    measured exactly against the triangles of mesh b in the neighbouring cells of a uniform grid
    with cell size reach."""
    best = None
    for src, dst in ((tris_a, tris_b),):
        lo, hi = dst.reshape(-1, 3).min(0) - reach, dst.reshape(-1, 3).max(0) + reach
        near = np.all((src.max(1) >= lo) & (src.min(1) <= hi), axis=1)
        if not near.any():
            continue
        s = subdivide(src[near], sample_edge, keep=(lo, hi))
        pts = np.concatenate([s.reshape(-1, 3), s.mean(1)])
        pts = pts[np.all((pts >= lo) & (pts <= hi), axis=1)]
        if not len(pts):
            continue
        plo, phi = pts.min(0) - reach, pts.max(0) + reach
        tri = dst[np.all((dst.max(1) >= plo) & (dst.min(1) <= phi), axis=1)]
        if not len(tri):
            continue
        tri = subdivide(tri, reach, keep=(plo, phi))
        origin = plo
        cells = {}
        tlo = np.floor((tri.min(1) - origin) / reach).astype(int)
        thi = np.floor((tri.max(1) - origin) / reach).astype(int)
        for k, (a, b) in enumerate(zip(tlo, thi)):
            for i in range(a[0], b[0] + 1):
                for j in range(a[1], b[1] + 1):
                    for m in range(a[2], b[2] + 1):
                        cells.setdefault((i, j, m), []).append(k)
        pc = np.floor((pts - origin) / reach).astype(int)
        order = np.lexsort((pc[:, 2], pc[:, 1], pc[:, 0]))  # x-major, the order of np.unique
        pc, pts = pc[order], pts[order]
        keys, starts = np.unique(pc, axis=0, return_index=True)
        ends = list(starts[1:]) + [len(pts)]
        for key, a, b in zip(keys, starts, ends):
            cand = set()
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    for dm in (-1, 0, 1):
                        cand.update(cells.get((key[0] + di, key[1] + dj, key[2] + dm), ()))
            if not cand:
                continue
            ct = tri[sorted(cand)]
            step = max(1, 100000 // len(ct))
            for i in range(a, b, step):
                d = float(_point_triangle_distance(pts[i:min(b, i + step)], ct).min())
                best = d if best is None else min(best, d)
    return None if best is None or best > reach else best
