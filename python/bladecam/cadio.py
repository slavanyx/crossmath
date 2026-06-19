"""Lightweight CAD / data I/O for BladeCAM (no heavy dependencies).

- STL mesh read/write (ASCII + binary) for blade/stock display & collision.
- Rail-polyline CSV read/write: the hub and shroud boundary curves that
  define the ruled blade flank, so externally-designed blades can be loaded.

STEP/IGES import needs a B-rep kernel (e.g. OpenCASCADE via pythonocc) and is
intentionally out of scope here; `read_rails_csv` / `read_stl` are the bridge.
"""
from __future__ import annotations

import struct
import numpy as np


# --------------------------------------------------------------------------
# Rail polylines (hub + shroud) <-> CSV
# --------------------------------------------------------------------------
def write_rails_csv(path: str, a: np.ndarray, b: np.ndarray) -> None:
    """Write hub rail a(nu,3) and shroud rail b(nu,3) to a CSV."""
    data = np.column_stack([np.arange(a.shape[0]), a, b])
    np.savetxt(path, data, delimiter=",",
               header="station,ax,ay,az,bx,by,bz", comments="")


def read_rails_csv(path: str):
    """Read (a, b) hub/shroud rails from a CSV written by write_rails_csv."""
    d = np.loadtxt(path, delimiter=",", skiprows=1)
    a = np.ascontiguousarray(d[:, 1:4])
    b = np.ascontiguousarray(d[:, 4:7])
    return a, b


# --------------------------------------------------------------------------
# STL mesh
# --------------------------------------------------------------------------
def surface_to_triangles(surf: np.ndarray):
    """Triangulate a (nu,nv,3) structured surface grid into (verts, faces)."""
    nu, nv, _ = surf.shape
    verts = surf.reshape(-1, 3)
    faces = []
    for i in range(nu - 1):
        for j in range(nv - 1):
            p00 = i * nv + j
            p01 = i * nv + j + 1
            p10 = (i + 1) * nv + j
            p11 = (i + 1) * nv + j + 1
            faces.append((p00, p10, p11))
            faces.append((p00, p11, p01))
    return verts, np.asarray(faces, dtype=np.int64)


def write_stl(path: str, verts: np.ndarray, faces: np.ndarray,
              binary: bool = True, name: str = "bladecam") -> None:
    """Write a triangle mesh to STL (binary by default)."""
    tris = verts[faces]                              # (nf, 3, 3)
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.divide(n, ln, out=np.zeros_like(n), where=ln > 0)
    if binary:
        with open(path, "wb") as f:
            f.write(b"\0" * 80)
            f.write(struct.pack("<I", len(faces)))
            for k in range(len(faces)):
                f.write(struct.pack("<3f", *n[k]))
                for vtx in tris[k]:
                    f.write(struct.pack("<3f", *vtx))
                f.write(struct.pack("<H", 0))
    else:
        with open(path, "w") as f:
            f.write(f"solid {name}\n")
            for k in range(len(faces)):
                f.write(f"facet normal {n[k,0]} {n[k,1]} {n[k,2]}\n outer loop\n")
                for vtx in tris[k]:
                    f.write(f"  vertex {vtx[0]} {vtx[1]} {vtx[2]}\n")
                f.write(" endloop\nendfacet\n")
            f.write(f"endsolid {name}\n")


def read_stl(path: str):
    """Read an STL (auto-detect ASCII/binary). Returns (verts, faces)."""
    with open(path, "rb") as f:
        head = f.read(5)
        f.seek(0)
        if head == b"solid":
            txt = f.read().decode("ascii", errors="replace")
            if "facet" in txt:                       # genuine ASCII STL
                return _read_ascii_stl(txt)
        return _read_binary_stl(path)


def _read_binary_stl(path: str):
    with open(path, "rb") as f:
        f.read(80)
        (ntri,) = struct.unpack("<I", f.read(4))
        verts = np.empty((ntri * 3, 3), dtype=np.float64)
        for k in range(ntri):
            f.read(12)                               # skip normal
            for j in range(3):
                verts[k*3 + j] = struct.unpack("<3f", f.read(12))
            f.read(2)                                # attr byte count
    faces = np.arange(ntri * 3).reshape(-1, 3)
    return verts, faces


def _read_ascii_stl(txt: str):
    vs = []
    for line in txt.splitlines():
        s = line.strip()
        if s.startswith("vertex"):
            vs.append([float(x) for x in s.split()[1:4]])
    verts = np.asarray(vs, dtype=np.float64)
    faces = np.arange(verts.shape[0]).reshape(-1, 3)
    return verts, faces


# --------------------------------------------------------------------------
# STEP / IGES (requires OpenCASCADE bindings: pip install cadquery-ocp)
# --------------------------------------------------------------------------
def _shape_to_mesh(shape, lin_defl: float = 0.5):
    """Tessellate an OCC shape into (verts, faces) in millimetres."""
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_FACE
    from OCP.BRep import BRep_Tool
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS

    BRepMesh_IncrementalMesh(shape, lin_defl)
    verts, faces = [], []
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = TopoDS.Face_s(exp.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            trsf = loc.Transformation()
            base = len(verts)
            for i in range(1, tri.NbNodes() + 1):
                p = tri.Node(i).Transformed(trsf)
                verts.append([p.X(), p.Y(), p.Z()])
            for i in range(1, tri.NbTriangles() + 1):
                t = tri.Triangle(i)
                faces.append([base + t.Value(1) - 1,
                              base + t.Value(2) - 1,
                              base + t.Value(3) - 1])
        exp.Next()
    return np.asarray(verts, dtype=np.float64), np.asarray(faces, dtype=np.int64)


def _occ_or_raise():
    try:
        import OCP  # noqa: F401
    except ImportError as e:
        raise ImportError("STEP/IGES needs OpenCASCADE: pip install "
                          "cadquery-ocp") from e


def _read_step_shape(path: str):
    _occ_or_raise()
    from OCP.STEPControl import STEPControl_Reader
    from OCP.IFSelect import IFSelect_RetDone
    rd = STEPControl_Reader()
    if rd.ReadFile(path) != IFSelect_RetDone:
        raise IOError(f"failed to read STEP file: {path}")
    rd.TransferRoots()
    return rd.OneShape()


def _read_iges_shape(path: str):
    _occ_or_raise()
    from OCP.IGESControl import IGESControl_Reader
    from OCP.IFSelect import IFSelect_RetDone
    rd = IGESControl_Reader()
    if rd.ReadFile(path) != IFSelect_RetDone:
        raise IOError(f"failed to read IGES file: {path}")
    rd.TransferRoots()
    return rd.OneShape()


def read_step(path: str, lin_defl: float = 0.5):
    """Read a STEP file and return a tessellated (verts, faces) mesh."""
    return _shape_to_mesh(_read_step_shape(path), lin_defl)


def read_iges(path: str, lin_defl: float = 0.5):
    """Read an IGES file and return a tessellated (verts, faces) mesh."""
    return _shape_to_mesh(_read_iges_shape(path), lin_defl)


def read_cad(path: str):
    """Dispatch by extension: .stl / .step / .stp / .iges / .igs -> mesh."""
    ext = path.lower().rsplit(".", 1)[-1]
    if ext == "stl":
        return read_stl(path)
    if ext in ("step", "stp"):
        return read_step(path)
    if ext in ("iges", "igs"):
        return read_iges(path)
    raise ValueError(f"unsupported CAD format: .{ext}")


# --------------------------------------------------------------------------
# Automatic ruled-rail extraction from a STEP/IGES blade face
# --------------------------------------------------------------------------
def _largest_face(shape):
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopoDS import TopoDS
    from OCP.GProp import GProp_GProps
    from OCP.BRepGProp import BRepGProp
    best, best_area = None, -1.0
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = TopoDS.Face_s(exp.Current())
        g = GProp_GProps()
        BRepGProp.SurfaceProperties_s(face, g)
        if g.Mass() > best_area:
            best, best_area = face, g.Mass()
        exp.Next()
    if best is None:
        raise ValueError("no faces found in CAD shape")
    return best


def _straightness(pts):
    """Max off-line deviation of a sampled curve, normalised by chord length."""
    d = pts[-1] - pts[0]
    L = np.linalg.norm(d)
    if L < 1e-9:
        return 1.0
    t = ((pts - pts[0]) @ d) / L**2
    proj = pts[0] + t[:, None] * d
    return float(np.max(np.linalg.norm(pts - proj, axis=1)) / L)


def _all_faces(shape):
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopoDS import TopoDS
    faces = []
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        faces.append(TopoDS.Face_s(exp.Current())); exp.Next()
    return faces


def _face_area(face):
    from OCP.GProp import GProp_GProps
    from OCP.BRepGProp import BRepGProp
    g = GProp_GProps()
    BRepGProp.SurfaceProperties_s(face, g)
    return g.Mass()


def rails_from_shape(shape, nu: int = 60, face_index=None, ndetect: int = 11):
    """Extract hub/shroud rails (a, b) of shape's blade face as (nu,3) arrays.

    The blade flank is taken as the largest face (or face_index). See
    _rails_from_face for the ruling-detection details and assumptions.
    """
    if face_index is None:
        face = _largest_face(shape)
    else:
        face = _all_faces(shape)[face_index]
    return _rails_from_face(face, nu, ndetect)


def _sample_edge(edge, n):
    """Uniform arc-length sampling of a B-rep edge into (n,3) points.
    Raises on a degenerate/uncomputable edge so callers can fall back."""
    from OCP.BRepAdaptor import BRepAdaptor_Curve
    from OCP.GCPnts import GCPnts_QuasiUniformAbscissa
    ac = BRepAdaptor_Curve(edge)
    gc = GCPnts_QuasiUniformAbscissa(ac, n)
    if not gc.IsDone() or gc.NbPoints() < n:
        raise ValueError("edge abscissa not computable")
    out = np.empty((n, 3))
    for i in range(1, n + 1):
        p = ac.Value(gc.Parameter(i))
        out[i - 1] = (p.X(), p.Y(), p.Z())
    return out


def _rails_from_face_edges(face, nu):
    """Edge-based rail extraction: follow the face's ACTUAL boundary curves.

    Works for a 4-sided flank face (two rails + leading/trailing ends), and is
    robust to trimming because it samples the real edges, not the UV box. The
    two rails are the curved pair (rulings are straight); returns None to signal
    fallback when the outer wire is not 4-sided.
    """
    from OCP.BRepTools import BRepTools, BRepTools_WireExplorer
    try:
        we = BRepTools_WireExplorer(BRepTools.OuterWire_s(face))
        edges = []
        while we.More():
            edges.append(we.Current()); we.Next()
        if len(edges) != 4:
            return None
        st = [_straightness(_sample_edge(e, 21)) for e in edges]
        # opposite edges pair up; rails = the more-curved pair (rulings straight)
        rails = (0, 2) if (st[0] + st[2]) >= (st[1] + st[3]) else (1, 3)
        a = _sample_edge(edges[rails[0]], nu)
        b = _sample_edge(edges[rails[1]], nu)
    except Exception:
        return None    # degenerate/seamed/uncomputable face -> UV-box fallback
    if np.linalg.norm(a[0] - b[0]) > np.linalg.norm(a[0] - b[-1]):
        b = b[::-1]
    return np.ascontiguousarray(a), np.ascontiguousarray(b)


def _orient_hub_first(a, b):
    """Normalise rail orientation so extraction is consistent across blades:
    `a` is the HUB rail (smaller mean radius from the spin axis Z) and station 0
    is the lower-Z (hub-platform) end. Without this, different faces of a blisk
    come out with hub/shroud swapped or reversed, breaking batch optimisation,
    collision (neighbour rotation) and G-code ordering."""
    a = np.asarray(a, float); b = np.asarray(b, float)
    if np.mean(np.hypot(b[:, 0], b[:, 1])) < np.mean(np.hypot(a[:, 0], a[:, 1])):
        a, b = b, a                      # ensure a = hub (inner radius)
    if a[0, 2] > a[-1, 2]:               # ensure station 0 = lower-Z end
        a, b = a[::-1], b[::-1]
    return np.ascontiguousarray(a), np.ascontiguousarray(b)


def _rails_from_face(face, nu: int = 60, ndetect: int = 11):
    """Extract hub/shroud rails of a single B-rep face.

    Primary path: edge-based extraction following the face's actual boundary
    (robust to trimmed leading/trailing edges). Fallback for non-4-sided faces:
    auto-detect the ruling direction (straightest isocurves) and read the rails
    off the UV box -- valid for untrimmed rectangular-UV ruled faces. For
    (near-)planar/ambiguous faces the choice is geometrically benign and the
    tie-break is deterministic.
    """
    from OCP.BRep import BRep_Tool
    from OCP.BRepTools import BRepTools

    edge_rails = _rails_from_face_edges(face, nu)
    if edge_rails is not None:
        return _orient_hub_first(*edge_rails)

    s = BRep_Tool.Surface_s(face)
    umin, umax, vmin, vmax = BRepTools.UVBounds_s(face)

    def val(uu, vv):
        p = s.Value(uu, vv)
        return np.array([p.X(), p.Y(), p.Z()])

    # ruling direction = parameter with the straighter isocurves
    us = np.linspace(umin, umax, 5)
    vs = np.linspace(vmin, vmax, 5)
    kk = np.linspace(0.0, 1.0, ndetect)
    resid_v = np.mean([_straightness(np.array([val(uu, vmin + t*(vmax-vmin))
                                               for t in kk])) for uu in us])
    resid_u = np.mean([_straightness(np.array([val(umin + t*(umax-umin), vv)
                                               for t in kk])) for vv in vs])

    ps = np.linspace(0.0, 1.0, nu)
    # Deterministic tie-break: only treat u as the ruling direction when its
    # isocurves are CLEARLY straighter than v's; otherwise default to v. This
    # avoids floating-point noise flipping the choice on (near-)planar faces.
    if resid_u < resid_v - 1e-6:                # u is the ruling (hub->shroud)
        a = np.array([val(umin, vmin + t*(vmax-vmin)) for t in ps])
        b = np.array([val(umax, vmin + t*(vmax-vmin)) for t in ps])
    else:                                       # v is the ruling (hub->shroud)
        a = np.array([val(umin + t*(umax-umin), vmin) for t in ps])
        b = np.array([val(umin + t*(umax-umin), vmax) for t in ps])
    return _orient_hub_first(a, b)


def rails_from_all_faces(shape, nu: int = 60, min_area_frac: float = 0.3):
    """Extract rails for EVERY flank face of a blisk shape.

    Faces with area >= min_area_frac * (largest face area) are treated as blade
    flanks (this rejects small fillets/edges/platform slivers). Returns a list
    of (a, b) rail pairs, one per blade, ordered largest-area first.
    """
    faces = _all_faces(shape)
    if not faces:
        raise ValueError("no faces in shape")
    areas = [_face_area(f) for f in faces]
    amax = max(areas)
    kept = [f for f, ar in sorted(zip(faces, areas), key=lambda t: -t[1])
            if ar >= min_area_frac * amax]
    return [_rails_from_face(f, nu) for f in kept]


def _shape_from_cad(path: str):
    ext = path.lower().rsplit(".", 1)[-1]
    if ext in ("step", "stp"):
        return _read_step_shape(path)
    if ext in ("iges", "igs"):
        return _read_iges_shape(path)
    raise ValueError(f"rail extraction needs a STEP/IGES B-rep surface, not .{ext}")


def rails_from_cad(path: str, nu: int = 60, face_index=None):
    """Load a STEP/IGES blade and return ruled hub/shroud rails (a, b)."""
    return rails_from_shape(_shape_from_cad(path), nu=nu, face_index=face_index)


def rails_list_from_cad(path: str, nu: int = 60, min_area_frac: float = 0.3):
    """Load a STEP/IGES blisk and return a list of (a, b) rails, one per blade."""
    return rails_from_all_faces(_shape_from_cad(path), nu=nu,
                                min_area_frac=min_area_frac)
