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


def read_step(path: str, lin_defl: float = 0.5):
    """Read a STEP file and return a tessellated (verts, faces) mesh.

    Requires OpenCASCADE bindings. Enables display / collision / STL export of
    real CAD blades; ruled-rail extraction for the optimizer is separate (use
    rails CSV). Raises ImportError with guidance if OCP is unavailable.
    """
    try:
        from OCP.STEPControl import STEPControl_Reader
        from OCP.IFSelect import IFSelect_RetDone
    except ImportError as e:
        raise ImportError("STEP import needs OpenCASCADE: pip install "
                          "cadquery-ocp") from e
    rd = STEPControl_Reader()
    if rd.ReadFile(path) != IFSelect_RetDone:
        raise IOError(f"failed to read STEP file: {path}")
    rd.TransferRoots()
    return _shape_to_mesh(rd.OneShape(), lin_defl)


def read_iges(path: str, lin_defl: float = 0.5):
    """Read an IGES file and return a tessellated (verts, faces) mesh."""
    try:
        from OCP.IGESControl import IGESControl_Reader
        from OCP.IFSelect import IFSelect_RetDone
    except ImportError as e:
        raise ImportError("IGES import needs OpenCASCADE: pip install "
                          "cadquery-ocp") from e
    rd = IGESControl_Reader()
    if rd.ReadFile(path) != IFSelect_RetDone:
        raise IOError(f"failed to read IGES file: {path}")
    rd.TransferRoots()
    return _shape_to_mesh(rd.OneShape(), lin_defl)


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
