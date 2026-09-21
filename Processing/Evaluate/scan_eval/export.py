"""Export an analytic reference to a triangle mesh (.stl / .ply), from the same
definition the evaluator uses, so the part you print or look at is exactly
the surface the scans are scored against.

Coordinates are written in the definition's own units (normally mm, which is
what slicers assume for STL).
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import open3d as o3d

from .analytic import HeightFieldReference, ImplicitReference, ParametricReference
from .log import log
from .mesh_reference import MeshReference, is_closed
from .reference import SurfaceError


def _grid_triangles(nu, nv, keep_vertex=None):
    i, j = np.meshgrid(np.arange(nu - 1), np.arange(nv - 1), indexing="ij")
    a = (i * nv + j).ravel()
    b = a + nv
    c = a + 1
    d = b + 1
    tris = np.concatenate([np.stack([a, b, d], 1), np.stack([a, d, c], 1)])
    if keep_vertex is not None:
        tris = tris[keep_vertex[tris].all(axis=1)]
    return tris


def parametric_mesh(ref: ParametricReference, resolution: float):
    """Triangulate on a (u, v) grid whose spacing on the surface is about
    `resolution` (in the reference's units)."""
    U, V = ref._grid(200)
    Su = np.linalg.norm(ref._Su(U, V), axis=-1).max()
    Sv = np.linalg.norm(ref._Sv(U, V), axis=-1).max()
    nu = max(2, int(np.ceil(np.ptp(ref.u_range) * Su / resolution)) + 1)
    nv = max(2, int(np.ceil(np.ptp(ref.v_range) * Sv / resolution)) + 1)
    if nu * nv > 40_000_000:
        raise SurfaceError(f"resolution {resolution} would need {nu}x{nv} vertices - choose a coarser one")
    U, V = np.meshgrid(np.linspace(*ref.u_range, nu), np.linspace(*ref.v_range, nv), indexing="ij")
    vertices = ref._S(U.ravel(), V.ravel())
    keep = ref._mask(U.ravel(), V.ravel())
    tris = _grid_triangles(nu, nv, keep)
    # Orient faces so their normals agree with the surface's (flip-aware) normal.
    if len(tris):
        tri = vertices[tris]
        n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        centre_uv = np.stack([U.ravel()[tris].mean(1), V.ravel()[tris].mean(1)], 1)
        want = ref._normal(centre_uv[:, 0], centre_uv[:, 1])
        flip = np.einsum("ij,ij->i", n, want) < 0
        tris[flip] = tris[flip][:, ::-1]
    return vertices, tris, (nu, nv)


def solid_height_field(ref: HeightFieldReference, resolution: float, base_z: float):
    """Closed, printable solid: the surface on top, walls down to a flat base
    at z = base_z. Needs an unmasked rectangular domain."""
    if ref.mask_symbolic is not None:
        raise SurfaceError("a solid with a base can only be built for an unmasked rectangular "
                           "domain - export the surface alone (omit --export-base)")
    top, top_tris, (nu, nv) = parametric_mesh(ref, resolution)
    if base_z >= np.nanmin(top[:, 2]):
        raise SurfaceError(f"--export-base {base_z} must be below the lowest point of the surface "
                           f"({np.nanmin(top[:, 2]):.4g})")
    n_top = len(top)
    bottom = top.copy()
    bottom[:, 2] = base_z
    vertices = np.concatenate([top, bottom])
    idx = np.arange(n_top).reshape(nu, nv)
    tris = [top_tris, _grid_triangles(nu, nv)[:, ::-1] + n_top]  # base faces down

    def wall(ring):
        a, b = ring[:-1], ring[1:]
        return np.concatenate([np.stack([a, b + n_top, b], 1), np.stack([a, a + n_top, b + n_top], 1)])

    # Boundary walked counter-clockwise seen from above, so walls face outward.
    ring = np.concatenate([idx[:, 0], idx[-1, 1:], idx[::-1, -1][1:], idx[0, ::-1][1:]])
    tris.append(wall(ring))
    return vertices, np.concatenate(tris)


def implicit_mesh(ref: ImplicitReference, resolution: float):
    try:
        from skimage.measure import marching_cubes
    except ImportError:
        raise SurfaceError("exporting an implicit surface needs scikit-image "
                           "(.venv/bin/pip install scikit-image)") from None
    box = ref.box
    counts = np.maximum(2, np.ceil(np.ptp(box, axis=1) / resolution).astype(int) + 1)
    if np.prod(counts) > 300_000_000:
        raise SurfaceError(f"resolution {resolution} needs a {counts} grid - choose a coarser one")
    axes = [np.linspace(lo, hi, n) for (lo, hi), n in zip(box, counts)]
    X, Y, Z = np.meshgrid(*axes, indexing="ij")
    F = ref._F(X, Y, Z) * ref.flip
    spacing = tuple(np.ptp(box, axis=1) / (counts - 1))
    vertices, faces, _, _ = marching_cubes(F, level=0.0, spacing=spacing)
    vertices = vertices + box[:, 0]
    # Snap vertices exactly onto F=0 (marching cubes interpolates linearly).
    vertices = ref._newton_to_surface(vertices, iterations=10)
    # marching_cubes orients faces toward increasing F; our outward normal is
    # +grad F (times flip), so the winding is reversed.
    return vertices, faces[:, ::-1].astype(np.int64)


def export_reference(ref, out_path, resolution: float | None = None, base_z: float | None = None):
    out_path = Path(out_path)
    if out_path.suffix.lower() not in (".stl", ".ply", ".obj"):
        raise SurfaceError(f"export format must be .stl, .ply or .obj (got {out_path.suffix})")
    options = getattr(ref, "export_options", {}) or {}
    if resolution is None:
        resolution = options.get("resolution")
    if base_z is None:
        base_z = options.get("base_z")

    if isinstance(ref, MeshReference):
        vertices, tris = ref.vertices, ref.triangles
    elif isinstance(ref, (ParametricReference, ImplicitReference)):
        if resolution is None:
            lo, hi = ref.bounds()
            resolution = float(np.linalg.norm(hi - lo)) / 500
        if isinstance(ref, ImplicitReference):
            vertices, tris = implicit_mesh(ref, resolution)
        elif base_z is not None and isinstance(ref, HeightFieldReference):
            vertices, tris = solid_height_field(ref, resolution, float(base_z))
        else:
            vertices, tris, _ = parametric_mesh(ref, resolution)
    else:
        raise SurfaceError(f"cannot export a '{ref.kind}' reference to a mesh")

    mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(vertices),
                                     o3d.utility.Vector3iVector(np.asarray(tris, dtype=np.int32)))
    mesh.remove_unreferenced_vertices()
    mesh.compute_triangle_normals()
    mesh.compute_vertex_normals()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    if not o3d.io.write_triangle_mesh(str(out_path), mesh, write_ascii=False):
        raise SurfaceError(f"failed to write {out_path}")
    log(f"[export] {ref.name}: {len(mesh.triangles)} triangles, {len(mesh.vertices)} vertices, "
        f"resolution {resolution} {ref.units}, closed={is_closed(np.asarray(mesh.triangles))} -> {out_path}")
    return mesh
