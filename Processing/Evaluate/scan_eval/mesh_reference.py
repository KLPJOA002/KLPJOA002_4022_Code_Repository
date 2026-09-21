"""MeshReference: a triangle mesh (.stl / .ply / .obj) taken as ground truth.

Distances are to the triangles themselves (Open3D RaycastingScene, exact
point-to-triangle queries), never to the vertices, so the result does not
depend on how finely the mesh happens to be tessellated beyond the mesh's own
discretisation of the true shape. No attempt is made to recover an underlying
function: the mesh *is* the truth.
"""

from __future__ import annotations

import hashlib
from pathlib import Path

import numpy as np
import open3d as o3d

from .log import log, warn
from .reference import ReferenceSurface, SurfaceError, parse_direction

MESH_EXTENSIONS = {".stl", ".ply", ".obj", ".off", ".gltf", ".glb"}


class MeshReference(ReferenceSurface):
    kind = "mesh"
    exact = False

    def __init__(self, path, units: str = "mm", name: str | None = None,
                 height_field: bool = False, visible_from=None):
        path = Path(path)
        if not path.is_file():
            raise SurfaceError(f"reference mesh not found: {path}")
        if path.suffix.lower() not in MESH_EXTENSIONS:
            raise SurfaceError(f"unsupported mesh format '{path.suffix}' "
                               f"(use one of {', '.join(sorted(MESH_EXTENSIONS))})")

        mesh = o3d.io.read_triangle_mesh(str(path))
        if len(mesh.vertices) == 0:
            raise SurfaceError(f"could not read any vertices from {path}")
        if len(mesh.triangles) == 0:
            raise SurfaceError(
                f"{path} has vertices but no faces - it is a point cloud, not a surface. "
                "A reference must be a mesh (or an analytic definition): distances to "
                "sampled points would measure the sampling, not the part.")

        self.path = path
        self.units = units
        self.name = name or path.stem
        self.is_height_field = bool(height_field)
        self.visible_from = parse_direction(visible_from)
        if self.visible_from is None and self.is_height_field:
            self.visible_from = np.array([0.0, 0.0, 1.0])

        with open(path, "rb") as fh:
            self.file_sha256 = hashlib.sha256(fh.read()).hexdigest()

        self.mesh, self.topology = _clean_and_orient(mesh, path.name)
        self.vertices = np.asarray(self.mesh.vertices, dtype=float)
        self.triangles = np.asarray(self.mesh.triangles, dtype=np.int64)
        self.face_normals = _face_normals(self.vertices, self.triangles)
        tri = self.vertices[self.triangles]
        self.face_areas = 0.5 * np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)

        # RaycastingScene works in float32. Querying in coordinates centred on
        # the mesh keeps the rounding well below a micrometre for any part
        # that fits on a desk, whatever its absolute position.
        self._centre = self.vertices.mean(axis=0)
        tmesh = o3d.t.geometry.TriangleMesh()
        tmesh.vertex.positions = o3d.core.Tensor((self.vertices - self._centre).astype(np.float32))
        tmesh.triangle.indices = o3d.core.Tensor(self.triangles.astype(np.int32))
        self._scene = o3d.t.geometry.RaycastingScene()
        self._scene.add_triangles(tmesh)
        self._lo = self.vertices.min(axis=0)
        self._hi = self.vertices.max(axis=0)

    # --- interface ---------------------------------------------------------
    def sample(self, n, seed=0):
        rng = np.random.default_rng(seed)
        prob = self.face_areas / self.face_areas.sum()
        faces = rng.choice(len(self.triangles), size=n, p=prob)
        r1, r2 = rng.random(n), rng.random(n)
        s = np.sqrt(r1)
        tri = self.vertices[self.triangles[faces]]
        pts = (1 - s)[:, None] * tri[:, 0] + (s * (1 - r2))[:, None] * tri[:, 1] + (s * r2)[:, None] * tri[:, 2]
        return pts, self.face_normals[faces].copy()

    def closest_points(self, points):
        points = np.asarray(points, dtype=float)
        if len(points) == 0:
            return np.empty((0, 3)), np.empty((0, 3))
        query = o3d.core.Tensor((points - self._centre).astype(np.float32))
        result = self._scene.compute_closest_points(query)
        closest = result["points"].numpy().astype(float) + self._centre
        faces = result["primitive_ids"].numpy().astype(np.int64)
        return closest, self.face_normals[faces]

    def bounds(self):
        return self._lo.copy(), self._hi.copy()

    def height(self, x, y):
        if not self.is_height_field:
            return super().height(x, y)
        x, y = np.broadcast_arrays(np.asarray(x, float), np.asarray(y, float))
        shape = x.shape
        top = self._hi[2] + 1.0 + 0.01 * np.ptp(self.vertices[:, 2])
        rays = np.zeros((x.size, 6), dtype=np.float32)
        rays[:, 0] = x.ravel() - self._centre[0]
        rays[:, 1] = y.ravel() - self._centre[1]
        rays[:, 2] = top - self._centre[2]
        rays[:, 5] = -1.0
        t = self._scene.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy().astype(float)
        z = top - t
        z[~np.isfinite(t)] = np.nan
        return z.reshape(shape)

    def region_mask(self, points, margin=0.0):
        points = np.asarray(points, dtype=float)
        if not self.is_height_field:
            return super().region_mask(points, margin)
        # Height field: the region is an outline in x, y only - a point far
        # above the part is a gross error to be scored, not background.
        grow = max(margin, 0.0)
        box = np.all((points[:, :2] >= self._lo[:2] - grow) & (points[:, :2] <= self._hi[:2] + grow), axis=1)
        # A height field's region is its footprint seen from above, not its
        # bounding box - they differ for any non-rectangular outline.
        # A positive margin grows the outline (a hit at any offset counts), a
        # negative one insets it (every offset must hit), e.g. to leave out
        # the rim where a scan catches the part's side walls.
        idx = np.flatnonzero(box)
        m = abs(margin)
        offsets = [(0.0, 0.0)] + ([(m, 0), (-m, 0), (0, m), (0, -m)] if m > 0 else [])
        hits = np.stack([np.isfinite(self.height(points[idx, 0] + dx, points[idx, 1] + dy))
                         for dx, dy in offsets])
        inside = np.zeros(len(points), dtype=bool)
        inside[idx] = hits.any(axis=0) if margin >= 0 else hits.all(axis=0)
        return inside

    def visible_mask(self, points, normals):
        facing = super().visible_mask(points, normals)
        if self.visible_from is None or not facing.any():
            return facing
        # A face can point the right way and still be hidden - e.g. an internal
        # face where two shells were stacked without being merged, which no
        # scanner can ever see. Cast a ray from each sample toward the scanner:
        # if it hits the part, the sample is occluded.
        idx = np.flatnonzero(facing)
        eps = 1e-5 * float(np.linalg.norm(self._hi - self._lo))
        origin = np.asarray(points, dtype=float)[idx] + eps * self.visible_from - self._centre
        rays = np.hstack([origin, np.broadcast_to(self.visible_from, origin.shape)]).astype(np.float32)
        hit = np.isfinite(self._scene.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy())
        facing[idx[hit]] = False
        return facing

    def surface_area(self):
        if self.visible_from is None:
            return float(self.face_areas.sum())
        centres = self.vertices[self.triangles].mean(axis=1)
        visible = self.visible_mask(centres, self.face_normals)
        return float(self.face_areas[visible].sum())

    def definition(self):
        return {"kind": "mesh", "file_sha256": self.file_sha256, "units": self.units,
                "height_field": self.is_height_field,
                "visible_from": None if self.visible_from is None else self.visible_from.round(6).tolist()}

    def describe(self):
        d = super().describe()
        d.update({"path": str(self.path), "triangles": int(len(self.triangles)),
                  "file_sha256": self.file_sha256, **self.topology})
        return d


def is_closed(triangles: np.ndarray) -> bool:
    """Watertight in the sense that matters here: every edge is shared by
    exactly two faces. (Open3D's is_watertight also runs an O(n^2)
    self-intersection test - 20 s on a 200k-triangle part.)"""
    t = np.asarray(triangles, dtype=np.int64)
    if len(t) == 0:
        return False
    edges = np.sort(np.concatenate([t[:, [0, 1]], t[:, [1, 2]], t[:, [2, 0]]]), axis=1)
    _, counts = np.unique(edges, axis=0, return_counts=True)
    return bool(np.all(counts == 2))


def consistently_oriented(triangles: np.ndarray) -> bool:
    """Every directed edge used once: neighbouring faces wind the same way."""
    t = np.asarray(triangles, dtype=np.int64)
    directed = np.concatenate([t[:, [0, 1]], t[:, [1, 2]], t[:, [2, 0]]])
    return len(np.unique(directed, axis=0)) == len(directed)


def signed_volume(vertices: np.ndarray, triangles: np.ndarray) -> float:
    tri = np.asarray(vertices)[np.asarray(triangles)]
    return float(np.einsum("ij,ij->i", tri[:, 0], np.cross(tri[:, 1], tri[:, 2])).sum() / 6.0)


def _face_normals(vertices, triangles):
    tri = vertices[triangles]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    length = np.linalg.norm(n, axis=1, keepdims=True)
    return n / np.maximum(length, 1e-300)


def _clean_and_orient(mesh: o3d.geometry.TriangleMesh, label: str):
    """Repairs what can be repaired without changing the surface, and makes
    face normals point consistently outward so the sign of the error means
    something. Reports (does not hide) anything it could not fix."""
    n_before = len(mesh.triangles)
    mesh.remove_duplicated_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()

    removed_non_manifold = 0
    if not mesh.is_edge_manifold():
        # Typically an internal face left over from modelling (e.g. the plane
        # where a base plate was joined on). It is inside the part, so no
        # scanner can see it, but it makes the orientation undefined.
        before = len(mesh.triangles)
        mesh.remove_non_manifold_edges()
        mesh.remove_unreferenced_vertices()
        removed_non_manifold = before - len(mesh.triangles)
        warn(f"reference {label}: removed {removed_non_manifold} face(s) on non-manifold edges "
             f"(internal/duplicate faces) out of {n_before}")

    watertight = is_closed(np.asarray(mesh.triangles))
    # Open3D's orientation routines take tens of seconds on a million-face
    # mesh; most meshes are already consistent, which numpy checks instantly.
    if consistently_oriented(np.asarray(mesh.triangles)):
        orientable = True
    else:
        orientable = bool(mesh.is_orientable())
        if orientable:
            mesh.orient_triangles()

    vertices = np.asarray(mesh.vertices)
    triangles = np.asarray(mesh.triangles)
    normals = _face_normals(vertices, triangles)
    tri = vertices[triangles]
    areas = 0.5 * np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)
    if watertight:
        # Divergence theorem: positive volume <=> outward normals.
        outward = signed_volume(vertices, triangles) > 0
    else:
        # Open surface: call "outward" the side facing away from the bulk of
        # the part, by area-weighted vote.
        centres = tri.mean(axis=1)
        vote = np.einsum("ij,ij->i", normals, centres - vertices.mean(axis=0)) * areas
        outward = vote.sum() >= 0
    if not outward:
        mesh.triangles = o3d.utility.Vector3iVector(triangles[:, ::-1].copy())

    if not watertight:
        warn(f"reference {label} is not watertight (open surface or holes). Distances are "
             "still exact to its triangles, but inside/outside is undefined: the sign of the "
             "error comes from the oriented face normals" +
             ("" if orientable else " - and the mesh is NOT orientable, so signs may flip "
              "between patches of faces. Repair the mesh for trustworthy signed error."))
    log(f"[reference] mesh {label}: {len(mesh.triangles)} triangles, watertight={watertight}, "
        f"orientable={orientable}")
    return mesh, {"watertight": watertight, "orientable": orientable,
                  "removed_non_manifold_faces": removed_non_manifold}
