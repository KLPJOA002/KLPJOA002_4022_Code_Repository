#!/usr/bin/env python3
"""
Turn a fused point cloud (fuse_scans output) into a triangle mesh so it can be
rendered as a surface.

Why this is needed: the RealSense export is a mesh only because every point
sits on the camera's pixel grid, so neighbouring pixels are joined into
triangles. Cropping (Open3D read_point_cloud) drops those faces, and fusion
re-samples the surface into cells, so there is no pixel grid left to
triangulate. The surface has to be reconstructed from the points.

Methods:
  heightfield (default) - 2.5D Delaunay in the surface's best-fit plane.
                          Uses every point as a vertex (no smoothing, keeps
                          colours exactly), drops triangles with long edges
                          so holes stay holes. Right for sand / ice scanned
                          from one side.
  poisson               - Screened Poisson. Watertight-ish and smooth, fills
                          small holes, but slightly smooths detail. Low-
                          density (invented) regions are trimmed.

Usage:
  python3 mesh_fused.py fused.ply fused_mesh.ply
  python3 mesh_fused.py fused.ply fused_mesh.ply --method poisson --depth 10
  python3 mesh_fused.py fused.ply fused_mesh.ply --max-edge 0.006
"""

import argparse
import sys

import numpy as np
import open3d as o3d


def plane_frame(points):
    """Centroid and a basis whose 3rd row is the best-fit plane normal."""
    c = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points[:: max(1, len(points) // 200000)] - c, full_matrices=False)
    return c, vt


def heightfield_mesh(pcd, max_edge=None, max_edge_factor=6.0):
    from scipy.spatial import Delaunay

    pts = np.asarray(pcd.points)
    c, basis = plane_frame(pts)
    uv = (pts - c) @ basis[:2].T
    tri = Delaunay(uv).simplices

    # Drop long, thin triangles that bridge gaps or wrap around the boundary.
    if max_edge is None:
        spacing = np.median(np.asarray(pcd.compute_nearest_neighbor_distance()))
        max_edge = max_edge_factor * max(spacing, 1e-4)
        # Fused clouds are often denser locally than on average; use the
        # typical triangle edge as the yardstick instead if larger.
        e = np.linalg.norm(pts[tri[:, 0]] - pts[tri[:, 1]], axis=1)
        max_edge = max(max_edge, 3.0 * np.median(e))
    p0, p1, p2 = pts[tri[:, 0]], pts[tri[:, 1]], pts[tri[:, 2]]
    longest = np.max(np.stack([np.linalg.norm(p0 - p1, axis=1),
                               np.linalg.norm(p1 - p2, axis=1),
                               np.linalg.norm(p2 - p0, axis=1)]), axis=0)
    tri = tri[longest <= max_edge]
    print(f"  heightfield: {len(tri)} triangles kept (max edge {max_edge * 1000:.2f} mm)")

    # Orient all triangles to face the same side as the cloud's normals
    # (or the camera side of the plane if there are none).
    n = np.cross(pts[tri[:, 1]] - pts[tri[:, 0]], pts[tri[:, 2]] - pts[tri[:, 0]])
    up = basis[2]
    if pcd.has_normals() and np.asarray(pcd.normals).mean(axis=0) @ up < 0:
        up = -up
    flip = n @ up < 0
    tri[flip] = tri[flip][:, [0, 2, 1]]

    mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(pts),
                                     o3d.utility.Vector3iVector(tri))
    if pcd.has_colors():
        mesh.vertex_colors = pcd.colors
    return mesh


def poisson_mesh(pcd, depth=10, density_quantile=0.05):
    if not pcd.has_normals():
        pcd.estimate_normals(o3d.geometry.KDTreeSearchParamKNN(30))
    # Point all normals to the same side (the mean normal's side).
    n = np.asarray(pcd.normals)
    mean = n.mean(axis=0)
    n[n @ mean < 0] *= -1
    pcd.normals = o3d.utility.Vector3dVector(n)

    mesh, density = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=depth)
    density = np.asarray(density)
    mesh.remove_vertices_by_mask(density < np.quantile(density, density_quantile))
    # Poisson extrapolates past the data; clip to the cloud's footprint.
    mesh = mesh.crop(pcd.get_axis_aligned_bounding_box())
    print(f"  poisson: {len(mesh.triangles)} triangles (depth {depth})")
    return mesh


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input")
    ap.add_argument("output")
    ap.add_argument("--method", choices=["heightfield", "poisson"], default="heightfield")
    ap.add_argument("--max-edge", type=float, help="heightfield: drop triangles with an edge longer than this (m)")
    ap.add_argument("--depth", type=int, default=10, help="poisson: octree depth")
    ap.add_argument("--trim", type=float, default=0.05, help="poisson: drop lowest-density fraction")
    ap.add_argument("--smooth", type=int, default=0, help="Taubin smoothing iterations after meshing")
    args = ap.parse_args()

    pcd = o3d.io.read_point_cloud(args.input)
    pcd = pcd.remove_non_finite_points()
    if len(pcd.points) == 0:
        sys.exit("no points")
    print(f"Loaded {len(pcd.points)} points")

    if args.method == "heightfield":
        mesh = heightfield_mesh(pcd, args.max_edge)
    else:
        mesh = poisson_mesh(pcd, args.depth, args.trim)

    mesh.remove_unreferenced_vertices()
    mesh.remove_degenerate_triangles()
    if args.smooth:
        mesh = mesh.filter_smooth_taubin(args.smooth)
    mesh.compute_vertex_normals()
    o3d.io.write_triangle_mesh(args.output, mesh, write_ascii=False)
    print(f"Saved {args.output}: {len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles")


if __name__ == "__main__":
    main()
