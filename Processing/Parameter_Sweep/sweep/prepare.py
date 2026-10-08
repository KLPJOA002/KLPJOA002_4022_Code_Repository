"""Per-scan preparation before Evaluate: AprilTag crop (reusing ../Crop) and a
seed pose from the table plane."""

from __future__ import annotations

import sys
import types
from pathlib import Path

import numpy as np
import open3d as o3d

from . import config as C

if str(C.CROP_DIR) not in sys.path:
    sys.path.insert(0, str(C.CROP_DIR))
import crop_pointclouds as crop_tool  # noqa: E402


class PrepareError(ValueError):
    """A scan that cannot be cropped or seeded."""


_layouts: dict = {}


def tag_layout(object_dir: Path):
    """The corner-tag order, learned once per object from all its sessions
    (as crop_pointclouds.py does for its input directory)."""
    key = str(object_dir)
    if key not in _layouts:
        _layouts[key] = crop_tool.learn_tag_layout(object_dir)
    return _layouts[key]


def crop_scan(ply: Path, object_dir: Path, out_path: Path) -> dict:
    """Automatic tag crop of one scan, written like the Crop tool writes it.
    Returns crop info for the results row."""
    pcd = o3d.io.read_point_cloud(str(ply))
    if len(pcd.points) == 0:
        raise PrepareError(f"{ply.name} contains no points")
    args = types.SimpleNamespace(frame_window=C.CROP_FRAME_WINDOW, margin=C.CROP_MARGIN_PX)
    try:
        auto = crop_tool.compute_auto_crop(pcd, ply, object_dir, tag_layout(object_dir), args)
    except crop_tool.AutoCropError as exc:
        raise PrepareError(f"crop: {exc}") from None
    cropped = pcd.select_by_index(np.flatnonzero(auto.mask))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    crop_tool.write_ply_like_original(out_path, cropped)
    return {"points_raw": len(pcd.points), "points_cropped": int(auto.mask.sum()),
            "crop_notes": " | ".join(auto.notes)}


def seed_transform(points: np.ndarray, reference_bounds_m) -> tuple[np.ndarray, dict]:
    """4x4 scan -> reference seed (metres): table plane to z = 0 with its
    normal towards the camera as +z, the object's footprint centre over the
    reference's xy centre. Yaw is arbitrary; Evaluate tries quarter turns."""
    pts = points[np.isfinite(points).all(axis=1)]
    sub = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts)).voxel_down_sample(0.004)
    if len(sub.points) < 100:
        raise PrepareError("seed: too few points to fit the table plane")
    (a, b, c, d), inliers = sub.segment_plane(0.003, 3, 2000)
    n = np.array([a, b, c])
    s = np.linalg.norm(n)
    n, d = n / s, d / s
    if d < 0:  # camera (origin) on the positive side
        n, d = -n, -d
    h = pts @ n + d
    raised = pts[(h > C.SEED_MIN_HEIGHT_M) & (h < C.SEED_MAX_HEIGHT_M)]
    if len(raised) < 50:
        raise PrepareError("seed: no object found above the table plane")
    labels = np.asarray(o3d.geometry.PointCloud(o3d.utility.Vector3dVector(raised))
                        .cluster_dbscan(C.SEED_CLUSTER_EPS_M, 20))
    sizes = {lab: int((labels == lab).sum()) for lab in set(labels.tolist()) - {-1}}
    # Ignore fragments far smaller than the biggest cluster: at 641 mm a 74-point
    # speck 40 mm off axis beat the 10k-point part 44 mm off axis.
    min_size = max(50, 0.2 * max(sizes.values(), default=0))
    best = None
    for lab, size in sizes.items():
        if size < min_size:
            continue
        q = raised[labels == lab]
        # The camera looks at the part: take the cluster nearest the optical axis.
        off_axis = np.linalg.norm(np.cross(q.mean(axis=0), [0.0, 0.0, 1.0]))
        if best is None or off_axis < best[0]:
            best = (off_axis, q)
    if best is None:
        raise PrepareError("seed: no object cluster found above the table plane")
    obj = best[1]

    e1 = np.cross(n, [0.0, 1.0, 0.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    R = np.vstack([e1, e2, n])
    uv = obj @ np.vstack([e1, e2]).T
    lo, hi = np.percentile(uv, 1, axis=0), np.percentile(uv, 99, axis=0)
    centre = (lo + hi) / 2
    rlo, rhi = reference_bounds_m
    target = (np.asarray(rlo)[:2] + np.asarray(rhi)[:2]) / 2
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [target[0] - centre[0], target[1] - centre[1], d]
    info = {"seed_object_points": int(len(obj)),
            "seed_footprint_x_mm": round(float(hi[0] - lo[0]) * 1e3, 1),
            "seed_footprint_y_mm": round(float(hi[1] - lo[1]) * 1e3, 1),
            "seed_table_inliers": int(len(inliers))}
    return T, info
