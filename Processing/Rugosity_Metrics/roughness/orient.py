"""Orient a scan so the bed is the XY plane, then crop it.

Scans arrive in an arbitrary camera / fusion frame. A robust plane fit
(PCA with sigma-clipping) gives the bed's normal; the scan is rotated so that
normal is +Z, translated so the plane-fit centroid is the origin, and the
in-plane axes become x and y. Everything after this works in that frame.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy import ndimage

from scan_eval.log import log, warn


@dataclass
class Frame:
    """Scan frame -> oriented frame: p_oriented = (p - centroid) @ R.T"""
    R: np.ndarray            # 3x3, rows = oriented x, y, z axes in scan coordinates
    centroid: np.ndarray     # 3, metres, scan frame
    tilt_deg: float          # angle between the plane normal and the nearest input +-Z
    plane_rms: float         # RMS distance of plane-fit inliers to the plane, m
    inlier_fraction: float

    def apply(self, points: np.ndarray) -> np.ndarray:
        return (np.asarray(points) - self.centroid) @ self.R.T

    def as_dict(self):
        return {"rotation": self.R, "centroid_mm": self.centroid * 1e3,
                "tilt_removed_deg": self.tilt_deg, "plane_rms_mm": self.plane_rms * 1e3,
                "plane_inlier_fraction": self.inlier_fraction}


def robust_sigma(r: np.ndarray) -> float:
    """MAD-based standard deviation: ignores the tails that sigma-clipping
    is meant to reject."""
    return float(1.4826 * np.median(np.abs(r - np.median(r))))


def fit_plane(points: np.ndarray, clip_sigma: float, iterations: int):
    """Robust total-least-squares plane: (centroid, unit normal, inlier mask)."""
    keep = np.ones(len(points), dtype=bool)
    for _ in range(max(1, iterations)):
        c = points[keep].mean(axis=0)
        q = points[keep] - c
        _, vecs = np.linalg.eigh(q.T @ q)
        n = vecs[:, 0]
        r = (points - c) @ n
        sigma = robust_sigma(r[keep])
        if sigma <= 0:
            break
        new = np.abs(r) <= clip_sigma * sigma
        if new.sum() < 3 or np.array_equal(new, keep):
            break
        keep = new
    c = points[keep].mean(axis=0)
    q = points[keep] - c
    vals, vecs = np.linalg.eigh(q.T @ q)
    return c, vecs[:, 0], vecs[:, 2], keep


def parse_origin(text):
    if text is None or str(text).strip().lower() in ("", "none"):
        return None
    try:
        v = np.array([float(t) for t in str(text).split(",")])
    except ValueError:
        raise ValueError(f"cannot read sensor_origin '{text}' (use x,y,z in mm, or none)") from None
    if v.shape != (3,):
        raise ValueError(f"sensor_origin needs three numbers, got '{text}'")
    return v * 1e-3


def identity_frame(points: np.ndarray) -> tuple[np.ndarray, Frame]:
    """--orient none: the input frame is kept, so coordinates stay those of
    the reference the scan was aligned to. The detrend still removes any
    leftover tilt."""
    frame = Frame(np.eye(3), np.zeros(3), 0.0, float("nan"), 1.0)
    log("[orient] none: keeping the input frame (scan already aligned, e.g. to a reference)")
    return np.asarray(points, dtype=float).copy(), frame


def orient(points: np.ndarray, params) -> tuple[np.ndarray, Frame]:
    if params.orientation == "none":
        return identity_frame(points)
    c, n, major, keep = fit_plane(points, params.orient_clip_sigma, params.orient_iterations)
    origin = parse_origin(params.sensor_origin)
    side = None if origin is None else float((origin - c) @ n)
    if side is not None and abs(side) > 1e-3:
        if side < 0:
            n = -n
    else:
        if side is not None:
            warn(f"sensor origin {params.sensor_origin} mm lies on the fitted plane - cannot tell "
                 "which side is up; using +Z of the input frame instead (set --sensor-origin)")
        if n[2] < 0:
            n = -n
    if params.inplane_axis == "pca":
        ex = major - (major @ n) * n
    else:
        ref = np.array([1.0, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1.0, 0])
        ex = ref - (ref @ n) * n
    ex /= np.linalg.norm(ex)
    ey = np.cross(n, ex)
    R = np.vstack([ex, ey, n])
    tilt = float(np.degrees(np.arccos(np.clip(abs(n[2]), 0, 1))))
    oriented = (points - c) @ R.T
    rms = float(np.sqrt(np.mean(oriented[keep, 2] ** 2)))
    frame = Frame(R, c, tilt, rms, float(keep.mean()))
    log(f"[orient] plane normal (scan frame) ({n[0]:+.4f}, {n[1]:+.4f}, {n[2]:+.4f}), "
        f"{'facing the sensor' if side is not None and abs(side) > 1e-3 else 'input +Z side up'}")
    log(f"[orient] tilt removed {tilt:.2f} deg; plane RMS {rms * 1e3:.3f} mm over "
        f"{keep.mean():.1%} inliers")
    if keep.mean() < 0.3:
        warn(f"the levelling plane fit kept only {keep.mean():.0%} of the points: the scan is not "
             "mostly a flat bed (an object, a table and a bed together?). Crop with --roi / "
             "--pick-roi, or use --orient none for a scan already aligned to a reference")
    return oriented, frame


def point_spacing(xy: np.ndarray, rng, n_query: int = 20000) -> float:
    """Median xy distance to the nearest neighbour, from a seeded subsample."""
    from scipy.spatial import cKDTree
    tree = cKDTree(xy)
    q = xy if len(xy) <= n_query else xy[rng.choice(len(xy), n_query, replace=False)]
    d, _ = tree.query(q, k=2)
    d = d[:, 1]
    d = d[d > 0]
    if len(d) == 0:
        raise ValueError("all points coincide in xy - cannot estimate the point spacing")
    return float(np.median(d))


def round_sig(x: float, digits: int = 2) -> float:
    if x <= 0:
        return x
    return float(round(x, -int(np.floor(np.log10(x))) + digits - 1))


def _empty_crop(what, points):
    lo, hi = points[:, :2].min(axis=0) * 1e3, points[:, :2].max(axis=0) * 1e3
    return ValueError(f"{what} contains no points; the scan spans x {lo[0]:.1f}..{hi[0]:.1f}, "
                      f"y {lo[1]:.1f}..{hi[1]:.1f} mm in the analysis frame")


def crop(points: np.ndarray, params, grid: float, region: np.ndarray | None = None) -> tuple[np.ndarray, dict]:
    """Reference footprint, ROI rectangle, ROI polygon and edge trim, in the
    analysis frame. Returns the keep mask and a summary. `region` is a
    per-point mask (the reference's outline). The edge trim measures distance
    to the footprint's outer border (interior holes do not count as border)
    on an occupancy grid."""
    keep = np.ones(len(points), dtype=bool)
    info = {"n_before": int(len(points))}
    if region is not None:
        keep &= region
        info["reference_footprint_points"] = int(region.sum())
        log(f"[crop] reference footprint keeps {region.sum()} of {len(points)} points")
        if keep.sum() == 0:
            raise ValueError("no scan point lies over the reference's outline - is the scan aligned "
                             "into the reference frame (e.g. Evaluate's aligned_error.ply)? Or pass "
                             "--reference-crop none")
    if params.roi_mm:
        x0, y0, x1, y1 = (v * 1e-3 for v in params.roi_mm)
        keep &= (points[:, 0] >= x0) & (points[:, 0] <= x1) & (points[:, 1] >= y0) & (points[:, 1] <= y1)
        info["roi_mm"] = list(params.roi_mm)
        log(f"[crop] ROI x {params.roi_mm[0]:g}..{params.roi_mm[2]:g}, y {params.roi_mm[1]:g}.."
            f"{params.roi_mm[3]:g} mm keeps {keep.sum()} of {len(points)} points")
        if keep.sum() == 0:
            raise _empty_crop(f"--roi {params.roi_mm}", points)
    if params.roi_polygon_mm:
        from matplotlib.path import Path as PolyPath
        poly = np.asarray(params.roi_polygon_mm, dtype=float) * 1e-3
        inside = PolyPath(poly).contains_points(points[:, :2])
        keep &= inside
        info["roi_polygon_mm"] = [list(v) for v in params.roi_polygon_mm]
        log(f"[crop] ROI polygon ({len(poly)} vertices) keeps {keep.sum()} of {len(points)} points")
        if keep.sum() == 0:
            raise _empty_crop("the ROI polygon", points)
    if params.edge_trim_mm > 0:
        p = points[keep]
        lo = p[:, :2].min(axis=0)
        ij = np.floor((p[:, :2] - lo) / grid).astype(np.int64)
        shape = ij.max(axis=0) + 1
        occ = np.zeros((shape[1], shape[0]), dtype=bool)
        occ[ij[:, 1], ij[:, 0]] = True
        foot = ndimage.binary_fill_holes(occ)
        dist = ndimage.distance_transform_edt(np.pad(foot, 1))[1:-1, 1:-1]
        inside = (dist - 0.5) * grid >= params.edge_trim_mm * 1e-3
        sub = inside[ij[:, 1], ij[:, 0]]
        idx = np.flatnonzero(keep)
        keep[idx[~sub]] = False
        log(f"[crop] edge trim {params.edge_trim_mm:g} mm drops {int((~sub).sum())} border points")
        if keep.sum() == 0:
            raise ValueError(f"--edge-trim-mm {params.edge_trim_mm:g} removed every point; the "
                             "footprint is narrower than twice the trim")
    info["n_after"] = int(keep.sum())
    return keep, info
