"""Rigid (optionally similarity) alignment of a scan to a reference surface.

Ported from Fuse (fusion_pipeline.cpp / fusion_session.cpp) rather than
rewritten, keeping its structure and its conclusions:

  * preprocessing for alignment only - voxel downsample then statistical
    outlier removal (preprocessCloud), normals oriented toward the sensor at
    the scan frame's origin (estimateSensorNormals);
  * FPFH + RANSAC feature matching for the coarse pose (coarseAlignFPFH),
    point-to-plane ICP to refine it (refineICP);
  * competing hypotheses are *scored, not trusted*: every candidate pose is
    rated by one measure independent of the algorithm that produced it, and
    a winner whose margin over the runner-up is below 15% is flagged as
    ambiguous (direction_ambiguous_margin). The measure itself had to change
    - see Scorer for why rmse / overlap picks the wrong pose here.

What is new here is the guard against symmetric / periodic references
(egg-carton, sinusoids, spheres, planes): several seeded RANSAC starts plus
systematic perturbations of the best pose (quarter turns about the surface
normal, slides of a quarter and half of the part's extent, upside-down) are
all refined and scored, and the overlap in the score is *symmetric* - how
much of the scan lies on the reference *and* how much of the reference is
covered by the scan - so sliding the scan off the part's edges, boundaries or
base plate costs score even where the relief repeats. The final pose is then
polished against the exact surface (not samples of it), and the normal
equations of that last step are analysed for degrees of freedom the surface
does not constrain.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from .log import debug, log, warn

reg = o3d.pipelines.registration


# --------------------------------------------------------------------------
# small transform helpers
def apply(T: np.ndarray, points: np.ndarray) -> np.ndarray:
    return points @ T[:3, :3].T + T[:3, 3]


def rotation_about(axis: np.ndarray, angle: float, centre: np.ndarray) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = centre - R @ centre
    return T


def translation(v: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = v
    return T


def rotation_angle_deg(R: np.ndarray) -> float:
    s = np.linalg.det(R) ** (1 / 3) if np.linalg.det(R) > 0 else 1.0
    c = (np.trace(R / s) - 1) / 2
    return float(np.degrees(np.arccos(np.clip(c, -1, 1))))


# --------------------------------------------------------------------------
@dataclass
class Candidate:
    name: str
    transform: np.ndarray
    score: float = np.inf
    rmse: float = np.inf
    overlap_scan: float = 0.0
    overlap_ref: float = 0.0
    starts: list = field(default_factory=list)

    @property
    def overlap(self) -> float:
        return float(np.sqrt(self.overlap_scan * self.overlap_ref))

    def as_dict(self):
        return {"name": self.name, "score": self.score, "rmse_mm": self.rmse * 1e3,
                "overlap_scan": self.overlap_scan, "overlap_ref": self.overlap_ref,
                "starts": list(self.starts), "transform": self.transform.tolist()}


@dataclass
class AlignmentResult:
    transform: np.ndarray                   # scan (metres) -> reference frame (metres)
    scale: float                            # recovered scale (1.0 unless --allow-scale)
    best: Candidate | None
    runner_up: Candidate | None
    margin: float | None                    # (runner_up - best) / runner_up score
    ambiguous: bool
    candidates: list
    degeneracy: dict
    refine: dict
    voxel: float
    eval_distance: float


# --------------------------------------------------------------------------
def to_pcd(points, normals=None) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.asarray(points, dtype=float)))
    if normals is not None:
        pcd.normals = o3d.utility.Vector3dVector(np.asarray(normals, dtype=float))
    return pcd


def prepare_scan(points: np.ndarray, voxel: float, sor_neighbours: int, sor_std: float,
                 sensor_origin: np.ndarray | None = None):
    """Fuse's preprocessCloud + estimateSensorNormals: voxel first (cheaper, and
    gives SOR a saner neighbourhood density), then SOR, then normals oriented
    toward the sensor at the scan frame's origin. Alignment copy only - the
    metrics use every original point."""
    pcd = to_pcd(points).voxel_down_sample(voxel)
    if sor_neighbours > 0 and len(pcd.points) > sor_neighbours * 2:
        pcd, _ = pcd.remove_statistical_outlier(sor_neighbours, sor_std)
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=4 * voxel, max_nn=30))
    if sensor_origin is not None:
        pcd.orient_normals_towards_camera_location(sensor_origin)
    return pcd


def parse_sensor_origin(text) -> np.ndarray | None:
    if text is None or str(text).strip().lower() == "none":
        return None
    try:
        v = np.array([float(t) for t in str(text).split(",")])
    except ValueError:
        v = np.zeros(0)
    if v.shape != (3,):
        raise ValueError(f"--sensor-origin expects x,y,z in mm or 'none' (got '{text}')")
    return v * 1e-3


def fpfh(pcd, voxel):
    return reg.compute_fpfh_feature(
        pcd, o3d.geometry.KDTreeSearchParamHybrid(radius=10 * voxel, max_nn=100))


def ransac(src, tgt, f_src, f_tgt, voxel, seed, max_iterations, confidence):
    """Fuse's coarseAlignFPFH: RANSAC over FPFH correspondences with an
    edge-length pre-rejection (PCL's similarity threshold 0.9)."""
    o3d.utility.random.seed(int(seed))
    dist = 2.0 * voxel
    result = reg.registration_ransac_based_on_feature_matching(
        src, tgt, f_src, f_tgt, True, dist,
        reg.TransformationEstimationPointToPoint(False), 3,
        [reg.CorrespondenceCheckerBasedOnEdgeLength(0.9),
         reg.CorrespondenceCheckerBasedOnDistance(dist)],
        reg.RANSACConvergenceCriteria(max_iterations, confidence))
    return np.asarray(result.transformation)


def icp(src, tgt, T0, voxel, max_iterations):
    """Fuse's refineICP (point-to-plane, target normals only) run as a
    coarse-to-fine schedule so a seed a few voxels off still converges."""
    T = np.asarray(T0, dtype=float)
    for factor in (6.0, 3.0, 1.5):
        result = reg.registration_icp(
            src, tgt, factor * voxel, T, reg.TransformationEstimationPointToPlane(),
            reg.ICPConvergenceCriteria(relative_fitness=1e-7, relative_rmse=1e-7,
                                       max_iteration=max_iterations))
        T = np.asarray(result.transformation)
    return T


class Scorer:
    """Fuse's evaluateAlignment, made symmetric and made honest. For a pose T:

    overlap_scan  fraction of the scan points lying over the part (inside the
                  reference's region) that are within the evaluation distance
                  of the surface;
    overlap_ref   fraction of visible reference samples with a scan point
                  within the evaluation distance;
    rmse          RMS point-to-plane residual of the scan inliers (reported);
    score         truncated least squares over *both* directions, lower is
                  better: every point costs min(d, tau)^2, so a point that
                  does not match costs tau^2 instead of vanishing.

    Fuse ranks by rmse / overlap. Measured on real egg-carton scans that
    picks a pose a quarter period off: its rmse over inliers is lower
    precisely because it has fewer of them, and that outweighs its lower
    overlap. The truncated cost cannot be improved by discarding points.
    Scan points outside the part's region (table, other objects) are left out
    of the scan-side term entirely, so clutter cannot dilute the comparison.
    """

    def __init__(self, ref, scan_points, scan_normals, ref_points, ref_normals, eval_distance,
                 region_margin, max_samples=20000):
        stride = max(1, len(scan_points) // max_samples)
        self.ref = ref
        self.scan = scan_points[::stride]
        self.scan_normals = None if scan_normals is None else scan_normals[::stride]
        self.scan_tree = cKDTree(scan_points)
        self.ref_tree = cKDTree(ref_points)
        self.ref_points = ref_points
        self.ref_normals = ref_normals
        r_stride = max(1, len(ref_points) // max_samples)
        self.ref_sub = ref_points[::r_stride]
        self.ref_sub_normals = ref_normals[::r_stride]
        self.d = eval_distance
        self.margin = region_margin

    def score(self, cand: Candidate) -> Candidate:
        T = cand.transform
        tau2 = self.d**2
        moved = apply(T, self.scan)
        over = self.ref.region_mask(moved, self.margin)
        _, idx = self.ref_tree.query(moved[over], workers=-1)
        offset = moved[over] - self.ref_points[idx]
        plane = np.abs(np.einsum("ij,ij->i", offset, self.ref_normals[idx]))
        # The scan's normals face its sensor; the reference's face the side a
        # scanner can see. A match seen from the wrong side is no match - this
        # is what tells a thin surface right way up from upside down.
        if self.scan_normals is None:
            facing = np.ones(len(idx), dtype=bool)
        else:
            facing = np.einsum("ij,ij->i", self.scan_normals[over] @ T[:3, :3].T, self.ref_normals[idx]) > 0
        inlier = (np.linalg.norm(offset, axis=1) < self.d) & facing
        scan_cost = np.where(inlier, np.minimum(plane**2, tau2), tau2)

        # Reference -> scan, in the scan's frame. Point-to-plane along the
        # reference's exact normal, so the scan's own point spacing does not
        # count as error.
        Tinv = np.linalg.inv(T)
        q = apply(Tinv, self.ref_sub)
        nq = self.ref_sub_normals @ Tinv[:3, :3].T
        dist, j = self.scan_tree.query(q, workers=-1, distance_upper_bound=self.d)
        found = np.isfinite(dist)
        ref_cost = np.full(len(q), tau2)
        if found.any():
            gap = self.scan_tree.data[j[found]] - q[found]
            ref_cost[found] = np.minimum(np.einsum("ij,ij->i", gap, nq[found]) ** 2, tau2)

        cand.overlap_scan = float(inlier.mean()) if over.any() else 0.0
        cand.overlap_ref = float(found.mean())
        cand.rmse = float(np.sqrt(np.mean(plane[inlier] ** 2))) if inlier.any() else np.inf
        scan_term = scan_cost.mean() if over.any() else tau2
        cand.score = float(np.sqrt(0.5 * (scan_term + ref_cost.mean())))
        return cand


def same_pose(Ta, Tb, corners, tolerance) -> bool:
    return float(np.max(np.linalg.norm(apply(Ta, corners) - apply(Tb, corners), axis=1))) < tolerance


def reference_frame(points, normals, visible_from):
    """Fuse's estimatePlaneFrame on the reference samples: centroid, in-plane
    axes (largest spread first) and the plane normal (least spread)."""
    centre = points.mean(axis=0)
    w, v = np.linalg.eigh(np.cov((points - centre).T))
    n = v[:, 0]
    up = visible_from if visible_from is not None else normals.mean(axis=0)
    if np.linalg.norm(up) > 1e-9 and n @ up < 0:
        n = -n
    u = v[:, 2]
    return centre, u, np.cross(n, u), n



# --------------------------------------------------------------------------
def refine_exact(ref, scan_points, T0, region_margin, corr, allow_scale, max_iterations=50):
    """Point-to-plane ICP against the exact surface: every iteration asks the
    reference for true closest points and normals, so there is no sampling
    error left in the final pose. Robust to stragglers via MAD trimming.
    Returns the pose, the recovered scale, and the normal-equation matrix of
    the final iteration for the degeneracy analysis."""
    T = np.asarray(T0, dtype=float).copy()
    info = {"iterations": 0, "points_used": 0, "converged": False}
    J = None
    for it in range(max_iterations):
        Q = apply(T, scan_points)
        C, N = ref.closest_points(Q)
        offset = Q - C
        r = np.einsum("ij,ij->i", offset, N)
        keep = (np.linalg.norm(offset, axis=1) < corr) & ref.region_mask(Q, region_margin)
        if keep.sum() < 20:
            warn(f"exact refinement: only {int(keep.sum())} scan points lie on the reference - "
                 "keeping the coarse pose")
            break
        med = np.median(r[keep])
        mad = 1.4826 * np.median(np.abs(r[keep] - med))
        keep &= np.abs(r - med) < max(3.0 * mad, 1e-6)
        X = Q[keep] - Q[keep].mean(axis=0)
        n = N[keep]
        cols = [np.cross(X, n), n]
        if allow_scale:
            cols.append(np.einsum("ij,ij->i", X, n)[:, None])
        J = np.hstack(cols)
        rms = float(np.sqrt(np.mean(r[keep] ** 2)))
        if it > 0 and rms > 1.5 * info["residual_rms"] + 1e-9:
            # The last step made things worse (can happen along a direction
            # the surface barely constrains): undo it and stop.
            T = T_prev
            break
        # Balance the columns (rotation x radius ~ displacement) and drop
        # directions the surface does not constrain, rather than letting a
        # near-zero eigenvalue turn noise into a huge step - a sphere or a
        # plane has exact null directions here.
        L = max(float(np.sqrt(np.mean(np.sum(X**2, axis=1)))), 1e-12)
        D = np.ones(J.shape[1])
        D[:3] = 1.0 / L
        if allow_scale:
            D[6] = 1.0 / L
        Js = J * D
        x = -np.linalg.lstsq(Js.T @ Js, Js.T @ r[keep], rcond=1e-6)[0] * D
        w, t = x[:3], x[3:6]
        T_prev = T.copy()
        s = 1.0 + (x[6] if allow_scale else 0.0)
        angle = np.linalg.norm(w)
        R = rotation_about(w / angle, angle, np.zeros(3))[:3, :3] if angle > 1e-15 else np.eye(3)
        c0 = Q[keep].mean(axis=0)
        M = np.eye(4)
        M[:3, :3] = s * R
        M[:3, 3] = c0 + t - s * R @ c0
        T = M @ T
        info.update(iterations=it + 1, points_used=int(keep.sum()), residual_rms=rms)
        # Stop when the update moves the points by under 2 um - far below any
        # depth camera's noise. Tighter is pointless: against a mesh the
        # closest-point queries (float32, and switching between faces) make
        # the pose dither at about a micron indefinitely.
        radius = float(np.sqrt(np.mean(np.sum(X**2, axis=1))))
        motion = np.linalg.norm(t) + angle * radius + abs(s - 1) * radius
        if motion < 2e-6:
            info["converged"] = True
            break
    scale = float(np.linalg.det(T[:3, :3]) ** (1 / 3))
    return T, scale, J, info


_DOF_NAMES = ["rotation about X", "rotation about Y", "rotation about Z",
              "translation along X", "translation along Y", "translation along Z", "scale"]


def degeneracy(J: np.ndarray | None, points_rms_radius: float, weak_ratio: float,
               unconstrained_ratio: float) -> dict:
    """Which degrees of freedom the surface does not pin down.

    The rotation columns of the point-to-plane Jacobian are scaled by the RMS
    radius of the points about their centroid, so a unit of rotation and a
    unit of translation move the surface by comparable amounts, and the
    eigenvalues of J^T J are comparable. An eigenvalue that is tiny relative
    to the largest is a direction the surface cannot resolve; each canonical
    DOF (rotation about X, ...) lying mostly inside that subspace is named.
    """
    if J is None or len(J) < J.shape[1]:
        return {"available": False}
    Js = J.copy()
    Js[:, :3] /= max(points_rms_radius, 1e-12)
    if Js.shape[1] > 6:
        Js[:, 6] /= max(points_rms_radius, 1e-12)
    H = Js.T @ Js / len(Js)
    lam, vec = np.linalg.eigh(H)
    ratios = lam / lam[-1]
    report = {"available": True, "eigenvalue_ratios": ratios.tolist(),
              "condition": float(lam[-1] / max(lam[0], 1e-300)), "weak": [], "unconstrained": []}
    for level, bound in (("unconstrained", unconstrained_ratio), ("weak", weak_ratio)):
        sub = vec[:, ratios < bound]
        if sub.shape[1] == 0:
            continue
        named = []
        for i in range(H.shape[0]):
            share = float(np.sum(sub[i, :] ** 2))
            if share > 0.8:
                named.append(_DOF_NAMES[i])
        if not named:
            for k in range(sub.shape[1]):
                v = sub[:, k]
                rot, tr = v[:3], v[3:6]
                named.append(f"coupled motion: rotation axis {np.round(rot / max(np.linalg.norm(rot), 1e-12), 2).tolist()}"
                             f" with translation {np.round(tr / max(np.linalg.norm(tr), 1e-12), 2).tolist()}")
        report[level] = named
    # A DOF already reported unconstrained need not be repeated as weak.
    report["weak"] = [d for d in report["weak"] if d not in report["unconstrained"]]
    return report


# --------------------------------------------------------------------------
def align(ref, scan_points: np.ndarray, params, seed: int, init_transform=None) -> AlignmentResult:
    """Full alignment: scan (metres, own frame) -> reference frame (metres)."""
    lo, hi = ref.bounds()
    diag = float(np.linalg.norm(hi - lo))
    voxel = params.align_voxel_mm * 1e-3 if params.align_voxel_mm > 0 else diag / 60
    eval_distance = params.eval_distance_mm * 1e-3 if params.eval_distance_mm > 0 else 3 * voxel
    region_margin = params.region_margin_mm * 1e-3
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    log(f"[align] voxel {voxel * 1e3:.2f} mm, evaluation distance {eval_distance * 1e3:.2f} mm "
        f"(reference diagonal {diag * 1e3:.1f} mm)")

    # Reference samples: visible surface only, with exact normals.
    rp, rn = ref.sample(max(params.reference_samples // 2, 20000), seed)
    vis = ref.visible_mask(rp, rn)
    rp, rn = rp[vis], rn[vis]
    ref_pcd = to_pcd(rp, rn)
    ref_icp = ref_pcd.voxel_down_sample(voxel / 2)
    ref_icp.normalize_normals()
    sensor = parse_sensor_origin(params.sensor_origin)
    scan_pcd = prepare_scan(scan_points, voxel, params.sor_neighbours, params.sor_std, sensor)
    scan_ds = np.asarray(scan_pcd.points)
    if len(scan_ds) < 10:
        raise ValueError(f"only {len(scan_ds)} scan points left after downsampling at "
                         f"{voxel * 1e3:.2f} mm - the scan is (nearly) empty or --align-voxel is too large")
    debug(f"[align] scan {len(scan_points)} -> {len(scan_ds)} points for alignment; "
          f"reference {len(rp)} visible samples")
    scorer = Scorer(ref, scan_ds, None if sensor is None else np.asarray(scan_pcd.normals),
                    rp, rn, eval_distance, region_margin)

    candidates: list[Candidate] = []

    def add(name, T, refine=True):
        T = icp(scan_pcd, ref_icp, T, voxel, params.icp_max_iterations) if refine else np.asarray(T, float)
        cand = scorer.score(Candidate(name, T, starts=[name]))
        for other in candidates:
            if same_pose(other.transform, cand.transform, corners, eval_distance):
                other.starts.append(name)
                if cand.score < other.score:
                    other.transform, other.score, other.rmse = cand.transform, cand.score, cand.rmse
                    other.overlap_scan, other.overlap_ref = cand.overlap_scan, cand.overlap_ref
                return other
        candidates.append(cand)
        return cand

    mode = params.align
    if mode == "none":
        T = np.eye(4) if init_transform is None else init_transform
        best = scorer.score(Candidate("given", np.asarray(T, float), starts=["given"]))
        log("[align] --align none: using the given transform as-is")
        return AlignmentResult(T, float(np.linalg.det(T[:3, :3]) ** (1 / 3)), best, None, None,
                               False, [best], {"available": False}, {}, voxel, eval_distance)

    if init_transform is not None:
        add("init-transform", init_transform)
    add("identity", np.eye(4))
    centre, u, v, n = reference_frame(rp, rn, ref.visible_from)

    if mode == "full":
        # Moments: scan centroid onto the reference's, principal axes onto its
        # principal axes, with the scan's plane normal pointing toward its
        # sensor matched to the reference's visible side. Crude when the scan
        # holds clutter, but the one start that works on a featureless shape
        # (sphere, plane, cylinder) where FPFH has nothing to match; the turns
        # and flip below cover the axis-sign ambiguity.
        s_centre, s_u, s_v, s_n = reference_frame(scan_ds, np.zeros_like(scan_ds), None)
        if sensor is not None and s_n @ (sensor - s_centre) < 0:
            s_n, s_v = -s_n, -s_v
        R = np.column_stack([u, v, n]) @ np.column_stack([s_u, s_v, s_n]).T
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = centre - R @ s_centre
        add("moments", T)

    if mode == "full":
        ref_fp = ref_pcd.voxel_down_sample(voxel)
        ref_fp.normalize_normals()
        f_ref = fpfh(ref_fp, voxel)
        f_scan = fpfh(scan_pcd, voxel)
        for k in range(params.ransac_starts):
            T = ransac(scan_pcd, ref_fp, f_scan, f_ref, voxel, seed + k,
                       params.ransac_max_iterations, params.ransac_confidence)
            add(f"ransac-{k}", T)

        # Perturbations of the best pose so far, in the reference frame.
        if params.perturb:
            leader = min(candidates, key=lambda c: c.score)
            extent_u = np.ptp(rp @ u)
            extent_v = np.ptp(rp @ v)
            moves = [(f"turn-{a}", rotation_about(n, np.radians(a), centre)) for a in (90, 180, 270)]
            moves.append(("upside-down", rotation_about(u, np.pi, centre)))
            for frac in (0.25, 0.5):
                for sign in (1, -1):
                    moves.append((f"slide-u{sign * frac:+.2f}", translation(sign * frac * extent_u * u)))
                    moves.append((f"slide-v{sign * frac:+.2f}", translation(sign * frac * extent_v * v)))
            for name, P in moves:
                add(f"{leader.name}/{name}", P @ leader.transform)

    candidates.sort(key=lambda c: c.score)
    best = candidates[0]
    runner = candidates[1] if len(candidates) > 1 else None
    margin = None
    ambiguous = False
    failed = best.overlap_scan == 0 or not np.isfinite(best.rmse)
    if not failed and runner is not None and np.isfinite(runner.score) and runner.score > 0:
        margin = (runner.score - best.score) / runner.score
        ambiguous = margin < params.ambiguous_margin

    # Final polish against the exact surface, on original points: voxel
    # centroids sit inside convex relief (and outside concave), which would
    # bias the pose by a fraction of the voxel.
    stride = max(1, len(scan_points) // 100_000)
    raw = scan_points[::stride]
    T, scale, J, info = refine_exact(ref, raw, best.transform, region_margin,
                                     corr=eval_distance, allow_scale=params.allow_scale)
    Q = apply(T, raw)
    on = ref.region_mask(Q, region_margin) & (ref.distance(Q) < eval_distance)
    rms_radius = float(np.sqrt(np.mean(np.sum((Q[on] - Q[on].mean(0)) ** 2, axis=1)))) if on.any() else 1.0
    degen = degeneracy(J, rms_radius, params.degeneracy_weak, params.degeneracy_unconstrained)
    return AlignmentResult(T, scale, best, runner, margin, ambiguous, candidates, degen, info,
                           voxel, eval_distance)


def print_alignment_report(result: AlignmentResult, params):
    if result.best is not None and (result.best.overlap_scan == 0 or not np.isfinite(result.best.rmse)):
        warn("ALIGNMENT FAILED: no candidate pose puts any of the scan on the reference. "
             "Check the units, that the scan contains the part, or seed with --init-transform.")
    log("[align] candidate poses (after ICP), best first:")
    log("    #  candidate                          cost     rmse   overlap scan/ref  starts")
    for i, c in enumerate(result.candidates[:8]):
        log(f"   {i:2d}  {c.name[:32]:32s}  {c.score * 1e3:7.3f}  {c.rmse * 1e3:5.2f}mm"
            f"   {c.overlap_scan:5.1%} / {c.overlap_ref:5.1%}   {len(c.starts)}")
    if len(result.candidates) > 8:
        log(f"   ... {len(result.candidates) - 8} more distinct pose(s)")
    if result.runner_up is not None and result.margin is not None:
        b, r = result.best, result.runner_up
        log(f"[align] chosen {b.name}: rmse {b.rmse * 1e3:.3f} mm, overlap {b.overlap_scan:.1%} scan / "
            f"{b.overlap_ref:.1%} ref;  runner-up {r.name}: rmse {r.rmse * 1e3:.3f} mm, overlap "
            f"{r.overlap_scan:.1%} / {r.overlap_ref:.1%};  margin {result.margin:.0%}")
        if result.ambiguous:
            warn(f"[AMBIGUOUS] the best two alignments are too close to call (margin "
                 f"{result.margin:.0%} < {params.ambiguous_margin:.0%}). The reference may be "
                 "symmetric or periodic over the part the scan covers. Check the aligned .ply "
                 "visually, or seed the alignment with --init-transform.")
    info = result.refine
    if info:
        log(f"[align] exact-surface refinement: {info.get('iterations', 0)} iterations, "
            f"{info.get('points_used', 0)} points, residual rms "
            f"{info.get('residual_rms', float('nan')) * 1e3:.3f} mm"
            + ("" if info.get("converged") else "  (did not fully converge)"))
    if params.allow_scale:
        log(f"[align] recovered scale {result.scale:.5f} ({(result.scale - 1) * 100:+.3f}%)")
    d = result.degeneracy
    if d.get("available"):
        if d["unconstrained"]:
            warn("alignment is UNCONSTRAINED in: " + "; ".join(d["unconstrained"]) +
                 " - the reference surface cannot fix these, so they (and any error "
                 "along them) are invisible to the metrics")
        if d["weak"]:
            warn("alignment is only weakly constrained in: " + "; ".join(d["weak"]))
        if not d["unconstrained"] and not d["weak"]:
            log(f"[align] all degrees of freedom constrained (condition number {d['condition']:.3g})")
