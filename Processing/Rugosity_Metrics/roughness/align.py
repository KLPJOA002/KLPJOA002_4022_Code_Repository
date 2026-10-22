"""Putting the scan onto the reference before it is compared.

The comparison samples the reference under every scan cell, so the scan has
to sit in the reference's frame. Evaluate's aligned_error.ply (or its
transform.txt via --transform) usually gets it there; a raw fused scan, in
camera coordinates, is first auto-placed. And a symmetric or periodic part
(pyramids, egg carton) can fit almost as well turned 90 deg, slid half a
period, or upside down. So the scan can be moved:

    base (--transform, else identity) -> --flip -> --rotate-deg -> --shift-mm
    -> auto-place, if it is still nowhere near the part (level on the bed,
       find what stands up from it, centre it on the part, best rotation, ICP)
    -> --refine (height match + point-to-surface ICP against the reference)
    -> --adjust (move it by hand in a window, with a live comparison)

and then checks the result (ALIGN, UPSIDEDOWN, ALIGNCOVER). The final
matrix is written as <run>/transform.txt in Evaluate's format.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from scipy import ndimage

from scan_eval.log import log, warn
from scan_eval.registration import apply, refine_exact, rotation_about, rotation_angle_deg, translation

from .grid import GridGeometry, cell_counts, cell_means

X_AXIS, Z_AXIS = np.array([1.0, 0, 0]), np.array([0, 0, 1.0])


def centre_of(reference, points) -> np.ndarray:
    """Pivot for flip / rotate: the reference's bounding-box centre."""
    if reference is not None:
        lo, hi = reference.bounds()
        return 0.5 * (np.asarray(lo) + np.asarray(hi))
    return 0.5 * (points.min(axis=0) + points.max(axis=0))


def pose_matrix(base, flip, rotate_deg, shift_mm, centre) -> np.ndarray:
    """base -> flip (180 deg about x) -> rotate about z -> shift; metres."""
    T = np.asarray(base, dtype=float).copy()
    if flip:
        T = rotation_about(X_AXIS, np.pi, centre) @ T
    if rotate_deg:
        T = rotation_about(Z_AXIS, np.radians(rotate_deg), centre) @ T
    if shift_mm is not None and np.any(shift_mm):
        T = translation(np.asarray(shift_mm, dtype=float) * 1e-3) @ T
    return T


def motion(T_from, T_to, points) -> tuple[float, float]:
    """How far a change of pose moves the points' centroid (mm) and turns them (deg)."""
    c = points.mean(axis=0)
    moved = float(np.linalg.norm(apply(T_to, c[None])[0] - apply(T_from, c[None])[0])) * 1e3
    return moved, rotation_angle_deg(T_to[:3, :3] @ T_from[:3, :3].T)


def subsample(points, n, rng):
    return points if len(points) <= n else points[rng.choice(len(points), n, replace=False)]


def ref_box(reference):
    lo, hi = (np.asarray(v, dtype=float) for v in reference.bounds())
    return lo, hi


def near_part(reference, points, T, margin=0.3):
    """Scan points (scan frame) that T puts over the reference's box, grown by
    `margin` of its size: a raw scan is mostly bed, which says nothing about
    the part's pose."""
    lo, hi = ref_box(reference)
    grow = margin * (hi[:2] - lo[:2])
    Q = apply(T, points)
    sel = np.all((Q[:, :2] >= lo[:2] - grow) & (Q[:, :2] <= hi[:2] + grow), axis=1)
    return points[sel]


# --- how well does it line up? ---------------------------------------------------
class Comparison:
    """A fixed grid over the reference's outline (plus a margin) with the
    reference height precomputed, so a pose can be scored, and drawn, in
    milliseconds. Heights in metres; cell size `g` (m) or `cells` across."""

    def __init__(self, reference, cells=160, margin=0.2, g=None):
        lo, hi = ref_box(reference)
        span = hi[:2] - lo[:2]
        g = float(g or max(span) / cells)
        lo2, hi2 = lo[:2] - margin * span, hi[:2] + margin * span
        self.geom = GridGeometry.from_points(np.array([lo2, hi2]), g)
        cx, cy = self.geom.centres()
        CX, CY = np.meshgrid(cx, cy)
        self.href = reference.height(CX.ravel(), CY.ravel()).reshape(CX.shape)
        self.outline = np.isfinite(self.href)
        self.centre = 0.5 * (lo + hi)
        self.zrange = (lo[2], hi[2])

    def scan_map(self, points):
        idx = self.geom.index(points[:, :2])
        counts = cell_counts(idx, self.geom)
        return cell_means(idx, points[:, 2], counts, self.geom)

    def fill_fraction(self, z) -> float:
        return float((self.outline & np.isfinite(z)).sum() / max(self.outline.sum(), 1))

    def stats(self, points) -> dict:
        return self.stats_of_map(self.scan_map(points))

    def stats_of_map(self, z) -> dict:
        both = self.outline & np.isfinite(z)
        # Coverage: outline cells within 2 cells of a scan point, so a sparse
        # scan on a fine grid still counts as covering the part.
        near = ndimage.binary_dilation(np.isfinite(z), iterations=2)
        out = {"coverage": float((self.outline & near).sum() / max(self.outline.sum(), 1)),
               "median_dz_mm": float("nan"), "rms_dz_mm": float("nan"), "height_corr": float("nan"),
               "reference_sd_mm": float(np.nanstd(self.href)) * 1e3}
        if both.sum() >= 20:
            dz = z[both] - self.href[both]
            out["median_dz_mm"] = float(np.median(np.abs(dz))) * 1e3
            out["rms_dz_mm"] = float(np.sqrt(np.mean(dz ** 2))) * 1e3
            if np.std(self.href[both]) > 0 and np.std(z[both]) > 0:
                out["height_corr"] = float(np.corrcoef(z[both], self.href[both])[0, 1])
        return out


def fill_for_display(z, sigma=1.0, reach=2):
    """Gaps in a sparse height map filled for display only: nan-aware Gaussian
    smoothing (normalised convolution), within `reach` cells of real data."""
    have = np.isfinite(z)
    if not have.any():
        return z
    v = ndimage.gaussian_filter(np.where(have, z, 0.0), sigma)
    w = ndimage.gaussian_filter(have.astype(float), sigma)
    near = ndimage.binary_dilation(have, iterations=reach)
    with np.errstate(invalid="ignore", divide="ignore"):
        filled = np.where(near & (w > 1e-3), v / w, np.nan)
    return np.where(have, z, filled)


def display_cell(reference, points, spacing, target=0.9):
    """Cell size (m) for the comparison window: the finest, from 2 x the point
    spacing upward, at which >= `target` of the outline holds scan points
    (points already near the part, reference frame)."""
    lo, hi = ref_box(reference)
    span = float(max(hi[:2] - lo[:2]))
    g = max(2 * spacing, span / 250)
    while g < span / 40:
        cmp = Comparison(reference, g=g, margin=0.3)
        if cmp.fill_fraction(cmp.scan_map(points)) >= target:
            break
        g *= 1.25
    return min(g, span / 40)


def check(stats, params, reference, T) -> list[tuple[str, str]]:
    """Warnings about the final pose, each naming the fix."""
    out = []
    fix = "move the scan with --adjust, or --flip / --rotate-deg / --shift-mm (then --refine)"
    if stats["coverage"] < 0.5:
        out.append(("ALIGNCOVER", f"the scan covers only {stats['coverage']:.0%} of the reference's outline: "
                    f"it is off the part or only partly over it - {fix}"))
    if np.isfinite(stats["height_corr"]) and stats["height_corr"] < 0:
        out.append(("UPSIDEDOWN", f"scan heights run opposite to the reference's (correlation "
                    f"{stats['height_corr']:+.2f}): the scan looks upside down or mirrored - try --flip "
                    "(f in --adjust)"))
    if params.transform:
        from .orient import parse_origin
        origin = parse_origin(params.sensor_origin)
        if origin is not None:
            cam = apply(T, origin[None])[0]
            up = reference.visible_from if reference.visible_from is not None else Z_AXIS
            lo, hi = ref_box(reference)
            if (cam - 0.5 * (lo + hi)) @ up < 0:
                out.append(("UPSIDEDOWN", f"the camera ({params.sensor_origin} mm in the scan) lands at "
                            f"z = {cam[2] * 1e3:.0f} mm, behind the reference's visible side: the pose is "
                            "upside down - try --flip"))
    limit = max(1.0, 0.5 * stats["reference_sd_mm"])
    if not np.isfinite(stats["median_dz_mm"]) or stats["median_dz_mm"] > limit:
        what = (f"differ by {stats['median_dz_mm']:.2f} mm (median, over {stats['coverage']:.0%} of the outline)"
                if np.isfinite(stats["median_dz_mm"]) else "can't be compared (no scan over the part)")
        out.append(("ALIGN", f"scan and reference heights {what}: the scan is not lined up with the "
                    f"reference, so err_ columns are not meaningful - {fix}"))
    return out


# --- refine (ICP) ----------------------------------------------------------------
def match_height(cmp, points, T):
    """Shift T in z so the scan's median height over the outline matches the
    reference's: after a flip the scan can sit far above or below the part,
    out of ICP's reach."""
    z = cmp.scan_map(apply(T, points))
    both = cmp.outline & np.isfinite(z)
    if both.sum() < 20:
        return T, 0.0
    dz = float(np.median(z[both] - cmp.href[both]))
    return translation(np.array([0.0, 0.0, -dz])) @ T, -dz


def refine(reference, points, T, params, rng, n=20000, cmp=None):
    """Height match, then ICP from T against the exact reference surface, on
    the scan points near the part (scan frame in). If ICP makes the fit worse
    (it wandered off a periodic part) the pre-ICP pose is kept. Returns
    (new T, info)."""
    cmp = cmp or Comparison(reference)
    near = near_part(reference, points, T, margin=0.1)
    sub = subsample(near if len(near) >= 100 else points, n, rng)
    T1, dz = match_height(cmp, sub, T)
    T2, _, _, info = refine_exact(reference, sub, T1, 0.0, params.align_corr_mm * 1e-3, allow_scale=False)
    before, after = cmp.stats(apply(T1, sub)), cmp.stats(apply(T2, sub))
    worse = (not np.isfinite(after["rms_dz_mm"]) or after["coverage"] < 0.8 * before["coverage"]
             or after["rms_dz_mm"] > before["rms_dz_mm"])
    moved, turned = motion(T, T2, sub)
    info = {k: info[k] for k in ("iterations", "points_used", "converged") if k in info} | {
        "height_shift_mm": dz * 1e3, "residual_rms_mm": info.get("residual_rms", float("nan")) * 1e3,
        "moved_mm": moved, "turned_deg": turned, "rejected": bool(worse)}
    if worse:
        warn(f"refine: ICP moved the scan {moved:.1f} mm / {turned:.1f} deg and made the fit worse (RMS dz "
             f"{before['rms_dz_mm']:.2f} -> {after['rms_dz_mm']:.2f} mm, outline covered {before['coverage']:.0%} "
             f"-> {after['coverage']:.0%}), so it is ignored; only the {dz * 1e3:+.2f} mm height match is kept. "
             "Get closer by hand first (--adjust, or --rotate-deg / --shift-mm)")
        return T1, info
    log(f"[align] refine: height {dz * 1e3:+.2f} mm, then {info['iterations']} ICP iterations on "
        f"{info['points_used']} points; moved {moved:.2f} mm / {turned:.2f} deg in all, RMS dz "
        f"{before['rms_dz_mm']:.3f} -> {after['rms_dz_mm']:.3f} mm"
        f"{'' if info.get('converged') else ' (ICP did not fully converge)'}")
    return T2, info


# --- auto-place (raw scans) --------------------------------------------------------
def is_far(reference, points, T, rng) -> bool:
    """True when T leaves the scan nowhere near the part: no scan over the
    outline, or heights off by more than the part is tall."""
    near = near_part(reference, points, T, margin=0.1)
    if len(near) < 100:
        return True
    lo, hi = ref_box(reference)
    st = Comparison(reference, cells=80).stats(apply(T, subsample(near, 100000, rng)))
    return (st["coverage"] < 0.3 or not np.isfinite(st["median_dz_mm"])
            or st["median_dz_mm"] > max(10.0, (hi[2] - lo[2]) * 1e3))


def auto_place(reference, points, T0, params, rng):
    """Coarse pose for a scan that isn't on the part yet (e.g. raw camera
    coordinates): level it on the bed, find what stands up from the bed,
    centre that on the part with its base on the bed, try every 2 deg about
    the vertical, then refine. Returns (T, info)."""
    from .orient import fit_plane, parse_origin
    lo, hi = ref_box(reference)
    P = apply(T0, subsample(points, 400000, rng))
    # 1. level: the bed is the dominant plane; +z towards the sensor.
    c, n, _, keep = fit_plane(P, params.orient_clip_sigma, params.orient_iterations)
    origin = parse_origin(params.sensor_origin)
    if origin is not None:
        origin = apply(T0, origin[None])[0]
        if (origin - c) @ n < 0:
            n = -n
    elif n[2] < 0:
        n = -n
    ref_x = X_AXIS if abs(n[0]) < 0.9 else np.array([0, 1.0, 0])
    ex = ref_x - (ref_x @ n) * n
    ex /= np.linalg.norm(ex)
    R = np.vstack([ex, np.cross(n, ex), n])
    L = np.eye(4)
    L[:3, :3], L[:3, 3] = R, -R @ c
    Q = apply(L, P)
    # 2. what stands up from the bed: the biggest blob on a 2 mm grid.
    height = hi[2] - lo[2]
    thr = max(5e-3, 0.25 * height)
    up = Q[Q[:, 2] > thr]
    if len(up) < 50:
        raise ValueError(f"auto-place: nothing stands more than {thr * 1e3:.0f} mm up from the bed in this scan, "
                         "so the part can't be found. Move the scan by hand (--transform / --shift-mm ...), "
                         "or align it with Evaluate first, or pass --no-auto-place")
    g = 2e-3
    lo_up = up[:, :2].min(axis=0)
    ij = np.floor((up[:, :2] - lo_up) / g).astype(int)
    occ = np.zeros(ij.max(axis=0)[::-1] + 1, dtype=bool)
    occ[ij[:, 1], ij[:, 0]] = True
    occ = ndimage.binary_closing(occ, iterations=2)
    lab, nlab = ndimage.label(occ, structure=np.ones((3, 3), bool))
    sizes = ndimage.sum(occ, lab, range(1, nlab + 1))
    blob = lab == int(np.argmax(sizes)) + 1
    ys, xs = np.nonzero(blob)
    bc = lo_up + (np.array([xs.min() + xs.max() + 1, ys.min() + ys.max() + 1]) / 2) * g
    area, ref_area = blob.sum() * g * g, float(np.prod(hi[:2] - lo[:2]))
    if area < 0.15 * ref_area:
        warn(f"auto-place: the biggest thing standing up from the bed is only {area * 1e6:.0f} mm^2, vs the "
             f"part's {ref_area * 1e6:.0f} mm^2 - the placement may be wrong")
    # 3. blob centre -> part centre, bed -> the part's underside.
    centre = 0.5 * (lo + hi)
    T = translation(np.array([centre[0] - bc[0], centre[1] - bc[1], lo[2]])) @ L @ T0
    # 4. best rotation about the vertical.
    near = near_part(reference, points, T, margin=0.3)
    sub = subsample(near, 60000, rng)
    coarse = Comparison(reference, cells=50, margin=0.3)
    scores = []
    for a in range(0, 360, 2):
        Ta = rotation_about(Z_AXIS, np.radians(a), centre) @ T
        st = coarse.stats(apply(Ta, sub))
        ok = st["coverage"] >= 0.8 and np.isfinite(st["median_dz_mm"])
        scores.append((st["median_dz_mm"] if ok else np.inf, a))
    scores.sort()
    best = []
    for s, a in scores:
        if np.isfinite(s) and all(min(abs(a - b), 360 - abs(a - b)) >= 10 for _, b in best):
            best.append((s, a))
        if len(best) == 3:
            break
    if not best:
        raise ValueError("auto-place: no rotation puts the scan over the part's outline - move it by hand "
                         "(--adjust / --shift-mm) or align it with Evaluate first")
    angle = best[0][1]
    T = rotation_about(Z_AXIS, np.radians(angle), centre) @ T
    log(f"[align] auto-place: levelled on the bed (tilt {np.degrees(np.arccos(min(abs(n[2]), 1))):.1f} deg), "
        f"part found at ({bc[0] * 1e3:.0f}, {bc[1] * 1e3:.0f}) mm in the levelled scan "
        f"({area * 1e6:.0f} mm^2); best rotations "
        + ", ".join(f"{a} deg ({s:.2f} mm)" for s, a in best) + " - median |dz|")
    if len(best) > 1 and best[1][0] < 1.15 * best[0][0]:
        warn(f"auto-place: rotations {best[0][1]} and {best[1][1]} deg fit almost equally well (the part is "
             "symmetric): check the pose (reference_comparison.png, or --adjust; r turns by 90 deg)")
    T, rinfo = refine(reference, points, T, params, rng)
    return T, {"angle_deg": angle, "candidates": [{"angle_deg": a, "median_dz_mm": s} for s, a in best],
               "part_centre_levelled_mm": (bc * 1e3).tolist(), "part_area_mm2": area * 1e6,
               "bed_tilt_deg": float(np.degrees(np.arccos(min(abs(n[2]), 1)))), "refine": rinfo}


# --- the stage -------------------------------------------------------------------
def align(points, params, reference, rng, title=""):
    """Scan points (scan frame, metres) -> (points in the reference frame,
    alignment summary with 'matrix' and 'warnings')."""
    from scan_eval.pipeline import read_transform
    base = read_transform(params.transform) if params.transform else np.eye(4)
    centre = centre_of(reference, apply(base, subsample(points, 20000, rng)))
    T = pose_matrix(base, params.flip, params.rotate_deg, params.shift_mm, centre)
    steps = []
    if params.transform:
        steps.append(f"transform {params.transform}")
    if params.flip:
        steps.append("flip")
    if params.rotate_deg:
        steps.append(f"rotate {params.rotate_deg:g} deg")
    if params.shift_mm is not None and np.any(params.shift_mm):
        steps.append("shift " + ",".join(f"{v:g}" for v in params.shift_mm) + " mm")
    info = {"base_transform": params.transform, "flip": bool(params.flip), "rotate_deg": params.rotate_deg,
            "shift_mm": params.shift_mm, "auto_placed": False, "refined": False, "adjusted": False}
    if steps:
        log(f"[align] {' -> '.join(steps)}")
    if reference is not None and is_far(reference, points, T, rng):
        if params.auto_place:
            log("[align] the scan is nowhere near the part (raw camera coordinates?) - auto-placing it")
            T, info["auto_place"] = auto_place(reference, points, T, params, rng)
            info["auto_placed"] = True
        else:
            log("[align] the scan is nowhere near the part, and --no-auto-place is set")
    if reference is not None and params.refine:
        T, info["refine"] = refine(reference, points, T, params, rng)
        info["refined"] = True
    if reference is not None and params.adjust:
        T_new = adjust(reference, points, T, params, rng, title)
        info["adjusted"] = not np.allclose(T_new, T)
        info["adjust_moved_mm"], info["adjust_turned_deg"] = motion(T, T_new, points)
        T = T_new
    moved, turned = motion(np.eye(4), T, points)
    info.update(matrix=T, moved_mm=moved, turned_deg=turned, warnings=[])
    out = apply(T, points)
    if reference is not None:
        # Own seeded draw, so the score doesn't depend on how many random
        # numbers refine used: the same pose always scores the same.
        near = near_part(reference, out, np.eye(4), margin=0.3)
        st = Comparison(reference).stats(subsample(near, 200000, np.random.default_rng(params.seed)))
        info.update({f"align_{k}": v for k, v in st.items()})
        info["warnings"] = check(st, params, reference, T)
        log(f"[align] vs reference: median |dz| {st['median_dz_mm']:.3f} mm, RMS dz {st['rms_dz_mm']:.3f} mm, "
            f"height correlation {st['height_corr']:+.3f}, outline covered {st['coverage']:.0%}")
    return out, info


def write_transform(path, T, scan, reference_name, info) -> None:
    parts = [f"transform {info['base_transform']}"] if info.get("base_transform") else []
    parts += [k for k in ("flip",) if info.get(k)]
    if info.get("rotate_deg"):
        parts.append(f"rotate {info['rotate_deg']:g} deg")
    if info.get("shift_mm") is not None and np.any(info["shift_mm"]):
        parts.append("shift " + ",".join(f"{v:g}" for v in info["shift_mm"]) + " mm")
    if info.get("auto_placed"):
        parts.append(f"auto-placed (rotation {info['auto_place']['angle_deg']} deg)")
    parts += [k for k in ("refined", "adjusted") if info.get(k)]
    header = ["scan_eval transform: scan -> reference frame, metres (written by roughness_scans.py)",
              f"scan {scan}", f"reference {reference_name}",
              f"includes: {' -> '.join(parts) or 'identity'}",
              "reuse with: roughness_scans.py --transform <this file> (and none of --flip/--rotate-deg/"
              "--shift-mm/--refine/--adjust) <same scan>",
              "       or:  evaluate_scans.py --init-transform <this file> --align refine|none <same scan>"]
    np.savetxt(path, T, fmt="%.12f", header="\n".join(header))


# --- by hand ---------------------------------------------------------------------
STEPS = [0.1, 1.0, 10.0, 100.0]          # mm (and deg, capped at 10) per press, keys 1-4
KEYS = {
    "left": ("move", 0, -1), "j": ("move", 0, -1), "right": ("move", 0, 1), "l": ("move", 0, 1),
    "up": ("move", 1, 1), "i": ("move", 1, 1), "down": ("move", 1, -1), "k": ("move", 1, -1),
    "pageup": ("move", 2, 1), "w": ("move", 2, 1), "+": ("move", 2, 1), "=": ("move", 2, 1),
    "pagedown": ("move", 2, -1), "s": ("move", 2, -1), "-": ("move", 2, -1),
    "[": ("rot", -1), "]": ("rot", 1), "r": ("rot90",),
    "1": ("step", 0), "2": ("step", 1), "3": ("step", 2), "4": ("step", 3),
    "f": ("flip",), "a": ("fit",), "p": ("place",), "u": ("undo",), "0": ("reset",), "c": ("rescale",),
    "enter": ("accept",), "escape": ("cancel",),
}
KEY_HELP = ("x/y: arrows or i j k l    z: PgUp/PgDn, w/s or +/-    rotate: [ ]  (r = 90 deg)    "
            "step: 1 2 3 4 = 0.1 / 1 / 10 / 100 mm\n"
            "f flip   p auto-place   a auto-fit (ICP)   u undo   0 reset   c rescale colours   "
            "click a map: move the profile cuts   Enter accept   Esc cancel")
BUTTONS = [("x -", ("move", 0, -1)), ("x +", ("move", 0, 1)), ("y -", ("move", 1, -1)), ("y +", ("move", 1, 1)),
           ("z -", ("move", 2, -1)), ("z +", ("move", 2, 1)), ("rot -", ("rot", -1)), ("rot +", ("rot", 1)),
           ("90°", ("rot90",)), ("step", ("cycle",)), ("flip", ("flip",)), ("place", ("place",)),
           ("fit", ("fit",)), ("undo", ("undo",)), ("reset", ("reset",)), ("accept", ("accept",))]


class Aligner:
    """The window behind --adjust. Pose = base (auto-place / auto-fit fold
    into it) -> flip -> rotate -> shift about the reference centre. Kept
    apart from show() so it runs (and is tested) on a non-interactive backend."""

    def __init__(self, reference, points, T0, params, rng, title=""):
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Button

        from scan_eval.report import ERROR_CMAP, INK, INK_2, SURFACE

        from .orient import point_spacing
        self.reference, self.params, self.rng = reference, params, rng
        self.points = subsample(points, 3_000_000, rng)
        self.T0 = np.asarray(T0, dtype=float)
        self.reset()
        self.history, self.accepted, self.step_i, self.last_key = [], False, 1, "-"
        near = apply(self.T0, near_part(reference, self.points, self.T0, margin=0.1))
        spacing = point_spacing(near[:, :2], rng) if len(near) > 100 else 1e-3
        self.cmp = Comparison(reference, g=display_cell(reference, near, spacing), margin=0.3)
        cmp, g = self.cmp, self.cmp.geom
        self.cx, self.cy = (c * 1e3 for c in g.centres())
        self.row, self.col = g.ny // 2, g.nx // 2
        extent = g.extent_mm()
        zlo, zhi = (v * 1e3 for v in cmp.zrange)
        pad = 0.1 * (zhi - zlo)

        self.fig = plt.figure(figsize=(15, 10.5), facecolor=SURFACE)
        gs = self.fig.add_gridspec(2, 2, height_ratios=[3, 1.5], left=0.05, right=0.98, top=0.86, bottom=0.17,
                                   hspace=0.3, wspace=0.15)
        self.ax_h, self.ax_d = self.fig.add_subplot(gs[0, 0]), self.fig.add_subplot(gs[0, 1])
        self.ax_px, self.ax_py = self.fig.add_subplot(gs[1, 0]), self.fig.add_subplot(gs[1, 1])
        blank = np.full((g.ny, g.nx), np.nan)
        self.im_h = self.ax_h.imshow(blank, origin="lower", extent=extent, cmap="viridis",
                                     vmin=zlo - pad, vmax=zhi + pad, interpolation="nearest")
        self.im_d = self.ax_d.imshow(blank, origin="lower", extent=extent,
                                     cmap=ERROR_CMAP.with_extremes(bad=SURFACE), interpolation="nearest")
        href_mm = np.where(cmp.outline, cmp.href * 1e3, np.nan)
        levels = np.linspace(np.nanmin(href_mm), np.nanmax(href_mm), 8)[1:-1]
        for ax in (self.ax_h, self.ax_d):
            ax.contour(self.cx, self.cy, cmp.outline.astype(float), levels=[0.5], colors=["#e8871a"], linewidths=2)
            ax.set_aspect("equal")
            ax.set_xlabel("x, mm (reference frame)")
            ax.set_ylabel("y, mm")
        self.ax_h.contour(self.cx, self.cy, np.nan_to_num(href_mm, nan=np.nanmin(href_mm)), levels=levels,
                          colors=["#e8871a"], linewidths=0.9)
        self.ax_h.set_title("scan height  (orange: reference outline and height contours)",
                            loc="left", fontsize=10, color=INK)
        self.ax_d.set_title("scan - reference height", loc="left", fontsize=10, color=INK)
        cb = self.fig.colorbar(self.im_h, ax=self.ax_h, shrink=0.85, pad=0.02)
        cb.set_label("z, mm", color=INK_2)
        cb = self.fig.colorbar(self.im_d, ax=self.ax_d, shrink=0.85, pad=0.02)
        cb.set_label("dz, mm", color=INK_2)
        self.cross = [ax.axhline(self.cy[self.row], color=INK, lw=0.8, ls=(0, (4, 2))) for ax in (self.ax_h, self.ax_d)]
        self.cross += [ax.axvline(self.cx[self.col], color=INK, lw=0.8, ls=(0, (4, 2))) for ax in (self.ax_h, self.ax_d)]
        self.lines = {}
        for ax, along in ((self.ax_px, "x"), (self.ax_py, "y")):
            pos = self.cx if along == "x" else self.cy
            self.lines[along] = (ax.plot(pos, np.full_like(pos, np.nan), color="#e8871a", lw=2.4, label="reference z")[0],
                                 ax.plot(pos, np.full_like(pos, np.nan), color="#2a78d6", lw=1.4, label="scan z")[0])
            ax.set_xlim(pos[0], pos[-1])
            ax.set_xlabel(f"{along}, mm")
            ax.set_ylabel("z, mm")
            ax.set_ylim(zlo - 3 * pad, zhi + 3 * pad)
            ax.grid(color="#e4e3df", lw=0.6)
            ax.legend(frameon=False, fontsize=8, loc="upper right")
        self.status = self.fig.text(0.01, 0.975, "", fontsize=12, color=INK, va="top", family="monospace")
        self.fig.text(0.01, 0.065, KEY_HELP, fontsize=9, color=INK_2, va="bottom")
        self.fig.suptitle(title, x=0.99, ha="right", y=0.995, fontsize=10, color=INK_2)
        self.buttons = []
        w = 0.98 / len(BUTTONS)
        for i, (label, action) in enumerate(BUTTONS):
            b = Button(self.fig.add_axes([0.01 + i * w, 0.01, w * 0.94, 0.04]), label)
            b.on_clicked(lambda _e, a=action: self.do(a))
            self.buttons.append(b)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)
        self.fig.canvas.mpl_connect("button_press_event", self.on_click)
        self.dlim = None
        self.update()

    # -- pose --
    def reset(self):
        self.base, self.flip, self.rot, self.shift = self.T0.copy(), False, 0.0, np.zeros(3)

    @property
    def T(self):
        return pose_matrix(self.base, self.flip, self.rot, self.shift, centre_of(self.reference, None))

    def state(self):
        return (self.base.copy(), self.flip, self.rot, self.shift.copy())

    @property
    def step(self):
        return STEPS[self.step_i]

    # -- drawing --
    def update(self):
        cmp = self.cmp
        z = cmp.scan_map(apply(self.T, self.points))
        self.stats = st = cmp.stats_of_map(z)
        zd = fill_for_display(z) * 1e3
        href = cmp.href * 1e3
        dz = np.where(cmp.outline, zd - href, np.nan)
        if self.dlim is None:
            self.rescale(dz)
        self.im_h.set_data(zd)
        self.im_d.set_data(dz)
        r, c = self.row, self.col
        band = slice(max(r - 2, 0), r + 3), slice(max(c - 2, 0), c + 3)
        with np.errstate(invalid="ignore"), _quiet_nanmean():
            self.lines["x"][0].set_ydata(np.nanmean(np.where(cmp.outline, href, np.nan)[band[0]], axis=0))
            self.lines["x"][1].set_ydata(np.nanmean(zd[band[0]], axis=0))
            self.lines["y"][0].set_ydata(np.nanmean(np.where(cmp.outline, href, np.nan)[:, band[1]], axis=1))
            self.lines["y"][1].set_ydata(np.nanmean(zd[:, band[1]], axis=1))
        for ln in self.cross[:2]:
            ln.set_ydata([self.cy[r]] * 2)
        for ln in self.cross[2:]:
            ln.set_xdata([self.cx[c]] * 2)
        self.ax_px.set_title(f"profile along x at y = {self.cy[r]:.1f} mm", loc="left", fontsize=10)
        self.ax_py.set_title(f"profile along y at x = {self.cx[c]:.1f} mm", loc="left", fontsize=10)
        self.status.set_text(
            f"flip {'yes' if self.flip else 'no '}  rotate {self.rot:+7.1f} deg  shift x {self.shift[0]:+7.1f}  "
            f"y {self.shift[1]:+7.1f}  z {self.shift[2]:+7.1f} mm"
            f"{'  (+ placed/fitted)' if not np.allclose(self.base, self.T0) else ''}\n"
            f"median |dz| {st['median_dz_mm']:7.2f} mm   RMS dz {st['rms_dz_mm']:7.2f} mm   correlation "
            f"{st['height_corr']:+.2f}   outline covered {st['coverage']:4.0%}\n"
            f"step {self.step:g} mm / {min(self.step, 10):g} deg   last key: {self.last_key}")
        self.fig.canvas.draw_idle()

    def rescale(self, dz=None):
        if dz is None:
            z = self.cmp.scan_map(apply(self.T, self.points))
            dz = np.where(self.cmp.outline, fill_for_display(z) - self.cmp.href, np.nan) * 1e3
        finite = dz[np.isfinite(dz)]
        self.dlim = max(float(np.percentile(np.abs(finite), 98)) if len(finite) else 1.0, 1.0)
        self.im_d.set_clim(-self.dlim, self.dlim)

    # -- input --
    def on_key(self, event):
        k = event.key
        action = KEYS.get(k)
        self.last_key = f"{k}" if action else f"{k} (unused)"
        if action:
            self.do(action)
        else:
            self.update()

    def on_click(self, event):
        if event.inaxes in (self.ax_h, self.ax_d) and event.xdata is not None:
            self.col = int(np.argmin(np.abs(self.cx - event.xdata)))
            self.row = int(np.argmin(np.abs(self.cy - event.ydata)))
            self.update()

    def do(self, action):
        import matplotlib.pyplot as plt
        kind = action[0]
        if kind in ("move", "rot", "rot90", "flip", "fit", "place", "reset"):
            self.history.append(self.state())
        if kind == "move":
            self.shift[action[1]] += action[2] * self.step
        elif kind == "rot":
            self.rot += action[1] * min(self.step, 10.0)
        elif kind == "rot90":
            self.rot += 90.0
        elif kind == "step":
            self.step_i = action[1]
        elif kind == "cycle":
            self.step_i = (self.step_i + 1) % len(STEPS)
        elif kind == "flip":
            self.flip = not self.flip
        elif kind == "fit":
            T, _ = refine(self.reference, self.points, self.T, self.params, self.rng, cmp=self.cmp)
            self.base, self.flip, self.rot, self.shift = T, False, 0.0, np.zeros(3)
        elif kind == "place":
            T, _ = auto_place(self.reference, self.points, self.T, self.params, self.rng)
            self.base, self.flip, self.rot, self.shift = T, False, 0.0, np.zeros(3)
        elif kind == "undo" and self.history:
            self.base, self.flip, self.rot, self.shift = self.history.pop()
        elif kind == "reset":
            self.reset()
        elif kind == "rescale":
            self.rescale()
        elif kind in ("accept", "cancel"):
            self.accepted = kind == "accept"
            if not self.accepted:
                self.reset()
            plt.close(self.fig)
            return
        self.update()


class _quiet_nanmean:
    def __enter__(self):
        import warnings
        self._w = warnings.catch_warnings()
        self._w.__enter__()
        warnings.simplefilter("ignore", RuntimeWarning)

    def __exit__(self, *exc):
        return self._w.__exit__(*exc)


def adjust(reference, points, T0, params, rng, title=""):
    """Show the aligner and block until Enter / Esc / window closed. Returns
    the chosen scan -> reference matrix (T0 on Esc)."""
    import matplotlib.pyplot as plt

    from .roi import NO_KEYMAPS, interactive_backend
    interactive_backend("--adjust")
    try:
        with plt.rc_context(NO_KEYMAPS):
            aligner = Aligner(reference, points, T0, params, rng, title)
            log("[align] move the scan in the window (keys and buttons at the bottom); Enter accepts, Esc cancels")
            plt.show(block=True)
    finally:
        plt.switch_backend("Agg")
    if not aligner.accepted:
        log("[align] --adjust cancelled - keeping the starting pose")
        return np.asarray(T0, dtype=float)
    log(f"[align] adjusted: flip {'yes' if aligner.flip else 'no'}, rotate {aligner.rot:+.2f} deg, shift "
        + ",".join(f"{v:+.2f}" for v in aligner.shift) + " mm"
        + (" (after auto-place / auto-fit)" if not np.allclose(aligner.base, T0) else ""))
    return aligner.T


def reference_name(reference) -> str:
    return getattr(reference, "name", None) or Path(str(reference)).stem
