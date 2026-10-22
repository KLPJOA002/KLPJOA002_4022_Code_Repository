"""One scan through the stages:

    load -> [align] -> orient -> crop/ROI -> detrend -> grid -> metrics -> [validate] -> write

Every detrend method gets the same orientation, crop and grid geometry; only
the polynomial removed differs, so their rows compare like for like.
"""

from __future__ import annotations

import contextlib
import csv
import dataclasses
import datetime as _dt
import hashlib
import io
import platform
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from scan_eval import log as L
from scan_eval.log import log, stage, warn
from scan_eval.pipeline import load_scan
from scan_eval.report import append_results, write_json

from .detrend import TrendFit, fit_trend
from .grid import FillPlan, GridGeometry, apply_fill, cell_counts, cell_means, plan_fill
from .metrics import area_metrics, footprint, height_metrics, surface_metrics
from .orient import Frame, crop, orient, point_spacing, round_sig
from .params import DETREND_ORDERS

# Columns that lead every roughness_results.csv row, in this order; all other
# columns follow alphabetically.
LEAD_COLUMNS = ["timestamp", "label", "scan", "detrend", "primary", "grid_mm", "Sq_mm", "Sa_mm",
                "Ssk", "Sku", "Sdr_pct", "rugosity", "Sdq", "Sal_mm", "Str", "Std_deg",
                "dominant_wavelength_mm", "dominant_direction_deg", "hurst", "fractal_dimension",
                "valid_fraction", "interpolated_fraction", "footprint_x_mm", "footprint_y_mm",
                "n_points", "Sq_noisecorr_mm", "Sq_ci95_lo_mm", "Sq_ci95_hi_mm", "flags", "seed"]

PACKAGE_DIR = Path(__file__).resolve().parent


@dataclass
class Prepared:
    points: np.ndarray        # oriented and cropped, metres
    frame: Frame
    geom: GridGeometry
    n_loaded: int
    spacing: float            # median xy point spacing, m
    grid_auto: bool
    crop_info: dict
    all_points: np.ndarray    # oriented, before the crop (for the ROI preview)
    keep: np.ndarray          # crop mask over all_points
    alignment: dict | None = None   # scan -> reference moves and checks (align.align)


def prepare(source, params, rng, grid: float | None = None, reference=None) -> Prepared:
    """Load (a .ply path, or an (n, 3) array in metres), orient and crop.
    `grid` (m) overrides the grid choice, e.g. to match a noise scan to a scan.
    With a reference and reference_crop = "footprint" only the scan over the
    reference's outline is kept. With pick_roi the ROI is drawn by hand and
    written into `params` (pass a copy)."""
    stage("load")
    if isinstance(source, np.ndarray):
        points = np.asarray(source, dtype=float)
        if len(points) < 100:
            raise ValueError(f"only {len(points)} points - too few to evaluate")
        log(f"[scan] array: {len(points)} points")
    else:
        points = load_scan(source, params.units, params.scale)
        log(f"[scan] {Path(source).name}: {len(points)} points")
    alignment = None
    moves = params.transform or params.flip or params.rotate_deg or (
        params.shift_mm is not None and np.any(params.shift_mm))
    if reference is not None or moves:
        stage("align")
        from .align import align
        title = "array" if isinstance(source, np.ndarray) else Path(source).name
        points, alignment = align(points, params, reference, rng, title)
    stage("orient")
    oriented, frame = orient(points, params)

    stage("crop")
    spacing = point_spacing(oriented[:, :2], rng)
    auto = grid is None and not params.grid_mm
    g = grid or (params.grid_mm * 1e-3 if params.grid_mm else round_sig(2 * spacing * 1e3) * 1e-3)
    log(f"[grid] median point spacing {spacing * 1e3:.3f} mm -> grid {g * 1e3:g} mm"
        f"{' (auto: 2 x spacing)' if auto else ''}")
    if g < 0.5 * spacing:
        warn(f"grid {g * 1e3:g} mm is finer than half the point spacing ({spacing * 1e3:.3f} mm): "
             "most cells will be empty or hold a single noisy point")
    region = None
    if reference is not None and params.reference_crop == "footprint":
        scan_ref = oriented @ frame.R + frame.centroid
        region = reference.region_mask(scan_ref, -params.reference_inset_mm * 1e-3)
    if params.pick_roi:
        from . import roi
        title = "array" if isinstance(source, np.ndarray) else Path(source).name
        params.roi_mm, params.roi_polygon_mm = roi.pick(oriented, g, region, title, params.roi_mm)
    keep, info = crop(oriented, params, g, region)
    pts = oriented[keep]
    if len(pts) < 100:
        raise ValueError(f"only {len(pts)} points left after the crop - too few for roughness "
                         "(check --roi / --edge-trim-mm)")
    geom = GridGeometry.from_points(pts[:, :2], g)
    log(f"[crop] {len(pts)} points; footprint {geom.nx * g * 1e3:.1f} x {geom.ny * g * 1e3:.1f} mm "
        f"-> {geom.nx} x {geom.ny} cells")
    if min(geom.nx, geom.ny) < params.min_grid_cells:
        raise ValueError(f"grid too coarse for the footprint: {geom.nx} x {geom.ny} cells of "
                         f"{g * 1e3:g} mm over {geom.nx * g * 1e3:.1f} x {geom.ny * g * 1e3:.1f} mm; "
                         f"need >= {params.min_grid_cells} across the short side. Use a smaller "
                         f"--grid-mm (<= {min(geom.nx, geom.ny) * g * 1e3 / params.min_grid_cells:.3g}) "
                         "or a larger ROI")
    return Prepared(pts, frame, geom, len(points), spacing, auto, info, oriented, keep, alignment)


def _quiet(fn, *args):
    """Run fn without console or run.log output (warnings still show)."""
    fh_save, L._files[:] = list(L._files), []
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            return fn(*args)
    finally:
        L._files[:] = fh_save


def detrend_all(pts, params, methods):
    """Fit and subtract every method. Returns {method: (fit, residual, stats)}."""
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    var_total = float(np.var(z))
    out = {}
    for m in methods:
        t0 = time.time()
        fit = fit_trend(pts, m, params.detrend_clip_sigma, params.detrend_iterations)
        trend = fit.evaluate(x, y)
        r = z - trend
        stats = {"detrend_order": fit.order, "detrend_n_coef": len(fit.coefficients),
                 "trend_rms_mm": float(np.std(trend)) * 1e3,
                 "trend_variance_removed": 1 - float(np.var(r)) / var_total if var_total > 0 else 0.0,
                 "detrend_residual_Sq_mm": float(np.std(r)) * 1e3,
                 "detrend_inlier_fraction": fit.inlier_fraction}
        out[m] = (fit, r, stats)
        log(f"[detrend] {m:9s} {stats['detrend_n_coef']:2d} coef  trend RMS {stats['trend_rms_mm']:8.3f} mm  "
            f"removes {stats['trend_variance_removed']:6.1%} of height variance  residual Sq "
            f"{stats['detrend_residual_Sq_mm']:.3f} mm  ({fit.inlier_fraction:.1%} inliers, "
            f"{fit.iterations} it, {time.time() - t0:.1f} s)")
    return out


def grid_all(pts, residuals: dict, geom: GridGeometry, params):
    """Bin every method's residual onto the shared grid. Returns (maps, valid,
    counts, plan, stats)."""
    idx = geom.index(pts[:, :2])
    counts = cell_counts(idx, geom)
    measured = counts >= params.min_points_per_cell
    if not measured.any():
        raise ValueError("no grid cell has enough points (lower --min-points-per-cell or coarsen --grid-mm)")
    plan = plan_fill(measured, params.max_hole_cells)
    maps = {}
    for m, r in residuals.items():
        h = cell_means(idx, r, counts, geom)
        maps[m] = apply_fill(h, plan)
    valid = np.isfinite(next(iter(maps.values())))
    fx, fy = footprint(valid, geom.g)
    stats = {
        "grid_mm": geom.g * 1e3, "grid_nx": geom.nx, "grid_ny": geom.ny,
        "valid_fraction": float(valid.sum() / max(plan.footprint.sum(), 1)),
        "interpolated_fraction": float((valid & plan.fill).sum() / max(valid.sum(), 1)),
        "holes_filled": plan.n_holes_filled, "holes_left": plan.n_holes_left,
        "footprint_x_mm": fx * 1e3, "footprint_y_mm": fy * 1e3,
        "points_per_cell_mean": float(counts[measured].mean()),
    }
    log(f"[grid] {valid.sum()} valid cells of {plan.footprint.sum()} in the footprint "
        f"({stats['valid_fraction']:.1%}); {plan.n_holes_filled} small hole(s) interpolated "
        f"({stats['interpolated_fraction']:.2%} of valid cells), {plan.n_holes_left} larger hole(s) "
        f"masked; {stats['points_per_cell_mean']:.1f} points / cell")
    log(f"[grid] footprint {stats['footprint_x_mm']:.1f} x {stats['footprint_y_mm']:.1f} mm")
    return maps, valid, counts, plan, stats


# --- noise -------------------------------------------------------------------
class NoiseModel:
    """The noise floor of the height map, from --noise-sigma-mm (per point,
    averaged down by points per cell) or --noise-scan (a flat scan through the
    same pipeline at the same grid). Noise-scan results are cached per grid."""

    def __init__(self, params):
        self.params = params
        self.cache = {}

    def active(self):
        return bool(self.params.noise_scan) or self.params.noise_sigma_mm > 0

    def for_grid(self, g, counts, valid, rng):
        p = self.params
        if p.noise_scan:
            key = round(g * 1e9)
            if key not in self.cache:
                self.cache[key] = noise_floor(p.noise_scan, p, g, rng)
            n = dict(self.cache[key])
            ratio = float(counts[counts > 0].mean()) / n["points_per_cell"]
            if not 0.5 <= ratio <= 2:
                warn(f"the noise scan has {n['points_per_cell']:.1f} points / cell vs this scan's "
                     f"{counts[counts > 0].mean():.1f}: its noise floor may not transfer (density differs >2x)")
            return n
        c = counts[valid & (counts > 0)]
        sigma = p.noise_sigma_mm * 1e-3 * np.sqrt(np.mean(1.0 / c))
        return {"source": f"sigma {p.noise_sigma_mm:g} mm per point", "sigma_map": sigma,
                "Sdq": float(np.sqrt(2) * sigma / g)}


def noise_floor(path, params, g, rng):
    log(f"[noise] processing flat scan {path} at grid {g * 1e3:g} mm (primary detrend {params.primary})")
    p2 = dataclasses.replace(params, roi_mm=None, roi_polygon_mm=None, pick_roi=False, transform=None,
                             flip=False, rotate_deg=0.0, shift_mm=None, refine=False, adjust=False)
    prep = _quiet(prepare, path, p2, rng, g)
    fit = fit_trend(prep.points, params.primary, params.detrend_clip_sigma, params.detrend_iterations)
    r = prep.points[:, 2] - fit.evaluate(prep.points[:, 0], prep.points[:, 1])
    maps, valid, counts, _, _ = _quiet(grid_all, prep.points, {"n": r}, prep.geom, params)
    h = maps["n"]
    h = np.where(valid, h - h[valid].mean(), np.nan)
    sq = height_metrics(h, valid)["Sq_mm"] * 1e-3
    sdq = area_metrics(h, g)["Sdq"]
    log(f"[noise] flat scan: Sq {sq * 1e3:.4f} mm, Sdq {sdq:.4f} over {valid.sum()} cells")
    return {"source": f"noise scan {path}", "sigma_map": sq, "Sdq": sdq,
            "points_per_cell": float(counts[counts > 0].mean())}


def apply_noise(m, noise, params):
    sig = noise["sigma_map"]
    sq = m["Sq_mm"] * 1e-3
    m["noise_source"] = noise["source"]
    m["noise_sigma_map_mm"] = sig * 1e3
    m["Sq_noisecorr_mm"] = float(np.sqrt(max(sq ** 2 - sig ** 2, 0))) * 1e3
    frac = noise["Sdq"] ** 2 / m["Sdq"] ** 2 if m["Sdq"] > 0 else float("inf")
    m["Sdq_noise_fraction"] = float(frac)
    m["Sdq_noisecorr"] = float(np.sqrt(max(m["Sdq"] ** 2 - noise["Sdq"] ** 2, 0)))
    m["noise_inflated"] = bool(frac > params.noise_warn_fraction)
    out = []
    if m["noise_inflated"]:
        out.append(("NOISE", f"noise accounts for {min(frac, 9.99):.0%} of the slope variance (Sdq^2): "
                    "rugosity, Sdr and Sdq here are noise-inflated - coarsen --grid-mm or compare "
                    "Sdq_noisecorr"))
    if sq > 0 and sig >= sq:
        out.append(("NOISE", f"the noise floor ({sig * 1e3:.4f} mm) is at or above Sq ({sq * 1e3:.4f} mm): "
                    "this surface's roughness is not resolved above sensor noise"))
    return out


# --- reporting ---------------------------------------------------------------
def print_metrics(method, m, primary):
    log(f"[metrics] {method}{' (primary)' if method == primary else ''}")
    log(f"  height    Sq {m['Sq_mm']:.4f}  Sa {m['Sa_mm']:.4f}  Sp {m['Sp_mm']:.3f}  Sv {m['Sv_mm']:.3f}  "
        f"Sz {m['Sz_mm']:.3f} mm   Ssk {m['Ssk']:+.3f}  Sku {m['Sku']:.3f}")
    log(f"  area      rugosity {m['rugosity']:.5f}  Sdr {m['Sdr_pct']:.3f}%  Sdq {m['Sdq']:.4f}  "
        f"mean slope {m['slope_mean_deg']:.2f} deg")
    log(f"  spatial   Sal {m['Sal_mm']:.2f} mm  Str {m['Str']:.3f}{' (upper bound)' if m['Str_bounded'] else ''}  "
        f"Std {m['Std_deg']:.0f} deg  anisotropy {m['anisotropy_ratio']:.3f}")
    log(f"  dominant  {m['dominant_wavelength_mm']:.2f} mm, wave vector at {m['dominant_direction_deg']:.1f} deg "
        f"(peak {m['dominant_peak_ratio']:.0f}x median)")
    log(f"  spectrum  slope {m['psd_slope']:.3f}  Hurst {m['hurst']:.3f}  D {m['fractal_dimension']:.3f}  "
        f"R^2 {m['psd_fit_r2']:.3f}  over {m['bandwidth_min_mm']:.3g}..{m['bandwidth_max_mm']:.3g} mm")
    if "ms_window_min_mm" in m:
        log(f"  windows   Sq {m['ms_Sq_min_window_mm']:.4f} -> {m['ms_Sq_max_window_mm']:.4f} mm and rugosity "
            f"{m['ms_rugosity_min_window']:.4f} -> {m['ms_rugosity_max_window']:.4f} from "
            f"{m['ms_window_min_mm']:.3g} to {m['ms_window_max_mm']:.3g} mm windows")
    if m.get("n_windows_used", 0) >= 2:
        log(f"  95% CI    Sq [{m['Sq_ci95_lo_mm']:.4f}, {m['Sq_ci95_hi_mm']:.4f}] mm  Sdr "
            f"[{m['Sdr_ci95_lo_pct']:.3f}, {m['Sdr_ci95_hi_pct']:.3f}]%  Sal [{m['Sal_ci95_lo_mm']:.2f}, "
            f"{m['Sal_ci95_hi_mm']:.2f}] mm  ({m['n_windows_used']} windows of {m['bootstrap_window_mm']:.3g} mm)")
    if "TRI_mm" in m:
        log(f"  rugged    TRI {m['TRI_mm']:.4f} mm  VRM {m['VRM']:.2e}  ({m['ruggedness_window_cells']}x"
            f"{m['ruggedness_window_cells']} cells)")
    if "Sq_noisecorr_mm" in m:
        log(f"  noise     floor {m['noise_sigma_map_mm']:.4f} mm -> Sq corrected {m['Sq_noisecorr_mm']:.4f} mm; "
            f"Sdq corrected {m['Sdq_noisecorr']:.4f} (noise {min(m['Sdq_noise_fraction'], 9.99):.0%} of Sdq^2)")


def write_curves(run_dir, method, curves):
    r = curves["radial_psd"]
    with open(run_dir / f"psd_{method}.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["k_per_mm", "wavelength_mm", "psd_mm4", "count"])
        for row in zip(r["k_per_mm"], r["wavelength_mm"], r["psd_mm4"], r["count"]):
            if row[3] > 0:
                w.writerow([f"{v:.8g}" for v in row[:3]] + [int(row[3])])
    rows = curves["multiscale"]
    if rows:
        with open(run_dir / f"multiscale_{method}.csv", "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)


def file_sha256(path, chunk=1 << 20):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        while block := fh.read(chunk):
            h.update(block)
    return h.hexdigest()


def versions():
    import matplotlib
    import scipy
    code = hashlib.sha256()
    for p in sorted(PACKAGE_DIR.glob("*.py")) + [PACKAGE_DIR.parent / "roughness_scans.py"]:
        if p.exists():
            code.update(p.read_bytes())
    try:
        commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=PACKAGE_DIR, capture_output=True,
                                text=True, timeout=5).stdout.strip() or None
        dirty = bool(subprocess.run(["git", "status", "--porcelain", "--", "."], cwd=PACKAGE_DIR.parent,
                                    capture_output=True, text=True, timeout=5).stdout.strip())
    except (OSError, subprocess.SubprocessError):
        commit, dirty = None, None
    try:
        import open3d
        o3d_version = open3d.__version__
    except ImportError:
        o3d_version = None
    return {"python": platform.python_version(), "numpy": np.__version__, "scipy": scipy.__version__,
            "matplotlib": matplotlib.__version__, "open3d": o3d_version,
            "roughness_code_sha256": code.hexdigest(), "git_commit": commit, "git_dirty": dirty}


def _round(v):
    if isinstance(v, (float, np.floating)):
        return round(float(v), 6) if np.isfinite(v) else float(v)
    if isinstance(v, np.integer):
        return int(v)
    if isinstance(v, np.bool_):
        return bool(v)
    return v


@dataclass
class Analysis:
    prep: Prepared
    trends: dict          # method -> (TrendFit, residual per point, stats)
    maps: dict            # method -> filled height map (m, NaN invalid), before mean removal
    valid: np.ndarray
    gstats: dict
    noise: dict | None
    results: dict         # method -> scalar metrics
    curves: dict
    aux: dict             # method -> arrays for plots (mean-removed height map, PSD, ACF)
    warns: dict           # method -> [(code, text)]
    validation: dict | None
    params: object        # the parameters actually used (with a picked ROI filled in)
    raw_map: np.ndarray   # cell-mean height before detrending, m (NaN invalid)
    trend_maps: dict      # method -> fitted trend surface on the grid, m (NaN invalid)


def analyse(source, params, noise: NoiseModel | None = None, reference=None) -> Analysis:
    """Stages load .. metrics for a .ply path or an (n, 3) array (metres)."""
    params = dataclasses.replace(params)          # a picked ROI must not leak into the next scan
    rng = np.random.default_rng(params.seed)
    methods, primary = params.methods, params.primary
    noise = noise or NoiseModel(params)
    prep = prepare(source, params, rng, reference=reference)
    pts, geom, g = prep.points, prep.geom, prep.geom.g

    stage("detrend")
    trends = detrend_all(pts, params, methods)

    stage("grid")
    maps, valid, counts, plan, gstats = grid_all(pts, {"_raw": pts[:, 2], **{m: t[1] for m, t in trends.items()}},
                                                 geom, params)
    raw_map = maps.pop("_raw")
    cx, cy = geom.centres()
    CX, CY = np.meshgrid(cx, cy)
    trend_maps = {m: np.where(valid, trends[m][0].evaluate(CX, CY), np.nan) for m in methods}
    noise_info = noise.for_grid(g, counts, valid, rng) if noise and noise.active() else None
    grid_warns = list(prep.alignment["warnings"]) if prep.alignment else []
    if gstats["valid_fraction"] < 0.8:
        grid_warns.append(("SPARSE", f"only {gstats['valid_fraction']:.0%} of the footprint's cells are "
                           f"valid at {g * 1e3:g} mm ({gstats['points_per_cell_mean']:.1f} points / cell): "
                           "masked cells weaken the spatial and multi-scale metrics - try a coarser "
                           f"--grid-mm (e.g. {round_sig(2 * g * 1e3):g})"))

    stage("metrics")
    results, curves, aux, warns = {}, {}, {}, {}
    for m in methods:
        t0 = time.time()
        # Each method draws its bootstrap from the same seeded stream, so a
        # method's numbers do not depend on which other methods ran.
        m_rng = np.random.default_rng([params.seed, DETREND_ORDERS[m]])
        res, cur, ax, wl = surface_metrics(maps[m], valid, g, params, m_rng)
        res.update(trends[m][2])
        if noise_info:
            wl += apply_noise(res, noise_info, params)
        results[m], curves[m], aux[m], warns[m] = res, cur, ax, grid_warns + wl
        print_metrics(m, res, primary)
        log(f"  ({time.time() - t0:.1f} s)")

    # Is a higher-order fit eating real roughness? A degree-n polynomial
    # can follow ~n/2 periods, so it matters when the footprint holds
    # fewer than n wavelengths and the fit removed a lot of variance.
    p = results[primary]
    lam = (params.expected_wavelength_mm * 1e-3 if params.expected_wavelength_mm
           else (None if p.get("dominant_at_limit") else p["dominant_wavelength_mm"] * 1e-3))
    base = results["planar"]["trend_variance_removed"] if "planar" in results else 0.0
    foot_min = min(gstats["footprint_x_mm"], gstats["footprint_y_mm"]) * 1e-3
    for m in methods:
        order = DETREND_ORDERS[m]
        extra = results[m]["trend_variance_removed"] - base
        if order >= 2 and lam and extra > params.detrend_warn_fraction and foot_min < order * lam:
            warns[m].append(("DETREND", f"{m} removes {extra:.0%} more height variance than a plane, "
                             f"over a footprint only {foot_min / lam:.1f} wavelengths across "
                             f"({lam * 1e3:.1f} mm{' expected' if params.expected_wavelength_mm else ' measured'}): "
                             "the polynomial may be fitting real bedforms"))

    validation = None
    if reference is not None:
        stage("validate")
        from .validate import compare_to_reference
        validation = compare_to_reference(reference, prep, valid, params, methods, results, raw_map)
        for m in methods:
            warns[m] += validation["warnings"]

    for m in methods:
        for code, text in warns[m]:
            warn(f"{m}: {text}" if len(methods) > 1 else text)

    return Analysis(prep, trends, maps, valid, gstats, noise_info, results, curves, aux, warns,
                    validation, params, raw_map, trend_maps)


# --- one scan ----------------------------------------------------------------
def process_scan(scan_path, params, out_root, run_name, noise: NoiseModel | None = None,
                 reference=None) -> list[dict]:
    """Every metric for every detrend method of one scan; returns the rows
    (also appended to <out_root>/roughness_results.csv)."""
    out_root = Path(out_root)
    run_dir = out_root / run_name
    run_dir.mkdir(parents=True, exist_ok=True)
    log_path = run_dir / "run.log"
    log_path.write_text("")
    fh = L.attach(log_path)
    started = time.time()
    methods, primary = params.methods, params.primary
    try:
        log(f"\n######## {scan_path}  ->  {run_dir}")
        A = analyse(scan_path, params, noise, reference)
        params = A.params
        prep, geom, pts = A.prep, A.prep.geom, A.prep.points
        trends, results, curves, aux, warns = A.trends, A.results, A.curves, A.aux, A.warns
        gstats, noise_info, validation = A.gstats, A.noise, A.validation

        stage("write")
        extent = geom.extent_mm()
        for m in methods:
            np.save(run_dir / f"heightmap_{m}.npy", (aux[m]["h"] * 1e3).astype(np.float32))
            write_curves(run_dir, m, curves[m])
        written = [f"heightmap_<method>.npy ({', '.join(methods)}, mm, NaN = invalid)",
                   "psd_<method>.csv", "multiscale_<method>.csv"]
        if prep.alignment:
            from .align import reference_name, write_transform
            write_transform(run_dir / "transform.txt", prep.alignment["matrix"], scan_path,
                            reference_name(reference) if reference is not None else "none", prep.alignment)
            written.append("transform.txt")
        if params.pick_roi and (params.roi_mm or params.roi_polygon_mm):
            from .roi import write_roi_toml
            write_roi_toml(run_dir / "roi.toml", params.roi_mm, params.roi_polygon_mm, scan_path,
                           params.orientation)
            written.append("roi.toml")
        if params.write_plots:
            from . import plots
            title = Path(scan_path).name
            for m in methods:
                plots.plot_heightmap(run_dir / f"heightmap_{m}.png", aux[m]["h"] * 1e3, extent,
                                     f"{title}  -  {m} detrend", results[m])
            plots.plot_detrend_comparison(run_dir / "detrend_comparison.png",
                                          {m: aux[m]["h"] * 1e3 for m in methods}, results, extent,
                                          f"{title}  -  detrend comparison", primary)
            plots.plot_roi_preview(run_dir / "roi_preview.png", prep, params,
                                   f"{title}  -  evaluated area")
            y_row = plots.profile_row(A.valid, geom, params.profile_y_mm)
            plots.plot_detrend_explained(run_dir / "detrend_explained.png", A.raw_map, A.trend_maps,
                                         {m: aux[m]["h"] for m in methods}, geom, y_row,
                                         f"{title}  -  what each detrend removes", primary)
            plots.plot_detrend_metrics(run_dir / "detrend_metrics.png", results, methods, primary,
                                       validation and validation["columns"],
                                       f"{title}  -  metrics by detrend method")
            if validation:
                plots.plot_reference_comparison(run_dir / "reference_comparison.png", validation["_aux"],
                                                aux[primary]["h"], A.raw_map, geom, y_row, primary,
                                                f"{title}  -  scan vs reference ({primary} detrend)")
            plots.plot_psd(run_dir / "psd.png", aux[primary], curves[primary], results[primary],
                           f"{title}  -  {primary} detrend")
            plots.plot_multiscale(run_dir / "multiscale.png", curves[primary]["multiscale"],
                                  f"{title}  -  {primary} detrend, scale dependence")
            written.append("heightmap_<method>.png, roi_preview.png, detrend_comparison.png, "
                           "detrend_explained.png, detrend_metrics.png, psd.png, multiscale.png"
                           + (", reference_comparison.png" if validation else ""))
        log(f"[write] {'; '.join(written)}")

        timestamp = _dt.datetime.now().isoformat(timespec="seconds")
        scan_level = {
            "n_points_loaded": prep.n_loaded, "n_points": len(pts),
            "point_spacing_mm": prep.spacing * 1e3, "grid_auto": prep.grid_auto,
            "tilt_removed_deg": prep.frame.tilt_deg, "plane_rms_mm": prep.frame.plane_rms * 1e3,
            "roi_mm": ",".join(f"{v:g}" for v in params.roi_mm) if params.roi_mm else "",
            "roi_polygon_mm": (";".join(f"{x:g},{y:g}" for x, y in params.roi_polygon_mm)
                               if params.roi_polygon_mm else ""),
            "orientation": params.orientation,
            **({k: prep.alignment[k] for k in ("align_median_dz_mm", "align_rms_dz_mm", "align_height_corr",
                                               "align_coverage") if k in prep.alignment}
               | {"align_auto_placed": prep.alignment["auto_placed"],
                  "align_refined": prep.alignment["refined"], "align_adjusted": prep.alignment["adjusted"],
                  "align_moved_mm": prep.alignment["moved_mm"]}
               if prep.alignment else {}),
            "edge_trim_mm": params.edge_trim_mm,
        }
        scan_level.update(gstats)
        document = {
            "timestamp": timestamp, "label": params.label, "scan": str(scan_path),
            "scan_sha256": file_sha256(scan_path), "run_dir": str(run_dir), "seed": params.seed,
            "parameters": params.to_dict(), "versions": versions(),
            "alignment": prep.alignment and {k: v for k, v in prep.alignment.items() if k != "warnings"},
            "orientation": prep.frame.as_dict(), "crop": prep.crop_info, "grid": geom.as_dict(),
            "scan_summary": scan_level, "noise": noise_info, "primary_detrend": primary,
            "detrend": {m: trends[m][0].as_dict() | trends[m][2] for m in methods},
            "metrics": results, "curves": curves,
            "warnings": {m: [{"code": c, "text": t} for c, t in warns[m]] for m in methods},
            "validation": validation and {k: v for k, v in validation.items()
                                          if k != "warnings" and not k.startswith("_")},
            "elapsed_s": round(time.time() - started, 2),
        }
        write_json(run_dir / "metrics.json", document)

        rows = []
        for m in methods:
            row = {"timestamp": timestamp, "label": params.label or Path(scan_path).stem,
                   "scan": str(scan_path), "detrend": m, "primary": m == primary,
                   "units": params.units, "scale": params.scale, "seed": params.seed,
                   "flags": " ".join(sorted({c for c, _ in warns[m]})), "run_dir": str(run_dir)}
            row.update(scan_level)
            row.update(results[m])
            if validation:
                row.update(validation["columns"][m])
            rows.append({k: _round(v) for k, v in row.items()})
            append_results(out_root / "roughness_results.csv", rows[-1], lead_columns=LEAD_COLUMNS)
        log(f"[write] metrics.json, run.log; {len(rows)} row(s) appended to "
            f"{out_root / 'roughness_results.csv'}")
        log(f"[done] {time.time() - started:.1f} s")
        return rows
    finally:
        L.detach(fh)
