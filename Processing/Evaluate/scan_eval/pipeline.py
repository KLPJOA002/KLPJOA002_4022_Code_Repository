"""One scan through the stages: load -> units check -> align -> metrics -> outputs."""

from __future__ import annotations

import datetime as _dt
import time
from pathlib import Path

import numpy as np
import open3d as o3d

from . import log as L
from .log import log, loud, stage, warn
from .metrics import compute_metrics
from .reference import unit_factor
from .registration import align, apply, print_alignment_report
from .report import (append_results, plot_error_map, plot_histogram, safe_name,
                     write_error_ply, write_json)

o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)


class ScanError(ValueError):
    """A scan that cannot be evaluated (missing, unreadable, empty)."""


def load_scan(path, units: str, scale: float) -> np.ndarray:
    path = Path(path)
    if not path.is_file():
        raise ScanError(f"scan not found: {path}")
    if path.suffix.lower() != ".ply":
        raise ScanError(f"scans must be .ply point clouds (got {path.name})")
    pcd = o3d.io.read_point_cloud(str(path))
    pts = np.asarray(pcd.points, dtype=float)
    faces = len(o3d.io.read_triangle_mesh(str(path)).triangles)
    if faces:
        warn(f"{path.name} is a mesh ({faces} faces), not a point cloud - is it a reference "
             "rather than a scan? Evaluating its vertices as scan points.")
    if len(pts) == 0:
        raise ScanError(f"{path.name} contains no points (empty or unreadable PLY)")
    finite = np.all(np.isfinite(pts), axis=1)
    if not finite.all():
        warn(f"{path.name}: dropped {int((~finite).sum())} non-finite points")
        pts = pts[finite]
    if len(pts) < 100:
        raise ScanError(f"{path.name} has only {len(pts)} valid points - too few to evaluate")
    return pts * unit_factor(units) * scale


def read_transform(path) -> np.ndarray:
    try:
        T = np.loadtxt(path, comments="#", dtype=float)
    except (OSError, ValueError) as exc:
        raise ValueError(f"cannot read --init-transform {path}: {exc}") from None
    if T.shape != (4, 4) or not np.all(np.isfinite(T)):
        raise ValueError(f"--init-transform {path} must hold a 4x4 matrix (got shape {T.shape})")
    if not np.allclose(T[3], [0, 0, 0, 1]):
        raise ValueError(f"--init-transform {path}: last row must be 0 0 0 1")
    return T


def check_units(ref, points, params) -> str | None:
    """Print both bounding boxes; shout if they differ by a unit-sized factor.
    Never corrects anything - the scale is part of what is measured."""
    rlo, rhi = ref.bounds()
    slo, shi = points.min(axis=0), points.max(axis=0)
    fmt = lambda v: "(" + ", ".join(f"{x * 1e3:9.2f}" for x in v) + ")"
    log(f"[units] reference ({getattr(ref, "units_label", ref.units)}):  min {fmt(rlo)}  max {fmt(rhi)}  extent {fmt(rhi - rlo)} mm")
    log(f"[units] scan      ({params.units}{'' if params.scale == 1 else f' x {params.scale:g}'}):"
        f"  min {fmt(slo)}  max {fmt(shi)}  extent {fmt(shi - slo)} mm")
    r_ext = float(np.max(rhi - rlo))
    s_ext = float(np.max(shi - slo))
    if r_ext <= 0 or s_ext <= 0:
        return None
    ratio = s_ext / r_ext
    # A scan legitimately includes surroundings (table, other parts) or only
    # part of the reference, so a factor of a few either way is normal. A
    # factor near a unit conversion is not.
    if 1 / 60 < ratio < 60:
        return None
    guesses = {"mm vs m (1000x)": 1000.0, "cm vs m (100x)": 100.0, "mm vs cm (10x)": 10.0,
               "in vs mm (25.4x)": 25.4}
    best = min(guesses, key=lambda k: min(abs(np.log(ratio / guesses[k])), abs(np.log(ratio * guesses[k]))))
    bigger = "scan" if ratio > 1 else "reference"
    message = (f"UNITS MISMATCH? The {bigger} is {max(ratio, 1 / ratio):.0f}x larger than the other "
               f"(scan extent {s_ext * 1e3:.4g} mm vs reference {r_ext * 1e3:.4g} mm) - this looks "
               f"like {best}. Scans are read as '{params.units}' (--units) and the reference as "
               f"'{getattr(ref, "units_label", ref.units)}' (--reference-units / the definition's units). Nothing has been "
               "rescaled; results below are meaningless until this is fixed.")
    loud(message)
    return message


def unique_run_names(paths, label):
    stems = [Path(p).stem for p in paths]
    names = []
    for p, s in zip(paths, stems):
        base = s if stems.count(s) == 1 else f"{Path(p).parent.name}_{s}"
        names.append(safe_name(f"{label}__{base}" if label else base))
    return names


def evaluate_scan(ref, scan_path, params, out_root, run_name, init_transform=None) -> dict:
    """Evaluate one scan; returns its results row (also appended to results.csv)."""
    out_root = Path(out_root)
    run_dir = out_root / run_name
    run_dir.mkdir(parents=True, exist_ok=True)
    log_path = run_dir / "run.log"
    log_path.write_text("")
    fh = L.attach(log_path)
    started = time.time()
    try:
        log(f"\n######## {scan_path}  ->  {run_dir}")
        stage("load")
        points = load_scan(scan_path, params.units, params.scale)
        log(f"[scan] {Path(scan_path).name}: {len(points)} points")
        log(f"[reference] {ref.name}  ({ref.kind}, {'exact' if ref.exact else 'approximate'}, "
            f"{'height field, ' if ref.is_height_field else ''}hash {ref.definition_hash[:16]})")
        units_warning = check_units(ref, points, params)

        stage("align")
        result = align(ref, points, params, params.seed, init_transform)
        print_alignment_report(result, params)
        T = result.transform
        aligned = apply(T, points)

        stage("metrics")
        metrics, per_point = compute_metrics(ref, aligned, params, params.seed)
        print_metrics(metrics, params)

        stage("write")
        header = (f"scan_eval transform: scan -> reference frame, metres\n"
                  f"scan {scan_path}  units {params.units}  scale {params.scale}\n"
                  f"reference {ref.name}  hash {ref.definition_hash}\n"
                  f"seed {params.seed}  recovered scale {result.scale:.8f}\n"
                  f"reuse with: --init-transform {run_dir / 'transform.txt'} --align refine|none")
        np.savetxt(run_dir / "transform.txt", T, fmt="%.12f", header=header)

        region = per_point["region"]
        signed_mm = per_point["signed"] * 1e3
        title = f"{run_name}  vs  {ref.name}"
        if params.write_ply:
            lim = write_error_ply(run_dir / "aligned_error.ply", aligned, per_point["signed"], region)
            log(f"[write] aligned_error.ply (colour range +-{lim:.3f} mm; grey = not over the part)")
        if params.write_plots:
            plot_histogram(run_dir / "error_histogram.png", signed_mm, metrics, title)
            z = per_point["z_error"]
            plot_error_map(run_dir / "error_map.png", aligned[region], signed_mm, ref, title,
                           None if z is None else z * 1e3)
            log("[write] error_histogram.png, error_map.png")

        align_info = {
            "transform": T, "recovered_scale": result.scale,
            "voxel_mm": result.voxel * 1e3, "eval_distance_mm": result.eval_distance * 1e3,
            "chosen": result.best.as_dict() if result.best else None,
            "runner_up": result.runner_up.as_dict() if result.runner_up else None,
            "margin": result.margin, "ambiguous": result.ambiguous,
            "candidates": [c.as_dict() for c in result.candidates],
            "degeneracy": result.degeneracy, "exact_refinement": result.refine,
        }
        timestamp = _dt.datetime.now().isoformat(timespec="seconds")
        document = {
            "timestamp": timestamp, "label": params.label, "scan": str(scan_path),
            "run_dir": str(run_dir), "seed": params.seed,
            "reference": ref.describe(), "reference_definition": ref.definition(),
            "parameters": params.to_dict(), "units_warning": units_warning,
            "alignment": align_info, "metrics": metrics,
            "elapsed_s": round(time.time() - started, 2),
        }
        write_json(run_dir / "metrics.json", document)

        row = {
            "timestamp": timestamp, "label": params.label or Path(scan_path).stem,
            "scan": str(scan_path), "reference": ref.name, "reference_type": ref.kind,
            "reference_exact": ref.exact, "reference_hash": ref.definition_hash[:16],
            "units": params.units, "scale": params.scale, "allow_scale": params.allow_scale,
            "recovered_scale": round(result.scale, 8), "seed": params.seed,
            "align_margin": None if result.margin is None else round(result.margin, 4),
            "align_ambiguous": result.ambiguous,
            "align_unconstrained": "; ".join(result.degeneracy.get("unconstrained", [])),
            "align_weak": "; ".join(result.degeneracy.get("weak", [])),
            "units_warning": bool(units_warning), "run_dir": str(run_dir),
        }
        row.update({k: (round(v, 6) if isinstance(v, float) else v) for k, v in metrics.items()})
        append_results(out_root / "results.csv", row)
        log(f"[write] metrics.json, transform.txt, run.log; row appended to {out_root / 'results.csv'}")
        log(f"[done] {time.time() - started:.1f} s")
        return row
    finally:
        L.detach(fh)


def print_metrics(m, params):
    log(f"[metrics] points: {m['points_total']} total, {m['points_scored']} over the part, "
        f"{m['points_excluded']} excluded (table / surroundings / other objects)")
    log(f"  accuracy    RMSE {m['acc_rmse_mm']:.3f}  MAE {m['acc_mae_mm']:.3f}  median "
        f"{m['acc_median_mm']:.3f}  std {m['acc_std_mm']:.3f}  p95 {m['acc_p95_mm']:.3f}  "
        f"max {m['acc_max_mm']:.3f} mm")
    trims = "  ".join(f"trim{p:g}% RMSE {m[f'acc_rmse_trim{p:g}_mm']:.3f}" for p in params.trim_percent)
    log(f"  robust      {trims} mm")
    log(f"  signed      bias {m['signed_bias_mm']:+.3f}  std {m['signed_std_mm']:.3f} mm")
    if "z_rmse_mm" in m:
        log(f"  vertical    bias {m['z_bias_mm']:+.3f}  std {m['z_std_mm']:.3f}  RMSE {m['z_rmse_mm']:.3f} mm")
    log(f"  coverage    {m['covered_fraction']:.1%} of the visible reference is covered "
        f"(within {m['coverage_radius_mm']:g} mm of a scan point)")
    for tau in params.thresholds_mm:
        t = f"{tau:g}mm"
        log(f"  @{t:>6}    precision {m[f'precision@{t}']:6.1%}  completeness {m[f'completeness@{t}']:6.1%}"
            f"  F-score {m[f'fscore@{t}']:6.1%}")
    if "chamfer_mm" in m:
        log(f"  chamfer {m['chamfer_mm']:.3f} mm   Hausdorff {m['hausdorff_mm']:.3f} mm   "
            f"Hausdorff95 {m['hausdorff95_mm']:.3f} mm")
    log(f"  outliers    {m['outlier_fraction']:.2%} beyond {m['outlier_distance_mm']:g} mm")
    if "density_pts_per_cm2" in m:
        log(f"  density     {m['density_pts_per_cm2']:.1f} points/cm^2, median spacing "
            f"{m.get('spacing_median_mm', float('nan')):.3f} mm")
    if "warning_coverage_radius" in m:
        warn(m["warning_coverage_radius"])
