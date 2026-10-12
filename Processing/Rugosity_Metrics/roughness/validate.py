"""Optional validation against an analytic height field (--reference-def).

The scan must already be aligned into the reference's frame (e.g. Evaluate's
aligned output). The reference is sampled on a dense lattice over the scan's
footprint, pushed through the scan's own levelling transform, restricted to
the scan's valid cells, and then run through exactly the same detrend, grid
and metric code. Each metric's error is scan minus reference, so the error
budget of the whole capture + fusion + roughness chain is measured directly.
"""

from __future__ import annotations

import numpy as np

from scan_eval.log import log
from scan_eval.reference import UnitScaled

from .detrend import fit_trend
from .grid import apply_fill, cell_counts, cell_means, plan_fill
from .metrics import surface_metrics
from .params import DETREND_ORDERS

# Metrics compared (scalars that mean the same thing on both surfaces).
KEYS = ["Sq_mm", "Sa_mm", "Ssk", "Sku", "Sp_mm", "Sv_mm", "Sz_mm", "rugosity", "Sdr_pct", "Sdq",
        "slope_mean_deg", "Sal_mm", "Str", "Std_deg", "dominant_wavelength_mm",
        "dominant_direction_deg", "anisotropy_ratio", "hurst", "fractal_dimension", "TRI_mm", "VRM"]


def load_height_reference(params):
    from scan_eval.surface_def import load_reference
    ref = load_reference(params.reference_def, units_override=params.reference_units)
    if not ref.is_height_field:
        raise ValueError(f"{params.reference_def}: validation needs a height-field reference "
                         f"(z = h(x, y)); this one is {ref.kind}")
    ref_m = UnitScaled(ref)
    log(f"[reference] {ref.name}: height field, units {ref.units}, hash {ref.definition_hash[:16]}")
    return ref_m


def compare_to_reference(ref, prep, valid, params, methods, results) -> dict:
    frame, geom, g = prep.frame, prep.geom, prep.geom.g
    warnings = []
    # Scan footprint back in the reference (= original scan) frame.
    scan_ref = prep.points @ frame.R + frame.centroid
    lo, hi = scan_ref[:, :2].min(axis=0), scan_ref[:, :2].max(axis=0)
    rlo, rhi = ref.bounds()
    lo, hi = np.maximum(lo, rlo[:2]), np.minimum(hi, rhi[:2])
    if np.any(hi <= lo):
        raise ValueError("the scan does not overlap the reference in xy - is it aligned into the "
                         "reference frame (e.g. Evaluate's aligned output)?")

    # Is the scan really in the reference frame? Heights should agree.
    sub = scan_ref[:: max(1, len(scan_ref) // 50000)]
    href = ref.height(sub[:, 0], sub[:, 1])
    ok = np.isfinite(href)
    offset = float(np.median(np.abs(sub[ok, 2] - href[ok]))) if ok.any() else float("inf")

    # Dense lattice (~2 samples per cell each way) on the reference surface.
    step = g / 2
    xs = np.arange(lo[0], hi[0] + step / 2, step)
    ys = np.arange(lo[1], hi[1] + step / 2, step)
    X, Y = np.meshgrid(xs, ys)
    Z = ref.height(X.ravel(), Y.ravel())
    keep = np.isfinite(Z)
    pts = frame.apply(np.column_stack([X.ravel()[keep], Y.ravel()[keep], Z[keep]]))
    idx = geom.index(pts[:, :2])
    on = idx >= 0
    on[on] = valid.ravel()[idx[on]]
    pts, idx = pts[on], idx[on]
    if len(pts) < 100:
        raise ValueError("the reference covers almost none of the scan's valid cells")

    counts = cell_counts(idx, geom)
    plan = plan_fill(counts >= 1, params.max_hole_cells)
    ref_results, columns = {}, {}
    for m in methods:
        fit = fit_trend(pts, m, params.detrend_clip_sigma, params.detrend_iterations)
        r = pts[:, 2] - fit.evaluate(pts[:, 0], pts[:, 1])
        h = apply_fill(cell_means(idx, r, counts, geom), plan)
        mask = valid & np.isfinite(h)
        rng = np.random.default_rng([params.seed, DETREND_ORDERS[m]])
        res, _, _, _ = surface_metrics(np.where(mask, h, np.nan), mask, g, params, rng)
        ref_results[m] = res
        cols = {}
        for k in KEYS:
            s, t = results[m].get(k), res.get(k)
            if s is None or t is None or not (np.isfinite(s) and np.isfinite(t)):
                continue
            cols[f"ref_{k}"] = t
            cols[f"err_{k}"] = s - t
            cols[f"err_pct_{k}"] = 100 * (s - t) / abs(t) if t != 0 else float("nan")
        columns[m] = cols
    coverage = float(mask.sum() / max(valid.sum(), 1))
    log(f"[validate] {len(pts)} reference samples over {coverage:.1%} of the scan's valid cells; "
        f"median |scan z - reference z| {offset * 1e3:.3f} mm")
    pm = params.primary
    c = columns[pm]
    for k in ("Sq_mm", "Sa_mm", "Sdr_pct", "Sdq", "Sal_mm", "dominant_wavelength_mm", "hurst"):
        if f"ref_{k}" in c:
            log(f"  {pm:9s} {k:24s} scan {results[pm][k]:10.4f}  reference {c[f'ref_{k}']:10.4f}  "
                f"error {c[f'err_{k}']:+10.4f} ({c[f'err_pct_{k}']:+.1f}%)")
    up = ref.visible_from if ref.visible_from is not None else np.array([0, 0, 1.0])
    if frame.R[2] @ up < 0:
        warnings.append(("UPSIDEDOWN", "the levelled +Z points away from the reference's visible side, "
                         "so heights are inverted (Ssk, Sp/Sv flip). For scans aligned into the "
                         "reference frame pass --sensor-origin none"))
    if coverage < 0.95:
        warnings.append(("REFCOVER", f"the reference covers only {coverage:.0%} of the scan's valid "
                         "cells; the comparison uses the overlap"))
    if offset > max(1e-3, 0.5 * ref_results[pm]["Sq_mm"] * 1e-3):
        warnings.append(("ALIGN", f"scan and reference heights differ by {offset * 1e3:.2f} mm (median): "
                         "the scan does not look aligned into the reference frame, so the errors "
                         "below are not meaningful"))
    return {"reference": ref.describe(), "coverage": coverage, "samples": int(len(pts)),
            "height_offset_median_mm": offset * 1e3, "metrics": ref_results, "columns": columns,
            "warnings": warnings}
