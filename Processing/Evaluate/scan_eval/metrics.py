"""Error metrics of an aligned scan against the reference.

All inputs are in metres in the reference frame; every reported distance is
in millimetres (keys end in _mm) so tables from different runs line up.

Directions:
  accuracy     scan -> reference: exact distance from each scan point to the
               true surface (to the triangles for a mesh, to the analytic
               surface otherwise). Only points over the part (region_mask)
               are scored; the rest are counted as excluded.
  completeness reference -> scan: for area-uniform samples of the visible
               reference surface, the distance to the nearest scan point.
               Restricted to the covered region (samples with a scan point
               within coverage_radius), so a scan of half the part is judged
               on the half it saw; the covered fraction is reported separately.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree

MM = 1e3


def _stats(values: np.ndarray, prefix: str) -> dict:
    if len(values) == 0:
        return {f"{prefix}_n": 0}
    v = np.asarray(values) * MM
    return {
        f"{prefix}_mae_mm": float(np.mean(v)),
        f"{prefix}_median_mm": float(np.median(v)),
        f"{prefix}_rmse_mm": float(np.sqrt(np.mean(v**2))),
        f"{prefix}_std_mm": float(np.std(v)),
        f"{prefix}_p90_mm": float(np.percentile(v, 90)),
        f"{prefix}_p95_mm": float(np.percentile(v, 95)),
        f"{prefix}_p99_mm": float(np.percentile(v, 99)),
        f"{prefix}_max_mm": float(np.max(v)),
    }


def _signed_stats(values: np.ndarray, prefix: str) -> dict:
    if len(values) == 0:
        return {}
    v = np.asarray(values) * MM
    return {
        f"{prefix}_bias_mm": float(np.mean(v)),
        f"{prefix}_std_mm": float(np.std(v)),
        f"{prefix}_median_mm": float(np.median(v)),
        f"{prefix}_rmse_mm": float(np.sqrt(np.mean(v**2))),
        f"{prefix}_p05_mm": float(np.percentile(v, 5)),
        f"{prefix}_p95_mm": float(np.percentile(v, 95)),
    }


def compute_metrics(ref, points: np.ndarray, params, seed: int) -> tuple[dict, dict]:
    """Returns (metrics, per_point) where per_point carries the arrays needed
    for the coloured PLY and the plots."""
    thresholds = np.asarray(params.thresholds_mm, dtype=float) / MM
    margin = params.region_margin_mm / MM
    coverage = params.coverage_radius_mm / MM
    outlier = params.outlier_distance_mm / MM

    m: dict = {}
    n_total = len(points)
    region = ref.region_mask(points, margin)
    inside = points[region]
    m["points_total"] = int(n_total)
    m["points_scored"] = int(region.sum())
    m["points_excluded"] = int(n_total - region.sum())
    if len(inside) == 0:
        raise ValueError("no scan points lie over the reference after alignment - the alignment "
                         "failed or the scan does not contain the part")

    # --- accuracy (scan -> reference), exact surface distance ---------------
    signed = ref.signed_distance(inside)
    dist = np.abs(signed)
    m.update(_stats(dist, "acc"))
    m.update(_signed_stats(signed, "signed"))
    m["outlier_fraction"] = float(np.mean(dist > outlier))
    m["outlier_distance_mm"] = params.outlier_distance_mm

    # Robust versions: drop the worst k% by |error|.
    order = np.argsort(dist)
    for pct in params.trim_percent:
        keep = order[: max(1, int(round(len(order) * (1 - pct / 100))))]
        tag = f"trim{pct:g}"
        m[f"acc_rmse_{tag}_mm"] = float(np.sqrt(np.mean(dist[keep] ** 2)) * MM)
        m[f"acc_mae_{tag}_mm"] = float(np.mean(dist[keep]) * MM)
        m[f"signed_bias_{tag}_mm"] = float(np.mean(signed[keep]) * MM)
        m[f"signed_std_{tag}_mm"] = float(np.std(signed[keep]) * MM)

    # --- vertical (Z) error, height fields only --------------------------
    z_err = None
    if ref.is_height_field:
        h = ref.height(inside[:, 0], inside[:, 1])
        ok = np.isfinite(h)
        z_err = np.full(len(inside), np.nan)
        z_err[ok] = inside[ok, 2] - h[ok]
        if ok.any():
            z = z_err[ok] * MM
            m.update({"z_bias_mm": float(np.mean(z)), "z_std_mm": float(np.std(z)),
                      "z_rmse_mm": float(np.sqrt(np.mean(z**2))), "z_mae_mm": float(np.mean(np.abs(z))),
                      "z_p95_abs_mm": float(np.percentile(np.abs(z), 95)),
                      "z_max_abs_mm": float(np.max(np.abs(z)))})

    # --- completeness (reference -> scan) ---------------------------------
    rp, rn = ref.sample(params.reference_samples, seed + 1)
    visible = ref.visible_mask(rp, rn)
    rp = rp[visible]
    tree = cKDTree(points)
    d_ref, _ = tree.query(rp, workers=-1, distance_upper_bound=coverage)
    covered = np.isfinite(d_ref)
    m["reference_samples_visible"] = int(len(rp))
    m["covered_fraction"] = float(covered.mean()) if len(rp) else 0.0
    m["coverage_radius_mm"] = params.coverage_radius_mm
    area = ref.surface_area()
    if area:
        m["reference_area_cm2"] = float(area * 1e4)
        m["covered_area_cm2"] = float(area * m["covered_fraction"] * 1e4)
    d_cov = d_ref[covered]

    # --- threshold metrics -------------------------------------------------
    for tau in thresholds:
        tag = f"{tau * MM:g}mm"
        precision = float(np.mean(dist <= tau))
        recall = float(np.mean(d_cov <= tau)) if len(d_cov) else 0.0
        m[f"precision@{tag}"] = precision
        m[f"completeness@{tag}"] = recall
        m[f"completeness_full@{tag}"] = float(np.mean(np.nan_to_num(d_ref, nan=np.inf) <= tau)) if len(rp) else 0.0
        m[f"fscore@{tag}"] = (2 * precision * recall / (precision + recall)) if precision + recall > 0 else 0.0
    if thresholds.max() >= coverage:
        m["warning_coverage_radius"] = ("coverage radius <= largest threshold: completeness at that "
                                        "threshold is 100% by construction")

    # --- symmetric summaries ----------------------------------------------
    if len(d_cov):
        m["completeness_mean_mm"] = float(np.mean(d_cov) * MM)
        m["completeness_p95_mm"] = float(np.percentile(d_cov, 95) * MM)
        m["chamfer_mm"] = float(0.5 * (np.mean(dist) + np.mean(d_cov)) * MM)
        m["chamfer_rms_mm"] = float(np.sqrt(0.5 * (np.mean(dist**2) + np.mean(d_cov**2))) * MM)
        m["hausdorff_mm"] = float(max(dist.max(), d_cov.max()) * MM)
        m["hausdorff95_mm"] = float(max(np.percentile(dist, 95), np.percentile(d_cov, 95)) * MM)

    # --- density ------------------------------------------------------------
    if len(inside) > 1:
        nn, _ = cKDTree(inside).query(inside, k=2, workers=-1)
        m["spacing_median_mm"] = float(np.median(nn[:, 1]) * MM)
    if area and m["covered_fraction"] > 0:
        m["density_pts_per_cm2"] = float(len(inside) / (area * m["covered_fraction"] * 1e4))

    per_point = {"region": region, "signed": signed, "z_error": z_err}
    return m, per_point
