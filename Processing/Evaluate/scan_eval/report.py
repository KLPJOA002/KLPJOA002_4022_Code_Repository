"""Everything written to disk for one evaluated scan, plus the shared table.

    <output>/<run>/metrics.json        every metric, the parameters, the reference
                                       definition + hash, the alignment report
    <output>/<run>/transform.txt       4x4 scan->reference (metres), reusable as
                                       --init-transform to reproduce the run
    <output>/<run>/aligned_error.ply   aligned scan coloured by signed error, with
                                       the error as a scalar property
    <output>/<run>/error_histogram.png
    <output>/<run>/error_map.png
    <output>/<run>/run.log             console output of this scan's evaluation
    <output>/results.csv               one row appended per scan, across runs
"""

from __future__ import annotations

import csv
import json
import os
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402

# Diverging pair from the shared chart palette: blue = below/inside the
# surface, red = above/outside, neutral gray at zero error (a gray a little
# darker than the surface so near-zero points stay visible).
BLUE, RED, NEUTRAL = "#2a78d6", "#e34948", "#c9c8c3"
SURFACE, INK, INK_2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
ERROR_CMAP = LinearSegmentedColormap.from_list("signed_error", [BLUE, NEUTRAL, RED])
OUTSIDE_RGB = (110, 110, 110)

# Columns that lead every results.csv row, in this order; all other metrics
# follow alphabetically.
LEAD_COLUMNS = ["timestamp", "label", "scan", "reference", "reference_type", "reference_exact",
                "reference_hash", "units", "scale", "allow_scale", "recovered_scale", "seed",
                "acc_rmse_mm", "acc_mae_mm", "acc_median_mm", "signed_bias_mm", "signed_std_mm",
                "acc_rmse_trim1_mm", "acc_rmse_trim5_mm", "covered_fraction", "chamfer_mm",
                "hausdorff95_mm", "outlier_fraction", "align_margin", "align_ambiguous",
                "align_unconstrained", "points_scored"]


def safe_name(text: str) -> str:
    return re.sub(r"[^A-Za-z0-9._-]+", "_", text).strip("_") or "scan"


def _style(ax):
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=INK_2, labelsize=9)
    ax.xaxis.label.set_color(INK_2)
    ax.yaxis.label.set_color(INK_2)


def error_limit(signed_mm: np.ndarray) -> float:
    """Symmetric colour range: 98th percentile of |error|, so a handful of
    outliers do not wash the whole map out to gray."""
    if len(signed_mm) == 0:
        return 1.0
    return float(max(np.percentile(np.abs(signed_mm), 98), 1e-3))


def write_error_ply(path, points, signed_m, region):
    """Binary PLY: xyz, rgb heatmap, and 'error_mm' (signed; NaN off the part)
    plus 'scored' (1 if the point was over the part) as scalar fields, which
    CloudCompare/MeshLab can re-colour however you like."""
    n = len(points)
    err = np.full(n, np.nan, dtype=np.float32)
    err[region] = signed_m * 1e3
    lim = error_limit(err[region])
    rgb = np.tile(np.array(OUTSIDE_RGB, np.uint8), (n, 1))
    if region.any():
        rgb[region] = (ERROR_CMAP(np.clip(err[region] / lim * 0.5 + 0.5, 0, 1))[:, :3] * 255).astype(np.uint8)
    rec = np.empty(n, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("red", "u1"), ("green", "u1"),
                             ("blue", "u1"), ("error_mm", "<f4"), ("scored", "u1")])
    rec["x"], rec["y"], rec["z"] = points[:, 0], points[:, 1], points[:, 2]
    rec["red"], rec["green"], rec["blue"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
    rec["error_mm"] = err
    rec["scored"] = region.astype(np.uint8)
    header = ("ply\nformat binary_little_endian 1.0\n"
              f"comment scan_eval aligned scan, metres, reference frame; colour range +-{lim:.3f} mm\n"
              f"element vertex {n}\nproperty float x\nproperty float y\nproperty float z\n"
              "property uchar red\nproperty uchar green\nproperty uchar blue\n"
              "property float error_mm\nproperty uchar scored\nend_header\n")
    with open(path, "wb") as fh:
        fh.write(header.encode("ascii"))
        fh.write(rec.tobytes())
    return lim


def plot_histogram(path, signed_mm, metrics, title):
    fig, ax = plt.subplots(figsize=(7.5, 4.2), dpi=130, facecolor=SURFACE)
    _style(ax)
    lo, hi = np.percentile(signed_mm, [0.5, 99.5])
    pad = 0.1 * (hi - lo + 1e-9)
    bins = np.linspace(lo - pad, hi + pad, 80)
    clipped = np.clip(signed_mm, bins[0], bins[-1])
    ax.hist(clipped, bins=bins, color=BLUE, edgecolor=SURFACE, linewidth=0.6)
    bias, std = metrics["signed_bias_mm"], metrics["signed_std_mm"]
    ax.axvline(0, color=INK_2, lw=1, ls=(0, (2, 2)))
    ax.axvline(bias, color=INK, lw=1.5)
    ax.axvspan(bias - std, bias + std, color=INK_2, alpha=0.08, lw=0)
    ax.set_xlabel("signed error, mm   (negative = below / inside the reference surface)")
    ax.set_ylabel("points")
    ax.grid(axis="y", color=GRID, lw=0.6)
    ax.set_axisbelow(True)
    text = (f"bias {bias:+.3f} mm   std {std:.3f} mm\nRMSE {metrics['acc_rmse_mm']:.3f} mm   "
            f"MAE {metrics['acc_mae_mm']:.3f} mm   n = {len(signed_mm):,}")
    ax.text(0.99, 0.97, text, transform=ax.transAxes, ha="right", va="top", fontsize=9, color=INK)
    ax.set_title(title, loc="left", fontsize=11, color=INK)
    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


def plot_error_map(path, points, signed_mm, ref, title, z_error_mm=None):
    """Top-down (x, y) for height fields; otherwise the points are projected
    onto their best-fit plane, viewed from the side the scanner sees."""
    if ref.is_height_field:
        a, b = points[:, 0], points[:, 1]
        xlabel, ylabel = "x, mm", "y, mm"
        lo, hi = ref.bounds()
        frame = (lo[0], hi[0], lo[1], hi[1])
    else:
        c = points.mean(axis=0)
        _, vecs = np.linalg.eigh(np.cov((points - c).T))
        n, u = vecs[:, 0], vecs[:, 2]
        if ref.visible_from is not None and n @ ref.visible_from < 0:
            n = -n
        v = np.cross(n, u)
        a, b = (points - c) @ u, (points - c) @ v
        xlabel, ylabel = "in-plane axis 1, mm", "in-plane axis 2, mm"
        frame = None
    panels = [("signed distance to surface", signed_mm)]
    if z_error_mm is not None and np.isfinite(z_error_mm).any():
        panels.append(("vertical (Z) error", z_error_mm))
    fig, axes = plt.subplots(1, len(panels), figsize=(6.2 * len(panels), 5.6), dpi=130,
                             facecolor=SURFACE, squeeze=False)
    order = np.argsort(np.abs(np.nan_to_num(signed_mm)))  # largest errors drawn on top
    size = float(np.clip(6e4 / max(len(points), 1), 0.3, 6))
    for ax, (label, values) in zip(axes[0], panels):
        _style(ax)
        lim = error_limit(values[np.isfinite(values)])
        sc = ax.scatter(a[order] * 1e3, b[order] * 1e3, c=values[order], cmap=ERROR_CMAP,
                        vmin=-lim, vmax=lim, s=size, linewidths=0, rasterized=True)
        if frame is not None:
            x0, x1, y0, y1 = (f * 1e3 for f in frame)
            ax.plot([x0, x1, x1, x0, x0], [y0, y0, y1, y1, y0], color=INK_2, lw=0.8, ls=(0, (3, 2)))
        ax.set_aspect("equal")
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.set_title(label, loc="left", fontsize=10, color=INK)
        cb = fig.colorbar(sc, ax=ax, shrink=0.8, pad=0.02)
        cb.set_label("mm", color=INK_2)
        cb.ax.tick_params(colors=INK_2, labelsize=8)
        cb.outline.set_visible(False)
    fig.suptitle(title, x=0.01, ha="left", fontsize=11, color=INK)
    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


def write_json(path, data):
    def default(o):
        if isinstance(o, np.ndarray):
            return o.tolist()
        if isinstance(o, (np.floating, np.integer)):
            return o.item()
        if isinstance(o, Path):
            return str(o)
        return str(o)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=2, default=default, allow_nan=True)


def append_results(csv_path, row: dict):
    """Append one row. If the columns changed (e.g. different thresholds), the
    file is rewritten with the union of columns so nothing is lost and the
    table stays rectangular."""
    csv_path = Path(csv_path)
    row = {k: ("" if v is None else v) for k, v in row.items()}
    rows, columns = [], []
    if csv_path.exists() and csv_path.stat().st_size > 0:
        with open(csv_path, newline="", encoding="utf-8") as fh:
            reader = csv.DictReader(fh)
            columns = list(reader.fieldnames or [])
            rows = list(reader)
    new_cols = [c for c in row if c not in columns]
    if new_cols or not columns:
        ordered = [c for c in LEAD_COLUMNS if c in row or c in columns]
        rest = sorted((set(columns) | set(row)) - set(ordered))
        columns = ordered + rest
        tmp = csv_path.with_suffix(".csv.tmp")
        with open(tmp, "w", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=columns)
            writer.writeheader()
            writer.writerows(rows)
            writer.writerow(row)
        os.replace(tmp, csv_path)
    else:
        with open(csv_path, "a", newline="", encoding="utf-8") as fh:
            csv.DictWriter(fh, fieldnames=columns).writerow(row)
