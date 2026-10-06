"""Shared by plotting and ranking: which parameters varied, which pairs were
varied together, and metric metadata. Works only from all_scans.csv."""

from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations

import pandas as pd


@dataclass(frozen=True)
class Metric:
    column: str
    label: str
    unit: str            # "mm" or "%"
    lower_is_better: bool
    percent: bool = False  # stored as a fraction, shown as %

    def values(self, df):
        v = pd.to_numeric(df[self.column], errors="coerce")
        return v * 100 if self.percent else v

    @property
    def axis_label(self):
        return f"{self.label} ({self.unit})"


METRICS = {m.column: m for m in [
    Metric("acc_rmse_mm", "RMSE", "mm", True),
    Metric("acc_mae_mm", "Mean abs. error", "mm", True),
    Metric("acc_rmse_trim5_mm", "RMSE, worst 5% removed", "mm", True),
    Metric("signed_bias_mm", "Signed bias", "mm", True),
    Metric("signed_std_mm", "Signed error std. dev.", "mm", True),
    Metric("acc_p95_mm", "95th pct. abs. error", "mm", True),
    Metric("fscore@2mm", "F-score at 2 mm", "%", False, True),
    Metric("completeness@2mm", "Completeness at 2 mm (seen area)", "%", False, True),
    Metric("completeness_full@2mm", "Completeness at 2 mm", "%", False, True),
    Metric("covered_fraction", "Coverage of reference", "%", False, True),
    Metric("outlier_fraction", "Outliers beyond 5 mm", "%", True, True),
    Metric("chamfer_mm", "Chamfer distance", "mm", True),
]}

# Metrics shown in each per-parameter figure (2 x 3 grid).
PANEL_METRICS = ["acc_rmse_mm", "acc_rmse_trim5_mm", "signed_bias_mm",
                 "signed_std_mm", "fscore@2mm", "completeness_full@2mm"]


@dataclass(frozen=True)
class Param:
    column: str
    label: str
    kind: str                 # "numeric" (line plot) or "categorical" (box + points)
    order: tuple | None = None
    log_x: bool = False
    x_column: str | None = None   # numeric position (e.g. megapixels for resolution)


PRESET_ORDER = ("default", "high_accuracy", "high_density", "medium_density", "remove_ir_pattern")
PRESET_LABELS = {"default": "Default", "high_accuracy": "High accuracy", "high_density": "High density",
                 "medium_density": "Medium density", "remove_ir_pattern": "Remove IR pattern"}
PRESET_SHORT = {"default": "Default", "high_accuracy": "High acc.", "high_density": "High dens.",
                "medium_density": "Med. dens.", "remove_ir_pattern": "Remove IR"}

KNOWN_PARAMS = [
    Param("resolution", "Depth resolution (px)", "numeric", x_column="res_megapixels"),
    Param("depth_units_mm", "Depth units (mm)", "numeric", log_x=True),
    Param("preset", "Visual preset", "categorical", PRESET_ORDER),
    Param("laser_power", "Laser power (mW)", "numeric"),
    Param("exposure_us", "Exposure (µs)", "numeric"),
    Param("gain", "Gain", "numeric"),
    Param("auto_exposure", "Auto exposure", "categorical"),
    Param("emitter", "Emitter", "categorical"),
    Param("filter_chain", "Filter chain", "categorical"),
    Param("fps", "Frame rate (fps)", "numeric"),
]
# Raw scan_info keys that are camera *settings* (others are paths, times, results).
SETTING_PREFIXES = ("info.Stereo Module.", "info.filter.", "info.RGB Camera.", "info.Motion Module.")
# Covered by `preset` (requested) with `preset_actual` / `preset_mismatch` alongside.
NOT_PARAMETERS = {"info.Stereo Module.Visual Preset"}


def _partition(series):
    return frozenset(frozenset(idx) for idx in series.astype(str).groupby(series.astype(str)).groups.values())


def varied_parameters(df) -> list[Param]:
    """Parameters with more than one value, in KNOWN_PARAMS order, plus any
    raw camera setting that varies and isn't just another name for one of them
    (e.g. 'Stereo Module.Depth Units' duplicates depth_units_mm)."""
    out, partitions = [], []
    for p in KNOWN_PARAMS:
        if p.column in df and df[p.column].nunique(dropna=True) > 1:
            out.append(p)
            partitions.append(_partition(df[p.column]))
    for col in sorted(c for c in df.columns if c.startswith(SETTING_PREFIXES) and c not in NOT_PARAMETERS):
        if df[col].nunique(dropna=True) <= 1:
            continue
        part = _partition(df[col])
        # Same grouping as a known parameter, or a refinement of the preset
        # grouping (Visual Preset 'Custom' etc.), adds nothing.
        if any(part == q for q in partitions):
            continue
        if any(all(any(g <= h for h in q) for g in part) for q in partitions):
            continue
        numeric = pd.to_numeric(df[col], errors="coerce").notna().all()
        out.append(Param(col, col.removeprefix("info."), "numeric" if numeric else "categorical"))
        partitions.append(part)
    return out


def levels(df, p: Param) -> list:
    present = [v for v in df[p.column].dropna().unique()]
    if p.order:
        return [v for v in p.order if v in present] + sorted(set(present) - set(p.order), key=str)
    if p.x_column and p.x_column in df:
        pos = df.groupby(p.column)[p.x_column].first()
        return sorted(present, key=lambda v: pos.get(v, 0))
    try:
        return sorted(present, key=float)
    except (TypeError, ValueError):
        return sorted(present, key=str)


def level_label(p: Param, v, short=False) -> str:
    if p.column == "preset":
        return (PRESET_SHORT if short else PRESET_LABELS).get(v, str(v))
    if p.column == "resolution":
        return str(v).replace("x", "×")
    if isinstance(v, float) and v.is_integer():
        return f"{int(v)}"
    return f"{v:g}" if isinstance(v, float) else str(v)


def crossed_pairs(df, params) -> list[tuple[Param, Param]]:
    """Pairs that were varied *together*: some 2 levels of A each appear with
    the same 2 levels of B (a 2 x 2 sub-grid exists) within one object. A one-factor-at-a-time
    sweep has no such pair, so it gets no heatmaps."""
    out = []
    for a, b in combinations(params, 2):
        for _, sub in df.groupby("object_label"):   # crossed within one object, not pooled
            combos = sub[[a.column, b.column]].dropna().astype(str).drop_duplicates()
            sets = combos.groupby(a.column)[b.column].apply(frozenset)
            if any(len(s1 & s2) >= 2 for s1, s2 in combinations(sets.tolist(), 2)):
                out.append((a, b))
                break
    return out


def config_key(df, params) -> pd.Series:
    return df[[p.column for p in params]].astype(str).agg(" | ".join, axis=1)


def usable(df, include_suspect=False):
    keep = ["ok", "suspect"] if include_suspect else ["ok"]
    return df[df["status"].isin(keep)].copy()
