"""The results table: one JSON row per (scan, config) job under
results/rows/, assembled into results.parquet / results.csv, plus ranking of
configurations (shared by the stage builder and the plots)."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pandas as pd

# Metrics where bigger is better; everything else (errors in mm, outlier
# fraction, ...) is smaller-is-better.
HIGHER_IS_BETTER = ("completeness", "precision", "fscore", "covered_fraction", "density",
                    "points_", "best_valid_fraction")

LEAD = ["status", "failure_stage", "failure_reason", "stage_first", "config_id", "config_label",
        "shape", "height", "height_mm", "scan_id", "scan_key", "session_kind"]


def higher_is_better(metric: str) -> bool:
    return any(metric.startswith(p) or p in metric for p in HIGHER_IS_BETTER)


def rows_dir(out_dir: Path) -> Path:
    return Path(out_dir) / "rows"


def row_path(out_dir: Path, config_id: str, scan_key: str) -> Path:
    return rows_dir(out_dir) / config_id / f"{scan_key}.json"


def load_rows(out_dir: Path) -> pd.DataFrame:
    rows = []
    for p in sorted(rows_dir(out_dir).glob("*/*.json")):
        try:
            rows.append(json.loads(p.read_text()))
        except (OSError, json.JSONDecodeError):
            continue
    return pd.DataFrame(rows)


def check_registration(df: pd.DataFrame, min_covered: float) -> pd.DataFrame:
    """status 'suspect' where the aligned scan covers too little of the
    reference (a slid-off pose can still show a low RMSE)."""
    if df.empty or "covered_fraction" not in df:
        return df
    ok = df["status"].isin(["ok", "suspect"])
    low = ok & (df["covered_fraction"].astype(float) < min_covered)
    df.loc[ok, "status"] = "ok"
    df.loc[ok, "failure_reason"] = None
    df.loc[low, "status"] = "suspect"
    df.loc[low, "failure_stage"] = "registration"
    df.loc[low, "failure_reason"] = df.loc[low, "covered_fraction"].map(
        lambda c: f"covers only {c:.0%} of the reference (< {min_covered:.0%})")
    return df


def add_relative(df: pd.DataFrame, metric: str, baseline_id: str) -> pd.DataFrame:
    """<metric>_rel: change vs the same scan's baseline (no filters), as a
    fraction (-0.1 = 10% lower). Comparable across shapes and heights."""
    if df.empty or metric not in df:
        return df
    base = df[(df["config_id"] == baseline_id) & (df["status"] == "ok")].set_index("scan_key")[metric]
    df[f"{metric}_rel"] = df[metric].astype(float) / df["scan_key"].map(base).astype(float) - 1.0
    return df


def assemble(out_dir: Path, min_covered: float, baseline_id: str, rank_metric: str) -> pd.DataFrame:
    df = load_rows(out_dir)
    if df.empty:
        return df
    df = check_registration(df, min_covered)
    df = add_relative(df, rank_metric, baseline_id)
    lead = [c for c in LEAD if c in df.columns]
    df = df[lead + [c for c in df.columns if c not in lead]]
    df = df.sort_values(["config_id", "shape", "height_mm", "scan_id"]).reset_index(drop=True)
    out_dir = Path(out_dir)
    tmp = out_dir / "results.parquet.tmp"
    _parquet_safe(df).to_parquet(tmp, index=False)
    tmp.replace(out_dir / "results.parquet")
    df.to_csv(out_dir / "results.csv", index=False)
    return df


def _parquet_safe(df: pd.DataFrame) -> pd.DataFrame:
    """Object columns holding mixed types (e.g. numbers and text) as text."""
    out = df.copy()
    for c in out.columns:
        if out[c].dtype == object:
            types = {type(v) for v in out[c] if v is not None and not (isinstance(v, float) and np.isnan(v))}
            if len(types) > 1:
                out[c] = out[c].map(lambda v: None if v is None else str(v))
    return out


def rank_configs(df: pd.DataFrame, metric: str, config_ids=None, min_ok_fraction=0.8) -> pd.DataFrame:
    """Configurations ranked by the mean per-scan relative change of `metric`
    vs baseline (ok rows only). Configs that failed / were suspect on more
    than 1 - min_ok_fraction of their scans are ranked last."""
    d = df if config_ids is None else df[df["config_id"].isin(config_ids)]
    d = d[d["status"].isin(["ok", "suspect", "failed"])]
    if d.empty:
        return pd.DataFrame()
    rel = f"{metric}_rel"
    ok = d[d["status"] == "ok"]
    g = ok.groupby("config_id")
    out = pd.DataFrame({
        "config_label": d.groupby("config_id")["config_label"].first(),
        "n_scans": d.groupby("config_id").size(),
        "n_ok": g.size(),
        f"mean_{metric}": g[metric].mean(),
        f"std_{metric}": g[metric].std(),
        f"mean_{rel}": g[rel].mean() if rel in ok else np.nan,
    }).fillna({"n_ok": 0})
    out["ok_fraction"] = out["n_ok"] / out["n_scans"]
    key = f"mean_{rel}" if rel in ok else f"mean_{metric}"
    sign = -1 if higher_is_better(metric) else 1
    out["score"] = sign * out[key]
    out.loc[out["ok_fraction"] < min_ok_fraction, "score"] = np.inf
    return out.sort_values("score").reset_index().rename(columns={"index": "config_id"})
