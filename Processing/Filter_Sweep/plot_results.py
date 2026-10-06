#!/usr/bin/env python3
"""Plots and summary tables from a filter sweep's results table. Reads only
results/results.parquet (or .csv) and results/stages/*.csv, so it can be re-run
at any time without re-running the sweep.

  plot_results.py                         all figures into results/plots/
  plot_results.py --results results_validation
  plot_results.py --include-suspect       also use rows that failed the registration check

Figures (PNG + PDF):
  one_factor_overview        stage 1: change of every key metric vs baseline, per filter parameter
  one_factor_<param>         stage 1: key metrics vs one parameter; thin line per height
                             (mean of its scans), thick = mean +- std over all scans of the shape
  heatmap_<metric>__...      stage 2: spatial alpha x temporal alpha, faceted by spatial
                             magnitude (rows) and spatial delta (cols), one figure per
                             (temporal delta, persistence); cell = mean change vs baseline
  baseline_vs_best           baseline vs best configuration, per shape and height
Tables:
  summary_best_per_metric.csv/.md   best configuration for each key metric (all stages)
  ranking_<stage>.csv               configurations ranked by the main metric
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap, TwoSlopeNorm  # noqa: E402

from filter_sweep.config import load_settings  # noqa: E402
from filter_sweep.results import higher_is_better, rank_configs  # noqa: E402

# Evaluate's metric names (results columns) -> axis label with units
METRICS = {
    "acc_rmse_mm": "Accuracy RMSE (mm)",
    "acc_rmse_trim5_mm": "Accuracy RMSE, worst 5% trimmed (mm)",
    "signed_std_mm": "Signed error std (mm)",
    "abs_signed_bias_mm": "|Signed bias| (mm)",
    "z_rmse_mm": "Vertical (Z) RMSE (mm)",
    "covered_fraction": "Covered fraction of reference",
    "completeness_full@2mm": "Completeness @2 mm, whole part",
    "fscore@2mm": "F-score @2 mm",
    "outlier_fraction": "Outlier fraction (> 5 mm)",
    "density_pts_per_cm2": "Point density (pts/cm²)",
}
KEY = ["acc_rmse_mm", "signed_std_mm", "covered_fraction", "fscore@2mm", "completeness_full@2mm",
       "density_pts_per_cm2"]
HEATMAP_METRICS = ["acc_rmse_mm", "signed_std_mm", "fscore@2mm", "completeness_full@2mm"]

PARAM_LABEL = {
    "decimation.magnitude": "Decimation: filter_magnitude",
    "spatial.magnitude": "Spatial: filter_magnitude (iterations)",
    "spatial.alpha": "Spatial: filter_smooth_alpha",
    "spatial.delta": "Spatial: filter_smooth_delta",
    "spatial.holes_fill": "Spatial: holes_fill",
    "temporal.alpha": "Temporal: filter_smooth_alpha",
    "temporal.delta": "Temporal: filter_smooth_delta",
    "temporal.persistence": "Temporal: holes_fill (persistency index)",
    "hole_filling.mode": "Hole filling: holes_fill mode",
}

# Reference palette (dataviz skill, light mode)
INK, INK2, MUTED, GRID, SURFACE = "#0b0b0b", "#52514e", "#8a8984", "#e4e3df", "#fcfcfb"
SHAPE_COLOURS = ["#2a78d6", "#eb6834"]                       # categorical slots 1, 2
HEIGHT_RAMP = ["#86b6ef", "#5598e7", "#2a78d6", "#1c5cab", "#104281", "#0d366b"]  # ordinal blue
DIVERGING = LinearSegmentedColormap.from_list(
    "better_worse", ["#104281", "#3987e5", "#9ec5f4", "#f0efec", "#f2a3a3", "#e34948", "#a32a2a"])

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "axes.edgecolor": MUTED, "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
    "text.color": INK, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
    "axes.spines.top": False, "axes.spines.right": False, "font.size": 9, "axes.titlesize": 10,
    "legend.frameon": False, "lines.linewidth": 2,
})


def save(fig, out: Path, name: str):
    out.mkdir(parents=True, exist_ok=True)
    fig.savefig(out / f"{name}.png", dpi=150, bbox_inches="tight")
    fig.savefig(out / f"{name}.pdf", bbox_inches="tight")
    plt.close(fig)


def load(results_dir: Path, include_suspect: bool):
    p = results_dir / "results.parquet"
    df = pd.read_parquet(p) if p.is_file() else pd.read_csv(results_dir / "results.csv")
    keep = ["ok", "suspect"] if include_suspect else ["ok"]
    stages = {}
    for f in sorted((results_dir / "stages").glob("*.csv")):
        if not f.stem.endswith("_scans"):
            stages[f.stem] = pd.read_csv(f)
    for m in METRICS:
        if m in df:
            df[m] = pd.to_numeric(df[m], errors="coerce")
    if "signed_bias_mm" in df:      # a signed quantity: rank its size
        df["abs_signed_bias_mm"] = pd.to_numeric(df["signed_bias_mm"], errors="coerce").abs()
    return df, df[df["status"].isin(keep)].copy(), stages


def relative(df: pd.DataFrame, metric: str, baseline_id: str, base_from: pd.DataFrame = None) -> pd.Series:
    """Change vs the same scan's baseline row (from base_from, default df)."""
    src = df if base_from is None else base_from
    base = src[(src["config_id"] == baseline_id) & (src["status"] == "ok")].drop_duplicates("scan_key")
    base = base.set_index("scan_key")[metric]
    return df[metric] / df["scan_key"].map(base) - 1.0


def baseline_id(stages, df):
    for s in stages.values():
        b = s[s["config_label"].str.startswith("baseline")]
        if len(b):
            return b["config_id"].iloc[0]
    return df.loc[df["config_label"].str.startswith("baseline"), "config_id"].iloc[0]


# --- stage 1 ---------------------------------------------------------------------------

def one_factor_rows(ok, stage1, settings, base_id):
    """{dotted param: DataFrame with column 'value'} - the param's sweep plus the baseline
    (filter off), placed at the off position for decimation (1)."""
    out = {}
    s1 = stage1.set_index("config_id")
    for dotted, values in settings.stages["1"]["factors"].items():
        name, param = dotted.split(".")
        if dotted == "hdr_merge.enabled":
            continue
        col = f"{name}_{param}"
        others = [f"{f}_on" for f in ("decimation", "hdr_merge", "spatial", "temporal", "hole_filling") if f != name]
        ids = [c for c in s1.index if s1.loc[c, f"{name}_on"] and not s1.loc[c, others].any()]
        d = ok[ok["config_id"].isin(ids)].copy()
        if col in d:
            d["value"] = pd.to_numeric(d[col], errors="coerce")
        d = d[d["value"].isin(values)] if len(d) else d
        if d.empty:
            continue
        b = ok[ok["config_id"] == base_id].copy()
        b["value"] = 1 if dotted == "decimation.magnitude" else np.nan
        out[dotted] = (d, b)
    return out


def plot_one_factor(ok, stage1, settings, base_id, out):
    shapes = [s for s in settings.shapes if s in set(ok["shape"])]
    heights = sorted(ok["height_mm"].dropna().unique())
    hcol = {h: HEIGHT_RAMP[min(i * len(HEIGHT_RAMP) // max(len(heights), 1), len(HEIGHT_RAMP) - 1)]
            for i, h in enumerate(heights)}
    rows = one_factor_rows(ok, stage1, settings, base_id)
    metrics = [m for m in KEY if m in ok]

    for dotted, (d, b) in rows.items():
        fig, axes = plt.subplots(len(metrics), len(shapes), figsize=(4.2 * len(shapes), 2.3 * len(metrics)),
                                 squeeze=False, sharex=True)
        if dotted == "decimation.magnitude":
            d = pd.concat([d, b])
        for j, shape in enumerate(shapes):
            for i, m in enumerate(metrics):
                ax = axes[i, j]
                ds, bs = d[d["shape"] == shape], b[b["shape"] == shape]
                for h in heights:
                    g = ds[ds["height_mm"] == h].groupby("value")[m].mean()
                    ax.plot(g.index, g.values, color=hcol[h], lw=1, marker="o", ms=3, alpha=0.9,
                            label=f"{int(h)} mm")
                g = ds.groupby("value")[m].agg(["mean", "std"])
                ax.fill_between(g.index, g["mean"] - g["std"], g["mean"] + g["std"], color=INK, alpha=0.08, lw=0)
                ax.plot(g.index, g["mean"], color=INK, lw=2, marker="o", ms=4, label="all heights, mean ± std")
                if len(bs) and dotted != "decimation.magnitude":
                    ax.axhline(bs[m].mean(), color=MUTED, lw=1.2, ls="--", label="filter off (baseline)")
                if i == 0:
                    ax.set_title(shape.replace("_", " "))
                if j == 0:
                    ax.set_ylabel(METRICS.get(m, m), fontsize=8)
                if i == len(metrics) - 1:
                    ax.set_xlabel(PARAM_LABEL.get(dotted, dotted))
        handles, labels = axes[0, 0].get_legend_handles_labels()
        fig.legend(handles, labels, loc="upper center", ncol=min(len(labels), 5), bbox_to_anchor=(0.5, 1.0),
                   fontsize=8)
        fig.suptitle(f"Stage 1 - {PARAM_LABEL.get(dotted, dotted)} (all other filters off)", y=1.03)
        fig.tight_layout(rect=(0, 0, 1, 0.97))
        save(fig, out, f"one_factor_{dotted.replace('.', '_')}")

    # overview: mean relative change of each key metric, one panel per parameter
    rel_metrics = [m for m in ["acc_rmse_mm", "signed_std_mm", "fscore@2mm", "covered_fraction"] if m in ok]
    params = list(rows)
    fig, axes = plt.subplots(len(rel_metrics), len(params), figsize=(2.3 * len(params), 2.0 * len(rel_metrics)),
                             squeeze=False, sharey="row")
    for j, dotted in enumerate(params):
        d, b = rows[dotted]
        d = pd.concat([d, b]) if dotted == "decimation.magnitude" else d
        for i, m in enumerate(rel_metrics):
            ax = axes[i, j]
            ax.axhline(0, color=MUTED, lw=1, ls="--")
            for k, shape in enumerate(shapes):
                ds = d[d["shape"] == shape].copy()
                ds["rel"] = relative(ds, m, base_id, ok) * 100
                g = ds.groupby("value")["rel"].agg(["mean", "std"])
                ax.errorbar(g.index, g["mean"], yerr=g["std"], color=SHAPE_COLOURS[k], lw=1.5, marker="o", ms=3,
                            capsize=2, elinewidth=0.8, label=shape.replace("_", " "))
            if i == 0:
                ax.set_title(PARAM_LABEL.get(dotted, dotted).replace(": ", ":\n"), fontsize=8)
            if j == 0:
                ax.set_ylabel(f"{METRICS[m]}\n% change vs baseline", fontsize=7.5)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=len(labels), bbox_to_anchor=(0.5, 1.0))
    fig.suptitle("Stage 1 overview - each filter parameter on its own; mean ± std over scans", y=1.04)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    save(fig, out, "one_factor_overview")


# --- stage 2 ---------------------------------------------------------------------------

def plot_heatmaps(ok, stage2, base_id, out, shapes):
    ids = set(stage2["config_id"])
    d = ok[ok["config_id"].isin(ids)].copy()
    if d.empty:
        return
    cat = lambda on, v: "off" if not on else f"{float(v):g}"
    d["sp_alpha"] = [cat(o, v) for o, v in zip(d["spatial_on"], d["spatial_alpha"])]
    d["tp_alpha"] = [cat(o, v) for o, v in zip(d["temporal_on"], d["temporal_alpha"])]
    d["sp_mag"] = [cat(o, v) for o, v in zip(d["spatial_on"], d["spatial_magnitude"])]
    d["sp_delta"] = [cat(o, v) for o, v in zip(d["spatial_on"], d["spatial_delta"])]
    d["tp_rest"] = ["temporal off" if not o else f"temporal delta {float(dl):g}, persistence {int(p)}"
                    for o, dl, p in zip(d["temporal_on"], d["temporal_delta"], d["temporal_persistence"])]
    order = lambda vals: sorted(set(vals), key=lambda v: (v != "off", float(v) if v != "off" else 0))
    base = ok[ok["config_id"] == base_id]

    for m in [m for m in HEATMAP_METRICS if m in d]:
        bmap = base.set_index("scan_key")[m]
        d["rel"] = (d[m] / d["scan_key"].map(bmap) - 1.0) * 100
        sign = -1 if higher_is_better(m) else 1        # blue = better than baseline, always
        for group in ["all"] + shapes:
            dg = d if group == "all" else d[d["shape"] == group]
            for rest in sorted(set(dg["tp_rest"]) - {"temporal off"}):
                # temporal-off cells belong to every figure (the "off" column)
                sub = dg[(dg["tp_rest"] == rest) | (dg["tp_rest"] == "temporal off")]
                mags = [v for v in order(sub["sp_mag"]) if v != "off"]
                deltas = [v for v in order(sub["sp_delta"]) if v != "off"]
                ys, xs = order(sub["sp_alpha"]), order(sub["tp_alpha"])
                vals = sub.groupby(["sp_mag", "sp_delta", "sp_alpha", "tp_alpha"])["rel"].mean()
                lim = max(1.0, float(np.nanmax(np.abs(vals.values)))) if len(vals) else 1.0
                norm = TwoSlopeNorm(0, -lim, lim)
                fig, axes = plt.subplots(len(mags), len(deltas), figsize=(1.0 + 0.62 * len(xs) * len(deltas),
                                                                           0.6 + 0.48 * len(ys) * len(mags)),
                                         squeeze=False)
                for i, mg in enumerate(mags):
                    for j, dl in enumerate(deltas):
                        grid = np.full((len(ys), len(xs)), np.nan)
                        for a, ya in enumerate(ys):
                            for b_, xa in enumerate(xs):
                                key = ("off", "off", "off", xa) if ya == "off" else (mg, dl, ya, xa)
                                if key in vals.index:
                                    grid[a, b_] = vals[key]
                        ax = axes[i, j]
                        ax.imshow(sign * grid, cmap=DIVERGING, norm=norm, origin="lower", aspect="auto")
                        for a in range(len(ys)):
                            for b_ in range(len(xs)):
                                if np.isfinite(grid[a, b_]):
                                    ax.text(b_, a, f"{grid[a, b_]:+.1f}", ha="center", va="center", fontsize=6.5,
                                            color=INK if abs(grid[a, b_]) < 0.6 * lim else "white")
                        ax.set_xticks(range(len(xs)), xs, fontsize=7)
                        ax.set_yticks(range(len(ys)), ys, fontsize=7)
                        ax.grid(False)
                        if i == len(mags) - 1:
                            ax.set_xlabel("temporal alpha", fontsize=8)
                        if j == 0:
                            ax.set_ylabel(f"spatial magnitude {mg}\nspatial alpha", fontsize=8)
                        if i == 0:
                            ax.set_title(f"spatial delta {dl}", fontsize=8)
                what = "both shapes" if group == "all" else group.replace("_", " ")
                fig.suptitle(f"{METRICS[m]}: % change vs baseline ({what}, mean over scans)\n"
                             f"{rest}; blue = better, red = worse; bottom row / left column = filter off",
                             fontsize=9)
                fig.tight_layout()
                tag = rest.replace("temporal ", "t").replace(", ", "_").replace(" ", "")
                save(fig, out / "heatmaps", f"heatmap_{m.replace('@', 'at')}__{group}__{tag}")


# --- comparison & tables ---------------------------------------------------------------------

def plot_baseline_vs_best(ok, base_id, best_id, best_label, out, shapes, metric):
    metrics = [m for m in ["acc_rmse_mm", "signed_std_mm", "fscore@2mm", "covered_fraction"] if m in ok]
    fig, axes = plt.subplots(len(metrics), len(shapes), figsize=(4.4 * len(shapes), 2.3 * len(metrics)),
                             squeeze=False, sharex=True)
    for j, shape in enumerate(shapes):
        dsh = ok[ok["shape"] == shape]
        heights = sorted(dsh["height_mm"].unique())
        # best per (shape, height) by the main metric
        per_group = {}
        for h, g in dsh.groupby("height_mm"):
            r = rank_configs(g.assign(**{f"{metric}_rel": relative(g, metric, base_id, dsh)}), metric)
            r = r[r["config_id"] != base_id]
            if len(r):
                per_group[h] = r.iloc[0]["config_id"]
        for i, m in enumerate(metrics):
            ax = axes[i, j]
            for k, (label, colour, pick) in enumerate([
                    ("baseline (no filters)", MUTED, lambda h: base_id),
                    (f"best overall: {best_label}", SHAPE_COLOURS[0], lambda h: best_id),
                    ("best for this shape & height", SHAPE_COLOURS[1], lambda h: per_group.get(h))]):
                xs, ys, es = [], [], []
                for n, h in enumerate(heights):
                    g = dsh[(dsh["height_mm"] == h) & (dsh["config_id"] == pick(h))][m]
                    if len(g):
                        xs.append(n + (k - 1) * 0.22)
                        ys.append(g.mean())
                        es.append(g.std() if len(g) > 1 else 0)
                ax.errorbar(xs, ys, yerr=es, fmt="o", color=colour, ms=5, capsize=2, elinewidth=1, label=label)
            if i == 0:
                ax.set_title(shape.replace("_", " "))
            if j == 0:
                ax.set_ylabel(METRICS[m], fontsize=8)
            ax.set_xticks(range(len(heights)), [f"{int(h)}" for h in heights])
            ax.set_xlim(-0.6, len(heights) - 0.4)
            if i == len(metrics) - 1:
                ax.set_xlabel("camera height (mm)")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=3, bbox_to_anchor=(0.5, 1.0), fontsize=8)
    fig.suptitle(f"Baseline vs best configuration (by {metric}); mean ± std over the scans of each height", y=1.03)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    save(fig, out, "baseline_vs_best")


def summary_tables(df, ok, stages, base_id, out, rank_metric):
    out.mkdir(parents=True, exist_ok=True)
    for name, st in stages.items():
        r = rank_configs(df, rank_metric, set(st["config_id"]))
        if len(r):
            r.to_csv(out / f"ranking_{name}.csv", index=False)
    rows = []
    for m in [m for m in METRICS if m in ok]:
        dd = df.copy()
        dd[f"{m}_rel"] = relative(dd, m, base_id)
        for group in ["all"] + sorted(ok["shape"].unique()):
            g = dd if group == "all" else dd[dd["shape"] == group]
            r = rank_configs(g, m)
            if r.empty or not np.isfinite(r.iloc[0]["score"]):
                continue
            best = r.iloc[0]
            base = ok[(ok["config_id"] == base_id) & ((ok["shape"] == group) | (group == "all"))][m].mean()
            rows.append({"metric": m, "label": METRICS[m], "shapes": group, "best_config": best["config_label"],
                         "config_id": best["config_id"], "best_mean": best[f"mean_{m}"],
                         "baseline_mean": base, "change_vs_baseline_pct": 100 * best[f"mean_{m}_rel"],
                         "n_ok": int(best["n_ok"]), "n_scans": int(best["n_scans"])})
    t = pd.DataFrame(rows)
    t.to_csv(out / "summary_best_per_metric.csv", index=False)
    with open(out / "summary_best_per_metric.md", "w") as f:
        f.write("| metric | shapes | best configuration | best mean | baseline mean | change | scans ok |\n"
                "|---|---|---|---|---|---|---|\n")
        for _, r in t.iterrows():
            f.write(f"| {r['label']} | {r['shapes']} | `{r['best_config']}` | {r['best_mean']:.4g} | "
                    f"{r['baseline_mean']:.4g} | {r['change_vs_baseline_pct']:+.1f}% | {r['n_ok']}/{r['n_scans']} |\n")
    return t


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config")
    p.add_argument("--results", help="results folder (default: out_dir from the config)")
    p.add_argument("--include-suspect", action="store_true")
    args = p.parse_args()
    settings = load_settings(args.config)
    results_dir = Path(args.results).resolve() if args.results else settings.out_dir
    out = results_dir / "plots"
    df, ok, stages = load(results_dir, args.include_suspect)
    base_id = baseline_id(stages, df)
    shapes = [s for s in settings.shapes if s in set(ok["shape"])]
    metric = settings.rank_metric
    print(f"{len(df)} rows ({len(ok)} used), stages {sorted(stages)}")

    if "1" in stages:
        plot_one_factor(ok, stages["1"], settings, base_id, out)
    if "2" in stages:
        plot_heatmaps(ok, stages["2"], base_id, out, shapes)
    ranked = rank_configs(df.assign(**{f"{metric}_rel": relative(df, metric, base_id)}), metric)
    ranked = ranked[ranked["config_id"] != base_id]
    if len(ranked):
        best = ranked.iloc[0]
        plot_baseline_vs_best(ok, base_id, best["config_id"], best["config_label"], out, shapes, metric)
    t = summary_tables(df, ok, stages, base_id, out, metric)
    print(t[t["shapes"] == "all"][["metric", "best_config", "best_mean", "baseline_mean",
                                   "change_vs_baseline_pct"]].to_string(index=False))
    print(f"plots and tables: {out}")


if __name__ == "__main__":
    main()
