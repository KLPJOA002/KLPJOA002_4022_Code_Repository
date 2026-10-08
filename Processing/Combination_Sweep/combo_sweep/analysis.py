"""Figures, tables and summary.md from a run directory's results.csv.

Figures (PNG + PDF) go to <run>/analysis/figures/, tables to <run>/analysis/.
Only status == ok rows are used unless include_suspect is set.
Palette and style as Parameter_Sweep/sweep/plots.py (validated, light mode).
"""

from __future__ import annotations

import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402

SURFACE = "#ffffff"
INK = "#0b0b0b"
INK_2 = "#52514e"
GRID = "#e4e3df"
CATEGORICAL = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7", "#e34948"]
MARKERS = ["o", "s", "^", "D", "v", "P", "X", "h"]
BLUE_RAMP = ["#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b"]
ORDINAL_RAMP = ["#86b6ef", "#5598e7", "#2a78d6", "#1c5cab", "#104281", "#0d366b"]
SEQ_CMAP = LinearSegmentedColormap.from_list("seq_blue", BLUE_RAMP)
MISSING = "#f0efec"
FULL_W = 7.0
FAMILY_ORDER = ["single", "rotation", "tilt", "joint", "all"]
FAMILY_LABEL = {"single": "single scan (unfused)", "rotation": "rotation subset, all tilts",
                "tilt": "tilt subset, all rotations", "joint": "rotation x tilt subset", "all": "all 44 scans"}

METRIC_LABEL = {
    "acc_rmse_mm": "Accuracy RMSE (mm)", "acc_rmse_trim5_mm": "Accuracy RMSE, worst 5% trimmed (mm)",
    "signed_std_mm": "Signed error std (mm)", "z_bias_mm": "Z bias (mm)",
    "completeness_full@1mm": "Completeness of whole part @1 mm", "covered_fraction": "Reference covered",
    "chamfer_mm": "Chamfer distance (mm)",
}


def _style():
    plt.rcParams.update({
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
        "font.size": 8.5, "axes.titlesize": 9, "axes.labelsize": 8.5, "xtick.labelsize": 7.5,
        "ytick.labelsize": 7.5, "legend.fontsize": 7.5, "axes.edgecolor": INK_2, "axes.labelcolor": INK,
        "xtick.color": INK_2, "ytick.color": INK_2, "text.color": INK, "axes.linewidth": 0.6,
        "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True, "axes.grid.axis": "y",
        "grid.color": GRID, "grid.linewidth": 0.6, "lines.linewidth": 2.0, "lines.markersize": 4.5,
        "legend.frameon": False, "pdf.fonttype": 42, "ps.fonttype": 42,
    })


def _save(fig, out_dir: Path, name: str, saved: list):
    out_dir.mkdir(parents=True, exist_ok=True)
    for ext in ("png", "pdf"):
        fig.savefig(out_dir / f"{name}.{ext}", dpi=300, bbox_inches="tight")
    plt.close(fig)
    saved.append(name)


def _slug(text):
    return re.sub(r"[^a-z0-9]+", "_", str(text).lower()).strip("_")


def mlabel(m):
    return METRIC_LABEL.get(m, m)


def better(direction, a, b):
    """True if value a is better than b."""
    if direction == "higher":
        return a > b
    if direction == "abs":
        return abs(a) < abs(b)
    return a < b


def score(direction, s: pd.Series) -> pd.Series:
    """Lower is better, for sorting."""
    return -s if direction == "higher" else (s.abs() if direction == "abs" else s)


def _waves(df):
    order = ["Small_Wave", "Big_Wave"]
    return [w for w in order if w in set(df["wave"])] + sorted(set(df["wave"]) - set(order))


def _panels(n, h=2.6):
    fig, axes = plt.subplots(1, n, figsize=(FULL_W, h), sharey=True, squeeze=False)
    return fig, axes[0]


def _legend_below(fig, axes, ncol=4):
    seen = {}
    for ax in axes:
        for h, lab in zip(*ax.get_legend_handles_labels()):
            seen.setdefault(lab, h)
    fig.legend(seen.values(), seen.keys(), loc="upper center", bbox_to_anchor=(0.5, 0.0), ncol=ncol)


def _family_order(cfg, kind):
    return [str(x) for x in cfg["grid"]["rotations" if kind == "rot" else "tilts"]]


# ─────────────────────────────────────────────────────────────────────────────
#  Figures
# ─────────────────────────────────────────────────────────────────────────────

def fig_vs_height(df, cfg, metric, out, saved):
    """Per family: metric vs height, one line per combination, panel per wave."""
    for family, group_col, title in (("rotation", "rot_family", "Rotation subsets (all tilts)"),
                                     ("tilt", "tilt_family", "Tilt subsets (all rotations)")):
        order = _family_order(cfg, "rot" if family == "rotation" else "tilt")
        sub = df[df["family"].isin([family, "all"])]
        if sub.empty:
            continue
        fig, axes = _panels(len(_waves(df)))
        for ax, wave in zip(axes, _waves(df)):
            w = sub[sub["wave"] == wave]
            for i, name in enumerate(order):
                g = w[w[group_col] == name].sort_values("height_mm")
                if g.empty:
                    continue
                lab = f"{group_col.split('_')[0]} {name}" + (" (all 44)" if name == "all" else "")
                ax.plot(g["height_mm"], g[metric], marker=MARKERS[i % 8], color=CATEGORICAL[i % 8], label=lab)
            ax.set_title(wave.replace("_", " "))
            ax.set_xlabel("Camera height (mm)")
        axes[0].set_ylabel(mlabel(metric))
        fig.suptitle(f"{title}: {mlabel(metric)} vs height", y=1.02)
        _legend_below(fig, axes, ncol=4)
        _save(fig, out, f"height_{family}_{_slug(metric)}", saved)

    # Overview: single-scan median (IQR band), key fused combos.
    key = ["rot-0_tilt-all", "rot-all_tilt-0", "rot-0-90_tilt-0-pm10-pm20", "rot-all_tilt-all"]
    fig, axes = _panels(len(_waves(df)))
    for ax, wave in zip(axes, _waves(df)):
        w = df[df["wave"] == wave]
        s = w[w["kind"] == "single"].groupby("height_mm")[metric]
        if len(s):
            q = s.quantile([0.25, 0.5, 0.75]).unstack()
            ax.fill_between(q.index, q[0.25], q[0.75], color=GRID, lw=0, label="single scans, IQR")
            ax.plot(q.index, q[0.5], color=INK_2, lw=1.5, ls="--", label="single scans, median")
        for i, cid in enumerate(key):
            g = w[w["combo_id"] == cid].sort_values("height_mm")
            if len(g):
                ax.plot(g["height_mm"], g[metric], marker=MARKERS[i], color=CATEGORICAL[i], label=cid)
        ax.set_title(wave.replace("_", " "))
        ax.set_xlabel("Camera height (mm)")
    axes[0].set_ylabel(mlabel(metric))
    _legend_below(fig, axes, ncol=3)
    _save(fig, out, f"height_overview_{_slug(metric)}", saved)


def fig_vs_angle(df, cfg, metric, out, saved):
    singles = df[df["kind"] == "single"].copy()
    if not singles.empty:
        singles["rot"] = singles["rotations"].astype(int)
        singles["tilt"] = singles["tilts"].astype(int)
        # vs tilt: one line per rotation, median over heights
        fig, axes = _panels(len(_waves(df)))
        for ax, wave in zip(axes, _waves(df)):
            w = singles[singles["wave"] == wave]
            for i, r in enumerate(sorted(w["rot"].unique())):
                g = w[w["rot"] == r].groupby("tilt")[metric].median()
                ax.plot(g.index, g.values, marker=MARKERS[i], color=CATEGORICAL[i], label=f"rotation {r}°")
            ax.set_title(wave.replace("_", " "))
            ax.set_xlabel("Tilt (°)")
        axes[0].set_ylabel(f"{mlabel(metric)}\nmedian over heights")
        fig.suptitle("Single scans (unfused) vs tilt", y=1.02)
        _legend_below(fig, axes)
        _save(fig, out, f"angle_single_vs_tilt_{_slug(metric)}", saved)

        # vs rotation: one line per |tilt|
        fig, axes = _panels(len(_waves(df)))
        for ax, wave in zip(axes, _waves(df)):
            w = singles[singles["wave"] == wave].assign(abs_tilt=lambda d: d["tilt"].abs())
            mags = sorted(w["abs_tilt"].unique())
            for i, t in enumerate(mags):
                g = w[w["abs_tilt"] == t].groupby("rot")[metric].median()
                c = ORDINAL_RAMP[min(i, len(ORDINAL_RAMP) - 1)]
                ax.plot(g.index, g.values, marker="o", color=c, label=f"|tilt| {t}°")
            ax.set_xticks(sorted(w["rot"].unique()))
            ax.set_title(wave.replace("_", " "))
            ax.set_xlabel("Rotation (°)")
        axes[0].set_ylabel(f"{mlabel(metric)}\nmedian over heights and tilt sign")
        fig.suptitle("Single scans (unfused) vs rotation", y=1.02)
        _legend_below(fig, axes, ncol=6)
        _save(fig, out, f"angle_single_vs_rotation_{_slug(metric)}", saved)

    # fused: which tilts / rotations are included (dots = heights, line = median)
    for family, col, xlabel in (("tilt", "tilt_family", "Tilts fused (all rotations)"),
                                ("rotation", "rot_family", "Rotations fused (all tilts)")):
        order = _family_order(cfg, "tilt" if family == "tilt" else "rot")
        sub = df[df["family"].isin([family, "all"])]
        if sub.empty:
            continue
        fig, axes = _panels(len(_waves(df)), h=3.0)
        heights = sorted(df["height_mm"].unique())
        hcol = {h: ORDINAL_RAMP[int(round(i * (len(ORDINAL_RAMP) - 1) / max(len(heights) - 1, 1)))]
                for i, h in enumerate(heights)}
        for ax, wave in zip(axes, _waves(df)):
            w = sub[sub["wave"] == wave]
            names = [n for n in order if n in set(w[col])]
            x = {n: i for i, n in enumerate(names)}
            for h in heights:
                g = w[w["height_mm"] == h]
                ax.scatter([x[n] for n in g[col]], g[metric], s=14, color=hcol[h], label=f"{h} mm", zorder=3)
            med = w.groupby(col)[metric].median().reindex(names)
            ax.plot(range(len(names)), med.values, color=INK, lw=1.2, label="median over heights", zorder=2)
            ax.set_xticks(range(len(names)))
            ax.set_xticklabels([n.replace("pm", "±") for n in names], rotation=45, ha="right")
            ax.set_title(wave.replace("_", " "))
            ax.set_xlabel(xlabel)
        axes[0].set_ylabel(mlabel(metric))
        _legend_below(fig, axes, ncol=7)
        _save(fig, out, f"angle_fused_{family}_{_slug(metric)}", saved)


def fig_vs_nscans(df, metric, out, saved):
    fams = [f for f in FAMILY_ORDER if f in set(df["family"])]
    fig, axes = _panels(len(_waves(df)), h=3.0)
    for ax, wave in zip(axes, _waves(df)):
        w = df[df["wave"] == wave]
        for i, f in enumerate(fams):
            g = w[w["family"] == f]
            ax.scatter(g["n_scans"], g[metric], s=12, marker=MARKERS[i], color=CATEGORICAL[i],
                       label=FAMILY_LABEL[f], alpha=0.8, zorder=3)
        med = w.groupby("n_scans")[metric].median()
        ax.plot(med.index, med.values, color=INK, lw=1.2, label="median at each scan count", zorder=2)
        ax.set_xscale("log", base=2)
        ax.set_xticks([1, 2, 4, 8, 16, 32, 44])
        ax.get_xaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())
        ax.set_title(wave.replace("_", " "))
        ax.set_xlabel("Scans fused (log scale)")
    axes[0].set_ylabel(mlabel(metric))
    _legend_below(fig, axes, ncol=3)
    _save(fig, out, f"nscans_{_slug(metric)}", saved)


def fig_runtime(df, metric, out, saved):
    fused = df[df["kind"] == "fused"]
    if fused.empty or "fuse_s" not in fused:
        return
    fig, axes = plt.subplots(1, 2, figsize=(FULL_W, 2.8), squeeze=False)
    axes = axes[0]
    for i, wave in enumerate(_waves(fused)):
        w = fused[fused["wave"] == wave]
        axes[0].scatter(w["n_scans"], w["fuse_s"] / 60, s=12, color=CATEGORICAL[i], marker=MARKERS[i],
                        label=wave.replace("_", " "))
        axes[1].scatter(w["fuse_s"] / 60, w[metric], s=12, color=CATEGORICAL[i], marker=MARKERS[i],
                        label=wave.replace("_", " "))
    axes[0].set_xlabel("Scans fused")
    axes[0].set_ylabel("Fusion time (min)")
    axes[0].set_title("Cost")
    axes[1].set_xlabel("Fusion time (min)")
    axes[1].set_ylabel(mlabel(metric))
    axes[1].set_title("Quality vs cost")
    axes[0].legend()
    fig.tight_layout()
    _save(fig, out, f"runtime_{_slug(metric)}", saved)


def fig_heatmaps(df, cfg, metric, out, saved):
    fused = df[df["kind"] == "fused"]
    if fused.empty:
        return
    rots, tilts = _family_order(cfg, "rot"), _family_order(cfg, "tilt")
    waves = _waves(fused)
    fig, axes = plt.subplots(1, len(waves), figsize=(FULL_W, 3.4), squeeze=False)
    axes = axes[0]
    for ax, wave in zip(axes, waves):
        w = fused[fused["wave"] == wave]
        grid = w.groupby(["rot_family", "tilt_family"])[metric].median().unstack()
        grid = grid.reindex(index=rots, columns=tilts)
        cmap = SEQ_CMAP if metric in ("completeness_full@1mm", "covered_fraction") else SEQ_CMAP.reversed()
        cmap = cmap.copy()
        cmap.set_bad(MISSING)
        im = ax.imshow(np.ma.masked_invalid(grid.values.astype(float)), cmap=cmap, aspect="auto")
        vals = grid.values.astype(float)
        lo, hi = np.nanmin(vals) if np.isfinite(vals).any() else 0, np.nanmax(vals) if np.isfinite(vals).any() else 1
        for (r, c), v in np.ndenumerate(vals):
            if np.isfinite(v):
                frac = (v - lo) / (hi - lo) if hi > lo else 0.5
                dark = frac > 0.55 if cmap.name == "seq_blue" else frac < 0.45
                ax.text(c, r, f"{v:.2f}", ha="center", va="center", fontsize=6,
                        color=SURFACE if dark else INK)
        ax.set_xticks(range(len(tilts)))
        ax.set_xticklabels([t.replace("pm", "±") for t in tilts], rotation=45, ha="right")
        ax.set_yticks(range(len(rots)))
        ax.set_yticklabels(rots)
        ax.set_xlabel("Tilt family")
        ax.set_title(wave.replace("_", " "))
        ax.grid(False)
        fig.colorbar(im, ax=ax, shrink=0.8, label=mlabel(metric))
    axes[0].set_ylabel("Rotation family")
    fig.suptitle(f"Rotation x tilt grid, median over heights ({metric})", y=1.02)
    fig.tight_layout()
    _save(fig, out, f"heatmap_{_slug(metric)}", saved)


def fig_voxel(df, metric, out, saved):
    v = df[(df["group"] == "voxel") & (df["kind"] == "fused")]
    if v.empty:
        return
    v = v.assign(pair=v["scan_voxel"].astype(float).map(lambda x: f"{x * 1e3:g}") + "/" +
                 v["fusion_cell"].astype(float).map(lambda x: f"{x * 1e3:g}"))
    combos = sorted(v["combo_id"].unique())
    fig, axes = _panels(len(_waves(v)), h=3.0)
    for ax, wave in zip(axes, _waves(v)):
        w = v[v["wave"] == wave]
        for i, cid in enumerate(combos):
            g = w[w["combo_id"] == cid].groupby("pair").agg(t=("fuse_s", "median"), m=(metric, "median"))
            g = g.sort_values("t")
            ax.plot(g["t"] / 60, g["m"], marker=MARKERS[i % 8], color=CATEGORICAL[i % 8], label=cid, lw=1.2)
            for pair, r in g.iterrows():
                ax.annotate(pair, (r["t"] / 60, r["m"]), fontsize=5.5, color=INK_2,
                            xytext=(3, 3), textcoords="offset points")
        ax.set_xscale("log")
        ax.set_title(wave.replace("_", " "))
        ax.set_xlabel("Fusion time (min, log); labels: scan-voxel/fusion-cell mm")
    axes[0].set_ylabel(f"{mlabel(metric)}\nmedian over heights")
    _legend_below(fig, axes, ncol=2)
    _save(fig, out, f"voxel_{_slug(metric)}", saved)


# ─────────────────────────────────────────────────────────────────────────────
#  Tables
# ─────────────────────────────────────────────────────────────────────────────

def rankings(df, metric, direction, top=10):
    out = []
    for (wave, h), g in df.groupby(["wave", "height_mm"]):
        g = g.assign(_s=score(direction, g[metric])).sort_values(["_s", "n_scans"]).head(top)
        g = g.assign(rank=range(1, len(g) + 1))
        out.append(g[["wave", "height_mm", "rank", "combo_id", "family", "n_scans", metric, "fuse_s", "total_s"]])
    return pd.concat(out) if out else pd.DataFrame()


def smallest_within(df, metric, direction, pct):
    """Per (wave, height): the fewest scans whose metric is within pct% of all-44."""
    rows = []
    for (wave, h), g in df.groupby(["wave", "height_mm"]):
        ref = g[g["combo_id"] == "rot-all_tilt-all"]
        if ref.empty:
            continue
        ref_v = float(ref[metric].iloc[0])
        tol = abs(ref_v) * pct / 100
        if direction == "higher":
            ok = g[g[metric] >= ref_v - tol]
        elif direction == "abs":
            ok = g[g[metric].abs() <= abs(ref_v) + tol]
        else:
            ok = g[g[metric] <= ref_v + tol]
        if ok.empty:
            continue
        ok = ok.assign(_s=score(direction, ok[metric])).sort_values(["n_scans", "_s"])
        best = ok.iloc[0]
        rows.append({"wave": wave, "height_mm": h, "all44_" + metric: ref_v,
                     "all44_fuse_s": ref["fuse_s"].iloc[0], "smallest_combo": best["combo_id"],
                     "n_scans": int(best["n_scans"]), metric: best[metric],
                     "fuse_s": best.get("fuse_s"), "n_candidates_within": len(ok)})
    return pd.DataFrame(rows)


def _md_table(df, floatfmt=".3f"):
    if df.empty:
        return "_(none)_\n"
    cols = list(df.columns)
    lines = ["| " + " | ".join(cols) + " |", "|" + "|".join("---" for _ in cols) + "|"]
    for _, r in df.iterrows():
        cells = []
        for c in cols:
            v = r[c]
            cells.append(f"{v:{floatfmt}}" if isinstance(v, float) and np.isfinite(v) else str(v))
        lines.append("| " + " | ".join(cells) + " |")
    return "\n".join(lines) + "\n"


# ─────────────────────────────────────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────────────────────────────────────

def analyze(cfg, run_dir: Path, within=None, include_suspect=None):
    run_dir = Path(run_dir)
    res = pd.read_csv(run_dir / "results.csv")
    acfg = cfg["analysis"]
    within = within if within is not None else acfg.get("within_percent", 5)
    include_suspect = acfg.get("include_suspect", False) if include_suspect is None else include_suspect
    metric, metrics = acfg["primary_metric"], acfg["metrics"]
    direction = metrics.get(metric, "lower")
    out = run_dir / "analysis"
    figs = out / "figures"
    out.mkdir(parents=True, exist_ok=True)
    _style()

    keep = ["ok", "suspect"] if include_suspect else ["ok"]
    df = res[res["status"].isin(keep)].copy()
    main = df[df["group"] != "voxel"]
    present = [m for m in metrics if m in df.columns]
    saved = []
    if not main.empty:
        for m in present:
            fig_vs_height(main, cfg, m, figs, saved)
            fig_vs_nscans(main, m, figs, saved)
            fig_heatmaps(main, cfg, m, figs, saved)
        for m in [metric] + [x for x in ("completeness_full@1mm",) if x in present and x != metric]:
            fig_vs_angle(main, cfg, m, figs, saved)
        fig_runtime(main, metric, figs, saved)
    for m in [metric] + [x for x in ("completeness_full@1mm",) if x in present and x != metric]:
        fig_voxel(df, m, figs, saved)

    rank = rankings(main, metric, direction) if not main.empty else pd.DataFrame()
    small = smallest_within(main, metric, direction, within) if not main.empty else pd.DataFrame()
    rank.to_csv(out / "ranking_top10.csv", index=False)
    small.to_csv(out / f"smallest_within_{within:g}pct.csv", index=False)
    write_summary(cfg, res, main, df, rank, small, metric, direction, within, saved, out)
    print(f"{len(saved)} figures -> {figs}\nTables and summary.md -> {out}")


def write_summary(cfg, res, main, df, rank, small, metric, direction, within, saved, out):
    L = ["# Rotation x tilt combination sweep: summary", ""]
    counts = res["status"].value_counts().to_dict()
    L += [f"Rows: {len(res)} ({', '.join(f'{k} {v}' for k, v in counts.items())}). "
          f"Primary metric: `{metric}` ({direction} is better). Figures: `analysis/figures/`.", ""]
    failed = res[res["status"] != "ok"]
    if len(failed):
        L += ["## Failed / suspect jobs", "",
              _md_table(failed[["wave", "height", "combo_id", "n_scans", "status", "failure_stage", "failure_reason"]]
                        .fillna("").head(40)), ""]

    if not main.empty:
        L += ["## Key numbers (median over heights)", ""]
        rows = []
        for wave, w in main.groupby("wave"):
            all44 = w[w["combo_id"] == "rot-all_tilt-all"]
            singles = w[w["kind"] == "single"]
            r = {"wave": wave, "all-44 " + metric: all44[metric].median() if len(all44) else np.nan,
                 "all-44 fuse min": all44["fuse_s"].median() / 60 if len(all44) else np.nan,
                 "single median": singles[metric].median() if len(singles) else np.nan,
                 "best single (median over heights)": np.nan, "best single scan": ""}
            if len(singles):
                per = singles.groupby("combo_id")[metric].median()
                b = per.loc[score(direction, per).idxmin()]
                r["best single (median over heights)"] = b
                r["best single scan"] = score(direction, per).idxmin()
            fused = w[w["kind"] == "fused"]
            if len(fused):
                per = fused.groupby("combo_id")[metric].median()
                bid = score(direction, per).idxmin()
                r["best fused combo"] = bid
                r["best fused " + metric] = per.loc[bid]
            rows.append(r)
        L += [_md_table(pd.DataFrame(rows)), ""]

        L += [f"## Smallest subset within {within:g}% of all-44 ({metric})", "", _md_table(small), ""]
        if len(small):
            freq = small["smallest_combo"].value_counts().head(5)
            L += ["Most frequent smallest subsets: " + ", ".join(f"`{k}` ({v}x)" for k, v in freq.items()), ""]

        # tilt sign: 0-pos vs symmetric of similar size
        sign = main[main["tilt_family"].isin(["0-pos", "all"]) & (main["rot_family"] == "all")]
        if sign["tilt_family"].nunique() == 2:
            p = sign.pivot_table(index=["wave", "height_mm"], columns="tilt_family", values=metric)
            d = (p["0-pos"] - p["all"]).median()
            L += [f"Tilt sign: fusing only positive tilts (6 per rotation) vs all 11 tilts changes "
                  f"`{metric}` by a median of {d:+.3f} over (wave, height).", ""]

        L += ["## Best 3 combinations per height", "",
              _md_table(rank[rank["rank"] <= 3]) if len(rank) else "_(none)_\n", ""]

    v = df[df["group"] == "voxel"]
    if len(v):
        L += ["## Voxel / cell sensitivity", ""]
        t = v.groupby(["combo_id", "scan_voxel", "fusion_cell"]).agg(
            n=(metric, "size"), metric=(metric, "median"), fuse_min=("fuse_s", lambda s: s.median() / 60),
            rss_mb=("fuse_peak_rss_mb", "max")).reset_index().rename(columns={"metric": metric})
        L += [_md_table(t), ""]

    L += ["## Caveats on the metrics", "",
          "- **Single scans are not like-for-like with fused clouds.** fuse_scans resamples onto a "
          "`fusion-cell` height-field grid, averages overlapping scans and smooths (MLS), which lowers random "
          "error by itself. Single scans are scored as cropped, at native density. Compare single vs fused "
          "with that in mind; fused vs fused comparisons share the same resampling.",
          "- **Completeness and density metrics depend on point count**: `completeness@τ` is measured "
          "from reference samples to the nearest scan point, so sparser clouds lose completeness even when "
          "accurate. Fused clouds all share one grid spacing; single scans do not.",
          "- **Accuracy RMSE is computed only over scored points**: a combination that leaves holes "
          "(e.g. steep wave flanks no scan saw) can show a lower RMSE by having fewer hard points. Read it "
          "together with `covered_fraction` / `completeness_full@1mm`.",
          "- Each (wave, height, combination) is one fusion of one capture; differences smaller than the "
          "spread between neighbouring heights are probably noise.", ""]
    L += ["## Figures", ""] + [f"- `{n}`" for n in saved] + [""]
    (out / "summary.md").write_text("\n".join(L))
