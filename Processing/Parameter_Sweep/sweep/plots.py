"""All figures, from all_scans.csv only (re-plot without re-evaluating).

Each figure is written as PNG (300 dpi) and PDF to the plots directory:

  param_<name>.png         metrics vs each varied parameter, objects side by side
  object_comparison.png    same configuration on each object (does the best generalise?)
  heatmap_<a>_x_<b>.png    main metric over each pair of parameters varied together
  tradeoff_<param>.png     completeness vs error, coloured by each parameter
  diagnostics.png          outlier influence and registration quality per scan
"""

from __future__ import annotations

import re
from itertools import combinations
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

from . import config as C  # noqa: E402
from .analysis import (METRICS, PANEL_METRICS, Metric, config_key, crossed_pairs, level_label,  # noqa: E402
                       levels, usable, varied_parameters)

# --- palette (validated reference palette, light mode: thesis figures are printed) ---
SURFACE = "#ffffff"
INK = "#0b0b0b"
INK_2 = "#52514e"
GRID = "#e4e3df"
CATEGORICAL = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7", "#e34948"]
MARKERS = ["o", "s", "^", "D", "v", "P", "X", "h"]
BLUE_RAMP = ["#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b"]
ORDINAL_RAMP = ["#86b6ef", "#5598e7", "#2a78d6", "#1c5cab", "#104281", "#0d366b"]  # >= step 250
SEQ_CMAP = LinearSegmentedColormap.from_list("seq_blue", BLUE_RAMP)
MISSING = "#f0efec"

FULL_W = 7.0  # in: a thesis text width with a little room


def _style():
    plt.rcParams.update({
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
        "font.size": 8.5, "axes.titlesize": 9, "axes.labelsize": 8.5, "xtick.labelsize": 7.5,
        "ytick.labelsize": 7.5, "legend.fontsize": 7.5, "axes.edgecolor": INK_2, "axes.labelcolor": INK,
        "xtick.color": INK_2, "ytick.color": INK_2, "text.color": INK, "axes.linewidth": 0.6,
        "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True, "axes.grid.axis": "y",
        "grid.color": GRID, "grid.linewidth": 0.6, "lines.linewidth": 2.0, "legend.frameon": False,
        "pdf.fonttype": 42, "ps.fonttype": 42,
    })


def _save(fig, out_dir: Path, name: str):
    out_dir.mkdir(parents=True, exist_ok=True)
    for ext in ("png", "pdf"):
        fig.savefig(out_dir / f"{name}.{ext}", dpi=300, bbox_inches="tight")
    plt.close(fig)
    return name


def _slug(text):
    return re.sub(r"[^a-z0-9]+", "_", str(text).lower()).strip("_")


def _objects(df):
    return sorted(df["object_label"].unique())


def _object_style(objs):
    return {o: (CATEGORICAL[i], MARKERS[i]) for i, o in enumerate(objs)}


def _level_styles(df, p):
    """Colour + marker per level: categorical hues in fixed order for an
    unordered parameter, an ordinal blue ramp for an ordered one."""
    levs = levels(df, p)
    if p.kind == "categorical":
        return {v: (CATEGORICAL[i % 8], MARKERS[i % 8]) for i, v in enumerate(levs)}
    idx = np.linspace(0, len(ORDINAL_RAMP) - 1, len(levs)).round().astype(int) if len(levs) > 1 else [2]
    return {v: (ORDINAL_RAMP[k], MARKERS[i % 8]) for i, (v, k) in enumerate(zip(levs, idx))}


def _jitter(n, width, seed):
    return np.random.default_rng(seed).uniform(-width, width, n)


def _x_positions(df, p):
    levs = levels(df, p)
    if p.kind == "numeric" and p.log_x:
        try:
            return {v: np.log10(float(v)) for v in levs}, True
        except (TypeError, ValueError):
            pass
    return {v: float(i) for i, v in enumerate(levs)}, False


def _tick_labels(ax, p, levs):
    crowded = p.kind == "categorical" and len(levs) > 3
    ax.set_xticklabels([level_label(p, v, short=crowded) for v in levs],
                       rotation=35 if crowded else 0, ha="right" if crowded else "center",
                       rotation_mode="anchor" if crowded else "default")


# --- 1. metrics vs each parameter ------------------------------------------------------
def plot_parameter(df, p, out_dir):
    objs = _objects(df)
    ostyle = _object_style(objs)
    pos, is_log = _x_positions(df, p)
    levs = list(pos)
    span = (max(pos.values()) - min(pos.values())) or 1.0
    step = span / max(len(levs) - 1, 1)
    metrics = [METRICS[m] for m in PANEL_METRICS if m in df]
    ncol = 3
    nrow = int(np.ceil(len(metrics) / ncol))
    fig, axes = plt.subplots(nrow, ncol, figsize=(FULL_W, 2.25 * nrow + 0.5), squeeze=False)
    dodge = step * 0.2 if len(objs) > 1 else 0.0

    for ax, m in zip(axes.flat, metrics):
        for j, obj in enumerate(objs):
            color, marker = ostyle[obj]
            sub = df[df["object_label"] == obj]
            off = (j - (len(objs) - 1) / 2) * dodge
            vals = [m.values(sub[sub[p.column] == v]).dropna().to_numpy() for v in levs]
            xs = np.array([pos[v] + off for v in levs])
            if p.kind == "categorical":
                ok = [i for i, v in enumerate(vals) if len(v)]
                ax.boxplot([vals[i] for i in ok], positions=xs[ok], widths=dodge * 1.6 or step * 0.5,
                                showfliers=False, patch_artist=True, manage_ticks=False,
                                medianprops=dict(color=color, linewidth=1.6),
                                boxprops=dict(facecolor=color + "26", edgecolor=color, linewidth=0.8),
                                whiskerprops=dict(color=color, linewidth=0.8), capprops=dict(color=color, linewidth=0.8))
            else:
                med = np.array([np.median(v) if len(v) else np.nan for v in vals])
                q1 = np.array([np.percentile(v, 25) if len(v) else np.nan for v in vals])
                q3 = np.array([np.percentile(v, 75) if len(v) else np.nan for v in vals])
                ax.fill_between(xs, q1, q3, color=color, alpha=0.14, linewidth=0)
                ax.plot(xs, med, color=color, marker=marker, markersize=5, markeredgecolor=SURFACE,
                        markeredgewidth=0.8, zorder=3)
            for i, v in enumerate(vals):
                if len(v):
                    ax.scatter(xs[i] + _jitter(len(v), dodge * 0.35 or step * 0.08, i + 7 * j), v, s=9,
                               color=color, marker=marker, alpha=0.55, linewidths=0.4, edgecolors=SURFACE,
                               zorder=2)
        if m.column == "signed_bias_mm":
            ax.axhline(0, color=INK_2, linewidth=0.7, linestyle="--", zorder=1)
        ax.set_xticks([pos[v] for v in levs])
        _tick_labels(ax, p, levs)
        ax.set_xlim(min(pos.values()) - step * 0.5, max(pos.values()) + step * 0.5)
        ax.set_ylabel(m.unit)
        ax.set_title(m.label, loc="left", color=INK)
    for ax in axes.flat[len(metrics):]:
        ax.axis("off")
    for ax in axes[-1]:
        ax.set_xlabel(p.label)

    n_per = df.groupby(["object_label", p.column]).size()
    handles = [Line2D([], [], color=ostyle[o][0], marker=ostyle[o][1], markersize=5,
                      linewidth=2 if p.kind == "numeric" else 0, label=f"{o} (n = {n_per[o].min()}"
                      + (f"–{n_per[o].max()}" if n_per[o].max() != n_per[o].min() else "") + " per level)")
               for o in objs]
    summary = ("lines: median, band: interquartile range" if p.kind == "numeric"
               else "boxes: median and interquartile range, whiskers 1.5 IQR")
    fig.legend(handles=handles, loc="upper left", ncol=len(objs), bbox_to_anchor=(0.0, 1.0))
    fig.tight_layout(rect=(0, 0.03, 1, 0.95))
    fig.text(0.0, 0.0, f"{summary}; points: individual scans. Completeness: fraction of the whole visible "
             "reference within 2 mm of the scan.", ha="left", va="bottom", color=INK_2, fontsize=7)
    return _save(fig, out_dir, f"param_{_slug(p.column.removeprefix('info.'))}")


# --- 2. object comparison ----------------------------------------------------------------
def plot_object_comparison(df, params, out_dir, metric):
    objs = _objects(df)
    if len(objs) < 2:
        return None
    a, b = objs[:2]
    df = df.assign(_cfg=config_key(df, params))
    colour_by = next((p for p in params if p.column == "preset"), params[0])
    lstyle = _level_styles(df, colour_by)
    metric_list = [metric] + [m for m in ("fscore@2mm",) if m != metric and m in df]

    fig, axes = plt.subplots(1, len(metric_list) + 1, figsize=(FULL_W, 2.7))
    for ax, col in zip(axes, metric_list):
        m = METRICS[col]
        wide = df.assign(_v=m.values(df)).pivot_table(index="_cfg", columns="object_label", values="_v")
        wide = wide.dropna(subset=[a, b])
        if wide.empty:
            plt.close(fig)
            print(f"[plots] object comparison skipped: no configuration measured on both {a} and {b}")
            return None
        first = df.groupby("_cfg").first()
        lo = np.nanmin(wide[[a, b]].to_numpy())
        hi = np.nanmax(wide[[a, b]].to_numpy())
        pad = (hi - lo) * 0.06
        ax.plot([lo - pad, hi + pad], [lo - pad, hi + pad], color=INK_2, linewidth=0.7, linestyle="--", zorder=1)
        for cfg, r in wide.iterrows():
            lev = first.loc[cfg, colour_by.column]
            color, marker = lstyle[lev]
            ax.scatter(r[a], r[b], s=22, color=color, marker=marker, edgecolors=SURFACE, linewidths=0.6, zorder=3)
        rho = wide[a].corr(wide[b], method="spearman")
        ax.set_xlim(lo - pad, hi + pad)
        ax.set_ylim(lo - pad, hi + pad)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, axis="both")
        ax.set_xlabel(f"{a}: {m.axis_label}")
        ax.set_ylabel(f"{b}: {m.axis_label}")
        ax.set_title(m.label, loc="left")
        # Note in the empty half: points mostly above y = x leave the lower right free.
        above = (wide[b] > wide[a]).mean() > 0.5
        ax.text(*((0.96, 0.04) if above else (0.04, 0.96)), f"Spearman ρ = {rho:.2f}\nn = {len(wide)} configurations",
                transform=ax.transAxes, ha="right" if above else "left", va="bottom" if above else "top",
                fontsize=7, color=INK_2)

    # rank agreement on the main metric
    ax = axes[-1]
    m = METRICS[metric]
    wide = df.assign(_v=m.values(df)).pivot_table(index="_cfg", columns="object_label", values="_v").dropna(subset=[a, b])
    asc = m.lower_is_better
    ra = (wide[a].abs() if metric == "signed_bias_mm" else wide[a]).rank(ascending=asc)
    rb = (wide[b].abs() if metric == "signed_bias_mm" else wide[b]).rank(ascending=asc)
    first = df.groupby("_cfg").first()
    n = len(wide)
    ax.plot([1, n], [1, n], color=INK_2, linewidth=0.7, linestyle="--", zorder=1)
    for cfg in wide.index:
        color, marker = lstyle[first.loc[cfg, colour_by.column]]
        ax.scatter(ra[cfg], rb[cfg], s=22, color=color, marker=marker, edgecolors=SURFACE, linewidths=0.6, zorder=3)
    top = max(1, n // 5)
    both = ((ra <= top) & (rb <= top)).sum()
    ax.set_xlim(n + 1, 0)
    ax.set_ylim(n + 1, 0)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, axis="both")
    ax.set_xlabel(f"rank on {a} (1 = best)")
    ax.set_ylabel(f"rank on {b} (1 = best)")
    ax.set_title(f"{m.label}, rank", loc="left")
    ax.text(0.96, 0.04, f"{both} of each top {top} shared", transform=ax.transAxes, ha="right", va="bottom",
            fontsize=7, color=INK_2)

    handles = [Line2D([], [], color=c, marker=mk, linewidth=0, markersize=5, label=level_label(colour_by, v))
               for v, (c, mk) in lstyle.items()]
    fig.legend(handles=handles, loc="upper left", ncol=len(handles), bbox_to_anchor=(0.0, 1.03),
               title=colour_by.label, title_fontsize=7.5, alignment="left")
    fig.tight_layout(rect=(0, 0, 1, 0.88))
    return _save(fig, out_dir, "object_comparison")


# --- 3. interaction heatmaps -------------------------------------------------------------
def plot_heatmap(df, pa, pb, out_dir, metric):
    objs = _objects(df)
    m = METRICS[metric]
    la, lb = levels(df, pa), levels(df, pb)
    grids, counts = {}, {}
    for o in objs:
        sub = df[df["object_label"] == o].assign(_v=lambda d: m.values(d))
        g = sub.groupby([pa.column, pb.column])["_v"]
        grids[o] = g.median().unstack().reindex(index=la, columns=lb)
        counts[o] = g.size().unstack().reindex(index=la, columns=lb)
    cmap = SEQ_CMAP.copy()
    cmap.set_bad(MISSING)

    w = min(FULL_W, 0.6 + len(objs) * (0.62 * len(lb) + 1.6))
    fig, axes = plt.subplots(1, len(objs), figsize=(w, 0.5 * len(la) + 1.9), squeeze=False)
    for ax, o in zip(axes[0], objs):
        g = grids[o].to_numpy(dtype=float)
        # Own colour scale per object: the objects differ in baseline error, and
        # the point is the pattern within each one. Values are printed in the cells.
        vmin, vmax = np.nanmin(g), np.nanmax(g)
        im = ax.imshow(np.ma.masked_invalid(g), cmap=cmap, vmin=vmin, vmax=vmax, aspect="auto")
        ax.grid(False)
        for i in range(len(la)):
            for k in range(len(lb)):
                v = g[i, k]
                if np.isnan(v):
                    ax.text(k, i, "not\nmeasured", ha="center", va="center", fontsize=6, color=INK_2)
                    continue
                frac = (v - vmin) / (vmax - vmin) if vmax > vmin else 0
                ax.text(k, i, f"{v:.2f}", ha="center", va="center", fontsize=7,
                        color=SURFACE if frac > 0.55 else INK)
        ax.set_xticks(range(len(lb)))
        _tick_labels(ax, pb, lb)
        ax.set_yticks(range(len(la)))
        ax.set_yticklabels([level_label(pa, v, short=True) for v in la] if ax is axes[0][0] else [])
        ax.set_xlabel(pb.label)
        ax.set_title(o, loc="left")
        for s_ in ax.spines.values():
            s_.set_visible(False)
        ax.tick_params(length=0)
        cb = fig.colorbar(im, ax=ax, shrink=0.9, pad=0.03, fraction=0.06)
        cb.outline.set_visible(False)
        cb.ax.tick_params(labelsize=6.5)
    cb.set_label(m.axis_label + (", lower is better" if m.lower_is_better else ", higher is better"))
    axes[0][0].set_ylabel(pa.label)
    nmax = int(max(c.max().max() for c in counts.values()))
    nmin = int(min(c.min().min() for c in counts.values()))
    others = [q.label.split(" (")[0].lower() for q in varied_parameters(df) if q not in (pa, pb)]
    note = f"median of {nmin}" + (f"–{nmax}" if nmax != nmin else "") + " scans per cell"
    if others:
        note += f" (pooled over {', '.join(others)})"
    fig.suptitle(f"{m.label}: {pa.label.split(' (')[0]} × {pb.label.split(' (')[0]}   ·   {note}",
                 x=0.0, y=1.04, ha="left", fontsize=8, color=INK_2)
    return _save(fig, out_dir, f"heatmap_{_slug(pa.column)}_x_{_slug(pb.column)}")


# --- 4. completeness vs error trade-off --------------------------------------------------
def plot_tradeoff(df, params, out_dir, metric, x_metric="completeness_full@2mm"):
    if x_metric not in df:
        x_metric = "covered_fraction"
    objs = _objects(df)
    mx, my = METRICS[x_metric], METRICS[metric]
    names = []
    for p in params:
        lstyle = _level_styles(df, p)
        fig, axes = plt.subplots(1, len(objs), figsize=(FULL_W, 2.9), squeeze=False)
        for ax, o in zip(axes[0], objs):
            sub = df[df["object_label"] == o]
            for v, (color, marker) in lstyle.items():
                s = sub[sub[p.column] == v]
                ax.scatter(mx.values(s), my.values(s), s=22, color=color, marker=marker,
                           edgecolors=SURFACE, linewidths=0.6, zorder=3, label=level_label(p, v))
            ax.grid(True, axis="both")
            ax.set_title(o, loc="left")
            ax.set_xlabel(mx.axis_label + (" →" if not mx.lower_is_better else ""))
        axes[0][0].set_ylabel(my.axis_label)
        handles = [Line2D([], [], color=c, marker=mk, linewidth=0, markersize=5, label=level_label(p, v))
                   for v, (c, mk) in lstyle.items()]
        fig.legend(handles=handles, loc="upper left", ncol=min(len(handles), 6), bbox_to_anchor=(0.0, 1.04),
                   title=p.label, title_fontsize=7.5, alignment="left")
        fig.text(1.0, 1.0, "better: right (more complete), down (lower error)", ha="right", va="top",
                 fontsize=7, color=INK_2)
        fig.tight_layout(rect=(0, 0, 1, 0.84))
        names.append(_save(fig, out_dir, f"tradeoff_{_slug(p.column.removeprefix('info.'))}"))
    return names


# --- 5. diagnostics ------------------------------------------------------------------------
def plot_diagnostics(df_all, out_dir):
    df = df_all[df_all["status"].isin(["ok", "suspect"])].copy()
    if df.empty:
        return None
    objs = _objects(df)
    ostyle = _object_style(objs)
    fig, axes = plt.subplots(1, 3, figsize=(FULL_W, 2.6))

    ax = axes[0]
    for o in objs:
        s = df[df["object_label"] == o]
        ax.scatter(s["acc_rmse_mm"], s["acc_rmse_trim5_mm"], s=16, color=ostyle[o][0], marker=ostyle[o][1],
                   edgecolors=SURFACE, linewidths=0.5, label=o, zorder=3)
    lo, hi = df[["acc_rmse_mm", "acc_rmse_trim5_mm"]].min().min(), df["acc_rmse_mm"].max()
    ax.plot([lo, hi], [lo, hi], color=INK_2, linestyle="--", linewidth=0.7)
    ratio = (df["acc_rmse_trim5_mm"] / df["acc_rmse_mm"])
    ax.set_title("Outlier influence", loc="left")
    ax.text(0.96, 0.04, f"trimmed / full RMSE\n{ratio.min():.2f}–{ratio.max():.2f}", transform=ax.transAxes,
            ha="right", va="bottom", fontsize=7, color=INK_2)
    ax.set_xlabel("RMSE (mm)")
    ax.set_ylabel("RMSE, worst 5% removed (mm)")
    ax.grid(True, axis="both")

    ax = axes[1]
    for o in objs:
        s = df[df["object_label"] == o]
        sus = s["status"] == "suspect"
        ax.scatter(s.loc[~sus, "covered_fraction"] * 100, s.loc[~sus, "acc_rmse_mm"], s=16,
                   color=ostyle[o][0], marker=ostyle[o][1], edgecolors=SURFACE, linewidths=0.5, zorder=3)
        ax.scatter(s.loc[sus, "covered_fraction"] * 100, s.loc[sus, "acc_rmse_mm"], s=30, facecolors="none",
                   edgecolors=ostyle[o][0], marker=ostyle[o][1], linewidths=1.0, zorder=3)
    ax.axvline(C.MIN_COVERED_FRACTION * 100, color=INK_2, linestyle="--", linewidth=0.7)
    ax.text(C.MIN_COVERED_FRACTION * 100, ax.get_ylim()[1], "suspect ← ", ha="right", va="top", fontsize=6.5, color=INK_2)
    ax.set_title("Registration", loc="left")
    ax.text(0.04, 0.04, f"{(df['status'] == 'suspect').sum()} suspect (hollow)", transform=ax.transAxes,
            ha="left", va="bottom", fontsize=7, color=INK_2)
    ax.set_xlabel("Reference covered by scan (%)")
    ax.set_ylabel("RMSE (mm)")
    ax.grid(True, axis="both")

    ax = axes[2]
    for i, o in enumerate(objs):
        v = df.loc[df["object_label"] == o, "acc_rmse_mm"].to_numpy()
        ax.boxplot([v], positions=[i], widths=0.45, showfliers=False, patch_artist=True, manage_ticks=False,
                   medianprops=dict(color=ostyle[o][0], linewidth=1.6),
                   boxprops=dict(facecolor=ostyle[o][0] + "26", edgecolor=ostyle[o][0], linewidth=0.8),
                   whiskerprops=dict(color=ostyle[o][0], linewidth=0.8), capprops=dict(color=ostyle[o][0], linewidth=0.8))
        ax.scatter(i + _jitter(len(v), 0.12, i), v, s=10, color=ostyle[o][0], marker=ostyle[o][1], alpha=0.6,
                   edgecolors=SURFACE, linewidths=0.4)
        q1, q3 = np.percentile(v, [25, 75])
        far = df[(df["object_label"] == o) & (df["acc_rmse_mm"] > q3 + 1.5 * (q3 - q1))]
        for _, r in far.iterrows():
            ax.annotate(r["session"][-6:], (i, r["acc_rmse_mm"]), xytext=(6, 0), textcoords="offset points",
                        fontsize=6, color=INK_2, va="center")
    ax.set_xticks(range(len(objs)))
    ax.set_xticklabels(objs)
    ax.set_xlim(-0.6, len(objs) - 0.4)
    ax.set_ylabel("RMSE (mm)")
    ax.set_title("Spread", loc="left")
    handles = [Line2D([], [], color=ostyle[o][0], marker=ostyle[o][1], linewidth=0, markersize=5, label=o) for o in objs]
    fig.legend(handles=handles, loc="upper left", ncol=len(objs), bbox_to_anchor=(0.0, 1.04))
    fig.tight_layout(rect=(0, 0, 1, 0.9))
    fig.text(1.0, 0.0, "Spread: points beyond 1.5 IQR are labelled with their session time",
             ha="right", va="top", fontsize=6.5, color=INK_2)
    return _save(fig, out_dir, "diagnostics")


def make_all(df_all, out_dir: Path, metric: str, include_suspect=False):
    _style()
    df = usable(df_all, include_suspect)
    if df.empty:
        raise SystemExit("no usable scans to plot")
    if metric not in METRICS:
        METRICS[metric] = Metric(metric, metric, "", True)
    params = varied_parameters(df)
    print(f"\n[plots] {len(df)} scans ({'ok + suspect' if include_suspect else 'status ok'}); "
          f"varied parameters: {', '.join(p.column for p in params) or 'none'}")
    made = [plot_parameter(df, p, out_dir) for p in params]
    made.append(plot_object_comparison(df, params, out_dir, metric))
    pairs = crossed_pairs(df, params)
    skipped = [f"{a.column} x {b.column}" for a, b in combinations(params, 2) if (a, b) not in pairs]
    made += [plot_heatmap(df, a, b, out_dir, metric) for a, b in pairs]
    made += plot_tradeoff(df, params, out_dir, metric)
    made.append(plot_diagnostics(df_all, out_dir))
    made = [m for m in made if m]
    if skipped:
        print(f"[plots] no heatmap for pairs never varied together: {', '.join(skipped)}")
    print(f"[plots] {len(made)} figures (PNG + PDF) -> {out_dir}")
