"""Best / worst parameter configurations per object and overall."""

from __future__ import annotations

from pathlib import Path

import pandas as pd

from .analysis import METRICS, PANEL_METRICS, config_key, usable, varied_parameters

SHOW = 5


def _per_object(df, params, metric):
    cols = [p.column for p in params]
    extra = [m for m in PANEL_METRICS if m in df and m != metric]
    g = df.groupby(["object_label"] + cols, dropna=False)
    agg = g[[metric] + extra].mean().reset_index()
    agg["n_scans"] = g.size().values
    agg["sessions"] = g["session"].agg(",".join).values
    m = METRICS.get(metric)
    asc = m.lower_is_better if m else True
    if metric == "signed_bias_mm":       # best = closest to zero
        agg["_key"] = agg[metric].abs()
        asc = True
    else:
        agg["_key"] = agg[metric]
    agg["rank"] = agg.groupby("object_label")["_key"].rank(ascending=asc, method="min").astype(int)
    return agg.drop(columns="_key").sort_values(["object_label", "rank"])


def write_rankings(df, out_dir: Path, metric: str):
    out_dir.mkdir(parents=True, exist_ok=True)
    data = usable(df)
    params = varied_parameters(data)
    if metric not in data:
        raise SystemExit(f"metric {metric} not in the results")
    per = _per_object(data, params, metric)
    cols = [p.column for p in params]
    m = METRICS.get(metric)
    unit = "" if m is None else ("(fraction)" if m.percent else m.unit)

    better = "closest to 0" if metric == "signed_bias_mm" else ("lower" if m is None or m.lower_is_better else "higher")
    print(f"\n=== Ranking by {metric} ({better} is better), "
          f"{len(data)} usable scans; configuration = {' | '.join(cols)} ===")
    for obj, g in per.groupby("object_label"):
        path = out_dir / f"ranking_{obj.lower()}.csv"
        g.to_csv(path, index=False)
        _print_block(f"{obj}: best", g.head(SHOW), cols, metric, unit)
        _print_block(f"{obj}: worst", g.tail(SHOW), cols, metric, unit)

    # Overall: configurations measured on every object, by mean rank.
    wide = per.pivot_table(index=cols, columns="object_label", values=[metric, "rank"])
    wide.columns = [f"{a}_{b.lower()}" for a, b in wide.columns]
    wide = wide.dropna().reset_index()
    rank_cols = [c for c in wide.columns if c.startswith("rank_")]
    wide["mean_rank"] = wide[rank_cols].mean(axis=1)
    wide[f"mean_{metric}"] = wide[[c for c in wide.columns if c.startswith(f"{metric}_")]].mean(axis=1)
    wide = wide.sort_values(["mean_rank", f"mean_{metric}"]).reset_index(drop=True)
    wide.insert(0, "overall_rank", range(1, len(wide) + 1))
    wide.to_csv(out_dir / "ranking_overall.csv", index=False)
    if wide.empty:
        print("\n  Overall: no configuration was measured on every object")
        print(f"[write] rankings -> {out_dir}")
        return
    _print_block("Overall (mean rank across objects): best", wide.head(SHOW), cols, metric, unit, overall=True)
    _print_block("Overall (mean rank across objects): worst", wide.tail(SHOW), cols, metric, unit, overall=True)
    print(f"[write] rankings -> {out_dir}")


def _print_block(title, g, cols, metric, unit, overall=False):
    print(f"\n  {title}")
    for _, r in g.iterrows():
        cfg = "  ".join(f"{str(r[c]):>17s}" for c in cols)
        if overall:
            vals = "  ".join(f"{c.removeprefix(metric + '_')} {r[c]:.3f}" for c in g.columns
                             if c.startswith(f"{metric}_"))
            print(f"    #{int(r['overall_rank']):2d}  {cfg}   mean rank {r['mean_rank']:5.1f}   {vals} {unit}")
        else:
            print(f"    #{int(r['rank']):2d}  {cfg}   {r[metric]:.3f} {unit}  (n={int(r['n_scans'])})")
