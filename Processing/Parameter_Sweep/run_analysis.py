#!/usr/bin/env python3
"""Evaluate every scan of the hardware-parameter sweep against its reference
STL, then plot and rank the parameter settings.

    python run_analysis.py                       # evaluate new/changed scans, then plot
    python run_analysis.py --force               # re-evaluate everything
    python run_analysis.py --plots-only          # re-plot from results/all_scans.csv
    python run_analysis.py --sessions 20261002_165154,20261002_162132 --no-plots

Evaluation reuses ../Evaluate (alignment + metrics) and ../Crop (AprilTag
crop); see README.md. The data directory is only read, never written.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from sweep import config as C  # noqa: E402


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--data", type=Path, default=C.DEFAULT_DATA, help=f"sweep data (default {C.DEFAULT_DATA})")
    p.add_argument("--out", type=Path, default=C.DEFAULT_OUT, help=f"output directory (default {C.DEFAULT_OUT})")
    p.add_argument("--objects", help=f"comma-separated object folders (default {','.join(C.OBJECTS)})")
    p.add_argument("--sessions", help="only these session folders (comma-separated); written to "
                                      "subset_scans.csv instead of all_scans.csv")
    p.add_argument("--force", action="store_true", help="re-evaluate every scan, ignoring the cache")
    p.add_argument("--retry-failed", action="store_true", help="re-evaluate scans that failed last time")
    p.add_argument("--plots-only", action="store_true", help="skip evaluation; plot from all_scans.csv")
    p.add_argument("--no-plots", action="store_true", help="evaluate only")
    p.add_argument("--metric", default=C.PRIMARY_METRIC,
                   help=f"main metric for rankings and heatmaps (default {C.PRIMARY_METRIC})")
    p.add_argument("--include-suspect", action="store_true",
                   help="also plot scans whose registration check failed")
    p.add_argument("-v", "--verbose", action="store_true", help="show Evaluate's console output")
    args = p.parse_args(argv)

    out = args.out.resolve()
    data = args.data.resolve()
    if out == data or data in out.parents:
        p.error("--out must not be inside the (read-only) data directory")
    out.mkdir(parents=True, exist_ok=True)
    csv_path = out / "all_scans.csv"

    if not args.plots_only:
        from sweep.batch import Evaluator
        objects = args.objects.split(",") if args.objects else None
        unknown = set(objects or []) - set(C.OBJECTS)
        if unknown:
            p.error(f"unknown object(s) {sorted(unknown)}; known: {list(C.OBJECTS)}")
        sessions = set(args.sessions.split(",")) if args.sessions else None
        df = Evaluator(data, out).run(objects, sessions, args.force, args.retry_failed, args.verbose)
        target = out / "subset_scans.csv" if sessions else csv_path
        df.to_csv(target, index=False)
        print(f"[write] {target}  ({len(df)} rows)")
        report_status(df)
        if sessions:
            return 0

    if args.no_plots:
        return 0
    import pandas as pd
    from sweep import plots, ranking
    if not csv_path.is_file():
        p.error(f"{csv_path} not found - run the evaluation first")
    df = pd.read_csv(csv_path)
    ranking.write_rankings(df, out / "rankings", args.metric)
    plots.make_all(df, out / "plots", args.metric, include_suspect=args.include_suspect)
    return 0


def report_status(df):
    counts = df["status"].value_counts().to_dict()
    print("status: " + ", ".join(f"{k} {v}" for k, v in sorted(counts.items())))
    bad = df[df["status"] != "ok"]
    for _, r in bad.iterrows():
        print(f"  [{r['status']}] {r['object']}/{r['session']}: {r['failure_reason']}")


if __name__ == "__main__":
    sys.exit(main())
