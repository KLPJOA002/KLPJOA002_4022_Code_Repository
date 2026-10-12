#!/usr/bin/env python3
"""Rotation x tilt combination sweep: fuse subsets of each full-sweep session
with ../Fuse and score them with ../Evaluate. See README.md.

  run_sweep.py plan    [--from-run DIR]          job count, disk, time estimate
  run_sweep.py crop                               crop every full-sweep session into the cache
  run_sweep.py run     [--pilot] [--dry-run] [--out DIR] [--workers N] ...
  run_sweep.py voxel   [--dry-run] [--out DIR]    scan-voxel / fusion-cell sensitivity
  run_sweep.py analyze RUN_DIR                    plots, tables, summary.md
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import pandas as pd

from combo_sweep.config import DEFAULT_CONFIG, load_config
from combo_sweep.plan import summarize
from combo_sweep.runner import collect_results, plan_jobs, run_batch


def csv_list(s):
    return [x.strip() for x in s.split(",") if x.strip()] if s else None


def add_common(p):
    p.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    p.add_argument("--waves", type=csv_list, help="e.g. Big_Wave")
    p.add_argument("--heights", type=csv_list, help="e.g. 641mm,863mm")
    p.add_argument("--combos", type=csv_list, help="only these combination ids")
    p.add_argument("--workers", type=int)
    p.add_argument("--fuse-param", action="append", default=[], metavar="KEY=VALUE",
                   help="override/add a fuse_scans flag (repeatable), e.g. rotation-axis=z, "
                        "tag-align=false; changes the job cache key")


def measured_from(run_dir):
    if run_dir and (Path(run_dir) / "jobs").exists():
        return collect_results(Path(run_dir))
    return None


class Tee:
    """Print and append to <run_dir>/batch.log."""

    def __init__(self, path: Path):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.f = open(path, "a")

    def __call__(self, msg=""):
        line = f"{time.strftime('%H:%M:%S')} {msg}"
        print(line, flush=True)
        self.f.write(line + "\n")
        self.f.flush()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("plan", help="job count, disk and time estimate")
    add_common(p)
    p.add_argument("--mode", choices=["main", "pilot", "voxel"], default="main")
    p.add_argument("--from-run", type=Path, help="use timings measured in this run directory")
    p.add_argument("-v", "--verbose", action="store_true", help="list every job")

    p = sub.add_parser("crop", help="crop the full-sweep sessions into the shared cache")
    add_common(p)

    for name in ("run", "voxel"):
        p = sub.add_parser(name, help="main grid" if name == "run" else "voxel/cell sensitivity")
        add_common(p)
        if name == "run":
            p.add_argument("--pilot", action="store_true", help="the [pilot] subset only")
        p.add_argument("--out", type=Path, help="run directory (existing one = resume)")
        p.add_argument("--dry-run", action="store_true", help="print the planned jobs and exit")
        p.add_argument("--retry-failed", action="store_true")
        p.add_argument("--from-run", type=Path, help="timings for the estimate (default: --out)")

    p = sub.add_parser("analyze", help="plots, ranking tables and summary.md for a run directory")
    p.add_argument("run_dir", type=Path)
    p.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    p.add_argument("--within", type=float, help="percent of all-scans quality (default from config)")
    p.add_argument("--include-suspect", action="store_true")

    a = ap.parse_args()
    cfg = load_config(a.config)
    for kv in getattr(a, "fuse_param", []):
        key, _, value = kv.partition("=")
        value = {"true": True, "false": False}.get(value.lower(), value)
        cfg.raw["fuse"]["params"][key.lstrip("-")] = value

    if a.cmd == "analyze":
        from combo_sweep.analysis import analyze
        analyze(cfg, a.run_dir, within=a.within, include_suspect=a.include_suspect or None)
        return

    workers = a.workers or cfg["workers"]
    if a.cmd == "plan":
        jobs = plan_jobs(cfg, a.mode, a.waves, a.heights, a.combos)
        summarize(cfg, jobs, None, workers, measured_from(a.from_run), a.verbose)
        return

    if a.cmd == "crop":
        jobs = plan_jobs(cfg, "main", a.waves, a.heights, a.combos)
        import concurrent.futures as cf
        from combo_sweep.runner import _crop_one
        sessions = list({(j.session.wave, j.session.height): j.session for j in jobs}.values())
        with cf.ProcessPoolExecutor(max_workers=workers) as pool:
            for s, dt, failed in pool.map(_crop_one, [cfg.path] * len(sessions), sessions):
                print(f"{s.wave}/{s.height}/{s.name}: {dt:.0f} s" + (f"; FAILED {failed}" if failed else ""))
        return

    mode = "voxel" if a.cmd == "voxel" else ("pilot" if a.pilot else "main")
    jobs = plan_jobs(cfg, mode, a.waves, a.heights, a.combos)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    prefix = {"main": "", "pilot": "pilot_", "voxel": "voxel_"}[mode]
    run_dir = (a.out or cfg.experiments_root / f"{prefix}{stamp}").resolve()
    measured = measured_from(a.from_run or (run_dir if run_dir.exists() else None))
    summarize(cfg, jobs, run_dir if run_dir.exists() else None, workers, measured, verbose=a.dry_run)
    if a.dry_run:
        print(f"\n(dry run: nothing executed; would write to {run_dir})")
        return
    log = Tee(run_dir / "batch.log")
    log(f"=== {a.cmd} ({mode}) started; config {cfg.path}")
    df = run_batch(cfg, jobs, run_dir, workers, a.retry_failed, log)
    if not df.empty:
        counts = df["status"].value_counts().to_dict()
        log(f"Done: {counts}. Results: {run_dir / 'results.csv'}")
        log(f"Next: .venv/bin/python run_sweep.py analyze {run_dir}")


if __name__ == "__main__":
    sys.exit(main())
