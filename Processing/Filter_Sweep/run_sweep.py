#!/usr/bin/env python3
"""RealSense post-processing filter sweep over pre-recorded static scans.

For every scan and filter configuration: replay the recording (decoded once
into cache/), apply the filter chain, pick the best frame as rs_marker does,
crop it with ../Crop, evaluate it with ../Evaluate, and save one results row.

  run_sweep.py --smoke              one scan, a handful of configs (sweep_config.yaml: smoke)
  run_sweep.py --stage 1            one-factor-at-a-time sweeps
  run_sweep.py --stage 2            spatial x temporal grid
  run_sweep.py --stage 3            refinement of the best stage-2 results
  run_sweep.py --stage all          1, 2, 3 in turn
  run_sweep.py --stage 2 --dry-run  print the job count and runtime estimate only

Finished jobs are skipped, so the same command resumes an interrupted run.
Plots: plot_results.py (reads results/results.parquet only).
"""

import argparse
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from filter_sweep.memory import ensure_zeroed_malloc  # noqa: E402

ensure_zeroed_malloc()   # see filter_sweep/memory.py: re-execs once with MALLOC_PERTURB_=255
from filter_sweep.config import load_settings  # noqa: E402  (yaml only, no numpy yet)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", help="sweep config (default sweep_config.yaml next to this script)")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--stage", choices=["1", "2", "3", "all"])
    g.add_argument("--smoke", action="store_true", help="quick check on the 'smoke' subset")
    p.add_argument("--workers", type=int, help="parallel worker processes (default: config)")
    p.add_argument("--dry-run", action="store_true", help="print job counts and runtime estimate, run nothing")
    p.add_argument("--retry-failed", action="store_true", help="re-run jobs that failed last time")
    p.add_argument("--keep", action="store_true",
                   help="keep each job's folder (filtered.ply, trimmed.ply, Evaluate output) under results/jobs/")
    p.add_argument("--limit-configs", type=int, help="only the first N configs of the stage (testing)")
    p.add_argument("--configs", help="only these configs of the stage: comma-separated config_ids or exact "
                                     "config labels, e.g. 'baseline (no filters),sp m2 a0.5 d20 h5'")
    p.add_argument("--shapes", help="override: comma-separated shape folders")
    p.add_argument("--heights", help="override: comma-separated height folders, e.g. 641mm,863mm")
    p.add_argument("--scans", help="override: comma-separated scan ids (A1..A4, B)")
    p.add_argument("--out", help="override: results folder")
    args = p.parse_args()

    settings = load_settings(args.config)
    if args.shapes:
        settings.shapes = args.shapes.split(",")
    if args.heights:
        settings.heights = args.heights.split(",")
    if args.scans:
        settings.scans = args.scans.split(",")
    if args.out:
        settings.out_dir = Path(args.out).resolve()
    workers = args.workers or settings.workers
    # Before numpy / Open3D load: one OpenMP thread per worker unless told otherwise.
    threads = settings.threads_per_worker if workers > 1 else max(1, os.cpu_count() // 2)
    for var in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
        os.environ.setdefault(var, str(threads))
    settings.out_dir.mkdir(parents=True, exist_ok=True)

    from filter_sweep.runner import run_stage
    kw = dict(workers=workers, dry_run=args.dry_run, retry_failed=args.retry_failed, keep=args.keep,
              limit_configs=args.limit_configs,
              only_configs=[c.strip() for c in args.configs.split(",")] if args.configs else None)
    if args.smoke:
        run_stage("smoke", settings, smoke=True, **kw)
    else:
        for stage in (["1", "2", "3"] if args.stage == "all" else [args.stage]):
            run_stage(stage, settings, **kw)


if __name__ == "__main__":
    main()
