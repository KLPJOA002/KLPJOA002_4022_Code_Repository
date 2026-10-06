"""Run a stage: discover scans, build configs, skip finished jobs, decode
caches, then (scan, config) jobs in a process pool. Every job writes one
JSON row (results/rows/<config_id>/<scan_key>.json) whether it succeeded or
not, so a run can be stopped and resumed at any point."""

from __future__ import annotations

import contextlib
import json
import os
import shutil
import sys
import time
import traceback
from pathlib import Path

import numpy as np
import pandas as pd

from .config import PIPELINE_VERSION, config_id, config_label, flat_columns, load_settings
from .discover import discover
from .results import assemble, load_rows, row_path

# --- worker side -------------------------------------------------------------------

_W = {}   # per-process state: settings, open caches / software sensors, references, layouts


def _init_worker(settings_path, threads):
    for var in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
        os.environ[var] = str(threads)
    _W["settings"] = load_settings(settings_path)
    from . import evaluate  # noqa: F401  puts ../Crop on sys.path, so job tag layouts unpickle


def _json_default(o):
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.floating):
        return float(o)
    if isinstance(o, np.bool_):
        return bool(o)
    if isinstance(o, (np.ndarray, Path)):
        return o.tolist() if isinstance(o, np.ndarray) else str(o)
    raise TypeError(type(o))


def _write_row(path: Path, row: dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(f".{os.getpid()}.tmp")
    tmp.write_text(json.dumps(row, default=_json_default, indent=0))
    os.replace(tmp, path)


def _scan_state(cache_root, scan_key):
    """Open cache + software sensor for a scan, keeping the most recent one
    (jobs arrive grouped by scan)."""
    from .filters import SoftwareDepth
    from .frames import FrameCache
    cur = _W.get("scan")
    if cur and cur[0] == scan_key:
        return cur[1], cur[2]
    if cur:
        cur[1].close()
        cur[2].close()
    cache = FrameCache(cache_root, scan_key)
    sw = SoftwareDepth(cache.meta["stream"])
    _W["scan"] = (scan_key, cache, sw)
    return cache, sw


def run_job(job: dict) -> dict:
    from .evaluate import StageError, crop_seed_trim_evaluate, reference, stage_scan_folder
    from .filters import run_config
    from .frames import rs_frame_for

    s = _W["settings"]
    cfg, scan = job["config"], job["scan"]
    out_dir = Path(job["out_dir"])
    job_dir = out_dir / "jobs" / job["config_id"] / scan["scan_key"]
    started = time.time()
    row = {**scan, **flat_columns(cfg), "config_id": job["config_id"], "config_label": config_label(cfg),
           "config_json": json.dumps(cfg, sort_keys=True), "stage_first": job["stage"],
           "pipeline_version": PIPELINE_VERSION, "status": "failed", "failure_stage": None,
           "failure_reason": None}
    stage = "load"
    log_path = job_dir / "job.log"
    try:
        shutil.rmtree(job_dir, ignore_errors=True)
        job_dir.mkdir(parents=True)
        cache, sw = _scan_state(job["cache_dir"], scan["scan_key"])
        m = cache.meta
        row.update({"decoder": m["stream"].get("decoder"), "frame_map_method": m["map"]["anchor_method"],
                    "rs_best_frame_original": m["map"]["rs_best_frame"],
                    "rs_best_range": "{}:{}".format(*m["map"]["rs_best_range"]),
                    "recording_frames": len(m["frames"]), "hdr_available": m["hdr_available"]})
        if cfg["hdr_merge"] and not m["hdr_available"]:
            row.update(status="skipped", failure_stage="hdr_merge", failure_reason=m["hdr_note"])
            return _finish(row, job, job_dir, started)

        with open(log_path, "w") as log, contextlib.redirect_stdout(log), contextlib.redirect_stderr(log):
            stage = "filter"
            res = run_config(cache, cfg, sw, s.deprojection)
            points = res.pop("points")
            res["selected_rs_frame"] = rs_frame_for(m, res["best_bag_index"])
            row.update(res)
            if job.get("keep"):
                import open3d as o3d
                from .evaluate import crop_tool
                crop_tool.write_ply_like_original(
                    job_dir / "filtered.ply", o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points)))

            stage = "crop"
            ply_path = stage_scan_folder(job_dir / "scan", Path(scan["scan_dir"]), res["selected_rs_frame"])
            ref = reference(Path(job["reference"]), s.evaluate["reference_units"])
            result = crop_seed_trim_evaluate(points, ply_path, job["layout"], ref, s, job_dir,
                                             f"{job['config_id']}__{scan['scan_key']}", row)
        row.update({k: v for k, v in result.items()
                    if k not in ("label", "scan", "run_dir", "timestamp", "seed", "units", "scale")})
        try:
            doc = json.loads((job_dir / "eval" / "metrics.json").read_text())
            chosen = (doc.get("alignment") or {}).get("chosen") or {}
            row["align_chosen"] = chosen.get("name")
            row["align_overlap_ref"] = chosen.get("overlap_ref")
        except (OSError, json.JSONDecodeError):
            pass
        row["status"] = "ok"
    except Exception as exc:  # recorded in the table, never fatal to the run
        st = getattr(exc, "stage", stage)
        msg = str(exc).splitlines()[0] if str(exc) else type(exc).__name__
        row.update(failure_stage=st, failure_reason=msg if msg.startswith(f"{st}:") else f"{st}: {msg}")
        row["traceback"] = traceback.format_exc(limit=4)[-2000:]
    return _finish(row, job, job_dir, started)


def _finish(row, job, job_dir, started):
    row["elapsed_s"] = round(time.time() - started, 2)
    _write_row(row_path(Path(job["out_dir"]), job["config_id"], row["scan_key"]), row)
    if not job.get("keep"):
        shutil.rmtree(job_dir, ignore_errors=True)
    return {k: row.get(k) for k in ("status", "failure_reason", "scan_key", "config_label", "elapsed_s",
                                    "acc_rmse_mm", "covered_fraction", "best_bag_index", "best_valid_pixels")}


# --- main side ---------------------------------------------------------------------

def _stage_dir(settings) -> Path:
    return settings.out_dir / "stages"


def stage_config_ids(settings) -> dict:
    ids = {}
    for p in sorted(_stage_dir(settings).glob("*.csv")):
        if not p.stem.endswith("_scans"):
            ids[p.stem] = set(pd.read_csv(p)["config_id"])
    return ids


def write_stage_list(settings, stage: str, configs: list, scans: list):
    d = _stage_dir(settings)
    d.mkdir(parents=True, exist_ok=True)
    rows = [{"config_id": config_id(c), "config_label": config_label(c), **flat_columns(c),
             "config_json": json.dumps(c, sort_keys=True)} for c in configs]
    pd.DataFrame(rows).to_csv(d / f"{stage}.csv", index=False)
    pd.DataFrame([s.meta() for s in scans]).to_csv(d / f"{stage}_scans.csv", index=False)


def run_stage(stage: str, settings, workers: int, dry_run=False, retry_failed=False, keep=False,
              smoke=False, limit_configs=None):
    from .stages import build_smoke, build_stage
    from .config import normalise

    if smoke:
        sm = settings.smoke
        scans, warns = discover(settings.data_root, sm.get("shapes", settings.shapes),
                                sm.get("heights", "all"), sm.get("scans", "all"))
    else:
        scans, warns = discover(settings.data_root, settings.shapes, settings.heights, settings.scans)
    for w in warns:
        print(f"  [!] {w}")
    if not scans:
        raise SystemExit("no scans found")

    baseline_id = config_id(normalise({}, settings.filter_defaults))
    results = assemble(settings.out_dir, settings.min_covered_fraction, baseline_id, settings.rank_metric) \
        if (settings.out_dir / "rows").is_dir() else pd.DataFrame()
    configs = build_smoke(settings) if smoke else build_stage(stage, settings, results, stage_config_ids(settings))
    if limit_configs:
        configs = configs[:limit_configs]
    write_stage_list(settings, stage, configs, scans)

    done = load_rows(settings.out_dir)
    finished = set()
    if not done.empty:
        keep_status = {"ok", "suspect", "skipped"} if retry_failed else {"ok", "suspect", "skipped", "failed"}
        ok = done[(done["pipeline_version"] == PIPELINE_VERSION) & done["status"].isin(keep_status)]
        finished = set(zip(ok["config_id"], ok["scan_key"]))

    jobs = [(s, c) for s in scans for c in configs if (config_id(c), s.key) not in finished]
    per_job = settings.estimate_s_per_job
    if not done.empty and (done["status"] == "ok").sum() >= 5:
        per_job = float(done.loc[done["status"] == "ok", "elapsed_s"].median())
    est = len(jobs) * per_job / max(workers, 1)
    print(f"\n=== Stage {stage}: {settings.stages.get(stage, {}).get('description', 'smoke test') if not smoke else 'smoke test'}")
    print(f"  {len(configs)} configuration(s) x {len(scans)} scan(s) = {len(configs) * len(scans)} jobs; "
          f"{len(configs) * len(scans) - len(jobs)} already done, {len(jobs)} to run")
    print(f"  estimated runtime: {est / 3600:.1f} h ({per_job:.1f} s/job on {workers} worker(s)"
          f"{', measured' if per_job != settings.estimate_s_per_job else ', configured estimate'})"
          f" + decoding of new scans (~10 s each)")
    if dry_run or not jobs:
        return

    from .frames import ensure_cache
    from .evaluate import tag_layout

    todo_scans = sorted({s.key: s for s, _ in jobs}.values(), key=lambda s: s.key)
    print(f"\n  preparing {len(todo_scans)} scan cache(s) in {settings.cache_dir}")
    layouts = {}
    for s in todo_scans:
        ensure_cache(s, settings.cache_dir, settings.decoder, settings.warmup_frames,
                     log=lambda m: print(m, flush=True))
        if s.height_dir not in layouts:
            layouts[s.height_dir] = tag_layout(s.height_dir)

    payload = [{"scan": s.meta(), "config": c, "config_id": config_id(c), "stage": stage,
                "out_dir": str(settings.out_dir), "cache_dir": str(settings.cache_dir),
                "reference": str(settings.references[s.shape]), "layout": layouts[s.height_dir], "keep": keep}
               for s, c in sorted(jobs, key=lambda j: (j[0].key, config_id(j[1])))]

    from tqdm import tqdm
    print(f"\n  running {len(payload)} job(s) on {workers} worker(s)")
    counts = {}
    bar = tqdm(total=len(payload), unit="job", dynamic_ncols=True)
    try:
        if workers <= 1:
            _init_worker(str(settings.path), settings.threads_per_worker)
            it = map(run_job, payload)
            pool = None
        else:
            import multiprocessing as mp
            pool = mp.get_context("spawn").Pool(workers, _init_worker,
                                                (str(settings.path), settings.threads_per_worker))
            it = pool.imap_unordered(run_job, payload, chunksize=1)
        for i, r in enumerate(it, 1):
            counts[r["status"]] = counts.get(r["status"], 0) + 1
            bar.update(1)
            bar.set_postfix(counts)
            if r["status"] == "failed":
                tqdm.write(f"  [failed] {r['scan_key']} / {r['config_label']}: {r['failure_reason']}")
            if i % 200 == 0:
                assemble(settings.out_dir, settings.min_covered_fraction, baseline_id, settings.rank_metric)
        if pool:
            pool.close()
            pool.join()
    except KeyboardInterrupt:
        if pool:
            pool.terminate()
        print("\n  interrupted - finished jobs are saved; re-run the same command to resume")
        raise SystemExit(1)
    finally:
        bar.close()
        assemble(settings.out_dir, settings.min_covered_fraction, baseline_id, settings.rank_metric)
    print(f"  done: {counts}")
