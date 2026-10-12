"""Plan and run (wave x height x combination) jobs; collect the results table.

Run directory layout:
  run_config.json            config snapshot, fuse binary, crop cache
  sessions.csv               the full-sweep session picked per (wave, height)
  jobs/<wave>/<height>/<combo>__<param tag>/
      row.json               the finished row (its presence = job done)
      fuse.log  fused.ply  staging/  eval/  seed_transform.txt  job.log
  results.csv                long format, one row per job (rebuilt from row.json files)

A job is cached by its folder: combination id + fusion parameters (single
scans don't fuse, so their key has no parameters). Failed jobs also write a
row (status=failed) and are retried only with --retry-failed.
"""

from __future__ import annotations

import concurrent.futures as cf
import contextlib
import hashlib
import io
import json
import multiprocessing as mp
import os
import shutil
import time
import traceback
from dataclasses import dataclass, field
from pathlib import Path

import pandas as pd

from .combos import Combo, all_combos
from .discover import Session, discover

ROW_FILE = "row.json"


# ─────────────────────────────────────────────────────────────────────────────
#  Jobs
# ─────────────────────────────────────────────────────────────────────────────

def param_tag(params: dict) -> str:
    """Readable voxel/cell part plus a short hash of every fusion flag."""
    h = hashlib.sha1(json.dumps(params, sort_keys=True, default=str).encode()).hexdigest()[:6]
    return f"sv{params.get('scan-voxel', 'def')}_fc{params.get('fusion-cell', 'def')}_{h}"


@dataclass
class Job:
    session: Session
    combo: Combo
    params: dict = field(default_factory=dict)   # fuse_scans flags (empty for single scans)
    group: str = "main"                           # main | pilot | voxel

    @property
    def tag(self) -> str:
        return self.combo.combo_id if self.combo.kind == "single" else f"{self.combo.combo_id}__{param_tag(self.params)}"

    def dir(self, run_dir: Path) -> Path:
        return run_dir / "jobs" / self.session.wave / self.session.height / self.tag

    def label(self) -> str:
        return f"{self.session.wave}/{self.session.height}/{self.tag}"


def plan_jobs(cfg, mode: str = "main", waves=None, heights=None, combos=None) -> list[Job]:
    """mode: main (the whole grid + singles), pilot, or voxel."""
    params = cfg.fuse_params()
    if mode == "pilot":
        p = cfg["pilot"]
        waves, heights = waves or p["waves"], heights or p["heights"]
        combos = combos or (list(p["combos"]) + [f"single_{s}" for s in p.get("singles", [])])
    sessions = discover(cfg, waves, heights)
    if mode == "voxel":
        v = cfg["voxel_sweep"]
        sessions = discover(cfg, waves or v["waves"], heights or v["heights"])
        jobs = []
        for s in sessions:
            by_id = {c.combo_id: c for c in all_combos(cfg, s)}
            for cid in combos or v["combos"]:
                if cid not in by_id or by_id[cid].kind != "fused":
                    raise SystemExit(f"Error: voxel_sweep combo '{cid}' is not a fused combination of the grid")
                for sv, fc in v["pairs"]:
                    jobs.append(Job(s, by_id[cid], cfg.fuse_params({"scan-voxel": sv, "fusion-cell": fc}), "voxel"))
        return jobs

    jobs = []
    for s in sessions:
        available = all_combos(cfg, s)
        if combos:
            by_id = {c.combo_id: c for c in available}
            unknown = [c for c in combos if c not in by_id]
            if unknown:
                raise SystemExit(f"Error: unknown combination id(s): {unknown}")
            available = [by_id[c] for c in combos]
        for c in available:
            jobs.append(Job(s, c, params if c.kind == "fused" else {}, mode))
    return jobs


def job_state(job: Job, run_dir: Path) -> str | None:
    """None (not run), or the status recorded in row.json."""
    try:
        return json.loads((job.dir(run_dir) / ROW_FILE).read_text()).get("status")
    except (OSError, json.JSONDecodeError):
        return None


# ─────────────────────────────────────────────────────────────────────────────
#  Executing one job (in a worker process)
# ─────────────────────────────────────────────────────────────────────────────

def _base_row(job: Job) -> dict:
    s, c = job.session, job.combo
    return {
        "wave": s.wave, "height": s.height, "height_mm": s.height_mm, "session": s.name,
        "combo_id": c.combo_id, "kind": c.kind, "family": c.family,
        "rot_family": c.rot_family, "tilt_family": c.tilt_family,
        "rotations": ";".join(map(str, c.rotations)), "tilts": ";".join(map(str, c.tilts)),
        "n_rotations": len(c.rotations), "n_tilts": len(c.tilts),
        "max_abs_tilt": max(abs(t) for t in c.tilts), "n_scans": c.n_scans,
        "group": job.group, "job_tag": job.tag,
        "scan_voxel": job.params.get("scan-voxel"), "fusion_cell": job.params.get("fusion-cell"),
        "fuse_params_json": json.dumps(job.params, sort_keys=True) if job.params else "",
    }


def execute_job(cfg, job: Job, run_dir: Path) -> dict:
    from . import crop as crop_mod
    from .evaluate import StageError, seed_trim_evaluate
    from .fuse import FuseError, run_fuse, stage_session

    d = job.dir(run_dir)
    if d.exists():
        shutil.rmtree(d)
    d.mkdir(parents=True)
    row = _base_row(job)
    row.update(status="failed", failure_stage="", failure_reason="")
    t0 = time.monotonic()
    log = io.StringIO()
    try:
        with contextlib.redirect_stdout(log), contextlib.redirect_stderr(log):
            crop_dir = crop_mod.session_crop_dir(cfg, job.session)
            status = crop_mod.crop_status(cfg, job.session)
            scans = job.combo.scans(job.session)
            usable = [sc for sc in scans if status.get(sc.key, {}).get("status") == "ok"]
            missing = [sc.key for sc in scans if sc not in usable]
            row["scans_missing"] = ";".join(missing)
            row["n_scans_used"] = len(usable)
            if not usable:
                raise StageError("crop", f"no usable cropped scans ({';'.join(missing)})")

            if job.combo.kind == "single":
                cloud = crop_mod.cropped_ply(crop_dir, job.session, usable[0])
                row["fuse_s"] = 0.0
            else:
                if len(usable) < 2:
                    raise StageError("fuse", f"only {len(usable)} usable scan(s); fuse_scans needs 2")
                staged = stage_session(d, crop_dir, job.session, usable)
                cloud = d / "fused.ply"
                try:
                    info = run_fuse(cfg.fuse_binary, staged, cloud, job.params, d / "fuse.log",
                                    cfg["fuse"].get("timeout_s"), cfg.get("threads_per_worker"))
                except FuseError as exc:
                    row.update(exc.info)
                    raise StageError("fuse", exc) from None
                row.update(info)

            t1 = time.monotonic()
            row.update(seed_trim_evaluate(cfg, cloud, job.session.wave, d, job.label()))
            row["eval_s"] = round(time.monotonic() - t1, 2)
            if job.combo.kind != "single" and not cfg.get("keep_fused_ply", True):
                cloud.unlink(missing_ok=True)
        min_cov = cfg.get("min_covered_fraction", 0.85)
        cov = row.get("covered_fraction")
        if cov is not None and cov < min_cov:
            row.update(status="suspect", failure_reason=f"covers only {cov:.0%} of the reference (< {min_cov:.0%})")
        else:
            row["status"] = "ok"
    except StageError as exc:
        row.update(failure_stage=exc.stage, failure_reason=str(exc))
    except Exception as exc:  # recorded in the table, never fatal to the batch
        row.update(failure_stage="internal", failure_reason=f"{type(exc).__name__}: {exc}")
        log.write(traceback.format_exc())
    row["total_s"] = round(time.monotonic() - t0, 2)
    staging = d / "staging"
    if staging.exists():
        shutil.rmtree(staging)   # only symlinks
    (d / "job.log").write_text(log.getvalue())
    (d / ROW_FILE).write_text(json.dumps(row, indent=1, default=str))
    return row


def _worker_init(threads: int):
    os.environ["OMP_NUM_THREADS"] = str(threads)


def _run_one(cfg_path, job, run_dir):
    from .config import load_config
    return execute_job(load_config(cfg_path), job, run_dir)


def _crop_one(cfg_path, session):
    from .config import load_config
    from .crop import crop_session
    t0 = time.monotonic()
    m = crop_session(load_config(cfg_path), session)
    failed = [r["scan"] for r in m["scans"] if r["status"] != "ok"]
    return session, time.monotonic() - t0, failed


# ─────────────────────────────────────────────────────────────────────────────
#  Batch
# ─────────────────────────────────────────────────────────────────────────────

def write_run_metadata(cfg, run_dir: Path, jobs: list[Job]):
    run_dir.mkdir(parents=True, exist_ok=True)
    b = cfg.fuse_binary
    meta = {"config_path": str(cfg.path), "config": cfg.raw, "fuse_binary": str(b),
            "fuse_binary_mtime": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(b.stat().st_mtime)),
            "fuse_binary_sha1": hashlib.sha1(b.read_bytes()).hexdigest(),
            "crop_cache": str(cfg.crop_cache), "started": time.strftime("%Y-%m-%d %H:%M:%S")}
    path = run_dir / "run_config.json"
    history = []
    if path.exists():
        try:
            old = json.loads(path.read_text())
            history = old.pop("previous", []) + [old]
        except json.JSONDecodeError:
            pass
    meta["previous"] = history[-10:]
    path.write_text(json.dumps(meta, indent=2, default=str))
    sessions = {(j.session.wave, j.session.height): j.session for j in jobs}
    rows = [{"wave": s.wave, "height": s.height, "session_used": s.name, "n_scans": len(s.scans),
             "rotations": ";".join(map(str, s.rotations)), "tilts": ";".join(map(str, s.tilts)),
             "ignored_sessions": ";".join(f"{k} ({v} scans)" for k, v in s.other_sessions.items()),
             "path": str(s.path)} for s in sessions.values()]
    pd.DataFrame(rows).to_csv(run_dir / "sessions.csv", index=False)


def run_batch(cfg, jobs: list[Job], run_dir: Path, workers: int, retry_failed=False, log=print) -> pd.DataFrame:
    todo = []
    for j in jobs:
        st = job_state(j, run_dir)
        if st is None or (retry_failed and st == "failed"):
            todo.append(j)
    log(f"{len(jobs)} jobs planned, {len(jobs) - len(todo)} already done, {len(todo)} to run "
        f"({workers} workers x {cfg.get('threads_per_worker')} threads) -> {run_dir}")
    write_run_metadata(cfg, run_dir, jobs)
    if not todo:
        return collect_results(run_dir)

    from .crop import is_cropped
    sessions = {}
    for j in todo:
        sessions.setdefault((j.session.wave, j.session.height), j.session)
    to_crop = [s for s in sessions.values() if not is_cropped(cfg, s)]
    ctx = mp.get_context("spawn")
    threads = int(cfg.get("threads_per_worker") or 1)
    with cf.ProcessPoolExecutor(max_workers=workers, mp_context=ctx, initializer=_worker_init,
                                initargs=(threads,)) as pool:
        if to_crop:
            log(f"Cropping {len(to_crop)} session(s) into {cfg.crop_cache} ...")
            futs = [pool.submit(_crop_one, cfg.path, s) for s in to_crop]
            for f in cf.as_completed(futs):
                s, dt, failed = f.result()
                log(f"  cropped {s.wave}/{s.height}/{s.name} in {dt:.0f} s"
                    + (f"; FAILED: {', '.join(failed)}" if failed else ""))

        # Biggest fusions first: better packing of the long jobs.
        todo.sort(key=lambda j: -j.combo.n_scans if j.combo.kind == "fused" else 0)
        t0 = time.monotonic()
        futs = {pool.submit(_run_one, cfg.path, j, run_dir): j for j in todo}
        done = 0
        for f in cf.as_completed(futs):
            j = futs[f]
            done += 1
            try:
                row = f.result()
                msg = (f"{row['status']:7s} fuse {row.get('fuse_s', 0):6.1f}s eval {row.get('eval_s', 0) or 0:5.1f}s "
                       f"rss {row.get('fuse_peak_rss_mb', 0) or 0:6.0f}MB "
                       f"rmse {row.get('acc_rmse_mm', float('nan')):.3f}mm")
                if row["status"] == "failed":
                    msg += f"  [{row['failure_stage']}] {row['failure_reason'][:120]}"
            except Exception as exc:   # worker died (e.g. out of memory)
                msg = f"CRASHED {type(exc).__name__}: {exc}"
                d = j.dir(run_dir)
                d.mkdir(parents=True, exist_ok=True)
                row = _base_row(j)
                row.update(status="failed", failure_stage="worker", failure_reason=msg)
                (d / ROW_FILE).write_text(json.dumps(row, indent=1, default=str))
            el = time.monotonic() - t0
            eta = el / done * (len(todo) - done)
            log(f"[{done}/{len(todo)}  {el / 60:.0f} min, eta {eta / 60:.0f} min] {j.label()}: {msg}")
    return collect_results(run_dir)


def collect_results(run_dir: Path) -> pd.DataFrame:
    rows = []
    for p in sorted((run_dir / "jobs").rglob(ROW_FILE)):
        try:
            rows.append(json.loads(p.read_text()))
        except json.JSONDecodeError:
            continue
    df = pd.DataFrame(rows)
    if not df.empty:
        df = df.sort_values(["group", "wave", "height_mm", "kind", "n_scans", "combo_id"]).reset_index(drop=True)
        df.to_csv(run_dir / "results.csv", index=False)
    return df
