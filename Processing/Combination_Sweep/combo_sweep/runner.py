"""Plan and run (wave x height x combination) jobs; collect the results table.

Run directory layout:
  run_config.json            config snapshot, fuse binary, crop cache
  sessions.csv               the full-sweep session picked per (wave, height)
  sessions/<wave>/<height>/<registration tag>/      (registration mode session_poses)
      staging/               every usable scan of the session (symlinks; kept for stage 2 and review)
      poses.json  qc.json    stage 1: every scan's registered pose and QC
      register.log  register_fused.ply  register.json (state: clean | needs_review | reviewed | timeout | failed)
  jobs/<wave>/<height>/<combo>__<param tag>[__sp]/
      row.json               the finished row (its presence = job done)
      fuse.log  fused.ply  combo_qc.json  staging/ (per_combo only)  eval/  seed_transform.txt  job.log
  results.csv                long format, one row per job (rebuilt from row.json files)
  review_queue.csv           sessions waiting for a manual look (run_sweep.py run --review)

Registration modes (README, "Registration modes"):
  session_poses (default)  stage 1 registers each session once on all its scans; stage 2 fuses each
                           combination from those poses. A combination's poses therefore come from
                           registering all 44 scans: the sweep measures what a set of viewpoints is worth
                           *given good registration*, not how well that subset registers on its own.
  per_combo                every combination registers its own scans from scratch (the original sweep,
                           --legacy-per-combo); kept as a secondary comparison. Job folders have no __sp.

A job is cached by its folder: combination id + fusion parameters (+ __sp). In session_poses mode a job
is also re-run when the session's poses.json has changed since (e.g. after a review). Failed jobs also
write a row (status=failed) and are retried only with --retry-failed.
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
REGISTER_FILE = "register.json"
REVIEW_QUEUE = "review_queue.csv"
SESSION_POSES, PER_COMBO = "session_poses", "per_combo"

# fuse_scans flags that only affect fusion, not registration: sessions registered with the same
# remaining flags share one poses.json whatever these are.
FUSION_ONLY_FLAGS = {"fusion-cell", "fusion-mode", "fusion-voxel", "fusion-min-scans", "fusion-trim",
                     "mls-radius", "no-mls", "no-fusion-weights", "no-fusion-sor"}


# ─────────────────────────────────────────────────────────────────────────────
#  Jobs
# ─────────────────────────────────────────────────────────────────────────────

def param_tag(params: dict) -> str:
    """Readable voxel/cell part plus a short hash of every fusion flag."""
    h = hashlib.sha1(json.dumps(params, sort_keys=True, default=str).encode()).hexdigest()[:6]
    return f"sv{params.get('scan-voxel', 'def')}_fc{params.get('fusion-cell', 'def')}_{h}"


def registration_params(params: dict) -> dict:
    return {k: v for k, v in params.items() if str(k).lstrip("-") not in FUSION_ONLY_FLAGS}


def registration_tag(params: dict) -> str:
    rp = registration_params(params)
    h = hashlib.sha1(json.dumps(rp, sort_keys=True, default=str).encode()).hexdigest()[:6]
    return f"sv{rp.get('scan-voxel', 'def')}_{h}"


@dataclass
class Job:
    session: Session
    combo: Combo
    params: dict = field(default_factory=dict)   # fuse_scans flags (empty for single scans)
    group: str = "main"                           # main | pilot | voxel
    mode: str = SESSION_POSES                     # session_poses | per_combo (fused jobs only)

    @property
    def tag(self) -> str:
        if self.combo.kind == "single":
            return self.combo.combo_id
        suffix = "__sp" if self.mode == SESSION_POSES else ""
        return f"{self.combo.combo_id}__{param_tag(self.params)}{suffix}"

    def dir(self, run_dir: Path) -> Path:
        return run_dir / "jobs" / self.session.wave / self.session.height / self.tag

    def reg_dir(self, run_dir: Path) -> Path:
        return session_reg_dir(run_dir, self.session, self.params)

    @property
    def registers_per_session(self) -> bool:
        return self.combo.kind == "fused" and self.mode == SESSION_POSES

    def label(self) -> str:
        return f"{self.session.wave}/{self.session.height}/{self.tag}"


def session_reg_dir(run_dir: Path, session: Session, params: dict) -> Path:
    return run_dir / "sessions" / session.wave / session.height / registration_tag(params)


def plan_jobs(cfg, mode: str = "main", waves=None, heights=None, combos=None,
              registration_mode: str | None = None) -> list[Job]:
    """mode: main (the whole grid + singles), pilot, or voxel."""
    reg_mode = registration_mode or (cfg.get("registration") or {}).get("mode", SESSION_POSES)
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
                    jobs.append(Job(s, by_id[cid], cfg.fuse_params({"scan-voxel": sv, "fusion-cell": fc}),
                                    "voxel", reg_mode))
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
            jobs.append(Job(s, c, params if c.kind == "fused" else {}, mode, reg_mode))
    return jobs


def read_row(job: Job, run_dir: Path) -> dict | None:
    try:
        return json.loads((job.dir(run_dir) / ROW_FILE).read_text())
    except (OSError, json.JSONDecodeError):
        return None


def job_state(job: Job, run_dir: Path) -> str | None:
    """None (not run), or the status recorded in row.json."""
    row = read_row(job, run_dir)
    return row.get("status") if row else None


def file_sha1(path: Path) -> str | None:
    try:
        return hashlib.sha1(Path(path).read_bytes()).hexdigest()
    except OSError:
        return None


def job_is_current(job: Job, run_dir: Path, retry_failed: bool) -> bool:
    """True if the job's row exists and nothing it depends on has changed since."""
    row = read_row(job, run_dir)
    if row is None or (retry_failed and row.get("status") == "failed"):
        return False
    if job.registers_per_session:
        # A row fused from different poses (re-registered, or reviewed since) is stale. A row that
        # failed because stage 1 had not produced poses is retried once they exist.
        poses_sha = file_sha1(job.reg_dir(run_dir) / "poses.json")
        if poses_sha is not None and row.get("poses_sha1") != poses_sha:
            return False
    return True


# ─────────────────────────────────────────────────────────────────────────────
#  Stage 1: registering a session once (session_poses mode)
# ─────────────────────────────────────────────────────────────────────────────

def summarize_poses(poses_path: Path) -> dict:
    """Session state from a poses.json: clean | needs_review | reviewed, and why."""
    doc = json.loads(Path(poses_path).read_text())
    sess = doc.get("session") or {}
    scans = doc.get("scans") or {}
    flagged = sorted(k for k, s in scans.items() if s.get("needs_review") and s.get("status") != "excluded")
    reasons = [f"{k}:{'+'.join(scans[k].get('qc_reasons') or [])}" for k in flagged]
    excluded = sorted(k for k, s in scans.items() if s.get("status") == "excluded")
    return {"state": (sess.get("review") or {}).get("state") or ("needs_review" if sess.get("needs_review") else "clean"),
            "flagged_scans": ";".join(flagged), "n_flagged": len(flagged), "reasons": ";".join(reasons),
            "session_flags": ";".join(sess.get("flags") or []), "excluded_scans": ";".join(excluded),
            "reviewed_at": (sess.get("review") or {}).get("reviewed_at"),
            "imu_frame": (sess.get("tilt_calibration") or {}).get("frame"),
            "poses_sha1": file_sha1(poses_path)}


def read_register_state(reg_dir: Path) -> dict | None:
    try:
        return json.loads((reg_dir / REGISTER_FILE).read_text())
    except (OSError, json.JSONDecodeError):
        return None


def write_register_state(reg_dir: Path, state: dict):
    (reg_dir / REGISTER_FILE).write_text(json.dumps(state, indent=1, default=str))


def register_one(cfg, session: Session, params: dict, run_dir: Path, force: bool = False) -> dict:
    """Stage 1 for one session. Never raises: a failure or timeout is a state, recorded in
    register.json and the review queue, and the rest of the batch carries on."""
    from . import crop as crop_mod
    from .fuse import FuseError, register_session, stage_session

    reg = session_reg_dir(run_dir, session, params)
    poses = reg / "poses.json"
    base = {"wave": session.wave, "height": session.height, "session": session.name, "reg_dir": str(reg)}
    if poses.is_file() and not force:
        state = {**(read_register_state(reg) or {}), **base, **summarize_poses(poses), "skipped": True}
        write_register_state(reg, state)
        return state

    if reg.exists():
        shutil.rmtree(reg)
    reg.mkdir(parents=True)
    rcfg = cfg.get("registration") or {}
    state = dict(base)
    t0 = time.monotonic()
    try:
        status = crop_mod.crop_status(cfg, session)
        usable = [sc for sc in session.scans if status.get(sc.key, {}).get("status") == "ok"]
        state["scans_missing"] = ";".join(sc.key for sc in session.scans if sc not in usable)
        state["n_scans"] = len(usable)
        if len(usable) < 2:
            raise FuseError(f"only {len(usable)} usable cropped scan(s)")
        staged = stage_session(reg, crop_mod.session_crop_dir(cfg, session), session, usable)
        info = register_session(cfg.fuse_binary, staged, reg, params, rcfg.get("timeout_s", 7200),
                                cfg.get("threads_per_worker"), rcfg.get("auto_retry", True),
                                rcfg.get("auto_retry_max_scans", 12))
        state.update({k: v for k, v in info.items()})
        state.update(summarize_poses(poses))
    except FuseError as exc:
        state.update(exc.info)
        timed_out = "timed out" in str(exc)
        state.update(state="timeout" if timed_out else "failed", failure_reason=str(exc))
        if timed_out and poses.is_file():
            # Killed during --auto-retry: fuse_scans checkpointed the finished registration first
            # (session flag auto_retry_incomplete), so the session is usable and only needs review.
            state.update(summarize_poses(poses))
            state["failure_reason"] = f"{exc} during auto-retry; poses are the pre-retry checkpoint"
    except Exception as exc:   # recorded, never fatal to the batch
        state.update(state="failed", failure_reason=f"{type(exc).__name__}: {exc}")
    state["register_s"] = round(time.monotonic() - t0, 1)
    write_register_state(reg, state)
    return state


def _register_worker(cfg_path, session, params, run_dir, force):
    from .config import load_config
    return register_one(load_config(cfg_path), session, params, run_dir, force)


def write_review_queue(run_dir: Path, binary: Path | None = None) -> pd.DataFrame:
    """review_queue.csv: every registered session that is not clean (needs_review, timeout or
    failed), from the register.json files. Rebuilt from scratch each time, so it is always current."""
    rows = []
    for p in sorted((run_dir / "sessions").rglob(REGISTER_FILE)) if (run_dir / "sessions").exists() else []:
        try:
            st = json.loads(p.read_text())
        except json.JSONDecodeError:
            continue
        if st.get("state") in ("needs_review", "timeout", "failed"):
            rows.append({k: st.get(k) for k in ("wave", "height", "session", "state", "n_flagged", "flagged_scans",
                                                 "reasons", "session_flags", "failure_reason", "register_s",
                                                 "reg_dir")})
    df = pd.DataFrame(rows, columns=["wave", "height", "session", "state", "n_flagged", "flagged_scans", "reasons",
                                     "session_flags", "failure_reason", "register_s", "reg_dir"])
    df.to_csv(run_dir / REVIEW_QUEUE, index=False)
    return df


def session_states(run_dir: Path) -> pd.DataFrame:
    rows = []
    if (run_dir / "sessions").exists():
        for p in sorted((run_dir / "sessions").rglob(REGISTER_FILE)):
            with contextlib.suppress(json.JSONDecodeError):
                rows.append(json.loads(p.read_text()))
    return pd.DataFrame(rows)


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
        "registration_mode": job.mode if c.kind == "fused" else "none",
        "scan_voxel": job.params.get("scan-voxel"), "fusion_cell": job.params.get("fusion-cell"),
        "fuse_params_json": json.dumps(job.params, sort_keys=True) if job.params else "",
    }


def execute_job(cfg, job: Job, run_dir: Path) -> dict:
    from . import crop as crop_mod
    from .evaluate import StageError, seed_trim_evaluate
    from .fuse import FuseError, fuse_from_poses, qc_columns, run_fuse, stage_session

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
            elif job.registers_per_session:
                reg = job.reg_dir(run_dir)
                poses = reg / "poses.json"
                row["register_dir"] = str(reg)
                if not poses.is_file():
                    st = read_register_state(reg) or {}
                    raise StageError("register", f"session not registered ({st.get('state', 'not run')}): "
                                                 f"{st.get('failure_reason', '')}")
                row["poses_sha1"] = file_sha1(poses)
                stored = (json.loads(poses.read_text()).get("scans") or {})
                # Scans the registration left out (failed to load, or excluded in a review) have no
                # pose; the combination is fused without them and says so.
                excluded = [sc for sc in usable if (stored.get(sc.key) or {}).get("status", "excluded") == "excluded"]
                row["scans_excluded"] = ";".join(sc.key for sc in excluded)
                row["n_manually_excluded_in_combo"] = sum(
                    (stored.get(sc.key) or {}).get("excluded_by") == "manual" for sc in excluded)
                usable = [sc for sc in usable if sc not in excluded]
                row["n_scans_used"] = len(usable)
                if len(usable) < 2:
                    raise StageError("fuse", f"only {len(usable)} scan(s) with a stored pose; fuse_scans needs 2")
                cloud = d / "fused.ply"
                try:
                    info = fuse_from_poses(cfg.fuse_binary, reg / "staging" / job.session.name, poses,
                                           [sc.key for sc in usable], cloud, d / "combo_qc.json", job.params,
                                           d / "fuse.log", cfg["fuse"].get("timeout_s"), cfg.get("threads_per_worker"))
                except FuseError as exc:
                    row.update(exc.info)
                    raise StageError("fuse", exc) from None
                row.update(info)
                row.update(qc_columns(d / "combo_qc.json"))
            else:
                if len(usable) < 2:
                    raise StageError("fuse", f"only {len(usable)} usable scan(s); fuse_scans needs 2")
                staged = stage_session(d, crop_dir, job.session, usable)
                cloud = d / "fused.ply"
                try:
                    # --qc: per-combo registration gets the same flags and spreads, so the two
                    # modes can be compared on them too.
                    info = run_fuse(cfg.fuse_binary, staged, cloud, job.params, d / "fuse.log",
                                    cfg["fuse"].get("timeout_s"), cfg.get("threads_per_worker"),
                                    ["--qc", d / "combo_qc.json"])
                except FuseError as exc:
                    row.update(exc.info)
                    raise StageError("fuse", exc) from None
                row.update(info)
                row.update(qc_columns(d / "combo_qc.json"))
                row.setdefault("n_manually_excluded_in_combo", 0)

            t1 = time.monotonic()
            row.update(seed_trim_evaluate(cfg, cloud, job.session.wave, d, job.label()))
            row["eval_s"] = round(time.monotonic() - t1, 2)
            if job.combo.kind != "single" and not cfg.get("keep_fused_ply", True):
                cloud.unlink(missing_ok=True)
        min_cov = cfg.get("min_covered_fraction", 0.85)
        cov = row.get("covered_fraction")
        if cov is not None and cov < min_cov:
            row.update(status="suspect", failure_reason=f"covers only {cov:.0%} of the reference (< {min_cov:.0%})")
        elif (row.get("n_flagged_scans_in_combo") or 0) > 0:
            # Evaluated and kept like any other row - only labelled, so analysis can choose.
            row.update(status="needs_review",
                       failure_reason=f"includes flagged scan(s) {row.get('flagged_scans')} ({row.get('flag_reasons')})")
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
            "registration_modes": sorted({j.mode for j in jobs if j.combo.kind == "fused"}),
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


def _job_message(row: dict) -> str:
    msg = (f"{row['status']:12s} fuse {row.get('fuse_s', 0) or 0:6.1f}s eval {row.get('eval_s', 0) or 0:5.1f}s "
           f"rss {row.get('fuse_peak_rss_mb', 0) or 0:6.0f}MB "
           f"rmse {row.get('acc_rmse_mm', float('nan')):.3f}mm")
    if row["status"] == "failed":
        msg += f"  [{row['failure_stage']}] {row['failure_reason'][:120]}"
    elif row["status"] == "needs_review":
        msg += f"  flagged: {row.get('flagged_scans')}"
    return msg


def run_batch(cfg, jobs: list[Job], run_dir: Path, workers: int, retry_failed=False, log=print,
              force_register=False) -> pd.DataFrame:
    """Runs every job not already done. In session_poses mode, each session is registered once
    (stage 1) and its combinations are submitted (stage 2) as soon as its poses exist."""
    todo = [j for j in jobs if not job_is_current(j, run_dir, retry_failed)]

    # Stage-1 work: one registration per (session, registration parameters).
    reg_groups: dict[Path, list[Job]] = {}
    for j in jobs:
        if j.registers_per_session:
            reg_groups.setdefault(j.reg_dir(run_dir), []).append(j)
    to_register = {r: g for r, g in reg_groups.items() if force_register or not (r / "poses.json").is_file()}
    # Sessions registered earlier (or reviewed since): refresh their state from poses.json, so the
    # review queue and the summary describe the poses that will actually be fused.
    for r, g in reg_groups.items():
        if r not in to_register:
            s = g[0].session
            old = {k: v for k, v in (read_register_state(r) or {}).items()
                   if k != "failure_reason" or "checkpoint" in str(v)}
            write_register_state(r, {**old, "wave": s.wave, "height": s.height,
                                     "session": s.name, "reg_dir": str(r), **summarize_poses(r / "poses.json")})

    log(f"{len(jobs)} jobs planned, {len(jobs) - len(todo)} already done, {len(todo)} to run; "
        f"{len(reg_groups)} session registration(s), {len(to_register)} to run "
        f"({workers} workers x {cfg.get('threads_per_worker')} threads) -> {run_dir}")
    write_run_metadata(cfg, run_dir, jobs)
    if not todo and not to_register:
        write_review_queue(run_dir)
        return collect_results(run_dir)

    from .crop import is_cropped
    sessions = {}
    for j in todo + [g[0] for g in to_register.values()]:
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

        t0 = time.monotonic()
        pending: dict[cf.Future, tuple[str, object]] = {}

        def submit_jobs(js):
            # Biggest fusions first: better packing of the long jobs.
            for j in sorted(js, key=lambda j: -j.combo.n_scans if j.combo.kind == "fused" else 0):
                pending[pool.submit(_run_one, cfg.path, j, run_dir)] = ("job", j)

        # Registrations first: they are the long pole, and gate their sessions' combinations.
        for reg, group in sorted(to_register.items(), key=lambda kv: str(kv[0])):
            j = group[0]
            pending[pool.submit(_register_worker, cfg.path, j.session, j.params, run_dir, force_register)] = ("reg", reg)
        waiting = {r: [j for j in todo if j.registers_per_session and j.reg_dir(run_dir) == r] for r in to_register}
        submit_jobs([j for j in todo if not (j.registers_per_session and j.reg_dir(run_dir) in to_register)])

        n_total = len(todo) + len(to_register)
        done = 0
        while pending:
            finished, _ = cf.wait(pending, return_when=cf.FIRST_COMPLETED)
            for f in finished:
                kind, item = pending.pop(f)
                done += 1
                el = time.monotonic() - t0
                if kind == "reg":
                    try:
                        st = f.result()
                    except Exception as exc:   # worker died
                        st = {"state": "failed", "failure_reason": f"CRASHED {type(exc).__name__}: {exc}"}
                        item.mkdir(parents=True, exist_ok=True)
                        write_register_state(item, st)
                    extra = (f"flagged {st.get('n_flagged', 0)}: {st.get('flagged_scans', '')} "
                             f"{('flags ' + st['session_flags']) if st.get('session_flags') else ''}"
                             if st.get("state") in ("needs_review", "clean")
                             else st.get("failure_reason", ""))
                    log(f"[{done}/{n_total}  {el / 60:.0f} min] REGISTER {st.get('wave')}/{st.get('height')}: "
                        f"{st.get('state')} in {st.get('register_s', 0)} s  {extra}")
                    # Its combinations run either way: without poses they record why they could not.
                    # With new poses, every combination fused from the old ones is stale too.
                    counted = waiting.pop(item, [])
                    js = counted
                    if (item / "poses.json").is_file():
                        js = [j for j in reg_groups[item] if not job_is_current(j, run_dir, retry_failed)]
                    submit_jobs(js)
                    n_total += len(js) - len(counted)
                    continue
                j = item
                try:
                    msg = _job_message(f.result())
                except Exception as exc:   # worker died (e.g. out of memory)
                    msg = f"CRASHED {type(exc).__name__}: {exc}"
                    d = j.dir(run_dir)
                    d.mkdir(parents=True, exist_ok=True)
                    row = _base_row(j)
                    row.update(status="failed", failure_stage="worker", failure_reason=msg)
                    (d / ROW_FILE).write_text(json.dumps(row, indent=1, default=str))
                eta = el / done * (n_total - done)
                log(f"[{done}/{n_total}  {el / 60:.0f} min, eta {eta / 60:.0f} min] {j.label()}: {msg}")
    write_review_queue(run_dir)
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
        # Rows from before registration modes existed were all per-combination.
        if "registration_mode" not in df:
            df["registration_mode"] = None
        df["registration_mode"] = df["registration_mode"].fillna(
            df["kind"].map({"fused": PER_COMBO, "single": "none"}))
        df = df.sort_values(["group", "wave", "height_mm", "kind", "n_scans", "combo_id"]).reset_index(drop=True)
        df.to_csv(run_dir / "results.csv", index=False)
    return df


# ─────────────────────────────────────────────────────────────────────────────
#  Summary and review
# ─────────────────────────────────────────────────────────────────────────────

def print_summary(run_dir: Path, df: pd.DataFrame, log=print, review_cmd: str | None = None):
    st = session_states(run_dir)
    if not st.empty:
        counts = st["state"].value_counts().to_dict()
        log("Sessions: " + ", ".join(f"{counts.get(k, 0)} {k}" for k in
                                     ("clean", "needs_review", "reviewed", "timeout", "failed")))
        for _, r in st[st["state"] != "clean"].iterrows():
            why = r.get("reasons") if r.get("state") in ("needs_review", "reviewed") else r.get("failure_reason")
            flags = r.get("session_flags") if isinstance(r.get("session_flags"), str) else ""
            log(f"  {r.get('wave')}/{r.get('height')}: {r.get('state')}  {flags}  {str(why or '')[:200]}")
    if not df.empty:
        log(f"Jobs: {df['status'].value_counts().to_dict()}")
        if "n_flagged_scans_in_combo" in df:
            affected = df[(df["kind"] == "fused") & (df["n_flagged_scans_in_combo"].fillna(0) > 0)]
            log(f"Combinations that include a flagged scan: {len(affected)}")
    queue = run_dir / REVIEW_QUEUE
    if queue.exists():
        q = pd.read_csv(queue)
        reviewable = q[q["state"] == "needs_review"]
        if len(q):
            log(f"Review queue: {queue} ({len(reviewable)} to review, {len(q) - len(reviewable)} timed out / failed)")
        if len(reviewable) and review_cmd:
            log(f"Start the review with:\n  {review_cmd}")


def run_review(cfg, jobs: list[Job], run_dir: Path, workers: int, log=print) -> pd.DataFrame:
    """Walks review_queue.csv: opens fuse_scans --review on each session that needs review (one at
    a time, blocking until its viewer is closed), then re-runs that session's combinations, whose
    poses have changed. Timed-out or failed registrations cannot be reviewed - they have no poses."""
    import subprocess

    from .config import fuse_args

    queue = write_review_queue(run_dir)
    todo = queue[queue["state"] == "needs_review"]
    for _, r in queue[queue["state"] != "needs_review"].iterrows():
        log(f"  skip {r['wave']}/{r['height']}: {r['state']} ({r['failure_reason']}) - re-run with --force-register")
    if todo.empty:
        log("Nothing to review.")
        return collect_results(run_dir)
    for i, (_, r) in enumerate(todo.iterrows(), 1):
        reg = Path(r["reg_dir"])
        group = [j for j in jobs if j.registers_per_session and j.reg_dir(run_dir) == reg]
        if not group:
            log(f"  skip {r['wave']}/{r['height']}: not in the planned jobs (check --waves/--heights)")
            continue
        session = group[0].session
        poses = reg / "poses.json"
        cmd = ([str(cfg.fuse_binary), "--review", "--load-poses", str(poses), "--save-poses", str(poses),
                "--qc", str(reg / "qc.json"), "-o", str(reg / "register_fused.ply"),
                "--sessions", str(reg / "staging" / session.name)] + fuse_args(group[0].params))
        log(f"[review {i}/{len(todo)}] {r['wave']}/{r['height']}: flagged {r['flagged_scans']} "
            f"{r['session_flags'] if isinstance(r['session_flags'], str) else ''}")
        log("  " + " ".join(cmd))
        before = file_sha1(poses)
        subprocess.run(cmd, check=False)
        state = {**(read_register_state(reg) or {}), **summarize_poses(poses)}
        write_register_state(reg, state)
        if file_sha1(poses) == before:
            log("  poses unchanged - combinations left as they are")
            continue
        log(f"  {state['state']}; re-fusing {len(group)} combination(s) from the reviewed poses")
        run_batch(cfg, group, run_dir, workers, log=log)
    write_review_queue(run_dir)
    return collect_results(run_dir)
