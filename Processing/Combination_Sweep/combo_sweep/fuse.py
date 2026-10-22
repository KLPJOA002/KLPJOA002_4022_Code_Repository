"""Run fuse_scans on a session's scans.

The scans are presented as a normal session folder, so fuse_scans discovers
them exactly as it would a real capture (rotation and tilt from the folder
names, IMU files, tag sidecars):

  <dir>/staging/<session>/<session>_r<rot>/<session>_r<rot>_a<tilt>  -> symlink to the cropped tilt folder

Two ways of using it (see README, "Registration modes"):
  session_poses  register_session() once per session on every scan, writing
                 poses.json; then fuse_from_poses() per combination, which only
                 fuses (--load-poses --scans), no registration
  per_combo      run_fuse() per combination: registration and fusion from
                 scratch on just that combination's scans (the original sweep)

fuse_scans exits 3 when it wrote its output but something needs review; that
is a result, not a failure, and is reported in the returned info.
"""

from __future__ import annotations

import contextlib
import json
import os
import shlex
import signal
import subprocess
import threading
import time
from pathlib import Path

from .config import fuse_args
from .crop import cropped_ply


class FuseError(RuntimeError):
    def __init__(self, msg, info=None):
        super().__init__(msg)
        self.info = info or {}


def stage_session(job_dir: Path, crop_dir: Path, session, scans) -> Path:
    root = job_dir / "staging" / session.name
    for scan in scans:
        src = cropped_ply(crop_dir, session, scan).parent
        rot_dir = root / scan.tilt_dir.parent.name
        rot_dir.mkdir(parents=True, exist_ok=True)
        link = rot_dir / scan.tilt_dir.name
        if link.is_symlink() or link.exists():
            link.unlink()
        link.symlink_to(src, target_is_directory=True)
    return root


NEEDS_REVIEW_EXIT = 3


def run_fuse(binary: Path, staged_session: Path, out_ply: Path, params: dict, log_path: Path,
             timeout_s: float, threads: int | None, extra_args=()) -> dict:
    """Returns {fuse_s, fuse_peak_rss_mb, fuse_cmd, fuse_exit_code}. Raises FuseError on
    failure; exit code 3 (written, needs review) is not one."""
    cmd = ([str(binary), "-o", str(out_ply), "--sessions", str(staged_session)] + fuse_args(params)
           + [str(a) for a in extra_args] + ["--no-gui"])
    env = dict(os.environ)
    if threads:
        env["OMP_NUM_THREADS"] = str(threads)
    if out_ply.exists():
        out_ply.unlink()
    t0 = time.monotonic()
    with open(log_path, "w") as log:
        log.write("$ " + shlex.join(cmd) + "\n\n")
        log.flush()
        proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                env=env, start_new_session=True)
        timed_out = threading.Event()

        def kill():
            timed_out.set()
            with contextlib.suppress(ProcessLookupError, PermissionError):
                os.killpg(proc.pid, signal.SIGKILL)

        timer = threading.Timer(timeout_s, kill) if timeout_s else None
        if timer:
            timer.start()
        try:
            _, status, rusage = os.wait4(proc.pid, 0)   # rusage of this child only
        finally:
            if timer:
                timer.cancel()
        proc.returncode = os.waitstatus_to_exitcode(status)
    elapsed = time.monotonic() - t0
    info = {"fuse_s": round(elapsed, 2), "fuse_peak_rss_mb": round(rusage.ru_maxrss / 1024, 1),
            "fuse_cmd": shlex.join(cmd), "fuse_exit_code": proc.returncode}
    if timed_out.is_set():
        raise FuseError(f"timed out after {timeout_s:.0f} s", info)
    if proc.returncode not in (0, NEEDS_REVIEW_EXIT) or not out_ply.is_file():
        tail = log_path.read_text(errors="replace").strip().splitlines()[-3:]
        raise FuseError(f"exit code {proc.returncode}: {' / '.join(tail)}", info)
    return info



def register_session(binary: Path, staged_session: Path, reg_dir: Path, params: dict, timeout_s: float,
                     threads: int | None, auto_retry: bool, auto_retry_max_scans: int | None) -> dict:
    """Stage 1: register every scan of the session once, writing reg_dir/poses.json and
    qc.json (and register_fused.ply, the all-scans fusion as a by-product)."""
    extra = ["--save-poses", reg_dir / "poses.json", "--qc", reg_dir / "qc.json"]
    if auto_retry:
        extra.append("--auto-retry")
        if auto_retry_max_scans:
            extra += ["--auto-retry-max-scans", auto_retry_max_scans]
    info = run_fuse(binary, staged_session, reg_dir / "register_fused.ply", params, reg_dir / "register.log",
                    timeout_s, threads, extra)
    if not (reg_dir / "poses.json").is_file():
        raise FuseError("fuse_scans finished but wrote no poses.json", info)
    return info


def fuse_from_poses(binary: Path, staged_session: Path, poses: Path, scan_keys, out_ply: Path, qc_path: Path,
                    params: dict, log_path: Path, timeout_s: float, threads: int | None) -> dict:
    """Stage 2: fuse just these scans at their stored poses (no registration)."""
    extra = ["--load-poses", poses, "--scans", ",".join(scan_keys), "--qc", qc_path]
    return run_fuse(binary, staged_session, out_ply, params, log_path, timeout_s, threads, extra)


def qc_columns(qc_path: Path) -> dict:
    """results.csv columns from a fuse_scans --qc file (empty if there is none)."""
    try:
        qc = json.loads(Path(qc_path).read_text())
    except (OSError, json.JSONDecodeError):
        return {}
    scans = qc.get("scans") or {}
    flagged = sorted(k for k, s in scans.items() if s.get("needs_review") and s.get("status") != "excluded")
    reasons = sorted({r for k in flagged for r in scans[k].get("qc_reasons") or []})
    rec = qc.get("reconcile") or {}
    return {
        "session_review_state": qc.get("review_state"),
        "n_flagged_scans_in_combo": len(flagged),
        "flagged_scans": ";".join(flagged),
        "flag_reasons": ";".join(reasons),
        "fusion_flags": ";".join(qc.get("session_flags") or []),
        "spread_before_mm": rec.get("spread_before_mm"),
        "spread_after_mm": rec.get("spread_after_mm"),
    }
