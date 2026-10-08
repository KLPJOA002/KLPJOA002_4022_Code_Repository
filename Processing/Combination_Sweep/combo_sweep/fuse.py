"""Run fuse_scans (unmodified) on one combination of scans.

The combination's scans are presented as a normal session folder, so
fuse_scans discovers them exactly as it would a real capture (rotation and
tilt from the folder names, IMU files, tag sidecars):

  <job>/staging/<session>/<session>_r<rot>/<session>_r<rot>_a<tilt>  -> symlink to the cropped tilt folder
"""

from __future__ import annotations

import contextlib
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


def run_fuse(binary: Path, staged_session: Path, out_ply: Path, params: dict, log_path: Path,
             timeout_s: float, threads: int | None) -> dict:
    """Returns {fuse_s, fuse_peak_rss_mb, fuse_cmd}. Raises FuseError on failure."""
    cmd = [str(binary), "-o", str(out_ply), "--sessions", str(staged_session)] + fuse_args(params) + ["--no-gui"]
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
            "fuse_cmd": shlex.join(cmd)}
    if timed_out.is_set():
        raise FuseError(f"timed out after {timeout_s:.0f} s", info)
    if proc.returncode != 0 or not out_ply.is_file():
        tail = log_path.read_text(errors="replace").strip().splitlines()[-3:]
        raise FuseError(f"exit code {proc.returncode}: {' / '.join(tail)}", info)
    return info

