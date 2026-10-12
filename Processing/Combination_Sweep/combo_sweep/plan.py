"""Job listing, runtime and disk estimates (from config defaults, or from
timings measured in an earlier run directory)."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pandas as pd

from .runner import Job, job_state


class Estimator:
    """fuse_s = a + b * n_scans (per scan-voxel), eval_s = median per kind."""

    def __init__(self, cfg, measured: pd.DataFrame | None = None):
        e = cfg["estimate_s"]
        self.source = "config estimate_s (no measured timings)"
        self.fuse = {None: (0.0, float(e["fuse_per_scan"]))}
        self.eval = {"fused": float(e["evaluate"]), "single": float(e["evaluate"])}
        self.rss = None
        if measured is not None and not measured.empty and "fuse_s" in measured:
            ok = measured[measured["status"] != "failed"]
            fused = ok[(ok["kind"] == "fused") & ok["fuse_s"].notna()]
            if len(fused) >= 2:
                for sv, g in fused.groupby("scan_voxel"):
                    if g["n_scans"].nunique() >= 2:
                        b, a = np.polyfit(g["n_scans"], g["fuse_s"], 1)
                        self.fuse[sv] = (max(a, 0.0), max(b, 0.0))
                    else:
                        self.fuse[sv] = (0.0, float((g["fuse_s"] / g["n_scans"]).median()))
                self.fuse[None] = self.fuse.get(cfg.fuse_params().get("scan-voxel"), next(iter(self.fuse.values())))
                self.rss = fused.groupby("n_scans")["fuse_peak_rss_mb"].max()
                self.source = f"measured ({len(fused)} fusions)"
            for kind, g in ok.groupby("kind"):
                if g["eval_s"].notna().any():
                    self.eval[kind] = float(g["eval_s"].median())

    def seconds(self, job: Job) -> float:
        if job.combo.kind == "single":
            return self.eval["single"]
        a, b = self.fuse.get(job.params.get("scan-voxel"), self.fuse[None])
        return a + b * job.combo.n_scans + self.eval["fused"]


def summarize(cfg, jobs: list[Job], run_dir: Path | None, workers: int, measured=None, verbose=False, log=print):
    est = Estimator(cfg, measured)
    remaining = [j for j in jobs if run_dir is None or job_state(j, run_dir) is None]
    fused = [j for j in remaining if j.combo.kind == "fused"]
    singles = [j for j in remaining if j.combo.kind == "single"]
    sessions = {(j.session.wave, j.session.height): j.session for j in jobs}

    log(f"Sessions ({len(sessions)}):")
    for s in sessions.values():
        others = ", ".join(f"{k} ({v} scans)" for k, v in s.other_sessions.items())
        log(f"  {s.wave:10s} {s.height:6s} use {s.name} ({len(s.scans)} scans: rot {s.rotations}, "
            f"tilt {s.tilts[0]}..{s.tilts[-1]}); ignore {others}")

    if verbose:
        log("\nJobs:")
        for j in remaining:
            log(f"  {j.label():70s} n_scans={j.combo.n_scans:2d}  ~{est.seconds(j):5.0f} s")

    combos = {}
    for j in jobs:
        combos.setdefault(j.combo.combo_id, j.combo)
    n_fused_c = sum(c.kind == "fused" for c in combos.values())
    log(f"\nCombinations: {n_fused_c} fused + {len(combos) - n_fused_c} single per (wave, height)")
    log(f"Jobs: {len(jobs)} planned, {len(remaining)} to run ({len(fused)} fusions, {len(singles)} single-scan evaluations)")
    total = sum(est.seconds(j) for j in remaining)
    log(f"Timing source: {est.source}")
    for sv, (a, b) in est.fuse.items():
        if sv is not None:
            log(f"  scan-voxel {sv}: fuse ~ {a:.0f} s + {b:.1f} s/scan")
    log(f"  evaluate ~ {est.eval['fused']:.0f} s per fused cloud, {est.eval['single']:.0f} s per single scan")
    log(f"CPU-serial time: {total / 3600:.1f} h; wall time with {workers} workers: ~{total / workers / 3600:.1f} h "
        f"(assumes workers stay busy; the longest job sets a floor)")
    if est.rss is not None and len(est.rss):
        top = est.rss.index.max()
        log(f"Peak fuse RAM measured: {est.rss.max():.0f} MB (at {top} scans: {est.rss.loc[top]:.0f} MB) "
            f"-> {workers} workers need up to ~{workers * est.rss.max() / 1024:.1f} GB")

    mb = cfg["estimate_mb"]
    n_sessions_to_crop = len(sessions)
    disk = len(fused) * (mb["fused_ply"] if cfg.get("keep_fused_ply", True) else 0) + 1.0 * len(remaining)
    crop_disk = n_sessions_to_crop * 44 * mb["cropped_scan"]
    log(f"Disk: ~{disk / 1024:.1f} GB in the run directory (fused PLYs + logs), "
        f"crop cache up to ~{crop_disk / 1024:.1f} GB (shared, reused across runs)")
    return total
