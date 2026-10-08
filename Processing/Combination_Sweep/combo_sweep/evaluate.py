"""Seed -> part trim -> Evaluate, for a fused cloud or a single cropped scan.

Neither tool is reimplemented or modified (the same chain as Filter_Sweep,
minus the crop, which happened before fusion):
  * ../Parameter_Sweep/sweep/prepare.py  seed_transform() - table plane to
    z = 0, part footprint over the reference
  * ../Evaluate/scan_eval  evaluate_scan() - alignment and every metric
The part trim (points more than part_trim_margin_mm outside the reference's
footprint in the seeded frame are dropped) is Filter_Sweep's: without it
Evaluate's search can lock onto a tilted pose over the ~500 mm tag square.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

from .config import CROP_DIR, EVALUATE_DIR, PARAMETER_SWEEP_DIR

for _p in (CROP_DIR, EVALUATE_DIR, PARAMETER_SWEEP_DIR):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import open3d as o3d  # noqa: E402
import crop_pointclouds as crop_tool  # noqa: E402
from sweep.prepare import seed_transform  # noqa: E402  (Parameter_Sweep)
from scan_eval.params import EvalParams  # noqa: E402
from scan_eval.pipeline import evaluate_scan  # noqa: E402
from scan_eval.reference import UnitScaled  # noqa: E402
from scan_eval.surface_def import load_reference  # noqa: E402

o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)

# Evaluate result keys that describe the run, not the scan's quality.
_SKIP_KEYS = {"label", "scan", "run_dir", "timestamp", "seed", "units", "scale"}


class StageError(RuntimeError):
    def __init__(self, stage, msg):
        super().__init__(f"{stage}: {msg}")
        self.stage = stage


_references = {}


def reference(path: Path, units: str):
    key = (str(path), units)
    if key not in _references:
        ref = load_reference(path, units_override=units, height_field=True)
        ref_m = UnitScaled(ref)
        ref_m.units_label = f"{ref.units}, from {getattr(ref, 'units_source', 'definition')}"
        _references[key] = ref_m
    return _references[key]


def seed_trim_evaluate(cfg, cloud_path: Path, wave: str, job_dir: Path, run_name: str) -> dict:
    """Metrics row (Evaluate's result plus seed/trim info) for one cloud."""
    row = {}
    ref = reference(cfg.reference(wave), cfg["evaluate"]["reference_units"])
    pcd = o3d.io.read_point_cloud(str(cloud_path))
    points = np.asarray(pcd.points)
    row["points_input"] = len(points)
    if len(points) == 0:
        raise StageError("load", f"{cloud_path.name} has no points")

    try:
        T0, seed_info = seed_transform(points, ref.bounds())
    except Exception as exc:
        raise StageError("seed", exc) from None
    row.update(seed_info)

    lo, hi = ref.bounds()
    margin = cfg["part_trim_margin_mm"] / 1000.0
    q = points @ T0[:3, :3].T + T0[:3, 3]
    keep = ((q[:, 0] > lo[0] - margin) & (q[:, 0] < hi[0] + margin) &
            (q[:, 1] > lo[1] - margin) & (q[:, 1] < hi[1] + margin))
    if keep.sum() < 100:
        raise StageError("trim", f"only {int(keep.sum())} points near the part")
    row["points_trimmed"] = int(keep.sum())
    trimmed_path = job_dir / "trimmed.ply"
    crop_tool.write_ply_like_original(trimmed_path, o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points[keep])))
    np.savetxt(job_dir / "seed_transform.txt", T0, fmt="%.12f",
               header="seed pose, cloud (trimmed, metres) -> reference (metres)")

    params = EvalParams()
    for k, v in cfg["evaluate"].items():
        setattr(params, k, v)
    params.label = run_name
    params.output = str(job_dir)
    try:
        result = evaluate_scan(ref, trimmed_path, params, job_dir, "eval", T0)
    except Exception as exc:
        raise StageError("evaluate", exc) from None
    finally:
        trimmed_path.unlink(missing_ok=True)
    row.update({k: v for k, v in result.items() if k not in _SKIP_KEYS})
    try:
        doc = json.loads((job_dir / "eval" / "metrics.json").read_text())
        chosen = (doc.get("alignment") or {}).get("chosen") or {}
        row["align_chosen"] = chosen.get("name")
        row["align_overlap_ref"] = chosen.get("overlap_ref")
    except (OSError, json.JSONDecodeError):
        pass
    return row
