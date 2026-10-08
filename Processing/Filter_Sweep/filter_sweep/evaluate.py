"""Crop -> seed -> part trim -> Evaluate, for one filtered point cloud.

Neither tool is reimplemented or modified:
  * ../Crop/crop_pointclouds.py  compute_auto_crop() - the automatic AprilTag
    crop, run without its preview window (as Parameter_Sweep does)
  * ../Parameter_Sweep/sweep/prepare.py  seed_transform() - table plane to
    z = 0, part footprint over the reference
  * ../Evaluate/scan_eval  evaluate_scan() - alignment and every metric

The cropper finds its inputs next to the .ply path it is given: the
frame number (imu_a*.csv), the tag detections and the recording (for the
colour intrinsics). Each job gets a small staging folder holding those, with
the frame number set to the rs_marker frame of the selected frame, and
symlinks to the original detections and recording. The cloud itself is
passed in memory.
"""

from __future__ import annotations

import contextlib
import sys
import types
from pathlib import Path

import numpy as np

from .config import CROP_DIR, EVALUATE_DIR, PARAMETER_SWEEP_DIR

for _p in (CROP_DIR, EVALUATE_DIR, PARAMETER_SWEEP_DIR):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import open3d as o3d  # noqa: E402
import crop_pointclouds as crop_tool  # noqa: E402
from sweep import config as seed_config  # noqa: E402  (Parameter_Sweep)
from sweep.prepare import seed_transform  # noqa: E402
from scan_eval.params import EvalParams  # noqa: E402
from scan_eval.pipeline import evaluate_scan  # noqa: E402
from scan_eval.reference import UnitScaled  # noqa: E402
from scan_eval.surface_def import load_reference  # noqa: E402

o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)


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


def tag_layout(height_dir: Path):
    """Corner-tag layout learned from every session of this (shape, height),
    as crop_pointclouds.py learns it for its input directory. At 641 mm the
    far tags (2, 5) never decode, so no order can be learned there and the
    cropper would guess slots from geometry, taking an undecoded square on
    the part for a corner tag (crop lands on bare table). The tag board is
    the same at every height, so fall back to the layout of the whole shape."""
    layout = crop_tool.learn_tag_layout(height_dir)
    if layout.order is None:
        layout = crop_tool.learn_tag_layout(height_dir.parent)
    return layout


def stage_scan_folder(stage_root: Path, scan_dir: Path, rs_frame: int) -> Path:
    """<stage_root>/<scan folder name>/ with imu_a0.csv (frame number) and
    symlinks to the detections and the recording. Returns the .ply path the
    cropper should be told about (the file itself is not needed)."""
    d = stage_root / scan_dir.name
    d.mkdir(parents=True, exist_ok=True)
    for src in (scan_dir / f"{scan_dir.name}_detections.csv", scan_dir / f"{scan_dir.name}.db3"):
        link = d / src.name
        if not link.exists():
            link.symlink_to(src)
    (d / "imu_a0.csv").write_text("frame,timestamp_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z\n"
                                  f"{rs_frame},0,0,0,0,0,0,0\n")
    return d / f"{scan_dir.name}.ply"


@contextlib.contextmanager
def _seed_spacing(points, decimation: int, row: dict):
    """seed_transform clusters the raised part with a fixed 6 mm DBSCAN
    radius (Parameter_Sweep's SEED_CLUSTER_EPS_M), tuned for full-resolution
    clouds (~1.4 mm spacing at 0.9 m) and DBSCAN needs 20 neighbours inside
    it. Decimation by M spreads the points M times further apart, so for
    decimated clouds only, the radius is scaled by M for this call (measured
    at 863 mm: decimation 2 with 6 mm finds only fragments of <= 100 points,
    12 mm finds the part as one 1685-point cluster)."""
    if decimation <= 1:
        yield
        return
    old = seed_config.SEED_CLUSTER_EPS_M
    seed_config.SEED_CLUSTER_EPS_M = old * decimation
    row["seed_cluster_eps_mm"] = round(seed_config.SEED_CLUSTER_EPS_M * 1e3, 2)
    try:
        yield
    finally:
        seed_config.SEED_CLUSTER_EPS_M = old


def crop_seed_trim_evaluate(points: np.ndarray, ply_path: Path, layout, ref, settings, job_dir: Path,
                            run_name: str, row: dict) -> dict:
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points.astype(np.float64)))
    row["points_filtered"] = len(points)

    try:
        args = types.SimpleNamespace(frame_window=settings.crop_frame_window, margin=settings.crop_margin_px)
        auto = crop_tool.compute_auto_crop(pcd, ply_path, ply_path.parent, layout, args)
    except Exception as exc:
        raise StageError("crop", exc) from None
    cropped = np.asarray(pcd.points)[auto.mask]
    row["points_cropped"] = int(auto.mask.sum())
    row["crop_notes"] = " | ".join(auto.notes)

    try:
        with _seed_spacing(cropped, row.get("decimation_magnitude", 1), row):
            T0, seed_info = seed_transform(cropped, ref.bounds())
    except Exception as exc:
        raise StageError("seed", exc) from None
    row.update(seed_info)

    lo, hi = ref.bounds()
    margin = settings.part_trim_margin_mm / 1000.0
    q = cropped @ T0[:3, :3].T + T0[:3, 3]
    keep = ((q[:, 0] > lo[0] - margin) & (q[:, 0] < hi[0] + margin) &
            (q[:, 1] > lo[1] - margin) & (q[:, 1] < hi[1] + margin))
    if keep.sum() < 100:   # Evaluate's own minimum; decimation 8 leaves ~200 points on the part
        raise StageError("trim", f"only {int(keep.sum())} points near the part")
    row["points_trimmed"] = int(keep.sum())
    trimmed_path = job_dir / "trimmed.ply"
    crop_tool.write_ply_like_original(
        trimmed_path, o3d.geometry.PointCloud(o3d.utility.Vector3dVector(cropped[keep])))
    np.savetxt(job_dir / "seed_transform.txt", T0, fmt="%.12f",
               header="seed pose, scan (trimmed, metres) -> reference (metres)")

    params = EvalParams()
    for k, v in settings.evaluate.items():
        setattr(params, k, v)
    params.label = run_name
    params.output = str(job_dir)
    try:
        result = evaluate_scan(ref, trimmed_path, params, job_dir, "eval", T0)
    except Exception as exc:
        raise StageError("evaluate", exc) from None
    return result
