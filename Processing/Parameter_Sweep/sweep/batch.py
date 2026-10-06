"""Batch evaluation of every scan in the sweep, with a per-scan cache.

For each scan: parse scan_info.txt -> crop (../Crop) -> seed pose -> Evaluate's
evaluate_scan() against the object's reference -> registration check. The
result (or the failure and its reason) is cached as JSON keyed by a
fingerprint of the inputs and settings, so a re-run only redoes new or
changed scans. Nothing is written to the data directory.
"""

from __future__ import annotations

import contextlib
import hashlib
import json
import os
import sys
import time
from pathlib import Path

import numpy as np
import pandas as pd
from tqdm import tqdm

from . import config as C
from .scan_info import derive_parameters, find_scans, previous_presets, read_scan_info

if str(C.EVALUATE_DIR) not in sys.path:
    sys.path.insert(0, str(C.EVALUATE_DIR))
from scan_eval.params import EvalParams  # noqa: E402
from scan_eval.reference import UnitScaled  # noqa: E402

# Leading columns of all_scans.csv; everything else follows, raw scan_info last.
ID_COLUMNS = ["object", "object_label", "session", "status", "failure_reason",
              "resolution", "res_megapixels", "depth_units_mm", "preset", "preset_actual",
              "preset_mismatch", "preset_previous"]
KEY_METRICS = ["acc_rmse_mm", "acc_mae_mm", "acc_median_mm", "acc_rmse_trim5_mm", "signed_bias_mm",
               "signed_std_mm", "covered_fraction", "completeness@2mm", "precision@2mm", "fscore@2mm",
               "chamfer_mm", "outlier_fraction", "points_scored", "align_overlap_ref", "align_ambiguous"]


def _json_default(o):
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, (np.bool_,)):
        return bool(o)
    if isinstance(o, (np.ndarray,)):
        return o.tolist()
    if isinstance(o, Path):
        return str(o)
    raise TypeError(f"not JSON serialisable: {type(o)}")


def _file_stamp(path: Path | None):
    if path is None or not path.exists():
        return None
    st = path.stat()
    return [path.name, st.st_size, st.st_mtime_ns]


class Evaluator:
    def __init__(self, data_dir: Path, out_dir: Path):
        self.data_dir = Path(data_dir)
        self.out_dir = Path(out_dir)
        self.cache_dir = self.out_dir / "cache"
        self.crop_dir = self.out_dir / "cropped"
        self.runs_dir = self.out_dir / "eval_runs"
        self._refs = {}

    # --- references -----------------------------------------------------------
    def reference(self, obj: str):
        if obj not in self._refs:
            from scan_eval.surface_def import load_reference
            spec = C.OBJECTS[obj]
            ref = load_reference(spec.reference, units_override=C.EVAL_OVERRIDES["reference_units"],
                                 height_field=C.HEIGHT_FIELD)
            ref_m = UnitScaled(ref)
            ref_m.units_label = f"{ref.units}, from {getattr(ref, 'units_source', 'definition')}"
            self._refs[obj] = ref_m
        return self._refs[obj]

    def params(self, label: str) -> EvalParams:
        p = EvalParams()
        for k, v in C.EVAL_OVERRIDES.items():
            setattr(p, k, v)
        p.output = str(self.runs_dir)
        p.label = label
        return p

    # --- cache ------------------------------------------------------------------
    def fingerprint(self, scan) -> str:
        det = next(iter(sorted(scan.scan_dir.glob("*_detections.csv"))), None)
        imu = next(iter(sorted(scan.scan_dir.glob("imu*.csv"))), None)
        spec = C.OBJECTS[scan.object]
        payload = {
            "ply": _file_stamp(scan.ply), "info": _file_stamp(scan.info_path),
            "detections": _file_stamp(det), "imu": _file_stamp(imu),
            "reference": [str(spec.reference), _file_stamp(spec.reference)],
            "settings": C.eval_settings(),
        }
        return hashlib.sha256(json.dumps(payload, sort_keys=True, default=str).encode()).hexdigest()[:20]

    def cache_path(self, scan) -> Path:
        return self.cache_dir / scan.object / f"{scan.session}.json"

    def cached(self, scan, fp):
        path = self.cache_path(scan)
        if not path.is_file():
            return None
        try:
            doc = json.loads(path.read_text())
        except (OSError, json.JSONDecodeError):
            return None
        return doc["row"] if doc.get("fingerprint") == fp else None

    def store(self, scan, fp, row):
        path = self.cache_path(scan)
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(".tmp")
        tmp.write_text(json.dumps({"fingerprint": fp, "row": row}, indent=1, default=_json_default))
        os.replace(tmp, path)

    # --- one scan ---------------------------------------------------------------
    def evaluate_one(self, scan) -> dict:
        from scan_eval.pipeline import evaluate_scan
        from .prepare import PrepareError, crop_scan, seed_transform

        spec = C.OBJECTS[scan.object]
        row = {"object": scan.object, "object_label": spec.label, "session": scan.session,
               "scan_dir": str(scan.scan_dir), "ply": str(scan.ply) if scan.ply else None,
               "n_ply": scan.n_ply, "reference_file": spec.reference.name}
        info = read_scan_info(scan.info_path)
        row.update(derive_parameters(info))
        row.update({f"info.{k}": v for k, v in info.items()})

        started = time.time()
        stage = "load"
        try:
            if scan.ply is None:
                raise PrepareError("no .ply in the scan folder")
            stage = "crop"
            cropped = self.crop_dir / scan.object / f"{scan.session}_cropped.ply"
            row.update(crop_scan(scan.ply, self.data_dir / scan.object, cropped))
            row["cropped_ply"] = str(cropped)

            stage = "seed"
            ref = self.reference(scan.object)
            import open3d as o3d
            pts = np.asarray(o3d.io.read_point_cloud(str(cropped)).points, dtype=float)
            T0, seed_info = seed_transform(pts, ref.bounds())
            row.update(seed_info)

            stage = "evaluate"
            name = f"{scan.object}__{scan.session}"
            run_dir = self.runs_dir / name
            run_dir.mkdir(parents=True, exist_ok=True)
            np.savetxt(run_dir / "seed_transform.txt", T0, fmt="%.12f",
                       header="seed pose, scan (cropped, metres) -> reference (metres); "
                              "pass to evaluate_scans.py --init-transform to reproduce")
            params = self.params(spec.label)
            params.init_transform = str(run_dir / "seed_transform.txt")
            result = evaluate_scan(ref, cropped, params, self.runs_dir, name, T0)
            row.update({k: v for k, v in result.items() if k not in ("label", "scan")})
            row["eval_scan"] = result.get("scan")

            doc = json.loads((run_dir / "metrics.json").read_text())
            chosen = (doc.get("alignment") or {}).get("chosen") or {}
            row["align_chosen"] = chosen.get("name")
            row["align_overlap_scan"] = chosen.get("overlap_scan")
            row["align_overlap_ref"] = chosen.get("overlap_ref")
            row["align_candidate_rmse_mm"] = chosen.get("rmse_mm")

            row["status"] = "ok"            # registration check applied in check_registration()
            row["failure_reason"] = None
        except KeyboardInterrupt:
            raise
        except Exception as exc:  # recorded in the table, never fatal to the batch
            row["status"] = "failed"
            msg = str(exc).splitlines()[0] if str(exc) else type(exc).__name__
            row["failure_reason"] = f"{stage}: {msg}" if not msg.startswith(f"{stage}:") else msg
        if scan.n_ply > 1:
            row["note"] = f"{scan.n_ply} PLYs in folder, used {scan.ply.name}"
        row["elapsed_s"] = round(time.time() - started, 1)
        return row

    # --- batch ------------------------------------------------------------------
    def run(self, objects=None, sessions=None, force=False, retry_failed=False, verbose=False) -> pd.DataFrame:
        objects = objects or list(C.OBJECTS)
        scans = find_scans(self.data_dir, objects)
        previous = previous_presets(scans)
        if sessions:
            scans = [s for s in scans if s.session in sessions]
        if not scans:
            raise SystemExit(f"no scans found under {self.data_dir} for objects {objects}")

        rows, todo = [], []
        for scan in scans:
            fp = self.fingerprint(scan)
            row = None if force else self.cached(scan, fp)
            if row is not None and retry_failed and row.get("status") == "failed":
                row = None
            if row is None:
                todo.append((scan, fp))
            else:
                check_registration(row)
                rows.append(row)
        print(f"{len(scans)} scan(s): {len(rows)} cached, {len(todo)} to evaluate")

        devnull = open(os.devnull, "w")
        try:
            for scan, fp in tqdm(todo, desc="evaluating", unit="scan", dynamic_ncols=True):
                with contextlib.ExitStack() as stack:
                    if not verbose:
                        stack.enter_context(contextlib.redirect_stdout(devnull))
                        stack.enter_context(contextlib.redirect_stderr(devnull))
                    row = self.evaluate_one(scan)
                self.store(scan, fp, row)
                check_registration(row)
                if row["status"] != "ok":
                    tqdm.write(f"  [{row['status']}] {scan.object}/{scan.session}: {row['failure_reason']}")
                rows.append(row)
        finally:
            devnull.close()

        for row in rows:
            row["preset_previous"] = previous.get((row["object"], row["session"]))
        return to_frame(rows)


def check_registration(row):
    """Mark an evaluated scan `suspect` if its pose covers too little of the
    reference. Applied on every run (not cached), so the threshold can change
    without re-evaluating."""
    if row.get("status") == "failed":
        return
    cov = row.get("covered_fraction")
    if cov is None or cov < C.MIN_COVERED_FRACTION:
        row["status"] = "suspect"
        row["failure_reason"] = (f"registration: covers only {cov:.0%} of the reference "
                                 f"(< {C.MIN_COVERED_FRACTION:.0%})" if cov is not None
                                 else "registration: no coverage reported")
    else:
        row["status"] = "ok"
        row["failure_reason"] = None


def to_frame(rows) -> pd.DataFrame:
    df = pd.DataFrame(rows)
    lead = [c for c in ID_COLUMNS + KEY_METRICS if c in df.columns]
    info_cols = sorted(c for c in df.columns if c.startswith("info."))
    rest = [c for c in df.columns if c not in lead and c not in info_cols]
    return df[lead + rest + info_cols].sort_values(["object", "session"]).reset_index(drop=True)
