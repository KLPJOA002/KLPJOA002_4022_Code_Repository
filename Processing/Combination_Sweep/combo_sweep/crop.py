"""Crop every scan of a full-sweep session once, into a shared cache.

Uses ../Crop/crop_pointclouds.py as-is (as Bulk_Process_Pipeline does) and
writes each scan the way that tool writes it, so fuse_scans finds its
sidecars: <scan>_cropped.ply, imu_a<tilt>.csv, <tilt folder>_tags.csv,
*_detections.csv and scan_info.txt. The tag layout is learned per height,
falling back to the whole wave when no frame decodes all four corner tags
(at 641 mm the far tags never decode; Filter_Sweep's fix).

Cache layout: <crop_cache>/<wave>/<height>/<session>/<session>_r*/<..._a*>/
"""

from __future__ import annotations

import contextlib
import json
import shutil
import sys
import types
from pathlib import Path

import numpy as np

from .config import CROP_DIR

if str(CROP_DIR) not in sys.path:
    sys.path.insert(0, str(CROP_DIR))

import open3d as o3d  # noqa: E402
import crop_pointclouds as crop_tool  # noqa: E402

o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)

MARKER = "crop_settings.json"
CROP_VERSION = 1


def settings_key(cfg) -> dict:
    c = cfg["crop"]
    return {"version": CROP_VERSION, "margin_px": c["margin_px"], "frame_window": c["frame_window"]}


def session_crop_dir(cfg, session) -> Path:
    return cfg.crop_cache / session.wave / session.height / session.name


def cropped_ply(crop_dir: Path, session, scan) -> Path:
    rel = scan.ply.relative_to(session.path)
    return crop_dir / rel.with_name(rel.stem + "_cropped.ply")


def load_marker(crop_dir: Path) -> dict | None:
    try:
        return json.loads((crop_dir / MARKER).read_text())
    except (OSError, json.JSONDecodeError):
        return None


def is_cropped(cfg, session) -> bool:
    m = load_marker(session_crop_dir(cfg, session))
    return bool(m and m.get("complete") and m.get("settings") == settings_key(cfg))


def tag_layout(height_dir: Path):
    layout = crop_tool.learn_tag_layout(height_dir)
    source = "height"
    if layout.order is None:
        layout = crop_tool.learn_tag_layout(height_dir.parent)
        source = "wave (no four-tag frame at this height)"
    return layout, source


def crop_session(cfg, session, force: bool = False) -> dict:
    """Crop (or reuse) one session. Returns the marker dict: per-scan status."""
    out_dir = session_crop_dir(cfg, session)
    if not force and is_cropped(cfg, session):
        return load_marker(out_dir)
    if out_dir.exists():
        if load_marker(out_dir) is None and any(out_dir.iterdir()):
            raise RuntimeError(f"{out_dir} exists and wasn't written by this tool")
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True)

    log_path = out_dir / "crop.log"
    scans = []
    with open(log_path, "w") as log, contextlib.redirect_stdout(log):
        layout, layout_source = tag_layout(session.path.parent)
        print(f"tag layout from {layout_source}: order {layout.order}")
        args = types.SimpleNamespace(frame_window=cfg["crop"]["frame_window"], margin=cfg["crop"]["margin_px"])
        for scan in session.scans:
            row = {"scan": scan.key, "status": "failed", "note": "", "points_raw": 0, "points": 0}
            print(f"[{scan.key}] {scan.ply}")
            try:
                pcd = o3d.io.read_point_cloud(str(scan.ply))
                row["points_raw"] = len(pcd.points)
                if len(pcd.points) == 0:
                    raise crop_tool.AutoCropError("empty cloud")
                auto = crop_tool.compute_auto_crop(pcd, scan.ply, session.path, layout, args)
                for note in auto.notes:
                    print(f"    {note}")
                cropped = pcd.select_by_index(np.flatnonzero(auto.mask))
                out = cropped_ply(out_dir, session, scan)
                out.parent.mkdir(parents=True, exist_ok=True)
                crop_tool.write_ply_like_original(out, cropped)
                crop_tool.copy_imu_file(scan.ply, out.parent)
                crop_tool.write_tag_sidecar(scan.ply, pcd, session.path, out.parent)
                crop_tool.copy_scan_metadata(scan.ply, out.parent)
                row.update(status="ok", points=len(cropped.points), note=" | ".join(auto.notes))
            except Exception as exc:  # one bad scan must not stop the session
                row["note"] = f"{type(exc).__name__}: {exc}"
                print(f"    [!] crop failed: {row['note']}")
            log.flush()
            scans.append(row)

    marker = {"settings": settings_key(cfg), "complete": True, "source": str(session.path),
              "layout_source": layout_source, "layout_order": layout.order, "scans": scans}
    (out_dir / MARKER).write_text(json.dumps(marker, indent=2))
    return marker


def crop_status(cfg, session) -> dict:
    """scan key -> crop row, from the cache marker ({} if not cropped)."""
    m = load_marker(session_crop_dir(cfg, session)) or {}
    return {r["scan"]: r for r in m.get("scans", [])}
