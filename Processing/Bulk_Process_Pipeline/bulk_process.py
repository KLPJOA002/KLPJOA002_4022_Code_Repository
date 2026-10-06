#!/usr/bin/env python3
"""
Bulk crop + fuse every scan session found under a source directory.

For each session (<Session>/<Session>_r<deg>/<Session>_r<deg>_a<deg>/*.ply):

  1. Crop  - every scan gets the automatic AprilTag crop from
             ../Crop/crop_pointclouds.py (imported, not modified), written
             like that tool writes it: <scan>_cropped.ply plus the IMU file,
             tag sidecar, detections and scan_info.txt that fuse_scans reads.
  2. Fuse  - ../Fuse/build/fuse_scans runs headless (--no-gui) on the cropped
             session with the fusion parameters from --config and/or the
             arguments after "--".

Output layout (<out> is the output directory you give):

  <out>/fused/<rel>/<Session>.ply      the fused scan of each session
  <out>/cropped/<rel>/<Session>/...    the cropped scans (reused on re-runs)
  <out>/logs/<rel>/<Session>_fuse.log  full fuse_scans output per session
  <out>/summary.csv                    one row per session
  <out>/run_config.json                the settings this run used

<rel> is the session's parent folder relative to the source directory, so
sessions from different objects/distances don't overwrite each other.

Usage:
  python3 bulk_process.py <source_dir> <output_dir> [options] [-- fuse_scans args...]

  e.g.
  python3 bulk_process.py /data/3D_Error/Small_Pyramid out/ --config my_pipeline.toml
  python3 bulk_process.py /data/3D_Error out/ -- --tag-align --fusion-cell 0.002

Run it with the Crop tool's venv (it needs open3d + numpy):
  ../Crop/cropper/bin/python bulk_process.py ...
"""

from __future__ import annotations

import argparse
import csv
import fnmatch
import json
import re
import shlex
import shutil
import subprocess
import sys
import time
import tomllib
import types
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import open3d as o3d

HERE = Path(__file__).resolve().parent
PROCESSING = HERE.parent
CROP_DIR = PROCESSING / "Crop"
DEFAULT_FUSE_BIN = PROCESSING / "Fuse" / "build" / "fuse_scans"

if str(CROP_DIR) not in sys.path:
    sys.path.insert(0, str(CROP_DIR))
import crop_pointclouds as crop_tool  # noqa: E402

ROTATION_RE = re.compile(r"_r(-?\d+)(?:_|$)")
CROP_SETTINGS_FILE = "crop_settings.json"
# Bump when the crop step changes in a way that should invalidate cached crops.
CROP_VERSION = 1


# ─────────────────────────────────────────────────────────────────────────────
#  Session discovery
# ─────────────────────────────────────────────────────────────────────────────

def _rotation_dirs(d: Path):
    return [c for c in d.iterdir() if c.is_dir() and ROTATION_RE.search(c.name)]


def is_session(d: Path) -> bool:
    """A session folder holds rotation subfolders (..._r<deg>) that hold tilt
    subfolders with a .ply in them."""
    for rot in _rotation_dirs(d):
        for tilt in rot.iterdir():
            if tilt.is_dir() and any(tilt.glob("*.ply")):
                return True
    return False


def find_sessions(source: Path) -> list[Path]:
    """Every session folder at or below source. Doesn't descend into a
    session once found."""
    if is_session(source):
        return [source]
    sessions = []
    for child in sorted(p for p in source.iterdir() if p.is_dir()):
        sessions.extend(find_sessions(child))
    return sessions


def session_scans(session: Path) -> list[Path]:
    """The scan PLYs of one session, one per tilt folder (preferring
    <tilt folder>.ply, as fuse_scans does)."""
    scans = []
    for rot in sorted(_rotation_dirs(session)):
        for tilt in sorted(t for t in rot.iterdir() if t.is_dir()):
            plys = sorted(tilt.glob("*.ply"))
            if not plys:
                continue
            exact = tilt / f"{tilt.name}.ply"
            if exact in plys:
                scans.append(exact)
            elif len(plys) == 1:
                scans.append(plys[0])
            else:
                print(f"    [!] {tilt.relative_to(session)}: {len(plys)} .ply files and none "
                      f"named {exact.name}; skipping this folder")
    return scans


# ─────────────────────────────────────────────────────────────────────────────
#  Settings
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class CropSettings:
    margin: float = 0.0          # px the crop is pulled in from the tags' inner corners
    frame_window: int = 15       # detections this many frames either side fill missing tags
    tag_order: list | None = None  # clockwise corner tag IDs; None = learned per session
    on_fail: str = "skip"        # auto crop impossible: skip | keep | manual
    review: bool = False         # show the crop tool's preview window for every scan
    suffix: str = "_cropped"

    def cache_key(self) -> dict:
        # review/on_fail=manual are hand-made crops; still keyed so a change re-crops.
        return {"version": CROP_VERSION, "margin": self.margin, "frame_window": self.frame_window,
                "tag_order": self.tag_order, "on_fail": self.on_fail, "review": self.review,
                "suffix": self.suffix}


@dataclass
class FuseSettings:
    binary: Path = DEFAULT_FUSE_BIN
    gui: bool = False
    params: dict = field(default_factory=dict)  # flag name (no "--") -> value
    extra: list = field(default_factory=list)   # raw args after "--" on the command line
    config_dir: Path = HERE                     # relative paths in params resolve here

    def args(self) -> list[str]:
        """The fuse_scans flags for these settings. A true value is a bare
        flag, false is left out, a list repeats the flag."""
        out = []
        for key, value in self.params.items():
            flag = "--" + key.lstrip("-")
            if key == "rig-config":
                value = str((self.config_dir / value).resolve())
            if value is True:
                out.append(flag)
            elif value is False:
                continue
            elif isinstance(value, list):
                for v in value:
                    out += [flag, str(v)]
            else:
                out += [flag, str(value)]
        if not self.gui:
            out.append("--no-gui")
        return out + list(self.extra)


def load_config(path: Path | None):
    """[crop] and [fuse] tables from a TOML file (see pipeline_config.example.toml)."""
    crop, fuse = CropSettings(), FuseSettings()
    if path is None:
        return crop, fuse
    with open(path, "rb") as f:
        cfg = tomllib.load(f)
    unknown = set(cfg) - {"crop", "fuse"}
    if unknown:
        sys.exit(f"Error: unknown section(s) in {path}: {', '.join(sorted(unknown))}")

    for key, value in cfg.get("crop", {}).items():
        attr = key.replace("-", "_")
        if not hasattr(crop, attr):
            sys.exit(f"Error: unknown [crop] setting '{key}' in {path}")
        setattr(crop, attr, value)

    fuse_cfg = dict(cfg.get("fuse", {}))
    fuse.config_dir = path.resolve().parent
    if "binary" in fuse_cfg:
        fuse.binary = (fuse.config_dir / fuse_cfg.pop("binary")).resolve()
    fuse.gui = bool(fuse_cfg.pop("gui", False))
    fuse.params = fuse_cfg.pop("params", {})
    if fuse_cfg:
        sys.exit(f"Error: unknown [fuse] setting(s) in {path}: {', '.join(fuse_cfg)} "
                 f"(fusion flags go under [fuse.params])")
    return crop, fuse


# ─────────────────────────────────────────────────────────────────────────────
#  Crop step
# ─────────────────────────────────────────────────────────────────────────────

def crop_one(ply: Path, session: Path, layout, settings: CropSettings):
    """Returns (original cloud, cloud to save or None to skip it, status, note)."""
    pcd = o3d.io.read_point_cloud(str(ply))
    if len(pcd.points) == 0:
        return pcd, None, "skipped", "empty cloud"
    args = types.SimpleNamespace(frame_window=settings.frame_window, margin=settings.margin)
    try:
        auto = crop_tool.compute_auto_crop(pcd, ply, session, layout, args)
    except crop_tool.AutoCropError as exc:
        reason = str(exc)
        if settings.on_fail == "keep":
            return pcd, pcd, "uncropped", reason
        if settings.on_fail == "manual":
            return pcd, crop_tool.manual_crop(pcd, ply), "manual", reason
        return pcd, None, "skipped", reason

    for note in auto.notes:
        print(f"      {note}")
    cropped = pcd.select_by_index(np.flatnonzero(auto.mask))
    note = f"kept {int(auto.mask.sum())}/{len(auto.mask)} points"
    if not settings.review:
        return pcd, cropped, "auto", note

    choice = crop_tool.preview_auto_crop(
        pcd, auto, window_name=f"Auto crop - {ply.name} (Enter=accept, M=manual, N=keep original, Esc=quit all)")
    if choice == "quit":
        raise crop_tool.QuitAll()
    if choice == "keep":
        return pcd, pcd, "uncropped", "kept original in review"
    if choice == "manual":
        box = cropped.get_axis_aligned_bounding_box()
        return pcd, crop_tool.manual_crop(pcd, ply, box.min_bound, box.max_bound), "manual", "re-cropped in review"
    return pcd, cropped, "auto", note


def crop_session(session: Path, out_dir: Path, settings: CropSettings, force: bool) -> list[dict]:
    """Crop every scan of a session into out_dir (mirrored layout). Reuses a
    previous complete crop made with the same settings."""
    marker = out_dir / CROP_SETTINGS_FILE
    if not force and marker.is_file():
        try:
            previous = json.loads(marker.read_text())
        except json.JSONDecodeError:
            previous = {}
        if previous.get("settings") == settings.cache_key() and previous.get("complete"):
            print(f"  Crop: reusing {out_dir} (same settings; --recrop to redo)")
            return previous["scans"]
    if out_dir.exists():
        # Only clear folders this pipeline made: --cropped-dir may point at hand-made crops.
        if not marker.is_file() and any(out_dir.iterdir()):
            raise RuntimeError(f"{out_dir} exists and wasn't written by this pipeline; "
                               f"move it or choose another --cropped-dir")
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True)

    layout = crop_tool.learn_tag_layout(session, settings.tag_order)
    if layout.order:
        print(f"  Corner tag order (clockwise): {layout.order}")
    else:
        print("  No frame with all four tags decoded; tag positions will be guessed from geometry.")

    scans = session_scans(session)
    results = []
    for i, ply in enumerate(scans, start=1):
        rel = ply.relative_to(session)
        print(f"  [{i}/{len(scans)}] {rel}")
        original, cloud, status, note = crop_one(ply, session, layout, settings)
        row = {"scan": str(rel), "status": status, "note": note, "points": 0}
        if cloud is not None:
            out_path = out_dir / rel.with_name(rel.stem + settings.suffix + ".ply")
            out_path.parent.mkdir(parents=True, exist_ok=True)
            crop_tool.write_ply_like_original(out_path, cloud)
            crop_tool.copy_imu_file(ply, out_path.parent)
            # Measured on the original cloud: the crop has cut the tags away.
            crop_tool.write_tag_sidecar(ply, original, session, out_path.parent)
            crop_tool.copy_scan_metadata(ply, out_path.parent)
            row["points"] = len(cloud.points)
        print(f"      -> {status}: {note}")
        results.append(row)

    marker.write_text(json.dumps({"settings": settings.cache_key(), "complete": True,
                                  "source": str(session), "scans": results}, indent=2))
    return results


# ─────────────────────────────────────────────────────────────────────────────
#  Fuse step
# ─────────────────────────────────────────────────────────────────────────────

def fuse_session(cropped_dir: Path, out_ply: Path, log_path: Path, fuse: FuseSettings, verbose: bool):
    """Run fuse_scans on one cropped session. Returns (ok, message)."""
    cmd = [str(fuse.binary), "-o", str(out_ply), "--sessions", str(cropped_dir)] + fuse.args()
    out_ply.parent.mkdir(parents=True, exist_ok=True)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    print(f"  Fuse: {shlex.join(cmd)}")
    print(f"        log -> {log_path}")

    tail = []
    with open(log_path, "w") as log:
        log.write("$ " + shlex.join(cmd) + "\n\n")
        log.flush()
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        for line in proc.stdout:
            log.write(line)
            tail = (tail + [line.rstrip()])[-15:]
            if verbose:
                print("    | " + line.rstrip())
        code = proc.wait()

    if code == 0 and out_ply.is_file():
        return True, "ok"
    if not verbose:
        print("    Last lines of fuse_scans output:")
        for line in tail:
            print("    | " + line)
    if out_ply.exists() and code != 0:
        out_ply.unlink()  # don't leave a result that looks valid
    return False, f"fuse_scans exited with code {code}"


# ─────────────────────────────────────────────────────────────────────────────
#  Main
# ─────────────────────────────────────────────────────────────────────────────

def parse_args():
    argv = sys.argv[1:]
    extra = []
    if "--" in argv:
        split = argv.index("--")
        argv, extra = argv[:split], argv[split + 1:]

    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("source", type=Path, help="Directory holding the raw sessions (searched recursively), "
                                             "or a single session folder")
    p.add_argument("output", type=Path, help="Where fused/, cropped/, logs/ and summary.csv are written")
    p.add_argument("--config", type=Path, help="TOML file with [crop] and [fuse] settings "
                                               "(see pipeline_config.example.toml)")
    p.add_argument("--sessions", help="Only process sessions whose name matches one of these "
                                      "comma-separated glob patterns, e.g. '20261005_16*'")
    p.add_argument("--cropped-dir", type=Path, help="Keep the cropped scans here instead of <output>/cropped "
                                                    "(share one crop cache between several fusion runs)")
    g = p.add_argument_group("crop (override the config file)")
    g.add_argument("--crop-margin", type=float, help="Pixels the crop is pulled in from the tags' inner corners")
    g.add_argument("--frame-window", type=int, help="Detections this many frames either side fill missing tags")
    g.add_argument("--tag-order", type=lambda s: [int(v) for v in s.split(",")],
                   help="Clockwise corner tag IDs, e.g. 6,5,2,4 (default: learned per session)")
    g.add_argument("--on-crop-fail", choices=["skip", "keep", "manual"],
                   help="When the automatic crop isn't possible: leave the scan out of the fusion (skip, "
                        "default), fuse it uncropped (keep), or open the manual crop window (manual)")
    g.add_argument("--review-crops", action="store_true", help="Show the crop preview window for every scan")
    g.add_argument("--recrop", action="store_true", help="Crop again even when matching cropped scans exist")
    g = p.add_argument_group("fuse (override the config file)")
    g.add_argument("--fuse-bin", type=Path, help=f"fuse_scans executable (default {DEFAULT_FUSE_BIN})")
    g.add_argument("--fuse-gui", action="store_true", help="Open fuse_scans' viewer for each session "
                                                           "instead of running headless")
    g.add_argument("--crop-only", action="store_true", help="Crop, don't fuse")
    g.add_argument("--skip-fused", action="store_true", help="Don't re-fuse sessions whose fused PLY exists")
    p.add_argument("--dry-run", action="store_true", help="List the sessions and the fuse command, do nothing")
    p.add_argument("-v", "--verbose", action="store_true", help="Echo fuse_scans' output to the console")
    args = p.parse_args(argv)
    return args, extra


def main():
    args, extra = parse_args()
    source, output = args.source.resolve(), args.output.resolve()
    if not source.is_dir():
        sys.exit(f"Error: source directory does not exist: {source}")
    if source == output or source in output.parents:
        sys.exit("Error: the output directory must not be inside the source directory")

    crop, fuse = load_config(args.config)
    for attr, value in [("margin", args.crop_margin), ("frame_window", args.frame_window),
                        ("tag_order", args.tag_order), ("on_fail", args.on_crop_fail)]:
        if value is not None:
            setattr(crop, attr, value)
    crop.review = crop.review or args.review_crops
    if crop.on_fail not in ("skip", "keep", "manual"):
        sys.exit(f"Error: [crop] on_fail must be skip, keep or manual (got '{crop.on_fail}')")
    if crop.tag_order is not None and len(crop.tag_order) != 4:
        sys.exit("Error: tag order needs exactly 4 IDs")
    if args.fuse_bin:
        fuse.binary = args.fuse_bin.resolve()
    fuse.gui = fuse.gui or args.fuse_gui
    fuse.extra = extra
    if not args.crop_only and not fuse.binary.is_file():
        sys.exit(f"Error: fuse_scans not found at {fuse.binary} (build ../Fuse, or pass --fuse-bin)")

    sessions = find_sessions(source)
    if args.sessions:
        patterns = [s.strip() for s in args.sessions.split(",") if s.strip()]
        sessions = [s for s in sessions if any(fnmatch.fnmatch(s.name, pat) for pat in patterns)]
    if not sessions:
        sys.exit(f"No scan sessions found under {source}")

    cropped_root = (args.cropped_dir.resolve() if args.cropped_dir else output / "cropped")
    print(f"Found {len(sessions)} session(s) under {source}")
    print(f"Fuse arguments: {shlex.join(fuse.args())}")
    if args.dry_run:
        for s in sessions:
            rel = s.relative_to(source) if s != source else Path(s.name)
            print(f"  {rel}  ({len(session_scans(s))} scans) -> {output / 'fused' / rel.parent / (s.name + '.ply')}")
        return

    output.mkdir(parents=True, exist_ok=True)
    (output / "run_config.json").write_text(json.dumps({
        "source": str(source), "started": time.strftime("%Y-%m-%d %H:%M:%S"),
        "crop": crop.cache_key(), "cropped_dir": str(cropped_root),
        "fuse_binary": str(fuse.binary), "fuse_args": fuse.args(),
        "config_file": str(args.config.resolve()) if args.config else None,
    }, indent=2))

    summary_rows = []
    summary_path = output / "summary.csv"
    for n, session in enumerate(sessions, start=1):
        rel_parent = session.parent.relative_to(source) if session != source else Path()
        label = str(rel_parent / session.name)
        print(f"\n{'=' * 78}\n[{n}/{len(sessions)}] Session {label}\n{'=' * 78}")
        started = time.time()
        row = {"session": label, "scans": 0, "cropped_auto": 0, "cropped_manual": 0, "uncropped": 0,
               "skipped": 0, "fused": "", "status": "", "seconds": 0, "skipped_scans": ""}

        cropped_dir = cropped_root / rel_parent / session.name
        try:
            scans = crop_session(session, cropped_dir, crop, args.recrop)
        except crop_tool.QuitAll:
            print("\nQuit requested in a crop window. Stopping.")
            break
        except RuntimeError as exc:
            print(f"  Error: {exc}")
            scans = []
            row["status"] = f"not cropped: {exc}"
        row["scans"] = len(scans)
        for s in scans:
            key = {"auto": "cropped_auto", "manual": "cropped_manual"}.get(s["status"], s["status"])
            row[key] += 1
        row["skipped_scans"] = "; ".join(f"{s['scan']} ({s['note']})" for s in scans if s["status"] == "skipped")
        usable = len(scans) - row["skipped"]

        if row["status"]:
            pass  # the crop step already failed
        elif args.crop_only:
            row["status"] = "cropped"
        elif usable < 2:
            row["status"] = f"not fused: only {usable} usable scan(s), fuse_scans needs 2"
            print(f"  {row['status']}")
        else:
            out_ply = output / "fused" / rel_parent / f"{session.name}.ply"
            if args.skip_fused and out_ply.is_file():
                ok, msg = True, "existing fused scan kept (--skip-fused)"
            else:
                ok, msg = fuse_session(cropped_dir, out_ply, output / "logs" / rel_parent / f"{session.name}_fuse.log",
                                       fuse, args.verbose)
            row["status"] = msg
            if ok:
                row["fused"] = str(out_ply.relative_to(output))
                print(f"  Fused -> {out_ply}")
            else:
                print(f"  FAILED: {msg}")

        row["seconds"] = round(time.time() - started, 1)
        summary_rows.append(row)
        # Rewritten after every session so a long run that dies part way still has one.
        with open(summary_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(row))
            writer.writeheader()
            writer.writerows(summary_rows)

    fused = sum(1 for r in summary_rows if r["fused"])
    print(f"\nDone: {fused}/{len(summary_rows)} session(s) fused into {output / 'fused'}")
    for r in summary_rows:
        if not r["fused"] and not args.crop_only:
            print(f"  not fused: {r['session']}: {r['status']}")
    print(f"Summary: {summary_path}")


if __name__ == "__main__":
    main()
