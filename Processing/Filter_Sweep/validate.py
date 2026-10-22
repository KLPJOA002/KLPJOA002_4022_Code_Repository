#!/usr/bin/env python3
"""Checks that the sweep reproduces rs_marker and that its filters are the
real librealsense ones. Run on any machine before trusting a sweep there.

1. Baseline reproduction (per scan): with no filters, the selected frame must
   be rs_marker's own best frame, and its point cloud must match the .ply
   rs_marker saved.
2. Filter equivalence: the Python filter path (pyrealsense2 software device on
   the cached frames) against decoder/rs_decode --filters (the system
   librealsense, i.e. what rs_marker links), frame by frame.
3. History independence: spatial holes_fill configs give bit-identical output
   whatever ran before them on the same software sensor, as in a sweep worker
   (see filter_sweep/memory.py for why this can fail).

  validate.py --scans Small_Wave__863mm__A1,Big_Wave__863mm__A1
"""

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from filter_sweep.memory import ensure_zeroed_malloc  # noqa: E402

ensure_zeroed_malloc()   # as run_sweep.py, so this validates what the sweep runs

import numpy as np  # noqa: E402

from filter_sweep.config import DECODER_BIN, chain_spec, config_label, load_settings, normalise  # noqa: E402
from filter_sweep.discover import discover  # noqa: E402
from filter_sweep.filters import Chain, SoftwareDepth, run_config  # noqa: E402
from filter_sweep.frames import FrameCache, ensure_cache  # noqa: E402

CHECK_CONFIGS = [
    {"spatial": {}},
    {"temporal": {"alpha": 0.2, "persistence": 3}},
    {"decimation": {"magnitude": 3}, "spatial": {"magnitude": 3, "alpha": 0.4, "holes_fill": 2},
     "temporal": {}, "hole_filling": {"mode": 1}},
]


def check_baseline(cache, s) -> bool:
    import open3d as o3d
    from scipy.spatial import cKDTree
    m = cache.meta
    res = run_config(cache, normalise({}, s.filter_defaults), SoftwareDepth(m["stream"]))
    scan_dir = Path(m["scan"]["scan_dir"])
    orig = np.asarray(o3d.io.read_point_cloud(str(scan_dir / f"{scan_dir.name}.ply")).points)
    pts = res["points"].astype(np.float64)
    same_frame = res["best_bag_index"] == m["map"]["anchor_bag_index"]
    d, _ = cKDTree(orig).query(pts)
    ok = same_frame and len(pts) == len(orig) and d.max() < 1e-5
    print(f"  baseline: selected frame {res['best_bag_index']} (rs_marker's best = "
          f"{m['map']['anchor_bag_index']}, {m['map']['anchor_method']}), {res['best_valid_pixels']} valid px, "
          f"{len(pts)} points vs {len(orig)} in rs_marker's .ply, max distance {d.max() * 1e3:.4f} mm  "
          f"-> {'OK' if ok else 'MISMATCH'}")
    return ok


def check_filters(cache, scan, s) -> bool:
    if not DECODER_BIN.is_file():
        print("  filter equivalence: decoder/build/rs_decode not built, skipped")
        return True
    m = cache.meta
    cands = set(m["map"]["candidates"])
    all_ok = True
    env = dict(os.environ)
    if os.environ.get("RS_DECODE_LIBRARY_PATH"):
        env["LD_LIBRARY_PATH"] = os.environ["RS_DECODE_LIBRARY_PATH"]
    for raw in CHECK_CONFIGS:
        cfg = normalise(raw, s.filter_defaults)
        last = max(cands)
        meta_path = cache.dir / "check_meta.jsonl"
        out = subprocess.run([str(DECODER_BIN), str(scan.bag), str(meta_path), "--max-frames", str(last + 1),
                              "--filters", chain_spec(cfg)], capture_output=True, env=env)
        if out.returncode != 0:
            print(f"  rs_decode failed: {out.stderr.decode()}")
            return False
        lines = [json.loads(l) for l in meta_path.read_text().splitlines()]
        meta_path.unlink()
        st = lines[0]["stream"]
        ref = np.frombuffer(out.stdout, "<u2").reshape(-1, st["height"], st["width"])

        sw = SoftwareDepth(m["stream"])
        chain = Chain(cfg)
        identical, max_diff, n = 0, 0, 0
        for i in cache.stored:
            if i > last:
                break
            fm = m["frames"][i]
            arr = np.ascontiguousarray(cache.frame(i))
            o = np.asanyarray(chain.process(sw.frame(arr, fm["frame_number"], fm["timestamp_ms"])).get_data())
            if i in cands:
                diff = np.abs(o.astype(np.int32) - ref[i].astype(np.int32))
                identical += int((diff == 0).all())
                max_diff = max(max_diff, int(diff.max()))
                n += 1
        sw.close()
        ok = identical == n
        all_ok &= ok
        print(f"  filters [{config_label(cfg)}] -> {o.shape[1]}x{o.shape[0]}: {identical}/{n} candidate frames "
              f"identical to the system librealsense (max diff {max_diff} depth units)  -> {'OK' if ok else 'DIFFERS'}")
    return all_ok


# Configs that leave pixels unwritten (spatial holes_fill), and configs run before them that
# were seen to leave other data in the memory they then get (measured on Small_Wave 701mm A2).
HISTORY_TARGETS = [{"spatial": {"holes_fill": 1}}, {"spatial": {"holes_fill": 3}}, {"spatial": {"holes_fill": 5}}]
HISTORY_BEFORE = [{"decimation": {"magnitude": 4}}, {"decimation": {"magnitude": 8}}, {"temporal": {}}]


def _candidate_outputs(cache, cfg, sw) -> dict:
    m = cache.meta
    chain, out = Chain(cfg), {}
    for i in m["map"]["candidates"]:
        fm = m["frames"][i]
        f = chain.process(sw.frame(np.ascontiguousarray(cache.frame(i)), fm["frame_number"], fm["timestamp_ms"]))
        out[i] = np.asanyarray(f.get_data()).copy()
    return out


def check_history(cache, s) -> bool:
    sw = SoftwareDepth(cache.meta["stream"])
    all_ok = True
    for raw in HISTORY_TARGETS:
        cfg = normalise(raw, s.filter_defaults)
        ref = _candidate_outputs(cache, cfg, sw)
        bad = 0
        for before in HISTORY_BEFORE:
            run_config(cache, normalise(before, s.filter_defaults), sw)
            got = _candidate_outputs(cache, cfg, sw)
            bad += sum(int((got[i] != ref[i]).sum()) for i in ref)
        ok = bad == 0
        all_ok &= ok
        print(f"  history [{config_label(cfg)}] after {len(HISTORY_BEFORE)} other configs: {bad} pixel(s) differ "
              f"from its first run  -> {'OK' if ok else 'DEPENDS ON HISTORY'}")
    sw.close()
    return all_ok


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config")
    p.add_argument("--scans", required=True, help="comma-separated scan keys, e.g. Big_Wave__863mm__A1")
    p.add_argument("--no-filters", action="store_true", help="only the baseline check")
    args = p.parse_args()
    s = load_settings(args.config)
    wanted = args.scans.split(",")
    scans, _ = discover(s.data_root, sorted({k.split("__")[0] for k in wanted}),
                        sorted({k.split("__")[1] for k in wanted}), "all")
    scans = [x for x in scans if x.key in wanted]
    ok = True
    for scan in scans:
        print(f"\n{scan.key}  ({scan.scan_dir.name})")
        ensure_cache(scan, s.cache_dir, s.decoder, s.warmup_frames)
        cache = FrameCache(s.cache_dir, scan.key)
        ok &= check_baseline(cache, s)
        if not args.no_filters:
            ok &= check_filters(cache, scan, s)
            ok &= check_history(cache, s)
    print("\nALL OK" if ok else "\nSOME CHECKS FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
