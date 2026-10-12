"""Decode a recording once into a per-scan frame cache, and map rs_marker's
frame numbers onto the recording.

Cache (cache/<scan_key>/):
    depth.zbin   aligned depth frames (uint16, H x W), each zlib-compressed,
                 concatenated; offsets in meta.json
    meta.json    stream intrinsics, depth units, per-frame metadata, the
                 rs_marker <-> recording frame map and the candidate frames

Why a frame map: the recording holds every frame the camera produced
(~30 fps, ~245 frames), but rs_marker only processed ~90 (detection and
display run at ~9 fps), and its frame numbers (in scan_info.txt,
*_detections.csv and imu_*.csv) count those. The anchor is rs_marker's best
frame: the recorded frame whose unfiltered valid-pixel count equals
scan_info's best_valid_pixels (exact with the C++ decoder). The other
rs_marker frames are placed by their timestamps in *_detections.csv relative
to it.
"""

from __future__ import annotations

import csv
import json
import os
import subprocess
import zlib
from pathlib import Path

import numpy as np

from .config import DECODER_BIN
from .discover import Scan, read_scan_info

CACHE_VERSION = 3


class DecodeError(RuntimeError):
    pass


# --- decoding -------------------------------------------------------------------

def _decode_cpp(bag: Path, workdir: Path):
    meta_path = workdir / "decode_meta.jsonl"
    env = dict(os.environ)
    if os.environ.get("RS_DECODE_LIBRARY_PATH"):   # librealsense outside the linker path (decoder only)
        env["LD_LIBRARY_PATH"] = os.environ["RS_DECODE_LIBRARY_PATH"]

    # librealsense >= 2.58 chooses the reader by extension: *.db3 -> ROS2 sqlite.
    # These are ROS1 bags, so hand the decoder a .bag-named symlink.
    link = workdir / "recording.bag"
    link.unlink(missing_ok=True)
    link.symlink_to(Path(bag).resolve())
    try:
        proc = subprocess.Popen([str(DECODER_BIN), str(link), str(meta_path)],
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
        raw = proc.stdout.read()
        err = proc.stderr.read().decode(errors="replace")
        rc = proc.wait()
    finally:
        link.unlink(missing_ok=True)
    if rc != 0:
        raise DecodeError(f"rs_decode failed: {err.strip()}")

    lines = [json.loads(l) for l in meta_path.read_text().splitlines() if l.strip()]
    meta_path.unlink()
    stream, frames = lines[0]["stream"], lines[1:]
    w, h = stream["width"], stream["height"]
    depth = np.frombuffer(raw, dtype="<u2")
    if depth.size != len(frames) * w * h:
        raise DecodeError(f"rs_decode: {depth.size} values for {len(frames)} frames of {w}x{h}")
    return stream, frames, depth.reshape(len(frames), h, w)


def _decode_python(bag: Path, workdir: Path):
    import pyrealsense2 as rs
    # librealsense >= 2.58 opens *.db3 as a ROS2 sqlite bag; these are ROS1 bags.
    link = workdir / "recording.bag"
    link.unlink(missing_ok=True)
    link.symlink_to(Path(bag).resolve())
    cfg = rs.config()
    cfg.enable_device_from_file(str(link), repeat_playback=False)
    cfg.enable_stream(rs.stream.depth)
    cfg.enable_stream(rs.stream.color)
    pipe = rs.pipeline()
    profile = pipe.start(cfg)
    dev = profile.get_device()
    dev.as_playback().set_real_time(False)
    ds = dev.first_depth_sensor()
    opt = lambda o: float(ds.get_option(o)) if ds.supports(o) else -1.0
    align = rs.align(rs.stream.color)
    md = rs.frame_metadata_value
    meta = lambda f, v: int(f.get_frame_metadata(v)) if f.supports_frame_metadata(v) else -1
    stream, frames, depth = None, [], []
    try:
        while True:
            ok, fs = pipe.try_wait_for_frames(3000)
            if not ok:
                break
            d = align.process(fs).get_depth_frame()
            if not d:
                continue
            if stream is None:
                i = d.get_profile().as_video_stream_profile().get_intrinsics()
                stream = {"width": i.width, "height": i.height, "fx": i.fx, "fy": i.fy, "ppx": i.ppx,
                          "ppy": i.ppy, "model": str(i.model).split(".")[-1], "coeffs": list(i.coeffs),
                          "depth_units": ds.get_depth_scale(),
                          "stereo_baseline_mm": opt(rs.option.stereo_baseline),
                          "hdr_enabled": opt(rs.option.hdr_enabled), "decoder": "pyrealsense2"}
            a = np.asanyarray(d.get_data()).copy()
            frames.append({"i": len(frames), "frame_number": d.get_frame_number(),
                           "timestamp_ms": d.get_timestamp(), "valid": int((a > 0).sum()),
                           "sequence_id": meta(d, md.sequence_id), "sequence_size": meta(d, md.sequence_size),
                           "sequence_name": meta(d, md.sequence_name)})
            depth.append(a)
    finally:
        pipe.stop()
        link.unlink(missing_ok=True)
    if stream is None:
        raise DecodeError(f"no depth frames in {bag}")
    return stream, frames, np.stack(depth)


def decoder_kind(setting: str) -> str:
    if setting == "cpp" or (setting == "auto" and DECODER_BIN.is_file()):
        if not DECODER_BIN.is_file():
            raise DecodeError(f"decoder 'cpp' requested but {DECODER_BIN} is not built (see README)")
        return "cpp"
    return "python"


# --- rs_marker <-> recording frame map -------------------------------------------

def _rs_timestamps(det_path: Path, n_frames: int):
    first = {}
    with open(det_path, newline="") as f:
        for row in csv.DictReader(f):
            try:
                first.setdefault(int(row["frame"]), float(row["timestamp_ms"]))
            except (KeyError, ValueError):
                continue
    if len(first) < 2:
        raise DecodeError(f"{det_path.name}: too few timestamped frames to map rs_marker frames")
    k = np.array(sorted(first))
    t = np.array([first[i] for i in k])
    # Frames without any detection have no row; their times are interpolated
    # (and extrapolated at the ends with the mean frame period).
    ts = np.interp(np.arange(n_frames), k, t)
    period = (t[-1] - t[0]) / max(k[-1] - k[0], 1)
    ts[np.arange(n_frames) < k[0]] = t[0] - (k[0] - np.arange(n_frames)[np.arange(n_frames) < k[0]]) * period
    ts[np.arange(n_frames) > k[-1]] = t[-1] + (np.arange(n_frames)[np.arange(n_frames) > k[-1]] - k[-1]) * period
    return ts


def frame_map(scan: Scan, frames: list, depth=None, depth_units=None) -> dict:
    info = read_scan_info(scan.scan_info)
    n_rs = int(info["frames_captured"])
    best_rs = int(info["best_frame"])
    best_valid = int(info["best_valid_pixels"].split("/")[0])
    lo, _, hi = info.get("best_range", "0:end").partition(":")
    lo = int(lo or 0)
    hi = n_rs - 1 if hi in ("", "end") else min(int(hi), n_rs - 1)
    duration_ms = float(info["scan_duration_s"]) * 1000.0

    rs_ts = _rs_timestamps(scan.detections, n_rs)
    bag_ts = np.array([f["timestamp_ms"] for f in frames])
    valid = np.array([f["valid"] for f in frames])

    # Fallback estimate: the recording stops right after scan_info is written,
    # ~scan_duration_s after rs_marker's clock started.
    end_guess = bag_ts[-1] - (duration_ms - rs_ts[best_rs])
    hits = np.flatnonzero(valid == best_valid)
    if len(hits):
        anchor = int(hits[np.argmin(np.abs(bag_ts[hits] - end_guess))])
        method = "exact valid-pixel match" + (f" ({len(hits)} candidates)" if len(hits) > 1 else "")
    else:
        anchor = int(np.argmin(np.abs(bag_ts - end_guess)))
        method = "timing estimate (no exact valid-pixel match - decoder differs from rs_marker's)"

    offset = bag_ts[anchor] - rs_ts[best_rs]
    predicted = [int(np.argmin(np.abs(bag_ts - (rs_ts[k] + offset)))) for k in range(n_rs)]

    # Timing alone is +-1 frame (processing latency jitters, and rs_marker sees
    # every 2nd-3rd recorded frame). Each detection row also holds the
    # unfiltered depth at the tag centre pixel, read from that very frame
    # (rs_marker: get_distance((int)cx, (int)cy), printed to 1 mm), so the
    # recorded frame near the prediction whose depths match is the one.
    # rs_marker never sees a frame twice, so the map must be strictly
    # increasing: the best-scoring increasing path through each frame's
    # +-SEARCH window (dynamic programming), with the anchor fixed.
    rs_to_bag, matched = _match_by_depth(predicted, anchor, best_rs, _detection_depths(scan.detections),
                                         depth, float(depth_units) if depth_units else 1e-4, len(frames))
    candidates = sorted(set(rs_to_bag[lo:hi + 1]))
    return {
        "rs_frames": n_rs, "rs_best_frame": best_rs, "rs_best_valid_pixels": best_valid,
        "rs_best_range": [lo, hi], "anchor_bag_index": anchor, "anchor_method": method,
        "anchor_valid_pixels": int(valid[anchor]),
        "end_guess_error_ms": round(float(bag_ts[anchor] - end_guess), 1),
        "rs_timestamps_ms": [round(float(t), 3) for t in rs_ts], "rs_to_bag": rs_to_bag,
        "rs_to_bag_by_time": predicted, "rs_frames_depth_matched": matched,
        "bag_offset_ms": float(offset), "candidates": candidates,
    }


SEARCH = 5   # recorded frames either side of the timing prediction


def _match_by_depth(predicted, anchor, best_rs, dets, depth, units, n_bag):
    """Returns (rs_to_bag, number of rs frames whose every tag depth matched)."""
    n = len(predicted)
    windows, scores = [], []
    for k, p in enumerate(predicted):
        if k == best_rs:
            win = [anchor]
        else:
            win = list(range(max(0, p - SEARCH), min(n_bag, p + SEARCH + 1)))
        pts = dets.get(k)
        sc = []
        for j in win:
            if pts is None or depth is None:
                s, full = 0.0, False
            else:
                h, w = depth[j].shape
                xs = np.clip(pts[:, 0].astype(int), 0, w - 1)
                ys = np.clip(pts[:, 1].astype(int), 0, h - 1)
                # rs_marker printed float32 metres with %.3f: within half a mm,
                # inclusive (ties like 0.8635 round either way)
                hit = np.abs(depth[j][ys, xs] * units - pts[:, 2]) <= 5e-4 + 1e-6
                s, full = float(hit.sum()), bool(hit.all())
            # tiny preference for the timing prediction breaks ties
            sc.append((s + (1000.0 if k == best_rs else 0.0) - 1e-3 * abs(j - p), full))
        windows.append(win)
        scores.append(sc)

    # Viterbi over strictly increasing bag indices
    best = [np.array([s for s, _ in scores[0]])]
    back = [np.full(len(windows[0]), -1)]
    for k in range(1, n):
        prev_w, prev_b = np.array(windows[k - 1]), best[-1]
        cur, bk = [], []
        for j, (s, _) in zip(windows[k], scores[k]):
            ok = prev_w < j
            if ok.any():
                i = int(np.argmax(np.where(ok, prev_b, -np.inf)))
                cur.append(prev_b[i] + s)
                bk.append(i)
            else:
                cur.append(-np.inf)
                bk.append(-1)
        best.append(np.array(cur))
        back.append(np.array(bk))
    path = [int(np.argmax(best[-1]))]
    for k in range(n - 1, 0, -1):
        path.append(int(back[k][path[-1]]))
    path.reverse()
    if any(i < 0 for i in path) or not np.isfinite(best[-1][path[-1]]):
        return list(predicted), 0      # no increasing path in the windows: fall back to timing
    rs_to_bag = [windows[k][i] for k, i in enumerate(path)]
    matched = sum(scores[k][i][1] or k == best_rs for k, i in enumerate(path))
    return rs_to_bag, matched


def _detection_depths(det_path: Path) -> dict:
    """{rs frame: (n, 3) array of (int cx, int cy, depth_m)} for detections with a depth."""
    out = {}
    with open(det_path, newline="") as f:
        for row in csv.DictReader(f):
            try:
                k, x, y, d = int(row["frame"]), int(float(row["cx"])), int(float(row["cy"])), float(row["depth_m"])
            except (KeyError, ValueError):
                continue
            out.setdefault(k, []).append((x, y, d))
    return {k: np.array(v) for k, v in out.items()}


def rs_frame_for(meta: dict, bag_index: int) -> int:
    """rs_marker frame number nearest in time to a recorded frame (for the
    cropper, which looks tags up by rs_marker frame)."""
    t = meta["frames"][bag_index]["timestamp_ms"] - meta["map"]["bag_offset_ms"]
    return int(np.argmin(np.abs(np.asarray(meta["map"]["rs_timestamps_ms"]) - t)))


# --- cache -------------------------------------------------------------------------

def _source_stamp(scan: Scan, kind: str, warmup) -> dict:
    st = scan.bag.stat()
    return {"bag": str(scan.bag), "size": st.st_size, "mtime_ns": st.st_mtime_ns, "decoder": kind,
            "warmup_frames": warmup, "cache_version": CACHE_VERSION}


def cache_dir_for(cache_root: Path, scan: Scan) -> Path:
    return Path(cache_root) / scan.key


def ensure_cache(scan: Scan, cache_root: Path, decoder_setting: str, warmup_frames=None, log=print) -> dict:
    """Decode the scan into the cache unless an up-to-date cache exists.
    Returns the cache's meta dict."""
    kind = decoder_kind(decoder_setting)
    cdir = cache_dir_for(cache_root, scan)
    meta_path = cdir / "meta.json"
    stamp = _source_stamp(scan, kind, warmup_frames)
    if meta_path.is_file() and (cdir / "depth.zbin").is_file():
        meta = json.loads(meta_path.read_text())
        if meta.get("source") == stamp:
            return meta
    cdir.mkdir(parents=True, exist_ok=True)
    log(f"  decoding {scan.key} ({kind}) ...")
    stream, frames, depth = (_decode_cpp if kind == "cpp" else _decode_python)(scan.bag, cdir)
    fmap = frame_map(scan, frames, depth, stream["depth_units"])

    last = fmap["candidates"][-1]
    first = 0 if warmup_frames is None else max(0, fmap["candidates"][0] - int(warmup_frames))
    offsets, pos = {}, 0
    tmp = cdir / "depth.zbin.tmp"
    with open(tmp, "wb") as f:
        for i in range(first, last + 1):
            blob = zlib.compress(np.ascontiguousarray(depth[i]).tobytes(), 3)
            f.write(blob)
            offsets[i] = [pos, len(blob)]
            pos += len(blob)
    os.replace(tmp, cdir / "depth.zbin")

    seq_sizes = {f["sequence_size"] for f in frames}
    hdr = stream.get("hdr_enabled", 0) >= 1 and min(seq_sizes) >= 2
    meta = {"source": stamp, "scan": scan.meta(), "stream": stream, "frames": frames, "map": fmap,
            "stored": [first, last], "offsets": {str(k): v for k, v in offsets.items()},
            "hdr_available": bool(hdr),
            "hdr_note": ("HDR sequence metadata present" if hdr else
                         f"no HDR: hdr_enabled={stream.get('hdr_enabled')}, sequence_size in frames "
                         f"{sorted(seq_sizes)}")}
    tmp = meta_path.with_suffix(".tmp")
    tmp.write_text(json.dumps(meta))
    os.replace(tmp, meta_path)
    log(f"    {len(frames)} frames, kept {first}..{last} ({pos / 1e6:.0f} MB), anchor bag frame "
        f"{fmap['anchor_bag_index']} = rs_marker frame {fmap['rs_best_frame']} ({fmap['anchor_method']}), "
        f"{len(fmap['candidates'])} candidate frames {fmap['candidates'][0]}..{fmap['candidates'][-1]}; "
        f"{fmap['rs_frames_depth_matched']}/{fmap['rs_frames']} rs_marker frames matched by tag depths")
    return meta


class FrameCache:
    """Read access to one scan's cached frames."""

    def __init__(self, cache_root: Path, scan_key: str):
        self.dir = Path(cache_root) / scan_key
        self.meta = json.loads((self.dir / "meta.json").read_text())
        s = self.meta["stream"]
        self.shape = (s["height"], s["width"])
        self._f = open(self.dir / "depth.zbin", "rb")

    def frame(self, i: int) -> np.ndarray:
        pos, n = self.meta["offsets"][str(i)]
        self._f.seek(pos)
        return np.frombuffer(zlib.decompress(self._f.read(n)), dtype="<u2").reshape(self.shape)

    @property
    def stored(self):
        return range(self.meta["stored"][0], self.meta["stored"][1] + 1)

    def close(self):
        self._f.close()
