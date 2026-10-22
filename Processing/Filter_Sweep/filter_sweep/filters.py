"""Apply one RealSense post-processing configuration to a cached scan and
pick its best frame, the way rs_marker does.

Cached frames (already aligned to colour, as rs_marker aligns before
filtering) are injected through a pyrealsense2 software_device, so the real
librealsense filters run on them. Chain order (Intel's recommended order,
with HDR merge after decimation as specified for this study):

    [align depth->colour : done once, in the cache]
    decimation -> HDR merge -> depth->disparity -> spatial -> temporal
               -> disparity->depth -> hole filling

The disparity transforms are only inserted when spatial or temporal is on
(Intel recommends running both in the disparity domain).

Best frame: most valid (non-zero) depth pixels in the filtered output,
among the candidate frames (rs_marker's -b range mapped onto the recording),
ties to the earliest. The temporal filter is stateful, so when it (or HDR
merge) is on every stored frame is fed in recorded order from the start of
the recording; frames before the first candidate are warm-up only.
Otherwise frames are independent and only the candidates are processed.
"""

from __future__ import annotations

import numpy as np
import pyrealsense2 as rs

_DISTORTION = {
    "none": rs.distortion.none, "brown_conrady": rs.distortion.brown_conrady,
    "inverse_brown_conrady": rs.distortion.inverse_brown_conrady,
    "modified_brown_conrady": rs.distortion.modified_brown_conrady,
    "ftheta": rs.distortion.ftheta, "kannala_brandt4": rs.distortion.kannala_brandt4,
}


class FilterError(RuntimeError):
    pass


class SoftwareDepth:
    """A software depth sensor replaying one scan's cached frames."""

    def __init__(self, stream: dict):
        self.w, self.h = int(stream["width"]), int(stream["height"])
        self.units = float(stream["depth_units"])
        intr = rs.intrinsics()
        intr.width, intr.height = self.w, self.h
        intr.fx, intr.fy, intr.ppx, intr.ppy = (float(stream[k]) for k in ("fx", "fy", "ppx", "ppy"))
        intr.model = _DISTORTION[str(stream["model"]).lower().replace(" ", "_")]
        intr.coeffs = [float(c) for c in stream["coeffs"]]
        self.dev = rs.software_device()
        self.sensor = self.dev.add_sensor("Stereo Module")
        vs = rs.video_stream()
        vs.type, vs.index, vs.uid = rs.stream.depth, 0, 0
        vs.width, vs.height, vs.fps, vs.bpp, vs.fmt = self.w, self.h, 30, 2, rs.format.z16
        vs.intrinsics = intr
        self.profile = self.sensor.add_video_stream(vs).as_video_stream_profile()
        self.sensor.add_read_only_option(rs.option.depth_units, self.units)
        if float(stream.get("stereo_baseline_mm", -1)) > 0:     # used by the disparity transform
            self.sensor.add_read_only_option(rs.option.stereo_baseline, float(stream["stereo_baseline_mm"]))
        self.queue = rs.frame_queue(2, keep_frames=True)
        self.sensor.open(self.profile)
        self.sensor.start(self.queue)

    def frame(self, depth: np.ndarray, frame_number: int, timestamp_ms: float, seq=None):
        if seq is not None:
            self.sensor.set_metadata(rs.frame_metadata_value.sequence_id, int(seq[0]))
            self.sensor.set_metadata(rs.frame_metadata_value.sequence_size, int(seq[1]))
        f = rs.software_video_frame()
        f.pixels = depth
        f.stride, f.bpp = self.w * 2, 2
        f.timestamp, f.domain = float(timestamp_ms), rs.timestamp_domain.hardware_clock
        f.frame_number = int(frame_number)
        f.profile = self.profile
        f.depth_units = self.units
        self.sensor.on_video_frame(f)
        return self.queue.wait_for_frame(2000)

    def close(self):
        try:
            self.sensor.stop()
            self.sensor.close()
        except RuntimeError:
            pass


class Chain:
    """Fresh filter instances for one configuration (temporal state starts empty)."""

    def __init__(self, cfg: dict):
        self.steps = []
        if cfg["decimation"]:
            self.steps.append(("decimation", rs.decimation_filter(cfg["decimation"]["magnitude"])))
        if cfg["hdr_merge"]:
            self.steps.append(("hdr_merge", rs.hdr_merge()))
        disparity = bool(cfg["spatial"] or cfg["temporal"])
        if disparity:
            self.steps.append(("to_disparity", rs.disparity_transform(True)))
        if cfg["spatial"]:
            s = cfg["spatial"]
            f = rs.spatial_filter()
            f.set_option(rs.option.filter_magnitude, s["magnitude"])
            f.set_option(rs.option.filter_smooth_alpha, s["alpha"])
            f.set_option(rs.option.filter_smooth_delta, s["delta"])
            f.set_option(rs.option.holes_fill, s["holes_fill"])
            self.steps.append(("spatial", f))
        if cfg["temporal"]:
            t = cfg["temporal"]
            f = rs.temporal_filter()
            f.set_option(rs.option.filter_smooth_alpha, t["alpha"])
            f.set_option(rs.option.filter_smooth_delta, t["delta"])
            f.set_option(rs.option.holes_fill, t["persistence"])
            self.steps.append(("temporal", f))
        if disparity:
            self.steps.append(("to_depth", rs.disparity_transform(False)))
        if cfg["hole_filling"]:
            self.steps.append(("hole_filling", rs.hole_filling_filter(cfg["hole_filling"]["mode"])))
        self.stateful = bool(cfg["temporal"] or cfg["hdr_merge"])

    @property
    def order(self) -> str:
        return " > ".join(n for n, _ in self.steps) or "none"

    def process(self, f):
        for _, filt in self.steps:
            f = filt.process(f)
            if f.is_frameset():                 # hdr_merge may hand back a frameset
                f = f.as_frameset().get_depth_frame()
        return f.as_depth_frame()


def point_cloud(depth_frame, deprojection: str = "rs_marker") -> np.ndarray:
    """Nx3 float32 points as rs_marker's export_to_ply() writes them: zero
    depth dropped, (x, -y, -z), metres.

    rs_marker   : librealsense 2.57 (what rs_marker links) deprojects the
                  colour stream's Inverse Brown-Conrady pixels by iteratively
                  inverting the forward distortion. Reproduced here with the
                  cropper's own pixel_rays() (its 'forward' model, which puts
                  rs_marker's clouds on the pixel grid to 2e-5 px).
    pyrealsense2: rs.pointcloud of the installed wheel. 2.58 applies the
                  polynomial the other way round: identical z, x/y differ
                  by ~0.1 mm near the image centre and up to ~2.7% (25 mm at
                  0.87 m) in the corners.
    """
    if deprojection == "pyrealsense2":
        verts = np.asanyarray(rs.pointcloud().calculate(depth_frame).get_vertices()).view(np.float32).reshape(-1, 3)
        keep = (np.abs(verts) >= 1e-6).any(axis=1)          # export_to_ply's min_distance test
        return verts[keep] * np.array([1, -1, -1], dtype=np.float32)

    from .evaluate import crop_tool
    i = depth_frame.get_profile().as_video_stream_profile().get_intrinsics()
    intr = crop_tool.Intrinsics(i.width, i.height, i.fx, i.fy, i.ppx, i.ppy, tuple(i.coeffs),
                                str(i.model), direction="forward")
    z = np.asanyarray(depth_frame.get_data()).astype(np.float64) * depth_frame.get_units()
    v, u = np.nonzero(z > 0)
    rays = crop_tool.pixel_rays(intr, np.column_stack([u, v]))
    zz = z[v, u]
    return np.column_stack([rays[:, 0] * zz, -rays[:, 1] * zz, -zz]).astype(np.float32)


def depth_gate(points: np.ndarray, min_m=None, max_m=None):
    """Drop points whose depth (camera z; points are in export_to_ply's frame, so depth = -z)
    is below min_m or above max_m. Returns (kept points, number removed)."""
    depth = -points[:, 2]
    keep = np.ones(len(points), bool)
    if min_m is not None:
        keep &= depth >= min_m
    if max_m is not None:
        keep &= depth <= max_m
    return points[keep], int(len(points) - keep.sum())


def run_config(cache, cfg: dict, sw: SoftwareDepth, deprojection: str = "rs_marker") -> dict:
    """Filter the scan, pick the best frame, return it with its point cloud
    (Nx3 float32 in export_to_ply's frame: x, -y, -z, zero points dropped)."""
    meta = cache.meta
    candidates = meta["map"]["candidates"]
    cand = set(candidates)
    chain = Chain(cfg)
    frames = list(cache.stored) if chain.stateful else candidates
    frames = [i for i in frames if i <= candidates[-1]]
    use_seq = cfg["hdr_merge"] is not None

    best, best_valid, valids = None, -1, {}
    out_size = None
    for i in frames:
        fm = meta["frames"][i]
        arr = np.ascontiguousarray(cache.frame(i))
        seq = (fm["sequence_id"], fm["sequence_size"]) if use_seq else None
        out = chain.process(sw.frame(arr, fm["frame_number"], fm["timestamp_ms"], seq))
        if i not in cand:
            continue
        a = np.asanyarray(out.get_data())
        v = int(np.count_nonzero(a))
        valids[i] = v
        out_size = (out.get_width(), out.get_height())
        if v > best_valid:
            best, best_valid = (i, out, arr), v

    if best is None:
        raise FilterError("no candidate frame came out of the filter chain")
    i, depth_frame, _keep_alive = best
    points = point_cloud(depth_frame, deprojection)

    n_px = out_size[0] * out_size[1]
    first = candidates[0]
    return {
        "best_bag_index": i, "best_frame_number": meta["frames"][i]["frame_number"],
        "best_valid_pixels": best_valid, "best_valid_fraction": best_valid / n_px,
        "out_width": out_size[0], "out_height": out_size[1],
        "n_candidates": len(candidates), "n_tied_best": sum(v == best_valid for v in valids.values()),
        "candidate_valid_mean": float(np.mean(list(valids.values()))),
        "frames_fed": len(frames), "warmup_frames": sum(1 for f in frames if f < first),
        "chain_order": chain.order, "points": points,
    }
