#!/usr/bin/env python3
"""
Crop every .ply point cloud found under a session directory, writing the
cropped clouds to a mirrored directory structure.

Expected input layout:
    <Session>/<Session_rx>/<Session_rx_ay>/<Session_rx_ay>.ply
                                          /<Session_rx_ay>_detections.csv
                                          /<Session_rx_ay>.db3
                                          /imu_a<y>.csv

Usage:
    python3 crop_pointclouds.py <session_dir> <output_dir>

Automatic crop (tried first for every file):
  The frame the .ply was exported from is read from imu_a<y>.csv. The
  AprilTag detections for that frame (plus nearby frames, to fill in tags
  that flickered out) are taken from <Session_rx_ay>_detections.csv, and
  the cloud is cropped to the region bounded by the INNER corner of each of
  the four corner tags, so the tags themselves are cut away.
    - Decoded tags are preferred. Undecoded squares (marker_id -1) are only
      used to fill in corners that no decoded tag covers.
    - Missing corners are extrapolated: 3 tags -> parallelogram completion,
      2 tags -> square built on the shared side (or on the diagonal).
    - The clockwise order of the tag IDs around the square is learned from
      any frame in the session where all four tags were decoded (or given
      with --tag-order). This tells us whether two tags share a side or sit
      on a diagonal, and which side of them the square lies on.
    - Cloud points are mapped back to colour-image pixels using the colour
      camera intrinsics read from the scan's .db3 recording (any .db3 in the
      session is used if the scan's own one is missing).

  A preview window shows the kept points in colour, the removed points
  dimmed, and the crop boundary in green:
      Enter  -> accept the automatic crop and save
      M      -> reject it and crop manually (steps 1 and 2 below)
      N      -> skip cropping, save the ORIGINAL cloud
      Escape -> quit the whole batch immediately
  If the automatic crop is not possible (no detections, too few tags, no
  intrinsics) the manual crop is started directly.

Manual crop (two windows per file):

  Step 1 - size the box with the mouse:
    Shift + Left-click  : mark a corner point on the cloud (small sphere appears)
    Shift + Right-click : undo the last marked point
    Mark 2 points that sit at opposite corners of the volume you want to
    keep (e.g. one near-bottom-left point and one far-top-right point).
    You can still orbit/pan/zoom normally with plain mouse drags/scroll.
    Close the window (Q or Esc) once you've marked your 2 (or more) points.
    If you close without marking at least 2 points, the box around the
    automatic crop (or a default centered box) is used instead - you can
    still resize it in step 2.

  Step 2 - fine-tune and confirm with the keyboard:
    A red wireframe box (built from your picked corners) is shown over the
    cloud. Orbit/pan/zoom with the mouse as usual; adjust the box with:
      Move box:    Left/Right    -> move along -X / +X
                   Down/Up       -> move along -Y / +Y
                   PageDn/PageUp -> move along -Z / +Z
      Resize box:  1 / 2         -> shrink / grow X
                   3 / 4         -> shrink / grow Y
                   5 / 6         -> shrink / grow Z
      Step size:   7 / 8         -> smaller / bigger move+resize step
      R            -> reset box to the picked corners (or default box)
      Enter        -> crop to the current box and save, move to next file
      N            -> skip cropping, save the ORIGINAL cloud, move to next file
      Escape       -> quit the whole batch immediately (nothing more is saved)
"""

import argparse
import csv
import re
import shutil
import struct
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import open3d as o3d

# Standard GLFW key codes used by Open3D's legacy visualizer key callbacks.
KEY_ESCAPE = 256
KEY_ENTER = 257
KEY_RIGHT = 262
KEY_LEFT = 263
KEY_DOWN = 264
KEY_UP = 265
KEY_PAGE_UP = 266
KEY_PAGE_DOWN = 267

# An undecoded square is accepted as a missing corner tag if its centre lies
# within this fraction of the square's side from where that tag should be.
UNDECODED_MATCH_FRACTION = 0.35
# Fewer kept points than this means the automatic crop went wrong.
MIN_AUTO_CROP_POINTS = 1000
# RMS distance (px) of projected points from the integer pixel grid above
# which the intrinsics are assumed not to belong to this cloud.
MAX_GRID_RESIDUAL_PX = 0.15
# Surface the tags lie on: fitted to points on the tags and within this many
# pixels inside the crop edge.
PLANE_BAND_PX = 25.0
PLANE_INLIER_DIST_M = 0.005
MIN_PLANE_POINTS = 500


class QuitAll(Exception):
    """Raised to abort the entire batch immediately."""


class AutoCropError(Exception):
    """Raised when the automatic crop cannot be computed for a scan."""


def find_ply_files(root: Path):
    """Return all .ply files under root, sorted for a predictable order."""
    return sorted(root.rglob("*.ply"))


def find_imu_file(ply_path: Path):
    """Return imu_a<x>.csv from the scan's folder, where <x> is the tilt value
    in the folder name (e.g. ..._r135_a-10 -> imu_a-10.csv), or None."""
    match = re.search(r"_a(-?\d+)$", ply_path.parent.name)
    if match:
        imu_path = ply_path.parent / f"imu_a{match.group(1)}.csv"
        if imu_path.is_file():
            return imu_path
    candidates = sorted(ply_path.parent.glob("imu*.csv"))
    return candidates[0] if candidates else None


def copy_imu_file(ply_path: Path, out_dir: Path):
    """Copy the scan's IMU file next to the cropped cloud."""
    imu_path = find_imu_file(ply_path)
    if imu_path is None:
        print(f"    No IMU file found in '{ply_path.parent.name}', skipping.")
        return

    out_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(imu_path, out_dir / imu_path.name)
    print(f"    Copied IMU -> {out_dir / imu_path.name}")


def write_ply_like_original(path: Path, pcd):
    """Write a point cloud as float32 xyz (+ uint8 rgb), matching the
    dtypes of the original RealSense-exported .ply files. Open3D's legacy
    write_point_cloud() always writes points as float64, which downstream
    PCL-based tools (expecting float32) silently fail to parse."""
    positions = np.asarray(pcd.points, dtype=np.float32)
    tpcd = o3d.t.geometry.PointCloud(o3d.core.Tensor(positions, dtype=o3d.core.Dtype.Float32))

    if pcd.has_colors():
        colors = np.asarray(pcd.colors)
        colors_u8 = np.clip(np.round(colors * 255.0), 0, 255).astype(np.uint8)
        tpcd.point.colors = o3d.core.Tensor(colors_u8, dtype=o3d.core.Dtype.UInt8)

    o3d.t.io.write_point_cloud(str(path), tpcd, write_ascii=False)


# ─────────────────────────────────────────────────────────────────────────────
#  Colour camera intrinsics (read from the .db3 recording)
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class Intrinsics:
    width: int
    height: int
    fx: float
    fy: float
    ppx: float
    ppy: float
    coeffs: tuple  # k1, k2, p1, p2, k3
    model: str
    # Whether projecting a point applies the distortion polynomial forward
    # ("forward") or its inverse ("inverse"). Chosen per cloud by checking
    # which one lands the points on the integer pixel grid.
    direction: str = "forward"


def _bag_header_fields(raw):
    fields = {}
    i = 0
    while i < len(raw):
        (n,) = struct.unpack_from("<I", raw, i)
        i += 4
        key, _, value = raw[i:i + n].partition(b"=")
        fields[key.decode()] = value
        i += n
    return fields


def _bag_records(f, end):
    """Yield (header fields, data length) for each ROS1 bag record up to
    `end`, leaving the file positioned at the start of the record's data."""
    while f.tell() < end:
        (header_len,) = struct.unpack("<I", f.read(4))
        header = _bag_header_fields(f.read(header_len))
        (data_len,) = struct.unpack("<I", f.read(4))
        yield header, data_len


def _parse_camera_info(data):
    """Decode the fields we need from a sensor_msgs/CameraInfo message."""
    i = 12  # header: seq (uint32) + stamp (2x uint32)
    (n,) = struct.unpack_from("<I", data, i)
    i += 4 + n  # frame_id
    height, width = struct.unpack_from("<II", data, i)
    i += 8
    (n,) = struct.unpack_from("<I", data, i)
    i += 4
    model = data[i:i + n].decode()
    i += n
    (n,) = struct.unpack_from("<I", data, i)
    i += 4
    coeffs = struct.unpack_from(f"<{n}d", data, i)
    i += 8 * n
    k = struct.unpack_from("<9d", data, i)
    coeffs = tuple(coeffs) + (0.0,) * (5 - len(coeffs))
    return Intrinsics(width, height, k[0], k[4], k[2], k[5], coeffs[:5], model)


def read_color_intrinsics(bag_path: Path):
    """Read the colour stream's camera_info from a librealsense recording
    (the .db3 files are ROS1 bags). Returns Intrinsics or None."""
    with open(bag_path, "rb") as f:
        if not f.read(13).startswith(b"#ROSBAG V2.0"):
            return None
        f.seek(0, 2)
        size = f.tell()
        f.seek(13)
        topics = {}
        for header, data_len in _bag_records(f, size):
            start = f.tell()
            if header.get("op") == b"\x05" and header.get("compression") == b"none":  # chunk
                for inner, inner_len in _bag_records(f, start + data_len):
                    inner_start = f.tell()
                    op = inner.get("op")
                    if op == b"\x07":  # connection
                        topics[inner["conn"]] = inner["topic"].decode()
                    elif op == b"\x02" and topics.get(inner["conn"], "").endswith("Color_0/info/camera_info"):
                        return _parse_camera_info(f.read(inner_len))
                    f.seek(inner_start + inner_len)
            f.seek(start + data_len)
    return None


_intrinsics_cache = {}


def find_color_intrinsics(ply_path: Path, input_dir: Path):
    """Intrinsics from the scan's own .db3, else from any other .db3 in the
    session (same camera and stream settings)."""
    candidates = sorted(ply_path.parent.glob("*.db3"))
    candidates += [p for p in sorted(input_dir.rglob("*.db3")) if p not in candidates]
    for bag in candidates:
        if bag not in _intrinsics_cache:
            try:
                _intrinsics_cache[bag] = read_color_intrinsics(bag)
            except (OSError, struct.error, KeyError, UnicodeDecodeError):
                _intrinsics_cache[bag] = None
        if _intrinsics_cache[bag] is not None:
            return _intrinsics_cache[bag], bag
    return None, None


def _distort(intr: Intrinsics, x, y):
    k1, k2, p1, p2, k3 = intr.coeffs
    r2 = x * x + y * y
    f = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
    dx = x * f + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    dy = y * f + 2 * p2 * x * y + p1 * (r2 + 2 * y * y)
    return dx, dy


def _undistort(intr: Intrinsics, x, y, iterations=20):
    ux, uy = np.array(x, dtype=np.float64), np.array(y, dtype=np.float64)
    for _ in range(iterations):
        dx, dy = _distort(intr, ux, uy)
        ux += x - dx
        uy += y - dy
    return ux, uy


def _ply_to_camera(points):
    """librealsense's export_to_ply() writes (x, -y, -z); undo that."""
    return points * np.array([1.0, -1.0, -1.0])


def project_to_pixels(intr: Intrinsics, points, direction=None):
    """Colour-image pixel (u, v) of each .ply point."""
    cam = _ply_to_camera(np.asarray(points, dtype=np.float64))
    z = np.where(cam[:, 2] > 0, cam[:, 2], np.nan)
    x, y = cam[:, 0] / z, cam[:, 1] / z
    direction = direction or intr.direction
    if direction == "forward":
        x, y = _distort(intr, x, y)
    elif direction == "inverse":
        x, y = _undistort(intr, x, y)
    return np.column_stack([x * intr.fx + intr.ppx, y * intr.fy + intr.ppy])


def pixel_rays(intr: Intrinsics, uv):
    """Inverse of project_to_pixels(): (x/z, y/z) of each pixel's ray."""
    uv = np.asarray(uv, dtype=np.float64)
    x = (uv[:, 0] - intr.ppx) / intr.fx
    y = (uv[:, 1] - intr.ppy) / intr.fy
    if intr.direction == "forward":
        x, y = _undistort(intr, x, y)
    elif intr.direction == "inverse":
        x, y = _distort(intr, x, y)
    return np.column_stack([x, y])


def match_intrinsics_to_cloud(intr: Intrinsics, points):
    """Every point of a RealSense cloud comes from an integer pixel, so the
    right projection puts the points on the pixel grid. Picks the
    distortion direction that does, returning (intrinsics, RMS residual)."""
    sample = np.asarray(points)[:: max(1, len(points) // 50000)]
    best = None
    for direction in ("forward", "inverse", "none"):
        uv = project_to_pixels(intr, sample, direction)
        uv = uv[np.isfinite(uv).all(axis=1)]
        residual = float(np.sqrt(np.mean((uv - np.round(uv)) ** 2)))
        if best is None or residual < best[1]:
            best = (direction, residual)
    matched = Intrinsics(**{**intr.__dict__, "direction": best[0]})
    return matched, best[1]


# ─────────────────────────────────────────────────────────────────────────────
#  AprilTag detections
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class Tag:
    id: int
    decoded: bool
    frame: int
    centre: np.ndarray   # (2,) pixel
    corners: np.ndarray  # (4, 2) pixel

    @property
    def side(self):
        return float(np.mean(np.linalg.norm(self.corners - np.roll(self.corners, 1, axis=0), axis=1)))


def load_detections(csv_path: Path):
    """Return {frame: [Tag, ...]} from an rs_marker *_detections.csv."""
    by_frame = defaultdict(list)
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            try:
                corners = np.array([[float(row[f"c{i}x"]), float(row[f"c{i}y"])] for i in range(4)])
                tag = Tag(
                    id=int(row["marker_id"]),
                    decoded=row.get("decoded", "1").strip() == "1",
                    frame=int(row["frame"]),
                    centre=np.array([float(row["cx"]), float(row["cy"])]),
                    corners=corners,
                )
            except (KeyError, TypeError, ValueError):
                continue  # malformed / truncated row
            by_frame[tag.frame].append(tag)
    return by_frame


def find_detections_file(ply_path: Path):
    path = ply_path.parent / f"{ply_path.parent.name}_detections.csv"
    if path.is_file():
        return path
    candidates = sorted(ply_path.parent.glob("*_detections.csv"))
    return candidates[0] if candidates else None


def read_imu_frame(imu_path: Path):
    """Frame index the .ply was exported from (rs_marker's best frame)."""
    with open(imu_path, newline="") as f:
        for row in csv.DictReader(f):
            return int(float(row["frame"]))
    raise AutoCropError(f"{imu_path.name} has no rows")


def _angle_order(points):
    """Indices of points sorted by angle around their centroid. In image
    coordinates (y down) this runs clockwise on screen."""
    points = np.asarray(points)
    d = points - points.mean(axis=0)
    return list(np.argsort(np.arctan2(d[:, 1], d[:, 0])))


def _canonical_cycle(ids):
    """Rotate a cyclic ID order so it starts at the smallest ID."""
    i = ids.index(min(ids))
    return tuple(ids[i:] + ids[:i])


@dataclass
class TagLayout:
    order: list = None       # corner tag IDs, clockwise on screen (slot i = order[i])
    side_ratio: float = None  # square side (tag centre to centre) / tag side
    aspect: list = None      # per slot i: length of the sides next to side i->i+1,
                             # relative to side i->i+1 (crop quad, image space)


def learn_tag_layout(input_dir: Path, order=None):
    """Learn the corner-tag layout from frames in which exactly four tags
    were decoded: their clockwise order (unless given), the square/tag size
    ratio and the crop quad's side-length ratios, which are used to
    extrapolate missing corners."""
    frames = []
    for csv_path in sorted(input_dir.rglob("*_detections.csv")):
        try:
            by_frame = load_detections(csv_path)
        except OSError:
            continue
        for tags in by_frame.values():
            decoded = {t.id: t for t in tags if t.decoded and t.id >= 0}
            if len(decoded) == 4:
                ids = list(decoded)
                cycle = _canonical_cycle([ids[i] for i in _angle_order([decoded[i].centre for i in ids])])
                frames.append((cycle, decoded))

    if order is None:
        orders = Counter(cycle for cycle, _ in frames)
        if not orders:
            return TagLayout()
        (cycle, count), = orders.most_common(1)
        if len(orders) > 1:
            print(f"    Note: tag order {list(cycle)} seen in {count}/{len(frames)} "
                  f"four-tag frames; others: {[list(o) for o in orders if o != cycle]}")
        order = list(cycle)

    ratios, aspects = [], []
    for cycle, decoded in frames:
        if cycle != _canonical_cycle(list(order)):
            continue
        ring = np.array([decoded[i].centre for i in order])
        square_side = np.mean(np.linalg.norm(ring - np.roll(ring, 1, axis=0), axis=1))
        ratios.append(square_side / np.mean([t.side for t in decoded.values()]))
        quad, _, _ = compute_crop_quad(decoded, [], TagLayout(order), (0, 0))
        side = [np.linalg.norm(quad[(i + 1) % 4] - quad[i]) for i in range(4)]
        aspects.append([(side[(i + 1) % 4] + side[(i + 3) % 4]) / 2 / side[i] for i in range(4)])

    if not ratios:
        return TagLayout(order)
    return TagLayout(order, float(np.median(ratios)), list(np.median(aspects, axis=0)))


def collect_tags(by_frame, frame, window):
    """Decoded tags (one per ID, from the frame nearest `frame`) and
    undecoded squares (deduplicated by position), from frames within
    `window` of `frame`. The camera is static during a scan, so nearby
    frames fill in tags that flickered out of the exported frame."""
    frames = sorted((f for f in by_frame if abs(f - frame) <= window), key=lambda f: (abs(f - frame), f))

    decoded = {}
    for f in frames:
        for tag in by_frame[f]:
            if tag.decoded and tag.id >= 0:
                decoded.setdefault(tag.id, tag)

    undecoded = []
    for f in frames:
        for tag in by_frame[f]:
            if tag.decoded:
                continue
            known = list(decoded.values()) + undecoded
            if all(np.linalg.norm(tag.centre - k.centre) > 0.5 * max(tag.side, k.side) for k in known):
                undecoded.append(tag)
    return decoded, undecoded


# ─────────────────────────────────────────────────────────────────────────────
#  Crop square from tags
# ─────────────────────────────────────────────────────────────────────────────

def _cross(a, b):
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def _rot90(v):
    """Rotate so that, for side P_i -> P_i+1 of a quad in clockwise-on-screen
    slot order, the result points into the quad."""
    return np.array([-v[1], v[0]])


def complete_square(points_by_slot, aspect=None):
    """Given 2-4 of the square's corners by slot (0..3 clockwise on screen),
    extrapolate the missing ones. `aspect` (see TagLayout) turns the square
    into a rectangle when only 2 corners are known. Returns a (4, 2) array
    in slot order."""
    p = {s: np.asarray(v, dtype=np.float64) for s, v in points_by_slot.items()}
    if len(p) == 4:
        pass
    elif len(p) == 3:
        # Parallelogram completion: missing = its two neighbours - the opposite corner.
        m = next(s for s in range(4) if s not in p)
        p[m] = p[(m - 1) % 4] + p[(m + 1) % 4] - p[(m + 2) % 4]
    elif len(p) == 2:
        a, b = sorted(p)
        if (b - a) % 2 == 1:
            # Two corners sharing a side: build the rectangle on the side of
            # it that the clockwise slot order puts the rest of it on.
            i = a if (a + 1) % 4 == b else b
            e = p[(i + 1) % 4] - p[i]
        else:
            # Two corners on a diagonal D = e + k*rot90(e): solve for side e.
            i = a
            k = aspect[i] if aspect else 1.0
            d = p[b] - p[a]
            e = np.array([d[0] + k * d[1], d[1] - k * d[0]]) / (1 + k * k)
            p[(i + 1) % 4] = p[i] + e
        n = (aspect[i] if aspect else 1.0) * _rot90(e)
        p[(i + 2) % 4] = p[(i + 1) % 4] + n
        p[(i + 3) % 4] = p[i] + n
    else:
        raise AutoCropError(f"need at least 2 corners, have {len(p)}")
    return np.array([p[s] for s in range(4)])


def _inside_triangle(pt, a, b, c):
    s = [_cross(q - p, pt - p) for p, q in ((a, b), (b, c), (c, a))]
    return all(v >= 0 for v in s) or all(v <= 0 for v in s)


def _drop_interior(tags):
    """Corner tags sit on the outside of the region, so a square lying inside
    the triangle of three others (e.g. a false detection on the sample) is
    not one of them. Earlier tags in the list are preferred."""
    keep = list(tags)
    for t in reversed(tags):
        others = [o.centre for o in keep if o is not t]
        if len(others) >= 3 and any(
                _inside_triangle(t.centre, others[i], others[j], others[k])
                for i in range(len(others)) for j in range(i + 1, len(others))
                for k in range(j + 1, len(others))):
            keep.remove(t)
    return keep


def _slots_without_layout(tags, image_centre, side_ratio):
    """Assign 2-4 tags of unknown identity to square slots from geometry."""
    centres = np.array([t.centre for t in tags])
    if len(tags) >= 4:
        order = _angle_order(centres[:4])
        return {slot: tags[i] for slot, i in enumerate(order)}, "by position"
    if len(tags) == 3:
        # The corner opposite the longest side is the one between the other two.
        opposite_side = [np.linalg.norm(centres[(i + 1) % 3] - centres[(i + 2) % 3]) for i in range(3)]
        mid = int(np.argmax(opposite_side))
        return {0: tags[(mid + 1) % 3], 1: tags[mid], 2: tags[(mid + 2) % 3]}, "by position"

    a, b = tags
    distance = np.linalg.norm(b.centre - a.centre) / np.mean([a.side, b.side])
    if side_ratio and abs(distance - side_ratio * np.sqrt(2)) < abs(distance - side_ratio):
        return {0: a, 2: b}, "assumed diagonal (from tag spacing)"
    # Assume they share a side, and that the square lies towards the image centre.
    if _cross(b.centre - a.centre, image_centre - a.centre) < 0:
        a, b = b, a
    return {0: a, 1: b}, "assumed same side, square towards image centre"


def compute_crop_quad(decoded, undecoded, layout: TagLayout, image_size):
    """Pixel quad bounded by the inner corners of the four corner tags,
    extrapolating missing tags. Returns ((4, 2) quad, list of notes)."""
    notes = []
    slots = {}
    aspect = None
    if layout.order:
        slots = {i: decoded[tid] for i, tid in enumerate(layout.order) if tid in decoded}

    if len(slots) >= 2:
        aspect = layout.aspect
        # Fill empty slots with undecoded squares found where the tag should be.
        centres = complete_square({s: t.centre for s, t in slots.items()}, aspect)
        side = np.mean(np.linalg.norm(centres - np.roll(centres, 1, axis=0), axis=1))
        used = set()
        for s in range(4):
            if s in slots:
                continue
            dists = [np.linalg.norm(u.centre - centres[s]) if i not in used else np.inf
                     for i, u in enumerate(undecoded)]
            if dists and min(dists) < UNDECODED_MATCH_FRACTION * side:
                i = int(np.argmin(dists))
                used.add(i)
                slots[s] = undecoded[i]
        notes.append(f"slots from tag IDs {layout.order}")
    else:
        # Not enough known IDs: use whatever squares were found, decoded first,
        # then undecoded ones closest in size to the others.
        pool = list(decoded.values())
        ref_side = np.median([t.side for t in (pool or undecoded)]) if (pool or undecoded) else 0
        pool = _drop_interior(pool + sorted(undecoded, key=lambda t: abs(t.side - ref_side)))
        if len(pool) < 2:
            raise AutoCropError(f"only {len(pool)} tag(s) found near the exported frame")
        image_centre = np.array(image_size, dtype=np.float64) / 2.0
        slots, how = _slots_without_layout(pool[:4], image_centre, layout.side_ratio)
        notes.append("slots " + how)

    notes.append("tags used: " + ", ".join(
        f"{t.id if t.decoded else 'undecoded'}@frame{t.frame}" for t in slots.values()))
    if len(slots) < 4:
        notes.append(f"extrapolated {4 - len(slots)} missing corner(s)")

    # Inner corner of each real tag = its corner nearest the square's centre.
    centre = complete_square({s: t.centre for s, t in slots.items()}, aspect).mean(axis=0)
    inner = {s: t.corners[np.argmin(np.linalg.norm(t.corners - centre, axis=1))] for s, t in slots.items()}
    return complete_square(inner, aspect), notes, list(slots.values())


def points_in_quad(uv, quad):
    """Boolean mask of pixels lying inside a convex quad (any vertex order)."""
    quad = quad[_angle_order(quad)]
    inside = np.isfinite(uv).all(axis=1)
    for k in range(len(quad)):
        edge = quad[(k + 1) % len(quad)] - quad[k]
        inside &= _cross(edge, uv - quad[k]) >= 0
    return inside


def shrink_quad(quad, margin_px):
    """Move each vertex `margin_px` towards the quad's centroid."""
    centre = quad.mean(axis=0)
    d = centre - quad
    return quad + d / np.linalg.norm(d, axis=1, keepdims=True) * margin_px


def fit_plane(cam_points, sample):
    """RANSAC plane through cam_points[sample]. Returns (unit normal pointing
    towards the camera, offset d), with normal . p + d = height of p above
    the plane, or None if there are too few points."""
    if sample.sum() < MIN_PLANE_POINTS:
        return None
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(cam_points[sample]))
    (a, b, c, d), inliers = cloud.segment_plane(PLANE_INLIER_DIST_M, 3, 500)
    if len(inliers) < MIN_PLANE_POINTS:
        return None
    normal = np.array([a, b, c])
    scale = np.linalg.norm(normal)
    normal, d = normal / scale, d / scale
    if d < 0:  # the camera (origin) should be on the positive side
        normal, d = -normal, -d
    return normal, d


def intersect_rays(intr: Intrinsics, uv, plane):
    """Camera-coords points where the pixels' rays meet a plane."""
    normal, d = plane
    rays = np.column_stack([pixel_rays(intr, uv), np.ones(len(uv))])
    return rays * (-d / (rays @ normal))[:, None]


def prism_crop(cam_points, corners, normal):
    """Keep points inside the column through `corners` (3D, any height)
    running along `normal`, with no height limit."""
    e1 = corners[1] - corners[0]
    e1 -= (e1 @ normal) * normal
    e1 /= np.linalg.norm(e1)
    basis = np.column_stack([e1, np.cross(normal, e1)])
    return points_in_quad(cam_points @ basis, corners @ basis)


@dataclass
class AutoCrop:
    mask: np.ndarray
    outline: np.ndarray  # (8, 3) camera coords: bottom quad, then top quad
    notes: list


def compute_auto_crop(pcd, ply_path: Path, input_dir: Path, layout: TagLayout, args):
    imu_path = find_imu_file(ply_path)
    if imu_path is None:
        raise AutoCropError("no imu*.csv next to the .ply")
    det_path = find_detections_file(ply_path)
    if det_path is None:
        raise AutoCropError("no *_detections.csv next to the .ply")

    frame = read_imu_frame(imu_path)
    by_frame = load_detections(det_path)
    decoded, undecoded = collect_tags(by_frame, frame, args.frame_window)
    if not decoded and not undecoded:
        raise AutoCropError(f"no tags within {args.frame_window} frames of frame {frame}")

    intr, bag = find_color_intrinsics(ply_path, input_dir)
    if intr is None:
        raise AutoCropError("no .db3 recording with colour intrinsics found")
    points = np.asarray(pcd.points)
    intr, residual = match_intrinsics_to_cloud(intr, points)
    if residual > MAX_GRID_RESIDUAL_PX:
        raise AutoCropError(f"intrinsics from {bag.name} don't match this cloud "
                            f"(pixel-grid residual {residual:.2f}px)")

    quad, notes, tags = compute_crop_quad(decoded, undecoded, layout, (intr.width, intr.height))
    if args.margin:
        quad = shrink_quad(quad, args.margin)
    notes.insert(0, f"exported frame {frame}, intrinsics from {bag.name} ({intr.model})")

    cam_points = _ply_to_camera(points)
    uv = project_to_pixels(intr, points)
    # The surface inside the square (fitted to a band just inside the crop
    # edge, away from the object) sets the column's direction. The tags may
    # sit higher than it, so the column's corners go where the inner-corner
    # rays meet the tags' own plane.
    band = points_in_quad(uv, quad) & ~points_in_quad(uv, shrink_quad(quad, PLANE_BAND_PX))
    on_tags = np.zeros(len(points), dtype=bool)
    for tag in tags:
        on_tags |= points_in_quad(uv, tag.corners)
    surface = fit_plane(cam_points, band)
    tag_plane = fit_plane(cam_points, on_tags) or surface

    if surface is not None:
        normal, d = surface
        corners = intersect_rays(intr, quad[_angle_order(quad)], tag_plane)
        mask = prism_crop(cam_points, corners, normal)
        heights = cam_points[mask] @ normal + d if mask.any() else np.zeros(1)
        base = corners - (corners @ normal + d)[:, None] * normal  # dropped onto the surface
        outline = np.vstack([base + min(heights.min(), 0.0) * normal, base + heights.max() * normal])
        notes.append(f"crop is a column standing on the surface (tags {np.mean(corners @ normal + d) * 1000:.0f} mm "
                     f"above it, tallest kept point {heights.max() * 1000:.0f} mm)")
    else:
        # No usable surface: fall back to the camera's view through the quad.
        mask = points_in_quad(uv, quad)
        rays = np.column_stack([pixel_rays(intr, quad[_angle_order(quad)]), np.ones(4)])
        near, far = np.percentile(cam_points[mask, 2], [0.5, 99.5]) if mask.any() else (0.0, 0.0)
        outline = np.vstack([rays * far, rays * near])
        notes.append("couldn't fit the tag surface; cropping to the camera's view of the tag square")

    if mask.sum() < MIN_AUTO_CROP_POINTS:
        raise AutoCropError(f"crop region only contains {int(mask.sum())} points")
    return AutoCrop(mask, outline, notes)


# ─────────────────────────────────────────────────────────────────────────────
#  Interactive windows
# ─────────────────────────────────────────────────────────────────────────────

def _crop_outline(auto: AutoCrop):
    """Green wireframe of the crop region, spanning the kept points."""
    lines = [[i, (i + 1) % 4] for i in range(4)] + [[i + 4, (i + 1) % 4 + 4] for i in range(4)] \
        + [[i, i + 4] for i in range(4)]
    outline = o3d.geometry.LineSet(
        o3d.utility.Vector3dVector(_ply_to_camera(auto.outline)),  # the flip is its own inverse
        o3d.utility.Vector2iVector(lines))
    outline.paint_uniform_color((0, 1, 0))
    return outline


def preview_auto_crop(pcd, auto: AutoCrop, window_name):
    """Show the automatic crop; returns 'crop', 'manual', 'keep' or 'quit'."""
    preview = o3d.geometry.PointCloud(pcd)
    colors = np.asarray(preview.colors) if preview.has_colors() else np.full((len(pcd.points), 3), 0.7)
    colors = colors.copy()
    colors[~auto.mask] = colors[~auto.mask] * 0.25 + 0.1
    preview.colors = o3d.utility.Vector3dVector(colors)

    result = {"value": "keep"}

    def finish(value):
        def cb(vis):
            result["value"] = value
            vis.close()
            return False
        return cb

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name=window_name)
    vis.add_geometry(preview)
    vis.add_geometry(_crop_outline(auto))
    vis.register_key_callback(KEY_ENTER, finish("crop"))
    vis.register_key_callback(ord("M"), finish("manual"))
    vis.register_key_callback(ord("N"), finish("keep"))
    vis.register_key_callback(KEY_ESCAPE, finish("quit"))
    vis.run()
    vis.destroy_window()
    return result["value"]


def _default_box_bounds(pcd):
    full_aabb = pcd.get_axis_aligned_bounding_box()
    center = full_aabb.get_center()
    extent = np.maximum(full_aabb.get_extent(), 1e-6)
    half = extent / 2.0 * 0.5  # 50% of the cloud's size, centered
    return center - half, center + half


def _picked_point_index(picked):
    """Open3D has returned either plain ints or PickedPoint objects (with
    an .index attribute) for get_picked_points() across versions."""
    if isinstance(picked, (int, np.integer)):
        return int(picked)
    if hasattr(picked, "index"):
        return int(picked.index)
    return None


def pick_corners_with_mouse(pcd, window_name):
    """Let the user Shift+click points on the cloud to mark box corners.
    Returns (min_bound, max_bound) spanning the picked points, or
    (None, None) if fewer than 2 points were picked."""
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name=window_name)
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()

    picked = vis.get_picked_points()
    indices = [idx for idx in (_picked_point_index(p) for p in picked) if idx is not None]

    if len(indices) < 2:
        return None, None

    points = np.asarray(pcd.points)[indices]
    return points.min(axis=0), points.max(axis=0)


class BoxCropper:
    def __init__(self, pcd, initial_min=None, initial_max=None):
        self.pcd = pcd
        full_aabb = pcd.get_axis_aligned_bounding_box()
        self.full_extent = np.maximum(full_aabb.get_extent(), 1e-6)

        if initial_min is not None and initial_max is not None:
            self.reset_min, self.reset_max = initial_min.copy(), initial_max.copy()
        else:
            self.reset_min, self.reset_max = _default_box_bounds(pcd)

        self.min_bound = self.reset_min.copy()
        self.max_bound = self.reset_max.copy()

        self.step = float(np.max(self.full_extent)) * 0.02
        self.min_size = 0.01 * self.full_extent

        self.box = o3d.geometry.AxisAlignedBoundingBox(self.min_bound, self.max_bound)
        self.box.color = (1, 0, 0)

        self.result = None  # "crop" | "keep" | "quit"

    def _sync_box(self, vis):
        self.box.min_bound = self.min_bound
        self.box.max_bound = self.max_bound
        vis.update_geometry(self.box)
        vis.update_renderer()

    def move(self, axis, sign):
        def cb(vis):
            delta = sign * self.step
            self.min_bound[axis] += delta
            self.max_bound[axis] += delta
            self._sync_box(vis)
            return False

        return cb

    def resize(self, axis, sign):
        def cb(vis):
            delta = sign * self.step
            new_min = self.min_bound[axis] - delta / 2.0
            new_max = self.max_bound[axis] + delta / 2.0
            if new_max - new_min < self.min_size[axis]:
                mid = (new_max + new_min) / 2.0
                new_min = mid - self.min_size[axis] / 2.0
                new_max = mid + self.min_size[axis] / 2.0
            self.min_bound[axis] = new_min
            self.max_bound[axis] = new_max
            self._sync_box(vis)
            return False

        return cb

    def change_step(self, factor):
        def cb(vis):
            self.step *= factor
            print(f"    step size: {self.step:.4f}")
            return False

        return cb

    def reset(self, vis):
        self.min_bound = self.reset_min.copy()
        self.max_bound = self.reset_max.copy()
        self._sync_box(vis)
        return False

    def finish(self, result):
        def cb(vis):
            self.result = result
            vis.close()
            return False

        return cb


def manual_crop(pcd, path: Path, fallback_min=None, fallback_max=None):
    """Run the two-phase interactive crop (mouse-pick corners, then
    keyboard fine-tune) for one point cloud and return the result."""
    min_bound, max_bound = pick_corners_with_mouse(
        pcd,
        window_name=f"Step 1/2 - {path.name}: Shift+click 2 opposite corners, then Q/Esc",
    )
    if min_bound is None:
        min_bound, max_bound = fallback_min, fallback_max
    cropper = BoxCropper(pcd, min_bound, max_bound)

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(
        window_name=f"Step 2/2 - {path.name}: fine-tune box (Enter=crop&save, N=keep original, Esc=quit all)"
    )
    vis.add_geometry(pcd)
    vis.add_geometry(cropper.box)

    vis.register_key_callback(KEY_LEFT, cropper.move(0, -1))
    vis.register_key_callback(KEY_RIGHT, cropper.move(0, 1))
    vis.register_key_callback(KEY_DOWN, cropper.move(1, -1))
    vis.register_key_callback(KEY_UP, cropper.move(1, 1))
    vis.register_key_callback(KEY_PAGE_DOWN, cropper.move(2, -1))
    vis.register_key_callback(KEY_PAGE_UP, cropper.move(2, 1))

    vis.register_key_callback(ord("1"), cropper.resize(0, -1))
    vis.register_key_callback(ord("2"), cropper.resize(0, 1))
    vis.register_key_callback(ord("3"), cropper.resize(1, -1))
    vis.register_key_callback(ord("4"), cropper.resize(1, 1))
    vis.register_key_callback(ord("5"), cropper.resize(2, -1))
    vis.register_key_callback(ord("6"), cropper.resize(2, 1))

    vis.register_key_callback(ord("7"), cropper.change_step(1 / 1.5))
    vis.register_key_callback(ord("8"), cropper.change_step(1.5))

    vis.register_key_callback(ord("R"), cropper.reset)
    vis.register_key_callback(KEY_ENTER, cropper.finish("crop"))
    vis.register_key_callback(ord("N"), cropper.finish("keep"))
    vis.register_key_callback(KEY_ESCAPE, cropper.finish("quit"))

    vis.run()
    vis.destroy_window()

    if cropper.result == "quit":
        raise QuitAll()

    if cropper.result == "crop":
        cropped = pcd.crop(cropper.box)
        if len(cropped.points) == 0:
            print("    Warning: box contained no points, saving original cloud instead.")
            return pcd
        return cropped

    # "keep", or window closed without pressing a finishing key
    return pcd


def crop_point_cloud(path: Path, input_dir: Path, layout: TagLayout, args):
    """Try the tag-based automatic crop, fall back to the manual crop."""
    pcd = o3d.io.read_point_cloud(str(path))

    auto = None
    if not args.no_auto:
        try:
            auto = compute_auto_crop(pcd, path, input_dir, layout, args)
        except AutoCropError as e:
            print(f"    Automatic crop not possible: {e}. Falling back to manual crop.")

    if auto is not None:
        for note in auto.notes:
            print(f"    {note}")
        print(f"    Automatic crop keeps {int(auto.mask.sum())}/{len(auto.mask)} points")
        cropped = pcd.select_by_index(np.flatnonzero(auto.mask))

        choice = "crop" if args.auto_accept else preview_auto_crop(
            pcd, auto,
            window_name=f"Auto crop - {path.name} (Enter=accept, M=manual, N=keep original, Esc=quit all)",
        )
        if choice == "quit":
            raise QuitAll()
        if choice == "crop":
            return cropped
        if choice == "keep":
            return pcd
        box = cropped.get_axis_aligned_bounding_box()
        return manual_crop(pcd, path, box.min_bound, box.max_bound)

    return manual_crop(pcd, path)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input_dir", type=Path, help="Path to the top-level <Session> directory")
    parser.add_argument("output_dir", type=Path, help="Where to write the cropped point clouds")
    parser.add_argument(
        "--suffix",
        default="_cropped",
        help="Suffix appended to each output filename (before .ply). Default: '_cropped'",
    )
    parser.add_argument(
        "--skip-existing",
        action="store_true",
        help="Skip files whose cropped output already exists",
    )
    parser.add_argument(
        "--no-auto",
        action="store_true",
        help="Don't try the tag-based automatic crop; always crop manually",
    )
    parser.add_argument(
        "--auto-accept",
        action="store_true",
        help="Save automatic crops without showing the preview window "
             "(scans where it fails still open the manual crop)",
    )
    parser.add_argument(
        "--tag-order",
        type=lambda s: [int(v) for v in s.split(",")],
        help="Clockwise order of the four corner tag IDs as seen by the camera, "
             "e.g. 6,5,2,4. Default: learned from the session's detections",
    )
    parser.add_argument(
        "--frame-window",
        type=int,
        default=15,
        help="Also use detections up to this many frames either side of the "
             "exported frame to fill in missing tags. Default: 15",
    )
    parser.add_argument(
        "--margin",
        type=float,
        default=0.0,
        help="Pull the automatic crop boundary this many pixels further in from "
             "the tags' inner corners (negative grows it). Default: 0",
    )
    args = parser.parse_args()

    input_dir = args.input_dir.resolve()
    output_dir = args.output_dir.resolve()

    if not input_dir.is_dir():
        print(f"Error: input directory does not exist: {input_dir}", file=sys.stderr)
        sys.exit(1)

    ply_files = find_ply_files(input_dir)
    if not ply_files:
        print(f"No .ply files found under {input_dir}")
        sys.exit(0)

    print(f"Found {len(ply_files)} point cloud(s) under {input_dir}")
    print(__doc__)

    layout = TagLayout()
    if not args.no_auto:
        if args.tag_order and len(args.tag_order) != 4:
            print("Error: --tag-order needs exactly 4 IDs", file=sys.stderr)
            sys.exit(1)
        layout = learn_tag_layout(input_dir, args.tag_order)
        if layout.order:
            print(f"Corner tag order (clockwise): {layout.order}")
            if layout.aspect:
                print("Crop side ratios learned for extrapolation: "
                      + ", ".join(f"{a:.2f}" for a in layout.aspect))
        else:
            print("No frame with all four tags decoded; tag positions will be guessed from geometry.")

    for i, ply_path in enumerate(ply_files, start=1):
        rel_path = ply_path.relative_to(input_dir)
        out_path = output_dir / rel_path.with_name(rel_path.stem + args.suffix + ".ply")

        if args.skip_existing and out_path.exists():
            print(f"[{i}/{len(ply_files)}] Skipping (already exists): {out_path}")
            copy_imu_file(ply_path, out_path.parent)
            continue

        print(f"\n[{i}/{len(ply_files)}] Cropping: {rel_path}")
        try:
            cropped = crop_point_cloud(ply_path, input_dir, layout, args)
        except QuitAll:
            print("\nQuit requested. Stopping without processing remaining files.")
            sys.exit(0)

        out_path.parent.mkdir(parents=True, exist_ok=True)
        write_ply_like_original(out_path, cropped)
        print(f"    Saved -> {out_path}  ({len(cropped.points)} points)")
        copy_imu_file(ply_path, out_path.parent)

    print("\nDone.")


if __name__ == "__main__":
    main()
