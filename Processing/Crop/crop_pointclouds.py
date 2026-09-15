#!/usr/bin/env python3
"""
Interactively crop every .ply point cloud found under a session directory,
writing the cropped clouds to a mirrored directory structure.

Expected input layout:
    <Session>/<Session_rx>/<Session_rx_ay>/<session_rx_ay>.ply

Usage:
    python3 crop_pointclouds.py <session_dir> <output_dir>

How cropping works (two windows per file):

  Step 1 - size the box with the mouse:
    Shift + Left-click  : mark a corner point on the cloud (small sphere appears)
    Shift + Right-click : undo the last marked point
    Mark 2 points that sit at opposite corners of the volume you want to
    keep (e.g. one near-bottom-left point and one far-top-right point).
    You can still orbit/pan/zoom normally with plain mouse drags/scroll.
    Close the window (Q or Esc) once you've marked your 2 (or more) points.
    If you close without marking at least 2 points, a default centered
    box is used instead - you can still resize it in step 2.

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
import re
import shutil
import sys
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


class QuitAll(Exception):
    """Raised to abort the entire batch immediately."""


def find_ply_files(root: Path):
    """Return all .ply files under root, sorted for a predictable order."""
    return sorted(root.rglob("*.ply"))


def copy_imu_file(ply_path: Path, out_dir: Path):
    """Copy imu_a<x>.csv from the scan's folder, where <x> is the tilt value
    in the folder name (e.g. ..._r135_a-10 -> imu_a-10.csv)."""
    match = re.search(r"_a(-?\d+)$", ply_path.parent.name)
    if not match:
        print(f"    No tilt value in folder name '{ply_path.parent.name}', not copying IMU file.")
        return

    imu_path = ply_path.parent / f"imu_a{match.group(1)}.csv"
    if not imu_path.is_file():
        print(f"    No IMU file found ({imu_path.name}), skipping.")
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


def crop_point_cloud(path: Path):
    """Run the two-phase interactive crop (mouse-pick corners, then
    keyboard fine-tune) for one point cloud and return the result."""
    pcd = o3d.io.read_point_cloud(str(path))

    min_bound, max_bound = pick_corners_with_mouse(
        pcd,
        window_name=f"Step 1/2 - {path.name}: Shift+click 2 opposite corners, then Q/Esc",
    )
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

    for i, ply_path in enumerate(ply_files, start=1):
        rel_path = ply_path.relative_to(input_dir)
        out_path = output_dir / rel_path.with_name(rel_path.stem + args.suffix + ".ply")

        if args.skip_existing and out_path.exists():
            print(f"[{i}/{len(ply_files)}] Skipping (already exists): {out_path}")
            copy_imu_file(ply_path, out_path.parent)
            continue

        print(f"\n[{i}/{len(ply_files)}] Cropping: {rel_path}")
        try:
            cropped = crop_point_cloud(ply_path)
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
