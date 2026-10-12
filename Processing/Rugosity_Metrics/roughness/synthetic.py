"""Synthetic sediment surfaces with known roughness, for the tests and for
trying the tool without a sensor.

    python -m roughness.synthetic ripples.ply            # 1M-point ripple scan
    python -m roughness.synthetic --help

Surfaces are evaluated analytically at arbitrary (x, y), so any point density
or hole pattern can be drawn from them; the result can be put into a tilted
camera frame (camera at the origin looking down at the bed, like a RealSense
capture or Fuse output).
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


def uniform_xy(n, size_x, size_y, rng):
    return np.column_stack([rng.uniform(-size_x / 2, size_x / 2, n), rng.uniform(-size_y / 2, size_y / 2, n)])


def lattice_xy(size_x, size_y, spacing):
    """Regular lattice, one point per `spacing` cell, centred on the origin."""
    xs = (np.arange(int(round(size_x / spacing))) + 0.5) * spacing - size_x / 2
    ys = (np.arange(int(round(size_y / spacing))) + 0.5) * spacing - size_y / 2
    X, Y = np.meshgrid(xs, ys)
    return np.column_stack([X.ravel(), Y.ravel()])


def sinusoid(xy, amplitude, wavelength, direction_deg=0.0):
    """z = A sin(2 pi (x cos t + y sin t) / L): wave vector at direction_deg."""
    t = np.radians(direction_deg)
    return amplitude * np.sin(2 * np.pi * (xy[:, 0] * np.cos(t) + xy[:, 1] * np.sin(t)) / wavelength)


def ripples(xy, amplitude, wavelength, direction_deg=0.0, asymmetry=0.35, wobble=0.15, seed=0):
    """Asymmetric, slightly sinuous ripples (steep lee face from a second
    harmonic; crest lines wander by `wobble` wavelengths)."""
    rng = np.random.default_rng(seed)
    t = np.radians(direction_deg)
    u = xy[:, 0] * np.cos(t) + xy[:, 1] * np.sin(t)
    v = -xy[:, 0] * np.sin(t) + xy[:, 1] * np.cos(t)
    phase0 = rng.uniform(0, 2 * np.pi)
    u = u + wobble * wavelength * np.sin(2 * np.pi * v / (6 * wavelength) + phase0)
    p = 2 * np.pi * u / wavelength
    return amplitude * (np.sin(p) + asymmetry * np.sin(2 * p)) / np.sqrt(1 + asymmetry ** 2)


class RandomField:
    """Isotropic Gaussian random surface as a sum of cosines with random wave
    vectors |k| in [1/lambda_max, 1/lambda_min], amplitude ~ |k|^-(1+H) so the
    2D PSD follows q^-2(1+H) (H=None: flat spectrum). Evaluable anywhere;
    scaled so its Sq over a large area is `sq`."""

    def __init__(self, sq, lambda_min, lambda_max, hurst=None, n_modes=400, seed=0):
        rng = np.random.default_rng(seed)
        # Log-uniform |k| sampling; amplitude weights make the PSD a power law.
        k = np.exp(rng.uniform(np.log(1 / lambda_max), np.log(1 / lambda_min), n_modes))
        th = rng.uniform(0, np.pi, n_modes)
        self.kx, self.ky = k * np.cos(th), k * np.sin(th)
        amp = np.ones(n_modes) if hurst is None else k ** (-(1 + hurst)) * k   # x k: log sampling density
        self.amp = amp * sq / np.sqrt(0.5 * np.sum(amp ** 2))
        self.phase = rng.uniform(0, 2 * np.pi, n_modes)

    def __call__(self, xy, chunk=200000):
        out = np.empty(len(xy))
        for s in range(0, len(xy), chunk):
            p = xy[s:s + chunk]
            arg = 2 * np.pi * (np.outer(p[:, 0], self.kx) + np.outer(p[:, 1], self.ky)) + self.phase
            out[s:s + chunk] = np.cos(arg) @ self.amp
        return out


def fractal_grid(n, g, hurst, sq, seed=0):
    """Periodic isotropic self-affine height map (n x n, spacing g) by FFT
    filtering of white noise: 2D PSD ~ q^-2(1+H)."""
    rng = np.random.default_rng(seed)
    k = np.fft.fftfreq(n, g)
    K = np.hypot(*np.meshgrid(k, k))
    K[0, 0] = np.inf
    F = np.fft.fft2(rng.normal(size=(n, n))) * K ** (-(1 + hurst))
    z = np.real(np.fft.ifft2(F))
    return z / z.std() * sq


def polynomial_trend(xy, order, size, magnitude, seed=0):
    """Random polynomial of total degree `order` whose highest-degree terms are
    significant, peak-to-peak about `magnitude` over a square of `size`."""
    rng = np.random.default_rng(seed)
    xn, yn = xy[:, 0] / (size / 2), xy[:, 1] / (size / 2)
    z = np.zeros(len(xy))
    for d in range(1, order + 1):
        for i in range(d + 1):
            z += rng.uniform(0.5, 1.0) * rng.choice([-1, 1]) * xn ** i * yn ** (d - i)
    return z / max(np.ptp(z), 1e-12) * magnitude


def punch_holes(xy, centres, radius):
    keep = np.ones(len(xy), dtype=bool)
    for c in centres:
        keep &= np.hypot(xy[:, 0] - c[0], xy[:, 1] - c[1]) > radius
    return keep


def to_camera_frame(points, tilt_deg=20.0, height=0.7, spin_deg=0.0):
    """Put a z-up bed into a camera frame: camera at the origin, `height`
    metres above the bed, optical axis +Z pointing down at it, tilted by
    tilt_deg about the camera x axis and spun by spin_deg about the optical
    axis. With no spin, roughness_scans' levelled frame is the bed frame again
    (x = bed x), so measured directions can be checked against the truth."""
    from scipy.spatial.transform import Rotation
    flip = Rotation.from_euler("x", 180, degrees=True)           # bed z-up -> camera looks along +Z
    tilt = Rotation.from_euler("x", tilt_deg, degrees=True)
    spin = Rotation.from_euler("z", spin_deg, degrees=True)
    R = (spin * tilt * flip).as_matrix()
    camera = np.array([0, 0, height])
    return (points - camera) @ R.T


def write_ply(path, points):
    import open3d as o3d
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.asarray(points, dtype=float)))
    o3d.io.write_point_cloud(str(path), pcd, write_ascii=False)


def make_ripple_scan(n_points=1_000_000, size=(0.5, 0.35), wavelength=0.045, amplitude=0.004,
                     direction_deg=15.0, small_sq=0.0003, noise=0.0005, bowl=0.01, tilt_deg=20.0,
                     seed=1):
    """A fused-scan stand-in: ripples + small-scale roughness + a gentle
    quadratic bowl + sensor noise, with a few dropout holes and a ragged
    border, in a tilted camera frame. Metres."""
    rng = np.random.default_rng(seed)
    xy = uniform_xy(int(n_points * 1.08), size[0], size[1], rng)
    holes = rng.uniform(-0.4, 0.4, (12, 2)) * np.array(size)
    keep = punch_holes(xy, holes[:8], 0.004) & punch_holes(xy, holes[8:], 0.012)
    edge = 0.5 * size[0] - np.abs(xy[:, 0]) > 0.006 * (1 + np.sin(xy[:, 1] * 90))      # ragged x border
    xy = xy[keep & edge][:n_points]
    z = (ripples(xy, amplitude, wavelength, direction_deg, seed=seed)
         + RandomField(small_sq, 0.002, 0.012, hurst=0.7, n_modes=300, seed=seed + 1)(xy)
         + bowl * ((xy[:, 0] / size[0]) ** 2 + 0.6 * (xy[:, 1] / size[1]) ** 2)
         + rng.normal(0, noise, len(xy)))
    return to_camera_frame(np.column_stack([xy, z]), tilt_deg)


def main(argv=None):
    p = argparse.ArgumentParser(description="Write a synthetic rippled-bed scan (metres, camera frame).")
    p.add_argument("output", help="output .ply")
    p.add_argument("--points", type=float, default=1e6)
    p.add_argument("--wavelength-mm", type=float, default=45.0)
    p.add_argument("--amplitude-mm", type=float, default=4.0)
    p.add_argument("--direction-deg", type=float, default=15.0, help="ripple wave-vector direction")
    p.add_argument("--noise-mm", type=float, default=0.5, help="per-point sensor noise (1 sigma)")
    p.add_argument("--tilt-deg", type=float, default=20.0)
    p.add_argument("--flat", action="store_true", help="a flat plate with sensor noise only (noise scan)")
    p.add_argument("--seed", type=int, default=1)
    a = p.parse_args(argv)
    if a.flat:
        pts = make_ripple_scan(int(a.points), amplitude=0.0, small_sq=1e-12, bowl=0.0,
                               noise=a.noise_mm * 1e-3, tilt_deg=a.tilt_deg, seed=a.seed)
    else:
        pts = make_ripple_scan(int(a.points), wavelength=a.wavelength_mm * 1e-3,
                               amplitude=a.amplitude_mm * 1e-3, direction_deg=a.direction_deg,
                               noise=a.noise_mm * 1e-3, tilt_deg=a.tilt_deg, seed=a.seed)
    Path(a.output).parent.mkdir(parents=True, exist_ok=True)
    write_ply(a.output, pts)
    print(f"wrote {a.output}: {len(pts)} points")


if __name__ == "__main__":
    main()
