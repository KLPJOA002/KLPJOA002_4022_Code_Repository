"""Shared helpers: run the analysis on synthetic point arrays (no sensor, no
PLY) with the console output swallowed."""

import contextlib
import io
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import roughness  # noqa: E402,F401  (puts ../Evaluate on sys.path)
from roughness import synthetic as S  # noqa: E402
from roughness.params import RoughnessParams  # noqa: E402
from roughness.pipeline import analyse  # noqa: E402


def params(**overrides) -> RoughnessParams:
    p = RoughnessParams(sensor_origin="none", write_plots=False, bootstrap_samples=200)
    for k, v in overrides.items():
        setattr(p, k, v)
    return p


def run(points, **overrides):
    """Analysis of an (n, 3) array in metres, quietly."""
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return analyse(np.asarray(points, dtype=float), params(**overrides))


def flags(analysis, method="planar"):
    return {code for code, _ in analysis.warns[method]}


def sinusoid_points(A=1e-3, L=20e-3, size=(0.2, 0.16), spacing=0.25e-3, direction=0.0):
    xy = S.lattice_xy(size[0], size[1], spacing)
    return np.column_stack([xy, S.sinusoid(xy, A, L, direction)])


@pytest.fixture
def ripple_ply(tmp_path):
    path = tmp_path / "ripples.ply"
    S.write_ply(path, S.make_ripple_scan(n_points=60000, size=(0.24, 0.18), wavelength=0.03))
    return path


# --- a reference part (STL) and a scan of it, for the comparison and alignment tests
PART_MM = 100.0          # the reference part spans 0..100 mm in x and y
LAMBDA_MM = 25.0


def part_height_mm(x, y):
    """Egg-carton top 5 mm above the base, 2 mm amplitude, 25 mm period."""
    return 5.0 + 2.0 * np.sin(2 * np.pi * x / LAMBDA_MM) * np.sin(2 * np.pi * y / LAMBDA_MM)


def write_part_stl(path, height_mm):
    """Open triangulated height field z = height_mm(x, y) over 0..PART_MM, as an STL (mm)."""
    import open3d as o3d
    n = 201
    xs = np.linspace(0, PART_MM, n)
    X, Y = np.meshgrid(xs, xs)
    verts = np.column_stack([X.ravel(), Y.ravel(), height_mm(X, Y).ravel()])
    i = np.arange(n - 1)
    a = (i[:, None] * n + i[None, :]).ravel()
    tris = np.vstack([np.column_stack([a, a + 1, a + n + 1]), np.column_stack([a, a + n + 1, a + n])])
    mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(verts), o3d.utility.Vector3iVector(tris))
    mesh.compute_triangle_normals()
    assert o3d.io.write_triangle_mesh(str(path), mesh)
    return path


def write_part_scan(path, height_mm):
    """A scan already in the part's frame (metres) that also sees 30 mm of
    flat table around it, like Evaluate's aligned_error.ply."""
    rng = np.random.default_rng(1)
    xy = rng.uniform(-30, 130, (260000, 2))
    on = np.all((xy >= 0) & (xy <= PART_MM), axis=1)
    z = np.where(on, height_mm(xy[:, 0], xy[:, 1]), 0.0) + rng.normal(0, 0.02, len(xy))
    S.write_ply(path, np.column_stack([xy, z]) * 1e-3)
    return path


@pytest.fixture(scope="module")
def part_stl(tmp_path_factory):
    return write_part_stl(tmp_path_factory.mktemp("ref") / "part.stl", part_height_mm)


@pytest.fixture(scope="module")
def aligned_scan(tmp_path_factory):
    return write_part_scan(tmp_path_factory.mktemp("scan") / "aligned.ply", part_height_mm)
