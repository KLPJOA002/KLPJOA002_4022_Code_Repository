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
