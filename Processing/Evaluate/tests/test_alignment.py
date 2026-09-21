"""Synthetic validation: sample a reference, move it by a known rigid transform
into a camera frame, add Gaussian noise, and check the program recovers the
transform and reports an RMSE close to the injected noise - against analytic
and mesh references alike."""

import numpy as np
import pytest
from conftest import SURFACES, fast_params, pose_error, synthetic_scan

from scan_eval.metrics import compute_metrics
from scan_eval.reference import UnitScaled
from scan_eval.registration import align, apply
from scan_eval.surface_def import load_reference

SIGMA = 0.0003   # 0.3 mm, isotropic


def _run(ref, scan, params, init=None):
    result = align(ref, scan, params, params.seed, init)
    metrics, _ = compute_metrics(ref, apply(result.transform, scan), params, params.seed)
    return result, metrics


def _check_recovery(ref, scan, T_true, result, metrics, sigma=SIGMA):
    lo, hi = ref.bounds()
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    corners_scan = apply(np.linalg.inv(T_true), corners)
    shift, angle = pose_error(result.transform, T_true, corners_scan)
    # Pose recovered to well under the noise, anywhere on the part.
    assert shift < 0.2e-3, f"pose off by {shift * 1e3:.3f} mm at the part's corners"
    assert angle < 0.1
    # Isotropic noise sigma -> distance to a smooth surface has RMS sigma.
    assert metrics["acc_rmse_mm"] == pytest.approx(sigma * 1e3, rel=0.08)
    assert abs(metrics["signed_bias_mm"]) < 0.05 * sigma * 1e3 + 0.01
    assert metrics["covered_fraction"] > 0.97
    assert metrics["outlier_fraction"] == 0.0
    assert not result.ambiguous


@pytest.mark.parametrize("which", ["analytic", "mesh"])
def test_egg_carton_recovers_transform_and_noise(which, egg, egg_mesh):
    ref = egg if which == "analytic" else egg_mesh
    scan, T_true = synthetic_scan(ref, sigma=SIGMA, seed=3)
    result, metrics = _run(ref, scan, fast_params())
    _check_recovery(ref, scan, T_true, result, metrics)


@pytest.mark.parametrize("which", ["analytic", "mesh"])
def test_one_period_shift_is_undone(which, egg, egg_mesh):
    """Points moved one full period (50 mm) along x: the relief lines up again
    at the shifted pose, but half the part now hangs off the reference. The
    program must find the true pose, not the look-alike."""
    ref = egg if which == "analytic" else egg_mesh
    scan, T_true = synthetic_scan(ref, sigma=SIGMA, seed=5, shift=[0.05, 0, 0])
    shifted_back = T_true.copy()
    shifted_back[:3, 3] -= [0.05, 0, 0]          # the pose that undoes the shift
    # Seed with the wrong (unshifted) pose to make it hard: ICP from there
    # converges to the look-alike, only the multi-start can escape it.
    result, metrics = _run(ref, scan, fast_params(), init=T_true)
    shift_err, _ = pose_error(result.transform, shifted_back, scan)
    assert shift_err < 0.3e-3, f"did not undo the one-period shift ({shift_err * 1e3:.2f} mm off)"
    assert result.best.overlap_ref > 0.95
    assert not result.ambiguous
    assert metrics["acc_rmse_mm"] == pytest.approx(SIGMA * 1e3, rel=0.08)


def test_partial_periodic_coverage_is_flagged_ambiguous(egg):
    """A scan of one period-wide strip of the egg-carton fits equally well one
    period over: the program cannot know which, and must say so."""
    strip = lambda p: (p[:, 0] > 0.0) & (p[:, 0] < 0.05)
    scan, T_true = synthetic_scan(egg, sigma=SIGMA, seed=7, keep=strip)
    result, _ = _run(egg, scan, fast_params())
    assert result.ambiguous, f"margin {result.margin}"
    # ...and the two contenders both fit, but put the strip in different places.
    for cand in (result.best, result.runner_up):
        assert cand.rmse < 1.5 * SIGMA and cand.overlap_scan > 0.95
    a = apply(result.best.transform, scan).mean(axis=0)
    b = apply(result.runner_up.transform, scan).mean(axis=0)
    assert np.linalg.norm(a - b) > 0.02


def test_plugin_reference():
    ref = UnitScaled(load_reference(SURFACES / "sphere_plugin.toml"))
    scan, T_true = synthetic_scan(ref, sigma=SIGMA, seed=11)
    result, metrics = _run(ref, scan, fast_params())
    # A sphere cap: rotation about its axis is free, so compare distances
    # only, and check the degeneracy report names it.
    assert metrics["acc_rmse_mm"] == pytest.approx(SIGMA * 1e3, rel=0.08)
    assert "rotation about Z" in result.degeneracy["unconstrained"] + result.degeneracy["weak"]


def test_parametric_and_implicit_references():
    for name in ("cylinder_section", "torus_implicit"):
        ref = UnitScaled(load_reference(SURFACES / f"{name}.toml"))
        scan, T_true = synthetic_scan(ref, sigma=SIGMA, seed=13, n=30000)
        result, metrics = _run(ref, scan, fast_params())
        assert metrics["acc_rmse_mm"] == pytest.approx(SIGMA * 1e3, rel=0.1), name


def test_degeneracy_plane_and_paraboloid(tmp_path):
    plane_def = tmp_path / "plane.toml"
    plane_def.write_text('name="plane"\ntype="height_field"\nunits="mm"\n[surface]\nz="0"\n'
                         '[domain]\nx=[0,100]\ny=[0,100]\n')
    plane = UnitScaled(load_reference(plane_def))
    scan, T_true = synthetic_scan(plane, sigma=SIGMA, seed=1, max_tilt_deg=0)
    result = align(plane, scan, fast_params(align="refine"), 0, T_true)
    free = result.degeneracy["unconstrained"]
    for dof in ("rotation about Z", "translation along X", "translation along Y"):
        assert dof in free, free
    assert "translation along Z" not in free

    para = UnitScaled(load_reference(SURFACES / "paraboloid.toml"))
    scan, T_true = synthetic_scan(para, sigma=SIGMA, seed=2)
    result = align(para, scan, fast_params(align="refine"), 0, T_true)
    assert result.degeneracy["unconstrained"] == ["rotation about Z"]


def test_allow_scale_recovers_scale(egg):
    scan, T_true = synthetic_scan(egg, sigma=SIGMA, seed=17, scale=1.02)
    result = align(egg, scan, fast_params(allow_scale=True), 0)
    assert result.scale == pytest.approx(1 / 1.02, abs=5e-4)
    rigid = align(egg, scan, fast_params(), 0)
    assert rigid.scale == pytest.approx(1.0, abs=1e-9)


def test_deterministic(egg):
    scan, _ = synthetic_scan(egg, sigma=SIGMA, seed=19)
    a = align(egg, scan, fast_params(), 0).transform
    b = align(egg, scan, fast_params(), 0).transform
    # Open3D's ICP sums in parallel threads, so the last bits can differ;
    # anything visible in a metric cannot.
    assert np.allclose(a, b, rtol=0, atol=1e-9)
