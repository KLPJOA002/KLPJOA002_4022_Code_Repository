"""Metrics against surfaces with known answers."""

import numpy as np
import pytest
from conftest import S, flags, run, sinusoid_points
from scipy.special import ellipe

METHODS = ["planar", "quadratic", "cubic", "quartic"]


# --- detrending --------------------------------------------------------------
@pytest.fixture(scope="module")
def rough_field():
    """Isotropic random roughness, Sq 0.5 mm at 2-10 mm wavelengths, on a
    200 x 200 mm lattice, plus its own Sq with nothing to detrend."""
    xy = S.lattice_xy(0.2, 0.2, 0.4e-3)
    z = S.RandomField(0.5e-3, 2e-3, 10e-3, n_modes=300, seed=5)(xy)
    truth = run(np.column_stack([xy, z]), grid_mm=0.8).results["planar"]["Sq_mm"]
    return xy, z, truth


@pytest.mark.parametrize("order", [1, 2, 3, 4])
def test_matching_detrend_recovers_known_sq(rough_field, order):
    xy, z, truth = rough_field
    trend = S.polynomial_trend(xy, order, 0.2, 0.02, seed=order)      # 20 mm peak-to-peak
    a = run(np.column_stack([xy, z + trend]), grid_mm=0.8, detrend=METHODS)
    sq = {m: a.results[m]["Sq_mm"] for m in METHODS}
    for m in METHODS[order - 1:]:                 # the matching order and anything higher
        assert sq[m] == pytest.approx(truth, rel=0.03), (m, sq, truth)
    if order >= 2:                                 # one order too low leaves trend behind
        assert sq[METHODS[order - 2]] > 1.2 * truth, (sq, truth)
    r = a.results[METHODS[order - 1]]
    assert r["detrend_n_coef"] == (order + 1) * (order + 2) // 2
    assert r["trend_variance_removed"] > 0.5 or order == 1      # order 1: the orientation removed it


def test_detrend_warns_when_footprint_is_short(rough_field):
    """Over ~1.4 ripple wavelengths a cubic / quartic fits the ripple itself
    (it removes most of the variance) and says so."""
    xy = S.lattice_xy(0.1, 0.1, 0.4e-3)
    z = S.sinusoid(xy, 2e-3, 70e-3)
    a = run(np.column_stack([xy, z]), grid_mm=0.8, detrend=METHODS, expected_wavelength_mm=70)
    for m in ("cubic", "quartic"):
        assert "DETREND" in flags(a, m)
        assert a.results[m]["Sq_mm"] < 0.5 * a.results["planar"]["Sq_mm"]
    for m in ("planar", "quadratic"):             # odd ripple: nothing for these to absorb
        assert "DETREND" not in flags(a, m)


# --- sinusoid ----------------------------------------------------------------
@pytest.fixture(scope="module")
def sinusoid():
    A, L = 1e-3, 20e-3
    return A, L, run(sinusoid_points(A, L), grid_mm=0.5).results["planar"]


def test_sinusoid_height_metrics(sinusoid):
    A, L, r = sinusoid
    # Cells average 2 x 2 lattice points 0.25 mm apart: cos(2 pi 0.125 / 20) = 0.997.
    assert r["Sq_mm"] == pytest.approx(A / np.sqrt(2) * 1e3, rel=0.01)
    assert r["Sa_mm"] == pytest.approx(2 * A / np.pi * 1e3, rel=0.01)
    assert r["Ssk"] == pytest.approx(0, abs=0.02)
    assert r["Sku"] == pytest.approx(1.5, rel=0.02)


def test_sinusoid_dominant_wavelength_and_direction(sinusoid):
    A, L, r = sinusoid
    assert r["dominant_wavelength_mm"] == pytest.approx(L * 1e3, rel=0.01)
    assert min(r["dominant_direction_deg"], 180 - r["dominant_direction_deg"]) < 1
    assert r["Std_deg"] == pytest.approx(90, abs=2)          # crests run along y
    assert not r["dominant_at_limit"]


def test_sinusoid_sdr_matches_arc_length(sinusoid):
    A, L, r = sinusoid
    m = 2 * np.pi * A / L
    # Mean arc length of A sin(2 pi x / L) per unit x = (2/pi) sqrt(1+m^2) E(m^2 / (1+m^2)).
    sdr = (2 / np.pi * np.sqrt(1 + m * m) * ellipe(m * m / (1 + m * m)) - 1) * 100
    assert r["Sdr_pct"] == pytest.approx(sdr, rel=0.02)
    assert r["Sdq"] == pytest.approx(m / np.sqrt(2), rel=0.02)


def test_sinusoid_is_anisotropic(sinusoid):
    A, L, r = sinusoid
    assert r["Str"] < 0.15
    assert r["anisotropy_ratio"] < 0.2
    # ACF of a sinusoid is cos(2 pi t / L): falls to 0.2 at t = L acos(0.2) / 2 pi.
    assert r["Sal_mm"] == pytest.approx(np.arccos(0.2) / (2 * np.pi) * L * 1e3, rel=0.03)


def test_oblique_sinusoid_direction():
    r = run(sinusoid_points(direction=30.0), grid_mm=0.5).results["planar"]
    assert r["dominant_direction_deg"] == pytest.approx(30, abs=2)
    assert r["Std_deg"] == pytest.approx(120, abs=3)
    assert r["dominant_wavelength_mm"] == pytest.approx(20, rel=0.02)


# --- fractal -----------------------------------------------------------------
@pytest.mark.parametrize("hurst", [0.5, 0.8])
def test_hurst_exponent_recovered(hurst):
    g, n = 0.5e-3, 512
    z = S.fractal_grid(n, g, hurst, 1e-3, seed=3)
    xy = S.lattice_xy(n * g, n * g, g)
    r = run(np.column_stack([xy, z.ravel()]), grid_mm=0.5).results["planar"]
    assert r["hurst"] == pytest.approx(hurst, abs=0.1)
    assert r["psd_slope"] == pytest.approx(-2 * (1 + hurst), abs=0.2)
    assert r["fractal_dimension"] == pytest.approx(3 - hurst, abs=0.1)
    assert r["psd_fit_r2"] > 0.95
    assert r["Str"] > 0.3                 # isotropic (one realisation: noisy)


# --- noise -------------------------------------------------------------------
def test_white_noise_sq_is_sigma():
    sigma = 0.5e-3
    rng = np.random.default_rng(0)
    xy = S.lattice_xy(0.2, 0.2, 1e-3)                      # exactly one point per 1 mm cell
    r = run(np.column_stack([xy, rng.normal(0, sigma, len(xy))]), grid_mm=1.0,
            noise_sigma_mm=0.5).results["planar"]
    assert r["Sq_mm"] == pytest.approx(sigma * 1e3, rel=0.03)
    assert r["noise_sigma_map_mm"] == pytest.approx(sigma * 1e3, rel=1e-6)
    assert r["Sq_noisecorr_mm"] < 0.1 * r["Sq_mm"]
    assert r["noise_inflated"]


def test_noise_inflates_slopes_with_grid_refinement():
    sigma = 0.5e-3
    rng = np.random.default_rng(1)
    xy = S.uniform_xy(400000, 0.2, 0.2, rng)
    pts = np.column_stack([xy, rng.normal(0, sigma, len(xy))])
    res = [run(pts, grid_mm=g, noise_sigma_mm=0.5).results["planar"] for g in (4.0, 2.0, 1.0)]
    for coarse, fine in zip(res, res[1:]):
        assert fine["Sdq"] > 2 * coarse["Sdq"]
        assert fine["rugosity"] > coarse["rugosity"]
    for r in res:
        # Map noise = sigma / sqrt(points per cell); after correction ~nothing is left.
        assert r["Sq_mm"] == pytest.approx(r["noise_sigma_map_mm"], rel=0.05)
        assert r["Sq_noisecorr_mm"] < 0.25 * r["Sq_mm"]


def test_noise_correction_recovers_sinusoid():
    A, L, sigma = 1e-3, 20e-3, 0.3e-3
    rng = np.random.default_rng(2)
    pts = sinusoid_points(A, L, spacing=0.5e-3)
    pts[:, 2] += rng.normal(0, sigma, len(pts))
    r = run(pts, grid_mm=0.5, noise_sigma_mm=0.3).results["planar"]
    assert r["Sq_mm"] == pytest.approx(np.hypot(A / np.sqrt(2), sigma) * 1e3, rel=0.03)
    assert r["Sq_noisecorr_mm"] == pytest.approx(A / np.sqrt(2) * 1e3, rel=0.03)


# --- holes and edges ---------------------------------------------------------
@pytest.fixture(scope="module")
def textured():
    rng = np.random.default_rng(4)
    xy = S.uniform_xy(500000, 0.2, 0.16, rng)
    z = S.sinusoid(xy, 1e-3, 20e-3) + S.RandomField(0.2e-3, 2e-3, 8e-3, n_modes=200, seed=9)(xy)
    return xy, z, run(np.column_stack([xy, z]), grid_mm=1.0).results["planar"]


KEYS = [("Sq_mm", 0.02), ("Sa_mm", 0.02), ("Sdr_pct", 0.04), ("Sdq", 0.02),
        ("dominant_wavelength_mm", 0.02), ("Sal_mm", 0.05)]


def test_holes_do_not_change_metrics(textured):
    xy, z, base = textured
    rng = np.random.default_rng(6)
    small = rng.uniform(-0.09, 0.09, (150, 2))            # 1-2 cell dropouts: interpolated
    large = rng.uniform(-0.07, 0.07, (4, 2))              # 16 mm holes: masked
    keep = S.punch_holes(xy, small, 1.0e-3) & S.punch_holes(xy, large, 8e-3)
    a = run(np.column_stack([xy, z])[keep], grid_mm=1.0)
    r = a.results["planar"]
    assert a.gstats["holes_filled"] > 50
    assert a.gstats["holes_left"] >= 4
    assert 0 < a.gstats["interpolated_fraction"] < 0.05
    assert a.gstats["valid_fraction"] < 0.98
    for k, tol in KEYS:
        assert r[k] == pytest.approx(base[k], rel=tol), k


def test_edge_trim_does_not_change_metrics(textured):
    xy, z, base = textured
    a = run(np.column_stack([xy, z]), grid_mm=1.0, edge_trim_mm=10)
    assert a.gstats["footprint_x_mm"] == pytest.approx(180, abs=2)
    assert a.gstats["footprint_y_mm"] == pytest.approx(140, abs=2)
    for k, tol in KEYS:
        assert a.results["planar"][k] == pytest.approx(base[k], rel=max(tol, 0.03)), k


def test_roi_crops_in_the_oriented_frame(textured):
    xy, z, _ = textured
    a = run(np.column_stack([xy, z]), grid_mm=1.0, roi_mm=[-50, -40, 50, 40])
    assert a.gstats["footprint_x_mm"] == pytest.approx(100, abs=2)
    assert a.gstats["footprint_y_mm"] == pytest.approx(80, abs=2)


# --- orientation -------------------------------------------------------------
def test_tilted_camera_frame_is_levelled():
    A, L = 1e-3, 20e-3
    pts = sinusoid_points(A, L, spacing=0.5e-3)
    cam = S.to_camera_frame(pts, tilt_deg=25)
    a = run(cam, grid_mm=0.5, sensor_origin="0,0,0")
    r = a.results["planar"]
    assert a.prep.frame.tilt_deg == pytest.approx(25, abs=0.2)
    assert r["Sq_mm"] == pytest.approx(A / np.sqrt(2) * 1e3, rel=0.02)
    assert r["dominant_direction_deg"] == pytest.approx(0, abs=1) or r["dominant_direction_deg"] > 179
    assert r["Ssk"] == pytest.approx(0, abs=0.05)
