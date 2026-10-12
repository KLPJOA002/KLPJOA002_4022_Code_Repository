"""Roughness metrics of a height map h (metres, NaN = invalid) on a square
grid of spacing g (metres). Everything is computed on the validity mask.

Height (ISO 25178-2)  Sa Sq Ssk Sku Sp Sv Sz
Area / slope          true area of the 2-triangles-per-quad mesh, rugosity,
                      Sdr, Sdq (RMS gradient) and mean slope
Spatial               masked FFT autocorrelation -> Sal, Str; Hann-windowed
                      masked PSD -> dominant wavelength / direction, Std,
                      anisotropy; radially averaged PSD -> power-law slope,
                      Hurst exponent, fractal dimension
Multi-scale           Sq and rugosity in non-overlapping windows of growing size
Bootstrap             95% CIs of Sq, Sdr, Sal by resampling windows
Ruggedness            TRI (Riley 1999), VRM (Sappington 2007)

Keys returned end in _mm / _deg / _pct where they carry units; the rest are
dimensionless.
"""

from __future__ import annotations

import warnings

import numpy as np
from scipy import fft as sfft
from scipy import ndimage


# --- height ------------------------------------------------------------------
def height_metrics(h, valid):
    z = h[valid]
    z = z - z.mean()
    sq = float(np.sqrt(np.mean(z ** 2)))
    sp, sv = float(z.max()), float(-z.min())
    return {
        "Sa_mm": float(np.mean(np.abs(z))) * 1e3, "Sq_mm": sq * 1e3,
        "Ssk": float(np.mean(z ** 3) / sq ** 3) if sq > 0 else float("nan"),
        "Sku": float(np.mean(z ** 4) / sq ** 4) if sq > 0 else float("nan"),
        "Sp_mm": sp * 1e3, "Sv_mm": sv * 1e3, "Sz_mm": (sp + sv) * 1e3,
    }


# --- area and slope ----------------------------------------------------------
def quads(z, g):
    """Per-quad (2x2 neighbouring cells) true area of its two triangles and
    gradient. Works on (..., H, W); NaN wherever a corner is invalid."""
    z00, z10 = z[..., :-1, :-1], z[..., :-1, 1:]      # x + 1 is the last axis
    z01, z11 = z[..., 1:, :-1], z[..., 1:, 1:]
    g2 = g * g
    area = 0.5 * g * (np.sqrt((z10 - z00) ** 2 + (z10 - z11) ** 2 + g2)
                      + np.sqrt((z01 - z11) ** 2 + (z01 - z00) ** 2 + g2))
    gx = ((z10 - z00) + (z11 - z01)) / (2 * g)
    gy = ((z01 - z00) + (z11 - z10)) / (2 * g)
    return area, gx, gy


def area_metrics(h, g):
    area, gx, gy = quads(h, g)
    ok = np.isfinite(area)
    if not ok.any():
        raise ValueError("no 2x2 block of valid cells - the height map is too sparse for area "
                         "metrics (use a coarser --grid-mm or a larger --max-hole-cells)")
    true, proj = float(area[ok].sum()), float(ok.sum() * g * g)
    rug = true / proj
    s2 = gx[ok] ** 2 + gy[ok] ** 2
    return {
        "true_area_mm2": true * 1e6, "projected_area_mm2": proj * 1e6, "rugosity": rug,
        "Sdr_pct": (rug - 1) * 100, "Sdq": float(np.sqrt(np.mean(s2))),
        "slope_mean_deg": float(np.degrees(np.mean(np.arctan(np.sqrt(s2))))),
    }


# --- autocorrelation ---------------------------------------------------------
def masked_acf(h, valid, lag_y, lag_x, min_overlap=0.2):
    """Normalised autocorrelation for lags |ty| <= lag_y, |tx| <= lag_x (cells),
    centred, over (..., H, W). Each lag is averaged over the cell pairs that
    are both valid; lags supported by fewer than min_overlap of the valid
    cells are NaN. h must be zero-mean over the mask."""
    H, W = h.shape[-2:]
    fy, fx = sfft.next_fast_len(H + lag_y + 1), sfft.next_fast_len(W + lag_x + 1)
    h0 = np.where(valid, h, 0.0)
    m = valid.astype(float)
    Fh = sfft.rfft2(h0, s=(fy, fx), axes=(-2, -1))
    Fm = sfft.rfft2(m, s=(fy, fx), axes=(-2, -1))
    num = sfft.irfft2(np.abs(Fh) ** 2, s=(fy, fx), axes=(-2, -1))
    cnt = np.rint(sfft.irfft2(np.abs(Fm) ** 2, s=(fy, fx), axes=(-2, -1)))
    iy = np.r_[fy - lag_y:fy, 0:lag_y + 1]
    ix = np.r_[fx - lag_x:fx, 0:lag_x + 1]
    num = num[..., iy, :][..., ix]
    cnt = cnt[..., iy, :][..., ix]
    n0 = cnt[..., lag_y, lag_x][..., None, None]
    var = (num[..., lag_y, lag_x][..., None, None]) / np.maximum(n0, 1)
    lagvalid = cnt >= min_overlap * n0
    with np.errstate(invalid="ignore", divide="ignore"):
        acf = num / np.maximum(cnt, 1) / var
    acf = np.where(lagvalid, acf, np.nan)
    return acf, lagvalid


def lag_radius(shape, g):
    H, W = shape[-2:]
    ry = (np.arange(H) - H // 2) * g
    rx = (np.arange(W) - W // 2) * g
    return np.hypot(ry[:, None], rx[None, :])


def acf_crossings(acf, lagvalid, s, g, above_mask=None):
    """Radii (m) where the ACF crosses s between 4-neighbouring lags, linearly
    interpolated: (..., K), NaN where no crossing."""
    R = lag_radius(acf.shape, g)
    above = lagvalid & (np.nan_to_num(acf, nan=-np.inf) > s)
    if above_mask is not None:
        above &= above_mask
    below = lagvalid & (np.nan_to_num(acf, nan=np.inf) <= s)
    lead = acf.shape[:-2]
    out = []
    pairs = [((slice(None), slice(None, -1)), (slice(None), slice(1, None))),
             ((slice(None, -1), slice(None)), (slice(1, None), slice(None)))]
    with np.errstate(invalid="ignore", divide="ignore"):
        for sa, sb in pairs:
            A, B = acf[(Ellipsis, *sa)], acf[(Ellipsis, *sb)]
            RA, RB = R[sa], R[sb]
            for (P, Q, RP, RQ, up, dn) in ((A, B, RA, RB, above[(Ellipsis, *sa)], below[(Ellipsis, *sb)]),
                                           (B, A, RB, RA, above[(Ellipsis, *sb)], below[(Ellipsis, *sa)])):
                t = (P - s) / (P - Q)
                r = np.where(up & dn, RP + t * (RQ - RP), np.nan)
                out.append(r.reshape(*lead, -1))
    return np.concatenate(out, axis=-1)


def _nanmin(a, axis=-1):
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        return np.nanmin(a, axis=axis)


def acf_metrics(h, valid, g, s):
    """Sal (fastest decay to s), Str (Sal / slowest decay within the central
    lobe), plus the ACF itself for the record."""
    H, W = h.shape
    acf, lagvalid = masked_acf(h, valid, H // 2, W // 2)
    cross = acf_crossings(acf, lagvalid, s, g)
    sal = float(_nanmin(cross))
    out = {"Sal_mm": sal * 1e3, "Str": float("nan"), "Str_bounded": False}
    cy, cx = H // 2, W // 2
    lab, _ = ndimage.label(lagvalid & (np.nan_to_num(acf, nan=-np.inf) > s))
    lobe = lab == lab[cy, cx]
    R = lag_radius(acf.shape, g)
    lobe_cross = acf_crossings(acf, lagvalid, s, g, lobe)
    rmax = float(np.nanmax(lobe_cross)) if np.isfinite(lobe_cross).any() else 0.0
    # Where the lobe runs into the edge of the usable lags the ACF has not
    # decayed there: the slowest decay is at least that far (Str is an upper bound).
    interior = ndimage.binary_erosion(np.pad(lagvalid, 1))[1:-1, 1:-1]
    touching = lobe & ~interior
    if touching.any():
        rmax = max(rmax, float(R[touching].max()))
        out["Str_bounded"] = True
    if np.isfinite(sal) and rmax > 0:
        out["Str"] = sal / rmax
    out["acf_decay_max_mm"] = rmax * 1e3
    return out, acf


# --- spectrum ----------------------------------------------------------------
def psd2d(h, valid, g):
    """Hann-windowed 2D PSD (m^4), mask-aware: invalid cells are zero and the
    normalisation counts only the valid, windowed cells, so the PSD integrates
    to the height variance. Returns (C, kx, ky) with zero frequency centred,
    k in cycles / m."""
    H, W = h.shape
    w = np.outer(np.hanning(H), np.hanning(W))
    wv = w * valid
    z = np.where(valid, h, 0.0)
    z = np.where(valid, z - (z * wv).sum() / wv.sum(), 0.0) * w
    F = sfft.fft2(z)
    C = (g * g) * np.abs(F) ** 2 / (wv ** 2).sum()
    C = sfft.fftshift(C)
    return C, sfft.fftshift(sfft.fftfreq(W, g)), sfft.fftshift(sfft.fftfreq(H, g))


def spectral_metrics(C, kx, ky, g, foot_min, foot_max, params, band):
    """Dominant wavelength / direction, Std, anisotropy, radial PSD fit."""
    KX, KY = np.meshgrid(kx, ky)
    K = np.hypot(KX, KY)
    half = (KY > 0) | ((KY == 0) & (KX > 0))
    k_lo = 1.0 / (params.wavelength_max_fraction * foot_min)
    k_hi = 1.0 / band[0]
    search = half & (K >= k_lo) & (K <= k_hi)
    out, warn_list = {}, []
    if search.sum() < 4:
        raise ValueError("too few frequencies between the grid and the footprint for spectral "
                         "metrics - use a finer --grid-mm or a larger area")
    i = int(np.argmax(np.where(search, C, -np.inf)))
    iy, ix = np.unravel_index(i, C.shape)
    ys = slice(max(iy - 1, 0), iy + 2)
    xs = slice(max(ix - 1, 0), ix + 2)
    wts = C[ys, xs]
    kxc, kyc = float((wts * KX[ys, xs]).sum() / wts.sum()), float((wts * KY[ys, xs]).sum() / wts.sum())
    kd = np.hypot(kxc, kyc)
    out["dominant_wavelength_mm"] = 1e3 / kd
    out["dominant_direction_deg"] = float(np.degrees(np.arctan2(kyc, kxc)) % 180)
    out["dominant_peak_ratio"] = float(C[iy, ix] / np.median(C[search]))
    dk = max(abs(kx[1] - kx[0]), abs(ky[1] - ky[0]))
    out["dominant_at_limit"] = bool(K[iy, ix] < k_lo + 1.5 * dk or K[iy, ix] > k_hi - 1.5 * dk)
    if out["dominant_at_limit"]:
        warn_list.append(("NOPEAK", "no clear spectral peak: the dominant wavelength is at the edge of "
                          f"the search range ({1e3 / k_hi:.3g}..{1e3 / k_lo:.3g} mm), so the surface "
                          "has no distinct ripple wavelength there (or it is longer than "
                          "wavelength_max_fraction x footprint)"))
    # Angular spectrum -> texture direction. Std is the lay (crest) direction,
    # perpendicular to the wave vector carrying the most power.
    theta = np.degrees(np.arctan2(KY[search], KX[search])) % 180
    hist = np.bincount(np.minimum(theta.astype(int), 179), weights=C[search], minlength=180)
    hist = np.convolve(np.r_[hist[-1], hist, hist[0]], [0.25, 0.5, 0.25], mode="valid")
    out["Std_deg"] = float((np.argmax(hist) + 0.5 + 90) % 180)
    # Spectral moment tensor (slope covariance): isotropic surfaces have equal
    # eigenvalues; ripples put nearly all power on one axis.
    c = C[search]
    M = np.array([[np.sum(c * KX[search] ** 2), np.sum(c * KX[search] * KY[search])],
                  [np.sum(c * KX[search] * KY[search]), np.sum(c * KY[search] ** 2)]])
    ev = np.linalg.eigvalsh(M)
    out["anisotropy_ratio"] = float(np.sqrt(max(ev[0], 0) / ev[1])) if ev[1] > 0 else float("nan")

    # Radially averaged PSD and its power-law fit.
    edges = np.geomspace(1.0 / foot_max, 0.5 / g, params.psd_bins + 1)
    b = np.digitize(K.ravel(), edges) - 1
    ok = (b >= 0) & (b < params.psd_bins)
    cnt = np.bincount(b[ok], minlength=params.psd_bins)
    with np.errstate(invalid="ignore", divide="ignore"):
        kc = np.bincount(b[ok], weights=K.ravel()[ok], minlength=params.psd_bins) / cnt
        pc = np.bincount(b[ok], weights=C.ravel()[ok], minlength=params.psd_bins) / cnt
    radial = {"k_per_mm": kc * 1e-3, "wavelength_mm": 1e3 / kc, "psd_mm4": pc * 1e12, "count": cnt}
    sel = (cnt > 0) & (kc >= 1.0 / band[1]) & (kc <= 1.0 / band[0]) & (pc > 0)
    fit = {"band_mm": [band[0] * 1e3, band[1] * 1e3], "n_bins": int(sel.sum())}
    out.update({"psd_slope": float("nan"), "hurst": float("nan"),
                "fractal_dimension": float("nan"), "psd_fit_r2": float("nan")})
    if sel.sum() >= 4:
        lx, ly = np.log10(kc[sel]), np.log10(pc[sel])
        slope, icpt = np.polyfit(lx, ly, 1)
        pred = slope * lx + icpt
        r2 = 1 - np.sum((ly - pred) ** 2) / max(np.sum((ly - ly.mean()) ** 2), 1e-300)
        hurst = -slope / 2 - 1
        out.update({"psd_slope": float(slope), "hurst": float(hurst),
                    "fractal_dimension": float(3 - hurst), "psd_fit_r2": float(r2)})
        fit.update({"slope": slope, "intercept_log10": icpt})
        if not 0 <= hurst <= 1:
            warn_list.append(("HURST", f"Hurst exponent {hurst:.2f} is outside the self-affine range "
                              "0..1 over the PSD band: the surface is not a simple fractal there "
                              "(e.g. a ripple peak or noise floor inside the band)"))
    else:
        warn_list.append(("BAND", f"only {int(sel.sum())} PSD bins between {band[0] * 1e3:.3g} and "
                          f"{band[1] * 1e3:.3g} mm - no power-law fit (widen --psd-band-mm)"))
    return out, {"radial_psd": radial, "psd_fit": fit}, warn_list


# --- multi-scale and bootstrap ------------------------------------------------
def blocks(a, w):
    """Non-overlapping w x w windows (centred tiling) -> (n, w, w)."""
    H, W = a.shape
    by, bx = H // w, W // w
    oy, ox = (H - by * w) // 2, (W - bx * w) // 2
    a = a[oy:oy + by * w, ox:ox + bx * w]
    return a.reshape(by, w, bx, w).swapaxes(1, 2).reshape(by * bx, w, w)


def window_sizes(n_min_side, count):
    top = n_min_side // 2
    if top < 4:
        return []
    return sorted({int(round(s)) for s in np.geomspace(4, top, max(count, 1))})


def window_residuals(h, valid, w, min_valid, mode):
    B, V = blocks(h, w), blocks(valid, w)
    use = V.mean(axis=(1, 2)) >= min_valid
    B, V = B[use], V[use]
    n = len(B)
    if n == 0:
        return B, V
    z = np.where(V, B, 0.0).reshape(n, -1)
    v = V.reshape(n, -1).astype(float)
    if mode == "plane":
        yy, xx = np.mgrid[0:w, 0:w]
        X = np.stack([np.ones(w * w), xx.ravel() - (w - 1) / 2, yy.ravel() - (w - 1) / 2], axis=1)
        A = np.einsum("np,pi,pj->nij", v, X, X)
        b = np.einsum("np,pi->ni", v * z, X)
        coef = np.linalg.solve(A, b[..., None])[..., 0]
        z = z - coef @ X.T
    else:
        z = z - (z * v).sum(axis=1, keepdims=True) / v.sum(axis=1, keepdims=True)
    return np.where(V, z.reshape(n, w, w), np.nan), V


def multiscale(h, valid, g, params):
    rows = []
    for w in window_sizes(min(h.shape), params.multiscale_sizes):
        R, V = window_residuals(h, valid, w, params.window_min_valid, params.window_detrend)
        if len(R) == 0:
            continue
        sq = np.sqrt(np.nanmean(R ** 2, axis=(1, 2)))
        area, _, _ = quads(R, g)
        ok = np.isfinite(area)
        rug = np.nansum(area, axis=(1, 2)) / np.maximum(ok.sum(axis=(1, 2)), 1) / (g * g)
        rows.append({"window_cells": w, "window_mm": w * g * 1e3, "n_windows": int(len(R)),
                     "Sq_mean_mm": float(sq.mean() * 1e3), "Sq_std_mm": float(sq.std() * 1e3),
                     "rugosity_mean": float(rug.mean()), "rugosity_std": float(rug.std())})
    out = {}
    if rows:
        out = {"ms_window_min_mm": rows[0]["window_mm"], "ms_window_max_mm": rows[-1]["window_mm"],
               "ms_Sq_min_window_mm": rows[0]["Sq_mean_mm"], "ms_Sq_max_window_mm": rows[-1]["Sq_mean_mm"],
               "ms_rugosity_min_window": rows[0]["rugosity_mean"],
               "ms_rugosity_max_window": rows[-1]["rugosity_mean"]}
        if len(rows) >= 3:
            lw = np.log10([r["window_mm"] for r in rows])
            ls = np.log10([max(r["Sq_mean_mm"], 1e-12) for r in rows])
            out["ms_Sq_scaling_exponent"] = float(np.polyfit(lw, ls, 1)[0])
    return out, rows


def bootstrap(h, valid, g, w, params, rng):
    """95% percentile CIs by resampling w x w windows with replacement.
    Sq and Sdr use the globally detrended heights pooled over the resampled
    windows, so their point estimate is the global value; Sal is the mean of
    the per-window Sal."""
    B, V = blocks(h, w), blocks(valid, w)
    use = V.mean(axis=(1, 2)) >= params.window_min_valid
    B, V = B[use], V[use]
    n = len(B)
    out = {"bootstrap_window_mm": w * g * 1e3, "n_windows_used": int(n)}
    for k in ("Sq_ci95_lo_mm", "Sq_ci95_hi_mm", "Sdr_ci95_lo_pct", "Sdr_ci95_hi_pct",
              "Sal_ci95_lo_mm", "Sal_ci95_hi_mm"):
        out[k] = float("nan")
    if n < 2:
        return out
    Bz = np.where(V, B, np.nan)
    sumsq = np.nansum(Bz ** 2, axis=(1, 2))
    cnt = V.sum(axis=(1, 2))
    area, _, _ = quads(Bz, g)
    asum = np.nansum(area, axis=(1, 2))
    qn = np.isfinite(area).sum(axis=(1, 2))
    Bm = np.where(V, Bz - np.nanmean(Bz, axis=(1, 2), keepdims=True), 0.0)
    acf, lv = masked_acf(Bm, V, w // 2, w // 2)
    sal = _nanmin(acf_crossings(acf, lv, params.acf_threshold, g))
    idx = rng.integers(0, n, (params.bootstrap_samples, n))
    sq_b = np.sqrt(sumsq[idx].sum(1) / cnt[idx].sum(1))
    sdr_b = (asum[idx].sum(1) / np.maximum(qn[idx].sum(1), 1) / (g * g) - 1) * 100
    out["Sq_ci95_lo_mm"], out["Sq_ci95_hi_mm"] = (np.percentile(sq_b, [2.5, 97.5]) * 1e3).tolist()
    out["Sdr_ci95_lo_pct"], out["Sdr_ci95_hi_pct"] = np.percentile(sdr_b, [2.5, 97.5]).tolist()
    fin = sal[np.isfinite(sal)]
    if len(fin) >= 2:
        sal_b = fin[rng.integers(0, len(fin), (params.bootstrap_samples, len(fin)))].mean(1)
        out["Sal_ci95_lo_mm"], out["Sal_ci95_hi_mm"] = (np.percentile(sal_b, [2.5, 97.5]) * 1e3).tolist()
    return out


def choose_bootstrap_window(h, valid, g, params):
    if params.bootstrap_window_mm > 0:
        return max(4, int(round(params.bootstrap_window_mm * 1e-3 / g))), None
    best, most = None, (0, None)
    for w in window_sizes(min(h.shape), params.multiscale_sizes):
        V = blocks(valid, w)
        n = int((V.mean(axis=(1, 2)) >= params.window_min_valid).sum())
        if n >= params.bootstrap_min_windows:
            best = w
        if n > most[0]:
            most = (n, w)
    if best is not None:
        return best, None
    return most[1], (f"fewer than {params.bootstrap_min_windows} usable windows at every size; "
                     "the bootstrap CIs rest on few windows")


# --- ruggedness --------------------------------------------------------------
def ruggedness(h, valid, g, n):
    """Mean TRI (Riley: sqrt of summed squared differences to the centre cell
    over the n x n window) and VRM (1 - resultant length of the unit normals)."""
    N = n * n
    v = valid.astype(float)
    z = np.where(valid, h, 0.0)
    full = (ndimage.uniform_filter(v, n, mode="constant") * N > N - 0.5) & valid
    s1 = ndimage.uniform_filter(z, n, mode="constant") * N
    s2 = ndimage.uniform_filter(z * z, n, mode="constant") * N
    tri = np.sqrt(np.maximum(s2 - 2 * z * s1 + N * z * z, 0))[full]
    gy, gx = np.gradient(np.where(valid, h, np.nan), g)
    norm = np.sqrt(gx ** 2 + gy ** 2 + 1)
    comps = [-gx / norm, -gy / norm, 1 / norm]
    okn = np.isfinite(comps[2])
    fulln = ndimage.uniform_filter(okn.astype(float), n, mode="constant") * N > N - 0.5
    avg = [ndimage.uniform_filter(np.where(okn, c, 0.0), n, mode="constant") for c in comps]
    vrm = (1 - np.sqrt(avg[0] ** 2 + avg[1] ** 2 + avg[2] ** 2))[fulln & okn]
    return {"TRI_mm": float(tri.mean() * 1e3) if len(tri) else float("nan"),
            "VRM": float(vrm.mean()) if len(vrm) else float("nan"),
            "ruggedness_window_cells": n}


# --- everything --------------------------------------------------------------
def footprint(valid, g):
    cols, rows = np.flatnonzero(valid.any(axis=0)), np.flatnonzero(valid.any(axis=1))
    return (cols[-1] - cols[0] + 1) * g, (rows[-1] - rows[0] + 1) * g


def psd_band(params, g, foot_min):
    lo, hi = (params.psd_band_mm or [0, 0])[:2]
    return (lo * 1e-3 if lo else 4 * g), (hi * 1e-3 if hi else foot_min / 4)


def surface_metrics(h, valid, g, params, rng):
    """All metrics for one height map. Returns (scalars, curves, aux, warnings)
    where aux holds arrays for the plots only and warnings are (code, text)."""
    warn_list = []
    h = np.where(valid, h - h[valid].mean(), np.nan)
    fx, fy = footprint(valid, g)
    foot_min, foot_max = min(fx, fy), max(fx, fy)
    band = psd_band(params, g, foot_min)
    out = {}
    out.update(height_metrics(h, valid))
    out.update(area_metrics(h, g))
    acf_out, acf = acf_metrics(h, valid, g, params.acf_threshold)
    out.update(acf_out)
    if not np.isfinite(out["Sal_mm"]):
        warn_list.append(("NODECAY", f"the autocorrelation never falls to {params.acf_threshold} "
                          "within half the footprint - Sal / Str undefined; the footprint is "
                          "smaller than the surface's correlation length"))
    C, kx, ky = psd2d(h, valid, g)
    spec, curves, w2 = spectral_metrics(C, kx, ky, g, foot_min, foot_max, params, band)
    out.update(spec)
    warn_list += w2
    out["bandwidth_min_mm"], out["bandwidth_max_mm"] = band[0] * 1e3, band[1] * 1e3
    ms, ms_rows = multiscale(h, valid, g, params)
    out.update(ms)
    curves["multiscale"] = ms_rows
    w, note = choose_bootstrap_window(h, valid, g, params)
    if note:
        warn_list.append(("FEWWIN", note))
    if w is not None:
        out.update(bootstrap(h, valid, g, w, params, rng))
    else:
        out["n_windows_used"] = 0
    if params.ruggedness_window and params.ruggedness_window > 1:
        out.update(ruggedness(h, valid, g, int(params.ruggedness_window)))
    lam = out["dominant_wavelength_mm"] * 1e-3
    if not out.get("dominant_at_limit") and foot_min < params.footprint_wavelengths_min * lam:
        warn_list.append(("FOOTPRINT", f"footprint short side {foot_min * 1e3:.1f} mm is only "
                          f"{foot_min / lam:.1f}x the dominant wavelength {lam * 1e3:.1f} mm (want >= "
                          f"{params.footprint_wavelengths_min:g}x): spectral and ACF metrics are unstable"))
    aux = {"psd2d": C, "kx": kx, "ky": ky, "acf": acf, "h": h}
    return out, curves, aux, warn_list
