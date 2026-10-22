"""Plots, in scan_eval.report's style and palette: blue below the mean
surface, red above, neutral grey at zero; light surface, quiet axes."""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import warnings  # noqa: E402

from scan_eval.report import BLUE, ERROR_CMAP, GRID, INK, INK_2, SURFACE, _style, error_limit  # noqa: E402

HEIGHT_CMAP = ERROR_CMAP.with_extremes(bad=SURFACE)


def _limit(h_mm):
    return error_limit(h_mm[np.isfinite(h_mm)])


def _map(ax, fig, h_mm, extent, lim, title):
    _style(ax)
    im = ax.imshow(h_mm, origin="lower", extent=extent, cmap=HEIGHT_CMAP, vmin=-lim, vmax=lim,
                   interpolation="nearest")
    ax.set_aspect("equal")
    ax.set_xlabel("x, mm")
    ax.set_ylabel("y, mm")
    ax.set_title(title, loc="left", fontsize=10, color=INK)
    cb = fig.colorbar(im, ax=ax, shrink=0.8, pad=0.02)
    cb.set_label("height, mm", color=INK_2)
    cb.ax.tick_params(colors=INK_2, labelsize=8)
    cb.outline.set_visible(False)


def plot_heightmap(path, h_mm, extent, title, m):
    fig, ax = plt.subplots(figsize=(7.2, 6.0), dpi=130, facecolor=SURFACE)
    _map(ax, fig, h_mm, extent, _limit(h_mm), "residual height (detrended)")
    ax.text(0.0, -0.13, f"Sq {m['Sq_mm']:.3f} mm   Sa {m['Sa_mm']:.3f} mm   Sdr {m['Sdr_pct']:.2f}%   "
            f"Sal {m['Sal_mm']:.2f} mm   dominant {m['dominant_wavelength_mm']:.1f} mm",
            transform=ax.transAxes, fontsize=9, color=INK, va="top")
    fig.suptitle(title, x=0.01, ha="left", fontsize=11, color=INK)
    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


def plot_detrend_comparison(path, maps, metrics, extent, title, primary):
    """Residual maps side by side on one colour scale, plus Sq and Sdr bars."""
    methods = list(maps)
    n = len(methods)
    lim = max(_limit(maps[k]) for k in methods)
    fig = plt.figure(figsize=(4.6 * n + 0.6, 7.8), dpi=120, facecolor=SURFACE)
    gs = fig.add_gridspec(2, max(n, 2), height_ratios=[3, 1.4])
    for i, k in enumerate(methods):
        ax = fig.add_subplot(gs[0, i])
        m = metrics[k]
        _map(ax, fig, maps[k], extent, lim,
             f"{k}{' (primary)' if k == primary else ''}\nSq {m['Sq_mm']:.3f} mm   Sdr {m['Sdr_pct']:.2f}%")
    for j, (key, label) in enumerate([("Sq_mm", "Sq, mm"), ("Sdr_pct", "Sdr, %")]):
        ax = fig.add_subplot(gs[1, :max(n, 2) // 2] if j == 0 else gs[1, max(n, 2) // 2:])
        _style(ax)
        vals = [metrics[k][key] for k in methods]
        ax.bar(methods, vals, color=[BLUE if k == primary else INK_2 for k in methods], width=0.6)
        for x, v in enumerate(vals):
            ax.text(x, v, f"{v:.3g}", ha="center", va="bottom", fontsize=8, color=INK)
        ax.set_ylabel(label)
        ax.grid(axis="y", color=GRID, lw=0.6)
        ax.set_axisbelow(True)
    fig.suptitle(title, x=0.01, ha="left", fontsize=11, color=INK)
    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


def plot_psd(path, aux, curves, m, title):
    C, kx, ky = aux["psd2d"], aux["kx"] * 1e-3, aux["ky"] * 1e-3      # cycles / mm
    fig, axes = plt.subplots(1, 2, figsize=(12.5, 5.2), dpi=130, facecolor=SURFACE)
    ax = axes[0]
    _style(ax)
    # Zoom on the dominant peak (4x its wavenumber), within the fit band.
    kmax = min(4.0 / m["dominant_wavelength_mm"], 1.0 / m["bandwidth_min_mm"], kx.max(), ky.max())
    sx = np.abs(kx) <= kmax
    sy = np.abs(ky) <= kmax
    sub = C[np.ix_(sy, sx)] * 1e12
    logp = np.log10(np.maximum(sub, 1e-300))
    # Colour floor at the median: the background reads light, power dark.
    im = ax.imshow(logp, origin="lower", extent=[kx[sx][0], kx[sx][-1], ky[sy][0], ky[sy][-1]],
                   cmap="Greys", vmin=np.median(logp), vmax=logp.max(), interpolation="nearest")
    lam, ang = m["dominant_wavelength_mm"], np.radians(m["dominant_direction_deg"])
    for sgn in (1, -1):
        ax.plot(sgn * np.cos(ang) / lam, sgn * np.sin(ang) / lam, "o", mfc="none", mec=BLUE, ms=10, mew=1.5)
    ax.set_aspect("equal")
    ax.set_xlabel("kx, cycles / mm")
    ax.set_ylabel("ky, cycles / mm")
    ax.set_title(f"2D PSD (log10 mm^4); dominant {lam:.1f} mm at {m['dominant_direction_deg']:.0f} deg",
                 loc="left", fontsize=10, color=INK)
    cb = fig.colorbar(im, ax=ax, shrink=0.8, pad=0.02)
    cb.ax.tick_params(colors=INK_2, labelsize=8)
    cb.outline.set_visible(False)

    ax = axes[1]
    _style(ax)
    r = curves["radial_psd"]
    ok = np.isfinite(r["psd_mm4"]) & (r["count"] > 0)
    ax.loglog(r["k_per_mm"][ok], r["psd_mm4"][ok], color=BLUE, lw=1.6, label="radially averaged PSD")
    lo, hi = m["bandwidth_min_mm"], m["bandwidth_max_mm"]
    ax.axvspan(1 / hi, 1 / lo, color=GRID, alpha=0.6, lw=0, label="fit band")
    fit = curves["psd_fit"]
    if "slope" in fit:
        kk = np.array([1 / hi, 1 / lo]) * 1e3          # cycles / m, as fitted
        pp = 10 ** (fit["slope"] * np.log10(kk) + fit["intercept_log10"]) * 1e12
        ax.loglog(kk * 1e-3, pp, color=INK, lw=1.2, ls=(0, (4, 2)),
                  label=f"slope {m['psd_slope']:.2f}  H {m['hurst']:.2f}  D {m['fractal_dimension']:.2f}")
    ax.set_xlabel("spatial frequency |k|, cycles / mm")
    ax.set_ylabel("PSD, mm^4")
    ax.grid(color=GRID, lw=0.6, which="major")
    ax.legend(frameon=False, fontsize=9, labelcolor=INK_2)
    sec = ax.secondary_xaxis("top", functions=(lambda k: 1 / np.maximum(k, 1e-12),
                                               lambda l: 1 / np.maximum(l, 1e-12)))
    sec.set_xlabel("wavelength, mm", color=INK_2)
    sec.tick_params(colors=INK_2, labelsize=8)
    fig.suptitle(title, x=0.01, ha="left", fontsize=11, color=INK)
    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


def plot_multiscale(path, rows, title):
    if not rows:
        return
    w = np.array([r["window_mm"] for r in rows])
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.4), dpi=130, facecolor=SURFACE)
    for ax, key, label in ((axes[0], "Sq", "Sq, mm"), (axes[1], "rugosity", "rugosity (true / projected area)")):
        _style(ax)
        mean = np.array([r[f"{key}_mean_mm" if key == "Sq" else f"{key}_mean"] for r in rows])
        std = np.array([r[f"{key}_std_mm" if key == "Sq" else f"{key}_std"] for r in rows])
        ax.fill_between(w, mean - std, mean + std, color=BLUE, alpha=0.15, lw=0)
        ax.plot(w, mean, "o-", color=BLUE, ms=4, lw=1.5)
        ax.set_xscale("log")
        if key == "Sq":
            ax.set_yscale("log")
        ax.set_xlabel("window size, mm")
        ax.set_ylabel(label)
        ax.grid(color=GRID, lw=0.6)
        ax.set_axisbelow(True)
    axes[0].set_title("mean +- std across windows", loc="left", fontsize=10, color=INK)
    fig.suptitle(title, x=0.01, ha="left", fontsize=11, color=INK)
    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


# --- evaluated area, detrending, reference -----------------------------------
# Detrend methods keep one colour each, in a fixed order (categorical slots
# 1-4), and a line style each so profiles still read in greyscale / for CVD.
METHOD_COLORS = {"planar": "#2a78d6", "quadratic": "#eb6834", "cubic": "#1baf7a", "quartic": "#eda100"}
METHOD_STYLES = {"planar": "-", "quadratic": (0, (5, 2)), "cubic": (0, (2, 1.5)), "quartic": (0, (6, 2, 1, 2))}
REFERENCE_COLOR = "#e8871a"        # Evaluate's reference orange (overlay.ply)


def profile_row(valid, geom, profile_y_mm=None) -> int:
    """Grid row of the profile cut: the requested y, else the well-covered
    row nearest the middle of the evaluated area."""
    if profile_y_mm is not None:
        return int(np.clip(np.floor((profile_y_mm * 1e-3 - geom.y0) / geom.g), 0, geom.ny - 1))
    n = valid.sum(axis=1)
    rows = np.flatnonzero(n >= 0.5 * n.max())
    mid = 0.5 * (np.flatnonzero(n)[0] + np.flatnonzero(n)[-1])
    return int(rows[np.argmin(np.abs(rows - mid))])


PROFILE_HALF_BAND = 2      # the profile averages 2 x 2 + 1 grid rows, so empty cells don't break it


def band(a, iy):
    """Mean over the rows iy +- PROFILE_HALF_BAND, ignoring invalid cells."""
    rows = a[max(iy - PROFILE_HALF_BAND, 0):iy + PROFILE_HALF_BAND + 1]
    with np.errstate(invalid="ignore"), warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        return np.nanmean(rows, axis=0)


def _profile_axes(ax, xlabel="x, mm"):
    _style(ax)
    ax.set_xlabel(xlabel)
    ax.grid(color=GRID, lw=0.6)
    ax.set_axisbelow(True)


def plot_roi_preview(path, prep, params, title):
    """The whole scan (analysis frame, before cropping) with the evaluated
    area outlined: shows which part of the scan the numbers describe."""
    from .roi import overview
    z, extent, frac = overview(prep.all_points, prep.geom.g, prep.keep)
    fig, ax = plt.subplots(figsize=(7.6, 6.4), dpi=130, facecolor=SURFACE)
    _style(ax)
    lo, hi = np.nanpercentile(z, [1, 99])
    im = ax.imshow(z, origin="lower", extent=extent, cmap="Greys_r", vmin=lo, vmax=hi, interpolation="nearest")
    ny, nx = frac.shape
    xs = np.linspace(extent[0], extent[1], nx + 1)
    ys = np.linspace(extent[2], extent[3], ny + 1)
    ax.contour(0.5 * (xs[1:] + xs[:-1]), 0.5 * (ys[1:] + ys[:-1]), np.nan_to_num(frac), levels=[0.5],
               colors=[BLUE], linewidths=2)
    if params.roi_mm:
        x0, y0, x1, y1 = params.roi_mm
        ax.plot([x0, x1, x1, x0, x0], [y0, y0, y1, y1, y0], color=INK, lw=1, ls=(0, (4, 2)))
    if params.roi_polygon_mm:
        poly = np.array(params.roi_polygon_mm + params.roi_polygon_mm[:1])
        ax.plot(poly[:, 0], poly[:, 1], color=INK, lw=1, ls=(0, (4, 2)))
    ax.set_aspect("equal")
    ax.set_xlabel("x, mm")
    ax.set_ylabel("y, mm")
    parts = []
    if "reference_footprint_points" in prep.crop_info:
        parts.append("reference outline" + (f" inset {params.reference_inset_mm:g} mm"
                                            if params.reference_inset_mm else ""))
    if params.roi_mm or params.roi_polygon_mm:
        parts.append("ROI (dashed)")
    if params.edge_trim_mm:
        parts.append(f"edge trim {params.edge_trim_mm:g} mm")
    ax.set_title(f"blue outline = evaluated: {len(prep.points)} of {len(prep.all_points)} points"
                 + (f"  ({', '.join(parts)})" if parts else "  (whole scan)")
                 + f"\nframe: {params.orientation}", loc="left", fontsize=9, color=INK)
    cb = fig.colorbar(im, ax=ax, shrink=0.8, pad=0.02)
    cb.set_label("height, mm", color=INK_2)
    cb.ax.tick_params(colors=INK_2, labelsize=8)
    cb.outline.set_visible(False)
    fig.suptitle(title, x=0.01, ha="left", fontsize=11, color=INK)
    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


def plot_detrend_explained(path, raw, trends, residuals, geom, iy, title, primary):
    """Top: the measured surface and each fitted trend (one colour scale).
    Middle: what is left after subtracting each trend (one colour scale).
    Bottom: a profile cut through all of them. All inputs in metres."""
    methods = list(trends)
    n = len(methods)
    extent = geom.extent_mm()
    cx, cy = (c * 1e3 for c in geom.centres())
    base = np.nanmean(raw)
    raw_mm = (raw - base) * 1e3
    trend_mm = {m: (trends[m] - base) * 1e3 for m in methods}
    res_mm = {m: residuals[m] * 1e3 for m in methods}
    lim_top = _limit(raw_mm)
    lim_res = max(_limit(res_mm[m]) for m in methods)
    y_cut = cy[iy]

    fig = plt.figure(figsize=(3.9 * (n + 1) + 0.6, 11.5), dpi=110, facecolor=SURFACE)
    gs = fig.add_gridspec(3, n + 1, height_ratios=[3, 3, 2.4])
    ax = fig.add_subplot(gs[0, 0])
    _map(ax, fig, raw_mm, extent, lim_top, "measured surface\n(cell mean, minus its mean)")
    ax.axhline(y_cut, color=INK, lw=0.8, ls=(0, (4, 2)))
    for i, m in enumerate(methods, start=1):
        ax = fig.add_subplot(gs[0, i])
        _map(ax, fig, trend_mm[m], extent, lim_top, f"{m} trend (removed){' - primary' if m == primary else ''}")
        ax.axhline(y_cut, color=INK, lw=0.8, ls=(0, (4, 2)))
        ax = fig.add_subplot(gs[1, i])
        _map(ax, fig, res_mm[m], extent, lim_res, f"{m} residual (measured - trend)")
        ax.axhline(y_cut, color=INK, lw=0.8, ls=(0, (4, 2)))
    ax = fig.add_subplot(gs[1, 0])
    ax.axis("off")
    ax.text(0.0, 0.95, "How to read this\n\n"
            "Each column fits a polynomial\nz = p(x, y) of higher degree\n"
            "(planar 1, quadratic 2, cubic 3,\nquartic 4) and subtracts it.\n\n"
            "Top: what the fit treats as\nshape (removed).\n"
            "Middle: what the metrics see\nas roughness (kept).\n\n"
            "If a trend starts to follow\nindividual bumps, that order\nis eating real roughness.\n\n"
            f"Dashed line: profile at\ny = {y_cut:.1f} mm (bottom), mean of\n"
            f"{2 * PROFILE_HALF_BAND + 1} grid rows.",
            transform=ax.transAxes, va="top", fontsize=9, color=INK_2)

    split = (n + 2) // 2
    ax = fig.add_subplot(gs[2, :split])
    _profile_axes(ax)
    ax.plot(cx, band(raw_mm, iy), color=INK, lw=1.2, label="measured")
    for m in methods:
        ax.plot(cx, band(trend_mm[m], iy), color=METHOD_COLORS[m], ls=METHOD_STYLES[m], lw=2, label=f"{m} trend")
    ax.set_ylabel("height, mm")
    ax.set_title(f"profile at y = {y_cut:.1f} mm: surface and trends", loc="left", fontsize=10, color=INK)
    ax.legend(frameon=False, fontsize=8, labelcolor=INK_2, ncol=2)
    ax = fig.add_subplot(gs[2, split:])
    _profile_axes(ax)
    for m in methods:
        ax.plot(cx, band(res_mm[m], iy), color=METHOD_COLORS[m], ls=METHOD_STYLES[m], lw=2, label=m)
    ax.axhline(0, color=INK_2, lw=0.6)
    ax.set_ylabel("residual height, mm")
    ax.set_title("profile: residual (what is measured as roughness)", loc="left", fontsize=10, color=INK)
    ax.legend(frameon=False, fontsize=8, labelcolor=INK_2, ncol=2)
    fig.suptitle(title, x=0.01, ha="left", fontsize=11, color=INK)
    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


DETREND_METRIC_PANELS = [("Sq_mm", "Sq, mm"), ("rugosity", "rugosity (true / projected area)"),
                         ("Sal_mm", "Sal, mm"), ("dominant_wavelength_mm", "dominant wavelength, mm")]


def plot_detrend_metrics(path, results, methods, primary, ref_columns, title):
    """How the headline metrics move with detrend order; with a reference,
    the reference's value under the same detrend beside the scan's."""
    fig, axes = plt.subplots(2, 2, figsize=(11, 7.6), dpi=130, facecolor=SURFACE)
    x = np.arange(len(methods))
    labels = [f"{m}\n(primary)" if m == primary else m for m in methods]
    for ax, (key, label) in zip(axes.ravel(), DETREND_METRIC_PANELS):
        _style(ax)
        scan = np.array([results[m].get(key, np.nan) for m in methods], dtype=float)
        if ref_columns:
            ref = np.array([ref_columns[m].get(f"ref_{key}", np.nan) for m in methods], dtype=float)
            err = np.array([ref_columns[m].get(f"err_{key}", np.nan) for m in methods], dtype=float)
            pct = np.array([ref_columns[m].get(f"err_pct_{key}", np.nan) for m in methods], dtype=float)
            scan = np.where(np.isfinite(err), ref + err, scan)      # the overlap value the error uses
            w = 0.36
            ax.bar(x - w / 2 - 0.01, scan, w, color=BLUE, label="scan")
            ax.bar(x + w / 2 + 0.01, ref, w, color=REFERENCE_COLOR, label="reference")
            for xi, (s, p) in enumerate(zip(scan, pct)):
                if np.isfinite(s):
                    ax.text(xi - w / 2, s, f"{s:.3g}", ha="center", va="bottom", fontsize=7, color=INK)
                if np.isfinite(p):
                    ax.text(xi + w / 2, ref[xi], f"{p:+.0f}%", ha="center", va="bottom", fontsize=7, color=INK_2)
        else:
            ax.bar(x, scan, 0.6, color=BLUE)
            for xi, s in enumerate(scan):
                if np.isfinite(s):
                    ax.text(xi, s, f"{s:.3g}", ha="center", va="bottom", fontsize=8, color=INK)
        if key == "rugosity":
            finite = scan[np.isfinite(scan)]
            if ref_columns:
                finite = np.r_[finite, ref[np.isfinite(ref)]]
            if len(finite):
                ax.set_ylim(1.0, 1.0 + 1.15 * (finite.max() - 1.0) + 1e-6)
        ax.set_xticks(x, labels, fontsize=8)
        ax.set_ylabel(label)
        ax.grid(axis="y", color=GRID, lw=0.6)
        ax.set_axisbelow(True)
    if ref_columns:
        axes[0, 0].legend(frameon=False, fontsize=8, labelcolor=INK_2)
        axes[0, 0].set_title("labels: scan value; error vs reference, %", loc="left", fontsize=9, color=INK_2)
    fig.suptitle(title, x=0.01, ha="left", fontsize=11, color=INK)
    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


def plot_reference_comparison(path, vaux, scan_h, scan_raw, geom, iy, method, title):
    """Scan vs reference on the shared cells: residual maps, their
    difference, and profiles of the raw heights (an alignment check: they
    should lie on top of each other) and of the residuals."""
    mask = vaux["mask"]
    extent = geom.extent_mm()
    cx, cy = (c * 1e3 for c in geom.centres())
    s = np.where(mask, scan_h, np.nan)
    s = (s - np.nanmean(s)) * 1e3
    r = vaux["h"][method] * 1e3
    d = s - r
    lim = max(_limit(s), _limit(r))
    fig = plt.figure(figsize=(15.5, 10), dpi=110, facecolor=SURFACE)
    gs = fig.add_gridspec(2, 3, height_ratios=[3, 2.2])
    for i, (h, lab, lm) in enumerate([(s, "scan residual", lim), (r, "reference residual", lim),
                                      (d, f"scan - reference  (RMS {np.sqrt(np.nanmean(d ** 2)):.3f} mm)",
                                       _limit(d))]):
        ax = fig.add_subplot(gs[0, i])
        _map(ax, fig, h, extent, lm, lab)
        ax.axhline(cy[iy], color=INK, lw=0.8, ls=(0, (4, 2)))
    ax = fig.add_subplot(gs[1, :2])
    _profile_axes(ax)
    raw_s = band(np.where(mask, scan_raw, np.nan), iy) * 1e3
    raw_r = band(vaux["raw"], iy) * 1e3
    ax.plot(cx, raw_r, color=REFERENCE_COLOR, lw=2.4, label="reference z")
    ax.plot(cx, raw_s, color=BLUE, lw=1.4, label="scan z")
    off = np.nanmedian(raw_s - raw_r) if np.isfinite(raw_s - raw_r).any() else float("nan")
    ax.set_ylabel("height, mm (analysis frame)")
    ax.set_title(f"profile at y = {cy[iy]:.1f} mm, raw heights - alignment check "
                 f"(median scan - reference {off:+.2f} mm)", loc="left", fontsize=10, color=INK)
    ax.legend(frameon=False, fontsize=8, labelcolor=INK_2)
    ax = fig.add_subplot(gs[1, 2])
    _profile_axes(ax)
    ax.plot(cx, band(r, iy), color=REFERENCE_COLOR, lw=2.4, label="reference")
    ax.plot(cx, band(s, iy), color=BLUE, lw=1.4, label="scan")
    ax.axhline(0, color=INK_2, lw=0.6)
    ax.set_ylabel("residual, mm")
    ax.set_title(f"profile: residuals ({method} detrend)", loc="left", fontsize=10, color=INK)
    ax.legend(frameon=False, fontsize=8, labelcolor=INK_2)
    fig.suptitle(title, x=0.01, ha="left", fontsize=11, color=INK)
    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)
