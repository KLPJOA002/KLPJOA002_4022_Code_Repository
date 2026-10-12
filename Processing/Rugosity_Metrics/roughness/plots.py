"""Plots, in scan_eval.report's style and palette: blue below the mean
surface, red above, neutral grey at zero; light surface, quiet axes."""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

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
