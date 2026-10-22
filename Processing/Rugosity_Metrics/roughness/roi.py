"""Choosing the evaluated area by hand.

--pick-roi opens a top-down mean-height image of the scan (analysis frame,
before any crop) with the reference's outline drawn on it, and lets you drag
a rectangle or click a polygon. The choice becomes roi_mm / roi_polygon_mm,
is printed as the equivalent flag, and is written to <run>/roi.toml so the
exact same area can be re-run (--config roi.toml) for a report.

The picker needs an interactive matplotlib backend (Qt or Tk). Neither ships
with a bare venv: `.venv/bin/pip install PyQt6`, or install your
distribution's python3-tkinter.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from scan_eval.log import log

from .grid import GridGeometry, cell_counts, cell_means

INTERACTIVE_BACKENDS = ("QtAgg", "TkAgg", "GTK4Agg", "GTK3Agg", "MacOSX")


def overview(points: np.ndarray, g: float, keep: np.ndarray | None = None, max_cells: int = 700):
    """Mean height (mm) on a display grid, its extent (mm), and, if `keep` is
    given, the fraction of each cell's points that are kept (for outlining
    the evaluated area)."""
    xy = points[:, :2]
    span = float(np.max(np.ptp(xy, axis=0)))
    g = max(g, span / max_cells)
    geom = GridGeometry.from_points(xy, g)
    idx = geom.index(xy)
    counts = cell_counts(idx, geom)
    z = cell_means(idx, points[:, 2], counts, geom) * 1e3
    frac = None
    if keep is not None:
        with np.errstate(invalid="ignore"):
            frac = cell_means(idx, keep.astype(float), counts, geom)
    return z, geom.extent_mm(), frac


def roi_from_rectangle(x0, y0, x1, y1) -> list[float]:
    x0, x1 = sorted((float(x0), float(x1)))
    y0, y1 = sorted((float(y0), float(y1)))
    return [round(x0, 2), round(y0, 2), round(x1, 2), round(y1, 2)]


def roi_from_polygon(vertices) -> list[list[float]]:
    return [[round(float(x), 2), round(float(y), 2)] for x, y in vertices]


def as_flags(roi_mm, polygon_mm) -> str:
    out = []
    if roi_mm:
        out.append("--roi " + ",".join(f"{v:g}" for v in roi_mm))
    if polygon_mm:
        out.append('--roi-polygon "' + ";".join(f"{x:g},{y:g}" for x, y in polygon_mm) + '"')
    return " ".join(out)


def write_roi_toml(path, roi_mm, polygon_mm, scan, orientation) -> None:
    lines = [f"# Evaluated area for {scan}, chosen with --pick-roi.",
             f"# mm in the analysis frame (orientation: {orientation}). Re-run exactly with",
             "#   roughness_scans.py --config roi.toml <same flags> <scan>",
             "# or copy these lines into your own config.",
             f'orient_mode = "{orientation}"',
             "[crop]"]
    if roi_mm:
        lines.append("roi_mm = [" + ", ".join(f"{v:g}" for v in roi_mm) + "]")
    if polygon_mm:
        lines.append("roi_polygon_mm = [" + ", ".join(f"[{x:g}, {y:g}]" for x, y in polygon_mm) + "]")
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")


class RoiPicker:
    """The figure and its two selectors. Kept apart from show() so the
    selection logic runs (and is tested) on a non-interactive backend."""

    def __init__(self, points, g, region=None, title="", initial=None):
        import matplotlib.pyplot as plt
        from matplotlib.widgets import PolygonSelector, RectangleSelector

        from scan_eval.report import INK, INK_2, SURFACE

        self.roi_mm, self.polygon_mm, self.accepted = None, None, False
        z, extent, frac = overview(points, g, region)
        self.fig, self.ax = plt.subplots(figsize=(9, 7.5), facecolor=SURFACE)
        ax = self.ax
        lo, hi = np.nanpercentile(z, [1, 99])
        im = ax.imshow(z, origin="lower", extent=extent, cmap="Greys_r", vmin=lo, vmax=hi,
                       interpolation="nearest")
        if frac is not None and np.nanmax(frac) > 0:
            ny, nx = frac.shape
            xs = np.linspace(extent[0], extent[1], nx + 1)
            ys = np.linspace(extent[2], extent[3], ny + 1)
            ax.contour(0.5 * (xs[1:] + xs[:-1]), 0.5 * (ys[1:] + ys[:-1]), np.nan_to_num(frac),
                       levels=[0.5], colors=["#e8871a"], linewidths=1.5)
        ax.set_aspect("equal")
        ax.set_xlabel("x, mm")
        ax.set_ylabel("y, mm")
        cb = self.fig.colorbar(im, ax=ax, shrink=0.8, pad=0.02)
        cb.set_label("height, mm (analysis frame)", color=INK_2)
        self.help = ax.set_title("drag a rectangle  |  p: polygon mode  r: rectangle mode  |  "
                                 "Enter: accept   Esc: cancel (whole scan)"
                                 + ("\norange = reference outline" if frac is not None else ""),
                                 loc="left", fontsize=9, color=INK)
        self.fig.suptitle(title, x=0.01, ha="left", fontsize=11, color=INK)
        props = dict(facecolor="#2a78d6", edgecolor="#2a78d6", alpha=0.25)
        self.rect = RectangleSelector(ax, self.on_rectangle, useblit=False, interactive=True,
                                      props=props, button=[1])
        self.poly = PolygonSelector(ax, self.on_polygon, useblit=False,
                                    props=dict(color="#2a78d6", linewidth=1.5))
        self.poly.set_active(False)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)
        if initial:
            self.roi_mm = list(initial)
            self.rect.extents = (initial[0], initial[2], initial[1], initial[3])

    def on_rectangle(self, press, release):
        if None in (press.xdata, release.xdata):
            return
        roi = roi_from_rectangle(press.xdata, press.ydata, release.xdata, release.ydata)
        if roi[2] - roi[0] > 0 and roi[3] - roi[1] > 0:
            self.roi_mm, self.polygon_mm = roi, None

    def on_polygon(self, vertices):
        if len(vertices) >= 3:
            self.polygon_mm, self.roi_mm = roi_from_polygon(vertices), None

    def on_key(self, event):
        import matplotlib.pyplot as plt
        if event.key == "p":
            self.rect.set_active(False)
            self.rect.set_visible(False)
            self.poly.set_active(True)
        elif event.key == "r":
            self.poly.set_active(False)
            self.rect.set_active(True)
            self.rect.set_visible(True)
        elif event.key == "enter":
            self.accepted = True
            plt.close(self.fig)
        elif event.key == "escape":
            self.roi_mm = self.polygon_mm = None
            self.accepted = True
            plt.close(self.fig)
        self.fig.canvas.draw_idle()


def _no_keymaps():
    import matplotlib
    return {k: [] for k in matplotlib.rcParams if k.startswith("keymap.")}


# matplotlib's own shortcuts (p = pan, r = home, f = fullscreen, arrows =
# back/forward, ...) are switched off while a picker window is open.
NO_KEYMAPS = _no_keymaps()


def interactive_backend(flag="--pick-roi"):
    import matplotlib
    import matplotlib.pyplot as plt
    for name in INTERACTIVE_BACKENDS:
        try:
            plt.switch_backend(name)
            return name
        except (ImportError, RuntimeError, ValueError):
            continue
    plt.switch_backend("Agg")
    raise ValueError(f"{flag} needs an interactive matplotlib backend and none works here "
                     f"(matplotlib {matplotlib.__version__}; tried {', '.join(INTERACTIVE_BACKENDS)}). "
                     "Install one (`.venv/bin/pip install PyQt6`, or your distribution's "
                     "python3-tkinter) and check a display is available, or use the typed "
                     "equivalent (--roi / --roi-polygon; --flip / --rotate-deg / --shift-mm / --refine)")


def pick(points, g, region=None, title="", initial=None):
    """Show the picker and block until Enter / Esc / window closed. Returns
    (roi_mm, polygon_mm); both None means the whole scan."""
    import matplotlib.pyplot as plt
    interactive_backend("--pick-roi")
    try:
        with plt.rc_context(NO_KEYMAPS):
            picker = RoiPicker(points, g, region, title, initial)
            log("[roi] pick the area in the window: drag a rectangle, or press p and click a polygon "
                "(click the first vertex to close it); Enter accepts, Esc keeps the whole scan")
            plt.show(block=True)
    finally:
        plt.switch_backend("Agg")
    if picker.roi_mm is None and picker.polygon_mm is None:
        log("[roi] no area chosen - evaluating the whole scan")
    else:
        log(f"[roi] chosen: {as_flags(picker.roi_mm, picker.polygon_mm)}")
    return picker.roi_mm, picker.polygon_mm
