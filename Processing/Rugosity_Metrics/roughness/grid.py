"""Residual heights -> regular height map with a validity mask.

Cells hold the mean z of their points. Interior holes no wider than
max_hole_cells are filled by linear interpolation from the surrounding ring of
measured cells; bigger holes, and anything open to the border, stay invalid.
The fill plan (which cells, which neighbours, the Delaunay triangulation) is
built once per scan and reused for every detrend method.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy import ndimage
from scipy.interpolate import LinearNDInterpolator
from scipy.spatial import Delaunay, QhullError

EIGHT = np.ones((3, 3), dtype=bool)


@dataclass
class GridGeometry:
    x0: float        # lower-left cell corner, m (oriented frame)
    y0: float
    g: float         # cell size, m
    nx: int
    ny: int

    @classmethod
    def from_points(cls, xy: np.ndarray, g: float):
        # Centred on the bounding box, so a regular point lattice at spacing g
        # lands on cell centres rather than on cell edges (where rounding would
        # put two points in one cell and none in the next).
        lo, hi = xy.min(axis=0), xy.max(axis=0)
        n = np.floor((hi - lo) / g).astype(int) + 1
        x0, y0 = 0.5 * (lo + hi) - 0.5 * n * g
        return cls(float(x0), float(y0), g, int(n[0]), int(n[1]))

    def index(self, xy: np.ndarray) -> np.ndarray:
        """Flat cell index of each point; -1 outside the grid."""
        ix = np.floor((xy[:, 0] - self.x0) / self.g).astype(np.int64)
        iy = np.floor((xy[:, 1] - self.y0) / self.g).astype(np.int64)
        ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        return np.where(ok, iy * self.nx + ix, -1)

    def centres(self):
        return (self.x0 + (np.arange(self.nx) + 0.5) * self.g,
                self.y0 + (np.arange(self.ny) + 0.5) * self.g)

    def extent_mm(self):
        return [self.x0 * 1e3, (self.x0 + self.nx * self.g) * 1e3,
                self.y0 * 1e3, (self.y0 + self.ny * self.g) * 1e3]

    def as_dict(self):
        return {"x0_mm": self.x0 * 1e3, "y0_mm": self.y0 * 1e3, "grid_mm": self.g * 1e3,
                "nx": self.nx, "ny": self.ny}


def cell_counts(index: np.ndarray, geom: GridGeometry) -> np.ndarray:
    ok = index >= 0
    return np.bincount(index[ok], minlength=geom.nx * geom.ny).reshape(geom.ny, geom.nx)


def cell_means(index: np.ndarray, z: np.ndarray, counts: np.ndarray, geom: GridGeometry) -> np.ndarray:
    ok = index >= 0
    sums = np.bincount(index[ok], weights=z[ok], minlength=geom.nx * geom.ny).reshape(geom.ny, geom.nx)
    with np.errstate(invalid="ignore", divide="ignore"):
        return np.where(counts > 0, sums / np.maximum(counts, 1), np.nan)


@dataclass
class FillPlan:
    measured: np.ndarray      # cells with enough points
    footprint: np.ndarray     # measured + every enclosed hole
    fill: np.ndarray          # small holes that get interpolated
    interp: object            # (Delaunay, ring mask, target mask) or None
    n_holes_filled: int
    n_holes_left: int

    @property
    def valid(self):
        return self.measured | self.fill


def plan_fill(measured: np.ndarray, max_hole_cells: int) -> FillPlan:
    footprint = ndimage.binary_fill_holes(measured)
    holes = footprint & ~measured
    lab, n = ndimage.label(holes, structure=EIGHT)
    fill = np.zeros_like(measured)
    n_small = 0
    if n and max_hole_cells > 0:
        small = []
        for i, sl in enumerate(ndimage.find_objects(lab), start=1):
            if max(sl[0].stop - sl[0].start, sl[1].stop - sl[1].start) <= max_hole_cells:
                small.append(i)
        if small:
            fill = np.isin(lab, small)
            n_small = len(small)
    interp = None
    if fill.any():
        ring = ndimage.binary_dilation(fill, structure=EIGHT, iterations=2) & measured
        try:
            interp = (Delaunay(np.argwhere(ring).astype(float)), ring, fill)
        except QhullError:
            fill = np.zeros_like(measured)
            n_small = 0
    return FillPlan(measured, footprint, fill, interp, n_small, n - n_small)


def apply_fill(h: np.ndarray, plan: FillPlan) -> np.ndarray:
    """Copy of h with the planned holes interpolated (NaN where impossible)."""
    out = np.where(plan.measured, h, np.nan)
    if plan.interp is None:
        return out
    tri, ring, fill = plan.interp
    f = LinearNDInterpolator(tri, h[ring])
    out[fill] = f(np.argwhere(fill).astype(float))
    return out
