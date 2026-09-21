"""The ReferenceSurface interface: everything downstream (alignment, metrics,
plots) talks to the ground truth only through this, so it does not matter
whether the truth is a mesh, an analytic expression or a user plugin.

A backend works in its own units (whatever the definition says - usually mm
for printed parts) and has to implement only three things:

    sample(n, seed)       -> points (n,3), unit normals (n,3) on the surface
    closest_points(P)     -> closest surface point (m,3) and the surface's
                             unit normal there (m,3) for arbitrary points P
    bounds()              -> (lo, hi) axis-aligned box of the surface

Everything else - unsigned and signed distance, region mask, metadata and
the definition hash - is derived here, and a backend may override any of it.
The evaluator never calls a backend directly: it wraps it in `UnitScaled` so
the pipeline itself always works in metres, like Fuse.
"""

from __future__ import annotations

import hashlib
import json
from abc import ABC, abstractmethod

import numpy as np

UNITS_TO_METRES = {"m": 1.0, "cm": 1e-2, "mm": 1e-3, "um": 1e-6, "in": 0.0254}


class SurfaceError(ValueError):
    """A reference that cannot be loaded or used."""


def unit_factor(units: str) -> float:
    try:
        return UNITS_TO_METRES[units]
    except KeyError:
        raise SurfaceError(f"unknown units '{units}' (use one of: "
                             f"{', '.join(UNITS_TO_METRES)})") from None


def parse_direction(text) -> np.ndarray | None:
    """'+z', '-y', 'none' or three numbers -> unit vector (or None)."""
    if text is None:
        return None
    if isinstance(text, (list, tuple, np.ndarray)):
        v = np.asarray(text, dtype=float)
    else:
        s = str(text).strip().lower()
        if s in ("", "none", "all"):
            return None
        axes = {"x": 0, "y": 1, "z": 2}
        if s.lstrip("+-") in axes:
            v = np.zeros(3)
            v[axes[s.lstrip("+-")]] = -1.0 if s.startswith("-") else 1.0
        else:
            try:
                v = np.array([float(t) for t in s.split(",")])
            except ValueError:
                raise SurfaceError(f"cannot read direction '{text}' (use +z, -x, or x,y,z)") from None
    if v.shape != (3,) or not np.all(np.isfinite(v)) or np.linalg.norm(v) < 1e-12:
        raise SurfaceError(f"invalid direction {text!r}")
    return v / np.linalg.norm(v)


class ReferenceSurface(ABC):
    """Ground-truth surface. See the module docstring for the contract."""

    #: Display name, recorded in every results row.
    name: str = "reference"
    #: Units the backend works in ("mm", "m", ...).
    units: str = "mm"
    #: "mesh", "analytic" or "plugin".
    kind: str = "plugin"
    #: True when distances are to the exact mathematical surface, False when
    #: the surface is itself an approximation (a mesh).
    exact: bool = False
    #: True when the surface is z = h(x, y); enables height() and Z-error stats.
    is_height_field: bool = False
    #: Optional unit vector: only the side of the surface facing this way is
    #: visible to the scanner (e.g. +z for a part scanned from above).
    visible_from: np.ndarray | None = None

    # --- the three methods a backend must provide -------------------------
    @abstractmethod
    def sample(self, n: int, seed: int = 0) -> tuple[np.ndarray, np.ndarray]:
        """n points distributed uniformly by area on the surface, with normals."""

    @abstractmethod
    def closest_points(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Closest point on the surface to each of `points`, and the unit normal there."""

    @abstractmethod
    def bounds(self) -> tuple[np.ndarray, np.ndarray]:
        """Axis-aligned bounding box (lo, hi) of the surface."""

    # --- derived; override when a backend can do better ------------------
    def distance(self, points: np.ndarray) -> np.ndarray:
        closest, _ = self.closest_points(points)
        return np.linalg.norm(np.asarray(points) - closest, axis=1)

    def signed_distance(self, points: np.ndarray) -> np.ndarray:
        """Distance, positive on the side the normal points to (outside/above)."""
        points = np.asarray(points, dtype=float)
        closest, normals = self.closest_points(points)
        offset = points - closest
        dist = np.linalg.norm(offset, axis=1)
        side = np.sign(np.einsum("ij,ij->i", offset, normals))
        side[side == 0] = 1.0
        return dist * side

    def region_mask(self, points: np.ndarray, margin: float = 0.0) -> np.ndarray:
        """Points that lie over the reference, i.e. whose error is meaningful.
        Default: inside the bounding box grown by `margin`."""
        lo, hi = self.bounds()
        p = np.asarray(points)
        return np.all((p >= lo - margin) & (p <= hi + margin), axis=1)

    def height(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """z of the surface above (x, y), NaN outside it. Height fields only."""
        raise NotImplementedError(f"{self.name} is not a height field")

    def visible_mask(self, points: np.ndarray, normals: np.ndarray) -> np.ndarray:
        """Which surface samples a scanner looking from `visible_from` could
        see. Default: those whose normal faces that way. Backends that can
        test self-occlusion (meshes) also drop hidden surface."""
        return filter_visible(points, normals, self.visible_from)

    def surface_area(self) -> float | None:
        """Total area of the (visible part of the) surface, or None if unknown."""
        return None

    def definition(self) -> dict:
        """Everything that defines this surface, used for the hash. Two runs
        with the same hash compared against exactly the same truth. The
        display name is deliberately left out: renaming changes nothing."""
        return {"kind": self.kind, "units": self.units}

    @property
    def definition_hash(self) -> str:
        blob = json.dumps(self.definition(), sort_keys=True, default=str).encode()
        return hashlib.sha256(blob).hexdigest()

    def describe(self) -> dict:
        lo, hi = self.bounds()
        return {
            "name": self.name,
            "kind": self.kind,
            "exact": self.exact,
            "units": self.units,
            "height_field": self.is_height_field,
            "visible_from": None if self.visible_from is None else self.visible_from.tolist(),
            "bounds_min": np.asarray(lo).tolist(),
            "bounds_max": np.asarray(hi).tolist(),
            "definition_hash": self.definition_hash,
        }


class UnitScaled(ReferenceSurface):
    """Presents a reference in metres whatever units it was defined in.
    Points in are divided by the factor, points out multiplied by it; normals
    and the definition are passed through untouched."""

    def __init__(self, inner: ReferenceSurface, units: str | None = None):
        self.inner = inner
        self.source_units = units or inner.units
        self.k = unit_factor(self.source_units)
        self.name = inner.name
        self.kind = inner.kind
        self.exact = inner.exact
        self.is_height_field = inner.is_height_field
        self.visible_from = inner.visible_from
        self.units = "m"

    def sample(self, n, seed=0):
        p, n_ = self.inner.sample(n, seed)
        return p * self.k, n_

    def closest_points(self, points):
        c, n = self.inner.closest_points(np.asarray(points, dtype=float) / self.k)
        return c * self.k, n

    def distance(self, points):
        return self.inner.distance(np.asarray(points, dtype=float) / self.k) * self.k

    def signed_distance(self, points):
        return self.inner.signed_distance(np.asarray(points, dtype=float) / self.k) * self.k

    def bounds(self):
        lo, hi = self.inner.bounds()
        return np.asarray(lo) * self.k, np.asarray(hi) * self.k

    def region_mask(self, points, margin=0.0):
        return self.inner.region_mask(np.asarray(points, dtype=float) / self.k, margin / self.k)

    def height(self, x, y):
        return self.inner.height(np.asarray(x) / self.k, np.asarray(y) / self.k) * self.k

    def visible_mask(self, points, normals):
        return self.inner.visible_mask(np.asarray(points, dtype=float) / self.k, normals)

    def surface_area(self):
        a = self.inner.surface_area()
        return None if a is None else a * self.k**2

    def definition(self):
        d = dict(self.inner.definition())
        d["units"] = self.source_units
        return d

    def describe(self):
        d = self.inner.describe()
        d["units"] = self.source_units
        d["definition_hash"] = self.definition_hash
        return d


def filter_visible(points: np.ndarray, normals: np.ndarray,
                   visible_from: np.ndarray | None,
                   min_cos: float = 0.05) -> np.ndarray:
    """Mask of samples whose normal faces the scanner direction."""
    if visible_from is None:
        return np.ones(len(points), dtype=bool)
    return normals @ visible_from > min_cos
