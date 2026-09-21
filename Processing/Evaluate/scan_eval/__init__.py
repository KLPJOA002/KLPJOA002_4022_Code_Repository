"""scan_eval - geometric evaluation of 3D scans against a ground-truth reference.

The public surface for plugin authors is ReferenceSurface; see reference.py.
"""

from .reference import ReferenceSurface, SurfaceError, UnitScaled

__all__ = ["ReferenceSurface", "SurfaceError", "UnitScaled"]
