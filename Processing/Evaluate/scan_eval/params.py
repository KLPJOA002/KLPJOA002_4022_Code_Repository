"""Evaluation parameters, in the spirit of Fuse's PipelineParams: one place,
every default explained. A TOML config file (--config) sets the baseline and
command-line flags override it, exactly like Fuse's --rig-config.

User-facing distances are in millimetres throughout (flags, config, reports):
at the scale of a desk-sized reference part, mm is what anyone reads. The
pipeline itself works in metres.
"""

from __future__ import annotations

import tomllib
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path


@dataclass
class EvalParams:
    # Determinism. Every random choice (surface sampling, RANSAC) derives
    # from this, so two runs with the same inputs give identical output.
    seed: int = 0

    # --- Units and scale ------------------------------------------------------
    # Units the scan PLYs are in. RealSense captures and Fuse output are metres.
    units: str = "m"
    # Extra factor applied to the scans after unit conversion, e.g. to test a
    # suspected calibration error. Scale is NOT estimated unless allow_scale:
    # scale error is one of the things being measured.
    scale: float = 1.0
    # Units of a bare mesh reference (.stl/.ply carry none). Definition files
    # declare their own units; this overrides them.
    reference_units: str | None = None
    # Similarity instead of rigid alignment: also solve for a uniform scale
    # and report it. Metrics are then computed on the rescaled scan.
    allow_scale: bool = False

    # --- Alignment ------------------------------------------------------------
    # full   : feature matching (several starts) + perturbation multi-start + ICP
    # refine : ICP only, from --init-transform (or identity)
    # none   : use --init-transform (or identity) exactly as given
    align: str = "full"
    init_transform: str | None = None
    # Voxel size for the alignment copy of the scan, mm. 0 = automatic,
    # reference diagonal / 60 (2.4 mm for a 100x100x30 mm part). Metrics
    # always use the full-resolution cloud.
    align_voxel_mm: float = 0.0
    # Correspondence distance for *scoring* a pose (Fuse's
    # eval_correspondence_distance), mm. 0 = automatic, 3 x voxel. Too large
    # and every pose looks like 100% overlap; too small and a correct pose
    # looks like a miss.
    eval_distance_mm: float = 0.0
    # Independent RANSAC runs (different seeds). More starts = more chances
    # to find the right one of several look-alike poses on a periodic part.
    ransac_starts: int = 5
    ransac_max_iterations: int = 100000
    ransac_confidence: float = 0.999
    # Also try quarter turns, a flip, and slides of 1/4 and 1/2 of the part
    # around the best pose - the generic guard against periodic references.
    perturb: bool = True
    icp_max_iterations: int = 60
    # Fuse's direction_ambiguous_margin: winner beats runner-up by less than
    # this fraction of its score -> flagged [AMBIGUOUS].
    ambiguous_margin: float = 0.15
    # Eigenvalue ratios (to the largest) of the final normal equations below
    # which a degree of freedom is reported weak / unconstrained.
    degeneracy_weak: float = 1e-2
    degeneracy_unconstrained: float = 1e-4
    # Where the sensor was, in the scan's own frame (mm, after unit
    # conversion), as "x,y,z" - or "none". Scan normals are oriented toward
    # it, and a scan point only matches reference surface facing the same
    # way; that is what tells a thin surface right way up from upside down.
    # RealSense captures and Fuse output have the camera at the origin.
    sensor_origin: str = "0,0,0"
    # Statistical outlier removal on the alignment copy only (Fuse's
    # sor_mean_k / sor_stddev_mult). 0 disables.
    sor_neighbours: int = 30
    sor_std: float = 1.0

    # --- Metrics --------------------------------------------------------------
    # Distance thresholds for completeness / precision / F-score, mm.
    thresholds_mm: list = field(default_factory=lambda: [0.5, 1.0, 2.0, 5.0])
    # Scan points farther than this from the surface count as outliers, mm.
    outlier_distance_mm: float = 5.0
    # A reference sample counts as "covered" by the scan if a scan point lies
    # within this distance, mm. Completeness is computed over the covered part
    # only, and the covered fraction is reported separately. Must exceed the
    # largest threshold or completeness is trivially 100% there.
    coverage_radius_mm: float = 10.0
    # How far outside the reference's region (outline) a scan point may be and
    # still be scored. 0 = only points over the part itself; the table, other
    # objects and the part's surroundings are excluded and counted separately.
    region_margin_mm: float = 0.0
    # Percent of worst points dropped for the robust ("trimmed") statistics.
    trim_percent: list = field(default_factory=lambda: [1.0, 5.0])
    # Points sampled on the reference for completeness (area-uniform).
    reference_samples: int = 200000
    # Which side of the reference a scanner sees ("+z", "-y", "x,y,z" or
    # "none"). Default: from the definition (height fields: +z).
    visible_from: str | None = None

    # --- Output ---------------------------------------------------------------
    output: str = "evaluation_out"
    label: str | None = None
    write_ply: bool = True
    write_plots: bool = True
    # Also write overlay.ply: the reference and the aligned scan in one cloud,
    # for checking the alignment (and the choice of reference) by eye.
    write_overlay: bool = True
    # Reference points drawn in that overlay. Enough to read the shape;
    # raise it for a big part, lower it if your viewer struggles.
    overlay_samples: int = 150000

    def to_dict(self):
        return asdict(self)


def load_config(path) -> dict:
    """Read a TOML config and check every key is a real parameter, so a typo
    fails loudly instead of silently running with a default."""
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"config file not found: {path}")
    try:
        with open(path, "rb") as fh:
            data = tomllib.load(fh)
    except tomllib.TOMLDecodeError as exc:
        raise ValueError(f"{path}: invalid TOML: {exc}") from None
    flat = {}
    for key, value in data.items():
        if isinstance(value, dict):      # allow [alignment] / [metrics] sections
            flat.update(value)
        else:
            flat[key] = value
    return flat


def param_names():
    return {f.name for f in fields(EvalParams)}
