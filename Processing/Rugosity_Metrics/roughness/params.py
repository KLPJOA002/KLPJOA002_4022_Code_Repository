"""Roughness parameters, in the style of scan_eval.params.EvalParams: one
place, every default explained. A TOML config file (--config) sets the
baseline and command-line flags override it.

User-facing distances are millimetres throughout (flags, config, reports);
the pipeline itself works in metres.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field, fields

# Detrend methods -> total polynomial degree of z = p(x, y).
DETREND_ORDERS = {"planar": 1, "quadratic": 2, "cubic": 3, "quartic": 4}


@dataclass
class RoughnessParams:
    # Determinism. Every random choice (spacing subsample, bootstrap) derives
    # from this, so two runs with the same inputs give identical output.
    seed: int = 0

    # --- Units and scale ------------------------------------------------------
    # Units the scan PLYs are in. RealSense captures and Fuse output are metres.
    units: str = "m"
    # Extra factor applied to the scans after unit conversion.
    scale: float = 1.0

    # --- Orientation ----------------------------------------------------------
    # "plane": level the scan onto its robust best-fit plane (raw camera /
    # fusion output). "none": keep the input frame as it is (scans already
    # aligned into a reference's frame, e.g. Evaluate's aligned_error.ply),
    # so ROIs and plots are in the part's own coordinates. None = automatic:
    # "none" when a reference is given, otherwise "plane".
    orient_mode: str | None = None
    # Where the sensor was in the scan's own frame (mm, after unit conversion),
    # as "x,y,z", or "none". The fitted plane's normal is turned to face it,
    # so +Z points from the bed towards the camera. RealSense captures and
    # Fuse output have the camera at the origin. With "none" the normal is
    # chosen to have a positive input-Z component.
    sensor_origin: str = "0,0,0"
    # In-plane x axis of the oriented frame. "input": the scan's own x axis
    # projected onto the plane (keeps the camera's left-right, so ripple
    # directions read the same as in the images). "pca": the long axis of the
    # footprint.
    inplane_axis: str = "input"
    # Robust plane fit for the orientation: points farther than this many
    # (MAD-based) sigma from the plane are dropped and the plane refitted.
    orient_clip_sigma: float = 3.0
    orient_iterations: int = 5

    # --- Alignment to the reference -------------------------------------------
    # Moves applied to the scan before anything else, so it lines up with the
    # reference: base matrix -> flip -> rotate -> shift -> optional ICP.
    # 4x4 matrix file, scan metres -> reference metres (e.g. Evaluate's or a
    # previous run's transform.txt). None = identity (scan already aligned).
    transform: str | None = None
    # Turn the scan over: 180 deg about the x axis through the reference centre.
    flip: bool = False
    # Rotation about the vertical axis through the reference centre, degrees.
    rotate_deg: float = 0.0
    # Translation [dx, dy, dz], mm.
    shift_mm: list | None = None
    # Point-to-surface ICP against the reference from that pose. It finds the
    # nearest fit only: on a periodic part (pyramids, egg carton) get within
    # half a period first.
    refine: bool = False
    # ICP correspondence distance, mm: scan points farther than this from the
    # reference are ignored.
    align_corr_mm: float = 5.0
    # If the scan is nowhere near the part after the moves above (e.g. a raw
    # fused scan, in camera coordinates), place it automatically: level on the
    # bed, find what stands up from it, centre that on the part, best rotation,
    # ICP. False = leave it where it is.
    auto_place: bool = True
    # Move the scan by hand in a window (keys), with a live scan - reference
    # height-difference map. Needs a display and PyQt6 / tkinter.
    adjust: bool = False

    # --- Crop -----------------------------------------------------------------
    # Region of interest [x0, y0, x1, y1] in mm, in the oriented frame
    # (origin at the plane-fit centroid). None = the whole scan. Run once
    # without it and read the axes of heightmap_*.png to choose it.
    roi_mm: list | None = None
    # Polygon region of interest [[x, y], ...] in mm, same frame as roi_mm.
    # Applied after roi_mm (both may be given). --pick-roi fills this or
    # roi_mm interactively.
    roi_polygon_mm: list | None = None
    # Open a top-down view of each scan and draw the ROI by hand (rectangle,
    # or press p for a polygon). The choice is saved to <run>/roi.toml.
    pick_roi: bool = False
    # Drop everything within this distance of the scan's outer border, mm.
    # Fused scans often curl or thin out at the edges. 0 = keep everything.
    edge_trim_mm: float = 0.0

    # --- Detrending -----------------------------------------------------------
    # Polynomial surface(s) removed before the metrics: planar, quadratic,
    # cubic, quartic (all terms up to that total degree), a list, or "all".
    # Every metric is computed for every method listed.
    detrend: list = field(default_factory=lambda: ["planar"])
    # Method used for the main plots and flagged primary in the CSV.
    # None = the first one listed.
    primary_detrend: str | None = None
    # Robust fitting: iterative sigma-clipping so ripples and outliers do not
    # drag the trend. Sigma is MAD-based (1.4826 x median |residual|).
    detrend_clip_sigma: float = 3.0
    detrend_iterations: int = 5
    # Expected ripple wavelength, mm, for the "fit may be eating real
    # roughness" check. 0 = use the dominant wavelength measured on the
    # primary method's residual.
    expected_wavelength_mm: float = 0.0
    # That check fires when a higher-order fit removes more than this fraction
    # of the height variance (beyond the plane) and the footprint holds fewer
    # than `order` wavelengths (a degree-n polynomial has up to n-1 extrema,
    # enough to follow about n/2 ripple periods).
    detrend_warn_fraction: float = 0.25

    # --- Gridding -------------------------------------------------------------
    # Height-map cell size, mm. 0 = automatic: 2 x the xy point spacing
    # (sqrt of the area per point, which unlike the nearest-neighbour distance
    # is not fooled by fused clouds whose points come in tight clusters),
    # rounded to 2 significant figures, so most cells hold several points and
    # the cell mean averages down sensor noise. Rugosity / Sdr / Sdq depend on
    # the cell size, so scans that are compared need the same fixed value.
    grid_mm: float = 0.0
    # Interior holes up to this many cells across are filled by linear
    # interpolation (and counted); bigger holes stay masked out. Holes open
    # to the scan's border are never filled.
    max_hole_cells: int = 2
    # A cell needs at least this many points to count as measured.
    min_points_per_cell: int = 1
    # Fail if the grid is fewer than this many cells across the footprint's
    # short side: the spatial metrics are meaningless on a handful of cells.
    min_grid_cells: int = 32

    # --- Spatial structure ----------------------------------------------------
    # ISO 25178 autocorrelation threshold s for Sal / Str.
    acf_threshold: float = 0.2
    # The dominant-wavelength search covers wavelengths up to this fraction
    # of the footprint's short side (0.5 = at least two periods visible).
    wavelength_max_fraction: float = 0.5
    # Wavelength band [min, max] for the radial-PSD power-law fit, mm.
    # None (or 0 for either end) = 4 x grid to footprint / 4: below 4 cells
    # the gridding smooths the spectrum, above footprint/4 too few periods
    # fit to estimate power.
    psd_band_mm: list | None = None
    # Log-spaced bins for the radially averaged PSD.
    psd_bins: int = 60

    # --- Multi-scale and bootstrap --------------------------------------------
    # Number of window sizes, geometric from 4 cells up to half the
    # footprint's short side.
    multiscale_sizes: int = 8
    # A window is used only if at least this fraction of its cells is valid.
    window_min_valid: float = 0.8
    # Per-window detrend before Sq / rugosity: "plane" (local least-squares
    # plane, the usual scale-dependent-roughness convention) or "mean".
    window_detrend: str = "plane"
    # Bootstrap resamples for the 95% CIs of Sq, Sdr and Sal.
    bootstrap_samples: int = 1000
    # Window size the bootstrap resamples, mm. 0 = the largest multiscale
    # size that still gives bootstrap_min_windows windows.
    bootstrap_window_mm: float = 0.0
    bootstrap_min_windows: int = 16

    # --- Ruggedness -----------------------------------------------------------
    # Window for TRI (Riley 1999) and VRM (Sappington 2007), cells (odd).
    # 0 disables both.
    ruggedness_window: int = 3

    # --- Noise ----------------------------------------------------------------
    # Known per-point sensor noise (1 sigma, along Z), mm. The height map's
    # noise is lower, sigma / sqrt(points per cell), and that is what is
    # subtracted: Sq_noisecorr = sqrt(max(Sq^2 - sigma_map^2, 0)). 0 = none.
    noise_sigma_mm: float = 0.0
    # Alternatively, a scan of a flat surface taken with the same setup. It
    # runs through the same pipeline (same grid, primary detrend) and its Sq
    # and Sdq are the noise floor. This captures spatially correlated noise
    # that the per-point model does not.
    noise_scan: str | None = None
    # Warn (noise_inflated) when noise accounts for more than this fraction of
    # the slope variance Sdq^2: rugosity / Sdr / Sdq then mostly measure noise.
    noise_warn_fraction: float = 0.5

    # --- Validity -------------------------------------------------------------
    # Warn if the footprint's short side is less than this many dominant
    # wavelengths: too few periods for stable spectral / ACF estimates.
    footprint_wavelengths_min: float = 5.0

    # --- Optional validation against an analytic surface ----------------------
    # Height-field definition (.toml, as in ../Evaluate/surfaces). The scans
    # must already be aligned into its frame (e.g. Evaluate's aligned output).
    reference_def: str | None = None
    # Overrides the definition's units.
    reference_units: str | None = None
    # "footprint": evaluate only the part of the scan that lies over the
    # reference's outline (seen from above), so scan and reference metrics
    # cover the same area. "none": keep the whole scan (the comparison then
    # recomputes the scan's metrics on the overlap).
    reference_crop: str = "footprint"
    # Shrink that outline by this much, mm, e.g. to leave out the part's
    # side walls, which a top-down height map cannot represent.
    reference_inset_mm: float = 0.0

    # --- Output ---------------------------------------------------------------
    output: str = "roughness_out"
    label: str | None = None
    write_plots: bool = True
    # y (mm, analysis frame) of the profile cut in the detrend and reference
    # plots. None = the middle of the evaluated area.
    profile_y_mm: float | None = None

    def to_dict(self):
        return asdict(self)

    @property
    def methods(self) -> list[str]:
        return list(self.detrend)

    @property
    def primary(self) -> str:
        return self.primary_detrend or self.detrend[0]

    @property
    def orientation(self) -> str:
        return self.orient_mode or ("none" if self.reference_def else "plane")


def param_names():
    return {f.name for f in fields(RoughnessParams)}


def parse_detrend(value) -> list[str]:
    """'planar', 'planar,cubic', 'all' or a list -> ordered unique method names."""
    items = value if isinstance(value, (list, tuple)) else str(value).split(",")
    items = [str(i).strip().lower() for i in items if str(i).strip()]
    if items == ["all"]:
        return list(DETREND_ORDERS)
    bad = [i for i in items if i not in DETREND_ORDERS]
    if bad or not items:
        raise ValueError(f"unknown detrend method(s) {bad or value!r}: use "
                         f"{', '.join(DETREND_ORDERS)}, a comma list, or 'all'")
    out = []
    for i in items:
        if i not in out:
            out.append(i)
    return out


def parse_polygon(value) -> list[list[float]]:
    """'x,y;x,y;...' or [[x, y], ...] -> [[x, y], ...] (mm), at least 3 vertices."""
    if isinstance(value, str):
        try:
            pts = [[float(t) for t in v.split(",")] for v in value.split(";") if v.strip()]
        except ValueError:
            raise ValueError(f"cannot read polygon '{value}' (use x,y;x,y;x,y in mm)") from None
    else:
        pts = [[float(t) for t in v] for v in value]
    if len(pts) < 3 or any(len(v) != 2 for v in pts):
        raise ValueError(f"a polygon ROI needs at least 3 x,y vertices (got {value!r})")
    return pts
