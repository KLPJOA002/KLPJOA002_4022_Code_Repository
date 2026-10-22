# Rugosity_Metrics — roughness_scans.py

Surface-roughness metrics of fused 3D scans of a sediment bed. It's the sibling of
`../Evaluate/evaluate_scans.py` and reuses its `scan_eval` package (scan loading,
console log, config files, results table, reference meshes) by putting `../Evaluate`
on `sys.path`. No reference is needed, but a known part (.stl) can be compared against.

```
roughness_scans.py [options] scan.ply | dir/ ...
roughness_scans.py --detrend all --edge-trim-mm 10 --grid-mm 1 Fused_Scans/
roughness_scans.py --config roughness_config.example.toml --noise-scan flat_plate.ply scans/
roughness_scans.py --pick-roi --detrend all scan.ply          # draw the evaluated area by hand
roughness_scans.py --reference part.stl --detrend all aligned_error.ply
python -m roughness.synthetic ripples.ply          # 1M-point synthetic ripple scan to try it on
```

Each scan goes through these stages: `load -> [align] -> orient -> crop/ROI -> detrend -> grid -> metrics -> [validate] -> write`.

## Comparing against a known part (.stl)

1. Align the scan to the part with Evaluate: `evaluate_scans.py --reference References/PART.stl scan.ply`.
   Its `aligned_error.ply` is the scan in the part's frame (metres). Check `overlay.ply`
   first: if the alignment is wrong, every roughness error is meaningless.
2. `roughness_scans.py --reference ../Evaluate/References/PART.stl --detrend all <run>/aligned_error.ply`

With `--reference`:
* the frame is kept as it is (`--orient none` is the default), so all coordinates are the part's own mm;
* only the scan over the part's outline (seen from above) is evaluated (`--reference-crop footprint`);
  `--reference-inset-mm 3` also drops a rim, e.g. the side walls, which a height map can't represent;
* the mesh is read as a height field: rays cast straight down onto it give z = h(x, y);
* the reference is sampled onto the scan's own grid and valid cells, then goes through the
  **same detrend, grid and metric code**. `err_* = scan − reference` therefore compares the same
  cells with the same processing. If the reference doesn't cover every scan cell (with
  `--reference-crop none`), the scan is re-detrended and re-evaluated on the overlap minus a one-cell
  rim (`REFCOVER`), and the scan's own columns still describe its whole area;
* `reference_comparison.png` shows scan and reference residuals, their difference, and a profile of
  the **raw** heights. That profile is the alignment check: the two lines should lie on top of each other.

## Moving the scan onto the reference

The comparison is only meaningful if the scan lines up with the part. Two ways in:

* **A raw fused scan** (camera coordinates) with just `--reference part.stl`. It is nowhere near
  the part, so it is **auto-placed**: levelled on the bed, the thing standing up from the bed is
  found and centred on the part (base on the bed), every 2° of rotation is tried, then ICP. The log
  prints the best rotations; a symmetric part may need a 90° turn by hand (`r` in `--adjust`).
  `--no-auto-place` turns this off.
* **Evaluate first** (`evaluate_scans.py --height-field ...`), then pass its `aligned_error.ply`, or the
  raw scan with `--transform <run>/transform.txt`.

Evaluate (or auto-place) can still pick a wrong pose on symmetric or periodic parts (turned 90°,
slid half a period, upside down), so the scan can also be moved by hand. The moves are applied
in this order:

| flag | does |
|---|---|
| `--transform FILE` | 4×4 scan → reference matrix (Evaluate's or a previous run's `transform.txt`); lets you pass the raw scan |
| `--flip` | turn the scan over (180° about x through the part's centre) |
| `--rotate-deg A` | about the vertical axis through the part's centre |
| `--shift-mm DX,DY,DZ` | translate (negative values work as typed: `--shift-mm -4,3,-1.5`) |
| `--refine` | match heights, then ICP against the STL. ICP finds the *nearest* fit, so on a periodic part get within half a period first. It is ignored (with a warning) if it makes the fit worse |
| `--adjust` | a window (needs PyQt6, PyQt5 or GTK, like `--pick-roi`): the scan's height with the part's outline and height contours on top, **scan − reference**, and profiles along x and y (click a map to move them). The status line shows the pose, median/RMS dz, correlation, coverage, the step size and the last key received. Keys: x/y arrows or `i j k l`; z PgUp/PgDn, `w`/`s` or `+`/`-`; rotate `[ ]`, `r` = 90°; step `1 2 3 4` = 0.1 / 1 / 10 / 100 mm (deg capped at 10); `f` flip, `p` auto-place, `a` auto-fit (ICP), `u` undo, `0` reset, `c` rescale colours, Enter accept, Esc cancel. The same actions are buttons along the bottom |

With a reference, every run checks the final pose and flags `ALIGN` (median |scan z − reference z|
is large), `UPSIDEDOWN` (heights anti-correlated with the part, or the camera ends up behind it)
or `ALIGNCOVER` (the scan covers < 50 % of the outline). The stats are in the `align_*` columns.
The final pose is saved as `<run>/transform.txt` in Evaluate's format. Rerun with
`--transform <run>/transform.txt` (and none of the move flags), or seed Evaluate with
`--init-transform <run>/transform.txt --align refine`.

When aligning with Evaluate itself, pass `--height-field` for these parts: without it Evaluate
doesn't know which side is up and can return an upside-down pose.

**Mesh repair:** `scan_eval` drops faces on non-manifold edges, and on a part modelled as
separate shells (pyramids stacked on a base plate, e.g. `PYRAMID_2X2.stl`) it drops real top
faces, leaving holes. Roughness puts them back for its height field (and warns). Evaluate's own
reference for such a part still has the holes.

## Choosing the evaluated area

* `--roi X0,Y0,X1,Y1` and/or `--roi-polygon "x,y;x,y;x,y"` (mm, analysis frame), plus `--edge-trim-mm`.
* `--pick-roi` opens the whole scan top-down (with the reference outline in orange). Drag a rectangle,
  or press `p` and click a polygon (click the first vertex to close it). Enter accepts; Esc keeps the
  whole scan. The choice is printed as the matching flags and saved as `<run>/roi.toml`; rerun
  exactly with `--config <run>/roi.toml`. It needs a display and a GUI toolkit, which the venv
  lacks by default: `../Evaluate/.venv/bin/pip install PyQt6` (or `dnf install python3-tkinter`).
* `roi_preview.png` (always written) shows the whole scan with the evaluated area outlined, for the report.

## Seeing what the detrend does

* `detrend_explained.png`: the measured surface, each fitted trend (what is removed), each residual
  (what the metrics see), and a profile cut through all of them (`--profile-y-mm`, default: the middle).
  If a trend starts following individual bumps, that order is removing real roughness.
* `detrend_metrics.png`: Sq, rugosity, Sal and dominant wavelength for each method; with a reference
  the reference's value under the same detrend sits beside it, labelled with the % error.
* `detrend_comparison.png`: residual maps on one colour scale, plus Sq / Sdr bars.

* **orient** (`--orient plane`, the default without a reference): a robust (sigma-clipped PCA) plane is rotated to +Z, with the normal facing
  `--sensor-origin` (default: the camera at 0,0,0). The levelled x axis is the scan's x
  projected onto the plane, and the origin is the plane centroid. The tilt removed is
  reported. `--orient none` keeps the input frame (the default with `--reference`).
* **crop**: reference outline, `--roi` / `--roi-polygon` / `--pick-roi` (mm, analysis frame)
  and `--edge-trim-mm` (distance to the outer border, ignoring interior holes).
* **detrend**: a polynomial of total degree 1–4 in normalised x, y, fitted with MAD-based
  sigma-clipping. `--detrend` takes one method, a list or `all`, and every method gets its
  own CSV row. A `DETREND` flag means a higher order removed a lot of variance over a
  footprint that is only a few wavelengths long, so it may be fitting real bedforms.
* **grid**: the mean z per cell. Interior holes ≤ `--max-hole-cells` across are linearly
  interpolated and counted; larger holes and the exterior stay masked. The auto grid is
  2 × median point spacing. On randomly scattered clouds this leaves empty cells, which
  raises a `SPARSE` flag; use `--grid-mm` then.

## Conventions worth knowing

| column | meaning |
|---|---|
| `Sq_mm` … `Sz_mm`, `Ssk`, `Sku` | ISO 25178 height parameters; `Sv` is positive |
| `rugosity`, `Sdr_pct` | true area of a 2-triangles-per-quad mesh / projected area; `Sdr = (rugosity-1)·100` |
| `Sdq`, `slope_mean_deg` | RMS gradient (dimensionless) and mean slope angle |
| `Sal_mm`, `Str` | fastest ACF decay to s = 0.2 (sub-cell interpolated); `Str` = Sal / slowest decay in the central lobe. `Str_bounded` = the lobe reaches the edge of the usable lags, so Str is an upper bound |
| `dominant_wavelength_mm`, `dominant_direction_deg` | 2D PSD peak; the direction is the **wave vector** (ripple migration axis), 0–180° CCW from +x |
| `Std_deg` | **lay / crest** direction (wave-vector angle of the angular-spectrum maximum + 90°); resolution is limited by the k-grid (a few degrees) |
| `anisotropy_ratio` | √(λmin/λmax) of the spectral slope tensor: 1 = isotropic, → 0 = one-directional (same sense as Str) |
| `psd_slope`, `hurst`, `fractal_dimension` | log-log fit of the radially averaged PSD over `bandwidth_min_mm..bandwidth_max_mm`, using C(q) ∝ q^-2(1+H) and D = 3 − H. The default band includes ripple peaks, so for small-scale texture set `--psd-band-mm 2,20` |
| `ms_*` | Sq and rugosity in non-overlapping windows (local plane removed) from 4 cells to half the footprint; curves in `multiscale_<method>.csv` |
| `*_ci95_*`, `n_windows_used`, `bootstrap_Sq_point_mm` | bootstrap over windows of `bootstrap_window_mm` (seeded). It only uses windows ≥ `window_min_valid` valid, so the CI can miss the whole-map Sq on patchy maps; `bootstrap_Sq_point_mm` is the estimate over those same windows |
| `TRI_mm`, `VRM` | Riley TRI and Sappington VRM over `ruggedness_window_cells` |
| `Sq_noisecorr_mm`, `Sdq_noisecorr`, `noise_inflated` | `--noise-sigma-mm` is **per point** and is averaged down by points per cell. `--noise-scan` grids a flat-plate scan at the same grid and uses its Sq / Sdq directly |
| `ref_*`, `err_*`, `err_pct_*` | with `--reference` (.stl/.ply/.obj mesh or height-field .toml): the same metrics on the reference, sampled onto the scan's grid and mask (scan − reference, same cells) |
| `flags` | warning codes for that row (`SPARSE`, `DETREND`, `NOISE`, `FOOTPRINT`, `NOPEAK`, `HURST`, `BAND`, `NODECAY`, `ALIGN`, …); the full text is in `metrics.json` and `run.log` |

## Outputs

```
<out>/<run>/metrics.json        everything: parameters, orientation, detrend coefficients,
                                all metrics per method, curves, warnings, versions + hashes
<out>/<run>/heightmap_<m>.npy   residual height map, mm, NaN = invalid (geometry in metrics.json)
<out>/<run>/heightmap_<m>.png, roi_preview.png, detrend_explained.png, detrend_metrics.png,
            detrend_comparison.png, psd.png, multiscale.png, [reference_comparison.png]
<out>/<run>/roi.toml            with --pick-roi: the chosen area, reusable with --config
<out>/<run>/psd_<m>.csv, multiscale_<m>.csv, run.log
<out>/roughness_results.csv         one row per (scan, method), appended across runs
<out>/roughness_batch_summary.csv   this batch only
```

## Tests

```
cd Rugosity_Metrics && ../Evaluate/.venv/bin/python -m pytest -q tests
```

The tests use synthetic surfaces only: polynomial trends over known roughness, sinusoids
(Sq, Sa, Sdr from arc length, Sal, λ, direction), fractal surfaces (Hurst), white noise,
holes and edge trim, determinism, the CLI, and validation against Evaluate's egg-carton
definition.
