# Rugosity_Metrics — roughness_scans.py

Surface-roughness metrics of fused 3D scans of a sediment bed. It's the sibling of
`../Evaluate/evaluate_scans.py` and reuses its `scan_eval` package (scan loading,
console log, config files, results table) by putting `../Evaluate` on `sys.path`.
No reference surface is needed.

```
roughness_scans.py [options] scan.ply | dir/ ...
roughness_scans.py --detrend all --edge-trim-mm 10 --grid-mm 1 Fused_Scans/
roughness_scans.py --config roughness_config.example.toml --noise-scan flat_plate.ply scans/
python -m roughness.synthetic ripples.ply          # 1M-point synthetic ripple scan to try it on
```

Each scan goes through these stages: `load -> orient -> crop/ROI -> detrend -> grid -> metrics -> write`.

* **orient**: a robust (sigma-clipped PCA) plane is rotated to +Z, with the normal facing
  `--sensor-origin` (default: the camera at 0,0,0). The levelled x axis is the scan's x
  projected onto the plane, and the origin is the plane centroid. The tilt removed is
  reported. **Scans already aligned into a reference frame need `--sensor-origin none`.**
* **crop**: `--roi X0,Y0,X1,Y1` (mm, levelled frame; read it off `heightmap_*.png`)
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
| `*_ci95_*`, `n_windows_used` | bootstrap over windows of `bootstrap_window_mm` (seeded) |
| `TRI_mm`, `VRM` | Riley TRI and Sappington VRM over `ruggedness_window_cells` |
| `Sq_noisecorr_mm`, `Sdq_noisecorr`, `noise_inflated` | `--noise-sigma-mm` is **per point** and is averaged down by points per cell. `--noise-scan` grids a flat-plate scan at the same grid and uses its Sq / Sdq directly |
| `ref_*`, `err_*`, `err_pct_*` | with `--reference-def`: the same metrics on the analytic surface, sampled onto the scan's grid and mask (scan − reference) |
| `flags` | warning codes for that row (`SPARSE`, `DETREND`, `NOISE`, `FOOTPRINT`, `NOPEAK`, `HURST`, `BAND`, `NODECAY`, `ALIGN`, …); the full text is in `metrics.json` and `run.log` |

## Outputs

```
<out>/<run>/metrics.json        everything: parameters, orientation, detrend coefficients,
                                all metrics per method, curves, warnings, versions + hashes
<out>/<run>/heightmap_<m>.npy   residual height map, mm, NaN = invalid (geometry in metrics.json)
<out>/<run>/heightmap_<m>.png, detrend_comparison.png, psd.png, multiscale.png
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
