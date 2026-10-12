# 12 — Rugosity_Metrics: surface-roughness metrics of 3D scans

**Summary.** `Processing/Rugosity_Metrics` is a Python command-line tool, `roughness_scans.py`, plus a package, `roughness/`. It computes reference-free surface-roughness parameters of 3D point-cloud scans (README: "fused 3D scans of a sediment bed"). It is the sibling of `../Evaluate/evaluate_scans.py` and reuses Evaluate's `scan_eval` package for PLY loading, logging, TOML config loading and CSV results tables. For each scan it runs `load → orient → crop/ROI → detrend → grid → metrics → write`:

* **orient**: levels the scan with a robust sigma-clipped PCA plane.
* **crop/ROI**: optionally crops to a region of interest and trims the border.
* **detrend**: removes a robust polynomial trend of degree 1–4.
* **grid**: bins the residuals onto a regular height map, interpolating small holes.
* **metrics**: computes ISO 25178-2-style areal parameters (Sa, Sq, Ssk, Sku, Sp, Sv, Sz, Sdq, Sdr, Sal, Str, Std), a triangulated rugosity index, spectral descriptors (dominant wavelength and direction, anisotropy, radially averaged PSD power-law slope → Hurst exponent → fractal dimension), multi-scale roughness curves, block-bootstrap 95% CIs, and terrain-ruggedness indices (TRI, VRM).
* **noise correction** (optional), from a per-point σ or a flat-plate scan.
* **validation** (optional) against an analytic reference surface.

A synthetic-surface generator and a pytest suite check the metrics against closed-form answers.

**Version-control status: the whole folder is UNTRACKED in git** (`git status` shows `?? Processing/Rugosity_Metrics/`). There is therefore **no commit history** and no commit messages to use as design evidence. All rationale below comes from code comments and the README. File mtimes are all on 2026-10-12, between 10:57 and 11:27. The tool also depends on an **uncommitted modification** to `Processing/Evaluate/scan_eval/report.py` (see Observations). The behaviour described here comes from reading the source only. Nothing was run. `.pytest_cache/v/cache/lastfailed` is `{}`, which suggests the last local pytest run had no failures, but this was not re-verified.

### Files covered

| Path (repo-relative) | Lines | Role |
|---|---|---|
| `Processing/Rugosity_Metrics/README.md` | 72 | Usage, stage overview, column conventions, outputs |
| `Processing/Rugosity_Metrics/roughness_config.example.toml` | 74 | Example config (all parameters, sectioned) |
| `Processing/Rugosity_Metrics/roughness_scans.py` | 288 | CLI: argument parsing, parameter resolution and validation, batch loop, summary |
| `Processing/Rugosity_Metrics/roughness/__init__.py` | 17 | Puts `../Evaluate` on `sys.path` |
| `Processing/Rugosity_Metrics/roughness/params.py` | 188 | `RoughnessParams` dataclass (every default documented), detrend method parsing |
| `Processing/Rugosity_Metrics/roughness/orient.py` | 162 | Robust plane fit, levelling transform, point spacing, ROI and edge trim |
| `Processing/Rugosity_Metrics/roughness/detrend.py` | 80 | Robust polynomial trend fit (degree 1–4) |
| `Processing/Rugosity_Metrics/roughness/grid.py` | 119 | Grid geometry, cell means, small-hole interpolation plan |
| `Processing/Rugosity_Metrics/roughness/metrics.py` | 446 | All metrics |
| `Processing/Rugosity_Metrics/roughness/pipeline.py` | 485 | Stage orchestration, noise model, reporting, outputs |
| `Processing/Rugosity_Metrics/roughness/plots.py` | 150 | Height-map, detrend-comparison, PSD and multi-scale figures |
| `Processing/Rugosity_Metrics/roughness/synthetic.py` | 173 | Synthetic surfaces (sinusoid, ripples, random/fractal fields, trends, holes, camera frame) |
| `Processing/Rugosity_Metrics/roughness/validate.py` | 116 | Comparison against an analytic height-field reference |
| `Processing/Rugosity_Metrics/tests/conftest.py` | 47 | Test helpers and fixtures |
| `Processing/Rugosity_Metrics/tests/test_cli.py` | 146 | End-to-end CLI tests |
| `Processing/Rugosity_Metrics/tests/test_metrics.py` | 205 | Metric tests against known answers |

Also present: `.gitignore` (`__pycache__/`, `*.pyc`, `.pytest_cache/`, `roughness_out/`) and `.pytest_cache/`.

### Contents
1. Purpose
2. Inputs and outputs
3. Step-by-step walkthrough
4. Algorithms and methods (metric definitions with equations)
5. Why this approach
6. Parameters and tuning
7. Assumptions, limitations, failure modes
8. Dependencies
9. Validation and tests
10. Observations

---

## 1. Purpose

The tool quantifies the roughness of a scanned surface itself, with no reference surface needed (README.md:3-6; roughness_scans.py:9-12). It complements Evaluate, which measures a scan's *error* against a known part. The intended input is a fused point cloud from `../Fuse` or a RealSense capture, in an arbitrary camera or fusion frame. The intended output is a table of standardised roughness descriptors per scan and per detrending choice, for comparing surfaces, sensors and processing settings (e.g. `--label d435_close_tsdf` in roughness_config.example.toml:13). The README and docstrings speak of a "sediment bed" with "ripples" and "bedforms" (README.md:3; params.py:67-75). For this project's sea-ice/surface-roughness context, that vocabulary carries over to any quasi-planar rough surface.

## 2. Inputs and outputs

### 2.1 Inputs
| Input | Format / units |
|---|---|
| Scans | `.ply` point clouds (files or directories searched recursively via Evaluate's `find_scans`). Loaded by `scan_eval.pipeline.load_scan`, which drops non-finite points, requires ≥100 points, and multiplies by the unit factor and `scale` (Evaluate/scan_eval/pipeline.py:27-47). Default units `m` (params.py:25) |
| `--config` TOML | Flat or sectioned keys. Sections are flattened by `scan_eval.params.load_config`. Unknown keys raise (roughness_scans.py:156-160) |
| `--noise-scan` | PLY of a flat plate, same setup |
| `--reference-def` | Analytic height-field TOML from `../Evaluate/surfaces` (e.g. `egg_carton.toml`). The scan must already be in the reference frame |

Internally everything is in **metres**. User-facing distances (flags, config, reports) are **millimetres** (params.py:5-6).

### 2.2 CLI (roughness_scans.py:55-149)

| Group | Flags |
|---|---|
| General | `scans...`, `-o/--output`, `--label`, `--config`, `--seed`, `--list-only`, `-v` |
| Units/orientation | `--units {m,cm,mm,um,in}`, `--scale`, `--sensor-origin X,Y,Z|none`, `--inplane-axis {input,pca}` |
| Crop/grid | `--roi X0,Y0,X1,Y1`, `--edge-trim-mm`, `--grid-mm`, `--max-hole-cells`, `--min-points-per-cell` |
| Detrend | `--detrend planar|quadratic|cubic|quartic|list|all`, `--primary-detrend`, `--clip-sigma`, `--clip-iterations`, `--expected-wavelength-mm` |
| Metrics | `--acf-threshold`, `--psd-band-mm MIN,MAX`, `--multiscale-sizes`, `--window-detrend {plane,mean}`, `--bootstrap-samples`, `--bootstrap-window-mm`, `--ruggedness-window`, `--footprint-wavelengths-min` |
| Noise | `--noise-sigma-mm` or `--noise-scan` (mutually exclusive) |
| Validation | `--reference-def`, `--reference-units` |
| Output | `--no-plots` |

Precedence is dataclass defaults ← TOML ← command line (`resolve_params`, roughness_scans.py:152-171). Some parameters can be set only in TOML: `orient_clip_sigma`, `orient_iterations`, `detrend_warn_fraction`, `min_grid_cells`, `wavelength_max_fraction`, `psd_bins`, `window_min_valid`, `bootstrap_min_windows`, `noise_warn_fraction`.

### 2.3 Outputs (README.md:51-61; pipeline.py:422-481)

| File | Content |
|---|---|
| `<out>/<run>/metrics.json` | Parameters, versions and hashes (code SHA-256, scan SHA-256, git commit/dirty), orientation, crop, grid, scan summary, noise, per-method detrend coefficients, metrics, curves, warnings, validation |
| `<out>/<run>/heightmap_<method>.npy` | float32 residual height map, **mm**, mean-removed, NaN = invalid, shape (ny, nx). Geometry is in metrics.json `grid` |
| `<out>/<run>/heightmap_<m>.png`, `detrend_comparison.png`, `psd.png`, `multiscale.png` | Plots |
| `<out>/<run>/psd_<m>.csv` | `k_per_mm, wavelength_mm, psd_mm4, count` (radially averaged PSD) |
| `<out>/<run>/multiscale_<m>.csv` | `window_cells, window_mm, n_windows, Sq_mean_mm, Sq_std_mm, rugosity_mean, rugosity_std` |
| `<out>/<run>/run.log` | Console log |
| `<out>/roughness_results.csv` | One row per (scan, method), **appended** across runs, with columns unioned. Lead columns at pipeline.py:38-42 |
| `<out>/roughness_batch_summary.csv` | This batch only, sorted by scan then method (roughness_scans.py:269-274) |

`run` = `[label__]<stem>` (via Evaluate's `unique_run_names`). The output frame is the **levelled frame**: origin at the plane-fit centroid, +Z along the plane normal (towards the sensor by default), x = the input x axis projected onto the plane.

## 3. Step-by-step walkthrough

### 3.1 CLI (`roughness_scans.main`, roughness_scans.py:229-284)
1. Parse the arguments, resolve the parameters (TOML then flags), and run `validate_params` (roughness_scans.py:174-211). It checks the detrend list, primary-in-list, units, positivity, ROI ordering, PSD band, noise exclusivity, that the noise file exists, ACF threshold in (0,1), an odd ruggedness window ≥3, enumerations, and bootstrap ≥10.
2. `find_scans`. `--list-only` prints the scans and exits.
3. Optionally load the analytic reference (`validate.load_height_reference`, validate.py:29-37). It must be a height field.
4. For each scan, run `pipeline.process_scan`. Expected errors (`ValueError`, `FileNotFoundError`, `SurfaceError`; Evaluate's `ScanError` subclasses `ValueError`) are logged as failures and the loop continues. The rows are sorted, a summary is printed, and `roughness_batch_summary.csv` is rewritten. The exit status is 1 if any scan failed.

### 3.2 `pipeline.analyse` (pipeline.py:333-399)
1. Create `rng = default_rng(seed)`.
2. **`prepare`** (pipeline.py:58-96):
   * Load the scan, then **orient** it (`orient.orient`, §4.1).
   * Estimate the median xy nearest-neighbour spacing from a ≤20 000-point seeded subsample (orient.py:109-119).
   * Choose the grid: `grid_mm`, or auto = `round_sig(2 × spacing)` to 2 significant figures (pipeline.py:75-76). Warn if the grid is finer than half the spacing.
   * **Crop** (ROI and edge trim, orient.py:128-162), then build `GridGeometry.from_points`, centred on the bounding box (grid.py:31-38).
   * Fail if the short side has fewer than `min_grid_cells` (32) cells (pipeline.py:90-95).
3. **`detrend_all`** (pipeline.py:109-129): for each method, `fit_trend` on the **points** (§4.2) and residual $r = z - p(x,y)$. Stats: `trend_rms_mm`, `trend_variance_removed` $=1-\mathrm{var}(r)/\mathrm{var}(z)$, residual Sq, inlier fraction.
4. **`grid_all`** (pipeline.py:132-160):
   * Cell counts, `measured = counts ≥ min_points_per_cell`, and **one fill plan** shared by every method (`plan_fill`, §4.3).
   * Per method, cell means of the residuals, then `apply_fill`.
   * `valid` comes from the first map, and is the same for all.
   * Stats: valid fraction (of the hole-filled footprint), interpolated fraction, holes filled/left, footprint extents, mean points per cell.
5. **Noise floor** (if active): `NoiseModel.for_grid` (pipeline.py:164-191). See §4.9.
6. A `SPARSE` warning is raised if the valid fraction is below 0.8 (pipeline.py:348-352).
7. **Metrics per method**: `surface_metrics(map, valid, g, params, m_rng)`. Each method gets its own RNG, seeded `[seed, order]`, so its bootstrap does not depend on which other methods ran (pipeline.py:358-361). The detrend stats are merged in, and noise corrections are applied (`apply_noise`, pipeline.py:210-228).
8. **Over-fitting check**: for order ≥2, flag `DETREND` if the extra variance removed beyond planar exceeds `detrend_warn_fraction` (0.25) and the footprint short side is less than `order` × λ. Here λ is `expected_wavelength_mm`, or the primary method's measured dominant wavelength unless that was at the search limit (pipeline.py:369-384).
9. Optional **validation** (`validate.compare_to_reference`, §4.10).

### 3.3 `process_scan` (pipeline.py:403-485)
Attaches `run.log`, calls `analyse`, saves the `.npy` maps and curve CSVs, optionally plots, writes `metrics.json`, and appends one CSV row per method with values rounded to 6 decimals (pipeline.py:308-315, 468-479).

### 3.4 `surface_metrics` (metrics.py:406-446)
1. Mean-remove over the mask.
2. Footprint = bounding extent of the valid cells (metrics.py:396-398).
3. PSD fit band = [`psd_band_mm` or 4g, footprint_min/4] (metrics.py:401-403).
4. Compute, in order: `height_metrics`, `area_metrics`, `acf_metrics` (`NODECAY` warning if Sal is undefined), `psd2d` + `spectral_metrics`, `multiscale`, `choose_bootstrap_window` + `bootstrap`, `ruggedness`, and the `FOOTPRINT` check (short side < `footprint_wavelengths_min` × λ).

## 4. Algorithms and methods

Notation: $h_{ij}$ is the residual height map (m) on a square grid of spacing $g$. $V$ is the validity mask, with $N=|V|$. $z_{ij} = h_{ij} - \bar h$, the mean taken over $V$. The ISO references are to **ISO 25178-2:2012** (areal surface texture: terms, definitions and parameters). Where the code's discrete definition matches the ISO concept, this is noted. Note that ISO 25178 parameters are formally defined on a scale-limited surface after an S-filter, an L-filter or F-operator (ISO 16610 / ISO 25178-3). This code uses only an F-operator (polynomial detrend) plus the implicit low-pass of cell averaging. It applies **no Gaussian S/L filtering** (see §7).

### 4.1 Orientation: robust total-least-squares plane (orient.py:37-106)
* **Robust scale**: $\hat\sigma = 1.4826\,\operatorname{median}|r-\operatorname{median}(r)|$, the normal-consistent MAD (orient.py:37-40).
* **Plane fit** (`fit_plane`, orient.py:43-62). This is a PCA / total-least-squares plane. The centroid is $\mathbf c$, and the normal $\mathbf n$ is the eigenvector of the scatter matrix $\sum(\mathbf p-\mathbf c)(\mathbf p-\mathbf c)^T$ with the smallest eigenvalue (`np.linalg.eigh`, column 0). Iterative sigma-clipping keeps $|(\mathbf p-\mathbf c)\cdot\mathbf n|\le k\hat\sigma$ (k=3, ≤5 iterations) and stops when the inlier set is unchanged. The major axis (largest eigenvalue) is also returned.
* **Up direction**: if a sensor origin $\mathbf o$ is given (default (0,0,0) mm, the camera), flip $\mathbf n$ so that $(\mathbf o-\mathbf c)\cdot\mathbf n>0$. If $\mathbf o$ is `none`, or lies within 1 mm of the plane, use $n_z>0$ instead (orient.py:79-89).
* **In-plane axes**: $\mathbf e_x$ is the input x axis projected onto the plane, or the y axis if $|n_x|\ge0.9$, or the PCA major axis (`pca`). $\mathbf e_y=\mathbf n\times\mathbf e_x$. $R=[\mathbf e_x;\mathbf e_y;\mathbf n]$, and $\mathbf p' = R(\mathbf p-\mathbf c)$ (orient.py:90-99).
* Reported: tilt $=\arccos|n_z|$, plane RMS over inliers, inlier fraction.

### 4.2 Detrending: robust polynomial least squares (detrend.py:20-80)
Coordinates are normalised to $[-1,1]$ over the footprint: $x_n=(x-x_c)/x_h$, likewise $y_n$, which keeps the Vandermonde-type design well conditioned up to quartic (detrend.py:3-6). The model is
$$p(x,y)=\sum_{i+j\le n} a_{ij}\,x_n^i y_n^j,$$
with $(n+1)(n+2)/2$ coefficients: planar 3, quadratic 6, cubic 10, quartic 15 (`exponents`, detrend.py:20-21). The fit is ordinary least squares (`np.linalg.lstsq`) followed by iterative re-fits on points with $|r|\le k\hat\sigma_{MAD}$ (k=3, up to 5 fits total). It stops when the set is unchanged or fewer than $5\times$#coef points remain (detrend.py:56-80). This is the **F-operator** (form removal) of ISO 25178 terminology. Because the oriented frame already has the best-fit plane removed, "planar" mostly removes the small residual plane left by the robust fit (test comment, test_metrics.py:34).

### 4.3 Gridding and hole interpolation (grid.py)
* **Geometry** (grid.py:31-38): $n_x=\lfloor (x_{max}-x_{min})/g\rfloor+1$, with the grid **centred** on the bounding box so that a regular point lattice at spacing $g$ lands on cell centres.
* **Cell value**: the mean $z$ of the points in the cell (`bincount` sums ÷ counts, grid.py:60-69). Averaging $c$ points with independent noise σ reduces the noise to $\sigma/\sqrt c$. It also acts as a box low-pass filter of width $g$, which is the implicit S-filter.
* **Fill plan** (grid.py:86-108):
  * footprint = `binary_fill_holes(measured)` (enclosed holes only).
  * holes = footprint ∧ ¬measured, labelled with 8-connectivity.
  * A hole is "small" if its bounding box is ≤ `max_hole_cells` (2) in both dimensions.
  * The interpolation support is the ring of measured cells within 2 dilations of the small holes.
  * A Delaunay triangulation of the ring-cell centres is built once (`scipy.spatial.Delaunay`). Each method then uses `LinearNDInterpolator` on that triangulation (piecewise-linear barycentric interpolation) to fill the small-hole cells (grid.py:111-119).
* Larger holes and border-open gaps stay NaN.

### 4.4 Height parameters (metrics.py:29-39), matching ISO 25178-2 definitions in discrete form
$$S_a=\frac1N\sum|z|,\qquad S_q=\sqrt{\frac1N\sum z^2},\qquad S_{sk}=\frac{1}{S_q^3}\frac1N\sum z^3,\qquad S_{ku}=\frac{1}{S_q^4}\frac1N\sum z^4,$$
$$S_p=\max z,\qquad S_v=|\min z|\ (\text{reported positive}),\qquad S_z=S_p+S_v.$$
These are the areal analogues of ISO 4287 profile parameters Ra, Rq, Rsk, Rku, Rp, Rv, Rz. They are areal (S-), not profile (R-) parameters. Note that ISO 25178 $S_z$ is the maximum height, not ISO 4287's five-point Rz average, so the code matches ISO 25178 here.

### 4.5 Area, rugosity and slope (metrics.py:43-69)
For each 2×2 quad of cells with corners $z_{00}, z_{10}$ (x+1), $z_{01}$ (y+1), $z_{11}$, the quad is split along the $z_{00}$–$z_{11}$ diagonal into two triangles. Each triangle's area comes from the cross product of its edge vectors:
$$A_q=\tfrac g2\Big(\sqrt{(z_{10}-z_{00})^2+(z_{10}-z_{11})^2+g^2}+\sqrt{(z_{01}-z_{11})^2+(z_{01}-z_{00})^2+g^2}\Big).$$
(This was verified by expanding $\lVert(\mathbf v_{10}-\mathbf v_{00})\times(\mathbf v_{11}-\mathbf v_{10})\rVert$.) Quads with any invalid corner are excluded.
$$\text{rugosity}=\frac{\sum A_q}{N_q\,g^2},\qquad S_{dr}=(\text{rugosity}-1)\times100\%.$$
"Rugosity" here is the **surface-area ratio** (true/projected area), the areal analogue of the chain-and-tape rugosity index used in benthic ecology. $S_{dr}$ (developed interfacial area ratio) is the ISO 25178-2 parameter, with the code's triangulation as its discretisation.

Slopes per quad: $g_x=\frac{(z_{10}-z_{00})+(z_{11}-z_{01})}{2g}$, $g_y=\frac{(z_{01}-z_{00})+(z_{11}-z_{10})}{2g}$.
$$S_{dq}=\sqrt{\overline{g_x^2+g_y^2}}\quad(\text{ISO 25178 root-mean-square gradient}),\qquad \bar\theta=\overline{\arctan\sqrt{g_x^2+g_y^2}}\ (\text{mean slope angle, not an ISO parameter}).$$

### 4.6 Autocorrelation: Sal and Str (metrics.py:73-160)
* **Masked ACF via FFT** (`masked_acf`). With $h_0 = h\cdot V$ and $m=V$, both zero-padded to fast FFT lengths ≥ H+lag+1 to avoid circular wrap:
$$\text{num}(\tau)=\mathcal F^{-1}|\mathcal F h_0|^2,\quad \text{cnt}(\tau)=\operatorname{round}\mathcal F^{-1}|\mathcal F m|^2,\quad \text{ACF}(\tau)=\frac{\text{num}(\tau)/\text{cnt}(\tau)}{\text{num}(0)/\text{cnt}(0)}.$$
  Each lag is averaged only over pairs where both cells are valid, which is a normalised masked autocorrelation. Lags supported by fewer than 20% of the valid cells are NaN. The lag range is ±H/2, ±W/2.
* **Crossings** (`acf_crossings`, metrics.py:106-127): along every pair of 4-neighbouring lags where one is > s and the other ≤ s, the crossing radius is linearly interpolated, $r = r_A + \frac{A-s}{A-B}(r_B-r_A)$.
* **Sal** (ISO 25178-2 autocorrelation length) = the minimum crossing radius, i.e. the horizontal distance of fastest decay of the ACF to $s$ (default 0.2, as in ISO).
* **Str** (ISO texture-aspect ratio) $=\frac{\text{fastest decay}}{\text{slowest decay}}$, where the slowest decay is the largest crossing radius **within the connected central lobe** (ACF > s around zero lag). If the lobe touches the edge of the usable lags, the slowest decay is taken as at least that radius and `Str_bounded=True` (Str is then an upper bound) (metrics.py:144-159). Str is near 1 for isotropic surfaces and near 0 for strongly directional ones.

### 4.7 Spectral analysis (metrics.py:164-252)
* **2D PSD** (`psd2d`). A separable Hann window $w$ is applied. The weighted mean is removed over $V$, invalid cells are set to 0, and then
$$C(\mathbf k) = \frac{g^2\,|\mathrm{FFT}(z\,w)|^2}{\sum (wV)^2},$$
  fft-shifted, with $\mathbf k$ in cycles/m. With this normalisation $\sum_{\mathbf k} C\,\Delta k_x\Delta k_y$ equals the window-weighted height variance (Parseval; docstring metrics.py:165-168, consistent on inspection).
* **Dominant wavelength and direction** (metrics.py:192-208):
  * Search the half-plane for $1/(0.5\,L_{min}) \le |\mathbf k| \le 1/\lambda_{band,min}$.
  * Take the peak, refined by the power-weighted centroid of its 3×3 neighbourhood.
  * $\lambda_d=1/|\mathbf k_c|$, and the direction is $\operatorname{atan2}(k_y,k_x)\bmod 180°$. This is the **wave-vector** direction (normal to crests).
  * `dominant_peak_ratio` = peak/median. `NOPEAK` is flagged if the peak is within 1.5 bins of the search limits.
* **Std** (texture direction; metrics.py:211-214): build a 1°-bin angular power histogram over the search region, smooth it circularly with [¼,½,¼], take the argmax, and add 90°. This reports the **lay (crest) direction**, perpendicular to the dominant wave vector. ISO 25178-2 defines Std as the angle of the maximum of the angular power spectrum relative to a specified direction. Whether the +90° matches a given ISO-software convention should be checked before the thesis compares numbers; the README documents the code's convention (README.md:41).
* **Anisotropy ratio** (metrics.py:217-221): the spectral second-moment tensor
$$M=\sum_{\mathbf k\in S} C(\mathbf k)\begin{pmatrix}k_x^2&k_xk_y\\k_xk_y&k_y^2\end{pmatrix}\propto\operatorname{cov}(\nabla h),$$
  with ratio $\sqrt{\lambda_{min}/\lambda_{max}}$: 1 isotropic, →0 unidirectional. It is not an ISO parameter.
* **Radially averaged PSD and power-law fit** (metrics.py:224-251):
  * Bin $C$ into `psd_bins` (60) log-spaced bins of $|\mathbf k|$ from $1/L_{max}$ to Nyquist $1/(2g)$.
  * Fit $\log_{10} C = \beta\log_{10}k + c$ over the band $[1/\lambda_{max},1/\lambda_{min}]$ (default λ from 4g to $L_{min}/4$).
  * For a self-affine (fractal) isotropic surface, the 2D PSD scales as $C(q)\propto q^{-2(1+H)}$ (the form used e.g. by Persson et al. 2005 for rough-surface contact; the code cites the relation, not the paper). So
$$H=-\beta/2-1,\qquad D_f = 3-H,$$
  where $H$ is the Hurst (roughness) exponent and $D_f$ the fractal dimension of the surface.
  * $R^2$ of the fit is reported. `HURST` is flagged if $H\notin[0,1]$, and `BAND` if fewer than 4 bins fall in the band.

### 4.8 Multi-scale roughness, bootstrap CIs and ruggedness (metrics.py:256-392)
* **Multi-scale** (metrics.py:265-316):
  * Window sizes are geometric from 4 cells to half the short side (`multiscale_sizes` = 8, de-duplicated).
  * Each size tiles the map with non-overlapping centred windows; windows with ≥80% valid cells are used.
  * Per window, a weighted least-squares local plane is removed (normal equations solved in batch via `einsum`), or only the mean if `window_detrend=mean`.
  * Then per-window Sq and rugosity are computed, with mean ± std across windows.
  * `ms_Sq_scaling_exponent` is the slope of $\log S_q$ vs $\log L$. For a self-affine surface $S_q(L)\propto L^{H}$, so this is a second, independent Hurst estimate **[INFERRED interpretation; the code does not label it as Hurst]**.
* **Bootstrap** (metrics.py:319-370): a non-overlapping **block bootstrap**. The w×w windows are resampled with replacement, `bootstrap_samples` (1000) times, giving percentile 95% CIs.
  * Sq and Sdr pool the globally detrended heights and quad areas of the resampled windows, so their point estimate equals the global value.
  * Sal is the mean of per-window Sal values (ACF restricted to ±w/2 lags).
  * The window size is the **largest** multi-scale size that still gives ≥16 usable windows, or `bootstrap_window_mm`. `FEWWIN` is flagged otherwise.
* **Ruggedness** (metrics.py:373-392):
  * **TRI** (Terrain Ruggedness Index, Riley, DeGloria & Elliot 1999): $\text{TRI}_c=\sqrt{\sum_{i\in W}(z_i-z_c)^2}$ over an n×n window (default 3). It is computed with box filters, $\sum(z_i-z_c)^2 = S_2 - 2z_cS_1 + Nz_c^2$, and averaged over fully valid windows.
  * **VRM** (Vector Ruggedness Measure, Sappington, Longshore & Thompson 2007): the unit normals $\hat{\mathbf n}=(-h_x,-h_y,1)/\sqrt{h_x^2+h_y^2+1}$ come from `np.gradient`, and $\text{VRM}=1-\lVert\frac1N\sum_W\hat{\mathbf n}\rVert$, averaged over the map. Both come from DEM geomorphometry and are not ISO parameters.

### 4.9 Noise correction (pipeline.py:164-228)
* **Per-point σ** (`--noise-sigma-mm`). The map noise is $\sigma_{map}=\sigma\sqrt{\overline{1/c}}$, averaged over the valid measured cells, because the mean of $c$ points has variance $\sigma^2/c$. For white noise on the grid, the gradient estimator above gives $\operatorname{var}(g_x)=\operatorname{var}(g_y)=\sigma_{map}^2/g^2$, so $S_{dq,noise}=\sqrt2\,\sigma_{map}/g$ (pipeline.py:188-191, matching this derivation).
* **Noise scan** (`--noise-scan`). The flat scan is run through the same prepare (ROI removed), primary detrend and grid at the **same g**. Its Sq and Sdq become the floor, cached per grid size. A warning is raised if the points-per-cell density differs by more than 2× (pipeline.py:176-187, 194-207).
* **Corrections** assume additive, uncorrelated noise (variances add):
$$S_{q,corr}=\sqrt{\max(S_q^2-\sigma_{map}^2,0)},\qquad S_{dq,corr}=\sqrt{\max(S_{dq}^2-S_{dq,noise}^2,0)}.$$
  `noise_inflated` is set (and a `NOISE` warning raised) when $S_{dq,noise}^2/S_{dq}^2 > 0.5$. Rugosity, Sdr and Sdq then mostly measure noise. Note that fine grids inflate slopes, since $\propto 1/g$; this behaviour is tested in test_metrics.py:125-137.

### 4.10 Validation against an analytic surface (validate.py:40-116)
1. Map the scan points back to the original frame ($\mathbf p = R^T\mathbf p' + \mathbf c$) and intersect their xy bounds with the reference bounds.
2. Measure the alignment: the median $|z_{scan}-h_{ref}(x,y)|$ on ≤50k samples.
3. Sample the reference on a lattice of step $g/2$ and push it through the **scan's own** levelling transform. Keep only the samples landing in the scan's valid cells.
4. Run exactly the same detrend, grid, fill and `surface_metrics` per method.
5. Report `ref_*`, `err_* = scan − ref`, and `err_pct_*` for 21 keys (validate.py:24-26).
6. Raise warnings: `UPSIDEDOWN` (levelled +Z opposite to the reference's `visible_from`), `REFCOVER` (<95% coverage), `ALIGN` (median offset > max(1 mm, ½ ref Sq)).

This measures the error budget of the whole capture → fusion → roughness chain directly (validate.py:3-9).

### 4.11 Synthetic surfaces (synthetic.py)
* **Sinusoid**: $z=A\sin(2\pi(x\cos\theta+y\sin\theta)/L)$ (synthetic.py:33-36).
* **Ripples**: asymmetric (second harmonic, asymmetry 0.35), with sinuous crests (phase wobble 0.15 L over 6 L), normalised (synthetic.py:39-49).
* **RandomField**: an isotropic Gaussian surface as a sum of 400 cosines with log-uniform $|\mathbf k|\in[1/\lambda_{max},1/\lambda_{min}]$, random directions in [0,π) and random phases. The amplitude is ∝ $k^{-(1+H)}\cdot k$, where the extra $k$ compensates the log sampling density. It is scaled to the target Sq and is evaluable at arbitrary xy (synthetic.py:52-74).
* **fractal_grid**: periodic self-affine map by FFT filtering of white noise with $|\mathbf k|^{-(1+H)}$, so PSD ∝ $q^{-2(1+H)}$ (synthetic.py:77-86). This is the standard spectral synthesis method.
* **polynomial_trend**: random total-degree-n polynomial with significant top-degree terms, scaled to a given peak-to-peak (synthetic.py:89-98).
* **punch_holes**: circular dropouts (synthetic.py:101-105).
* **to_camera_frame**: flip 180° about x, tilt about camera x, spin about the optical axis, camera at height 0.7 m (synthetic.py:108-120).
* **make_ripple_scan**: ripples + small-scale fractal texture (Sq 0.3 mm, H 0.7) + quadratic bowl + 0.5 mm Gaussian noise + 12 holes + ragged border, tilted 20°, 1 M points, in metres (synthetic.py:129-145).
* CLI: `python -m roughness.synthetic out.ply [--flat] ...` (synthetic.py:148-169).

## 5. Why this approach

| Decision | Rationale | Label |
|---|---|---|
| Reuse `scan_eval` (load, log, config, CSV) | "the same way Parameter_Sweep and Filter_Sweep do"; nothing in Evaluate depends on this | **[EXPLICIT: roughness/__init__.py:1-5; README.md:3-6]** |
| Level each scan on a robust plane | Scans arrive in an arbitrary camera or fusion frame | **[EXPLICIT: orient.py:3-6; roughness_scans.py:12-13]** |
| Orient +Z towards the sensor | +Z points from the bed to the camera; RealSense and Fuse output have the camera at the origin | **[EXPLICIT: params.py:30-36]** |
| `input` in-plane axis | Keeps the camera's left-right so ripple directions read the same as in the images | **[EXPLICIT: params.py:37-40]** |
| Normalised monomials | Keeps the LSQ well conditioned up to quartic | **[EXPLICIT: detrend.py:3-6]** |
| Robust (MAD sigma-clip) fits | So ripple crests, troughs and outliers do not drag the trend | **[EXPLICIT: detrend.py:5-7; params.py:63-66]** |
| Multiple detrend orders with a DETREND warning | A degree-n polynomial can follow ~n/2 periods, so it may eat real bedforms | **[EXPLICIT: params.py:71-75; pipeline.py:369-371; README.md:23-26]** |
| Auto grid = 2 × spacing | Most cells hold several points; the mean averages noise | **[EXPLICIT: params.py:78-81]** |
| Only small interior holes interpolated | Larger and border-open holes stay masked; Delaunay plan reused per method | **[EXPLICIT: grid.py:3-8; params.py:82-85]** |
| Same orientation, crop and grid for every method | "their rows compare like for like" | **[EXPLICIT: pipeline.py:5-6]** |
| PSD band 4g … footprint/4 | Below 4 cells gridding smooths the spectrum; above footprint/4 too few periods | **[EXPLICIT: params.py:98-102]** |
| Hann window, mask-aware normalisation | So the PSD integrates to the height variance | **[EXPLICIT: metrics.py:165-168]**. The choice of Hann over other windows is not explained |
| Local plane in multi-scale windows | "the usual scale-dependent-roughness convention" | **[EXPLICIT: params.py:112-114]** |
| Per-method seeded RNG | Results independent of which other methods ran | **[EXPLICIT: pipeline.py:358-360]** |
| Noise scan alternative | Captures spatially correlated noise that the per-point model misses | **[EXPLICIT: params.py:132-136]** |
| Seed fixes every random choice | Determinism (spacing subsample, bootstrap) | **[EXPLICIT: params.py:19-21]**; tested test_cli.py:58-68 |
| Recording code and scan SHA-256 and git state | Provenance | **[INFERRED]** from pipeline.py:276-305 |

Alternatives the code does not take: ISO 16610 Gaussian S/L filtering instead of a polynomial F-operator plus box averaging; profile-based ISO 4287 parameters (Ra, Rq) along transects; a TIN surface area on the raw points instead of a grid; variogram or structure-function fractal estimators instead of PSD slopes **[INFERRED]**.

## 6. Parameters and tuning

All defaults are in `roughness/params.py` (line numbers below) and mirrored in `roughness_config.example.toml`.

| Name | Default | params.py | Effect |
|---|---|---|---|
| `seed` | 0 | 21 | All randomness |
| `units` / `scale` | m / 1.0 | 25-27 | Input conversion |
| `sensor_origin` | "0,0,0" (mm) | 35 | Up direction; `none` for reference-aligned scans |
| `inplane_axis` | input | 40 | x-axis choice |
| `orient_clip_sigma` / `orient_iterations` | 3.0 / 5 | 43-44 | Plane robustness |
| `roi_mm` | None | 50 | Crop in levelled frame |
| `edge_trim_mm` | 0 | 53 | Border trim (5–15 suggested for fused scans, toml:28) |
| `detrend` / `primary_detrend` | [planar] / first | 59-62 | Methods |
| `detrend_clip_sigma` / `detrend_iterations` | 3.0 / 5 | 65-66 | Trend robustness |
| `expected_wavelength_mm` | 0 (= measured) | 70 | DETREND check |
| `detrend_warn_fraction` | 0.25 | 75 | DETREND check |
| `grid_mm` | 0 (auto 2×spacing) | 81 | Resolution vs noise: smaller g raises Sdq and Sdr through noise |
| `max_hole_cells` | 2 | 85 | Interpolation extent |
| `min_points_per_cell` | 1 | 87 | Cell validity |
| `min_grid_cells` | 32 | 90 | Minimum short-side cells |
| `acf_threshold` | 0.2 | 94 | ISO s for Sal/Str |
| `wavelength_max_fraction` | 0.5 | 97 | Longest λ searched (≥2 periods) |
| `psd_band_mm` | None (4g…L/4) | 102 | Hurst fit band; README suggests `2,20` for small-scale texture (README.md:43) |
| `psd_bins` | 60 | 104 | Radial PSD bins |
| `multiscale_sizes` | 8 | 109 | Number of window sizes |
| `window_min_valid` | 0.8 | 111 | Window usability |
| `window_detrend` | plane | 114 | Per-window detrend |
| `bootstrap_samples` / `bootstrap_window_mm` / `bootstrap_min_windows` | 1000 / 0 (auto) / 16 | 116-120 | CI resolution |
| `ruggedness_window` | 3 | 125 | TRI/VRM window (odd, 0 off) |
| `noise_sigma_mm` / `noise_scan` | 0 / None | 131-136 | Noise floor |
| `noise_warn_fraction` | 0.5 | 139 | NOISE flag |
| `footprint_wavelengths_min` | 5 | 144 | FOOTPRINT flag |
| `reference_def` / `reference_units` | None | 149-151 | Validation |
| `output` / `label` / `write_plots` | roughness_out / None / True | 154-156 | Outputs |

## 7. Assumptions, limitations, failure modes

* **Quasi-planar, single-valued surface.** It is gridded as z = h(x,y) after levelling, so overhangs and steep faces are lost **[INFERRED]**. A wrong up direction inverts the heights: Ssk and Sp/Sv flip (validate.py:102-106).
* **Not strictly ISO-conformant scale-limited parameters.** There is no S-filter/L-filter per ISO 16610. The nesting index is implicit (grid cell, polynomial order, footprint), and Sal/Str are limited to half the footprint. Values are comparable within this tool, but should be qualified when compared with ISO instrument software **[INFERRED]**.
* **Sdr / rugosity depend on the triangulation diagonal** (always 00–11) and strongly on g and noise (§4.9). Sdr has a small anisotropic bias for features aligned with that diagonal **[INFERRED]**.
* **Spectral metrics need several periods.** `FOOTPRINT` is flagged below 5 λ, and `NOPEAK` when there is no clear peak. The default PSD band can include the ripple peak and break the power-law assumption (`HURST`; README.md:43).
* **The bootstrap treats windows as independent.** Adjacent windows are spatially correlated, so the CIs may be optimistic. Per-window Sal is truncated by the window size **[INFERRED]**.
* **The noise model assumes additive, independent noise.** Interpolated cells have lower effective noise, which this ignores **[INFERRED]**.
* **Failure modes with explicit errors**:
  * fewer than 100 points;
  * empty ROI;
  * edge trim removes everything;
  * grid too coarse (<32 cells);
  * no measured cells;
  * no 2×2 valid block;
  * too few PSD frequencies;
  * detrend with fewer than 5×coef points;
  * all xy coincident;
  * reference not overlapping.

  All are `ValueError`s, so they are caught per scan.
* **Performance**: the 2D FFTs and the Delaunay scale with grid size; there are no explicit limits.

## 8. Dependencies

**Internal**: `../Evaluate`, put on `sys.path` by `roughness/__init__.py:12-17`, which raises `ImportError` if it is missing.
* `evaluate_scans.UNIT_CHOICES`, `find_scans` (recursive `.ply` search).
* `scan_eval.log` (`log`, `warn`, `stage`, `attach`/`detach`, private `_files`).
* `scan_eval.params.load_config` (TOML, flattening sections).
* `scan_eval.pipeline.load_scan` (Open3D read, unit scaling) and `unique_run_names`.
* `scan_eval.report.append_results` (union-column CSV append, **requires the uncommitted `lead_columns` kwarg**), `write_json`, and the plot style constants (`BLUE`, `ERROR_CMAP`, `_style`, `error_limit`).
* `scan_eval.reference.SurfaceError`, `UnitScaled`; `scan_eval.surface_def.load_reference`.

**Third-party**:
* **numpy**: linear algebra (`eigh`, `lstsq`, `solve`, `einsum`), `bincount`, `polyfit`, `percentile`, `default_rng`.
* **scipy**:
  * `scipy.fft` (`rfft2`/`irfft2`/`fft2`/`next_fast_len`, used for the masked ACF and the PSD);
  * `scipy.ndimage` (`binary_fill_holes`, `label`, `find_objects`, `binary_dilation`/`erosion`, `distance_transform_edt` for the edge trim, `uniform_filter` for TRI/VRM);
  * `scipy.spatial` (`cKDTree` for point spacing; `Delaunay`);
  * `scipy.interpolate.LinearNDInterpolator`;
  * `scipy.spatial.transform.Rotation` (synthetic);
  * `scipy.special.ellipe` (tests).
* **matplotlib** (Agg) for plots. **open3d** for PLY read and write (via scan_eval; synthetic `write_ply`). **pytest** for tests. **tomllib** (stdlib, via scan_eval).

There is no `requirements.txt`. The README runs the tests with `../Evaluate/.venv/bin/python` (README.md:65-67).

## 9. Validation and tests

`tests/` (pytest; synthetic data only, no sensor):

| Test | Checks |
|---|---|
| `test_matching_detrend_recovers_known_sq` (test_metrics.py:23-34) | Polynomial trends of order 1–4 (20 mm p-p) over random roughness: the matching or higher order recovers the true Sq within 3%; one order too low leaves >1.2× |
| `test_detrend_warns_when_footprint_is_short` (37-47) | A 70 mm sinusoid over 100 mm: cubic and quartic flag DETREND and eat >50% of Sq |
| Sinusoid tests (57-96) | Sq = A/√2, Sa = 2A/π, Ssk≈0, Sku≈1.5; λ and direction; Std = 90°; **Sdr against the exact arc-length elliptic integral** $\frac{2}{\pi}\sqrt{1+m^2}E\!\left(\frac{m^2}{1+m^2}\right)-1$, $m=2\pi A/L$; Sdq = m/√2; Sal = L·arccos(0.2)/2π; Str < 0.15; oblique 30° |
| `test_hurst_exponent_recovered` (100-110) | H = 0.5 and 0.8 recovered within ±0.1, slope ≈ −2(1+H), $R^2$>0.95 |
| Noise tests (113-147) | White noise: Sq = σ, corrected ≈0; Sdq and rugosity grow as the grid is refined; noise correction recovers a sinusoid's Sq |
| Holes, edge trim, ROI (163-193) | Metrics stable within 2–5% when small holes are filled and large ones masked, and under a 10 mm edge trim |
| `test_tilted_camera_frame_is_levelled` (196-205) | 25° tilt removed within 0.2° |
| CLI tests (test_cli.py) | Output files and CSV columns; append semantics; **determinism** (same seed → identical CSV; a different seed changes only the bootstrap CIs); config plus flag override; unknown config key fails; noise scan; validation against `Evaluate/surfaces/egg_carton.toml` (err < 1%, λ = 50/√2 mm); loud failures for empty scans, coarse grids, empty ROIs and bad parameters |

The egg-carton validation test links this tool to the same reference surface that Filter_Sweep and Evaluate use for Small_Wave.

## 10. Observations

1. **Untracked and depends on an uncommitted change.** The whole folder is untracked. `roughness_scans.py:273` and `pipeline.py:479` call `append_results(..., lead_columns=...)`, but the `lead_columns` parameter exists only in the **uncommitted** working-tree diff of `Processing/Evaluate/scan_eval/report.py` (`git diff` shows +4/−3 lines adding it). At HEAD this would raise `TypeError`. Both need committing together.
2. **Provenance fields are misleading while untracked.** `versions()` records `git rev-parse HEAD` and `git status --porcelain -- .` (pipeline.py:291-297). Because the folder is untracked, `git_dirty` is always True and `git_commit` names a commit that does not contain this code. The `roughness_code_sha256` hash is the reliable identifier.
3. **No `requirements.txt`.** It relies on Evaluate's venv (README.md:66).
4. **Std convention.** Std is reported as wave-vector angle + 90° (lay direction). ISO software may report the angular-spectrum maximum directly, so this needs stating in the thesis to avoid a 90° ambiguity (README.md:41 documents it). Resolution is limited by the k-grid, as the README acknowledges.
5. **The DETREND warning baseline depends on the method list.** If `planar` is not among the methods, `base = 0` and the "extra variance" includes the plane's share (pipeline.py:375) **[minor]**.
6. **Private API use.** `_quiet` manipulates `scan_eval.log._files`, a private list (pipeline.py:99-106), which couples the tool to Evaluate internals.
7. **The noise scan inherits `edge_trim_mm` but not the ROI** (pipeline.py:196). This is probably intended, but undocumented.
8. **Domain vocabulary.** The tool is described for "sediment bed / ripples" rather than sea ice. The metrics are generic, but the thesis should explain the transfer.
9. **Sdr/rugosity are grid-dependent** (Sdq ∝ 1/g for noise). Results at different `grid_mm` values are not comparable. The tool flags this (NOISE) but does not normalise it.
10. **No ISO S/L filtering** (see §7). It is acceptable for relative comparison, but "ISO 25178" labels in the column names (README.md:36) overstate conformance.
11. **Good practice noted**: the defaults are thoroughly documented, the parameters are validated, the outputs are seeded and deterministic, closed-form test oracles are used, and warnings are machine-readable flags.

*Derived from reading source only. No code or tests were executed.*
