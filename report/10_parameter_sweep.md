# 10 — Parameter Sweep: ranking RealSense capture settings by geometric error (`Processing/Parameter_Sweep`)

**Summary.** `Processing/Parameter_Sweep` is a batch-analysis driver for the hardware-parameter characterisation experiment. It finds every scan captured by `rs_marker` under a data root (default an external drive), with two printed test objects: `Big_Pyramid` and `Big_Wave`. For each scan it:

1. parses `scan_info.txt` into typed sweep parameters (resolution, depth units, visual preset, and others);
2. crops the cloud with the Crop tool's automatic AprilTag crop;
3. computes a seed pose from the table plane;
4. calls Evaluate's `scan_eval.pipeline.evaluate_scan` against the object's CAD STL;
5. applies a coverage-based registration sanity check.

Results are cached per scan and gathered into `results/all_scans.csv`. From that table it writes per-object and overall rankings of parameter configurations and a set of thesis-ready figures: parameter effect plots, cross-object comparison with Spearman ρ, interaction heatmaps, completeness–error trade-offs and diagnostics.

The data set analysed is a full factorial of 3 resolutions × 3 depth units × 5 presets with one scan per configuration per object, so 90 scans. Laser power, exposure and filter chain are fixed. The tool reimplements no geometry: alignment and metrics come entirely from `Processing/Evaluate`, cropping from `Processing/Crop`. Everything below comes from reading the source and inspecting existing output files under `results/` (git-ignored). Nothing was run.

## Files covered

| Path (repo-relative) | Lines | Role |
|---|---|---|
| `Processing/Parameter_Sweep/README.md` | 157 | Usage, pipeline description, data-set notes, output catalogue |
| `Processing/Parameter_Sweep/requirements.txt` | 9 | open3d, numpy, scipy, sympy, matplotlib, pandas, tqdm |
| `Processing/Parameter_Sweep/run_analysis.py` | 86 | CLI: evaluate → write CSV → rank → plot |
| `Processing/Parameter_Sweep/sweep/config.py` | 84 | Paths, object→reference map, crop/seed/check constants, Evaluate overrides, cache fingerprint settings |
| `Processing/Parameter_Sweep/sweep/scan_info.py` | 127 | Scan discovery, `scan_info.txt` parser, parameter derivation, previous-preset lookup |
| `Processing/Parameter_Sweep/sweep/prepare.py` | 103 | AprilTag crop wrapper (../Crop), table-plane seed pose |
| `Processing/Parameter_Sweep/sweep/batch.py` | 253 | Per-scan evaluation, caching, failure capture, registration check, DataFrame assembly |
| `Processing/Parameter_Sweep/sweep/analysis.py` | 156 | Varied-parameter detection, crossed-pair detection, metric/param metadata |
| `Processing/Parameter_Sweep/sweep/ranking.py` | 80 | Per-object and overall rankings |
| `Processing/Parameter_Sweep/sweep/plots.py` | 427 | All figures (PNG 300 dpi + PDF) |

Git history: everything was added in one commit, `00b2c4e` (2026-10-06, "Added python code to evaluate performance across parameters, and python code to evaluate performance across different filters"). There have been no later changes. `results/` and `.venv/` are git-ignored (`Parameter_Sweep/.gitignore`).

## Contents

1. Purpose
2. Inputs and outputs
3. Step-by-step walkthrough
4. Algorithms and methods
   - 4.1 `scan_info.txt` parsing and parameter derivation
   - 4.2 Crop
   - 4.3 Seed pose
   - 4.4 Evaluation via Evaluate
   - 4.5 Registration check
   - 4.6 Caching
   - 4.7 Varied-parameter and crossed-pair detection
   - 4.8 Ranking
   - 4.9 Figures and statistics
5. Why this approach
6. Parameters and tuning
7. Assumptions, limitations, failure modes
8. Dependencies
9. What the existing results show
10. Observations

---

## 1. Purpose

The thesis question here is which RealSense D456 capture settings give the most accurate surface geometry, and whether the best settings carry over between objects of different shape. The tool turns a folder of sweep captures into:

- a per-scan metrics table;
- ranked configurations per object and overall;
- figures showing the effect of each varied parameter and their interactions.

It sits after Capture (`rs_marker` sessions with `scan_info.txt`, PLY, detections CSV, IMU CSV) and reuses Crop and Evaluate as libraries **[EXPLICIT: README.md:1-16; run_analysis.py:2-11]**.

## 2. Inputs and outputs

### 2.1 CLI (`run_analysis.py:25-74`)

| Flag | Default | Meaning |
|---|---|---|
| `--data` | `/run/media/joab/Seagate/roughness/Joab_Characterisation/Parameter_Error` (`config.py:15`) | Sweep root; read only |
| `--out` | `Processing/Parameter_Sweep/results` | Output root; must not be inside `--data` (`run_analysis.py:45-46`) |
| `--objects` | `Big_Pyramid,Big_Wave` | Subset of `config.OBJECTS`; unknown names are an error |
| `--sessions` | all | Comma list. Output goes to `subset_scans.csv`, and the run stops before ranking/plots (`run_analysis.py:58-63`) |
| `--force` | off | Ignore cache |
| `--retry-failed` | off | Re-evaluate cached `failed` rows |
| `--plots-only` | off | Skip evaluation; rank and plot from `all_scans.csv` |
| `--no-plots` | off | Evaluate only |
| `--metric` | `acc_rmse_mm` | Metric for rankings, heatmaps and object comparison |
| `--include-suspect` | off | Plot `suspect` rows too (rankings always use `ok` only, `ranking.py:34`) |
| `-v` | off | Show Evaluate's console output (otherwise stdout/stderr are sent to `/dev/null` with a tqdm bar) |

### 2.2 Input data layout (`scan_info.py:3-10`)

```
<data>/<Object>/<session>/<session>_r<rot>/<session>_r<rot>_a<tilt>/
    <..>.ply  <..>.db3  <..>_detections.csv  imu_a<tilt>.csv  scan_info.txt
```

- A scan is any folder containing `scan_info.txt`, found by recursive glob.
- The session is the first path component under the object folder.
- The first sorted `*.ply` in the folder is used (`find_scans`, `:40-50`).
- PLYs are in metres (`config.py:62`, "rs_marker PLYs are metres"). The STL references are in mm.

### 2.3 Outputs (`README.md:114-128`)

| Path | Content |
|---|---|
| `results/all_scans.csv` | One row per scan. Lead columns (`batch.py:33-38`): object, label, session, status, failure_reason, resolution, res_megapixels, depth_units_mm, preset, preset_actual, preset_mismatch, preset_previous, then key metrics. After those: all other Evaluate row fields, crop/seed info, `align_*`, and raw `info.<key>` columns last |
| `results/subset_scans.csv` | Same, for `--sessions` runs |
| `results/rankings/ranking_<object>.csv` | Configurations ranked by `--metric` with PANEL_METRICS, `n_scans`, `sessions`, `rank` |
| `results/rankings/ranking_overall.csv` | Configurations present for every object; per-object metric and rank, `mean_rank`, `mean_<metric>`, `overall_rank` |
| `results/plots/param_<p>.{png,pdf}` | 2×3 metric panels against each varied parameter |
| `results/plots/object_comparison.{png,pdf}` | Pyramid vs wave scatter per configuration (metric and F@2mm) plus rank–rank plot |
| `results/plots/heatmap_<a>_x_<b>.{png,pdf}` | Median metric over parameter pairs, per object |
| `results/plots/tradeoff_<p>.{png,pdf}` | completeness_full@2mm vs metric, coloured by parameter |
| `results/plots/diagnostics.{png,pdf}` | Outlier influence, registration quality, spread |
| `results/eval_runs/<Object>__<session>/` | Evaluate per-scan output (`metrics.json`, `transform.txt`, `run.log`, plots) plus `seed_transform.txt`. Also Evaluate's own appended `eval_runs/results.csv` |
| `results/cropped/<Object>/<session>_cropped.ply` | Cropped cloud, written by Crop's `write_ply_like_original` |
| `results/cache/<Object>/<session>.json` | `{"fingerprint", "row"}` |

`status` values: `ok`, `suspect` (failed the registration check) or `failed` (an exception in a named stage).

## 3. Step-by-step walkthrough

1. **`main`** (`run_analysis.py:25`). Parse args, resolve paths, refuse an `--out` inside `--data`, create `--out`.
2. **Evaluation phase** (unless `--plots-only`). `Evaluator(data, out).run(objects, sessions, force, retry_failed, verbose)` (`batch.py:188`):
   1. **Discover** with `find_scans` (`scan_info.py:40`). `previous_presets` (`:113-127`) records, for every scan of each object in session (time) order, the preset requested by the previous capture. It uses all scans, before the `--sessions` filter.
   2. **Check the cache.** For each scan, compute the `fingerprint` (`batch.py:92-102`) and look up the cache (`cached`, `:107-115`). A hit is reused unless `--force`, or unless `--retry-failed` and the cached row failed. Cached rows go straight to `check_registration`.
   3. **Evaluate the rest** under tqdm. Output is redirected to `/dev/null` unless `-v`. `evaluate_one` (`:125-185`) does:
      1. Build the row: object, label, session, paths. Add `derive_parameters(info)` and all raw `info.*` keys.
      2. **Stage `load`.** No PLY → `PrepareError`.
      3. **Stage `crop`.** `prepare.crop_scan` (`prepare.py:36-51`) writes `results/cropped/<obj>/<session>_cropped.ply` and records `points_raw`, `points_cropped`, `crop_notes`.
      4. **Stage `seed`.** Load the reference (`Evaluator.reference`, `batch.py:72-81`: `load_reference(STL, units_override="mm", height_field=True)` wrapped in `UnitScaled`, cached per object). Read the cropped cloud and call `prepare.seed_transform(pts, ref.bounds())` (`prepare.py:54-103`).
      5. **Stage `evaluate`.**
         - Save `seed_transform.txt`.
         - Build `EvalParams` from `EVAL_OVERRIDES` (`batch.py:83-89`): output = `eval_runs`, label = object label, `init_transform` = seed path.
         - Call `evaluate_scan(ref, cropped, params, runs_dir, "<Object>__<session>", T0)`. Evaluate runs its **full** multi-start alignment with the seed as one extra candidate.
         - Merge the returned row, except `label`/`scan`.
         - Read back `metrics.json` to add `align_chosen`, `align_overlap_scan`, `align_overlap_ref`, `align_candidate_rmse_mm` (`:167-172`).
         - Set `status=ok`.
      6. **Any exception** (except `KeyboardInterrupt`) gives `status=failed` and `failure_reason="<stage>: <first line>"` (`:176-181`).
      7. Record `note` when there are several PLYs, and `elapsed_s`.
   4. **Store** the row in the cache (atomic tmp + `os.replace`, `:117-122`). Apply `check_registration`. Print non-ok rows.
   5. Add `preset_previous` and build the DataFrame (`to_frame`, `:248-253`): lead columns, then others, then `info.*`, sorted by object and session.
3. **Write** `all_scans.csv` (or `subset_scans.csv`) and print the status counts and every non-ok row (`report_status`, `run_analysis.py:77-82`). A `--sessions` run ends here.
4. **Ranking and plotting** (unless `--no-plots`):
   1. Re-read `all_scans.csv` with pandas.
   2. `ranking.write_rankings(df, out/rankings, metric)` (`ranking.py:32`).
   3. `plots.make_all(df, out/plots, metric, include_suspect)` (`plots.py:407`).

## 4. Algorithms and methods

### 4.1 `scan_info.txt` parsing and parameter derivation (`sweep/scan_info.py`)

**Parsing** (`read_scan_info`, `:53-60`). Each line is `key: value`, split on the first colon. `#` header lines and blank lines are skipped. Section names are not used: keys are flat strings such as `Stereo Module.Laser Power` **[INFERRED from the keys used]**.

**Derived parameters** (`derive_parameters`, `:78-110`):

| Column | Source | Derivation |
|---|---|---|
| `resolution`, `res_width`, `res_height`, `res_megapixels`, `fps` | `resolution` | regex `(\d+)x(\d+)(?:@(\d+))?`; megapixels = w·h/1e6 |
| `depth_units_mm` | `depth_units_m` or `Stereo Module.Depth Units` | ×1000, rounded to 4 dp |
| `preset` | `preset` (rs_marker `-p` name) | as requested |
| `preset_actual` | `Stereo Module.Visual Preset` | text inside trailing `(...)` (`_enum_name`) |
| `preset_mismatch` | both | actual ≠ `EXPECTED_VISUAL_PRESET[preset]` (`:21-27`); `remove_ir_pattern` is expected to report `Custom` |
| `laser_power`, `exposure_us`, `gain`, `auto_exposure`, `emitter` | `Stereo Module.*` | first token parsed as number; emitter via `_enum_name` |
| `filter_chain`, `frames_captured`, `best_frame` | same-named keys | — |
| `valid_pixel_fraction` | `best_valid_pixels` "a / b" | a/b |
| `camera`, `camera_serial`, `firmware` | `device.*` | — |

**Swept capture parameters.** Code and README agree: depth **resolution** (640×360, 848×480, 1280×720), **depth units** (0.1, 0.5, 1.0 mm), and **visual preset** (default, high_accuracy, high_density, medium_density, remove_ir_pattern). Laser power 150 mW, exposure 8500 µs with auto exposure enabled, and the filter chain `threshold > to_disparity > spatial > temporal > to_depth > hole_fill (decimation off)` are constant across all 90 scans **[EXPLICIT: README.md:134-137; confirmed by reading `results/all_scans.csv`]**.

The code is also generic. Any other parameter in `KNOWN_PARAMS`, or any raw `info.Stereo Module.*`/`info.filter.*`/`info.RGB Camera.*`/`info.Motion Module.*` setting that varies, would be picked up automatically (§4.7).

### 4.2 Crop (`prepare.crop_scan`, `prepare.py:36-51`)

- Imports `../Crop/crop_pointclouds.py` as a module (`prepare.py:15-17`).
- Learns the corner-tag layout once per object folder (`crop_tool.learn_tag_layout`, cached in `_layouts`, `:27-33`), as the Crop tool does for its input directory **[EXPLICIT: :28-29]**.
- Calls `crop_tool.compute_auto_crop(pcd, ply, object_dir, layout, args)` with `margin=10 px` and `frame_window=15`. It uses the scan's `imu*.csv` and `*_detections.csv`: AprilTag corners → crop quad shrunk by the margin → mask of points whose projection lies inside.
- Keeps `pcd.select_by_index(mask)` and writes it with `write_ply_like_original`.

The AprilTag geometry itself belongs to the Crop report and is not re-derived here. `AutoCropError` becomes `PrepareError("crop: ...")`.

### 4.3 Seed pose (`prepare.seed_transform`, `prepare.py:54-103`)

**Purpose.** Even after cropping, the part is only about 4 % of the points, and on the wave Evaluate's moments start locks onto the table. The README reports one wave scan going from 69 % coverage and −8 mm Z bias without the seed to 99.9 % coverage and −0.46 mm with it **[EXPLICIT: README.md:65-68; config.py:20-23]**.

**Steps:**

1. **Downsample.** Drop non-finite points; voxel-downsample at 4 mm. At least 100 points are required.
2. **Table plane by RANSAC.** Open3D `segment_plane(distance_threshold=3 mm, ransac_n=3, num_iterations=2000)` gives the plane $ax+by+cz+d=0$; normalise to $\mathbf n$, $d$. If $d<0$, flip both, so the camera at the origin is on the positive side: the signed height of the origin is $\mathbf n\cdot\mathbf 0+d=d>0$. This is plane fitting by RANSAC (Fischler & Bolles 1981) as implemented by Open3D.
3. **Raised points.** Points with height $h=\mathbf n\cdot\mathbf p+d$ in (5 mm, 120 mm). At least 50 are required.
4. **Object cluster by DBSCAN.** Open3D `cluster_dbscan(eps=6 mm, min_points=20)` (DBSCAN, Ester et al. 1996). Among clusters with ≥ 50 points, pick the one whose centroid is nearest the optical axis (perpendicular distance $\lVert\bar{\mathbf q}\times\hat{\mathbf z}\rVert$), on the stated basis that "the camera looks at the part" **[EXPLICIT: prepare.py:79]**.
5. **Frame.**
   - $\mathbf e_1=\mathbf n\times\hat{\mathbf y}/\lVert\cdot\rVert$, $\mathbf e_2=\mathbf n\times\mathbf e_1$, $R=[\mathbf e_1;\mathbf e_2;\mathbf n]$ (rows). That is a proper rotation, since $\mathbf e_1\cdot(\mathbf e_2\times\mathbf n)=1$.
   - The footprint centre is the midpoint of the 1st–99th percentile range of the cluster's $(\mathbf e_1,\mathbf e_2)$ coordinates.
   - $T = [R \mid (c_x^{ref}-c_1,\ c_y^{ref}-c_2,\ d)]$.
   - So $z' = \mathbf n\cdot\mathbf p+d$ = height above the table, and the footprint is centred on the reference bounding-box $xy$ centre.
   - Yaw is arbitrary; Evaluate's quarter turns cover it **[EXPLICIT: :57]**.
6. **Info.** Returns $T$ plus `seed_object_points`, `seed_footprint_x/y_mm` and `seed_table_inliers`.

Implicit assumption: the reference STL's base sits at $z=0$, so table height = reference base. The code does not check this **[INFERRED]**.

### 4.4 Evaluation via Evaluate (`batch.py:154-172`, `config.py:60-70`)

Evaluate is used unmodified. The overrides (`EVAL_OVERRIDES`) are:
- `units="m"` and `reference_units="mm"`;
- `seed=0`, `align="full"`;
- `write_ply=False`, `write_overlay=False`, `write_plots=True`;
- `HEIGHT_FIELD=True` for both STLs.

Everything else uses Evaluate's defaults:
- voxel = reference diagonal/60, τ = 3 voxel;
- 5 RANSAC starts plus perturbations;
- thresholds 0.5/1/2/5 mm, coverage radius 10 mm, region margin 0, 200 000 reference samples, trims 1 % / 5 %.

The seed is passed both as `params.init_transform` (for provenance in `metrics.json`) and as the `init_transform` argument. It becomes the `init-transform` candidate inside Evaluate's multi-start. Identity, moments, RANSAC and perturbations still compete, and the lowest truncated-LS score wins (see report 09 §4.5). Metric definitions are those of report 09 §4.6.

The README reports that three scans per object were re-run with `evaluate_scans.py` directly: "All 71 numeric metrics agree: exactly on 5 scans, and to ≤ 0.005 mm on the sixth", attributed to Open3D's parallel ICP **[EXPLICIT: README.md:110-112]**. This audit has not reproduced it.

Object → reference (`config.py:30-33`):

| Object folder | Label | Reference |
|---|---|---|
| `Big_Pyramid` | Pyramid | `Processing/Evaluate/References/PYRAMID_1.stl` |
| `Big_Wave` | Wave | `Processing/Evaluate/References/EGG_1PK.stl` |

### 4.5 Registration check (`batch.check_registration`, `batch.py:231-245`)

Rule: `status = suspect` if `covered_fraction` < `MIN_COVERED_FRACTION = 0.85` or is missing; otherwise `ok`. Failed rows are left alone. The check is re-applied on every run and is not cached, so the threshold can change without re-evaluation **[EXPLICIT: :232-234; README.md:89-90]**.

The README gives the evidence for the threshold:
- RMSE alone cannot detect a bad registration, because a pose slid off the part can drop the hardest points and get a *lower* RMSE.
- Misregistered wave scans (without the seed) covered 69–81 %, and correctly aligned scans cover ≥ 90 %, so 85 % separates them.
- Evaluate's own `align_overlap_ref` was rejected because it is also low (63–77 %) on correctly aligned 640×360 wave scans with holes.

**[EXPLICIT: README.md:79-90; config.py:26-33]**

### 4.6 Caching (`batch.py:91-122`)

The fingerprint is SHA-256 (first 20 hex chars) of a sorted JSON of:
- `[name, size, mtime_ns]` for the PLY, `scan_info.txt`, the first `*_detections.csv` and the first `imu*.csv`;
- the reference path plus its stamp;
- `config.eval_settings()`: `PIPELINE_VERSION`, crop margin and window, seed constants, `EVAL_OVERRIDES`, `HEIGHT_FIELD`.

A mismatch triggers re-evaluation. Failed scans are cached too, with their reason **[EXPLICIT: README.md:47-53]**.

Code in `../Evaluate` and `../Crop` is **not** fingerprinted; `PIPELINE_VERSION` must be bumped by hand (`config.py:18-20`).

### 4.7 Varied-parameter and crossed-pair detection (`sweep/analysis.py`)

**`varied_parameters`** (`:87-109`):
- Takes every `KNOWN_PARAMS` column (`:65-76`) with more than one distinct value.
- Then scans raw `info.` camera-setting columns (`SETTING_PREFIXES`, `:78`), excluding `Visual Preset` (`:80`).
- A raw column is skipped if its row partition equals a known parameter's, or is a refinement of an already accepted partition (each of its groups is contained in some group of an existing partition). `_partition` (`:83-84`) groups row indices by value, so e.g. `Stereo Module.Depth Units` duplicates `depth_units_mm`.
- Raw columns that survive are typed numeric if they all parse as numbers, otherwise categorical.

On this data set the result is `resolution`, `depth_units_mm` and `preset` (from the produced plot names).

**`crossed_pairs`** (`:135-147`). A parameter pair counts as crossed if, within some object, two levels of A share at least 2 levels of B, i.e. a 2×2 sub-grid exists. A one-factor-at-a-time (OFAT) design has no such pair and gets no heatmaps **[EXPLICIT: :136-138]**. The full grid here gives all three pairs.

**`config_key`** (`:150-151`): the string join of the varied parameter values identifies a configuration.

**`usable`** (`:154-156`): `ok` rows, plus `suspect` when requested.

### 4.8 Ranking (`sweep/ranking.py`)

**Per object** (`_per_object`, `:14-29`):
1. Group by (object_label, *varied params*) with `dropna=False`.
2. Take the arithmetic mean of the metric and the other PANEL_METRICS, the group size `n_scans`, and the joined `sessions`.
3. Rank ascending if the metric is lower-is-better, descending otherwise, with `method="min"` (ties share the lowest rank). For `signed_bias_mm` the key is $|\text{bias}|$, so closest to zero is best (`:23-25`).

**Overall** (`:52-61`):
1. Pivot to wide with per-object metric and rank columns, and drop configurations missing on any object (`dropna`).
2. Compute $\text{mean\_rank} = \frac1K\sum_k \text{rank}_k$ over the $K$ objects, and the mean metric.
3. Sort by `mean_rank`, then by mean metric (tie-break), and assign `overall_rank` = 1…n.

This is an unweighted mean-rank (Borda-count-like) aggregation, which is scale-free across objects whose baseline errors differ. The scale-free property is **[INFERRED]** as the motivation; the comment only says "configurations measured on every object, by mean rank" (`:52`).

**Console output.** Best and worst 5 per object and overall (`SHOW=5`, `_print_block`).

### 4.9 Figures and statistics (`sweep/plots.py`)

Common settings: light theme, 7-inch text width, fixed categorical palette and markers, saved as PNG (300 dpi) and PDF with TrueType fonts (`pdf.fonttype=42`) (`:31-63`).

**`plot_parameter`** (`:110-172`). A 2×3 grid of the PANEL_METRICS (`analysis.py:45-46`): RMSE, RMSE trim 5 %, signed bias, signed std, F@2mm, completeness_full@2mm. Objects are dodged side by side.
- *Numeric* parameter: per-level median line and an interquartile band (25th–75th percentile).
- *Categorical* (preset): Matplotlib box plots (median, IQR box, whiskers at 1.5 IQR, no fliers).
- Individual scans are overlaid as points with seeded jitter.
- Depth units use a log-x axis. Resolution is placed in megapixel order but drawn at equal spacing (`_x_positions`, `plots.py:92-99`; `log_x` only for depth units).
- The legend shows n per level.

**`plot_object_comparison`** (`:176-248`). For each configuration, the pyramid value is plotted against the wave value for the main metric and F@2mm, with a $y=x$ line and the **Spearman rank correlation** $\rho$ computed by pandas `corr(method="spearman")`:
$$\rho = 1-\frac{6\sum d_i^2}{n(n^2-1)}$$
This form is exact when there are no ties; pandas computes the Pearson correlation of the ranks. A third panel plots rank against rank and counts how many of each object's top $\lfloor n/5\rfloor$ configurations are shared. Points are coloured by preset.

**`plot_heatmap`** (`:252-305`). For each crossed pair and each object: the **median** of the metric per cell, pooled over the remaining parameter(s). Each object has its own colour scale (because "the objects differ in baseline error", `:269-270`). Values are printed in cells, and missing cells say "not measured". The note gives the scans per cell (here 5 for resolution × depth units, pooled over presets, and 3 for the pairs with preset).

**`plot_tradeoff`** (`:309-336`). For each varied parameter: completeness_full@2mm (falling back to covered_fraction) on x against the main metric on y, per object, coloured by level.

**`plot_diagnostics`** (`:340-404`). Uses ok and suspect rows.
1. RMSE vs RMSE-trim5, with the min–max ratio, to show outlier influence.
2. Covered % vs RMSE, with suspect rows hollow and the 85 % line drawn.
3. Box plot of RMSE per object, labelling Tukey outliers (> Q3 + 1.5 IQR) with the session time.

**Statistical content, in total:**
- **Descriptive only:** means (rankings), medians/IQR (plots, heatmaps), boxplots, Spearman ρ, Tukey fences.
- **No inferential statistics:** no confidence intervals, significance tests, ANOVA/main-effect decomposition, or replicate-based noise estimate.

## 5. Why this approach

| Decision | Rationale | Label |
|---|---|---|
| Reuse Evaluate and Crop unmodified | "Evaluation is not reimplemented … Neither folder is modified." Keeps one definition of every metric | **[EXPLICIT: README.md:12-16]** |
| Seed pose from the table plane | Moments start locks onto the table; measured improvement on a wave scan | **[EXPLICIT: README.md:65-74; config.py:20-23]** |
| Keep Evaluate's full multi-start around the seed | Turns/flips/slides/RANSAC still guard against a wrong seed | **[EXPLICIT: README.md:72-74]** |
| Coverage check instead of RMSE or Evaluate overlap | Slid poses lower RMSE; Evaluate overlap is confounded by holes | **[EXPLICIT: README.md:79-90]** |
| Check not cached | Threshold can change without re-evaluating | **[EXPLICIT: batch.py:232-234]** |
| Per-scan cache with fingerprint | About 5–8 s per scan, about 10 min for 90; re-runs redo only changed scans | **[EXPLICIT: README.md:45-53]** |
| No per-scan PLYs or overlay | "skip the big per-scan PLYs" | **[EXPLICIT: config.py:66-67]** |
| Pyramid ambiguity not treated as failure | Quarter turns are true symmetries, so the competing poses are equivalent | **[EXPLICIT: README.md:97-99]** |
| Heatmaps only for crossed pairs | An OFAT sweep cannot show interactions | **[EXPLICIT: analysis.py:136-138; README.md:123]** |
| Per-object colour scale in heatmaps | Objects differ in baseline error; the within-object pattern is what matters | **[EXPLICIT: plots.py:269-270]** |
| Light theme, PDF + PNG | Thesis figures are printed | **[EXPLICIT: plots.py:31]** |
| Mean rank for overall ranking | Combines objects with different error scales | **[INFERRED]** |
| Data dir read-only, `--out` refused inside it | Protect raw data | **[EXPLICIT: README.md:38; run_analysis.py:45-46]** |

**Alternatives not taken [INFERRED]:**
- A factorial ANOVA or linear model of the metric on the three factors would quantify main effects and interactions with uncertainty. With n = 1 per cell, a full three-way model has no residual degrees of freedom, but a main-effects + two-way model would.
- Replicate captures per configuration would allow confidence intervals on rankings.
- Ranking on a robust metric (trimmed RMSE) or a multi-criteria score (RMSE together with completeness) would penalise settings that look accurate only because they return fewer points. The trade-off plots expose this visually but the ranking does not use it.

## 6. Parameters and tuning (`sweep/config.py`)

| Name | Value | Location | Effect |
|---|---|---|---|
| `DEFAULT_DATA` | `/run/media/joab/Seagate/.../Parameter_Error` | config.py:15 | Sweep input root (external drive) |
| `DEFAULT_OUT` | `Parameter_Sweep/results` | :16 | Output root |
| `PIPELINE_VERSION` | 1 | :20 | Manual cache invalidation |
| `OBJECTS` | Big_Pyramid→PYRAMID_1.stl, Big_Wave→EGG_1PK.stl | :30-33 | Object/reference map |
| `CROP_MARGIN_PX` | 10.0 | :37 | Crop quad pulled in from tag inner corners |
| `CROP_FRAME_WINDOW` | 15 | :39 | Frames around the exported one used to fill missing tags |
| `SEED_MIN_HEIGHT_M` / `SEED_MAX_HEIGHT_M` | 0.005 / 0.12 | :46-47 | Height band above table for object points |
| `SEED_CLUSTER_EPS_M` | 0.006 | :48 | DBSCAN eps |
| Seed internals | voxel 4 mm, plane threshold 3 mm, 2000 RANSAC iterations, DBSCAN min_points 20, cluster ≥ 50, footprint 1–99 % | prepare.py:59-92 | Hard-coded; not in fingerprint |
| `MIN_COVERED_FRACTION` | 0.85 | :58 | Registration check |
| `EVAL_OVERRIDES` | units m, reference mm, seed 0, align full, no PLY/overlay, plots on | :61-69 | Evaluate settings |
| `HEIGHT_FIELD` | True | :70 | Reference treated as $z=h(x,y)$ (outline region, Z error, +z visible) |
| `PRIMARY_METRIC` | `acc_rmse_mm` | :73 | Default ranking metric |
| `PANEL_METRICS` | RMSE, trim5 RMSE, bias, std, F@2, completeness_full@2 | analysis.py:45-46 | Parameter-plot panels and ranking extras |
| `SHOW` | 5 | ranking.py:11 | Rows printed per block |

## 7. Assumptions, limitations, failure modes

**Design and statistics:**
- **One scan per configuration.** Every ranking cell has `n_scans = 1` (confirmed in `results/rankings/ranking_wave.csv`). Mean aggregation is therefore trivial, and rank differences between neighbouring configurations may be within capture-to-capture noise. No repeatability estimate exists in this data set.
- **The ranking metric is RMSE after a best rigid fit**, over points inside the reference footprint (report 09 §7). A configuration that produces fewer points, or fills holes poorly, is not penalised in RMSE. That shows up only in completeness/coverage, which the ranking ignores unless `--metric` is changed.

**Seed pose assumptions:**
- the largest plane in the cropped cloud is the table;
- the object is the raised cluster nearest the optical axis, 5–120 mm high;
- the reference base is at $z=0$;
- the camera is not looking exactly along ±y (the $\mathbf e_1$ cross product would degenerate).

Failures raise `PrepareError` and are recorded as `failed: seed: ...`.

**Crop** depends on detections and IMU files next to the PLY. Missing files give `failed: crop: ...`.

**Determinism:**
- `segment_plane` and `cluster_dbscan` are called without an explicit seed. Open3D's global RNG is reseeded inside Evaluate's RANSAC calls (`registration.py:150`), so the seed pose of the first scan in a process depends on the unseeded RNG, and later scans depend on the previous scan's last RANSAC seed state **[INFERRED]**.
- Caching hides this nondeterminism on re-runs, but `--force` could change seeds slightly. The effect on the final pose is probably negligible, because the full multi-start and exact refinement follow **[INFERRED]**.

**Data-set specific notes** **[EXPLICIT: README.md:132-145]**:
- `remove_ir_pattern` reports "Custom" with the laser on, and its point counts follow the previous preset, so it may not be a distinct mode. `preset_previous` lets this be checked.
- Wave session `20261002_163703` requested `default` but reports Custom (`preset_mismatch=True`). It is kept as `default`.
- Big_Wave `scan_info.txt` paths say `Small_Wave` (the folder was renamed). The object name comes from the folder.
- The pyramid is always `align_ambiguous` because of 4-fold symmetry. That is benign for the metrics, since symmetric poses give identical errors.

## 8. Dependencies

**Internal:**
- `Processing/Evaluate/scan_eval`: `EvalParams`, `UnitScaled`, `surface_def.load_reference`, `pipeline.evaluate_scan`. Imported by `sys.path` insertion (`batch.py:27-30`).
- `Processing/Crop/crop_pointclouds.py`: `learn_tag_layout`, `compute_auto_crop`, `AutoCropError`, `write_ply_like_original` (`prepare.py:15-17`; definitions at `crop_pointclouds.py:121,162,428,693`).
- Inputs produced by `Capture/rs_marker`: PLY (metres, camera frame), `scan_info.txt`, `*_detections.csv`, `imu*.csv`, `.db3`.
- Reference STLs in `Processing/Evaluate/References/`, which is git-ignored.

**Third-party (`requirements.txt`; "Same versions as ../Evaluate plus pandas and tqdm"):**

| Library | Use |
|---|---|
| open3d ≥ 0.19 | PLY I/O, `voxel_down_sample`, `segment_plane` (RANSAC plane fit), `cluster_dbscan` (DBSCAN), `select_by_index`; plus everything Evaluate uses |
| numpy ≥ 1.26 | Arrays, percentiles, linear algebra |
| scipy, sympy | Required transitively by Evaluate |
| matplotlib ≥ 3.8 | Figures (Agg backend), box plots, imshow heatmaps |
| pandas ≥ 2.1 | DataFrame assembly, groupby/pivot, `rank`, `corr(method="spearman")` |
| tqdm ≥ 4.66 | Progress bar |

scikit-image is not listed. It is needed by Evaluate only for implicit references, which this tool does not use.

## 9. What the existing results show

These are read from git-ignored `results/` files; the audit did not run anything.

- **Last full run.** `results/full_run.log` records "90 scan(s): 0 cached, 90 to evaluate", about 10 min, `status: ok 90`.
- **Data set.** `all_scans.csv` has 45 Big_Pyramid + 45 Big_Wave rows, all `ok`. Resolution, depth units and preset each have 30/30/30 or 18×5 counts, consistent with a full 3×3×5 grid per object. Laser power (150), exposure (8500) and filter chain are constant.
- **Coverage:**
  - Pyramid covered_fraction ranges 0.970–1.0 and Wave 0.900–1.0, all above the 0.85 threshold.
  - Pyramid rows are all `align_ambiguous=True`; Wave rows are mixed.
- **RMSE:** Pyramid 1.18–2.30 mm; Wave 1.82–3.35 mm.
- **Overall ranking** (`ranking_overall.csv`, by `acc_rmse_mm`, mean rank):
  1. 1280×720 / 1.0 mm / high_accuracy (mean rank 1.5)
  2. 1280×720 / 0.1 mm / high_accuracy (2.5)
  3. 1280×720 / 0.5 mm / remove_ir_pattern and 1280×720 / 0.5 mm / high_accuracy (tied at 4.5, broken by mean RMSE)
- **Evaluate's own CSV** (`results/eval_runs/results.csv`) holds 180 data rows, i.e. two evaluation passes of 90 scans appended (see Observations).

The README's figure list matches the files present: 3 param plots, 3 heatmaps, 3 trade-off plots, object comparison and diagnostics.

## 10. Observations (bugs, bad practice, inconsistencies; not fixed)

1. **No replication, no uncertainty.**
   - Rankings and "best configuration" claims rest on a single capture per configuration, with no confidence intervals or tests.
   - Close-ranked configurations (e.g. mean ranks 4.5 vs 4.5, RMSE differences of a few hundredths of a mm) cannot be distinguished from noise with this design.
   - The README does not discuss this.
2. **`ranking.py:28` can crash on NaN.** `rank(...).astype(int)` raises if any configuration's metric is NaN, e.g. a metric column missing for a scan. The plots path handles NaN, but the ranking does not.
3. **`signed_bias_mm` is labelled lower-is-better** (`analysis.py:33`). Rankings and the rank–rank plot special-case it with `abs()`, but the heatmap colour-bar label still says "lower is better" (`plots.py:295`), and the trade-off axis direction is likewise wrong for that metric.
4. **The cache does not fingerprint the Evaluate or Crop code.** Any change in `../Evaluate/scan_eval` (e.g. registration or metric definitions) silently reuses stale cached rows unless `PIPELINE_VERSION` is bumped by hand (`config.py:18-20`). The seed internals (voxel 4 mm, plane threshold, DBSCAN min_points) are hard-coded in `prepare.py` and also not in `eval_settings()`.
5. **Evaluate's `eval_runs/results.csv` accumulates duplicates.** `evaluate_scan` appends a row on every (re-)evaluation. The sweep never reads it (it uses `all_scans.csv`), but it is misleading if opened (180 rows for 90 scans).
6. **Overall ranking with ordinal mean rank.**
   - It ignores effect size: a configuration that is 2nd on both objects by tiny margins outranks one that is 1st and 3rd by large margins.
   - Ranks are computed only among `ok` configurations per object. If an object loses configurations to `suspect`/`failed`, its rank scale differs from the other's, while `dropna` keeps only configurations present for both. Not an issue in the current all-ok data.
7. **Single-PLY assumption.** Only the first sorted PLY in a scan folder is evaluated, and the rest are noted (`batch.py:182-183`). The crop also learns the tag layout from the object directory, not from the session.
8. **Seed pose may not be reproducible.** No explicit seed is set for `segment_plane`/`cluster_dbscan` (see §7). `seed_transform.txt` is saved, which preserves reproducibility of the *evaluation* given the seed.
9. **Hard-coded external data path.** `config.py:15` points at a removable drive under a user-specific mount point. The tool is not portable without `--data`.
10. **`--sessions` writes `subset_scans.csv` and exits before plots.** `--plots-only` always reads `all_scans.csv`, so a subset cannot be plotted. That is documented behaviour, but easy to misread.
11. **Mixed provenance in `info.*`.** Raw `scan_info.txt` keys are stored verbatim, including absolute paths that refer to the old `Small_Wave` folder name. This is harmless, but the table carries stale paths.
12. **References are outside version control.** `PYRAMID_1.stl` and `EGG_1PK.stl` live under a git-ignored directory (`.gitignore:10`), so the ground truth for the published numbers is not versioned. The cache fingerprints them by size and mtime only, not content hash (`batch.py:55-59,99`). Evaluate's `reference_hash` column does record the SHA-256 of the file.
13. **Possible naming inconsistency elsewhere in the repo.** The most recent repo commit (`1e31b72`) mentions "small wave 641mm" in Bulk_Process, while this tool's wave object is `Big_Wave` → `EGG_1PK.stl`. Whether the two wave references are the same physical part cannot be determined from this component's source.
