# Parameter sweep analysis

Evaluates every scan of the RealSense hardware-parameter sweep against its
ground-truth STL, then plots and ranks the settings across the two objects.

```
scan_info.txt ─> parse ─┐
.ply ─> crop (../Crop) ─> seed pose ─> evaluate_scan (../Evaluate) ─> registration check ─> cache
                                                                         └─> all_scans.csv ─> rankings + plots
```

Evaluation is not reimplemented. Each scan goes through `../Evaluate`'s
`scan_eval.pipeline.evaluate_scan`, which does the alignment and all the
metrics, and through `../Crop`'s automatic AprilTag crop
(`compute_auto_crop`, margin 10 px, run without the preview window). Neither
folder is modified.

## Setup

```bash
cd Processing/Parameter_Sweep
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

## Usage

```bash
.venv/bin/python run_analysis.py                  # evaluate new/changed scans, then rank + plot
.venv/bin/python run_analysis.py --plots-only     # re-plot from results/all_scans.csv (no evaluation)
.venv/bin/python run_analysis.py --force          # re-evaluate everything
.venv/bin/python run_analysis.py --retry-failed   # re-evaluate only scans that failed
.venv/bin/python run_analysis.py --sessions 20261002_165154,20261002_162132 --no-plots   # spot check
```

| flag | default | |
|---|---|---|
| `--data` | `/run/media/joab/Seagate/roughness/Joab_Characterisation/Parameter_Error` | read only; nothing is written there |
| `--out` | `results/` (next to this README) | |
| `--objects` | `Big_Pyramid,Big_Wave` | |
| `--metric` | `acc_rmse_mm` | metric for rankings and heatmaps, any `all_scans.csv` column |
| `--include-suspect` | off | also plot scans that failed the registration check |
| `-v` | off | show Evaluate's console output instead of the progress bar |

About 5–8 s per scan, so the full 90 scans take about 10 minutes. Each result
is cached in `results/cache/<object>/<session>.json`. The cache key covers:

- the PLY, `scan_info.txt`, detections and IMU files (size and mtime);
- the reference STL;
- every setting in `sweep/config.py`.

Changing any of these re-evaluates only the affected scans. Failed scans are
cached too, with their reason; use `--retry-failed` or `--force` to retry them.

## What happens to each scan

1. **Parse** `scan_info.txt`. Every key is kept as an `info.<key>` column, and
   the sweep parameters are derived into typed columns:
   - `resolution`, `res_megapixels`, `depth_units_mm`;
   - `preset` (as requested), `preset_actual` (as the camera reported it);
   - `laser_power`, `exposure_us` and similar settings;
   - `valid_pixel_fraction`.
2. **Crop** with the Crop tool's automatic tag crop (`CROP_MARGIN_PX = 10`).
   The output goes to `results/cropped/`.
3. **Seed pose.** Even after cropping, the part is only about 4% of the
   points, and on the wave Evaluate's moments start locks onto the table.
   Measured on one wave scan: 69% coverage and a Z bias of −8 mm without the
   seed, against 99.9% coverage and −0.46 mm with it.
   - The seed fits the table plane and puts it at z = 0.
   - It takes the raised cluster nearest the optical axis and centres it on
     the reference.
   - It is passed to Evaluate as `--init-transform`, and Evaluate's full
     multi-start search (quarter turns, flips, slides, RANSAC) still runs
     around it.
4. **Evaluate** against the object's reference (mesh, mm, height field). The
   object-to-reference mapping is in `sweep/config.py`:
   - `Big_Pyramid` → `../Evaluate/References/PYRAMID_1.stl`;
   - `Big_Wave` → `../Evaluate/References/EGG_1PK.stl`.
5. **Registration check** (`MIN_COVERED_FRACTION = 0.85`). The scan is marked
   `suspect` if the aligned scan covers < 85% of the reference. A pose slid
   off the part can drop the hardest points and give a *lower* RMSE, so RMSE
   alone does not reveal a bad registration.
   - Measured on this data: misregistered poses (without the seed) covered
     69–81%, and every correctly aligned scan covers ≥ 90%.
   - Evaluate's pose overlap (`align_overlap_ref`) is kept in the table but
     not used for the check. It is also low (63–77%) on correctly aligned
     640×360 wave scans with holes, so it can't tell a bad pose from bad
     data.
   - The check runs on every invocation and is not cached, so changing the
     threshold needs no re-evaluation.

`status` is `ok`, `suspect` (registration check failed) or `failed` (missing
file, empty cloud, crop or seed failure, Evaluate error).
`failure_reason` says which stage failed and why. Plots and rankings use `ok`
rows only, unless `--include-suspect` is given.

The pyramid is always flagged `align_ambiguous` by Evaluate. Its quarter turns
are true symmetries, so the competing poses are equivalent, and this is not
treated as a failure.

### Reproducing one scan with Evaluate directly

```bash
cd ../Evaluate
python evaluate_scans.py --reference References/PYRAMID_1.stl --reference-units mm --height-field \
    --init-transform ../Parameter_Sweep/results/eval_runs/Big_Pyramid__<session>/seed_transform.txt \
    ../Parameter_Sweep/results/cropped/Big_Pyramid/<session>_cropped.ply
```

This was checked on 3 scans per object. All 71 numeric metrics agree: exactly
on 5 scans, and to ≤ 0.005 mm on the sixth. That difference is Open3D's
parallel ICP, as noted in Evaluate's README.

## Outputs (`results/`)

| path | |
|---|---|
| `all_scans.csv` | one row per scan: object, status, failure reason, parameters, every Evaluate metric, alignment details, crop/seed info, raw `info.*` |
| `rankings/ranking_<object>.csv` | configurations ranked by `--metric`, with the other headline metrics |
| `rankings/ranking_overall.csv` | configurations measured on every object, by mean rank across objects |
| `plots/param_<parameter>.png/.pdf` | six metrics vs each parameter that varies, objects side by side. Numeric parameters: median line with interquartile band. Categorical: box plot. Points are individual scans |
| `plots/object_comparison.png/.pdf` | each configuration's metric on the pyramid vs the wave (with Spearman ρ), and rank vs rank: do the best settings generalise? |
| `plots/heatmap_<a>_x_<b>.png/.pdf` | main metric over a pair of parameters, one panel per object, median over the remaining parameters. Made only for pairs varied together (a 2×2 sub-grid exists). A one-factor-at-a-time sweep gets none |
| `plots/tradeoff_<parameter>.png/.pdf` | completeness at 2 mm (whole part) vs RMSE, coloured by each parameter |
| `plots/diagnostics.png/.pdf` | outlier influence (RMSE vs trimmed RMSE), registration quality, spread with outliers labelled |
| `eval_runs/<object>__<session>/` | Evaluate's per-scan output: `metrics.json`, `transform.txt`, `seed_transform.txt`, `error_map.png`, `error_histogram.png`, `run.log` |
| `cropped/<object>/<session>_cropped.ply` | the cropped clouds that were evaluated |
| `cache/` | per-scan cached rows |

The metrics are defined in `../Evaluate/README.md`. All distances are in mm.

## Notes on this data set

- The sweep is a full grid for each object: 3 resolutions × 3 depth units
  (0.1, 0.5 and 1 mm) × 5 presets, one scan each.
- Laser power (150), exposure (8500 µs, auto) and the filter chain are the
  same for every scan.
- `remove_ir_pattern` scans report Visual Preset "Custom" with the laser still
  on. Their point counts follow the preset captured just before them (column
  `preset_previous`), so this preset may not be a distinct mode.
- One wave scan, `20261002_163703`, requested `default` but reports "Custom"
  (`preset_mismatch = True`). It is kept, labelled `default`, and flagged.
- The paths inside the Big_Wave `scan_info.txt` files say `Small_Wave`, because
  the folder was renamed after capture. The object name is taken from the
  folder.

## Code

| file | |
|---|---|
| `run_analysis.py` | CLI |
| `sweep/config.py` | paths, object → STL mapping, crop/seed/check settings |
| `sweep/scan_info.py` | scan discovery and `scan_info.txt` parsing |
| `sweep/prepare.py` | crop (wraps `../Crop`) and seed pose |
| `sweep/batch.py` | per-scan evaluation (wraps `../Evaluate`), caching, failure handling |
| `sweep/analysis.py` | varied-parameter and crossed-pair detection, metric metadata (shared by plots and rankings) |
| `sweep/ranking.py`, `sweep/plots.py` | rankings and figures, reading only `all_scans.csv` |
