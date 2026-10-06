# Filter sweep: RealSense post-processing settings vs scan accuracy

Replays the static scans of the wave surfaces through different RealSense
post-processing filter settings. For each setting it picks the best frame as
rs_marker does, crops it, evaluates it against the reference surface, and
plots how the metrics change.

```
.db3 ─> rs_decode (align depth→colour) ─> cache/ ──┐   once per scan
                                                   │
config ─> software device ─> filter chain ─> best frame ─> point cloud
       ─> ../Crop (auto tag crop) ─> seed pose (../Parameter_Sweep) ─> part trim
       ─> ../Evaluate ─> results/rows/<config>/<scan>.json ─> results.parquet ─> plot_results.py
```

Crop, Evaluate, Parameter_Sweep and rs_marker are used as they are. Nothing in
those folders is modified, and nothing is written to the data disk.

## Setup

```bash
cd Processing/Filter_Sweep
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt

# Frame decoder: links the system librealsense, the same library rs_marker uses.
cmake -S decoder -B decoder/build && cmake --build decoder/build
```

If the decoder is not built, `decoder: auto` falls back to pyrealsense2 and
prints the decoder in every row. Read [Why a C++ decoder](#why-a-c-decoder)
before running it that way. If librealsense is installed outside the linker
path, set `RS_DECODE_LIBRARY_PATH=/path/to/lib`. It applies only to the
decoder.

**On a new machine, run the validation first** (~2 min):

```bash
.venv/bin/python validate.py --scans Big_Wave__863mm__A1,Small_Wave__863mm__B
```

Every line should say OK. It checks two things:

- **Baseline.** With no filters, the sweep must pick rs_marker's own best frame
  and reproduce the `.ply` rs_marker saved, to within 0.001 mm.
- **Filters.** The Python filter path must match `rs_decode --filters` (the
  system librealsense) bit for bit.

## Running

```bash
.venv/bin/python run_sweep.py --smoke                # 1 scan, 8 configs, ~1 min
.venv/bin/python run_sweep.py --stage 1 --dry-run    # job count + runtime estimate only
.venv/bin/python run_sweep.py --stage all            # 1, then 2 (needs 1), then 3 (needs 2)
.venv/bin/python plot_results.py                     # figures + tables -> results/plots/
```

Each run prints the number of configurations, the number of jobs and an
estimated runtime before it starts. The estimate uses the measured median
seconds per job once there are results.

- **Resume.** Every (scan, config) job writes its own row file. Stopping and
  re-running the same command skips finished jobs. `--retry-failed` re-runs the
  failed ones.
- **Useful flags.** `--workers N`, `--heights 641mm,863mm`, `--scans A1,B`,
  `--shapes Big_Wave`, `--out DIR`, `--limit-configs N`.
- **`--keep`.** Keeps each job's folder under `results/jobs/` for inspection:
  `filtered.ply` (the whole selected frame), `trimmed.ply` (what Evaluate
  scored), `seed_transform.txt` and Evaluate's own output.

### Runtime

Measured here (16 threads, 8 workers): about 15 s per job without the temporal
filter and about 18 s with it, which is about 8 s of wall time per job overall.
Evaluate's alignment is most of that. Evaluate's nearest-neighbour queries use
every core (`workers=-1`), so more workers than about half the threads gains
little.

| stage | configs | jobs (60 scans) |
|---|---|---|
| 1 | 51 | 3060 |
| 2 | 133 | 7980 |
| 3 | ≤ 120, half of them HDR (skipped instantly) | ≤ 7200 |

That totals roughly 8–9 h on this machine. Widening the stage-2 grid
multiplies the stage-2 count, so check it with `--dry-run`.

## The data (as found on disk)

`<root>/<Shape>/<height>mm/<session>/<session>_r<rot>/<session>_r<rot>_a<tilt>/` contains:

- `<scan>.db3`: a ROS1 bag (librealsense recording), despite the extension
- `<scan>_detections.csv`
- `<scan>.ply`: rs_marker's best frame, **unfiltered**
- `imu_a<tilt>.csv`
- `scan_info.txt`

Big_Wave and Small_Wave each have six heights (641, 701, 763, 810, 863 and
933 mm). Each height has two sessions, in either time order. The session type
is decided by content, not by time:

| id | session | used |
|---|---|---|
| A1–A4 | the session with only `_a0` folders: four still scans labelled r0, r45, r90, r135 | all four, as repeats of one pose |
| B | the session with non-zero tilts (`_a-25` … `_a25`, 44 recordings) | `_r0_a0` only |

The A rotation labels are wrong: rs_marker labels rotations automatically, and
nothing was rotated. Every row carries `rotation_label` (as on disk),
`session_kind` and `rotation_label_reliable` (False for A). That gives 5 scans
per (shape, height) and 60 in total.

Other recording details:

- All scans: 1280×720 at 30 fps, `high_density` preset, depth units 0.1 mm.
- A scans: rs_marker `-n 90 -b 45:89`.
- B scans: `-n 60 -b 45:59`.

## What happens per (scan, config)

1. **Decode, once per scan** (`cache/<scan>/`). Every recorded depth frame is
   aligned to colour, as rs_marker aligns before filtering. Frames are stored
   zlib-compressed (50–95 MB per scan, about 5 GB for all 60).
   - **Frame map.** The recording has every camera frame (about 245, 30 fps).
     rs_marker only processed about 90 of them (about 9–15 fps live), and its
     frame numbers count those. The map is built in two steps:
     - **Anchor.** The recorded frame whose unfiltered valid-pixel count
       equals `best_valid_pixels` in scan_info.txt.
     - **Every other frame.** Placed by timestamp, then confirmed against the
       raw tag-centre depth logged in `*_detections.csv`. It is the strictly
       increasing path that matches the most logged depths.

     87–89 of 90 frames match every logged depth. Timing alone was off by 1–3
     frames for about half of them.
2. **Filter.** The cached frames are fed through a pyrealsense2
   `software_device`, so the real librealsense filters run on them. A fresh
   chain is built for every config. The order:

   ```
   [align depth→colour (cache)] → decimation → HDR merge → depth→disparity
        → spatial → temporal → disparity→depth → hole filling
   ```

   The disparity transforms are added only when spatial or temporal is on.
   This is Intel's recommended order, with HDR merge after decimation as
   specified. Filters run after the align because rs_marker filters
   colour-aligned depth.
3. **Best frame (rs_marker's rule).** The frame with the most valid
   (non-zero) depth pixels after filtering, among the frames rs_marker itself
   considered (its `-b` range, mapped onto the recording). Ties go to the
   earliest frame.
   - **Warm-up.** The temporal filter is stateful. When temporal or HDR merge
     is on, every recorded frame is fed in order from the start of the
     recording. Selection begins at the first candidate frame, after about
     100–155 frames (3.5–5 s) of history at 30 fps for A scans and about 95
     frames for B scans. That is far longer than the filter's time constant
     (1/alpha frames) or the 8-frame persistency window. `warmup_frames` in the
     config can shorten it.
   - **Stateless chains.** Only the candidate frames are processed.
   - **Columns.** Rows record `best_bag_index`, `selected_rs_frame`,
     `best_valid_fraction`, `n_tied_best`, `frames_fed` and `warmup_frames`.
4. **Point cloud.** It is written in the same frame as rs_marker's
   `export_to_ply` (see [deprojection](#deprojection)).
5. **Crop.** `crop_pointclouds.compute_auto_crop`, run without the preview
   window.
   - **Settings.** Margin 10 px and frame window 15, the same as
     Parameter_Sweep. The tag layout is learned from all sessions of the
     (shape, height).
   - **Staging.** The cropper reads its frame number, detections and
     intrinsics from files next to the `.ply` path. Each job gets a staging
     folder with `imu_a0.csv` (the selected rs_marker frame) and symlinks to
     the original detections and recording.
6. **Seed pose.** `Parameter_Sweep/sweep/prepare.seed_transform`: the table
   plane goes to z = 0 and the part's footprint goes over the reference. For
   decimated clouds only, its DBSCAN radius (6 mm) is multiplied by the
   decimation factor for that call. Without this, decimation ≥ 2 finds no part
   cluster.
7. **Part trim.** Points more than `part_trim_margin_mm` (30 mm) outside the
   reference's footprint are dropped in the seeded frame.
   - **Why.** The tag square is about 500 mm across, around a 100 mm part.
     Given the whole crop, Evaluate's search locked onto a pose tilted about
     24°. Measured on Big_Wave 863 mm: coverage went from 0.57 to 0.99 and the
     Z bias from −10.3 to −1.2 mm. Small_Wave was unchanged.
8. **Evaluate.** `scan_eval.pipeline.evaluate_scan`. Scans are in m and
   references are meshes in mm, treated as height fields. Settings: full
   alignment from the seed, seed 0, no PLY or plot output.
9. **Row.** Each row holds:
   - the config (flat columns plus `config_json`, `config_id`, `config_label`)
   - shape, height, scan id, session and the rotation-label flags
   - the selected frame and the frame-map method
   - crop, seed and trim point counts
   - every Evaluate metric
   - `status`: ok, suspect, failed or skipped, plus `failure_stage` and
     `failure_reason`
   - `elapsed_s`

   Any exception in any step is recorded as a failed row; nothing crashes the
   run. **suspect** means the aligned scan covers less than 85% of the
   reference (Parameter_Sweep's registration check, applied when the table is
   assembled). Plots use `ok` rows only, unless `--include-suspect` is given.

### References

| shape | reference | how it was chosen |
|---|---|---|
| Big_Wave | `../Evaluate/References/EGG_1PK.stl` | as Parameter_Sweep (100 mm period, 30 ± 20 mm) |
| Small_Wave | `../Evaluate/References/Egg carton-smaller v1 (1).stl` | see below |

There is no STL for the small part (`EGG_2PKS.PNG` has no mesh). Its drawing
gives a 50 mm period, the same as the other egg-carton meshes, which differ
only in amplitude. An rs_marker Small_Wave scan was evaluated against every
egg-carton mesh. The "smaller" carton fitted: 97.6% coverage and −1.0 mm
Z bias, against 72% and −6.3 mm for `Egg carton v5`. This is the same part as
`Egg-carton-smaller3cm.ply` and the analytic `surfaces/egg_carton.toml`. Change
it in `sweep_config.yaml` if you know better.

### HDR merge

The HDR merge filter is **skipped**: the recordings have no HDR. Every
scan_info.txt has `Hdr Enabled: 0`, and every frame's `sequence_size`
metadata is 0. HDR configs therefore produce `skipped` rows with that reason,
at no cost. The code path is still in place: frame sequence metadata is
injected into the software sensor, and `rs.hdr_merge` runs after decimation.
It is untested, because no HDR recording exists.

### Decimation

Decimation changes the depth resolution: 1280×720 becomes 640×360 at
magnitude 2 and 160×90 at magnitude 8. librealsense scales the intrinsics to
match. The cropper projects points back with the full-resolution colour
intrinsics. Decimated points still land on the pixel grid, because pixel
(i, j) maps to (M·i, M·j), so the crop region (defined in colour pixels) is
correct.

What changes with decimation:

- The cropper's surface-plane fit may fall back to its camera-view crop. The
  `crop_notes` column says when.
- Point density and therefore completeness drop by M².
- At magnitude 6–8 only 200–400 points remain on the part, so the metrics get
  noisy.

In the validation subset, decimation 8 on Small_Wave was flagged suspect.

## Why a C++ decoder

rs_marker links the system librealsense (2.57.6). The pip wheels behave
differently in three ways that matter here, all measured on these
recordings:

1. **Align.** The wheels (2.57.7 and 2.58.4) mark about 1.5% more pixels valid
   than the system build: about 849k instead of 835k per frame. No recorded
   frame then matches rs_marker's best-frame valid-pixel count, so neither the
   frame map nor the baseline reproduces. `rs_decode` uses the system library
   and matches exactly.
2. **`.db3` files.** pyrealsense2 2.58 opens `.db3` as a ROS2 sqlite bag. These
   are ROS1 bags, so the Python fallback plays them through a `.bag` symlink.
3. <a id="deprojection"></a>**Deprojection** (`rs.pointcloud`). For the colour
   stream's Inverse Brown-Conrady model, 2.57 and 2.58 apply the distortion in
   opposite directions. z is identical; x and y differ by about 0.1 mm near the
   image centre and up to 2.7% (about 25 mm at 0.87 m) in the corners.
   - The sweep reproduces rs_marker's 2.57 behaviour with the cropper's own
     `pixel_rays()` (`deprojection: rs_marker`). The validation shows 0.0002 mm
     against rs_marker's `.ply`.
   - The cropper's intrinsics auto-detect copes with either convention.
   - **If rs_marker is ever rebuilt against librealsense ≥ 2.58**, its clouds
     will change in this way.

The filters do not differ: pyrealsense2 2.58.4 on cached frames is bit-identical
to the system 2.57.6 for decimation, spatial, temporal and hole filling
(`validate.py`).

## Sweep design (`sweep_config.yaml`)

The whole sweep is defined in `sweep_config.yaml`: parameter lists, held
values, the shapes and heights to process, references, workers, and the
metric used for ranking (`rank_metric`, default `acc_rmse_mm`). The baseline
(no filters) runs in every stage. Configs are de-duplicated by `config_id`, a
hash of the full chain, so a config shared between stages runs once.

- **Stage 1, one factor at a time.** Each parameter is swept with its filter
  on, that filter's other parameters at librealsense defaults
  (`filter_defaults`), and every other filter off:
  - decimation: 2–8
  - spatial: magnitude, alpha, delta, holes_fill
  - temporal: alpha, delta, persistency
  - hole filling: modes 0–2
  - HDR
- **Stage 2, spatial × temporal grid.** Each axis also includes "off".
  Decimation and hole filling are held at a value, `off`, or `stage1_best`
  (the default holds them off, so resolution doesn't confound the grid).
- **Stage 3, refinement.** The `top_k` (5) best stage-2 combinations are
  crossed with decimation {1, 2, stage-1 best}, hole filling {off, 0, 1, 2}
  and HDR {off, on}.

**Ranking.** Configs are ranked by the mean, over scans, of each scan's
relative change from its own baseline. This makes shapes and heights
comparable. A config that is ok on fewer than 80% of its scans ranks last.

## Plots and tables (`plot_results.py` → `results/plots/`)

| file | |
|---|---|
| `one_factor_overview` | stage 1: % change vs baseline of RMSE, error std, F-score @2 mm and coverage, one column per filter parameter; Small vs Big wave, mean ± std over scans |
| `one_factor_<param>` | stage 1: six metrics vs one parameter, shapes side by side; thin line per height, thick = mean ± std over all scans of that shape, dashed = filter off |
| `heatmaps/heatmap_<metric>__<all\|shape>__<temporal delta, persistence>` | stage 2: spatial alpha × temporal alpha; rows = spatial magnitude, columns = spatial delta; cell = mean % change vs baseline; blue = better, red = worse, whichever direction the metric runs; the "off" row and column are the single-filter results |
| `baseline_vs_best` | per shape and height: baseline, the best config overall, and the best config for that shape & height (mean ± std over the 5 scans) |
| `summary_best_per_metric.csv/.md` | best config for each key metric, overall and per shape, with baseline and % change |
| `ranking_<stage>.csv` | every config of a stage ranked by `rank_metric` |

The metric names are Evaluate's results columns, all distances in mm (see
`../Evaluate/README.md`). `|Signed bias|` is ranked by size.

## Validation run (this machine, 2026-10-06)

The subset was 863 mm, scans A1 and B, both shapes (4 scans). It covered:

- stage 1 (51 configs)
- a reduced stage 2 (2 × 2 spatial × 2 × 2 temporal)
- stage 3 built from that

Output is in `results_validation/`: 560 rows, no failures except the
intended HDR skips, and 5 rows suspect (decimation 4 and 8, and strong
spatial smoothing).

With 2 scans per shape, most differences are within the scan-to-scan spread.
These are not conclusions, only a check that the pipeline works:

- Spatial smoothing with lower alpha or more iterations increases RMSE
  (alpha 0.25: +20–35%).
- Hole filling (mode 1) and decimation 2 reduce the trimmed RMSE by about 5%.
- Temporal smoothing changes RMSE by a few % either way.

## Code

| file | |
|---|---|
| `run_sweep.py` | CLI: stages, smoke test, dry run, resume |
| `plot_results.py` | figures and tables from the results table only |
| `validate.py` | baseline reproduction + filter equivalence checks |
| `sweep_config.yaml` | the sweep definition |
| `decoder/rs_decode.cpp` | replay + align with the system librealsense, optional `--filters` |
| `filter_sweep/discover.py` | sessions A/B, scan ids |
| `filter_sweep/frames.py` | decoding, cache, rs_marker ↔ recording frame map |
| `filter_sweep/filters.py` | software device, filter chain, best frame, point cloud |
| `filter_sweep/evaluate.py` | crop → seed → trim → Evaluate (wrappers only) |
| `filter_sweep/stages.py` | stage → configs |
| `filter_sweep/runner.py` | jobs, multiprocessing, failure rows |
| `filter_sweep/results.py` | results table, registration check, ranking |
