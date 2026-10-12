# 11 — Filter_Sweep: RealSense post-processing filters vs scan accuracy

**Summary.** `Processing/Filter_Sweep` is an offline experiment harness. It replays the static RealSense D456 recordings of the two calibration parts (the "Big_Wave" and "Small_Wave" egg-carton surfaces) through many configurations of the librealsense post-processing filters: decimation, HDR merge, depth↔disparity transform, spatial, temporal and hole filling. For each (scan, configuration) it picks the "best" frame with the same rule as the capture tool `rs_marker`, turns that frame into a point cloud in `rs_marker`'s PLY frame, and crops it with `../Crop`. It then seeds the pose with `../Parameter_Sweep`, trims to the part, scores the result against the reference mesh with `../Evaluate`, and writes one results row. The filters are **not reimplemented**. They are the real librealsense filters, run through a `pyrealsense2.software_device` on cached frames that were decoded once with a small C++ tool (`decoder/rs_decode`) linked against the same system librealsense (2.57.6) as `rs_marker`. Configurations run in three stages: one-factor-at-a-time, then a spatial × temporal grid, then refinement. They are ranked by the mean per-scan relative change of a chosen Evaluate metric (default `acc_rmse_mm`) against that scan's own unfiltered baseline. Everything below comes from reading the source, README, config and git history. No project code was run. The only data inspected were the existing `results_validation/` CSV and summary table.

### Files covered

| Path (repo-relative) | Lines | Role |
|---|---|---|
| `Processing/Filter_Sweep/README.md` | 348 | Design document: data layout, pipeline, rationale, validation results |
| `Processing/Filter_Sweep/requirements.txt` | 12 | Python deps (pyrealsense2, open3d, numpy, scipy, sympy, matplotlib, pandas, pyarrow, pyyaml, tqdm) |
| `Processing/Filter_Sweep/sweep_config.yaml` | 155 | The whole sweep definition (data root, references, stages, defaults) |
| `Processing/Filter_Sweep/run_sweep.py` | 73 | CLI entry point: stages, smoke test, dry run, resume |
| `Processing/Filter_Sweep/validate.py` | 127 | Baseline-reproduction and filter-equivalence checks |
| `Processing/Filter_Sweep/plot_results.py` | 393 | Figures and summary tables from the results table |
| `Processing/Filter_Sweep/filter_sweep/config.py` | 148 | Settings loader, canonical config form, hashing, labels |
| `Processing/Filter_Sweep/filter_sweep/discover.py` | 129 | Finds the 5 scans per (shape, height) on the data disk |
| `Processing/Filter_Sweep/filter_sweep/frames.py` | 350 | Decoding, zlib frame cache, rs_marker↔recording frame map |
| `Processing/Filter_Sweep/filter_sweep/filters.py` | 205 | Software device, filter chain, best-frame selection, deprojection |
| `Processing/Filter_Sweep/filter_sweep/evaluate.py` | 150 | Crop → seed → part trim → Evaluate wrappers |
| `Processing/Filter_Sweep/filter_sweep/stages.py` | 129 | Stage definitions → list of filter configurations |
| `Processing/Filter_Sweep/filter_sweep/runner.py` | 263 | Job building, multiprocessing pool, per-job row files |
| `Processing/Filter_Sweep/filter_sweep/results.py` | 122 | Results assembly, registration check, relative metrics, ranking |
| `Processing/Filter_Sweep/decoder/CMakeLists.txt` | 9 | Build for `rs_decode` (links system `realsense2`) |
| `Processing/Filter_Sweep/decoder/rs_decode.cpp` | 177 | Replays `.db3`, aligns depth→colour, optional filter chain, raw uint16 to stdout |

All of these files were added in a single commit, `00b2c4e` (6 Oct 2026, "Added python code to evaluate performance across parameters, and python code to evaluate performance across different filters"). No later commit touches them. Untracked or ignored artefacts present on disk: `.venv/`, `cache/` (≈270 MB, 4 scans), `decoder/build/rs_decode` (an ELF x86-64 executable with debug info), `results_validation/` (560 rows, plots) and `filter_sweep/__pycache__/`.

### Contents
1. Purpose
2. Inputs and outputs
3. Step-by-step walkthrough
4. Algorithms and methods (the filters, with maths)
5. Why this approach
6. Parameters and tuning
7. Assumptions, limitations, failure modes
8. Dependencies
9. Observations

---

## 1. Purpose

The capture tool `rs_marker` records each static scan as a librealsense recording (`.db3`). It saves the "best" frame, the one with the most valid depth pixels, as an **unfiltered** PLY (README.md:94). The thesis question this component addresses is which RealSense post-processing filter settings, if any, improve the geometric accuracy of those single-frame scans against a known reference surface, and by how much (README.md:1-6). Filter_Sweep answers this offline. Because the original recordings are replayed, every filter configuration sees exactly the same raw data. Accuracy is measured with the project's own evaluation tool (`../Evaluate`), using the same crop and seed logic as the earlier `Parameter_Sweep` study (README.md:16-17).

Place in the system: Capture (`rs_marker` → `.db3`, `_detections.csv`, `.ply`, `scan_info.txt`, `imu_a*.csv`) → **Filter_Sweep** (consumes these plus `../Crop`, `../Parameter_Sweep`, `../Evaluate`) → results table and plots for the thesis. Nothing it produces is consumed by another component.

## 2. Inputs and outputs

### 2.1 Input data (read-only, external disk)

`data_root: /run/media/joab/Seagate/roughness/Joab_Characterisation/3D_Error` (sweep_config.yaml:18). The layout is documented at `filter_sweep/discover.py:3-17` and README.md:90-96.

```
<root>/<Shape>/<height>mm/<session>/<session>_r<rot>/<session>_r<rot>_a<tilt>/
    <scan>.db3              ROS1 bag (librealsense recording), despite the extension
    <scan>_detections.csv   AprilTag detections per rs_marker frame (frame, timestamp_ms, cx, cy, depth_m, ...)
    <scan>.ply              rs_marker's best frame, unfiltered, metres
    imu_a<tilt>.csv         frame index (+ IMU)
    scan_info.txt           "key: value" lines: frames_captured, best_frame, best_valid_pixels "N/M",
                            best_range "lo:hi", scan_duration_s, Hdr Enabled, ...
```

| Item | Detail |
|---|---|
| Shapes | `Small_Wave`, `Big_Wave` (sweep_config.yaml:21). The pyramids were not scanned (yaml:20) |
| Heights | 641, 701, 763, 810, 863, 933 mm (README.md:98-99) |
| Scans per (shape, height) | 5: A1–A4 from the "still" session (four repeats of one pose; the rotation labels are wrong), and B = `_r0_a0` of the tilt session (README.md:102-110; discover.py:13-17) |
| Stream | 1280×720 at 30 fps, `high_density` preset, depth units 0.1 mm (README.md:114) |
| References | Small_Wave → `../Evaluate/References/Egg carton-smaller v1 (1).stl`; Big_Wave → `../Evaluate/References/EGG_1PK.stl`, both meshes in mm treated as height fields (sweep_config.yaml:31-33) |

### 2.2 CLI

`run_sweep.py` (run_sweep.py:28-44):

| Flag | Meaning |
|---|---|
| `--stage {1,2,3,all}` / `--smoke` | Mutually exclusive and required |
| `--config` | YAML (default `sweep_config.yaml`) |
| `--workers N` | Process pool size (default `workers: 12`) |
| `--dry-run` | Print the job count and runtime estimate only |
| `--retry-failed` | Re-run rows with status `failed` |
| `--keep` | Keep `results/jobs/<cfg>/<scan>/` (filtered.ply, trimmed.ply, seed_transform.txt, Evaluate output) |
| `--limit-configs N`, `--shapes`, `--heights`, `--scans`, `--out` | Subsetting and overrides |

`validate.py --scans KEY[,KEY] [--no-filters] [--config]` (validate.py:103-123). `plot_results.py [--results DIR] [--include-suspect] [--config]` (plot_results.py:362-389).

`rs_decode <recording.db3|.bag> <meta.jsonl> [--max-frames N] [--filters SPEC]` (rs_decode.cpp:10-22). SPEC is `dec=M,spatial=MAG:ALPHA:DELTA:HOLES,temporal=ALPHA:DELTA:PERSIST,hole=MODE`.

### 2.3 Intermediate and output files

| File | Format | Producer |
|---|---|---|
| `cache/<Shape>__<height>__<id>/depth.zbin` | Concatenated zlib (level 3) blobs, each one aligned depth frame, uint16 little-endian, H×W row-major | `frames.ensure_cache` (frames.py:300-310) |
| `cache/<key>/meta.json` | `source` stamp (bag path, size, mtime, decoder, warmup, cache_version=3), `scan`, `stream` (intrinsics, model, coeffs, depth_units, stereo_baseline_mm, hdr_enabled, decoder), per-frame list (`frame_number`, `timestamp_ms`, `valid`, `sequence_*`), `map` (frame map), `stored`, `offsets`, `hdr_available` | frames.py:312-322 |
| `results/rows/<config_id>/<scan_key>.json` | One flat JSON row per job (config columns, scan meta, frame selection, crop/seed/trim counts, every Evaluate metric, status) | runner.py:136-142 |
| `results/results.parquet`, `results/results.csv` | Assembled table, with the `<rank_metric>_rel` column and registration status | results.py:70-84 |
| `results/stages/<stage>.csv`, `<stage>_scans.csv` | Configs and scans of each stage | runner.py:159-165 |
| `results/plots/...` | PNG+PDF figures, `ranking_<stage>.csv`, `summary_best_per_metric.csv/.md` | plot_results.py |

**Units and frames.** Cached depth is in raw depth units (`depth_units`, 1e-4 m). The point cloud is Nx3 float32 in metres, in `rs_marker`'s `export_to_ply` frame: $(x, -y, -z)$ of the colour-camera optical frame, with zero-depth pixels dropped (filters.py:133-160). Evaluate takes the scan in m and the reference in mm (sweep_config.yaml:69-71). All reported error metrics are in mm (README.md:310-311).

## 3. Step-by-step walkthrough

### 3.1 Start-up (`run_sweep.py`)
1. Parse the arguments and load the YAML into `Settings` (run_sweep.py:45-53; `config.load_settings` config.py:58-69). Relative paths resolve against the YAML's folder. Any other YAML key is reachable as an attribute through `Settings.__getattr__` (config.py:51-55).
2. Set `OMP_NUM_THREADS`, `OPENBLAS_NUM_THREADS` and `MKL_NUM_THREADS` **before** numpy/Open3D load: `threads_per_worker` (1) when parallel, otherwise half the CPUs (run_sweep.py:56-58).
3. Call `runner.run_stage` for the smoke set or for each requested stage. With `all`, stages 1, 2 and 3 run in order (run_sweep.py:62-69).

### 3.2 `runner.run_stage` (runner.py:168-263)
1. **Discover scans** (`discover.discover`, discover.py:77-120). Height folders must match `\d+mm`. A session is classed "B" if any scan folder has non-zero tilt, otherwise "A" (discover.py:95). For A, the four `_a0` scans sorted by rotation label become A1–A4. For B, only `_r0_a0` is used (discover.py:100-109). Scans missing the `.db3`, detections or `scan_info.txt` are skipped with a warning (discover.py:115-118).
2. **Assemble earlier results** if `rows/` exists. Stages 2 and 3 need them (runner.py:184-186).
3. **Build configs** with `stages.build_stage` (§3.6), optionally truncate them, and write the stage list (runner.py:187-190).
4. **Resume.** Load every row. Pairs (config_id, scan_key) with the current `PIPELINE_VERSION` (=2, config.py:23) and status ok/suspect/skipped(/failed) are treated as done (runner.py:192-199).
5. **Estimate the runtime** from the median `elapsed_s` of ok rows (≥5), otherwise from `estimate_s_per_job: 17` (runner.py:200-209). A dry run stops here.
6. **Prepare the caches.** `frames.ensure_cache` runs per scan (§3.3), and `crop_pointclouds.learn_tag_layout(height_dir)` runs per height folder (runner.py:216-223; evaluate.py:64-67).
7. **Dispatch the jobs** sorted by scan, so each worker reuses its open cache and software sensor (`_scan_state`, runner.py:55-69). The pool is a `multiprocessing` "spawn" pool with `imap_unordered`, chunksize 1, or a plain `map` for one worker (runner.py:235-243). The table is re-assembled every 200 jobs and at the end (runner.py:250-262). Ctrl-C terminates the pool, and finished rows are kept.

### 3.3 Decode and cache (`frames.py`)
1. `decoder_kind`: "cpp" if `decoder/build/rs_decode` exists (for `auto`), else "python" (frames.py:113-118).
2. **C++ path** `_decode_cpp` (frames.py:43-61) runs `rs_decode bag meta.jsonl`. It reads the raw uint16 stream from stdout and the JSON lines (line 1 `stream`, then one per frame), and checks that the sizes match. `RS_DECODE_LIBRARY_PATH` becomes `LD_LIBRARY_PATH` for the child only.
3. Inside **`rs_decode`** (rs_decode.cpp:96-177): `cfg.enable_device_from_file(bag, false)`, then enable the depth and colour streams and start the pipeline. `set_real_time(false)` makes playback deliver every recorded frame and drop none (rs_decode.cpp:121). For each frameset it runs `rs2::align(RS2_STREAM_COLOR).process(fs)`, then the optional chain, writes W·H uint16 to stdout, counts the valid (non-zero) pixels, and writes the metadata line (rs_decode.cpp:134-168). The header records the *aligned* intrinsics, the distortion model string, `get_depth_scale()`, `RS2_OPTION_STEREO_BASELINE`, `RS2_OPTION_HDR_ENABLED` and the library version (rs_decode.cpp:140-152).
4. **Python fallback** `_decode_python` (frames.py:64-110) does the same with pyrealsense2. librealsense ≥2.58 opens `.db3` as a ROS2 sqlite bag, so it plays the file through a `recording.bag` symlink (frames.py:66-69).
5. **Frame map** `frame_map` (frames.py:144-191). See §4.2.
6. **Store** frames `first..last` with zlib, where `last` is the last candidate and `first` is 0 or `first candidate − warmup_frames` (frames.py:300-310). Writes are atomic (`.tmp` then `os.replace`). `hdr_available` is true only if `hdr_enabled ≥ 1` and every frame's `sequence_size ≥ 2` (frames.py:312-319).
7. The cache is reused when `meta["source"]` equals the current stamp, i.e. same bag size/mtime, decoder, warmup and cache version (frames.py:274-294).

### 3.4 One job (`runner.run_job`, runner.py:72-133)
1. Build the row skeleton: scan meta, flat config columns, `config_id`, label, JSON, `status="failed"` (runner.py:82-85).
2. If HDR is requested and the scan has no HDR, the job ends with status `skipped` at no cost (runner.py:97-99).
3. Stdout/stderr go to `job.log`. **Filter**: `filters.run_config` (§3.5). `selected_rs_frame = rs_frame_for(meta, best_bag_index)` (runner.py:101-106; frames.py:265-269).
4. **Crop/seed/trim/evaluate**: `stage_scan_folder` creates `jobs/.../scan/<scan_dir>/` with `imu_a0.csv` holding the selected rs_marker frame number and symlinks to the original detections and `.db3` (evaluate.py:70-82). Then `crop_seed_trim_evaluate` runs (§3.7).
5. Merge Evaluate's result dict and pull `alignment.chosen` from `eval/metrics.json` (runner.py:118-126). Status becomes `ok`.
6. Any exception is recorded as `failed` with `failure_stage`/`failure_reason` and the last 2000 chars of the traceback. Exceptions never abort the run (runner.py:128-132).
7. `_finish` adds `elapsed_s`, writes the row atomically, and deletes the job folder unless `--keep` (runner.py:136-142).

### 3.5 Filtering and best frame (`filters.py`)
1. `SoftwareDepth(stream)` (filters.py:41-64) creates an `rs.software_device` with one "Stereo Module" sensor and a Z16 depth `video_stream`. It carries the aligned stream's intrinsics and distortion model, read-only options `depth_units` and (if >0) `stereo_baseline`, and a `frame_queue(2, keep_frames=True)`.
2. `SoftwareDepth.frame` (filters.py:66-78) injects one numpy frame as a `software_video_frame` with stride W·2, hardware-clock timestamp and frame number. For HDR it also sets the `sequence_id`/`sequence_size` metadata. It waits up to 2 s for the frame to come back from the queue.
3. `Chain(cfg)` (filters.py:88-119) builds fresh filter objects in this order: `decimation_filter(M)` → `hdr_merge()` → `disparity_transform(True)` → `spatial_filter` (options `filter_magnitude`, `filter_smooth_alpha`, `filter_smooth_delta`, `holes_fill`) → `temporal_filter` (`filter_smooth_alpha`, `filter_smooth_delta`, `holes_fill` = persistence) → `disparity_transform(False)` → `hole_filling_filter(mode)`. The two disparity transforms are inserted only if spatial or temporal is on (filters.py:97-116). `stateful = temporal or hdr_merge` (filters.py:119). `process()` unwraps a frameset if hdr_merge returns one (filters.py:125-130).
4. `run_config` (filters.py:163-205):
   * Candidates are the recording indices of rs_marker's `-b` range, from the frame map.
   * Stateful chains feed **every stored frame from the start of the recording** up to the last candidate, so the temporal filter has a history. Stateless chains feed only the candidates (filters.py:170-171).
   * For each candidate output it counts the non-zero pixels and keeps the strict maximum, so ties go to the earliest frame (filters.py:181-188). The numpy source array is kept alive with the best frame (`_keep_alive`), because the software frame references that buffer.
   * `point_cloud(best_frame, deprojection)` (§4.3).
   * Returns the selection statistics (`best_bag_index`, `best_valid_fraction`, `n_tied_best`, `frames_fed`, `warmup_frames`, `chain_order`, output size).

### 3.6 Stage construction (`stages.py`)
* `build_stage` (stages.py:61-119) always starts with the baseline `{}`.
  * **one_factor**: for each dotted parameter and value, `{filter: {param: v}}`. The other parameters of that filter come from `filter_defaults`, and every other filter is off (stages.py:68-71).
  * **grid**: `hold` values (a number, "off" or "stage1_best", resolved via `_stage1_best`, stages.py:42-54). The Cartesian product of the spatial parameters and of the temporal parameters, each list plus `None` (off) when `include_off`, then spatial × temporal (stages.py:73-88).
  * **refine**: ranks the stage-2 configs, takes the `top_k` (5), and crosses each with the product of `vary` lists. Decimation 1 or off becomes "off" (stages.py:90-108).
* Every config goes through `config.normalise` (config.py:78-96): each filter key is present, off is `None`, and on is a dict of all parameters with defaults filled and types coerced. Decimation ≤1 and HDR disabled normalise to `None`. `config_id = sha1(json(sort_keys))[:12]` (config.py:99-100) de-duplicates the configs (stages.py:112-119).
* Counts: stage 1 has 6+5+6+6+6+7+6+9+3+1 = 55 one-factor values plus the baseline. Five of these coincide with the defaults (spatial m2/a0.5/d20/h0 → three duplicates; temporal a0.4/d20/p3 → two duplicates), which gives **51** configs as the README states (README.md:81). Stage 2 has (3·3·2·1+1)·(3·1·2+1) = 19·7 = **133**, as stated.

### 3.7 Crop → seed → trim → Evaluate (`filter_sweep/evaluate.py:106-150`)
1. **Crop**: `crop_pointclouds.compute_auto_crop(pcd, ply_path, ply_path.parent, layout, args)` with `frame_window=15` and `margin=10 px`. The cropper finds the frame number, detections and colour intrinsics next to the staged path. The cloud is passed in memory (evaluate.py:111-118).
2. **Seed**: `Parameter_Sweep.sweep.prepare.seed_transform(cropped, ref.bounds())` puts the table plane at z=0 and the part footprint over the reference. For decimated clouds the DBSCAN radius `SEED_CLUSTER_EPS_M` (6 mm) is temporarily multiplied by the decimation factor (evaluate.py:85-103, 120-125).
3. **Part trim**: transform with $T_0$ and keep the points within the reference's xy bounds ± `part_trim_margin_mm` (30 mm). Fewer than 100 points means failure (evaluate.py:127-134). It writes `trimmed.ply` (via the cropper's PLY writer) and `seed_transform.txt`.
4. **Evaluate**: `EvalParams()` with the YAML overrides (`units m`, `reference_units mm`, `seed 0`, `align full`, no PLY/plots). Then `scan_eval.pipeline.evaluate_scan(ref, trimmed_path, params, job_dir, "eval", T0)` (evaluate.py:141-150). The reference is loaded once per process as a height field and wrapped in `UnitScaled` (evaluate.py:54-61).

### 3.8 Assembly and ranking (`results.py`)
* `check_registration` (results.py:44-57): rows that are ok/suspect with `covered_fraction < min_covered_fraction` (0.85) become `suspect` with stage "registration". The check runs at assembly time, so changing the threshold does not invalidate cached rows.
* `add_relative` (results.py:60-67): `<metric>_rel = metric / baseline_same_scan − 1`.
* `rank_configs` (results.py:98-122): §4.6.

### 3.9 Plots (`plot_results.py`)
`load` keeps only `ok` rows, or ok+suspect with `--include-suspect`, and adds `abs_signed_bias_mm` (plot_results.py:92-105). It produces:
* Stage-1 per-parameter panels: a thin line per height, mean ± std per shape, and a dashed baseline (plot_results.py:150-187).
* An overview of % change vs baseline (plot_results.py:189-214).
* Stage-2 heatmaps of spatial α × temporal α, faceted by spatial magnitude × delta, one figure per (temporal delta, persistence). Cells show the mean % change, with a diverging map flipped so that blue always means "better" (plot_results.py:219-282).
* `baseline_vs_best` per height (plot_results.py:287-327).
* Ranking CSVs and the best-per-metric summary (plot_results.py:330-359).

### 3.10 Validation (`validate.py`)
* `check_baseline` (validate.py:39-54): runs the no-filter config. It requires the selected frame to equal the anchor (rs_marker's best frame), the point count to equal the count in rs_marker's `.ply`, and the max NN distance to be < 1e-5 m.
* `check_filters` (validate.py:57-100): for 3 configs (spatial defaults; temporal α 0.2 p3; dec3 + spatial m3/α0.4/h2 + temporal + hole-fill 1), runs `rs_decode --filters <chain_spec>` (system librealsense) and the Python software-device path frame by frame. It requires every candidate frame to be bit-identical.

## 4. Algorithms and methods

### 4.1 Depth→colour alignment (librealsense `rs2::align`)
Not implemented in this repo. It is called once per frame in `rs_decode` (rs_decode.cpp:130-135) or `_decode_python` (frames.py:80-89). Conceptually, each depth pixel $(u,v)$ with depth $Z$ is deprojected through the depth intrinsics, $\mathbf p_d = Z\,K_d^{-1}[u, v, 1]^T$ (with the depth lens model), transformed with the depth→colour extrinsics $\mathbf p_c = R\,\mathbf p_d + \mathbf t$, and projected with the colour intrinsics and distortion into the colour image. The output is a depth image on the colour pixel grid that carries the colour intrinsics. Alignment comes first because `rs_marker` aligns before filtering (README.md:144-146; filters.py:4-5). The README reports that the pip wheels' SSE align marks ~1.5% more pixels valid than the system (generic) build: about 849k vs 835k per frame (README.md:250-254; rs_decode.cpp:5-8). That is the reason for the C++ decoder.

### 4.2 Recording ↔ rs_marker frame map (frames.py:144-249)
`rs_marker` processed only about 90 of the ~245 recorded frames (≈9–15 fps live vs 30 fps recorded), and its frame numbers count processed frames (frames.py:10-17). The map has two parts.

1. **Anchor.** Find the recorded frames whose unfiltered valid-pixel count equals `best_valid_pixels` from `scan_info.txt`. If there are several, take the one nearest in time to `end_guess = t_last − (scan_duration − t_rs[best])` (frames.py:158-167). Otherwise fall back to timing only.
2. **Timing prediction.** rs_marker timestamps come from `_detections.csv`. They are linearly interpolated over frames without detections and extrapolated with the mean period (frames.py:123-141). With $\Delta = t_{bag}[a] - t_{rs}[k^*]$, each rs frame $k$ is predicted as $p_k = \arg\min_j |t_{bag}[j] - (t_{rs}[k]+\Delta)|$.
3. **Depth-confirmed dynamic programming** (`_match_by_depth`). For every rs frame $k$, the candidates are $j\in[p_k-5, p_k+5]$ (the anchor is fixed). The score is
   $$s_k(j) = \#\{\text{tags}: |D_j(c_x,c_y)\cdot u - d_{logged}| \le 0.5\,\text{mm}\} - 10^{-3}|j-p_k| \;(+1000 \text{ at the anchor}),$$
   where $D_j$ is the raw depth image, $u$ the depth unit and $d_{logged}$ the tag-centre depth logged by rs_marker to 1 mm (frames.py:207-220). A **Viterbi** recursion over strictly increasing $j$ ($j_{k-1}<j_k$) maximises $\sum_k s_k(j_k)$ (frames.py:224-247). If no increasing path exists, it falls back to the timing prediction. The README reports 87–89 of 90 frames matching every logged depth, against timing alone being off by 1–3 frames for about half of them (README.md:132-133).
4. **Candidates** = the sorted unique bag indices for rs frames in `best_range` (`-b lo:hi`).

### 4.3 Deprojection to a point cloud (filters.py:133-160)
For pixel $(u,v)$ with depth $z = D(u,v)\cdot\text{units}$:
$$x_n=\frac{u-c_x}{f_x},\quad y_n=\frac{v-c_y}{f_y},\quad (x', y') = \text{undistort}(x_n,y_n),\quad \mathbf P = (x' z,\; -y' z,\; -z).$$
The default `deprojection: rs_marker` uses the cropper's `pixel_rays` with `direction="forward"`, which numerically inverts the colour stream's Inverse Brown–Conrady polynomial (`_undistort`, Crop/crop_pointclouds.py:325-334). This reproduces librealsense 2.57 behaviour. The README states that 2.58 applies the polynomial the other way round: z is identical, and x/y differ by up to 2.7% (~25 mm at 0.87 m) in the corners (README.md:257-266; filters.py:137-146). The sign flip $(x,-y,-z)$ matches `rs_marker`'s `export_to_ply`. The alternative `pyrealsense2` path uses `rs.pointcloud().calculate` and a `min_distance` test of 1e-6 (filters.py:147-150).

### 4.4 The post-processing filters swept

All of these are librealsense's own C++ implementations, reached through `pyrealsense2` (filters.py:93-118) or directly through `rs2::` classes in `rs_decode` (rs_decode.cpp:37-89). **None is reimplemented in Python.** The descriptions below summarise Intel's documented algorithms ("Depth Post-Processing for Intel RealSense Depth Camera D400 Series", Intel white paper, and librealsense source). They are not taken from this repo's code, which treats the filters as black boxes. The option names and ranges are documented in sweep_config.yaml:4-16.

**(a) Decimation** (`rs.decimation_filter(M)`; swept M ∈ {2,3,4,5,6,8}, sweep_config.yaml:102). The image is subsampled by an integer factor $M$ by replacing each $M\times M$ block with one value computed from its **valid** (non-zero) pixels. Per Intel's documentation, the kernel is the median for small kernels (2×2, 3×3) and the mean for larger ones:
$$D'(i,j)=\operatorname{median/mean}\{D(Mi+a, Mj+b) : 0\le a,b<M,\ D>0\}.$$
The output resolution is $\lfloor W/M\rfloor\times\lfloor H/M\rfloor$ (1280×720 → 640×360 at M=2), and librealsense scales the intrinsics to match. Effects: point density drops by $M^2$ (README.md:236-241), random noise is averaged down, small holes are filled implicitly, and lateral resolution is lost. Because pixel $(i,j)$ maps to colour pixel $(Mi, Mj)$, crop regions defined in colour pixels remain valid (README.md:227-232).

**(b) Threshold filter.** **Not used.** No threshold filter appears in `Chain`, `rs_decode` or the config (searched: no occurrence in Filter_Sweep). `rs_marker` declares one, but its use is commented out (Capture/programs/rs_marker/main.cpp:52, 61-62, 78). Out-of-range points are instead removed geometrically by the AprilTag crop and the part trim.

**(c) Depth ↔ disparity transform** (`rs.disparity_transform(True/False)`; not swept, inserted automatically). Stereo depth error grows roughly as $\sigma_Z \propto Z^2/(fB)\,\sigma_d$, so a fixed smoothing threshold in depth units means different things at different ranges. In disparity space the error is roughly range-independent:
$$d = \frac{f_x\,B}{Z},$$
with focal length $f_x$ (px) and baseline $B$. librealsense stores disparity as float32 with an internal scale. The code inserts the transform pair only around spatial and temporal, following Intel's recommendation [EXPLICIT: filters.py:13-14; README.md:143-146]. The software sensor registers `stereo_baseline` from the recording so the transform has $B$ (filters.py:60-61).

**(d) Spatial edge-preserving filter** (`rs.spatial_filter`). The librealsense implementation is a 1-D recursive **domain-transform** filter in the spirit of Gastal & Oliveira (2011, "Domain Transform for Edge-Aware Image and Video Processing", ACM TOG). It makes alternating horizontal and vertical passes, each a causal and an anti-causal exponential moving average that is switched off across edges. For a scan line of disparities $d_i$:
$$S_i=\begin{cases}\alpha\, d_i + (1-\alpha)\, S_{i-1}, & |d_i - S_{i-1}| < \delta \text{ and both valid}\\ d_i, & \text{otherwise (edge, or invalid)}\end{cases}$$
applied left→right, then right→left, then the same vertically. The parameters:
* `filter_magnitude` (iterations, 1–5; swept 1–5) is the number of repetitions of the H+V passes.
* `filter_smooth_alpha` α (0.25–1; swept 0.25…1.0) is the EMA weight. α=1 means no smoothing, and smaller α gives a longer effective support $\approx 1/\alpha$ px.
* `filter_smooth_delta` δ (1–50 disparity units; swept 1…50) is the step threshold above which smoothing stops, which is what preserves edges.
* `holes_fill` (0–5: off, 2, 4, 8, 16 px, unlimited; swept 0–5) is an in-filter hole-filling radius.

The comment "rows = spatial magnitude… spatial alpha" in the heatmaps corresponds to these (plot_results.py:57-66). The mapping in this codebase is: Python `set_option(rs.option.filter_magnitude/…)` (filters.py:102-106); C++ `rs2::spatial_filter(alpha, delta, magnitude, hole_fill)`, with argument order `v[1], v[2], v[0], v[3]` (rs_decode.cpp:79).

**(e) Temporal filter** (`rs.temporal_filter`, **stateful**). This is a per-pixel exponential moving average over time, again in disparity, with an edge (motion) threshold and a "persistence" rule for missing data:
$$S_t(p)=\begin{cases}\alpha\, d_t(p) + (1-\alpha)\, S_{t-1}(p), & |d_t(p) - S_{t-1}(p)| < \delta\\ d_t(p), & \text{otherwise}\end{cases}$$
When $d_t(p)$ is invalid, the last valid $S$ is output if the pixel's validity history over the last 8 frames matches the persistency pattern selected by `holes_fill` (0 off; 1 "valid in 8/8"; 2 "valid in 2/last 3"; 3 "2/last 4"; 4 "2/8"; 5 "1/last 2"; 6 "1/last 5"; 7 "1/last 8"; 8 always), as listed at sweep_config.yaml:11-13. The swept values are α ∈ {0.05…1.0}, δ ∈ {1…100} and persistence 0–8. Because the filter has state, every recorded frame is fed in order from the start of the recording. The README notes that this warm-up (~95–155 frames) far exceeds the filter time constant ($\approx 1/\alpha$ frames) and the 8-frame persistency window (README.md:151-157). A fresh `Chain` per config means each config starts with empty history (filters.py:89).

**(f) Hole filling** (`rs.hole_filling_filter(mode)`; swept modes 0, 1, 2). Each invalid pixel is filled from its neighbours. Mode 0 ("fill_from_left") uses the nearest valid pixel to the left. Mode 1 ("farthest_from_around") uses the valid neighbour (of 4) with the **largest** depth, i.e. the background. Mode 2 ("nearest_from_around") uses the neighbour with the smallest depth. It runs last, in the depth domain (filters.py:117-118). It increases valid-pixel counts, so it directly affects which frame wins the best-frame rule.

**(g) HDR merge** (`rs.hdr_merge()`). This merges two alternating-exposure frames identified by the `sequence_id`/`sequence_size` metadata. In this study the chain places it after decimation (filters.py:6-7, "as specified for this study"). It is **never exercised**: every recording has `Hdr Enabled: 0` and `sequence_size` 0, so HDR configs are written as `skipped` (README.md:216-223; runner.py:97-99). In the validation set, 160 of 560 rows are these skips (counted from `results_validation/results.csv`).

### 4.5 Best-frame rule (filters.py:174-188)
$$j^\* = \arg\max_{j\in\mathcal C} \;\#\{p: D^{filt}_j(p) > 0\},$$
with ties going to the earliest $j$, as in rs_marker (README.md:147-150). This couples the filters to frame selection. Hole filling and spatial `holes_fill` raise the valid counts and so change which frame is chosen. The selection criterion is completeness, not accuracy.

### 4.6 Ranking (results.py:98-122)
For config $c$ and metric $m$, over the scans $s$ with status `ok`:
$$r_{c,s}=\frac{m_{c,s}}{m_{\text{baseline},s}}-1,\qquad \text{score}_c = \sigma_m\;\overline{r_{c,\cdot}},$$
where $\sigma_m=-1$ for higher-is-better metrics. These are identified by name substrings: `completeness`, `precision`, `fscore`, `covered_fraction`, `density`, `points_`, `best_valid_fraction` (results.py:15-23). Configs whose ok fraction is below 0.8 get score $+\infty$. The lowest score wins. Normalising per scan makes shapes and heights comparable [EXPLICIT: README.md:295-297; results.py:61-62].

### 4.7 Experimental design
The design is a classical staged screening. A one-factor-at-a-time screen (stage 1) is followed by a full factorial on the two interacting smoothing filters (stage 2, 133 configs) and a confirmatory refinement around the top-5 (stage 3: × decimation {1,2,stage1_best} × hole fill {off,0,1,2} × HDR {off,on}) (README.md:272-293; sweep_config.yaml:96-140). The baseline runs in every stage, and de-duplication by `config_id` prevents re-running shared configs.

## 5. Why this approach

| Decision | Rationale | Label |
|---|---|---|
| Replay recordings offline instead of re-capturing per setting | All configs see identical raw data. Evaluate, Crop and Parameter_Sweep are used "as they are" | **[EXPLICIT: README.md:3-6,16-17]** |
| C++ `rs_decode` linked to the system librealsense | Wheels 2.57.7/2.58.4 align differently (~1.5% more valid pixels), so neither the frame map nor the baseline reproduces | **[EXPLICIT: README.md:244-254; rs_decode.cpp:5-8]** |
| Filters via pyrealsense2 software_device, not a reimplementation | "the real librealsense filters run on them". Validated bit-identical to the system 2.57.6 filters | **[EXPLICIT: README.md:134-136,268-270; filters.py:4-6]** |
| Align before filtering | rs_marker filters colour-aligned depth | **[EXPLICIT: README.md:144-146]** |
| Intel recommended order; disparity only around spatial/temporal | Intel recommends both in the disparity domain | **[EXPLICIT: filters.py:6-14]** |
| HDR after decimation | "as specified for this study". The source of that specification is not stated | **[EXPLICIT: filters.py:7]**; origin unclear |
| Frame map via valid-pixel anchor + tag depths + Viterbi | Timing alone was off by 1–3 frames for about half the frames; rs_marker never revisits a frame, so the path must be monotone | **[EXPLICIT: frames.py:172-179; README.md:120-133]** |
| Full warm-up for temporal | Temporal is stateful; ~3.5–5 s of history ≫ time constant | **[EXPLICIT: README.md:151-157]** |
| Scale DBSCAN eps by decimation | Without it, decimation ≥2 finds no part cluster (6 mm → fragments of ≤100 pts, 12 mm → one 1685-pt cluster) | **[EXPLICIT: evaluate.py:87-93; README.md:172-176]** |
| Part trim 30 mm | Whole tag square (~500 mm) let Evaluate lock onto a ~24° tilted pose. Big_Wave coverage 0.57→0.99, z-bias −10.3→−1.2 mm | **[EXPLICIT: README.md:177-182; sweep_config.yaml:59-63]** |
| "smaller" egg-carton as Small_Wave reference | No STL for the part. Best fit among candidates (97.6% coverage, −1.0 mm bias vs 72%/−6.3 mm) | **[EXPLICIT: README.md:208-214]** |
| Registration check at assembly time | A slid-off pose can still show low RMSE; threshold adjustable without recompute | **[EXPLICIT: results.py:45-46; sweep_config.yaml:64-66]** |
| Relative-to-own-baseline ranking | Makes shapes and heights comparable | **[EXPLICIT: README.md:295-297]** |
| Stage 2 holds decimation/hole fill off | "so resolution doesn't confound the grid" | **[EXPLICIT: README.md:288-290]** |
| Stage-2 delta trimmed to 1 value (temporal) / 2 values (spatial) | Stage-1 validation showed delta barely matters | **[EXPLICIT: sweep_config.yaml:121-123]** |
| One JSON row per job | Resume and crash-safety | **[EXPLICIT: runner.py:1-4; README.md:62-64]** |
| zlib-compressed cache | Store all frames once per scan (50–95 MB per scan) | **[EXPLICIT: README.md:120-122]**. The choice of zlib over e.g. npz is not explained |

Alternatives not taken: re-implementing the filters in numpy (rejected implicitly by the bit-identity goal) **[INFERRED]**; evaluating every frame rather than rs_marker's best frame (the study mirrors the deployed capture rule) **[INFERRED]**; a full factorial over all filters (cost is prohibitive at ~8 s wall/job) **[INFERRED from README.md:71-86]**.

## 6. Parameters and tuning

| Name | Default | Location | Effect |
|---|---|---|---|
| `data_root` | `/run/media/joab/Seagate/.../3D_Error` | sweep_config.yaml:18 | Source of recordings (absolute, machine-specific) |
| `shapes` / `heights` / `scans` | `[Small_Wave, Big_Wave]` / all / all | yaml:21-26 | Subset |
| `references` | two STL paths | yaml:31-33 | Ground truth per shape |
| `decoder` | `auto` | yaml:41 | cpp if built, else python (non-reproducing align) |
| `warmup_frames` | `null` (all) | yaml:49 | History fed to stateful filters |
| `deprojection` | `rs_marker` | yaml:54 | 2.57-compatible vs installed-wheel deprojection |
| `crop_margin_px` / `crop_frame_window` | 10 / 15 | yaml:57-58 | Cropper settings (as Parameter_Sweep) |
| `part_trim_margin_mm` | 30 | yaml:63 | XY trim around reference footprint |
| `min_covered_fraction` | 0.85 | yaml:66 | `suspect` threshold |
| `evaluate.*` | m / mm / seed 0 / full / no outputs | yaml:69-76 | Evaluate overrides |
| `workers` / `threads_per_worker` | 12 / 1 | yaml:79-80 | Parallelism |
| `estimate_s_per_job` | 17 | yaml:82 | Runtime estimate until measured |
| `rank_metric` | `acc_rmse_mm` | yaml:85 | Ranking metric; drives stage 2/3 |
| `filter_defaults` | dec 2; spatial m2 α0.5 δ20 h0; temporal α0.4 δ20 p3; hole 1; hdr on | yaml:89-94 | Values when a filter is on but not swept (librealsense defaults) |
| Stage 1 factors | see yaml:102-111 | | One-factor ranges |
| Stage 2 grid | spatial m{1,2,3} α{0.25,0.5,0.75} δ{8,30} h{0}; temporal α{0.1,0.25,0.5} δ{20} p{0,3}; hold dec 1, hole off | yaml:117-131 | 133 configs |
| Stage 3 | top_k 5 × dec {1,2,stage1_best} × hole {off,0,1,2} × hdr {false,true} | yaml:136-140 | ≤120 configs |
| `PIPELINE_VERSION` | 2 | config.py:23 | Bump to invalidate old rows |
| `CACHE_VERSION` | 3 | frames.py:34 | Bump to invalidate caches |
| `SEARCH` | 5 frames | frames.py:194 | Frame-map search window |
| Tag depth tolerance | 0.5 mm (+1e-6) | frames.py:217 | Depth match |
| `min_ok_fraction` | 0.8 | results.py:98 | Ranking eligibility |
| Trim minimum | 100 points | evaluate.py:132 | Fails job below this |

## 7. Assumptions, limitations, failure modes

* **Bit-exact reproduction depends on the library version.** The frame-map anchor needs an exact valid-pixel match, which the README says only the system 2.57.6 build gives. With the Python decoder the map falls back to "timing estimate" (frames.py:165-167). All 560 validation rows used `rs_decode 2.57.6` with "exact valid-pixel match" (counted from `results_validation/results.csv`).
* **HDR is untested** (README.md:222-223). `rs_decode` has no HDR in its chain, so the equivalence check cannot cover it.
* **Statistics.** The validation run (863 mm only, 4 scans, 2 per shape) is explicitly "not conclusions"; most differences are within the scan-to-scan spread (README.md:325-331). The full design gives 5 scans per (shape, height), of which 4 are repeats of one pose.
* **Best-frame rule rewards completeness.** Filters that fill holes change the selected frame, so accuracy and selection effects are confounded **[INFERRED]**.
* **Decimation 6–8** leaves 200–400 points on the part, so the metrics get noisy. Decimation 8 on Small_Wave was flagged suspect (README.md:239-242).
* **Small_Wave reference is inferred**, not known (README.md:208-214).
* **References not in git.** The root `.gitignore` excludes `Processing/Evaluate/References/` (.gitignore:10), so a fresh clone cannot run the sweep.
* **Relative metrics near zero.** For metrics whose baseline is close to 0 (e.g. `|signed bias|` ≈ 0.2 mm), ratios like −77% (validation summary: `dec5`/`dec6`) are dominated by noise **[INFERRED]**.
* **Data disk is hard-coded** (yaml:18). Missing folders produce warnings and "no scans found" (runner.py:179-182).
* **Runtime** is ~8–9 h for the full design on a 16-thread machine (README.md:85).

## 8. Dependencies

**Internal**
* `../Crop/crop_pointclouds.py`: `learn_tag_layout`, `compute_auto_crop` (AprilTag-bounded crop in colour-pixel space), `Intrinsics`, `pixel_rays` (deprojection), `write_ply_like_original`.
* `../Parameter_Sweep/sweep/prepare.seed_transform`: table plane to z=0, part footprint over the reference (DBSCAN cluster of raised points, `SEED_CLUSTER_EPS_M = 0.006`, `Parameter_Sweep/sweep/config.py:48`).
* `../Evaluate/scan_eval`: `EvalParams`, `evaluate_scan` (alignment and all accuracy metrics), `load_reference(height_field=True)`, `UnitScaled`.
* Capture (`rs_marker`) file formats: `.db3`, `_detections.csv` (`frame`, `timestamp_ms`, `cx`, `cy`, `depth_m`), `scan_info.txt` keys, `imu_a*.csv`.

**Third-party**
* **librealsense2** (system, C++), used by `rs_decode`: `rs2::pipeline` playback, `rs2::align`, `rs2::decimation_filter`/`spatial_filter`/`temporal_filter`/`hole_filling_filter`/`disparity_transform`.
* **pyrealsense2**: `software_device`, `software_video_frame`, `frame_queue`, filter objects, `pointcloud`, playback (fallback).
* **numpy**, **zlib** (stdlib) for the frame cache. **scipy** `cKDTree` for the validation NN distance.
* **open3d** for PointCloud I/O and as a dependency of Crop/Evaluate.
* **pandas/pyarrow** for the results table (parquet). **pyyaml** for the config. **tqdm** for progress. **matplotlib** for plots.
* **multiprocessing** (spawn) and **subprocess** to run `rs_decode`.

## 9. Observations

1. **Root `.gitignore` ignores the whole folder.** `.gitignore:11` is `Processing/Filter_Sweep`, added in the same commit that added the tracked files (00b2c4e). Existing tracked files stay tracked, but any **new** file added under Filter_Sweep will silently not appear in `git status`. The folder's own `.gitignore` already covers `.venv/`, `cache/`, `results/`, `results_*/`, `decoder/build/` and `__pycache__/`, so the root line is redundant and risky.
2. **Validation results are not versioned.** `results_validation/`, cited in README.md:313-331, is ignored by `results_*/`, so the README's numbers cannot be traced to committed data.
3. **The frame passed to the cropper is chosen by timing, not by the depth-matched map.** `rs_frame_for` (frames.py:265-269) picks the rs_marker frame nearest in time, although `frame_map` built a depth-confirmed `rs_to_bag` that README.md:132-133 says corrects 1–3 frame timing errors for about half the frames. The selected rs frame given to the cropper can therefore be off by one. The cropper's 15-frame window probably masks this **[INFERRED]**.
4. **`validate.check_filters` assumes the cache starts at frame 0.** The Python side feeds `cache.stored`, while `rs_decode` always starts at recorded frame 0 (validate.py:71-90). With a non-null `warmup_frames` the temporal filter histories differ, which would produce a spurious "DIFFERS". The code also prints `o.shape` after the loop (validate.py:98), which would raise `NameError` if no frame was processed.
5. **HDR placement and testing.** HDR is placed after decimation, unlike Intel's usual order (HDR/sequence filter first). This is justified only as "as specified for this study" (filters.py:7). It is never tested, and `chain_spec` and `rs_decode` cannot express it.
6. **Fallback decoder is non-reproducing by design.** `decoder: auto` silently falls back to Python, which by the README's own account does not reproduce the baseline. A run on a machine without the C++ build would give a different frame map and valid counts, flagged only in the `decoder` and `frame_map_method` columns. Using `cpp` as the default would be safer.
7. **Global mutated per call.** `_seed_spacing` mutates the global `seed_config.SEED_CLUSTER_EPS_M` (evaluate.py:97-103). This is safe in a single-threaded worker process, but fragile if anything ever runs threaded.
8. **No YAML schema validation.** `Settings.__getattr__` exposes raw keys (config.py:51-55). A misspelt key (e.g. `min_coverd_fraction`) surfaces only as an `AttributeError` when first used, possibly mid-run.
9. **Threshold filter is absent.** The task brief lists it, but the sweep neither uses nor discusses it. The thesis should say it was excluded.
10. **Relative ranking of signed or near-zero metrics** (see §7) can be unstable. `plot_results` ranks `abs_signed_bias_mm` by relative change.
11. **Committed `.pyc` from this work.** Commit 00b2c4e also committed `Processing/Crop/__pycache__/crop_pointclouds.cpython-313.pyc`, a by-product of importing the cropper from Filter_Sweep (see file 13). It also committed `Capture/gdb_crash.log` (2017 lines).
12. **Hard-coded absolute paths** (`data_root`) and machine-specific runtime claims (README.md:71-86) limit portability, though both are clearly documented.
13. **The README's code table and Setup are accurate** against the source. The stage counts 51/133 check out arithmetically (§3.6).

*All behaviour above is derived from reading the source. Nothing was executed apart from read-only inspection of the existing `results_validation/results.csv` (status, decoder and frame-map columns).*
