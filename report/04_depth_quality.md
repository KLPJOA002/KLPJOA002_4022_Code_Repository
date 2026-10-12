# 04 — Depth-Quality Characterisation Tool (`depth_quality`)

**Summary.** `depth_quality` is a single-file C++17 program that characterises the depth accuracy and precision of the Intel RealSense D456 against a flat target (floor or board) at a known distance and tilt. It follows the general approach of Intel's Depth Quality Tool (DQT). For every depth frame it takes a region of interest (ROI) and computes the fill rate, the Z error against a ground-truth plane (median, mean, MAE, RMSE, and the median as a percentage), a least-squares plane fit (plane-fit RMS in mm and %, sub-pixel RMS in disparity pixels, fitted distance, pitch and roll), and the errors between the fitted and ground-truth planes. It also logs the IR brightness, exposure, gain, laser and emitter metadata, and the accelerometer reading. At the end of each trial it adds a temporal-noise figure: the median over ROI pixels of each pixel's standard deviation through time. The tool runs either live (it records a `.db3` file for each trial — ROS1 bag content despite the extension, per `Processing/Filter_Sweep/README.md:92` and prompts between trials so the rig can be changed) or in **reprocess** mode. Reprocess mode replays earlier `.db3` recordings without a camera, reads each trial's settings back from its `trial_info.txt`, and lets command-line flags override them. The ROI can be fixed or a "dynamic patch" ROI, which re-projects a fixed-size square in the target plane into the image each frame. Everything below comes from reading the source only. Nothing was compiled or run. Numbers quoted from an existing run folder were read from files on disk, not produced by running anything.

## Files covered

| Path | Lines | Role |
|---|---|---|
| `Capture/tools/Depth_Quality/depth_quality.cpp` | 1746 | The whole tool: CLI, capture/playback, metrics, outputs |
| `Capture/tools/Depth_Quality/depth_quality` | (binary, 281 KB) | Prebuilt ELF binary, committed to git (not analysed) |
| `Capture/tools/Depth_Quality/dq_runs/` | (data, 7.9 GB, git-ignored by `.gitignore:2`) | Example output run folders. One `trial_info.txt` and `summary.csv` were read for context |
| `Processing/Height_Analysis/Copy_Trial_Files.sh` | (context only) | Copies `*.csv` + `trial_info.txt` out of trial trees, e.g. `.../Joab_Characterisation/Flat_Error`. It is the probable downstream consumer |

Related commits: `bf56d08` (initial 1196-line tool, "for characterisation"), `c9f142d` (+250 lines: dynamic patch ROI, `roi_start`), `93c5bd9` (+464/−152: `--reprocess` of `.db3`), `5354dca` (use `depth_scale_m` from `trial_info.txt` during reprocess).

## Contents
1. Purpose
2. Inputs and outputs
3. Step-by-step walkthrough
4. Algorithms and methods (the metric maths)
5. Why this approach
6. Parameters and tuning
7. Assumptions, limitations, failure modes
8. Dependencies
9. Observations

---

## 1. Purpose

The tool measures how good the D456 depth is under controlled conditions (height, tilt, lighting, preset, emitter, exposure, filters). The goal is to quantify sensor error before it is used for sea-ice surface-roughness scanning. The file header calls it a "RealSense depth characterisation (Depth-Quality-Tool style)" tool with the "Same overall structure as the capture tool (main.cpp)" (`depth_quality.cpp:1-6`) **[EXPLICIT: depth_quality.cpp:1-6]**. The capture tool referred to is `rs_marker`. The commit message for `bf56d08` reads "added code to do use depth quality tool work, for characterisation" **[EXPLICIT: bf56d08]**.

Place in the system: this is a stand-alone side branch of the capture tooling. Its preset (`high_density`), depth units (0.1 mm) and post-processing chain deliberately copy the capture tool (`depth_quality.cpp:76, 173, 254`) so that the characterisation applies to real scans **[EXPLICIT: comments "as in the capture tool" at :141, :173, :254]**. Its CSV outputs appear to feed a separate height/error analysis. `Processing/Height_Analysis/Copy_Trial_Files.sh` gathers "every *.csv and trial_info.txt" from a characterisation tree (example path `Joab_Characterisation/Flat_Error`) **[INFERRED from that script's header]**.

## 2. Inputs and outputs

### 2.1 Camera streams (live mode, `start_pipeline`, `:236-265`)

| Stream | Index | Format | Default size/rate | Notes |
|---|---|---|---|---|
| Depth | 0 | Z16 (uint16) | 1280×720 @30 | Units set to 0.0001 m (0.1 mm) `:254` |
| Infrared | 1 (left) | Y8 | 1280×720 @30 | "pixel-aligned with depth" `:241`. Used for the ROI overlay and the `ir_mean` metric |
| Colour | 0 | RGB8 | 1280×720 @30 | Display only ("context only, not aligned" `:1244`); `-noColor` disables |
| Accel | — | MOTION_XYZ32F | default | Last sample logged per frame (m/s²); `-noIMU` disables |

Device options applied after `pipe.start` (`:251-263`): visual preset, `DEPTH_UNITS=0.0001`, `THERMAL_COMPENSATION=0`, and optionally emitter, laser power, and manual exposure (otherwise auto-exposure on).

### 2.2 Command-line interface (`parse_args`, `:979-1037`; usage text `:111-171`)

| Flag | Meaning | Default |
|---|---|---|
| `-H/--height MM` | Perpendicular distance from the depth origin to the target plane | 0 (unused) |
| `-Z/--axis-dist MM` | Alternative: distance along the optical axis | 0 |
| `-p/--pitch DEG`, `-R/--roll DEG` | Tilt; + pitch = top of image farther, + roll = right of image farther | 0 |
| `--gt-offset MM` | Added to measured distances (front glass → depth origin) | 0 |
| `-L/--label`, `--lux` | Logged metadata | "", NaN |
| `-r/--roi X,Y,W,H`, `--roi-pct P`, `--select-roi` | Static ROI | centred 40 % |
| `--patch-size MM`, `--patch-outer`, `--patch-smooth A`, `--patch-hyst PX` | Dynamic patch ROI | 0 (off), inscribed, 0.3, 3 |
| `-D/--dir`, `-d/--duration S`, `-n/--frames N` | Output root and trial length | `./dq_runs`, until `q` |
| `--res WxH`, `--fps N`, `--preset NAME` | Stream settings | 1280x720, 30, `high_density` |
| `--emitter on/off`, `--laser-power MW`, `--exposure US` | Projector/exposure | leave as-is / auto |
| `--filters`, `--min-dist M`, `--max-dist M` | Apply post-processing chain, threshold limits | off, 0.2, 3.0 |
| `--ext`, `-noRecord`, `-noColor`, `-noIMU` | Recording/streams | `.db3`, record on |
| `--print-every N`, `--err-range MM` | Console rate, error-map colour range | 1, 10 |
| `--reprocess PATH` (repeatable), `--show`, `--flip-pitch`, `--flip-roll`, `--all-frames` | Reprocessing | — |

Between live trials an interactive prompt (`prompt_next_trial`, `:931-973`) accepts `key=value` tokens: `h=`, `z=`, `pitch=`, `roll=`, `lux=`, `label=`, `gt=none`, `roi=x,y,w,h|select`, `patch=<mm>|off`, `emitter=`, `laser=`, `exposure=auto|<us>`. Enter repeats the last settings and `q` finishes.

### 2.3 Output layout

Live: `<dir>/<YYYYmmdd_HHMMSS>/` (`:1221-1222`). Each trial gets a folder named by `trial_name` (`:903-912`):

`<run_ts>_t<idx>_h<mm>|_z<mm>|_noGT_p<pitch.1>_r<roll.1>[_<label>]`

(e.g. `20260918_152706_t0_h1070_p0.0_r0.0_Test_New_ROI`).

| File | Content |
|---|---|
| `<trial>/<trial>.db3` | Raw recording (depth, IR1, colour, accel) via `enable_record_to_file` (`:246-247`); omitted with `-noRecord` and always omitted in reprocess |
| `<trial>/<trial>_metrics.csv` | One row per analysed frame, 37 columns (`:790-795`): `frame, timestamp_ms, device_ts_ms, roi_x..roi_h, roi_px, valid_px, fill_pct, mean_z_mm, std_z_mm, gt_axis_mm, z_err_median_mm, z_err_median_pct, z_err_mean_mm, z_err_mae_mm, z_err_rmse_mm, plane_rms_mm, plane_rms_pct, subpixel_rms_px, plane_dist_perp_mm, plane_dist_axis_mm, plane_pitch_deg, plane_roll_deg, plane_tilt_deg, dist_err_mm, pitch_err_deg, roll_err_deg, ir_mean, exposure, gain, laser_power, emitter_mode, accel_x/y/z`. Non-finite values are written as `NaN` (`:762-766`) |
| `<trial>/trial_info.txt` | `key: value` header (device, serial, firmware, GT, intrinsics, `depth_scale_m`, `baseline_mm`, preset, filters, ROI, `roi_start`, patch settings, record path, and when reprocessing `reprocessed_from`, `source_*`, `frame_selection`), then a blank line, then the human-readable summary block (`:1663-1698`) |
| `<run>/summary.csv` | One appended row per trial: setup columns, `frames`, `temporal_noise_mm`, plus `<field>_mean,<field>_std` for 14 summary fields (`:1700-1733`, `SUMMARY_FIELDS :840-855`) |
| `<trial>/snap_f<N>_{ir,depth,err}.png` | Only when `s` is pressed (`:1576-1582`) |

Reprocess: `<dir>/<ts>_reprocess/reprocess_info.txt` (command line plus a pitch/roll before→after table, `:1228-1240`). Under it, `<ts>_reprocess/<original run_ts>/<trial name>/` mirrors the original layout without the recording (`:1186-1187`, `:1289-1294`).

Units and frames: all metric distances are in mm. Coordinates are in the **depth camera optical frame** (x right, y down, z forward), as stated at `:371`. Angles are in degrees. `timestamp_ms` is milliseconds since the trial started (live) or media time since the first analysed frame (reprocess). `device_ts_ms` is the librealsense frame timestamp.

## 3. Step-by-step walkthrough

1. **Argument parsing** `main :1196-1203` → `parse_args`. The function applies `argv` on top of an existing `Options`, so the same function is reused later to override trial_info values (`:976-978` comment).
2. **Validation** `:1205-1216`: |pitch|, |roll| < 80°. `--select-roi` while reprocessing needs `--show`. Reprocess must find at least one recording.
3. **SIGINT/SIGTERM** set `g_running=false` (`:51-52, :1218-1219`). The frame loop then exits cleanly, and the trial summary is still written.
4. **Session folder** `make_session_dir` (`:881-894`; `mkdir_p` `:867-879`). The program calls `exit(1)` if the folder already exists.
5. **Reprocess job list** `build_jobs` (`:1157-1191`):
   - `find_recordings` (`:1138-1154`) recursively collects `.db3`/`.bag` files, canonicalised, sorted and de-duplicated in a `std::set`.
   - For each recording: `load_trial_info` (`:1085-1130`) reads the header up to the first blank line. `roi_start` takes precedence over `roi` (`:1116-1119`). `depth_scale_m` was added in `5354dca`.
   - The original pitch/roll are remembered. `parse_args(argc, argv, j.o)` is re-applied, so precedence is **defaults < trial_info < CLI** (`:1156`). Then the optional sign flips (`:1175-1176`).
   - The original `*_metrics.csv` is located and, unless `--all-frames`, its `device_ts_ms` column is loaded as integer microsecond keys (`load_frame_keys :1062-1081`, `ts_key :1059`).
6. **Windows** created only when `show` (always in live mode, opt-in for reprocess, `:1210, :1245-1250`).
7. **Trial loop** `:1258-1741`:
   - In live mode, the prompt runs at the *top* of the loop, after the previous pipeline has been destroyed. The comment explains that the recorder only writes the bag index when it is destroyed, so killing the program at the prompt can no longer leave an unindexed (unreadable) recording (`:1260-1263`) **[EXPLICIT]**.
   - Pipeline: `start_playback` (`:284-292`, `repeat_playback=false`, `set_real_time(false)`, so every recorded frame is delivered however slowly it is processed) or `start_pipeline_retry` (`:267-280`, 5 attempts with a 0.5·k s back-off).
   - Camera model: depth intrinsics (`:1318`). `scale_m = ds.get_depth_scale()`; in reprocess the `trial_info` value is used instead if it differs (`:1319-1324`, commit `5354dca`). The stereo baseline comes from `stereo_baseline_mm` (`:295-308`), falling back to `trial_info` because "IR2 is not in the recording" (`:1326`).
   - `analyzer.configure` (`:568-585`) precomputes a unit-depth ray for every pixel with `rs2_deproject_pixel_to_point(…, 1.0)` and allocates the per-pixel temporal accumulators.
   - ROI: explicit, or centred `roi_pct` % (`centred_roi :922-927`), clipped to the image. If clipping leaves nothing, the code falls back to the centred ROI.
   - Ground truth `make_gt` (`:382-396`). Dynamic-ROI anchor ray through the ROI centre (`:1347-1353`). If ground truth exists, the patch is seeded so frame 0 already has the right ROI size (`:1359-1367`).
   - Console banner (`:1373-1400`). `DepthFilters` constructed fresh each trial (`:1402`).
8. **Frame loop** `:1418-1627`:
   - Stop conditions: duration (media time when reprocessing) and frame limit (`:1419-1420`).
   - Get frames: `try_wait_for_frames(5000)` in playback (a timeout means end of file) or `wait_for_frames()` live (`:1422-1433`).
   - Store the last accel sample (`:1435-1436`).
   - Duplicate-frame guard: "With the IMU streaming, the syncer can hand over the same depth frame in consecutive framesets" (`:1440-1444`) **[EXPLICIT]**.
   - Reprocess frame selection: analyse a frame only if its µs key (±1 µs) is in the original CSV. The reasons given are that the original run "drops frames it cannot keep up with, and the recording starts before its loop" (`:1445-1451`) **[EXPLICIT]**.
   - Optional post-processing `filters.process(raw_depth)` (`:1456`).
   - Interactive ROI selection with `cv::selectROI` on the IR image. This resets the anchor, the patch and the history, then `continue`s (`:1464-1483`).
   - `analyzer.analyze(depth)` (`:613-721`, see §4), then metadata (`:1494-1500`): `ACTUAL_EXPOSURE`, `GAIN_LEVEL`, `FRAME_LASER_POWER`, `FRAME_EMITTER_MODE`, accel, and the IR mean over the ROI.
   - CSV row, history, console line (`:1502-1504`).
   - One-off sanity warnings at frame 30: fitted pitch/roll has the opposite sign to the given value, or fitted distance is more than 10 % from ground truth (`:1507-1520`).
   - Display: IR with the ROI and patch quad, text block, colourised depth, error map (`render_error_map :354-368`: blue = closer, red = farther, black = hole), colour. Keys `q`/Esc, `r`, `s` (`:1523-1583`).
   - Dynamic ROI update (`:1588-1624`, see §4.7).
9. **Trial end** `:1629-1740`: stop the pipeline (which also closes the recording), compute `temporal_noise_median_mm(max(2, n_frames/2))`, print the summary, write `trial_info.txt` and append to `summary.csv`. In live mode, set `ask=true` and increment `trial_index`.

## 4. Algorithms and methods

### 4.1 Camera model and back-projection

Pinhole deprojection via `rs2_deproject_pixel_to_point` (which also applies the stream's distortion model, if any). For pixel $(u,v)$ the code precomputes the ray at unit depth $\mathbf r_{uv} = (x_n, y_n, 1)$ with $x_n = (u - c_x)/f_x$ and $y_n = (v - c_y)/f_y$ in the undistorted case (`:574-580`). A valid raw depth value $D_{uv}$ (uint16) gives

$$Z = D_{uv}\cdot s\cdot 1000\ \text{mm},\qquad \mathbf P = Z\,\mathbf r_{uv} = (x_n Z,\ y_n Z,\ Z)$$

with $s$ the depth scale in m/unit (0.0001 → 0.1 mm/unit) (`:571, :633-635`). A zero value marks a hole.

### 4.2 Ground-truth plane (`make_gt`, `:370-396`)

The target plane is $\mathbf n\cdot\mathbf P = H$ with $\lVert\mathbf n\rVert=1$ and $n_z>0$ (`:372`). From the user's tilt angles:

$$\mathbf n \propto \big(-\tan(\text{roll}),\ \tan(\text{pitch}),\ 1\big)$$

so that $\text{pitch}=\operatorname{atan2}(n_y,n_z)$ and $\text{roll}=\operatorname{atan2}(-n_x,n_z)$, the same convention used for the fitted plane (`:373-374, :673-674`). In the y-down frame, $n_y>0$ makes points near the top of the image ($y<0$) lie farther away, which matches "+ = top of image farther".

The distance is given either perpendicular ($H = h + o\,n_z$) or along the optical axis ($H = (z_{ax}+o)\,n_z$), where $o$ is `--gt-offset` (front glass → depth origin, applied along the optical axis). The on-axis distance is $z_{ax}=H/n_z$ (`:390-393`).

### 4.3 Fill rate (`:642-643`)

$$\text{fill\%} = 100\cdot\frac{N_{\text{valid}}}{N_{\text{ROI}}}$$

where valid means a non-zero depth value. This is Intel's "fill rate" definition.

### 4.4 Z accuracy against ground truth (`:678-719`)

For each valid pixel, the reference depth *along the same ray* is where the ray meets the GT plane: $Z_{\text{ref}} = H/(\mathbf n\cdot\mathbf r)$ (`:686-688`). The error is

$$e_i = Z_i - Z_{\text{ref},i}\quad(\text{positive = measured farther})$$

Reported values: $\operatorname{median}(e)$, $\operatorname{median}(100\,e_i/Z_{\text{ref},i})$, $\bar e$, $\text{MAE}=\overline{|e|}$ and $\text{RMSE}=\sqrt{\overline{e^2}}$ (`:709-713`). `median_of` (`:522-531`) uses `nth_element` and averages the two middle elements for even counts. When there is no ground truth, the per-pixel error map is computed against the fitted plane instead (`:689-692`), but no Z-accuracy statistics are reported.

This matches the "Z accuracy" (median signed error, absolute and % of GT distance) in Intel's depth-testing methodology (Intel white paper *"Intel RealSense Camera Depth Testing Methodology"*, Grunnet-Jepsen et al., 2018, implemented in `rs-depth-quality`) **[INFERRED: the header only says "Depth-Quality-Tool style"; the white paper is not cited in the code]**.

### 4.5 Plane fit: total least squares by PCA (`fit_plane`, `:537-564`)

Centroid $\bar{\mathbf p}$, then the scatter matrix $C=\sum_i(\mathbf p_i-\bar{\mathbf p})(\mathbf p_i-\bar{\mathbf p})^T$. The normal $\mathbf n$ is the eigenvector of the **smallest** eigenvalue (`cv::eigen` returns them in descending order, so row 2 is taken, `:557-558`), oriented so that $n_z>0$. The offset is $d=-\mathbf n\cdot\bar{\mathbf p}$. This minimises the sum of squared **orthogonal** distances (orthogonal regression). No outlier rejection is applied (no RANSAC or iterative reweighting).

Derived quantities (`:654-675`):
- Plane-fit RMS: $\text{RMS}_\perp=\sqrt{\frac1N\sum_i(\mathbf n\cdot\mathbf p_i+d)^2}$ mm.
- Perpendicular distance $-d$ and on-axis distance $-d/n_z$. `plane_rms_pct` $=100\cdot\text{RMS}_\perp/(-d/n_z)$.
- Pitch $=\operatorname{atan2}(n_y,n_z)$, roll $=\operatorname{atan2}(-n_x,n_z)$, total tilt $=\arccos n_z$.
- Fit compared with GT: `dist_err` $=-d-H$, `pitch_err`, `roll_err` (`:714-718`).

### 4.6 Sub-pixel RMS (disparity-domain error, `:652-672`)

For a stereo camera, disparity is $\delta = fB/Z$, where $f$ is the focal length in pixels (`intr.fx`) and $B$ the baseline in mm. For each pixel the fitted plane's depth along the ray, $Z_{\text{fit}}=-d/(\mathbf n\cdot\mathbf r)$, gives

$$\varepsilon_i = fB\Big(\frac1{Z_i}-\frac1{Z_{\text{fit},i}}\Big)\ [\text{px}],\qquad \text{subpixel RMS}=\sqrt{\tfrac1N\sum\varepsilon_i^2}$$

This turns the depth error into the stereo-matching error in pixels, a figure that does not depend on distance. Because $\partial Z/\partial\delta = -Z^2/(fB)$, the expected depth RMS is approximately $Z^2\,\sigma_\delta/(fB)$. As a consistency check against the run `20260918_152706` (read from its `trial_info.txt`): $Z\approx1079$ mm, $f=647.2$ px, $B=94.81$ mm and $\sigma_\delta=0.346$ px give $1079^2\cdot0.346/(647.2\cdot94.81)\approx6.57$ mm, which matches the logged `plane_rms_mm` of 6.58 mm. The baseline comes from `RS2_OPTION_STEREO_BASELINE` or the IR1→IR2 extrinsic translation (`:295-308`). If no baseline is available ($B=0$), the sub-pixel metric is NaN.

### 4.7 Dynamic "patch" ROI (`project_patch`, `:398-496`; update `:1588-1624`)

Purpose: keep the area of target being characterised at a constant `patch_mm²` as the camera height and tilt change (`:405-407`) **[EXPLICIT]**. Steps:
1. Patch centre $\mathbf C = \mathbf a\,H/(\mathbf n\cdot\mathbf a)$, where $\mathbf a$ is the anchor ray through the centre of the starting ROI (ray–plane intersection, `:436-440`).
2. In-plane orthonormal axes: $\mathbf e_1=\mathrm{normalize}(\hat x - \mathbf n n_x)$ (Gram–Schmidt projection of camera +x onto the plane) and $\mathbf e_2=\mathbf n\times\mathbf e_1$ (`:442-448`).
3. The four corners $\mathbf C\pm\tfrac s2\mathbf e_1\pm\tfrac s2\mathbf e_2$ are projected with `rs2_project_point_to_pixel` (`:450-461`).
4. ROI = the bounding box of the quad (`--patch-outer`), or by default the largest box centred on the projected centre that fits inside the quad. That box is found by **bisection** (24 iterations) on a scale factor $t\in[0,1]$ of the half-extents, with a convex point-in-quad test using the signs of 2-D cross products (`point_in_quad :417-428`, `:475-493`).

Plane estimate for the next frame: this frame's fitted plane (ground truth only as a fallback), smoothed by an **exponential moving average** with $\alpha$ = `patch_smooth`: $\mathbf n\leftarrow\mathrm{normalize}((1-\alpha)\mathbf n+\alpha\mathbf n_{\text{meas}})$ and $H\leftarrow(1-\alpha)H+\alpha H_{\text{meas}}$ (`:1601-1607`). **Hysteresis**: the ROI moves only if a corner or size changes by more than `patch_hyst_px` (`:1614-1621`), and in that case the temporal statistics are *not* reset (`set_roi(..., false)`, rationale at `:587-591`) **[EXPLICIT]**. A minimum size of 8 px is enforced (`patch_min_px`, `:100`).

### 4.8 Temporal noise (`temporal_noise_median_mm`, `:723-738`)

Per pixel, running sums $\sum Z$, $\sum Z^2$ and a count are kept through time (`:637`). At the end of the trial, for ROI pixels seen in at least `max(2, n_frames/2)` frames, the unbiased sample standard deviation is

$$\sigma_{uv}=\sqrt{\frac{\sum Z^2 - c\,\bar Z^2}{c-1}}$$

and the metric is $\operatorname{median}_{uv}\sigma_{uv}$. This is the usual "temporal noise" definition in Intel's methodology **[INFERRED]**. In live mode the per-frame spatial `std_z_mm` (`:646`) uses the population form.

### 4.9 Per-trial aggregation (`stat_of`, `:822-837`)

For each of the 14 `SUMMARY_FIELDS`, the mean and sample standard deviation *across frames* are computed with NaN frames ignored. For example, `z_err_median_mm_std` is the frame-to-frame scatter of the per-frame median error.

### 4.10 RealSense post-processing chain (`DepthFilters`, `:173-208`, only with `--filters`)

The order is threshold → depth→disparity → spatial → temporal → disparity→depth → hole filling. This is the chain recommended in Intel's post-processing white paper (Grunnet-Jepsen & Tong, *Depth Post-Processing for Intel RealSense D400 Depth Cameras*) **[INFERRED: the code only says "same chain as the capture tool" `:173`]**. Settings: threshold limits `min_dist..max_dist` m; spatial edge-preserving filter α=0.8, δ=5, 1 iteration (magnitude); temporal filter α=0.06, δ=100, persistence mode 1; hole filling mode 1. Raw depth is analysed by default.

### 4.11 Reprocessing frame matching

Depth timestamps (ms, double) are rounded to integer µs (`ts_key`) and matched against the original CSV's `device_ts_ms` (written to 3 decimal places, i.e. µs) with a ±1 µs tolerance for rounding (`:1445-1451`). This lets reprocessed results "compare 1:1" with the original run (`:166-168`) **[EXPLICIT]**.

## 5. Why this approach

- **Copying the DQT metrics** (fill rate, Z accuracy, plane-fit RMS, sub-pixel RMS, temporal noise). The header says "Depth-Quality-Tool style" (`:1`) **[EXPLICIT]**. Using the vendor's metrics lets the results be compared with Intel's datasheet specifications (e.g. "≤2 % at 4 m" Z accuracy, sub-pixel RMS) **[INFERRED]**. The alternative, Intel's own `rs-depth-quality` GUI, does not log the IMU, metadata or tilt-aware ground truth in a scriptable way, and cannot reprocess `.db3` files with modified ground truth **[INFERRED]**.
- **Per-ray Z error rather than a single distance.** Comparing each pixel with the GT plane along its own ray handles tilted targets exactly. A single "distance" comparison only holds for a fronto-parallel target **[INFERRED]**.
- **PCA plane fit** rather than regressing $z=ax+by+c$. PCA is unbiased for noise in every axis and does not depend on the parametrisation. It is cheap (3×3 eigenproblem). The cost is that it is not robust to outliers **[INFERRED]**.
- **Dynamic patch ROI.** A fixed pixel ROI covers a different floor area at different heights, which makes characterisations at different heights hard to compare. The patch keeps the floor area constant (`:405-407`) **[EXPLICIT]**. EMA and hysteresis suppress jitter (`:1603, :1618-1619`) **[EXPLICIT]**.
- **Recording plus reprocessing.** This makes it possible to fix ground-truth mistakes (e.g. a pitch sign, see the `--flip-pitch` flag and the sign warnings at `:1508-1515`) or to try filters without repeating fieldwork (commit `93c5bd9` "reprocess previously captured .db3 scans, and update or keep parameters") **[EXPLICIT: 93c5bd9]**.
- **Depth scale from `trial_info.txt` when reprocessing** (`5354dca`, "use depth units stored in trial_info.txt instead of reinitialising") **[EXPLICIT]**. Depth units are set *after* `pipe.start()` (`:249-254`), so the recording's stored depth-sensor scale may be the device default (0.001) rather than 0.0001. That would make reprocessed distances 10× wrong **[INFERRED]**.
- **Prompting after the pipeline is destroyed**, to avoid unindexed recordings (`:1260-1263`) **[EXPLICIT]**.
- **Thermal compensation disabled** (`:255`): no rationale is given in the code. One plausible reason is to avoid periodic re-calibration steps in depth during a trial **[INFERRED; unclear from source]**.
- **0.1 mm depth units**: "as in capture tool" (`:254`) **[EXPLICIT]**. This gives finer quantisation (Z16 × 0.1 mm), at the cost of a maximum range of 6.55 m **[INFERRED]**.

## 6. Parameters and tuning

| Name | Default | Location | Effect |
|---|---|---|---|
| `roi_pct` | 40 % | `:92` | Centred static ROI (512×288 px at 1280×720) |
| `patch_size` | 0 (off) | `:96` | Side length of the floor patch in mm; enables the dynamic ROI |
| `patch_outer` | false | `:97` | Bounding box (covers patch plus extra) vs inscribed box (all on patch) |
| `patch_smooth` | 0.3 (clamped 0.01–1) | `:98, :1002` | EMA weight of the newest plane estimate |
| `patch_hyst_px` | 3 | `:99` | Minimum ROI change (px) before the ROI is moved |
| `patch_min_px` | 8 | `:100` | Minimum ROI side; **no CLI flag** |
| Bisection iterations | 24 | `:483` | Precision of the inscribed-box scale (2⁻²⁴) |
| `preset` | `high_density` | `:76` | D4xx visual preset |
| Depth units | 0.0001 m | `:254` | Hard-coded |
| Thermal compensation | 0 | `:255` | Hard-coded off |
| Spatial α/δ/mag | 0.8 / 5 / 1 | `:187-189` | Edge-preserving smoothing (with `--filters`) |
| Temporal α/δ/persist | 0.06 / 100 / 1 | `:191-193` | Strong temporal smoothing (with `--filters`) |
| Hole fill | 1 | `:195` | Fills holes (with `--filters`) |
| `min_dist`/`max_dist` | 0.2 / 3.0 m | `:81` | Threshold filter (with `--filters`) |
| `err_range_mm` | ±10 mm (min 0.1) | `:86` | Colour saturation of the error map |
| Temporal-noise min count | max(2, n_frames/2) | `:1635` | Pixels seen in fewer frames are excluded |
| Sanity warning frame | 30; thresholds 2°, 10 % | `:1507-1516` | One-off warnings |
| Pipeline retries | 5, back-off 0.5·k s | `:267-277` | Live start robustness |
| Playback timeout | 5000 ms | `:1425` | End-of-recording detection |
| Tilt limit | 80° | `:1205, :1277` | Rejects near-grazing setups |

## 7. Assumptions, limitations, failure modes, edge cases

- **The target is a plane.** The ROI must contain only the planar target. Edges, objects or flying pixels inflate the plane RMS because there is no outlier rejection (§4.5). The textured floor surface is assumed flat to well below the sensor noise.
- **Ground-truth tilt convention.** The normal is built from $\tan$ of the pitch and roll components (§4.2). That is not the same as composing two physical rotations (e.g. $R_x R_y$) when both angles are non-zero. If inclinometer angles are entered, combined tilts carry a small systematic GT error **[INFERRED]**.
- **The GT distance reference point** depends on `--gt-offset`, which defaults to 0 even though the help text describes it as "front glass → depth origin". In the example run, a consistent +9.2 mm `dist_err_mm` at 1070 mm was logged with `gt_offset_mm: 0` (read from `dq_runs/20260918_152706/.../trial_info.txt`). Part of that is plausibly the unmodelled glass-to-origin offset **[INFERRED]**.
- **With `--filters`**: hole filling makes `fill_pct` close to 100 % by construction, and the temporal filter suppresses temporal noise. The filtered metrics therefore do not describe raw sensor quality **[INFERRED]**.
- **Dynamic ROI feedback loop.** The plane used to place the ROI is fitted inside the ROI. If the ROI drifts onto a non-planar region, the fit and the ROI can both drift. The EMA damps this but does not prevent it **[INFERRED]**.
- **Temporal noise when the camera moves** (the patch comment talks about following the floor "as the scan moves", `:1586-1587`): the per-pixel standard deviation then includes real scene change, so `temporal_noise_mm` is only meaningful for a static rig **[INFERRED]**.
- **The first frames after setting depth units** may still be in the old units, because the options are set after `pipe.start` (`:249-254`). No warm-up frames are discarded **[INFERRED]**.
- **Reprocessing**: capture-only flags (`--preset`, `--res`, …) are still parsed into `o` and written into the new `trial_info.txt` and `summary.csv`, even though they cannot affect playback. The metadata can then misstate the recording (usage text admits they "do not" apply, `:160-161`).
- `make_session_dir` calls `exit(1)` if a trial folder already exists (`:888-892`). One collision in reprocess mode aborts the whole batch.
- A `.bag` file is accepted by `is_recording` (`:1135`), but whether the installed librealsense can play ROS1 bags depends on the version. This cannot be determined from source.
- With no ground truth, `z_err_*` are NaN and the error map is relative to the fitted plane.

## 8. Dependencies

| Library | Calls and what they do |
|---|---|
| librealsense2 (`rs.hpp`, `rsutil.h`) | `rs2::pipeline/config` start streams, `enable_record_to_file` (writes `.db3`; Filter_Sweep documents these as ROS1 bag content), `enable_device_from_file` + `playback::set_real_time(false)` (deterministic replay); `depth_sensor::set_option` (preset, units, emitter, laser, exposure, thermal comp.), `get_depth_scale`; `RS2_OPTION_STEREO_BASELINE` / `get_extrinsics_to` (baseline); `rs2_deproject_pixel_to_point` / `rs2_project_point_to_pixel` (pinhole + distortion model); frame metadata (`ACTUAL_EXPOSURE`, `GAIN_LEVEL`, `FRAME_LASER_POWER`, `FRAME_EMITTER_MODE`); processing blocks `threshold_filter`, `disparity_transform`, `spatial_filter`, `temporal_filter`, `hole_filling_filter`, `colorizer` |
| OpenCV 4 | `cv::eigen` (symmetric eigen-decomposition for the plane fit), `cv::selectROI`, `imshow/waitKey/namedWindow`, `cvtColor`, `resize`, `imwrite`, `putText`, `cv::mean` |
| C++17 std | `<filesystem>` (recursive recording search, canonical paths), `nth_element` (median), `<chrono>`, `<csignal>` |
| POSIX | `mkdir`, `stat` |

Internal: it mirrors settings from `Capture/programs/rs_marker/main.cpp` (preset, units, filter chain) but shares no code with it. It is not in `Capture/Makefile`. The build line is in the header comment (`:14-16`).

## 9. Observations

1. **Interactive ROI re-selection splits CSV and summary.** Pressing `r` clears `history` (`:1475`) but rows already written to `_metrics.csv` remain. `summary.csv` and `trial_info.txt` then describe only frames after the change, while the CSV holds both. Only the label "Frames (since last ROI change)" (`:1639`) hints at this.
2. **The recorded `.db3` contains frames from before any `selectROI` pause**, and the first frames are recorded before the loop starts. Reprocess deals with this by timestamp matching (by design), but `--all-frames` will include them.
3. **The depth-units race** (§7). Units are set after streaming begins. Commit `5354dca` works around the effect for reprocessing, but live trials may analyse a few early frames at the wrong scale. Setting the option before `pipe.start` (on the device from a `pipeline_profile` resolve) would avoid this.
4. **`plane_rms_pct`** divides by the on-axis distance $-d/n_z$, while Z-accuracy % divides per pixel by $Z_{\text{ref}}$. Both are reasonable, but they are different normalisations.
5. **`std_z_mm`** (population form, single-pass $\overline{Z^2}-\bar Z^2$) mixes tilt with noise for tilted targets, so it is not a noise metric. Temporal variance also uses the single-pass formula. With doubles and Z of about 10³ mm this is numerically acceptable but not ideal (Welford would be safer).
6. **Prebuilt binary committed to git** (`Capture/tools/Depth_Quality/depth_quality`, updated in every related commit). The 7.9 GB `dq_runs/` is correctly git-ignored.
7. **No build target.** The tool is built by hand from the header comment and is not in `Capture/Makefile`.
8. `patch_min_px` is not configurable. `--patch-smooth` is clamped silently.
9. **Sanity warnings** are evaluated only at exactly `history.size()==30`. If ground truth is fine there but goes wrong later (e.g. a mid-trial ROI change), nothing is reported.
10. **Error-map colours**: "blue=closer" is drawn with BGR `(255,k,k)`, which is blue in OpenCV, so it is consistent. No bug.
11. **Baseline sign**: `fabs` is applied (`:299, :305`), which is correct for disparity magnitude.
12. **Unclear**: whether the D456's `RS2_OPTION_STEREO_BASELINE` is preserved in playback. The fallback suggests the author saw it missing (`:1326` comment "IR2 is not in the recording").
