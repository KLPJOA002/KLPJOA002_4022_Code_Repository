# 02 — `rs_marker`: main capture tool (AprilTag detection, tilt/rotation sweep, recording, best-frame point cloud)

**Summary.** `rs_marker` is the project's main field and lab capture program. It runs the RealSense D456 with depth, colour, accelerometer and gyroscope streams. For each scan it:
- records the raw streams to a `.db3` file (ROS1 bag content despite the extension — **[EXPLICIT: `Processing/Filter_Sweep/decoder/rs_decode.cpp:24`, `Processing/Filter_Sweep/README.md:92,255`]**);
- aligns depth to the colour camera;
- detects AprilTag 36h11 fiducials in the colour image, using OpenCV's ArUco detector with lighting-robust tuning, an illumination-flattened second pass, and a geometric/photometric filter that keeps "candidate" squares whose ID failed to decode;
- logs every detection, with its depth, to a CSV;
- exports the frame with the fewest depth holes as a coloured `.ply` point cloud with the IMU sample captured alongside it;
- writes a full configuration dump (`scan_info.txt`) read back from the device.

The scans are organised as a **tilt sweep** (servo mount driven over USB serial by `mountSerial`, from $-\theta_{max}$ to $+\theta_{max}$ in fixed steps) nested inside a **manual rotation loop**: the operator rotates the mount, nominally by 45°, and presses Enter. A lot of the code deals with the reliability of repeated record start/stop in librealsense: one shared context, start retries, and a notification-callback workaround for a 2.57 segfault. Behaviour below comes from reading the source only. Nothing was compiled or run.

## Files covered

| Path | Lines | Role |
|---|---|---|
| `Capture/programs/rs_marker/main.cpp` | 1336 | Whole program: CLI, camera config, pipeline lifecycle, marker detection, logging, best-frame export, sweep loops |
| `Capture/programs/rs_marker/mountSerial.h` | 22 | Serial mount class declaration |
| `Capture/programs/rs_marker/mountSerial.cpp` | 118 | Arduino discovery, termios setup, `H`/`S`/`G` command protocol |

## Contents

- [A. rs_marker (main.cpp)](#a-rs_marker-captureprogramsrs_markermaincpp)
  - A.1 Purpose · A.2 Inputs/outputs (CLI, files, formats) · A.3 Walkthrough · A.4 Algorithms (camera/depth, alignment and point cloud, best frame, marker detection, candidates, illumination flattening, CLAHE) · A.5 Rationale · A.6 Parameters · A.7 Limitations · A.8 Dependencies · Observations
- [B. mountSerial (servo tilt mount)](#b-mountserial-servo-tilt-mount)
- [C. Development history (git log of main.cpp)](#c-development-history-of-maincpp)

---

## A. rs_marker (`Capture/programs/rs_marker/main.cpp`)

### A.1 Purpose
This program captures the per-viewpoint data that the downstream processing (cropping, multi-scan fusion, roughness evaluation) consumes:
- a dense coloured point cloud of a surface patch (sand or sea-ice analogue);
- the pixel and depth positions of AprilTag reference markers placed around the patch, which later serve as tie points for registering and fusing scans from different tilts and rotations;
- the camera intrinsics, options and IMU attitude sample for each scan;
- the raw recording, so any scan can be re-processed.

It is described as the program with "all the updated settings" **[EXPLICIT: `Capture/README.md:42`]**. Downstream, `Processing/Fuse` and `Processing/Crop` read `*_detections.csv` and `scan_info.txt` next to each PLY (e.g. `Processing/Fuse/src/fusion_session.cpp:597-607`).

### A.2 Inputs and outputs

#### A.2.1 Command-line options (`main.cpp:809-855`)

| Flag | Arg | Default | Effect |
|---|---|---|---|
| `-o`, `--output` | path | "" | Recording path. **Overwritten** in every iteration because the output dir is always non-empty (see Observations). In `-test` mode it would be the video path (`:770-774`) |
| `-D`, `--dir` | dir | "" | Parent directory for the session tree. Effectively required (see Observations) |
| `-d`, `--duration` | int s | 0 (off) | Stop each scan after N seconds (`:1040-1044`) |
| `-n`, `--frames` | int | 0 (off) | Stop each scan after N processed frames (`:1045`) |
| `-a`, `--angle` | int deg | 0 | Tilt half-range $\theta_{max}$, clamped to 60 (`:869-873`). Not in `print_usage` |
| `-s`, `--step` | int deg | 10 | Tilt step; ≤0 becomes 1 (`:888`). Not in `print_usage` |
| `-noTilt`, `--noTilt` | — | tilt on | Do not open the serial mount; forces `angle_limit=0` (`:875-878`). Not in `print_usage` |
| `-test`, `--test` | — | off | RGB-only. Detect `TARGET_IDS` with the simple detector, write annotated MP4 + CSV |
| `-id`, `--id` | — | off | RGB-only. Detect all tags and print/overlay each new unique ID |
| `-nTags`, `--nTags` | int | 4 | Expected tag count. If fewer are found, rerun detection on an illumination-flattened image (0 = never) |
| `-debugTags`, `--debugTags` | — | off | Overlay rejected candidates (red) and decoded/undecoded/filtered counts |
| `-r`, `--res` | `WxH` | 1280x720 | Depth and colour resolution (`sscanf "%dx%d"`, `:833-837`) |
| `--fps` | int | 30 | Depth and colour frame rate |
| `-u`, `--units` | float m | 0.0001 | Depth units (`RS2_OPTION_DEPTH_UNITS`) |
| `-p`, `--preset` | name | `high_density` | Visual preset: `default`, `high_density`, `high_accuracy`, `medium_density`, `hand`, `remove_ir_pattern` (`:109-118`) |
| `-b`, `--best-range` | `A:B`, `A:`, `:B` | all | Inclusive, 0-based frame window for best-frame selection (`parse_range`, `:302-312`) |
| `-h`, `--help` | — | — | Usage (`:23-47`) |

Validation: `-id` and `-test` are mutually exclusive (`:857-859`). `-b` must start before `-n` (`:861-864`). An unknown flag prints usage and exits 1 (`:854`).

#### A.2.2 Streams requested (`start_pipeline`, `main.cpp:139-182`)

| Mode | Streams |
|---|---|
| normal | Depth `Z16` W×H@fps; Colour `RGB8` W×H@fps; Accel `MOTION_XYZ32F`; Gyro `MOTION_XYZ32F`; record to file if the path is non-empty |
| `-test` / `-id` | Colour `RGB8` only, no recording (`:153-154`; the caller passes `""` and `test_mode \|\| id_mode`, `:943`) |

Units: depth Z16 counts × depth scale (default $10^{-4}$ m per count, i.e. 0.1 mm, maximum representable depth $65535\times10^{-4}=6.55$ m). Accelerometer in m/s², gyroscope in rad/s (librealsense `rs2_vector`).

#### A.2.3 Session folder layout (normal mode with `-D`)
The names are built at `main.cpp:881-885,911-914,929-934,946,957-961`:

```
<D>/
└── <TS>/                              TS = YYYYMMDD_HHMMSS (local time at program start)
    ├── <TS>_r0/                       rotation set 0
    │   ├── <TS>_r0_a0/                tilt 0°
    │   │   ├── <TS>_r0_a0.db3         raw recording (librealsense writer; ROS1 bag content, see §2)
    │   │   ├── <TS>_r0_a0_detections.csv
    │   │   ├── <TS>_r0_a0.ply         best frame point cloud (colour optical frame written as (x, −y, −z), metres)
    │   │   ├── imu_a0.csv             IMU sample at the best frame
    │   │   └── scan_info.txt          full configuration and results
    │   ├── <TS>_r0_a10/ …             +10°
    │   └── <TS>_r0_a-10/ …            −10° (negative sign kept in the name)
    └── <TS>_r45/ …                    next rotation; index advances by 45 (main.cpp:1324)
```

The rotation label is `rotation_index`, which starts at 0 and increases by 45 each time the operator continues. It is written to `scan_info.txt` as `rotation_deg` (`:1227`). It is a *nominal* angle: nothing measures the actual rotation. The on-disk form of the `.db3` is decided by librealsense and is not visible in this source; however, Filter_Sweep documents that these files are ROS1 bags despite the `.db3` extension **[EXPLICIT: `Processing/Filter_Sweep/decoder/rs_decode.cpp:24`, `Processing/Filter_Sweep/README.md:92`]**.

Note: the assignment brief mentions `trial_info.txt`. `rs_marker` writes **`scan_info.txt`**. `trial_info.txt` is written and read by the separate `Capture/tools/Depth_Quality/depth_quality.cpp` (e.g. `:1083-1169`), which is outside this file's scope.

#### A.2.4 `*_detections.csv` (`open_csv` `:265-271`, `log_detections` `:315-338`)
Header: `timestamp_ms,frame,marker_id,cx,cy,depth_m,c0x,c0y,c1x,c1y,c2x,c2y,c3x,c3y,decoded`.

| Column | Meaning / units |
|---|---|
| `timestamp_ms` | Host `steady_clock` ms since the start of **this scan** (`:1023,1140-1141`). Not a device or bag timestamp |
| `frame` | Processed-frame counter in this scan. Counts only loop iterations that reached the end, i.e. framesets that had depth (`:1133,1202`) |
| `marker_id` | Decoded 36h11 ID, or **−1** for a kept undecoded candidate |
| `cx, cy` | Tag centre in colour pixels (diagonal intersection, `quad_centre`, `:404-413`; corner average in `-test`) |
| `depth_m` | `get_distance` at the integer-truncated centre pixel of the aligned depth frame, clamped to the image (`:322-326`). 0 = hole or `-test` |
| `c0x…c3y` | Four corner pixels as OpenCV returns them (clockwise from the tag's own top-left for decoded tags; for rejected candidates the start corner is not tied to tag orientation) |
| `decoded` | 1 = ID decoded, 0 = candidate kept by the filter |

All numbers are printed with `std::fixed` and 3 decimals (`:327`). Rows are written only when at least one detection exists. The file is flushed every frame (`:337`).

#### A.2.5 `imu_a<tilt>.csv` (`write_imu_csv`, `:290-299`)
One header and **one** data row: `frame,timestamp_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z`, 6 decimals. The sample is the most recent accel and gyro motion frames seen before the best depth frame (`:1126-1129,1186-1187`).

#### A.2.6 `<…>.ply` (`main.cpp:1206-1214`)
`rs2::pointcloud::calculate(best_depth)` followed by `points.export_to_ply(ply_file, best_color)`. Because depth was aligned to colour first (`:1131`), the vertices are derived from the **colour camera's optical frame** (x right, y down, z forward, metres). Downstream code documents that `export_to_ply` writes them as **(x, −y, −z)**, i.e. y up and z towards the viewer; `Processing/Crop/crop_pointclouds.py:308` undoes this ("librealsense's export_to_ply() writes (x, -y, -z); undo that") **[EXPLICIT: that docstring; not verifiable from rs_marker itself]**. Texture comes from the same colour frame. The PLY layout (binary or ASCII, vertex filtering, faces) is decided by librealsense's `export_to_ply`. In the versions I am aware of it writes a binary little-endian PLY with per-vertex RGB, drops zero vertices and can include mesh faces, but that should be checked against the installed version.

#### A.2.7 `scan_info.txt` (`write_scan_info`, `:694-763`; results block `:1221-1250`)
This is a "key: value" text file, written **before** `pipe.stop()` so that options can be read from the live device (`:691-692`). Sections:
1. header: `written`, `command` (full command line);
2. per-scan results: `mode`, `session`, `rotation_deg`, `tilt_enabled`, `tilt_angle_deg`, `tilt_limit_deg`, `tilt_step_deg`, `duration_limit_s`, `frame_limit`, `frames_captured`, `scan_duration_s`, `record`, `csv`, `best_range`, `best_frames_in_range`, `best_frame`, `best_timestamp_ms`, `best_valid_pixels: v / N`, `ply`, `imu`, `video`;
3. `# requested`: resolution, preset, `depth_units_m`;
4. `# device`: every supported `rs2_camera_info` (name, serial, firmware, …);
5. `# streams (actual)`: per stream format and fps, and for video streams `stream.<Name>.intrinsics: fx fy ppx ppy model coeffs[5]`;
6. `# sensor options (actual)`: `depth_scale_m` and every readable option of every sensor (`dump_options`, `:678-689`, including value descriptions);
7. `# depth post-processing`: a fixed string `filter_chain: threshold > to_disparity > spatial > temporal > to_depth > hole_fill (decimation off)` plus the option values of those filter objects (but see Observations: the chain is **not applied**);
8. `# tag detection`: `expected_tags` and every `CAND_*`/`FLATTEN_*` constant.

`Processing/Fuse` reads the colour intrinsics from this file (`fusion_session.cpp:605`).

#### A.2.8 Console and windows
- A banner per scan (`:977-1016`).
- A live status line every 5 frames: frames, fps, mean valid depth, hole count (`print_stats`, `:232-245`).
- Windows "RGB + markers" and "Depth (filtered)" (colourised), each 1280×720 (`show_frames`, `:219-230`). `q` ends the current scan.

### A.3 Step-by-step walkthrough

1. **Parse arguments** (`:786-855`) and validate (`:857-864`). Build `cmdline` for `scan_info.txt` (`:866-867`).
2. **Tilt set-up.** Clamp `angle_limit` to 60 (`:869-873`). If tilt is enabled, construct `mountSerial` (opens the Arduino, §B), otherwise force `angle_limit = 0` (`:875-878`).
3. **Session root.** `run_ts = timestamp_str()` (`:881`, `strftime("%Y%m%d_%H%M%S")`, `:651-658`). `output_dir += "/" + run_ts` (`:884`). Normalise `step` (`:888`). Home the mount (`H0`, `:890-894`).
4. **Signals and context.** Install SIGINT/SIGTERM handlers that clear `g_running` (`:20-21,896-897`). Create **one** `PipelineCtx` (context + pipeline bound to it) for the whole run (`:93-98,898`).
5. **Rotation loop** (`:904-1326`). Reset `angle=0, side=+1, flipped=false`. Create `<root>/<TS>_r<rot>` (`:911-916`).
6. **Tilt loop**: $2\lfloor\theta_{max}/s\rfloor+1$ iterations (`:920-921`). Per iteration:
   1. Create `<TS>_r<rot>_a<angle·side>/` and set `output_file = …/<name>.db3` (`:929-934`).
   2. **`start_pipeline_retry`** (`:185-199`): up to 5 attempts with 500·k ms back-off. `start_pipeline` (`:139-182`) then:
      - takes `devices[0]` from the shared context;
      - applies the depth settings **before** streaming (preset → depth units → thermal compensation off, `apply_depth_settings` `:132-137` via `try_set` `:120-128`);
      - pins the config to that serial number;
      - enables the streams and recording, then `pipe.start(cfg)`;
      - registers a notifications callback on every sensor of the (recorder) device as the 2.57 crash workaround (`:165-173`);
      - reads back `get_depth_scale()` and re-applies the settings if it differs from the request by more than $10^{-9}$ (`:175-181`).
   3. Open the CSV (`:946-953`). Derive the PLY, IMU and info paths (`:956-963`). Open the video writer in `-test` (`:966-974`). Print the banner.
   4. Construct a fresh `DepthFilters` (no-op), `rs2::align(RS2_STREAM_COLOR)`, `rs2::colorizer`, `rs2::pointcloud` (`:1018-1021`) and reset the best-frame state (`:1029-1036`).
   5. **Frame loop** (`:1039-1203`). Stop on `-d` or `-n` (`:1040-1045`). `wait_for_frames()` (`:1049`). Then by mode:
      - **`-id`** (`:1051-1097`): colour → BGR → grey → `detect_all_markers(gray, expected_tags, …)`. Print `[NEW] ID` for unseen decoded IDs, draw the overlays, list unique IDs on screen.
      - **`-test`** (`:1100-1121`): colour → grey → `detect_markers(gray)` (TARGET_IDS only). Draw, write the MP4 frame, log CSV rows with `depth_m = 0`.
      - **normal** (`:1124-1200`):
        - update `last_accel`/`last_gyro` from any motion frames in the frameset (`:1126-1129`);
        - `align_to_color.process(frames)` (`:1131`), then skip if there is no depth (`:1133`);
        - `filters.process` (identity, `:75-85`);
        - convert colour to BGR and colourise depth for display (`:1136-1138`);
        - take the timestamp (`:1140-1141`);
        - grey conversion, then `detect_all_markers` (`:1144-1152`), optional debug overlays (`:1153-1156`), draw (`:1157-1158`);
        - log the CSV (`:1160-1161`) and show the windows (`:1164-1165`);
        - **hole count**: loop over every pixel with `get_distance` (`:1168-1175`);
        - **best-frame update** if `frame_count` is inside `[best_lo, best_hi]` and `valid > best_valid_pixels`, storing the depth frame, colour frame, IMU sample, index and timestamp (`:1177-1190`);
        - print stats every 5 frames (`:1192-1197`).
      - Increment `frame_count` (`:1202`).
   6. **Export the best frame** (`:1206-1214`): `keep()` both frames, `pc.map_to(color)`, `pc.calculate(depth)`, `export_to_ply`, `write_imu_csv`. Then release the held frames (`:1216-1217`).
   7. **Write `scan_info.txt`** while the device is still streaming (`:1221-1250`).
   8. `pipe.stop()`. Reset `ctx.profile` "destroys the recorder → .db3 closed before the next scan" (`:1252-1253`). Release the video writer.
   9. Print the per-scan summary (`:1258-1285`) and reset `frame_count` (`:1286`).
   10. **Advance the tilt** (`:1288-1300`): `angle += step`. Once `repetition ≥ ⌊θmax/s⌋` and not yet flipped, set `angle = step, side = −1`. Send `setCameraAngle(angle·side)`.
7. After the sweep, home the mount twice (`:1305-1311`). If `!g_running`, stop (`:1313`). Otherwise prompt "Rotate the mount to the next position, then press Enter … (or type q + Enter to finish)". `q`/`Q` ends the run. Anything else adds 45 to `rotation_index` (`:1315-1325`).
8. At the end, home twice and return 0 (`:1328-1335`).

**Tilt sequence.** With $N=\lfloor\theta_{max}/s\rfloor$ the scanned angles are $0, s, 2s, \dots, Ns, -s, -2s, \dots, -Ns$. That is $2N+1$ scans, confirmed by the 11-scan sequence 0…25, −5…−25 in `Capture/gdb_crash.log`. After the last scan the mount is commanded once more, to $-(N+1)s$, before homing (see Observations).

### A.4 Algorithms and methods

#### A.4.1 RealSense configuration: visual preset, depth units, thermal compensation
- **Ordering.** The preset is set first because "loading a preset rewrites many depth controls, so anything set before it would be overwritten" **[EXPLICIT: `main.cpp:130-131`]**. The options are set on the physical device *before* `pipe.start` "so the recording's header carries the right depth scale" **[EXPLICIT: `:143-144`]**, then verified after start (`:175-181`).
- **Depth units** $s$ (m/count). The reported depth is $z = s\cdot D$ with $D\in[0,65535]$. The default $s=10^{-4}$ (instead of the device default $10^{-3}$) gives 0.1 mm quantisation and 6.55 m range. The comment "change depth units from 0.001(1mm) default to 0.0001(0.1mm)" is **[EXPLICIT: commit `e9cd7a8`]**. Note that the physical depth resolution of an active stereo camera is governed by disparity resolution, not by the units:
  $$z=\frac{fB}{d}\quad\Rightarrow\quad \Delta z \approx \frac{z^2}{fB}\,\Delta d ,$$
  with focal length $f$ (px), baseline $B$ and sub-pixel disparity step $\Delta d$. At short range, 1 mm units can be coarser than $\Delta z$, which is presumably why finer units were chosen. **[INFERRED]** The code does not compute $\Delta z$.
- **Thermal compensation off** (`RS2_OPTION_THERMAL_COMPENSATION = 0`, `:136`). This D400 firmware feature periodically adjusts the depth calibration as the unit warms up. The commit that introduced it gives no reason beyond "disabled thermal compensation, rs_marker works mostly, still occasionally gives segmentation faults" **[EXPLICIT: commit `f4253ae`]**. It may have been disabled to stop background calibration updates and their control/notification traffic between repeated start/stop cycles, or to keep calibration fixed across the scans of one sweep. **[INFERRED]** Either way the trade-off is that depth can drift with temperature, so warm-up matters.
- **Visual presets** are firmware parameter bundles. High Density favours fill rate. High Accuracy rejects low-confidence depth (more holes, fewer outliers).

#### A.4.2 Depth-to-colour alignment and point cloud
`rs2::align(RS2_STREAM_COLOR)` reprojects each depth pixel into the colour image:
$$\mathbf P_d = z\,\pi_d^{-1}(u,v),\qquad \mathbf P_c = R_{d\to c}\mathbf P_d+\mathbf t_{d\to c},\qquad (u',v')=\pi_c(\mathbf P_c),$$
where $\pi$ is the pinhole projection with each stream's intrinsics and distortion model, and $R,\mathbf t$ the factory extrinsics. The output depth image has the colour stream's resolution and intrinsics. This is why tag pixel coordinates `cx,cy` can index the depth image directly (`:322-326`). `rs2::pointcloud::calculate` then deprojects every aligned pixel:
$$X = z\frac{u-c_x}{f_x},\quad Y = z\frac{v-c_y}{f_y},\quad Z=z$$
(plus distortion handling), using the colour intrinsics carried by the aligned frame. `map_to(color)` attaches texture coordinates.

#### A.4.3 Depth post-processing (currently disabled)
`DepthFilters` still declares decimation, threshold, disparity transforms, spatial, temporal and hole-filling filters (`:50-57`). Every `set_option` and `process` call is commented out, so `process()` returns its input (`:59-85`). Earlier versions applied, in this order:
- threshold (0.4–0.7 m in `fb0207f`, later 0.1–1.0 m in `c9f142d`);
- disparity transform;
- spatial filter ($\alpha=0.8,\ \delta=5$, magnitude 1);
- temporal filter ($\alpha$ variously 0.25 / 0.01 / 0.6 / 0.06, $\delta$ 10–100, persistence 1);
- disparity → depth;
- hole fill (mode 1).

They were all commented out in `00b2c4e`, the same commit that added Python code "to evaluate performance across different filters" **[EXPLICIT: commit message `00b2c4e`]**. It is therefore likely that raw (aligned) depth is now exported so filters can be compared offline. **[INFERRED]** For the theory of these filters see `01_capture_core.md` §1.4.

#### A.4.4 Best-frame selection ("fewest holes")
For frame $k$ in the window $[lo, hi]$, define the fill count $V_k = \#\{(x,y): z_k(x,y) > 0\}$. The exported frame is
$$k^* = \arg\max_{k\in[lo,hi]} V_k$$
(first maximum wins, since `>` is strict, `:1181`). The README describes it as "most valid depth pixels / fewest holes" **[EXPLICIT: `README.md:223-228`]**. The criterion only counts holes. It says nothing about noise, flying pixels or motion blur. The `-b` window lets early frames be skipped, e.g. `45:89` in `gdb_crash.log`, presumably while auto-exposure and the laser settle. **[INFERRED]**

#### A.4.5 Marker detection (`detect_all_markers`, `:547-620`) — OpenCV ArUco detector with an AprilTag 36h11 dictionary
The tags are **AprilTag 36h11** (Olson 2011; Wang & Olson 2016): 6×6 = 36 data bits and a minimum Hamming distance of 11 between codewords. Decoding uses **OpenCV's ArUco pipeline** (Garrido-Jurado et al. 2014) with that dictionary (`cv::aruco::DICT_APRILTAG_36h11`), *not* the AprilTag reference detector (`:571-572`). The pipeline steps are:
1. **Adaptive thresholding** at several window sizes $w\in\{5,13,21,29,37,45\}$ (min 5, max 45, step 8; `:555-557`). A pixel is foreground if $I(x,y) < \mu_{w\times w}(x,y) - C$ with $C=5$ (`:558`).
2. **Contour extraction and polygon approximation** (Douglas–Peucker, tolerance = `polygonalApproxAccuracyRate`·perimeter = 0.05, `:559`). Keep convex quadrilaterals. Candidates closer than `minMarkerDistanceRate`·perimeter = 0.02 are merged (`:569`).
3. **Perspective removal.** A homography maps the quad to a canonical grid with `perspectiveRemovePixelPerCell = 8` pixels per cell, and the outer 30 % of each cell is ignored (`:560-561`). Each cell is binarised with Otsu's threshold, attempted even on low-contrast cells (`minOtsuStdDev = 3`, `:562`).
4. **Border check.** Up to 50 % of border bits may be wrong (`maxErroneousBitsInBorderRate = 0.5`, `:563`).
5. **Identification.** The 36-bit word, in all four rotations, is matched to the dictionary by Hamming distance, using the full correction capacity (`errorCorrectionRate = 1.0`, `:564`). For 36h11 that is up to $\lfloor(11-1)/2\rfloor = 5$ bits.
6. **Sub-pixel corner refinement** (`CORNER_REFINE_SUBPIX`, `:565`), i.e. `cornerSubPix` gradient-based refinement.

The detector object is a function-local `static` built once (`:550-573`). `detectMarkers(img, corners, ids, rejected)` (`:581`) also returns the **rejected** quads, which feed the candidate logic below.

**Centre estimate** (`quad_centre`, `:404-413`). A projective map preserves incidence, so the image of the square's centre is the intersection of the image diagonals. With corners $c_0..c_3$, $\mathbf d_1=c_2-c_0$, $\mathbf d_2=c_3-c_1$, $\mathbf r=c_1-c_0$ and the 2-D cross product $a\times b=a_xb_y-a_yb_x$:
$$\mathbf p = c_0 + \frac{\mathbf r\times\mathbf d_2}{\mathbf d_1\times\mathbf d_2}\,\mathbf d_1 ,$$
used when $|\mathbf d_1\times\mathbf d_2| > 0.01\,|\mathbf d_1||\mathbf d_2|$ (diagonals not near-parallel). Otherwise the corner mean is used. The comment "perspective-correct, unlike the corner average" is **[EXPLICIT: `:402-403`]**.

#### A.4.6 Candidate detections with failed IDs (`:575-612`, `candidate_ok` `:476-490`, `border_contrast_ok` `:435-473`)
Purpose: keep reference points even when sun or shadow corrupts the bits **[EXPLICIT: commit `0969f45` "logs canditdate detections with a failed ID, to allow for reference point even when no ID is detected"; comment `:542-546`]**. The steps for each pass `run_pass(img)` are:
1. **Accept decoded tags.** Each is de-duplicated against existing results: centre distance $< 0.5\times$ mean side (`find_duplicate`, `:423-428`). A decoded tag replaces an earlier undecoded one at the same place ("upgrade", `:588`). Each decoded side length is recorded.
2. **Size gate.** If any tag decoded, the allowed side is $[0.5\,\tilde s, 1.5\,\tilde s]$ around the median decoded side $\tilde s$ (via `nth_element`). Otherwise it is $[15, 400]$ px (`:594-601`).
3. **For each rejected quad:**
   - reject it if it is not 4-sided (it then goes to the debug list);
   - skip it if it duplicates an existing result;
   - otherwise run `candidate_ok`: convexity, side ratio longest/shortest ≤ 2, size within the gate, then `border_contrast_ok`.
4. **`border_contrast_ok`**:
   - Warp the quad with a 4-point homography (`getPerspectiveTransform`) onto an 80×80 image of 10×10 cells of $C=8$ px, the quad occupying $[C,9C]^2$ (`:437-442`). Expected 36h11 layout: margin cell 0 white, cell 1 black border, cells 2–7 data, cell 8 black border, cell 9 white margin (`:430-431`).
   - For each side, take the mean $\bar m$ of a half-cell-thick band in the white margin and $\bar b$ in the black border, both spanning cells 2–7 so corners are skipped (`:444-448`). A side passes if
     $$\bar m-\bar b > 8 \quad\text{and}\quad \bar b < 0.75\,\bar m .$$
     At least 3 of the 4 sides must pass, so a shadow edge may spoil one (`:450-462`).
   - **Data-cell test.** Threshold $\tau=\tfrac18\sum_{k=1}^4(\bar m_k+\bar b_k)$ (midpoint between mean margin and mean border). Classify the central half of each of the 36 data cells and require ≥4 white **and** ≥4 black cells. This rejects solid dark blobs such as shells (`:464-472`, comment `:357-358`).
5. Survivors are stored with `id = −1`, `decoded = false` (`:611`) and drawn in orange with "?" (`:517-527`).

The ArUco `minMarkerDistanceRate` was lowered from the default 0.125 to 0.02 so that the white-margin outline and the black-border square, which are one cell apart, are not merged into one rejected quad that would then fail the border test **[EXPLICIT: `:566-569`]**.

#### A.4.7 Lighting resilience: illumination flattening second pass (`flatten_illumination`, `:494-510`)
If fewer than `expected_tags` results exist after the first pass, `run_pass` is repeated on a flattened image and the results are merged (`:615-617`). Flattening is **background division (flat-field correction)**: the grey image is divided by a heavily blurred copy of itself:
$$I'(x,y)=\operatorname{sat}_{[0,255]}\!\left(128\cdot\frac{I(x,y)}{(G_\sigma * I)(x,y)+1}\right),\qquad \sigma = 30\ \text{px}.$$
For efficiency the blur runs at ¼ resolution with $\sigma/4 = 7.5$ px and is bilinearly upsampled (`:503-506`) **[EXPLICIT: comment `:492-493`]**. The "+1" avoids division by zero (`:507`). Choosing $\sigma$ "≫ tag cell size, < shadow scale" **[EXPLICIT: `:362`]** removes slowly varying illumination (shadows, sun gradients) while keeping the tag's bit-level contrast. It is related in spirit to single-scale Retinex, but without the log.

#### A.4.8 CLAHE (Zuiderveld 1994) — present but inactive
Contrast-Limited Adaptive Histogram Equalisation equalises histograms per tile (8×8 tiles) with a clip limit (2.0) that limits noise amplification, and interpolates bilinearly between tiles. In the code:
- It is an alternative to background division in `flatten_illumination`, selected by `FLATTEN_USE_CLAHE` (`:363-364,497-501`), which is **false**.
- Commented-out always-on CLAHE calls remain in the `-id` and normal loops (`:1060-1061,1147-1148`). They were added in `e9cd7a8` ("added CLAHE filter"), with the comment "Applied CLAHE after gray conversion to reduce dynamic range of image" **[EXPLICIT]**, and commented out in `81094b3`, whose message gives no reason.

So **CLAHE is not used in any current code path**.

#### A.4.9 Test-mode detector (`detect_markers`, `:367-400`)
This uses the legacy `cv::aruco::detectMarkers` with adaptive-threshold windows 3…53 step 4, $C=7$ ("for better sunlight conditions" **[EXPLICIT: `:375`]**). It keeps only IDs in `TARGET_IDS = {0–7, 12–16, 93–95}` (`:344`) and uses the corner mean as the centre. It is only used in `-test` (`READMEdraft.md:192-193`).

### A.5 Why this approach (rationale)

| Decision | Rationale | Label |
|---|---|---|
| One `rs2::context` + pipeline reused for all scans | "building a new pipeline per scan piles up contexts on the same device (incl. the HID IMU) and segfaults after a few scans" | **[EXPLICIT: `main.cpp:89-92`, commit `00b2c4e`]** |
| User notifications callback on every sensor | Works around the librealsense 2.57 recorder leaving a dangling notification lambda | **[EXPLICIT: `:165-169`]**; crash evidence in `Capture/gdb_crash.log` (see `01_capture_core.md` §5.2) |
| Start retries with back-off | "avoid ocasional busy error" (`get_xu … Device or resource busy`, `Capture/Notes/error.txt:31`) | **[EXPLICIT: commit `22174f5`]** |
| Reset `profile` after stop | Destroys the recorder so the `.db3` is closed before the next scan | **[EXPLICIT: `:1253`]** |
| `.db3` instead of `.bag` | The README claims librealsense ≥2.57 needs rosbag2 `.db3`; Filter_Sweep later found the files are actually ROS1 bag content (`Filter_Sweep/README.md:255-256`). The author moved to 2.58 per the commit, but the gdb log shows 2.57 | **[EXPLICIT: commit `8f29459`; `README.md:8-11`]** |
| Preset before other options; options before start | Preset overwrites controls; recording header must have the right depth scale | **[EXPLICIT: `:130-131,143-144`]** |
| Depth units 0.1 mm | Finer quantisation than the 1 mm default | **[EXPLICIT: commit `e9cd7a8` comment]**; benefit for short-range roughness **[INFERRED]** |
| Thermal compensation off | Not stated beyond the commit | **[EXPLICIT change, commit `f4253ae`]**; reason **[INFERRED]** (see A.4.1) |
| Filters disabled | Same commit as the offline filter-evaluation scripts | **[INFERRED]** from `00b2c4e` |
| Align depth to colour | Tag pixels index depth directly; coloured PLY | **[INFERRED]**; `scan_info` notes "depth aligned to colour before filtering" (`:747`) |
| Detector tuning (windows, constant, Otsu, border tolerance, error correction, subpixel) | Each parameter's inline comment: small windows keep far tags, large windows bridge sun/shadow gradients, lower constant keeps washed-out borders, etc. | **[EXPLICIT: `:553-569`]** |
| Keep undecoded candidates | Reference points when IDs fail | **[EXPLICIT: commit `0969f45`, `:542-546`]**. That commit says "Untested" |
| Diagonal-intersection centre | Perspective-correct | **[EXPLICIT: `:402-403`]** |
| Flattened second pass rather than always-on | Only pay the cost when tags are missing; the blur runs at ¼ resolution because σ=30 at full resolution is slow | **[EXPLICIT: `:492-493`, help text `:34-35`]** |
| Tilt sweep and manual 45° rotations | Multiple viewpoints for fusion; rotation is manual ("rotation is changed manually") | **[EXPLICIT: `:1305-1306`; commit `fb0207f` "prompt user for rotation sweep consolidate files under one session folder"]**; 45° nominal step **[INFERRED]** (hard-coded `:1324`) |
| Store config with each scan | "Also now stores config settings with each scan" | **[EXPLICIT: commit `36abda2`]** |
| Best frame = fewest holes | Documented criterion | **[EXPLICIT: `README.md:223-228`]** |

**Alternatives not taken.**
- The native AprilTag 3 detector (better on small or blurred tags, slower).
- Pose estimation per tag (`solvePnP`/`estimatePoseSingleMarkers`) to get 6-DoF tag poses directly.
- Sampling depth over a neighbourhood or plane-fitting the tag's interior instead of one pixel.
- Selecting the best frame by a noise metric (e.g. temporal variance).
- Averaging IMU samples over the scan.
- Hardware-synchronised timestamps (frame metadata) instead of the host clock.

These are listed for comparison only. None is in the code.

### A.6 Parameters and tuning

| Name | Default | Location | Effect |
|---|---|---|---|
| `CamConfig.width/height` | 1280×720 | `:102-103`, `-r` | Depth and colour resolution (both identical) |
| `CamConfig.fps` | 30 | `:104`, `--fps` | Requested stream rate. Processing reaches ~11.7 fps at 1280×720 (`gdb_crash.log:94`) |
| `CamConfig.depth_units` | 1e-4 m | `:105`, `-u` | Depth quantisation and range |
| `CamConfig.preset` | `high_density` | `:106`, `-p` | Firmware depth preset |
| Thermal compensation | 0 (off) | `:136` | Fixed calibration |
| `expected_tags` | 4 | `:794`, `-nTags` | Second-pass trigger |
| `best_lo:best_hi` | 0:end | `:797-798`, `-b` | Best-frame window |
| `angle_limit` / `step` | 0 / 10 ° | `:803-804`, `-a`/`-s` | Sweep range and step (limit ≤60) |
| Rotation increment | 45 | `:1324` | Label only |
| `max_attempts` | 5 | `:186,943` | Pipeline start retries (sleep 500·k ms) |
| `adaptiveThreshWinSizeMin/Max/Step` | 5 / 45 / 8 | `:555-557` | Threshold scales |
| `adaptiveThreshConstant` | 5 | `:558` | Threshold offset $C$ |
| `polygonalApproxAccuracyRate` | 0.05 | `:559` | Polygon fit tolerance |
| `perspectiveRemovePixelPerCell` | 8 | `:560` | Bit sampling resolution |
| `perspectiveRemoveIgnoredMarginPerCell` | 0.3 | `:561` | Cell-edge margin ignored |
| `minOtsuStdDev` | 3.0 | `:562` | Allow Otsu on low-contrast cells |
| `maxErroneousBitsInBorderRate` | 0.5 | `:563` | Border bit tolerance |
| `errorCorrectionRate` | 1.0 | `:564` | Full 36h11 correction |
| `cornerRefinementMethod` | SUBPIX | `:565` | Corner accuracy |
| `minMarkerDistanceRate` | 0.02 | `:569` | Keeps margin and border quads separate |
| `CAND_MAX_SIDE_RATIO` | 2.0 | `:349` | Reject slivers |
| `CAND_MIN/MAX_SIDE_PX` | 15 / 400 | `:350-351` | Size gate when nothing decoded |
| `CAND_MEDIAN_SIDE_TOL` | 0.5 | `:352` | Size gate around median decoded side |
| `CAND_CELL_PX` | 8 | `:353` | Warp cell size |
| `CAND_MIN_CONTRAST` | 8 grey levels | `:354` | Margin − border |
| `CAND_MAX_BORDER_RATIO` | 0.75 | `:355` | Border/margin ratio |
| `CAND_MIN_SIDES_PASS` | 3 | `:356` | Sides that must pass |
| `CAND_MIN_DATA_CELLS` | 4 | `:357` | Minimum white and black data cells |
| `CAND_DEDUP_FRAC` | 0.5 | `:359` | Duplicate radius / side |
| `FLATTEN_SIGMA_PX` | 30 | `:362` | Background blur scale |
| `FLATTEN_USE_CLAHE` / `FLATTEN_CLAHE_CLIP` | false / 2.0 | `:363-364` | CLAHE alternative |
| Test-mode windows / constant | 3–53 step 4 / 7 | `:375-379` | `-test` detector only |
| `TARGET_IDS` | {0–7, 12–16, 93–95} | `:344` | `-test` filter |
| Video fps / codec | 5 fps / `mp4v` | `:779,970` | `-test` output |

### A.7 Assumptions, limitations, failure modes, edge cases
- **The first device wins** (`devices[0]`, `:148`). Multiple cameras are not supported.
- **Processing rate is below the stream rate.** The per-pixel `get_distance` loop over 921 600 pixels (`:1168-1175`), detection and display cut throughput (≈11.7 fps logged). The recorder writes at device rate, so CSV `frame` indices and host `timestamp_ms` **do not correspond one-to-one with recording frames**. The README claim that timestamps "match the bag timeline" (`README.md:233`) does not hold. **[INFERRED]**
- **Depth at the tag centre** is a single pixel. If the centre falls in a hole, `depth_m = 0`. Values are printed with 3 decimals (1 mm), discarding the 0.1 mm depth units (`:327`).
- **IMU** is one instantaneous sample, the latest before the best frame, with no averaging or time interpolation. It is noisy for levelling. `processing.md:66` suggests using it for plane adjustment.
- **Lighting.** All robustness is in software. Colour auto-exposure and other colour options are left at their defaults, and nothing is set on the RGB sensor.
- **Undecoded candidates** carry no ID, so downstream code must match them geometrically. False positives on textured sand are mitigated by the filters but not eliminated, and the feature is marked "Untested" (commit `0969f45`).
- **Second-pass trigger counts undecoded candidates** (`results.size()`, `:616`). Spurious candidates can therefore suppress the flattened pass.
- **The tilt angle is commanded, not measured.** No settle delay follows `setCameraAngle`. The program relies on the Arduino replying after (or while) the move completes (`mountSerial` read with a 2 s timeout). Whether the reply comes after motion ends cannot be determined: the firmware is not in the repo.
- **`wait_for_frames()`** throws after 5 s without frames. Nothing in the loop catches it, so the program terminates (no `try` around `:1049`).
- In the normal loop the colour frame is not null-checked before `to_bgr` (`:1136-1137`). If an aligned frameset lacked colour, the result would be undefined. **[INFERRED]**
- **Ctrl+C** ends the current scan loop and the sweep (outputs are still written for the interrupted scan). During the rotation prompt, `std::getline` keeps blocking because glibc `signal()` restarts the read, so Enter is needed. **[INFERRED]**

### A.8 Dependencies
- **librealsense2**:
  - `rs2::context`/`pipeline`/`config` (device discovery, stream negotiation, `enable_record_to_file` → recorder);
  - `sensor::set_option`/`get_option`/`get_supported_options`/`get_option_value_description` (control and introspection);
  - `depth_sensor::get_depth_scale`;
  - `set_notifications_callback` (async device events);
  - `rs2::align` (depth reprojection);
  - `rs2::colorizer` (depth → RGB visualisation);
  - `rs2::pointcloud` (`calculate` deprojects, `map_to` sets the texture);
  - `rs2::points::export_to_ply`;
  - `rs2::frame::keep` (keeps a frame beyond the pool's normal lifetime);
  - `rs2_camera_info_to_string`/`rs2_option_to_string`/`rs2_format_to_string`/`rs2_distortion_to_string`;
  - `video_stream_profile::get_intrinsics`.
- **OpenCV** (core, imgproc, highgui, videoio, objdetect/aruco):
  - `aruco::ArucoDetector`/`detectMarkers`, `getPredefinedDictionary(DICT_APRILTAG_36h11)`, `drawDetectedMarkers`;
  - `getPerspectiveTransform` + `warpPerspective` (homography rectification);
  - `GaussianBlur`, `resize`, `divide`, `createCLAHE`, `isContourConvex`, `cvtColor`, `mean`;
  - `VideoWriter` (MP4, `mp4v`);
  - `imshow`/`waitKey`/`namedWindow`.
- **POSIX**: `mkdir`, `strftime`/`localtime`, signals.
- **Internal**: `mountSerial` (§B).
- **Downstream consumers**: `Processing/Fuse` and `Processing/Crop` read the PLY, `_detections.csv` and `scan_info.txt`. The Python tools in `Capture/tools` (`collect_plys.py`, `marker_points.py`, …) were written for the older `.bag` layout.

### Observations (rs_marker)
1. **`-D` is effectively mandatory, and `-o`, `-test` and `-id` without it are broken.** `output_dir = output_dir + "/" + run_ts` (`:884`) makes the string non-empty even when `-D` was not given, producing `/<TS>`. `make_session_dir` then tries to create directories at filesystem root and calls `exit(1)` on failure (`:662-672`) unless run as root. The README's `./run rs_marker`, `./run rs_marker -o session.bag` and `-test`/`-id` invocations without `-D` therefore no longer work.
2. **`-o` is ignored.** It is overwritten with the generated `.db3` path every iteration (`:932`). The "`-o` and `-D` cannot be combined" check is commented out (`:926-928`).
3. **`-test` with `-D` passes a `.db3` filename to the MP4 writer.** `video_path(output_file)` (`:969`) gets the `.db3` path, so `VideoWriter` will probably fail to open, and the program then returns 1 (`:971-973`). The CSV in `-test` goes to `./detections.csv` (`:946`). **[INFERRED]**
4. **`scan_info.txt` misreports the depth processing.** It states `filter_chain: threshold > to_disparity > spatial > temporal > to_depth > hole_fill` and dumps those filters' option values (`:747-752`), but `DepthFilters::process` is an identity (`:75-85`). Anyone reading the file would conclude the PLY was filtered. The README also still says "fully post-processed" (`README.md:63,220`). The depth window is still titled "Depth (filtered)".
5. **The tilt mount overshoots after the last scan.** After the final iteration the code still increments and sends `setCameraAngle(-(N+1)·step)` (`:1290-1299`), e.g. −30° for `-a 25 -s 5` and −70° for `-a 60 -s 10`, which exceeds the 60° clamp before homing. The code does not show whether the firmware limits travel.
6. If $\theta_{max}$ is not a multiple of the step, it is silently truncated ($N=\lfloor\theta_{max}/s\rfloor$). A negative `-a` gives zero iterations.
7. `-a`, `-s` and `-noTilt` are missing from `print_usage` (`:23-47`). **Tilt is on by default**, so running without the Arduino fails in `mountSerial` (§B).
8. `ply_path(bag, angle, side)` ignores `angle` and `side` (`:275-280`). `-Wextra` would warn.
9. `draw_detections(gray, detections)` (`:1158`) draws on an image that is never used.
10. `detect_markers` rebuilds the dictionary on every call (`:370`).
11. `rotation_deg` is just a counter in 45 steps, not a measured angle (`:1227,1324`).
12. The summary prints `Bag :` for a `.db3` (`:1274`).
13. A large block of dead and commented code remains (filters, CLAHE, old start logic, sleep), which makes the active configuration harder to see.
14. In the 2 Oct crash run the banner showed `units 0.00 m` because of stream-format leakage. This is fixed in the current code (`std::defaultfloat`, `:1000`).
15. `frame_count` is reset at the end of each scan (`:1286`) and re-declared at the start (`:1024`), which is redundant.

---

## B. mountSerial (servo tilt mount)

### B.1 Purpose
This is a minimal driver for an Arduino-controlled tilt mount. It homes the camera and sets the tilt angle between scans. The firmware and mechanical design are **not in this repository**.

### B.2 Inputs and outputs

| Item | Detail |
|---|---|
| Port discovery | First (actually last) entry in `/dev/serial/by-id/` whose path contains `"Arduino"`, canonicalised (e.g. `/dev/ttyACM0`) (`mountSerial.cpp:28-42`) |
| Line settings | 115200 baud, 8N1, no flow control, raw (non-canonical, no echo, no output processing) (`:54-84`) |
| Read timeout | `VMIN=0, VTIME=20`, so `read` returns after the first bytes or after 2 s (`:80-81`) |
| `homeCamera` | sends `"H0"` (`:100-104`). Observed reply `H0000.0` (`Capture/Notes/error.txt:16`) |
| `setCameraAngle(a)` | sends `sprintf("S%05.1f", a)`, e.g. `S010.0`, `S-15.0` (`:106-111`). Observed replies `S0010.0`, `S0-15.0` (`gdb_crash.log`) |
| `getCameraAngle` | sends `"G"`, **always returns 0** (`:113-117`). Not called by `rs_marker` |
| Reply | copied into the caller's `read_buf` (size 30 in `main.cpp:801`) |

There is no terminator (newline) in commands. Angles are degrees, sign as used by the sweep (positive first, then negative). The physical axis and zero reference cannot be determined from source.

### B.3 Walkthrough
- **Constructor** (`:28-94`): iterate `/dev/serial/by-id`, open the port `O_RDWR`, clear `O_NONBLOCK` via `fcntl(F_SETFL, 0)`, then `tcgetattr`, modify the flags and `tcsetattr`. Errors are printed, but construction continues.
- **`serial_write`** (`:5-24`): `strcpy` the command to a 20-byte buffer, `write`, then a single `read` of up to 19 bytes, which is NUL-terminated because of the `-1` fix in commit `00b2c4e`. On a read error it closes the port and returns 1. Otherwise it copies the reply out.
- **Destructor**: `close(serial_port)`.

### B.4 Algorithms
A simple request/response ASCII protocol over termios. Nothing else.

### B.5 Rationale
- Raw termios settings follow the common Linux serial-programming recipe. The comments mark each flag ("most common"). **[EXPLICIT: comments `:60-76`]**
- Two `homeCamera` calls at the end of a sweep (`main.cpp:1308-1309,1331-1332`) are probably there to flush a stale reply or make sure the home command is acted on. **[INFERRED]** No explanation is given.
- The author's own "ToDo: Make this less shit" (`:32`), "ToDo: Clear buffer" (`:92`) and "ToDo: polling delay" (`:13`) mark it as provisional **[EXPLICIT]**.

### B.6 Parameters

| Name | Value | Location |
|---|---|---|
| Baud | 115200 | `:83-84` |
| `VTIME` | 20 (×0.1 s = 2 s; the comment wrongly says "10 deciseconds") | `:80` |
| Discovery key | `"Arduino"` in by-id name | `:35` |

### B.7 Limitations and failure modes
- **No Arduino or no `/dev/serial/by-id` directory.** `fs::directory_iterator` throws, or `linkPath` stays empty and `fs::canonical("")` throws `filesystem_error`. Neither is caught, so `std::terminate`. Users must pass `-noTilt` when no mount is attached.
- Only one `read` per command. A reply split across reads is truncated, and leftover bytes are read as the next reply (no buffer flush, `:92`).
- The `w_buf_len` and `r_buf_len` parameters are ignored. `strcpy` into fixed buffers has no bounds checks.
- If `open` fails, the code still calls `fcntl`/`tcsetattr` on −1, and later writes and reads fail silently. Return codes are ignored by `rs_marker`.
- `tty` is a public member, and `getCameraAngle` is a stub.

### B.8 Dependencies
POSIX termios/fcntl/unistd, C++17 `<filesystem>`.

### Observations (mountSerial)
- Missing include guard in `mountSerial.h`.
- With several matching devices, the *last* match is chosen, not the first.
- The read timeout comment and value disagree.

---

## C. Development history of main.cpp

From `git log --follow Capture/programs/rs_marker/main.cpp`, oldest first:

| Commit | Date | Change (message, plus what the diff shows) |
|---|---|---|
| `8e07093` | 2026-08-11 | "Added Existing RS_Code, updated to match current open_CV version": inherited code base |
| `1c1f7db` | 08-12 | Filter settings added (spatial α 0.8 δ 5 mag 2; temporal α 0.25 δ 10; hole fill 1); depth units 0.0001; decimation disabled |
| `1230363` | 08-12 | Decimation re-enabled, temporal α 0.01 δ 80; `poll_for_frames` → `wait_for_frames` |
| `e9cd7a8` | 08-17 | Test-mode detector tuned "for better sunlight conditions"; CLAHE added |
| `09dce55` | 08-18 | Comments for understanding |
| `c56364f` | 08-21 | Serial servo link added; "fails when storing multiple scans" |
| `1324632`, `81094b3` | 08-21/24 | One folder per angle; CLAHE commented out |
| `e459457`, `dc06e2d` | 08-24 | Post-scan sleep 500 ms → 1000 ms |
| `5855c99` | 08-27 | Moved under `Capture/` |
| `5b9a5ea` | 08-27 | `-noTilt`; per-angle session folders |
| `22174f5` | 08-28 | Start retries; full filter chain enabled; angle clamp fixed (`angle_limit = 60`) |
| `f4253ae` | 08-28 | Thermal compensation disabled; "still occasionally gives segmentation faults" |
| `8f29459` | 09-09 | librealsense 2.58, `.db3` output, hole count shown |
| `fb0207f` | 09-09 | Threshold filter 0.4–0.7 m; rotation prompt loop; session/rotation/angle tree; sleep removed |
| `c9f142d` | 09-21 | Threshold widened to 0.1–1.0 m |
| `59b3365` | 09-25 | Resizable ID window |
| `0969f45` | 09-25 | Lighting-robust detector, candidate squares with `decoded` column, flattened second pass, `-nTags`, `-debugTags` ("Untested") |
| `36abda2` | 10-02 | CLI for resolution, fps, units, preset, best-range; `scan_info.txt`; default 848×480 → 1280×720 |
| `00b2c4e` | 10-06 | Shared context, notification-callback workaround, recorder reset; **all depth filters disabled**; `gdb_crash.log` committed |
