# 05 — Capture-side Python Utilities

**Summary.** `Capture/tools/` and `Capture/More_Tools/` hold a set of small Python scripts that post-process the output of the `rs_marker` capture program. `collect_plys.py` flattens a recording tree into per-type folders. `marker_points.py` re-samples depth from the recordings at AprilTag centres and deprojects them to 3-D marker positions. `summarize_markers.py` produces a text report of a detections CSV. `tabulate_data.py` builds one metadata row per recording by reading rosbag headers. `plot_imu.py` and `plot_gyro_outliers.py` extract, plot and screen the IMU data. `play_video.py` plays the `-test` mode MP4. `view_point_cloud.py` is an Open3D GUI PLY viewer, and a near-identical copy lives in `Processing/`. `Capture/More_Tools/` contains two broken stub files. `Capture/tools/PLY` is an empty file. The most important cross-cutting finding is that **these tools were written for an older `rs_marker` output layout** (`<session>/<ts>/<ts>.bag` + `imu.csv`), which is described in `Capture/README.md`. The current `rs_marker` writes `.db3` recordings (ROS1 bag content under a `.db3` name, per `Processing/Filter_Sweep/README.md:92`), per-angle `imu_a<angle>.csv` files, and a rotation/angle folder hierarchy. As a result `collect_plys.py`, `marker_points.py`, `plot_imu.py` and `tabulate_data.py` (all of which glob `*.bag` or `imu.csv`) will not find current data without changes. Everything here was derived by reading source. Nothing was run.

## Files covered

| Path | Lines | Role |
|---|---|---|
| `Capture/tools/collect_plys.py` | 165 | Gather `.bag/.ply/_detections.csv/imu.csv` from a recording tree into flat per-type folders |
| `Capture/tools/marker_points.py` | 223 | 3-D AprilTag-centre positions from recordings plus detection CSVs (CSV or PLY) |
| `Capture/tools/summarize_markers.py` | 184 | Text report on one detections CSV (per-marker stats, best frames, timeline) |
| `Capture/tools/tabulate_data.py` | 188 | One-row-per-recording metadata CSV from ROS1 bag headers plus the collected files |
| `Capture/tools/plot_imu.py` | 178 | Per-recording IMU PNG plot and raw sample CSV |
| `Capture/tools/plot_gyro_outliers.py` | 162 | Dataset-wide gyro-magnitude outlier screening (modified z-score) |
| `Capture/tools/play_video.py` | 51 | Minimal OpenCV MP4 player with pause and frame step |
| `Capture/tools/view_point_cloud.py` | 325 | Open3D GUI PLY viewer (point cloud/mesh, colouring, Z crop) |
| `Processing/view_point_cloud.py` | 326 | Copy of the above with a Matplotlib ≥3.9 fix (commit `c9f142d`) |
| `Capture/More_Tools/Data_Processing.py` | 2 (wc: 1) | Stub: prints a greeting, then calls an undefined `readPlyGroup` |
| `Capture/More_Tools/readPlyGroup.py` | 2 | Stub, **syntax error** (missing colon) |
| `Capture/tools/PLY` | 0 | Empty file |
| `Capture/README.md` | 512 | Context: describes the (older) output layout and these tools |

Git history: all of `Capture/tools/*.py` and `More_Tools` arrived in `8e07093` ("Added Existing RS_Code…", 2026-08-11), which imported existing code. They were only moved in `5855c99` (restructure). `view_point_cloud.py` and `PLY` came in `e9cd7a8`. The `Processing/` viewer was later changed in `6de8e04` and `c9f142d`. None of the Capture Python tools has been modified since the move to `.db3` (`8f29459`, 2026-09-09).

## Contents
- A. Context: the rs_marker output layout (old vs current)
- B. `collect_plys.py`
- C. `marker_points.py`
- D. `summarize_markers.py`
- E. `tabulate_data.py`
- F. `plot_imu.py`
- G. `plot_gyro_outliers.py`
- H. `play_video.py`
- I. `view_point_cloud.py` (Capture vs Processing)
- J. `Capture/More_Tools/*` and `Capture/tools/PLY`

---

## A. Context: rs_marker output layout

The layout documented in `Capture/README.md:56-64, :211-221`, which is the one these tools assume:

```
<D>/<YYYYMMDD_HHMMSS>/<YYYYMMDD_HHMMSS>.bag
                     /<YYYYMMDD_HHMMSS>_detections.csv   timestamp_ms,frame,marker_id,cx,cy,depth_m,c0x..c3y
                     /<YYYYMMDD_HHMMSS>.ply              best (fewest holes) post-processed frame
                     /imu.csv
```

Current `rs_marker` (read only for cross-reference, `Capture/programs/rs_marker/main.cpp`):
- The recording is `<session_dir>/<session_name>.db3`, with `session_name = <run_ts>_r<k>_a<angle>` inside a rotation folder `<run_ts>_r<k>` (`main.cpp:910-933`). `.db3` arrived in commit `8f29459` ("Updated librealsense version to 2.58, changed output file to db3").
- The IMU file is `imu_a<angle>.csv`, and there is also a `scan_info.txt` (`main.cpp:958-961`).
- The detections CSV gained a 15th column `decoded` (`main.cpp:268-269`). Tools that use `csv.DictReader` ignore it.
- The `frame` column is the program's **loop counter** `frame_count` (starting at 0 for each scan), not the hardware frame number (`main.cpp:1161, :1202`).
- The PLY is built from depth that has been **aligned to colour** (`align_to_color.process`, `main.cpp:1136`), then filtered, then `pc.calculate(best_depth)` + `export_to_ply` (`main.cpp:1209-1212`). The cloud's points are therefore in the **colour** camera's optical frame **[INFERRED from rs_marker; outside this file's scope]**.

`Capture/README.md:8-11, :136-138` still tells users to install librealsense 2.56 for `.bag` support. That conflicts with the `.db3`/2.58 move, so the README is stale.

---

## B. `collect_plys.py`

### B.1 Purpose
Walks a top-level folder of rs_marker recordings and copies, symlinks or moves each output type into flat per-type folders, ready for batch processing (`collect_plys.py:1-43`, README `:253-316`).

### B.2 Inputs and outputs

| Arg | Meaning | Default |
|---|---|---|
| `src` (positional) | Recording tree | required |
| `dst` (optional) | Output root | none → **default mode** |
| `--types {bag,ply,detections,imu}…` | Types to collect (explicit `dst` only) | all four |
| `--mode {copy,symlink,move}` | Placement | `copy` |
| `--prefix-parent` | Always prefix `<parent>_` | off |
| `--flat` | Write into `dst` directly (explicit `dst` only) | off |

`CATEGORIES` (`:52-57`): `bag → *.bag`, `ply → *.ply`, `detections → *_detections.csv`, `imu → imu.csv` (always prefixed).

Default mode output: `<src>_collected/{ply,detections,imu}/` and `<src>_raw/*.bag` (flat), created next to `src` (`:134-153`).

### B.3 Walkthrough
- `main` (`:128-161`) validates `src`. Without `dst` it forbids `--types/--flat` (`:135-137`), derives the sibling folder names (with a trailing-slash fix at `:139`), and calls `run_collection` twice.
- `run_collection` (`:110-125`) runs `src.rglob(pattern)` for each type, sorts the results, and calls `collect`.
- `collect` (`:81-107`) groups files by basename. A file is renamed to `<parent>_<name>` if it is forced (imu), if `--prefix-parent` is set, or if its basename occurs more than once. An existing target is skipped with a warning. The file is then placed with `shutil.copy2` (which keeps mtime), `symlink_to(resolve())`, or `shutil.move`.

### B.4 Algorithms
A simple recursive glob, plus collision avoidance by basename counting.

### B.5 Why
- Recordings are "buried in its own `<YYYYMMDD_HHMMSS>/` session folder", and flattening them is "handy before batch-processing a day's worth of recordings or archiving just the point clouds" (README `:255-260`) **[EXPLICIT]**.
- `imu.csv` "is named identically in every recording folder, so it is always renamed" (`:34-38`) **[EXPLICIT]**.
- Bags go into a separate flat `_raw` folder because `marker_points.py`, `plot_imu.py` and `tabulate_data.py` expect a flat folder of bags **[INFERRED from those scripts' arguments]**.

### B.6 Parameters
Only the CLI flags above. The glob patterns are hard-coded in `CATEGORIES`.

### B.7 Limitations and edge cases
- **Does not match current data**: `*.bag` misses `.db3`, and `imu.csv` misses `imu_a<angle>.csv` (§A).
- The collision check works on basenames within one type only. `<parent>_<name>` can still collide if two parents share a name at different depths. The second file is then silently skipped (`:94-96`).
- If `dst` lies inside `src`, a re-run will `rglob` the earlier output too, and those duplicates get prefixed or skipped **[INFERRED]**.
- With `--mode move`, a skipped file stays in place, so a partial move is possible.

### B.8 Dependencies
Standard library only (`argparse`, `shutil`, `pathlib`, `collections.defaultdict`).

### Observations
- The exit code is non-zero only when *nothing* matched. Partial skips only print warnings.
- The name says "plys" but the script collects four types (a historical name).

---

## C. `marker_points.py`

### C.1 Purpose
Turns 2-D AprilTag detections (pixel centres in the colour image) into 3-D marker positions. Depth is re-sampled from the recording. The aim is to overlay or QC them against the reconstructed `.ply` clouds or to use them in later processing (`:1-46`).

### C.2 Inputs and outputs

| Item | Format |
|---|---|
| `raw_dir` | Flat folder of `<stem>.bag` (from `collect_plys` default mode) |
| `collected_dir/detections/<stem>_detections.csv` | Header required. Uses `frame, marker_id, cx, cy` only. `depth_m` is ignored (`:35-37`) |
| `--format csv` (default) | `collected_dir/marker_csv/<stem>.csv`: `marker_id,x,y,z`, the per-axis median per marker (metres) |
| `--format ply` | `collected_dir/marker_ply/<stem>.ply`: one point per detection, coloured by `marker_id % 8` from an 8-colour palette (`:87-90`) |

The output frame is meant to be the "same frame as the reconstructed point clouds": the depth optical frame with **y and z negated** (`:139-142`). Units are metres.

### C.3 Walkthrough
1. `main` (`:171-219`) lists `*_detections.csv`, derives `stem` by stripping `_detections.csv`, and pairs each with `raw_dir/<stem>.bag`. Missing bags and empty results are counted and skipped.
2. `load_detections` (`:59-69`) builds `frame → [(marker_id, cx, cy)]`.
3. `extract_markers` (`:93-148`):
   - Opens the bag for playback (`enable_device_from_file`, `repeat_playback=False`, `set_real_time(False)`).
   - Gets the depth and colour stream profiles and `color_to_depth = color_stream.get_extrinsics_to(depth_stream)` (`:105-107`).
   - `align = rs.align(rs.stream.color)` (`:109`).
   - Loop with `try_wait_for_frames(2000)` until timeout. Read `frame_num = get_color_frame().get_frame_number()` (`:121`) and look up detections. If there are any, align the frameset, take `color_intrin` from the aligned colour frame, and for each detection:
     - `median_depth` (`:72-83`): median of the non-zero `get_distance(x,y)` values in a (2·2+1)² = 5×5 window around the rounded centre.
     - `rs2_deproject_pixel_to_point(color_intrin, [cx,cy], d)` → point in the colour frame.
     - `rs2_transform_point_to_point(color_to_depth, p)` → point in the depth frame.
     - Negate y and z (`:142`).
4. The writers produce the CSV (`write_marker_csv :159-168`, `np.median(..., axis=0)` per marker) or the PLY (`write_marker_ply :151-156`, Open3D).

### C.4 Algorithms and geometry

**Depth-to-colour alignment.** `rs.align(color)` reprojects every depth pixel into the colour image. It deprojects with the depth intrinsics, transforms with the depth→colour extrinsics, and projects with the colour intrinsics. The result is that pixel $(u,v)$ in the aligned depth frame holds the depth (Z in the colour frame) of the surface seen at colour pixel $(u,v)$. Sampling the aligned depth at the tag centre $(c_x, c_y)$ is therefore valid.

**Robust depth sample.** $d = \operatorname{median}\{D(u+i,v+j)>0 : |i|,|j|\le 2\}$ gives robustness to holes and speckle. The rationale is that "single-pixel reads at the marker center frequently land on holes" (`:35-37`) **[EXPLICIT]**.

**Pinhole deprojection** (`rs2_deproject_pixel_to_point`, Brown–Conrady/none distortion handled internally). With colour intrinsics $(f_x,f_y,c_x^0,c_y^0)$:

$$\mathbf P_c = d\begin{pmatrix}(u-c_x^0)/f_x\\(v-c_y^0)/f_y\\1\end{pmatrix}$$

**Rigid transform to the depth frame** (`rs2_transform_point_to_point`): $\mathbf P_d = R\,\mathbf P_c + \mathbf t$, using the factory colour→depth extrinsics (rotation $R$, column-major in librealsense, and translation $\mathbf t$ in metres).

**Axis flip**: $(x,\,-y,\,-z)$ converts from the RealSense optical convention (x right, y down, z forward) to the convention that the comment says `export_to_ply` writes (y up, z toward the viewer, OpenGL-like). That claim is **[EXPLICIT: comment `:139-141`, citing librealsense/src/points.cpp]**. It was not verified against the librealsense source in this audit.

**Aggregation**: a per-axis median over all detections of one marker in the recording (a component-wise median, not a geometric median). This is robust to "the rare stray bad detection" (`:27`) **[EXPLICIT]**.

### C.5 Why this approach
- The depth is re-sampled from the recording rather than using the CSV `depth_m` because of holes (above) **[EXPLICIT]**.
- Colour → depth transform: "Detections were made on the COLOR image, but the reconstructed .ply lives in the DEPTH optical frame" (`:39-42`) **[EXPLICIT]**. See Observation 2 for why this premise appears false for the current rs_marker.
- An alternative would be to deproject the four corners and fit a tag pose (PnP, or the AprilTag pose estimate). The centre-only approach is simpler but gives no orientation **[INFERRED]**.

### C.6 Parameters

| Name | Default | Location | Effect |
|---|---|---|---|
| `win` | 2 (5×5) | `:72` | Depth-median window |
| playback timeout | 2000 ms | `:116` | End-of-file detection |
| `PALETTE` | 8 colours | `:87-90` | PLY colouring, cycles with `id % 8` |
| `--format` | `csv` | `:176` | Aggregated CSV vs raw PLY |

### C.7 Assumptions and limitations
- The tag centre lies on a surface with valid depth in a 5×5 neighbourhood. Otherwise the detection is skipped and counted.
- One median per marker assumes the camera and markers are static during the recording.
- Requires a colour stream in the recording (`get_stream(rs.stream.color)` throws otherwise). There is no try/except, so one bad bag aborts the batch.

### C.8 Dependencies
`pyrealsense2` (playback, `align`, `rs2_deproject_pixel_to_point`, `rs2_transform_point_to_point`, `get_extrinsics_to`), `numpy` (median), `open3d` (`PointCloud`, `write_point_cloud`), standard library `csv`.

### Observations
1. **Frame-matching key mismatch (likely bug).** Detections are keyed by the CSV `frame`, which rs_marker writes as its loop counter `frame_count` starting at 0 (`rs_marker/main.cpp:1161, :1202`). The lookup uses `get_color_frame().get_frame_number()` (`marker_points.py:121`), the hardware frame counter. That counter generally does not start at 0 and is not equal to the loop counter, so detections will be attached to the wrong frames or to none. The loop counter is also not incremented for skipped framesets (`continue` before `++frame_count`), so even an offset correction would not be exact. This cannot be confirmed without data, and the older imported rs_marker may have behaved differently.
2. **Frame mismatch with the PLY.** The current rs_marker builds the PLY from depth aligned to colour (`rs_marker/main.cpp:1136`), so the PLY points are in the colour optical frame. `marker_points.py` transforms into the depth frame, which would shift markers relative to the cloud by the colour–depth extrinsic translation (centimetre scale; the exact value is not in source) **[INFERRED]**.
3. **The output frame is not the frame `Processing/` uses.** The negated y/z frame is specific to `export_to_ply`. Any consumer must use the same convention.
4. `.bag` only (`:195`). Current `.db3` data is not paired.
5. When the IMU streams are in the recording, the syncer can deliver the same colour frame in several framesets (the same issue `depth_quality.cpp:1440-1441` guards against). Detections can then be appended more than once. The median is robust to this, but the PLY point counts are inflated **[INFERRED]**.

---

## D. `summarize_markers.py`

### D.1 Purpose
A console report on one rs_marker detections CSV: which markers were seen, how often, the depth range, how many were visible at once, the "best" frames, and an ASCII timeline (`:1-8`, README `:243-251`).

### D.2 Inputs and outputs
Input: a CSV with `timestamp_ms, frame, marker_id, depth_m` (other columns ignored). Args: `--top` (10) and `--timeline-width` (60). Output goes to stdout only.

### D.3 Walkthrough
- `load` (`:28-39`): converts the types and builds `frames[frame][marker_id] = row`. A duplicate marker within a frame overwrites the earlier row.
- `section_overview` (`:50-67`): total detections, frames with detections, duration = **max** `timestamp_ms` (time since program start, not since the first detection), the IDs, and the depth range over `depth_m>0`.
- `section_per_marker` (`:69-89`): per ID, the frame count, % of *frames-with-detections*, and the mean/min/max of valid depth.
- `section_marker_counts` (`:91-105`): a histogram of the number of markers per frame, as a bar chart.
- `section_best_frames` (`:107-130`): sorted by (−count, timestamp of the first row in that frame). Prints the top N, stars frames with the maximum count, and gives a "Recommended frame".
- `section_timeline` (`:132-156`): maps frames into `width` bins, $\text{slot}=\lfloor (f-f_{min})/(f_{max}-f_{min})\cdot(\text{width}-1)\rfloor$, with one character position per marker ID.

### D.4 Algorithms and geometry
No geometry. Only counting, histogramming and binning. The `depth_m` statistics are raw rs_marker values (the filtered depth at the integer centre pixel, clamped to the image, `rs_marker/main.cpp:322-325`). No deprojection is performed.

### D.5 Why
The README says the summary "reports which frames had the most markers visible simultaneously and recommends the best frame to use" (`README:250-251`) **[EXPLICIT]**. This is probably for choosing a registration frame with the most tag correspondences **[INFERRED]**.

### D.6 Parameters
`--top`, `--timeline-width`, label width `W=44` (`:43`), bar scale of 30 characters (`:102`).

### D.7 Limitations
- "% frames" uses frames *with detections* as the denominator, not total frames, so it overstates visibility.
- If `rs_marker` was run with `-test` (no depth), `depth_m=0` everywhere and a warning is printed (`:63-64`).
- When two frames' timeline slots coincide, the IDs are merged (OR), so the timeline is lossy.

### D.8 Dependencies
Standard library only.

### Observations
- `section_marker_counts` returns `max_seen`, but `main` recomputes it (`:177-178`). The duplication is harmless.
- It parses `frame` with `int(row["frame"])`, whereas `marker_points.py` uses `int(float(...))`, so this script is less tolerant of `"12.000"`.
- There is no awareness of the `decoded` column (current rs_marker), so undecoded or filtered detections are counted the same as decoded ones **[INFERRED: meaning of `decoded` not analysed here]**.

---

## E. `tabulate_data.py`

### E.1 Purpose
Builds one metadata row per recording (timing, device, per-stream frame counts, FPS, resolution, IMU counts, and which derived files exist) by reading the bag files directly with the pure-Python `rosbags` library. No pyrealsense2 or ROS is needed (`:1-20`). The README notes this makes aborted recordings visible (e.g. `Color_Frames=NaN`) (`README:423-426`) **[EXPLICIT]**.

### E.2 Inputs and outputs
Input: `src`. It expects the siblings `<src>_raw/*.bag` and `<src>_collected/{ply,detections,imu}/`. Output: `--out`, defaulting to `<src>_metadata.csv`. Columns (`:33-41`):

| Column | Source |
|---|---|
| `Name, Size` | filename, `stat().st_size` (bytes) |
| `Start, End, Duration` | `bag.start_time/end_time` (ns → s, 6 dp) |
| `Device, Serial, Firmware` | `/device_0/info` KeyValue messages (`Name`, `Serial Number`, `Firmware Version`) |
| `Depth_Frames, Color_Frames` | `msgcount` of topics ending `Depth_0/image/data` and `Color_0/image/data` |
| `Depth_FPS, Color_FPS` | count / duration |
| `Depth_Res, Color_Res` | first `…/info/camera_info` message `width x height` |
| `Accel_Samples, Gyro_Samples` | `msgcount` of `Accel_0/imu/data`, `Gyro_0/imu/data` |
| `marker_csv, ply_file, imu_csv` | whether `<stem>_detections.csv`, `<stem>.ply`, `<stem>_imu.csv` exist |
| `num_detections, num_unique_markers` | rows / unique `marker_id` in the detections CSV |

### E.3 Walkthrough
`main` (`:132-184`) → for each sorted `*.bag`, `read_bag_row` (`:74-109`, with the `rosbags.rosbag1.Reader` context manager, a `topic→connection` map, and `find_topic` suffix matching `:44-45`), wrapped in try/except so an unreadable bag gives a row of NaNs. It then cross-references the collected files and writes the CSV with `DictWriter`.

### E.4 Algorithms
It parses the librealsense ROS1 bag topic schema (`/device_0/sensor_<i>/<Stream>_<idx>/image/data`, etc.) by topic suffix. Messages are deserialised with the Noetic typestore (`get_typestore(Stores.ROS1_NOETIC)`, `:30`). FPS is the mean rate $N/T$.

### E.5 Why
Using `rosbags` avoids a librealsense/ROS dependency, and it follows an existing project script: "same approach as python-3DRecon/realsense/metadata_gen.py" (`:8-9`) **[EXPLICIT]**. That file exists outside this repo at `/home/joab/Documents/4022/3D-SeaIce/python-3DRecon/realsense/metadata_gen.py`, which suggests the tools were inherited from an earlier "3D-SeaIce" codebase **[INFERRED]**.

### E.6 Parameters
`--out` only. The topic suffixes are hard-coded.

### E.7 Limitations
- **ROS1 only** (`rosbags.rosbag1.Reader`, `*.bag`). It does not glob `.db3` files. Per `Processing/Filter_Sweep/README.md:92,255` those are ROS1 bag content, so the existing `rosbag1` reader may work if pointed at them (an extension change or symlink, as Filter_Sweep does), but the folder layout has also changed **[INFERRED]**.
- $N/T$ slightly underestimates the frame rate (it should be $(N-1)/T$), and $T$ is the span of *all* topics, not just that stream.
- Topic suffix matching takes the first match, so multiple sensors or streams with the same suffix are ambiguous.
- It assumes the `imu.csv` rename convention from `collect_plys` (`<recording_folder>_imu.csv`), which matches `<stem>` only in the old layout.

### E.8 Dependencies
`rosbags` (`Reader`, `connections`, `messages`, `deserialize_ros1`), standard library `csv`.

### Observations
- The column `marker_csv` actually refers to the *detections* CSV, not to `marker_points.py`'s `marker_csv/` folder. The name is confusing.
- `Size` is in bytes, but this is not documented.

---

## F. `plot_imu.py`

### F.1 Purpose
For every recording in a folder, extract all accelerometer and gyroscope samples, plot them over time (PNG), and dump them to CSV (`:1-20`).

### F.2 Inputs and outputs
Input: `bag_dir/*.bag`. Outputs: `--graphs-dir` (default `<bag_dir>/imu_graphs/<stem>_imu.png`, 150 dpi) and `--csv-dir` (default `<bag_dir>/imu_total/<stem>_imu.csv`, columns `timestamp_ms,stream,x,y,z`, units m/s² and rad/s, timestamps in device ms with 3 dp, values with 6 dp, sorted by time).

### F.3 Walkthrough
`extract_imu` (`:50-85`): non-real-time playback, then iterate every frame in each frameset, keep the motion frames, and sort them into accel and gyro by `stream_type()` using `get_timestamp()`. Each stream is sorted and returned as `(ts, xyz)` arrays, or `None`. `plot_imu` (`:88-114`) draws one subplot per available stream with time relative to the first sample. `write_imu_csv` (`:117-132`) merges both streams in time order.

### F.4 Algorithms
Data extraction and plotting only. There is no filtering, bias removal or gravity compensation.

### F.5 Why
"handy for a quick visual sanity check of IMU data alongside the .ply/detections outputs" (README `:358-361`) **[EXPLICIT]**.

### F.6 Parameters
Output folders. The figure styling constants are at `:33-47`.

### F.7 Limitations
- `.bag` only.
- Motion frames are only seen if the pipeline delivers them inside framesets. That is librealsense behaviour, and it is not clear from source whether every sample arrives exactly once in playback.
- The x-axis is relative per stream: each panel starts at its own first sample, so the accel and gyro t=0 may differ slightly (`:98`), even though `sharex=True`.

### F.8 Dependencies
`pyrealsense2` (playback, `as_motion_frame`, `get_motion_data`), `numpy`, `matplotlib` (Agg backend, headless).

### Observations
- If a sample appears in two framesets, duplicates would go to the CSV unchanged. There is no de-duplication by timestamp **[INFERRED risk]**.

---

## G. `plot_gyro_outliers.py`

### G.1 Purpose
Pools gyro samples from every `<stem>_imu.csv` written by `plot_imu.py` and flags samples whose angular-rate magnitude is anomalous. The aim is to find recordings where the camera moved or was bumped (`:1-23`).

### G.2 Inputs and outputs
Input: `csv_dir` (the `imu_total/` folder). Args: `--out` (default `<csv_dir>/../imu_outliers/`) and `--threshold` (3.5). Output: `gyro_outliers.png` (a scatter of |ω| over all samples with recording separators and outliers in red, plus a bar chart of outliers per recording) and the top 10 outliers on the console.

### G.3 Walkthrough
`load_gyro_magnitudes` (`:48-61`) → `flag_outliers` (`:64-72`) → `plot_outliers` (`:75-122`) → console report (`:149-158`).

### G.4 Algorithm: modified z-score (Iglewicz & Hoaglin, 1993)
For each sample, $|\boldsymbol\omega|=\sqrt{\omega_x^2+\omega_y^2+\omega_z^2}$. Then

$$\tilde m=\operatorname{median}(|\omega|),\quad \text{MAD}=\operatorname{median}\big(\big||\omega|-\tilde m\big|\big),\quad M_i = \frac{0.6745\,(|\omega|_i-\tilde m)}{\text{MAD}}$$

A sample is flagged when $|M_i|>3.5$. The constant 0.6745 is $\Phi^{-1}(0.75)$, so that MAD/0.6745 estimates σ for Gaussian data. If MAD = 0, all scores are set to 0 (`:68-69`).

### G.5 Why
"robust to the fact that most samples are near-stationary noise, so a handful of real spikes don't skew the threshold the way a mean/stdev-based z-score would" (`:15-19`). The cutoff of 3.5 is cited as "the standard Iglewicz & Hoaglin cutoff" (`:12-13`) **[EXPLICIT]**.

### G.6 Parameters

| Name | Default | Location | Effect |
|---|---|---|---|
| `--threshold` | 3.5 | `:130` | Modified-z cutoff |
| Top list | 10 | `:153` | Console listing |

### G.7 Limitations
- The statistics are **pooled across the whole dataset**. If some recordings are mostly moving (e.g. tilt sweeps), the median and MAD rise and stationary recordings' spikes are masked, or the reverse.
- |ω| includes gyro bias. Its distribution is non-negative and skewed, so the Gaussian calibration of 3.5 is approximate.
- Low-side outliers ($M<-3.5$) are also flagged by `np.abs`, but they are physically meaningless for a magnitude.
- `sorted(set(stems), key=stems.index)` and `stems.count` are O(U·N), which is slow for large datasets.

### G.8 Dependencies
`numpy`, `matplotlib` (Agg), standard library `csv`.

### Observations
- Recording boundaries assume the samples of each stem are contiguous, which holds because files are read one at a time (`:51`).

---

## H. `play_video.py`

### H.1 Purpose
A minimal MP4 player (`:2`). It is probably for reviewing the annotated `detections_test.mp4` written by `rs_marker -test` (`rs_marker/main.cpp:773-779`, `mp4v` fourcc) **[INFERRED]**.

### H.2 Inputs and outputs
`python3 play_video.py <file.mp4>`. Keys: space toggles pause, `q` quits, ←/→ step back/forward (keycodes 81/83 on Linux GTK after `&0xFF`, and 2/3 on macOS, `:37-43`). The output is a window only.

### H.3 Walkthrough
`play` (`:7-46`) opens `cv2.VideoCapture` and gets the FPS (falling back to 25 if 0) and the frame count, giving `delay = 1000/fps`. In the loop it reads a frame if not paused, shows it with the position in the title, and handles keys. Stepping back sets `POS_FRAMES` to `pos−2` and reads one frame.

### H.4–H.6
No algorithms. The only parameter is the FPS fallback of 25.

### H.7 Limitations and Observations
- At end of file during a right-arrow step, `cap.read()` returns `(False, None)` and the next `imshow` raises an error (`:41-43`).
- Seeking with `CAP_PROP_POS_FRAMES` is approximate for some codecs.
- `n_frames` can be wrong or 0 for some containers.

### H.8 Dependencies
`opencv-python` (`cv2`).

---

## I. `view_point_cloud.py` (Capture) vs `Processing/view_point_cloud.py`

### I.1 Purpose
An interactive Open3D GUI viewer for a single `.ply`. It shows a point cloud or mesh, with RGB, height (Z) colormap or grey colouring, a lit/unlit toggle and live Z-range crop sliders (`:1-14`). The docstring calls it `view_ply_interactive.py`, which suggests it was renamed.

### I.2 Inputs and outputs
`python3 view_point_cloud.py file.ply`. The output is a GUI window (1280×800, right-hand settings panel 22 em wide). Nothing is written.

### I.3 Walkthrough
- `PlyViewer.__init__` (`:77-129`): tries `read_triangle_mesh`. If it has triangles, it builds a point cloud from the mesh vertices, colours and normals. Otherwise it uses `read_point_cloud` and, if there are no normals, `estimate_normals` with `KDTreeSearchParamHybrid(radius=0.1, max_nn=30)` (0.1 in file units, so 10 cm for a metre-scaled RealSense PLY). It computes the bounding box and Z range, and avoids a degenerate slider range.
- `_build_ui` (`:133-222`): Open3D `gui.SceneWidget`, materials `defaultUnlit`/`defaultLit` with point size 3, combo boxes, checkbox, two `Slider.DOUBLE` and a reset button.
- Events update the state and call `_update_geometry` (`:286-309`): crop with an `AxisAlignedBoundingBox` (full XY, Z limited to [lo, hi]), recolour, replace the geometry, reset the camera (60° FOV, fit to bounds), and update the count label.
- `height_colors` (`:59-68`): normalise $z$ to $[0,1]$ over [vmin, vmax] and apply the Matplotlib turbo colormap, or fall back to `_turbo_colormap`, a 9-stop piecewise-linear approximation via `np.interp` (`:35-56`).

### I.4 Algorithms
- Normal estimation: Open3D hybrid KNN/radius neighbourhood with PCA of the local covariance.
- Colour mapping: linear normalisation then a colormap lookup.
- Cropping: an axis-aligned box test.

### I.5 Why
- Height colouring was made the default when the file has no colours (`:123`) **[EXPLICIT in code]**.
- The `_apply_coloring` docstring says it uses the *global* Z range "so colors stay consistent as you crop" (`:260-262`) **[EXPLICIT]**, but see the Observations.
- The `Processing/` copy's change in `c9f142d`, "Changed ply viewer to fix no colour ply file issue" **[EXPLICIT: c9f142d]**, replaced `matplotlib.cm.get_cmap("turbo")` with `matplotlib.colormaps["turbo"]` and widened the `except` to `(ImportError, AttributeError, KeyError)`. `cm.get_cmap` was removed in Matplotlib 3.9 and raises `AttributeError`, which the old `except ImportError` did not catch. Because height colouring is the default for PLYs *without colour*, those files crashed. This explains the commit title **[INFERRED: the Matplotlib version is not pinned in the repo]**.

### I.6 Parameters

| Name | Value | Location |
|---|---|---|
| Normal-estimation radius / max_nn | 0.1 / 30 | `:100-104` |
| Point size | 3.0 | `:146, :150` |
| Background | (0.05,0.05,0.05) | `:142` |
| Camera FOV | 60° | `:307` |
| Window | 1280×800, panel 22 em | `:135, :215` |

### I.7 Limitations
- "Height" means file Z. For `export_to_ply` clouds, Z is negated depth (§C.4) rather than height above the surface, so for a nadir-looking camera "height" colouring shows (negated) range, which is height only up to sign and offset **[INFERRED]**.
- Every slider move re-crops and re-uploads the whole geometry and resets the camera, so the user's view is lost each time.
- The RGB entry is offered even when the file has no colours. Choosing it silently shows grey (`:173-176, :268-269`).

### I.8 Dependencies
`open3d` (`io.read_triangle_mesh`, `io.read_point_cloud`, `estimate_normals`, `crop`, `visualization.gui`, `rendering.Open3DScene/MaterialRecord`), `numpy`, optionally `matplotlib`.

### Diff (Capture vs Processing)

```
65,67c65,67
<         import matplotlib.cm as cm
<         return cm.get_cmap("turbo")(norm)[:, :3]
<     except ImportError:
---
>         import matplotlib
>         return matplotlib.colormaps["turbo"](norm)[:, :3]
>     except (ImportError, AttributeError, KeyError):
326c326  (trailing newline only)
```

The **Capture copy still contains the Matplotlib ≥3.9 crash**. The two copies have diverged and have no shared module.

### Observations
1. Code and docstring disagree: `_apply_coloring` claims a global Z range (`:260-262`) but passes `self.crop_lo, self.crop_hi` (`:277`), so the colours *do* rescale to the crop window.
2. The duplicated file has diverged (only the Processing copy is fixed).
3. The docstring name (`view_ply_interactive.py`) does not match the file name.
4. The no-op `if not self.has_color: pass` block (`:173-176`) is dead code.

---

## J. `Capture/More_Tools/*` and `Capture/tools/PLY`

### J.1 Purpose
They have none in practice. They are placeholders imported in `8e07093` and never developed.

### J.2–J.3 Contents
- `Data_Processing.py` (2 lines, no trailing newline, so `wc -l` reports 1):
  ```
  print("Hellow world")
  readPlyGroup("Testing")
  ```
  `readPlyGroup` is never imported, so running this prints the greeting and then raises `NameError`.
- `readPlyGroup.py` (2 lines):
  ```
  def readPlyGroup(pathname)
      print(f"reading PlYGroup + {pathname}")
  ```
  This is a **SyntaxError** because the `def` line has no colon. Even if it were fixed, the function only prints its argument.
- `Capture/tools/PLY`: a zero-byte file (added in `e9cd7a8`). Its purpose cannot be determined from source. It may be a mistaken output (e.g. a redirect or a viewer save) or an intended directory.

### J.4–J.8
No algorithms, parameters or dependencies.

### Observations
- These three files are dead or broken and could confuse readers. Apart from `Data_Processing.py` calling `readPlyGroup` without importing it, nothing in the repo references them.
