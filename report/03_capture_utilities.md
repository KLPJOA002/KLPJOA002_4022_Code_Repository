# 03 — Capture utilities: `rec_viewer`, `rgb_viewer`, `rs_clean`, `test_res`, `rgb_test`

**Summary.** These five small C++ programs sit beside `rs_marker` in `Capture/`. They are bring-up and diagnostic tools for the RealSense D456:
- **`rgb_viewer`**: a live colour preview.
- **`rgb_test`**: an unbuilt, byte-identical copy of `rgb_viewer`.
- **`rec_viewer`**: records depth, colour and IMU to a file and shows live aligned and post-processed depth with IMU statistics.
- **`rs_clean`**: a tidier variant of `rec_viewer` at 848×480 @ 5 fps, which the README calls the "recommended recorder".
- **`test_res`**: a headless frame-drop test of depth plus both infrared streams at 1280×720.

`git log --follow` shows that none of them has changed since the code base was imported in commit `8e07093` ("Added Existing RS_Code, updated to match current open_CV version") and moved under `Capture/` in `5855c99`. They are inherited code, and none produces files the downstream `Processing/` pipeline consumes. All behaviour below comes from reading the source only. Nothing was compiled or run.

## Files covered

| Path | Lines | Role | Built by Makefile? |
|---|---|---|---|
| `Capture/programs/rec_viewer/main.cpp` | 206 | Record depth, colour and IMU; live preview with default post-processing and depth statistics | yes (OpenCV) |
| `Capture/programs/rgb_viewer/main.cpp` | 39 | Live colour preview (848×480 BGR8) | yes (OpenCV) |
| `Capture/programs/rs_clean/main.cpp` | 180 | Record depth, colour and IMU at 5 fps; live preview with post-processing | yes (OpenCV) |
| `Capture/programs/test_res/main.cpp` | 50 | Headless 1280×720 depth + IR1 + IR2 frame-drop check | yes (no OpenCV) |
| `Capture/rgb_test/main.cpp` | 39 | Identical copy of `rgb_viewer/main.cpp` | **no** (not under `programs/`, not in `PROGRAMS`) |

All built programs also link `build/lib/d456_recorder.o` without using it (`Capture/Makefile:46`; see `01_capture_core.md`).

## Contents
1. [rec_viewer](#1-rec_viewer-captureprogramsrec_viewermaincpp)
2. [rgb_viewer](#2-rgb_viewer-captureprogramsrgb_viewermaincpp)
3. [rs_clean](#3-rs_clean-captureprogramsrs_cleanmaincpp)
4. [test_res](#4-test_res-captureprogramstest_resmaincpp)
5. [rgb_test](#5-rgb_test-capturergb_testmaincpp)
6. [Cross-program comparison](#6-cross-program-comparison)

---

## 1. rec_viewer (`Capture/programs/rec_viewer/main.cpp`)

### 1.1 Purpose
`rec_viewer` records a session to a file while showing live colour and processed depth, and prints running fps, mean depth and IMU readings. The README lists it as "record + live depth/colour preview with IMU stats" (`Capture/README.md:94`). It is an operator's "is the camera working and what range am I at" tool.

### 1.2 Inputs and outputs

| Option | Default | Meaning (`main.cpp:39-52`) |
|---|---|---|
| `-o`, `--output` | `output.bag` | recording path |
| `-d`, `--duration` | 0 (until Ctrl+C or `q`) | seconds |
| `-h`, `--help` | — | usage |

Unknown arguments are **silently ignored** (there is no `else` branch).

| Stream (`:61-64`) | Format |
|---|---|
| Depth | 640×480 Z16 @30 |
| Colour | 640×480 RGB8 @30 |
| Accel / Gyro | MOTION_XYZ32F (default rates) |

Outputs:
- the raw recording (`enable_record_to_file`, `:67`);
- windows "Depth (colorized)" and "RGB";
- a console line every 30 frames: frames, fps, mean depth (m), accel (m/s²), gyro (rad/s) (`:166-179`);
- a final summary with the min/max valid depth seen and a list of the filters applied (`:187-203`).

### 1.3 Walkthrough
1. Parse the arguments and install SIGINT/SIGTERM handlers (`:39-55`).
2. Configure the streams and recording. Construct `PostProcessor` (`:16-33`), `rs2::colorizer` and `rs2::align(RS2_STREAM_COLOR)` (`:57-71`).
3. `pipe.start(cfg)` (`:76`). Then set the High Density preset on the depth sensor "after pipeline starts" (`:82-89`).
4. Loop until a signal, `q` or the duration (`:103-180`):
   - `poll_for_frames` (non-blocking; `continue` if nothing is ready, `:111-113`);
   - update the last accel/gyro from the frameset (`:115-118`);
   - align depth to colour (`:121`); skip if there is no depth (`:122-123`);
   - post-process: decimation → disparity → spatial → temporal → depth → hole fill, all at defaults (`:24-32,126`);
   - a per-pixel loop for min/max/mean of valid depth (`:129-145`);
   - colourise the depth; wrap the colour frame in a `cv::Mat` and convert RGB→BGR **in place** (`:148-157`); show both (`:159-160`);
   - every 30 frames, print the stats (`:166-179`).
5. `pipe.stop()` and print the summary (`:182-203`).

### 1.4 Algorithms and methods
- **Depth-to-colour alignment.** Each depth pixel is reprojected into the colour camera via depth intrinsics, the depth→colour extrinsics and the colour intrinsics. See `02_rs_marker.md` §A.4.2 for the equations.
- **RealSense post-processing chain** with default parameters, in Intel's recommended order: decimation (default factor 2), depth→disparity, spatial (edge-preserving domain-transform filter), temporal (EMA with persistence), disparity→depth, hole filling. Theory is in `01_capture_core.md` §1.4. Decimation is applied **after** alignment, so the processed and displayed depth is half the colour resolution (320×240).
- **Statistics**: $\bar z = \frac{1}{|V|}\sum_{p\in V} z_p$ over valid pixels $V=\{p: z_p>0\}$, plus running min and max.

### 1.5 Why this approach
- The preset must be set after the pipeline starts **[EXPLICIT: comment `:82`]**. `rs_marker` later chose the opposite, setting options before start so the recording header is correct **[EXPLICIT: `rs_marker/main.cpp:143-144`]**. The two programs make conflicting choices.
- The file "stores raw sensor frames … Replay with this tool or rs-record/rs-convert to apply filters offline" **[EXPLICIT: `:202-203`]**. In other words, filtering is for display only. The startup message "Recording raw + post-processed depth" (`:91`) contradicts this: only raw frames are recorded.
- `poll_for_frames` was used, presumably to keep the UI responsive. **[INFERRED]**

### 1.6 Parameters and tuning

| Name | Value | Location | Effect |
|---|---|---|---|
| Resolution / fps | 640×480 @30 | `:61-62` | Not a native D4xx depth optimum (848×480 is) |
| Preset | High Density | `:85` | Fill rate |
| Filters | all defaults | `:17-22` | Display only |
| Stats period | 30 frames | `:166` | Console update rate |

### 1.7 Assumptions, limitations, failure modes
- `poll_for_frames` in a tight loop with no sleep **busy-spins one CPU core** whenever no frameset is ready.
- The **in-place `cvtColor` on the librealsense colour buffer** (`:154-157`) mutates the frame's memory. The recorder may already have queued that frame for writing, so recorded colour could have red and blue swapped, depending on recorder timing. This is not verified. **[INFERRED]**
- The per-pixel `get_distance` loop is slow (a function call per pixel). It is acceptable at 320×240 after decimation.
- The `.bag` default name may fail on librealsense ≥ 2.57 (`README.md:8-11`).
- Min/max include every valid pixel, so outliers and flying pixels dominate the reported range.
- The colour frame is not null-checked after alignment.

### 1.8 Dependencies
librealsense2 (`pipeline`, `config`, `align`, the filters, `colorizer`, `motion_frame::get_motion_data`, `depth_frame::get_distance`); OpenCV (`Mat` wrapping external data, `cvtColor`, `imshow`, `waitKey`); `<csignal>`, `<atomic>`, `<chrono>` (used without an explicit `#include <chrono>`, so it relies on a transitive include).

### Observations (rec_viewer)
- Misleading "raw + post-processed" message (`:91`).
- Unknown CLI arguments are ignored silently.
- The help text does not mention `q` to quit.
- The summary prints `Depth range: [1e9 m, 0 m]` if no valid frame was processed (initial values `:100`).

---

## 2. rgb_viewer (`Capture/programs/rgb_viewer/main.cpp`)

### 2.1 Purpose
The simplest possible live colour preview, used to check that the camera and OpenCV display work ("live RGB preview — press q to quit", `README.md:174`).

### 2.2 Inputs and outputs
There are no arguments. The only stream is colour 848×480 **BGR8** @30 (`:9`), which needs no conversion for OpenCV. The output is the "D456 RGB" window. Nothing is written to disk.

### 2.3 Walkthrough
Start the pipeline (`:11-16`). Loop forever: `wait_for_frames`, take the colour frame, wrap it in a `cv::Mat` without copying, `imshow`, quit on `q` (`:20-35`). Then `pipe.stop()` (`:37`).

### 2.4 Algorithms
None. The firmware/librealsense supplies BGR8 directly.

### 2.5 Why this approach
A minimal sanity check. **[INFERRED]**

### 2.6 Parameters

| Name | Value | Location |
|---|---|---|
| Colour stream | 848×480 BGR8 @30 | `:9` |

### 2.7 Limitations
- There is no signal handler. Ctrl+C kills the process without `pipe.stop()`, which is harmless here because nothing is recorded.
- `wait_for_frames` throws after 5 s with no frames. The exception is uncaught, so the program terminates.

### 2.8 Dependencies
librealsense2, OpenCV highgui.

### Observations (rgb_viewer)
None of substance. It is functionally fine for its purpose.

---

## 3. rs_clean (`Capture/programs/rs_clean/main.cpp`)

### 3.1 Purpose
A cleaned-up recorder with live display. It has stricter CLI parsing, 848×480 streams and a 5 fps rate. The README calls it "clean recorder with live display (recommended)" (`README.md:95`) and "record + preview (recommended recorder)" (`README.md:180`). The second "Layout" section drops "recommended" (`README.md:489`).

### 3.2 Inputs and outputs

| Option | Default | Meaning (`main.cpp:13-19,49-57`) |
|---|---|---|
| `-o`, `--output` | `record.bag` | recording path |
| `-d`, `--duration` | 0 (until Ctrl+C or `q`) | seconds |
| `-h`, `--help` | — | usage |
| anything else | — | error + usage, exit 1 |

| Stream (`:65-68`) | Format |
|---|---|
| Depth | 848×480 Z16 @**5** |
| Colour | 848×480 RGB8 @**5** |
| Accel / Gyro | MOTION_XYZ32F |

Outputs: the raw recording (`:69`); windows "RGB" and "Depth (filtered)"; a status line every 5 frames with fps, mean depth, accel and gyro (`:149-166`); a final summary (`:174-177`).

### 3.3 Walkthrough
The structure is the same as `rec_viewer`:
1. Parse the arguments and set the signal handlers (`:49-60`).
2. Configure and start the pipeline, then apply High Density after start (`:62-83`).
3. Loop (`:102-167`):
   - `poll_for_frames`, IMU update, align to colour (`:110-119`);
   - run the `DepthFilters` chain: decimation → disparity → spatial → temporal → depth → hole fill (`:24-42,121`);
   - colourise, convert RGB→BGR in place, show (`:124-136`);
   - per-pixel mean-depth loop (`:139-145`);
   - status every 5 frames.
4. Stop and print the summary (`:169-177`).

### 3.4 Algorithms and methods
Identical to `rec_viewer` §1.4: alignment plus the default RealSense post-processing chain, for display only, and the valid-pixel mean depth.

### 3.5 Why this approach
- There is no threshold filter, because "it was silently zeroing frames and producing invalid bags in rs_record" **[EXPLICIT: `:21-23`]**. As noted in `01_capture_core.md` §1.5, the filtered output in `rs_record`'s `D456Recorder` never reaches the recording, so this diagnosis cannot be confirmed from the current code. An earlier, unseen version may have differed.
- 5 fps was chosen without a stated reason. **[INFERRED]** Likely reasons: smaller recordings, and leaving USB bandwidth for IMU and colour on a static scene.
- 848×480 is the D4xx's recommended depth resolution for best depth quality. That is general Intel guidance and is not stated in the code. **[INFERRED]**

### 3.6 Parameters and tuning

| Name | Value | Location | Effect |
|---|---|---|---|
| Resolution / fps | 848×480 @5 | `:65-66` | File size and depth quality |
| Preset | High Density | `:81-83` | Fill rate |
| Filters | defaults, no threshold | `:24-41` | Display only |
| Stats period | 5 frames | `:149` | Console update rate |

### 3.7 Assumptions, limitations, failure modes
- It has the same busy-spin `poll_for_frames`, the same in-place colour conversion on the frame buffer (`:127-133`), and the same lack of a colour null-check as `rec_viewer`.
- A 5 fps colour stream at 848×480 may not be a supported mode on every D4xx/firmware. `pipe.start` would then throw, and the code reports it and exits 1 (`:72-77`). Whether D456 supports it cannot be confirmed from the source.
- The `.bag` default name has the same librealsense-version caveat (`README.md:8-11`).

### 3.8 Dependencies
librealsense2, OpenCV (as for `rec_viewer`).

### Observations (rs_clean)
- The comment says the chain "Matches the filter chain used by rec_viewer" (`:22`), and it does.
- The header comment's claim about invalid bags caused by the threshold filter is unverified (see 3.5).
- Being "recommended" in the README conflicts with `rs_marker` being the tool actually used for data collection (`README.md:42`).

---

## 4. test_res (`Capture/programs/test_res/main.cpp`)

### 4.1 Purpose
A headless check that the camera can sustain depth plus both infrared imagers at 1280×720 @15 fps without dropping frames. It is probably meant to diagnose USB bandwidth or cable problems at high resolution. **[INFERRED]** There are no comments or README entry. It is not in the README's program list, although the Makefile builds it.

### 4.2 Inputs and outputs
There are no arguments.

| Stream (`:8-10`) | Format |
|---|---|
| Depth | 1280×720 Z16 @15 |
| Infrared index 1 (left) | 1280×720 Y8 @15 |
| Infrared index 2 (right) | 1280×720 Y8 @15 |

Output goes to stdout/stderr only: `NULL depth frame at iteration i`, `FRAME DROP: expected n+1, got m`, or a per-iteration capture error. Exit code 1 on a pipeline error.

### 4.3 Walkthrough
Start the pipeline (`:13`). For 150 iterations (`:16`), call `wait_for_frames(5000)` and take the depth frame. If it is null, report it. Otherwise compare `get_frame_number()` with the previous number plus 1 and report gaps (`:18-32`). Per-iteration `rs2::error`s are caught and reported (`:34-37`). Then `pipeline.stop()` (`:40`).

### 4.4 Algorithms and methods
**Frame-counter continuity.** The camera's hardware frame counter $n_k$ should satisfy $n_k = n_{k-1}+1$. Any jump means frames were dropped somewhere: in the device, USB transfer or the host queue. That covers 150 frames, about 10 s at 15 fps.

### 4.5 Why this approach
Frame numbers are the cheapest drop detector and need no display. **[INFERRED]**

### 4.6 Parameters

| Name | Value | Location |
|---|---|---|
| Resolution / fps | 1280×720 @15 (×3 streams) | `:8-10` |
| Iterations | 150 | `:16` |
| Frame timeout | 5000 ms | `:18` |

### 4.7 Limitations
- Only the depth counter is checked. IR drops are not reported.
- The `pipeline` keeps only the latest frames when the consumer is slow. This consumer is fast, so reported drops should reflect device or USB loss, though host-side queue drops cannot be fully excluded. **[INFERRED]**
- There is no summary count and no non-zero exit code on drops, so it cannot be used as an automated test.

### 4.8 Dependencies
librealsense2 only. No OpenCV in the Makefile and no extra flags.

### Observations (test_res)
- It is undocumented in the README and has no comments.
- `frame_number` is cast to `int` (`:26`). That is fine for short runs.

---

## 5. rgb_test (`Capture/rgb_test/main.cpp`)

### 5.1 Purpose
It appears to be a scratch copy of `rgb_viewer`. `diff` shows the two files are **byte-identical**. It has existed since the original import (`8e07093`) and was moved in `5855c99`.

### 5.2–5.8
Everything is as for `rgb_viewer` (§2). It is **not built**: the Makefile only globs `programs/<prog>/*.cpp` for names in `PROGRAMS` (`Capture/Makefile:6,43`), and `rgb_test` is neither under `programs/` nor listed.

### Observations (rgb_test)
- Dead duplicate code. It could be removed or folded into `rgb_viewer` without any effect on the build.

---

## 6. Cross-program comparison

| Program | Streams | Records? | Align | Filters | Preset | Display | CLI |
|---|---|---|---|---|---|---|---|
| `rs_record` (see 01) | D+C 848×480@30 | yes (raw) | no | full chain, output discarded | HD after start | none | positional |
| `rec_viewer` | D+C 640×480@30 + IMU | yes (raw) | yes | dec→disp→spat→temp→depth→hole (defaults) | HD after start | RGB + depth | `-o -d -h` (unknown ignored) |
| `rs_clean` | D+C 848×480@5 + IMU | yes (raw) | yes | same as rec_viewer, no threshold | HD after start | RGB + depth | `-o -d -h` (strict) |
| `rgb_viewer` / `rgb_test` | C 848×480 BGR8@30 | no | — | — | — | RGB | none |
| `test_res` | D + IR1 + IR2 1280×720@15 | no | — | — | — | none | none |
| `rs_marker` (see 02) | D+C 1280×720@30 + IMU (configurable) | yes, `.db3` | yes | none (disabled) | configurable, before start | RGB + depth | extensive |

Common facts:
- None of these utilities writes PLY, CSV or `scan_info.txt`, so none feeds the downstream `Processing/` tools directly.
- Their `.bag` recordings could be replayed with librealsense, subject to the version caveat.
- All IMU values are librealsense `rs2_vector` (accel m/s², gyro rad/s).
- All depth is metres via `get_distance` (Z16 × device depth scale, default 1 mm here, because none of the utilities changes the depth units).
