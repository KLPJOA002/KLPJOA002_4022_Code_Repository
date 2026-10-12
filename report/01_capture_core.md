# 01 — Capture core: D456 recorder library, `rs_record`, build system and capture conventions

**Summary.** The `Capture/` folder is a small Makefile-driven C++17 code base, built against librealsense2 (and OpenCV for most programs), that runs an Intel RealSense D456 stereo depth camera. This file covers the oldest layer of that code base. It has four parts. The first is a tiny "library" (`D456Recorder`) that records depth and colour to a RealSense recording file for a fixed time. The second is the `rs_record` command-line front end, plus a duplicate copy of it in `src/main.cpp`. The third is the build and launch tooling (`Makefile`, `run`, `run_both.sh`). The fourth is the documentation that sets the capture folder and file-naming conventions (`README.md`, `READMEdraft.md`, `processing.md`). Two diagnostic artefacts are also covered. `Notes/error.txt` is a console log of an early multi-angle run that failed with "Device or resource busy". `gdb_crash.log` is a full gdb thread dump of the librealsense 2.57 recorder segmentation fault that `rs_marker` hit on repeated record start/stop. Commit history shows these files were brought in as "Existing RS_Code" (commit `8e07093`) and later moved under `Capture/` (commit `5855c99`). Since then almost all development has gone into `rs_marker` (see `02_rs_marker.md`). All behaviour below comes from reading the source only. Nothing was compiled or run.

## Files covered

| Path | Lines | Role |
|---|---|---|
| `Capture/include/d456_recorder.hpp` | 37 | Declaration of the `D456Recorder` class (threaded fixed-duration recorder) |
| `Capture/src/d456_recorder.cpp` | 71 | Implementation of `D456Recorder` (the only "library" source compiled by the Makefile) |
| `Capture/src/main.cpp` | 26 | Byte-identical copy of `programs/rs_record/main.cpp`; **not built** by the Makefile |
| `Capture/programs/rs_record/main.cpp` | 26 | `rs_record` CLI: `rs_record [file] [seconds]` |
| `Capture/Makefile` | 65 | GNU Make build: one shared object + per-program template |
| `Capture/run` | 20 | Launcher wrapper (`./run <program> [args]`) |
| `Capture/run_both.sh` | 20 | Launches `rs_marker` and an external LiDAR capture script in parallel |
| `Capture/README.md` | 512 | Main user documentation (install, build, run, output conventions, Python tools) |
| `Capture/READMEdraft.md` | 221 | Older draft of the README |
| `Capture/processing.md` | 89 | Post-capture processing reference (collect → tabulate → marker points → IMU plots → MATLAB) |
| `Capture/Notes/error.txt` | 32 | Console log of an early tilt-sweep run that failed at iteration 1 |
| `Capture/gdb_crash.log` | 2017 | gdb log and all-thread backtrace of the recorder segfault (2 Oct 2026) |

## Contents

1. [D456Recorder library](#1-d456recorder-library-captureincluded456_recorderhpp-capturesrcd456_recordercpp)
2. [rs_record program (and src/main.cpp)](#2-rs_record-program-captureprogramsrs_recordmaincpp-and-capturesrcmaincpp)
3. [Build system: Makefile, run, run_both.sh](#3-build-system-makefile-run-run_bothsh)
4. [Capture folder and file conventions (READMEs, processing.md)](#4-capture-folder-and-file-conventions-readmes-processingmd)
5. [Diagnostic artefacts: Notes/error.txt and gdb_crash.log](#5-diagnostic-artefacts-noteserrortxt-and-gdb_crashlog)

---

## 1. D456Recorder library (`Capture/include/d456_recorder.hpp`, `Capture/src/d456_recorder.cpp`)

### 1.1 Purpose
`D456Recorder` wraps a librealsense `rs2::pipeline` that records depth and colour to a file (`rs2::config::enable_record_to_file`) for a fixed number of seconds. The acquisition loop runs on a background `std::thread`. It is the only code in `src/`, and the Makefile links it into **every** program (`Capture/Makefile:26-27,46`), but only `rs_record` actually uses it. In the system it is a legacy "record raw data now, process later" tool. The main capture tool, `rs_marker`, does not use it.

### 1.2 Inputs and outputs

| Item | Value | Source |
|---|---|---|
| Constructor `output_path` | path to the recording file (librealsense picks the container from the library version; see §4) | `d456_recorder.cpp:5-15` |
| Constructor `duration_s` | recording time in seconds (double) | `d456_recorder.cpp:6,45` |
| Constructor `high_density_preset` | default `true`: apply the RS400 High Density visual preset | `d456_recorder.hpp:10-11`, `d456_recorder.cpp:65-71` |
| Depth stream | 848×480, `RS2_FORMAT_Z16` (uint16 depth in device depth units, default 1 mm), 30 fps | `d456_recorder.cpp:12` |
| Colour stream | 848×480, `RS2_FORMAT_RGB8`, 30 fps | `d456_recorder.cpp:13` |
| IMU | not enabled | — |
| Output | one recording file holding the **raw** depth and colour frames plus device metadata/intrinsics, written by librealsense's recorder | `d456_recorder.cpp:14` |

The public API is `start()`, `stop()` and `is_running()` (`d456_recorder.hpp:13-15`).

### 1.3 Walkthrough
1. **Construction** (`d456_recorder.cpp:5-15`) stores the arguments, enables the two streams and calls `cfg_.enable_record_to_file(output_path_)`. The pipeline does not start yet.
2. **`start()`** (`:17-21`) sets `running_ = true` and launches `acquisition_loop` on `thread_`.
3. **`acquisition_loop()`** (`:34-63`):
   - `pipe_.start(cfg_)` (`:37`) opens the device and starts the recorder.
   - `apply_preset()` (`:38`, defined `:65-71`) takes `query_sensors()[0]`, assumed to be the stereo/depth module. If `high_density_` is set and the option is supported, it sets `RS2_OPTION_VISUAL_PRESET = RS2_RS400_VISUAL_PRESET_HIGH_DENSITY`.
   - The loop (`:41-58`) runs until `running_` is cleared or `duration_s_` elapses on `steady_clock` (`:43-45`). Each pass calls `pipe_.wait_for_frames()`, takes the depth frame and runs it through decimation → threshold → depth-to-disparity → spatial → temporal → disparity-to-depth → hole filling (`:51-57`), all at library defaults. **The filtered frame is never used.** It is not displayed, saved or returned, so the chain only costs CPU (see Observations).
   - Any exception is captured into `thread_exception_` (`:59-61`). `running_` is then cleared (`:62`).
4. **`stop()`** (`:23-30`) clears `running_`, joins the thread, calls `pipe_.stop()` (swallowing errors, which closes the recording) and rethrows any captured worker exception on the caller's thread.

### 1.4 Algorithms and methods
- **Recording.** `rs2::config::enable_record_to_file` inserts librealsense's record device wrapper. It serialises every frame the device delivers, plus stream profiles, intrinsics/extrinsics and device info, into a ROS-bag container. This is **unfiltered** sensor data. Any post-processing in the loop does not reach the file.
- **RealSense post-processing chain (unused output).** The order decimation → threshold → disparity → spatial → temporal → depth → hole-fill matches Intel's recommended order in the white paper "Depth Post-Processing for Intel RealSense D400 Depth Cameras" (Grunnet-Jepsen & Tong). In brief:
  - *Decimation* subsamples the image by an integer factor (default 2) using a median/mean of each block.
  - *Threshold* zeroes depths outside $[z_{min}, z_{max}]$.
  - The *disparity transform* maps $z \mapsto d \propto \frac{fB}{z}$, so the edge-preserving filters run in a domain where noise is roughly constant.
  - *Spatial* is a 1-D edge-preserving recursive (domain-transform) filter. Each new value is $\hat x_i=\alpha x_i+(1-\alpha)\hat x_{i-1}$ when $|x_i-\hat x_{i-1}|<\delta$.
  - *Temporal* is an exponential moving average over frames with a persistence-based hole filler.
  - *Hole filling* fills zero pixels from a neighbour (default "farthest from around").
  The theory is given here for completeness only. In `D456Recorder` the result is thrown away.
- **Visual preset.** High Density is a firmware parameter bundle (confidence thresholds, texture thresholds and similar) that favours fill rate over accuracy.

### 1.5 Why this approach
- A threaded recorder with a duration cap keeps `main` free to poll and print. **[INFERRED]** No comment explains the design.
- High Density was preferred, presumably to maximise fill on low-texture surfaces. **[INFERRED]** The only explicit statement is the `/*high_density=*/true` argument (`programs/rs_record/main.cpp:12`).
- Running the filters in the loop has no stated reason. **[INFERRED]** It looks like leftover code from a version that displayed or exported the filtered depth. `rs_clean` explicitly avoids the threshold filter because "it was silently zeroing frames and producing invalid bags in rs_record" **[EXPLICIT: `Capture/programs/rs_clean/main.cpp:22-23`]**. That claim does not fit this code: the filtered frame is never written, so it cannot change the recording. Either an earlier `rs_record` differed or the diagnosis was wrong. Which one cannot be determined.

### 1.6 Parameters and tuning

| Name | Default | Location | Effect |
|---|---|---|---|
| Depth stream | 848×480 Z16 @30 | `d456_recorder.cpp:12` | Recorded depth resolution/rate. 848×480 is the D4xx's best-performing depth resolution |
| Colour stream | 848×480 RGB8 @30 | `d456_recorder.cpp:13` | Recorded colour |
| `high_density_` | `true` | `d456_recorder.hpp:11` | Applies the High Density preset after start |
| Filter options | librealsense defaults | `d456_recorder.hpp:30-36` | No effect on the output |
| `duration_s_` | caller supplied (30 s in `rs_record`) | `d456_recorder.cpp:45` | Recording length (checked once per frame) |

### 1.7 Assumptions, limitations, failure modes
- `query_sensors()[0]` is assumed to be the depth sensor (`:67`). This holds on D4xx devices, but `first<rs2::depth_sensor>()` would be safer.
- The preset is applied **after** `pipe.start()` (`:37-38`), so the first frames, and possibly the recorded option state, use the previous/default preset.
- `wait_for_frames()` throws after its default 5 s timeout if the camera stalls. The exception ends the recording and is rethrown from `stop()`.
- If an object is destroyed without `stop()` while the thread is joinable, `std::thread`'s destructor calls `std::terminate`. `rs_record` always calls `stop()` on the normal path. On the exception path inside the `try`, the destructor would run with a joinable thread.
- The duration is checked only between frames, so a stall can overrun it by up to 5 s.

### 1.8 Dependencies
- **librealsense2** (`rs2::pipeline`, `rs2::config`, the filters, `rs2::sensor::set_option`). `pipeline::start(cfg)` resolves the config against the connected device, opens the sensors and, because of `enable_record_to_file`, wraps the device in a recorder. `wait_for_frames` blocks until a time-synchronised frameset is ready.
- **C++ standard library**: `<thread>`, `<atomic>`, `<chrono>` and `std::exception_ptr`. `<exception>` is only included transitively.

### Observations (D456Recorder)
- The filter chain output is discarded (`d456_recorder.cpp:51-57`): wasted CPU, and it misleads readers into thinking the recording is filtered.
- `src/main.cpp` is a byte-identical duplicate of `programs/rs_record/main.cpp` (checked with `diff`). The Makefile never compiles it (`LIB_SRCS := src/d456_recorder.cpp`, `Makefile:26`). It is dead code.
- Every program links `d456_recorder.o` (`Makefile:46`) even though only `rs_record` uses it.
- The class is named D456 but the default output filename is `d455_record.bag` (`programs/rs_record/main.cpp:8`).
- `stop()` swallows all errors from `pipe_.stop()` (`:27`), so a failure to finalise the file would go unreported.

---

## 2. rs_record program (`Capture/programs/rs_record/main.cpp`, and `Capture/src/main.cpp`)

### 2.1 Purpose
This is the minimal recorder front end: record raw depth and colour for N seconds with no preview. The README lists it as "record depth + colour to .bag" (`README.md:92`).

### 2.2 Inputs and outputs

| Positional arg | Default | Meaning |
|---|---|---|
| `argv[1]` | `d455_record.bag` | output recording path |
| `argv[2]` | `30.0` | duration in seconds (`std::stod`) |

Output: the recording file (see §1.2), plus `Recording …`/`Done.` messages on stdout. Exit code 1 on `rs2::error` or `std::exception`.

### 2.3 Walkthrough
`main` (`programs/rs_record/main.cpp:6-26`) parses the two positional arguments (`:8-9`) and builds `D456Recorder(outfile, duration, true)` (`:12`). It calls `start()`, polls `is_running()` every 500 ms (`:15-16`), then calls `stop()` (`:17`), which rethrows any worker exception into the handlers at `:19-25`.

### 2.4 Algorithms and methods
None beyond §1.4.

### 2.5 Why this approach
A thin CLI over the class keeps the program trivial. **[INFERRED]**

### 2.6 Parameters
Only the two positional arguments. There is no `--help`, and a non-numeric duration throws `std::invalid_argument`, which is caught and reported as `Error:`.

### 2.7 Limitations
- No Ctrl+C handler. SIGINT kills the process, and the recording may be left unfinalised.
- There is no IMU, unlike `rs_marker`/`rec_viewer`/`rs_clean`.
- With librealsense ≥ 2.57 a `.bag` path is reported to fail (README, see §4.2).

### 2.8 Dependencies
`D456Recorder`, librealsense2.

### Observations (rs_record)
- `src/main.cpp` duplicates this file (see §1, Observations).
- The default filename says "d455" (a different camera model).

---

## 3. Build system: Makefile, run, run_both.sh

### 3.1 Purpose
The build system compiles one shared object (`build/lib/d456_recorder.o`) and one executable per program directory. It places the binaries in `Capture/` and provides launchers.

### 3.2 Inputs and outputs

| Artefact | Location |
|---|---|
| Shared object | `build/lib/d456_recorder.o` (`Makefile:26-36`) |
| Per-program objects | `build/<prog>/<file>.o` (`Makefile:43-44,49-50`) |
| Executables | `Capture/<prog>` (target name = program name, `Makefile:46-47`) |
| Programs built | `rs_record rgb_viewer rec_viewer rs_clean test_res rs_marker` (`Makefile:6`) |

Make targets: `all` (the default, runs `check-deps` then all programs), one target per program, `clean`, and `check-deps`.

### 3.3 Walkthrough
- **Flags** (`Makefile:1-3`): `g++ -std=c++17 -O2 -Wall -Wextra -Iinclude`, linked with `-lrealsense2 -lpthread`.
- **Per-program extras** (`:8-18`): `rgb_viewer`, `rec_viewer`, `rs_clean` and `rs_marker` add `pkg-config --cflags/--libs opencv4`. `rs_record` and `test_res` do not.
- **`PROGRAM_template`** (`:39-51`) is expanded once per program with `$(foreach … $(eval $(call …)))` (`:53`). It globs `programs/<prog>/*.cpp`, so `rs_marker` gets `main.cpp` + `mountSerial.cpp` (comment at `:38`). It compiles each file to `build/<prog>/%.o` with an order-only dependency on the directory, then links the shared object, the program objects, `$(LIBS)` and the extra libs.
- **`check-deps`** (`:58-61`) succeeds if `pkg-config --exists realsense2` or `ldconfig -p | grep librealsense2` succeeds. Otherwise it prints a warning and exits 1, which aborts `make all`. Individual targets (`make rs_marker`) skip the check.
- **`run`** (`Capture/run:1-20`): with no arguments it prints usage and lists `programs/`. Otherwise it checks that `$SCRIPT_DIR/$PROG` is executable and `exec`s it with the remaining arguments.
- **`run_both.sh`** (`Capture/run_both.sh:1-20`) starts two jobs in the background:
  - `/home/aru/agi/3D-SeaIce/cpp/run rs_marker -n 25 -D /home/aru/Documents/validationTests`
  - `python3 …/Software_Sync/scripts/software_sync_capture_new.py --frames 100 --lidar-only --lidar-format lvx`, an external repo ("Masters-Project_Antarctic_Sensor_Suite"). The "lvx" format suggests a Livox LiDAR. **[INFERRED]**
  It prints both PIDs, waits for each and prints the exit codes.

### 3.4 Algorithms and methods
Standard GNU Make pattern rules and templated rule generation. There is nothing algorithmic.

### 3.5 Why this approach
- The `PROGRAMS` list plus template makes adding a program a one-line change **[EXPLICIT: `README.md:493-512` "Adding a new program"]**.
- The `run` launcher exists to apply a pthread fix needed on Ubuntu 22.04+ with snap **[EXPLICIT: `Makefile:21-24`, `run:3`, `README.md:167-168`]**. The Makefile comment explains that snap core20's `libpthread` (built for glibc 2.31) can be pulled in by Qt's platform plugin and lacks `__libc_pthread_init` in glibc ≥ 2.35, so preloading the system pthread "wins the race".
- `run_both.sh` exists for synchronised RealSense + LiDAR capture **[EXPLICIT: `README.md:71-74` "Lidar and Realsense"]**. How the two are synchronised is not stated. The script simply starts both at once.

### 3.6 Parameters and tuning

| Name | Default | Location | Effect |
|---|---|---|---|
| `CXX` | `g++` | `Makefile:1` | compiler |
| `CXXFLAGS` | `-std=c++17 -O2 -Wall -Wextra -Iinclude` | `Makefile:2` | C++17 is required (`std::filesystem` in `mountSerial.cpp`) |
| `LIBS` | `-lrealsense2 -lpthread` | `Makefile:3` | can be overridden for non-standard installs (`Makefile:63-65`) |
| `PROGRAMS` | 6 programs | `Makefile:6` | which programs are built |
| `<prog>_EXTRA_CFLAGS/LIBS` | OpenCV for 4 programs | `Makefile:8-18` | per-program dependencies |
| `PTHREAD_FIX` | `LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0` | `Makefile:24` | **defined but never used** |

### 3.7 Assumptions, limitations, failure modes
- **The pthread fix is not applied anywhere.** `PTHREAD_FIX` is never referenced in a recipe, and `run` does a plain `exec` with no `LD_PRELOAD` (`run:20`). The comments at `run:3` and `README.md:167` therefore describe behaviour the script does not have.
- Header dependencies are not tracked (no `-MMD -MP`). Editing `include/d456_recorder.hpp` or `programs/rs_marker/mountSerial.h` does not trigger a rebuild of dependants.
- The suggested override (`Makefile:63-65`, `READMEdraft.md:219`) uses `-std=c++14`, which would fail to compile `mountSerial.cpp` (`std::filesystem`).
- `run` checks `$SCRIPT_DIR/$PROG` but lists programs from the current directory's `programs/` (`run:8`). Usage output is therefore wrong when `run` is invoked from elsewhere.
- `run_both.sh` hard-codes absolute paths on another user's machine (`/home/aru/agi/3D-SeaIce/cpp/run`, `/home/aru/agi/Masters-Project_Antarctic_Sensor_Suite/...`). It will not run in this repository layout (`Capture/`, not `cpp/`).
- In `run_both.sh`, `rs_marker` runs as a **background job of a non-interactive script**, so its stdin is normally `/dev/null`. The current `rs_marker` (a) enables the tilt mount by default, so it needs the Arduino, and (b) after each rotation reads stdin to ask whether to continue (`programs/rs_marker/main.cpp:1315-1325`). On EOF the line is empty, which is treated as "continue". **[INFERRED]** As written, `run_both.sh` would therefore loop through rotations until it is killed.
- Committed binaries: `Capture/` contains the built executables, and the `build/` directory is untracked per git status. Earlier commits (e.g. `1c1f7db`, `e459457`) committed `.o` files and binaries.

### 3.8 Dependencies
GNU Make, `g++`, `pkg-config` (for `opencv4`/`realsense2`), `ldconfig`, `bash`. The external Python capture script in `run_both.sh` is not part of this repository.

### Observations (build)
- The unused `PTHREAD_FIX` variable means the documented launcher purpose is not implemented.
- `rgb_test/` is not in `PROGRAMS` and is never built (see `03_capture_utilities.md`).
- Both READMEs document `cpp/` as the folder name and say binaries "land in `cpp/`" (`README.md:163`). The folder is now `Capture/`.

---

## 4. Capture folder and file conventions (READMEs, processing.md)

### 4.1 Purpose
`README.md` (current) and `READMEdraft.md` (older) document installation, the build, how to run each program and the on-disk outputs of `rs_marker`. `processing.md` describes the post-capture pipeline that hands data to MATLAB. This section records what these documents claim and compares it with what the code does now.

### 4.2 Documented installation constraints
- **librealsense version pin.** "Install version 2.56.x, not the latest … librealsense 2.57+ switched `enable_record_to_file` to require the newer `.db3` (rosbag2) format — recording will fail with a `db3` error on those versions" **[EXPLICIT: `README.md:8-11`, repeated `:136-138`]**. The README pins `2.56.5-0~realsense.17055` and `apt-mark hold`s it (`README.md:17-24`).
  - **Inconsistency:** the current `rs_marker` writes `.db3` (`programs/rs_marker/main.cpp:932`), commit `8f29459` says "Updated librealsense version to 2.58, changed output file to db3", and `gdb_crash.log` shows the program running against `/lib64/librealsense2.so.2.57`. The README pin is therefore stale for `rs_marker`. The README's remaining programs (`rs_record`, `rec_viewer`, `rs_clean`) still default to `.bag` names.
- **OpenCV with ArUco** (`README.md:101-130`). On recent OpenCV (≥ 4.7) ArUco is part of the main `objdetect` module. `gdb_crash.log` shows `libopencv_objdetect.so.413`, i.e. OpenCV 4.13.
- **Platform.** The README targets Ubuntu 22.04 (apt, snap). `gdb_crash.log` shows a Fedora host (`debuginfod.fedoraproject.org`, `/lib64`).

### 4.3 Documented `rs_marker` outputs vs actual

| README claim (`README.md:59-64,211-228`) | Current code (`programs/rs_marker/main.cpp`) |
|---|---|
| Session dir `<dir>/<YYYYMMDD_HHMMSS>/` | `<dir>/<ts>/<ts>_r<rot>/<ts>_r<rot>_a<tilt>/` (`:884,911-914,930-931`) |
| `<YYYYMMDD_HHMMSS>.bag` | `<ts>_r<rot>_a<tilt>.db3` (`:932`) |
| `<YYYYMMDD_HHMMSS>_detections.csv` | `<ts>_r<rot>_a<tilt>_detections.csv`, with an extra `decoded` column (`:258-270`) |
| `<YYYYMMDD_HHMMSS>.ply` best frame, "fully post-processed (decimation → disparity → spatial → temporal → disparity → hole-filling)" | `<…>.ply`. The depth is aligned to colour but **not filtered**: `DepthFilters::process` is a no-op (`:75-85`) |
| `imu.csv` | `imu_a<tilt>.csv` (`:958-960`) |
| — | `scan_info.txt` (full configuration dump, `:694-763`) |
| CSV `timestamp_ms` "ms since program started (matches bag timeline)" | ms since the start of *this scan* on the host's `steady_clock` (`:1023,1140-1141`). It is not a device or bag timestamp |
| Flags `-o -D -d -n -test -id`; "`-o` and `-D` cannot be combined" | The mutual-exclusion check is commented out (`:926-928`). Many more flags exist (see `02_rs_marker.md`) |

`processing.md` and the `collect_plys.py` documentation (`README.md:253-316`) still describe `.bag` files and a single `imu.csv` that is "named identically in every session folder" and must be renamed. That matches the older layout, not the current `imu_a<tilt>.csv`. The Python tools in `Capture/tools/` are out of scope here. Whether they were updated for `.db3` and the nested rotation/tilt folders cannot be determined from these documents.

### 4.4 Post-processing chain (`processing.md`)
`processing.md:14-89` describes this order:
1. `collect_plys.py`: split each run into `<src>_collected/{ply,detections,imu}` and `<src>_raw/` (bags, flat).
2. `tabulate_data.py`: one metadata row per recording, from the bag headers plus the collected files.
3. `marker_points.py`: re-sample depth at each marker centre from the bag and deproject into "the same depth optical frame as the reconstructed `.ply` clouds", giving a per-marker median `x,y,z` (`README.md:318-334`).
4. `plot_imu.py` / `plot_gyro_outliers.py`: IMU sanity plots ("Ideally use the extracted imu csv to perform plane adjustment", `processing.md:66`).
5. Hand-off: "Upload outputs to Google Drive → move onto MATLAB processing" (`processing.md:89`).

This is an older processing path (MATLAB). The repository also contains a `Processing/` tree (Crop, Fuse, Evaluate) that reads `scan_info.txt` (for example `Processing/Fuse/src/fusion_session.cpp:597-607`, `Processing/Crop/crop_pointclouds.py:143-145`), which is covered elsewhere.

### 4.5 Why these conventions
- The `.bag` is large and is "only needed if you need to recapture something → use the smaller extracted files for processing" **[EXPLICIT: `processing.md:16-17`]**.
- Timestamp-named folders avoid overwriting runs and let tools match files by stem **[EXPLICIT: `README.md:313-316`]**.
- `READMEdraft.md:190-193` explains that the detection code is fenced between `═══` banners for easy editing, and that `detect_markers()` filters by `TARGET_IDS` (test mode only) while `detect_all_markers()` keeps every tag **[EXPLICIT]**. This still matches the code (`programs/rs_marker/main.cpp:340-647`).

### Observations (documentation)
- The README duplicates whole sections ("Build", "Run", "Layout", "Lidar and Realsense" appear twice: `:149-191` vs `:439-468`, `:77-97` vs `:471-491`).
- The README describes `rs_clean` as "(recommended)" in one place (`:95,180`) and not in the other (`:489`).
- Both the version-pin advice and the output-layout tables are stale relative to `rs_marker` (see §4.2, §4.3).
- `READMEdraft.md` is a strict subset/predecessor. It mentions only `.bag` + CSV outputs and no PLY/IMU.
- Run commands reference `/home/aru/...` paths, i.e. a different user's machine (the code was inherited, commit `8e07093`).

---

## 5. Diagnostic artefacts: `Notes/error.txt` and `gdb_crash.log`

### 5.1 `Notes/error.txt` (32 lines; last changed in commit `1324632`, 21 Aug 2026)
This is a console transcript of `./run rs_marker -a 30 -s 10 -n 15 -D ./Recordings` on host `aru@ARU-ASUS15`:
- The serial mount connects on `/dev/ttyACM0`, homing replies `H0000.0` (`:2-4,16`), and the sweep plans 7 iterations ($2\cdot30/10+1$, `:5`).
- Iteration 0 records `…_a0/…_a0.bag`, a CSV, a PLY and `imu_0.csv` at 848×480, about 4.6 fps, with an IMU sample `accel [0.07,-0.58,9.68]` (`:7-26`). This was the older layout, with no rotation level, `.bag`, and an `imu_<angle>.csv` name.
- Qt prints `QSocketNotifier: Can only be used with threads started with QThread` (`:18`), an OpenCV HighGUI warning.
- After `S0010.0` (moved to +10°), iteration 1 fails immediately: `RealSense error: get_xu(ctrl=1) failed! Last Error: Device or resource busy` (`:29-31`).

**Interpretation.** Restarting the pipeline straight after `stop()` found the UVC extension unit still claimed. **[INFERRED]** The commit history shows the responses that followed:
- a post-scan sleep (`e459457` 500 ms, `dc06e2d` 1000 ms, "added additional delay");
- a 5-attempt retry with back-off (`22174f5`: "retry piepline start a few times to avoid ocasional busy error") **[EXPLICIT]**;
- eventually a single shared `rs2::context` (`00b2c4e`; comment at `programs/rs_marker/main.cpp:89-92`) **[EXPLICIT]**.

### 5.2 `gdb_crash.log` (2017 lines; file dated 2 Oct 2026 16:10, committed in `00b2c4e`)
**What was run.** `rs_marker` under gdb on a Fedora host against librealsense **2.57** (`/lib64/librealsense2.so.2.57`, `librsutils.so.2.57`) and OpenCV 4.13. Session `…/Joab_Characterisation/Debug_segfault/20261002_160830`, rotation `r0` (`gdb_crash.log:79`). Settings visible in the log: 1280×720@30, preset high_density, depth units 0.001 m, `-n 90`, best range `45:89`, and a tilt sweep of 11 iterations at 5° steps to ±25° (`:80-92`).

**Progress before the crash.**
- Iterations 0–7 (tilts 0, 5, 10, 15, 20, 25, −5, −10) complete normally. Each records a `.db3`, CSV, PLY, `imu_a<tilt>.csv` and `scan_info.txt`. Throughput is about 11.7 fps at 1280×720 with about 101 k zero-depth pixels per frame, roughly 11 % of 921 600 (`:93-94`).
- Mount replies such as `S0005.0` and `S0-15.0` confirm the serial protocol.
- From iteration 1 on, the banner prints `units 0.00 m` (`:159` etc.). A sticky `std::fixed`/`setprecision(2)` from the end-of-scan "Duration" print was leaking into the next banner. The current code fixes this with `std::defaultfloat` (`programs/rs_marker/main.cpp:1000`).

**The crash** (`:660-676`, backtrace `:1401-1408`):
- Iteration 8 (tilt −15°) has just started streaming (frame 0) when thread 131 receives SIGSEGV at `0x000000c3000001b6 in ?? ()`, a garbage instruction pointer.
- Its stack is `librealsense::record_sensor::register_notifications_callback(...)::{lambda(rs2_notification*)#1}` → `notifications_processor::raise_notification(...)` lambda → `dispatcher` worker thread (`librsutils`).
- Many other threads show `polling_error_handler::polling` (`:805,1008,…`), the periodic hardware-error poller that raises notifications.
- The main thread (Thread 1, `:1999-2017`) is innocently inside `cv::aruco::ArucoDetector::detectMarkers`, called from `detect_all_markers` at `programs/rs_marker/main.cpp:571/605/1141` (line numbers of the version that crashed).
- The thread list runs to about 292 threads (`Thread 292 …`, `:676`). Threads from earlier scans pile up.

**Diagnosis (matches the code comment and project memory).** In librealsense 2.57 the recorder's `record_sensor` installs its own notification lambda on the live sensor. On teardown it restores the previous callback **only if the user had registered one**. Otherwise the lambda, which captures the destroyed `record_sensor`, stays installed. The next camera notification in a later scan calls through a dangling pointer and segfaults **[EXPLICIT: `programs/rs_marker/main.cpp:165-169` comment]**. The crash is intermittent because it needs a notification (e.g. from the polling error handler) to arrive after a recorder has been destroyed.

**Mitigations in the current code** (all `programs/rs_marker/main.cpp`):
1. Register a user notifications callback on every sensor of the recording device after each `pipe.start` (`:170-173`), so each recorder restores *that* callback instead of leaving its own lambda.
2. Use one `rs2::context`/`rs2::pipeline` for the whole run (`:89-98`), because a pipeline per scan "piles up contexts on the same device (incl. the HID IMU) and segfaults after a few scans" **[EXPLICIT]**.
3. Reset `ctx.profile` after `stop()` to destroy the recorder and close the `.db3` before the next scan (`:1253`) **[EXPLICIT comment]**.
4. Drop the held best-frame references before stopping (`:1215-1217`) **[EXPLICIT comment]**.

**Chronology from the commits.**
- `22174f5` (retry; "Still suffers segmentation fault occasionally").
- `f4253ae` ("disabled thermal compensation, rs_marker works mostly, still occasionally gives segmentation faults").
- `8f29459` ("Updated librealsense version to 2.58, changed output file to db3. no segmentation fault occured yet").
- The 2 Oct gdb log still shows the crash on 2.57. The `main.cpp` modification time is 16:13, three minutes after the log, and the 10-line offset between the log's line numbers (571/605/1141) and today's (581/615/1151) equals the size of the added callback block. Together these suggest the callback workaround was written right after this capture. **[INFERRED]**
- Whether the workaround fully removes the crash cannot be determined from the repository. No later crash log or commit message confirms it.

### Observations (diagnostics)
- The log was committed to the source tree (171 kB). It is useful as evidence but is not code.
- `error.txt` reflects an obsolete layout and naming (`.bag`, `imu_0.csv`, no `_r` level). Do not read it as current behaviour.
