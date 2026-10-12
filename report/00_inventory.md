# 00 — Inventory

**Repository:** `/home/joab/Documents/4022/Code` (git submodule `Code` of the parent thesis repo `4022/`, remote `github.com/KLPJOA002/4022_Code`)
**Snapshot audited:** `main` @ `1e31b72` (2026‑10‑06) plus working tree as of 2026‑10‑12
**Method:** read-only. Nothing was built, run, or tested; all behaviour described in this report is derived from reading source, docs and git history.

---

## 1. What the project is (one paragraph)

A capture-and-processing toolchain for measuring the 3D shape and surface roughness of small physical surfaces (test objects such as pyramids, wave forms, egg cartons, and ultimately a sediment bed — see `Processing/Rugosity_Metrics/README.md:3`) with an **Intel RealSense D456** stereo depth camera mounted on a **servo-driven tilt/rotation rig**. C++ programs (librealsense2 + OpenCV) capture depth/colour/IMU streams, detect AprilTag fiducials, and drive the rig over serial. Python and C++ tools then crop the per-angle point clouds to the tagged region, register and fuse them into one cloud (PCL), evaluate geometric accuracy against analytic or mesh ground truth (Open3D/SymPy), sweep hardware and post-processing parameters, and compute ISO 25178-style roughness metrics.

## 2. Directory map (project-owned files only)

```
Code/
├── .gitignore
├── Capture/                          ← C++ capture side (librealsense2, OpenCV)
│   ├── Makefile, run, run_both.sh
│   ├── README.md, READMEdraft.md, processing.md
│   ├── include/d456_recorder.hpp      src/d456_recorder.cpp, src/main.cpp
│   ├── programs/
│   │   ├── rs_marker/ main.cpp (1336 L), mountSerial.{cpp,h}   ← primary capture tool
│   │   ├── rs_record/ rec_viewer/ rgb_viewer/ rs_clean/ test_res/
│   ├── rgb_test/main.cpp
│   ├── tools/
│   │   ├── Depth_Quality/depth_quality.cpp (1746 L) + committed binary
│   │   ├── collect_plys.py marker_points.py summarize_markers.py tabulate_data.py
│   │   ├── plot_imu.py plot_gyro_outliers.py play_video.py view_point_cloud.py
│   │   └── PLY (empty file)
│   ├── More_Tools/ Data_Processing.py readPlyGroup.py (stubs)
│   ├── Notes/error.txt, gdb_crash.log
│   └── (untracked) build/, rec_viewer, rgb_viewer, rs_clean, rs_marker, rs_record, test_res (binaries)
└── Processing/
    ├── Crop/            crop_pointclouds.py (1231 L), rename_ply.sh, __pycache__/*.pyc (committed)
    ├── Fuse/            C++/PCL fusion tool fuse_scans (~6.5k L, 20 src files), CMakeLists.txt, README.md
    ├── Bulk_Process_Pipeline/  bulk_process.py, pipeline_config.example.toml, README.md
    ├── Evaluate/        evaluate_scans.py + scan_eval/ (13 modules) + surfaces/*.toml + plugin + tests/
    ├── Parameter_Sweep/ run_analysis.py + sweep/ (7 modules)
    ├── Filter_Sweep/    run_sweep.py, validate.py, plot_results.py, filter_sweep/ (8 modules), decoder/rs_decode.cpp
    ├── Rugosity_Metrics/ (UNTRACKED) roughness_scans.py + roughness/ (10 modules) + tests/
    ├── Height_Analysis/Copy_Trial_Files.sh
    └── view_point_cloud.py
```

Size: ≈ 30 k lines of project-owned source/docs (excluding the per-tool Python virtualenvs `Processing/*/.venv`, `Processing/Crop/cropper/`, `Processing/Viewer/`, which are gitignored and contain third-party site-packages, and excluding `Capture/build/` object files).

## 3. Component inventory and grouping

| # | Group | Component | Lang | Key files | Report file |
|---|---|---|---|---|---|
| 1 | **Capture – core** | D456 recorder library, minimal recorder `rs_record`, `src/main.cpp`, Makefile, launch scripts | C++ / sh | `Capture/include/d456_recorder.hpp`, `Capture/src/*`, `Capture/programs/rs_record/main.cpp`, `Capture/Makefile`, `Capture/run`, `Capture/run_both.sh` | [01](01_capture_core.md) |
| 2 | **Capture – core** | `rs_marker`: AprilTag detection, rig (servo) control over serial, recording, best-frame PLY, IMU, `scan_info.txt` (`trial_info.txt` belongs to depth_quality) | C++ | `Capture/programs/rs_marker/{main.cpp,mountSerial.cpp,mountSerial.h}` | [02](02_rs_marker.md) |
| 3 | **Capture – utilities** | `rec_viewer`, `rgb_viewer`, `rs_clean`, `test_res`, `rgb_test` | C++ | `Capture/programs/*/main.cpp`, `Capture/rgb_test/main.cpp` | [03](03_capture_utilities.md) |
| 4 | **Characterisation** | `depth_quality`: depth-quality metrics; reprocess `.db3` recordings | C++ | `Capture/tools/Depth_Quality/depth_quality.cpp` | [04](04_depth_quality.md) |
| 5 | **Capture – data utilities / visualisation** | collect/tabulate recordings, marker 3D points, IMU plots, video playback, PLY viewer | Python | `Capture/tools/*.py`, `Processing/view_point_cloud.py`, `Capture/More_Tools/*` | [05](05_capture_python_tools.md) |
| 6 | **Processing pipeline** | Crop: manual and automatic AprilTag-bounded crop of PLYs | Python | `Processing/Crop/crop_pointclouds.py`, `rename_ply.sh` | [06](06_crop.md) |
| 7 | **Processing pipeline** | Fuse: multi-angle registration and fusion (`fuse_scans`) with interactive viewer | C++ (PCL) | `Processing/Fuse/src/*` | [07](07_fuse.md) |
| 8 | **Processing pipeline / orchestration** | Bulk crop+fuse over many sessions | Python | `Processing/Bulk_Process_Pipeline/*` | [08](08_bulk_pipeline.md) |
| 9 | **Evaluation** | Evaluate: align scan to analytic/mesh ground truth, accuracy metrics; unit tests | Python | `Processing/Evaluate/**` | [09](09_evaluate.md) |
| 10 | **Experiments** | Parameter_Sweep: evaluate the hardware-parameter sweep, rank settings | Python | `Processing/Parameter_Sweep/**` | [10](10_parameter_sweep.md) |
| 11 | **Experiments** | Filter_Sweep: replay `.db3` through post-processing filter settings; `rs_decode` | Python + C++ | `Processing/Filter_Sweep/**` | [11](11_filter_sweep.md) |
| 12 | **Analysis** | Rugosity_Metrics: roughness metrics (ISO 25178 style) of fused scans; tests (**untracked in git**) | Python | `Processing/Rugosity_Metrics/**` | [12](12_rugosity_metrics.md) |
| 13 | **One-off / housekeeping** | `Copy_Trial_Files.sh`, `.gitignore`s, committed `.pyc`/binaries, untracked build outputs | sh / misc | various | [13](13_misc_scripts.md) |

### Third-party dependencies (from manifests)

| Area | Dependency | Source |
|---|---|---|
| Capture C++ | librealsense2 (README asks for 2.56.x; history shows a move to 2.58 and `.db3`), OpenCV 4 with ArUco (AprilTag 36h11), pthread | `Capture/Makefile:3-18`, `Capture/README.md:7-30`, commit `8f29459` |
| Fuse | PCL ≥ 1.10 (common, io, filters, features, registration, surface, visualization), Threads; Release build forced | `Processing/Fuse/CMakeLists.txt:20-35` |
| Filter_Sweep decoder | librealsense2 (CMake package) | `Processing/Filter_Sweep/decoder/CMakeLists.txt` |
| Evaluate | open3d ≥ 0.19, numpy, scipy, sympy, matplotlib, scikit-image (marching cubes), pytest | `Processing/Evaluate/requirements.txt` |
| Parameter_Sweep | Evaluate's dependencies plus pandas and tqdm | `Processing/Parameter_Sweep/requirements.txt` |
| Filter_Sweep | Evaluate's dependencies plus pyrealsense2 ≥ 2.57, pandas, pyarrow, pyyaml and tqdm | `Processing/Filter_Sweep/requirements.txt` |
| Crop / Bulk | open3d and numpy (from the `cropper` venv, which has no requirements file) | `Processing/Bulk_Process_Pipeline/README.md` |
| Rugosity | Reuses Evaluate's `scan_eval` via `sys.path`; numpy/scipy/matplotlib (no own requirements.txt) | `Processing/Rugosity_Metrics/README.md:3-5` |

### Build systems

- `Capture/Makefile`: a template rule builds each program in `PROGRAMS` from `programs/<name>/*.cpp`, linked with `src/d456_recorder.cpp`. The OpenCV programs add `pkg-config opencv4` flags. Binaries are written next to the Makefile.
- `Capture/run`: a launcher wrapper. The comment says it applies an `LD_PRELOAD` pthread fix, but the script itself only `exec`s the binary; see Observations in [01](01_capture_core.md).
- `Processing/Fuse/CMakeLists.txt`: refuses in-source builds and defaults to Release. The comment gives the reason: "83 s unoptimised, 15 s Release" on a 28-scan session (**EXPLICIT**, `Processing/Fuse/CMakeLists.txt:12-17`).

## 4. Git history: timeline of design decisions

39 commits, 2026‑08‑11 → 2026‑10‑06, single author (Joab / Joab Kloppers). No tags. Working tree has one uncommitted change (`Processing/Evaluate/scan_eval/report.py`: `append_results` gains a `lead_columns` parameter) and the untracked `Processing/Rugosity_Metrics/`.

| Date | Commit | Evidence of design decision |
|---|---|---|
| 08‑11 | `8e07093` | "Added Existing RS_Code, updated to match current open_CV version". The capture code was **inherited** from an earlier project. The `/home/aru/...` paths in `Capture/README.md` and `Capture/run_both.sh`, and the `cpp/` naming, are consistent with that. |
| 08‑12 → 08‑17 | `1c1f7db`, `1230363`, `e9cd7a8` | Filter-setting initialisation, frame waiting condition, AprilTag detector parameters and a **CLAHE** pre-filter added |
| 08‑18 | `09dce55` | "Read through and commented code for understanding" |
| 08‑21 → 08‑24 | `c56364f`, `1324632`, `81094b3`, `e459457`, `dc06e2d`, `513fdf2` | Servo serial link; one folder per angle; settling delays after each move; "Rig working" |
| 08‑27 | `5855c99`, `5b9a5ea` | Capture/Processing restructure; `noTilt` mode; session folder holds all angles |
| 08‑28 | `22174f5`, `f4253ae` | Pipeline start retried to avoid "busy" errors; thermal compensation disabled; occasional segfaults remain |
| 09‑09 | `8f29459` | librealsense upgraded to **2.58**, output switched to **`.db3`**; "no segmentation fault occured yet". This supersedes the README's advice to stay on 2.56. |
| 09‑09 | `fb0207f` | Threshold filter in rs_marker; prompt for rotation sweep; first point-cloud fusion code |
| 09‑10 | `6de8e04` | Fusion overhaul: interactive scan list, exclude/nudge; single fused cloud |
| 09‑11 → 09‑14 | `e26c38a`, `82468b8`, `538a460` | Python crop tool; voxel sizes; hole reduction in fusion; optimisation ("much faster") |
| 09‑15 | `bf56d08`, `9de4ac1`, `c1ce006` | depth_quality tool for characterisation; Fuse build fix (build/ removed from git); crop copies IMU files |
| 09‑21 | `0f33464` | Evaluate tool added ("Untested") |
| 09‑25 | `59b3365`, `0969f45` | Evaluate exports overlap PLYs; rs_marker more robust to harsh lighting and logs candidates with failed ID |
| 09‑28 | `93c5bd9`, `473583a` | depth_quality reprocesses `.db3`; crop auto-crops to the four-AprilTag region using `detections.csv` |
| 10‑01 → 10‑02 | `5354dca`, `36abda2` | Depth units read from `trial_info.txt`; rs_marker gains more CLI parameters and stores its config with each scan |
| 10‑06 | `00b2c4e`, `713bf4b`, `1e31b72` | Parameter_Sweep, Filter_Sweep, Fuse tag alignment; bulk pipeline ("outputs fail, need to debug"; "Cropped scans for small wave 641mm seem to be wrong") |

## 5. Data artefacts referenced but not in the repo

These are **gitignored or external** and were not audited:
- `Capture/Recordings/`
- `Processing/Fused_Scans/`, `Processing/Cropped_Scans/`
- `Processing/Evaluate/References/` (ground-truth STLs)
- the external data disk (`/run/media/joab/Seagate/roughness/...`, referenced in the Parameter_Sweep README)
