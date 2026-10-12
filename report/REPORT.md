# Technical audit: RealSense D456 surface capture, fusion, evaluation and roughness toolchain

**Subject:** `4022/Code` (git submodule of the 4022 thesis repo), `main` @ `1e31b72` (2026‑10‑06) plus the working tree on 2026‑10‑12
**Type:** read-only audit. Nothing in the repository was modified, built, run or tested. Behaviour is derived from reading source, READMEs and git history. Where existing output files on disk were consulted, the chapter says so.
**Evidence convention:** every "why" claim is labelled **[EXPLICIT: source]** (code comment, doc or commit, cited) or **[INFERRED]** (the auditor's reasoning). Code is cited as `path/file:line`, relative to the repo root.

## How to read this report

- **For an overview:** start with [90 — System overview](90_system_overview.md), which has the data-flow diagram, shared formats, design rationale and the consolidated issue list.
- **For inventory and history:** go to [00 — Inventory](00_inventory.md).
- **For depth:** each component chapter follows the same structure: purpose, inputs/outputs, walkthrough, algorithms and maths, why this approach, parameters, limitations, dependencies, observations.
- **For symbols and terms:** see [99 — Glossary](99_glossary.md).

## Table of contents

| # | Chapter | Covers |
|---|---|---|
| 00 | [Inventory](00_inventory.md) | Directory map, component grouping, dependencies, build systems, git timeline of design decisions |
| | **Capture (C++)** | |
| 01 | [Capture core](01_capture_core.md) | `D456Recorder` library, `rs_record`, Makefile, `run`, `run_both.sh`, capture README conventions, `gdb_crash.log` segfault analysis |
| 02 | [rs_marker](02_rs_marker.md) | The main capture tool: AprilTag detection (OpenCV ArUco, lighting-robust passes, candidates), tilt sweep over serial (`mountSerial`), recording, best-frame PLY, IMU, `scan_info.txt` |
| 03 | [Capture utilities](03_capture_utilities.md) | `rec_viewer`, `rgb_viewer`, `rs_clean`, `test_res`, `rgb_test` |
| 04 | [depth_quality](04_depth_quality.md) | Flat-target depth characterisation: fill rate, plane-fit RMS, sub-pixel RMS, Z error, temporal noise; `.db3` reprocessing |
| 05 | [Capture Python tools](05_capture_python_tools.md) | collect_plys, marker_points, summarize_markers, tabulate_data, plot_imu, plot_gyro_outliers, play_video, view_point_cloud (×2), More_Tools |
| | **Processing pipeline** | |
| 06 | [Crop](06_crop.md) | Manual and automatic AprilTag-bounded cropping (tag corners → bed plane → prism), `_tags.csv` sidecar, `rename_ply.sh` |
| 07 | [Fuse](07_fuse.md) | `fuse_scans` (C++/PCL): load, tilt calibration, rotation direction, coarse alignment (rig seed / FPFH+RANSAC / tags), point-to-plane ICP, weighted fusion, MLS, interactive viewer |
| 08 | [Bulk pipeline](08_bulk_pipeline.md) | Batch crop and fuse over sessions, plus an analysis of the known failures |
| | **Evaluation and experiments** | |
| 09 | [Evaluate](09_evaluate.md) | `scan_eval`: analytic, mesh and plugin references; safe expression parser; multi-start scored registration; exact distances; accuracy and completeness metrics; tests |
| 10 | [Parameter sweep](10_parameter_sweep.md) | Resolution × depth units × preset sweep on two objects, with ranking and plots |
| 11 | [Filter sweep](11_filter_sweep.md) | Replaying `.db3` through librealsense post-processing filters (`rs_decode` + software device); validation; ranking |
| 12 | [Rugosity metrics](12_rugosity_metrics.md) | ISO 25178 areal roughness, rugosity, ACF, PSD, Hurst and fractal dimension, multiscale, bootstrap; synthetic validation (**untracked in git**) |
| 13 | [Misc and repository hygiene](13_misc_scripts.md) | `Copy_Trial_Files.sh`, `.gitignore` rules, committed artefacts, untracked build outputs |
| | **Synthesis** | |
| 90 | [System overview](90_system_overview.md) | End-to-end data flow (Mermaid), shared data structures, interactions, configuration, rationale, strengths and weaknesses, consolidated observations, open questions |
| 99 | [Glossary](99_glossary.md) | Hardware, file formats, vision and registration terms, evaluation metrics, roughness parameters |

## Key findings at a glance

1. **The pipeline:**
   - `rs_marker` captures multi-tilt scans with AprilTags.
   - Crop isolates the tagged region.
   - Fuse registers the scans with a score-don't-trust multi-hypothesis strategy and fuses them.
   - Evaluate scores accuracy against exact references.
   - The sweeps rank hardware settings and filter settings.
   - Rugosity_Metrics computes roughness descriptors.
2. **Most reusable assets.**
   - Evaluate's `scan_eval` is reused by three other tools.
   - Crop's `compute_auto_crop` is reused by three others.
   - The Processing READMEs document the design rationale well.
3. **Main risks.**
   - Untracked Rugosity_Metrics plus an uncommitted dependency in Evaluate.
   - Reference STLs and data are not in the repo.
   - Capture documentation is out of date (`.bag`, librealsense 2.56, "fully post-processed").
   - Unresolved recorder segfault.
   - Known bulk-pipeline failures; likely causes are in chapter 08 §9.
4. **Coordinate convention to remember:** rs_marker PLYs are in the colour optical frame stored as **(x, −y, −z)** in metres, and the `.db3` recordings are **ROS1 bags** despite the extension.
