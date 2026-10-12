# 08 — Bulk crop + fuse pipeline (`Processing/Bulk_Process_Pipeline`)

**Summary.** `bulk_process.py` is a Python orchestrator that finds every capture *session* under a source directory, crops every scan with the AprilTag auto-crop from `Processing/Crop/crop_pointclouds.py` (imported as a module, preview window off by default), caches the crops per session keyed on the crop settings, and then runs the C++ `fuse_scans` binary headless (`--no-gui`) on each cropped session, logging its output and writing a per-session `summary.csv` and a `run_config.json`. Fusion flags come from a TOML config (`[fuse.params]`, one key per `fuse_scans` flag) and/or raw arguments after `--`. The two most recent commits say the pipeline's *"outputs fail, need to debug"* (`713bf4b`) and that *"Cropped scans for small wave 641mm seem to be wrong, causing problems"* (`1e31b72`). Section 9 lists plausible causes found by code reading only; all are labelled **[INFERRED]**. Nothing was run.

## Files covered

| Path | Lines | Role |
|---|---|---|
| `Processing/Bulk_Process_Pipeline/bulk_process.py` | 481 | Orchestrator: discovery, config, crop step (via Crop module), fuse step (subprocess), summary |
| `Processing/Bulk_Process_Pipeline/README.md` | 94 | Usage and behaviour |
| `Processing/Bulk_Process_Pipeline/pipeline_config.example.toml` | 41 | Example `[crop]` / `[fuse]` / `[fuse.params]` config |
| (context) `Processing/Crop/crop_pointclouds.py` | 1231 | Crop functions called (see 06) |
| (context) `Processing/Fuse/src/*` | — | `fuse_scans` behaviour (see 07) |

## Contents

1. Purpose · 2. Inputs and outputs · 3. Walkthrough · 4. Algorithms and methods · 5. Why this approach · 6. Parameters · 7. Assumptions and limitations · 8. Dependencies · 9. Why outputs may fail / why small-wave 641 mm crops look wrong (INFERRED) · 10. Observations

---

## 1. Purpose

Turn many raw sessions (e.g. several objects × standoff distances such as `863mm`, `641mm`) into fused clouds without opening a window per scan. *"Neither tool is reimplemented or modified. The crop step imports `../Crop/crop_pointclouds.py` and calls its automatic crop without the preview window. The fuse step runs `../Fuse/build/fuse_scans`."* **[EXPLICIT: README 11-13]**. Added in commit `713bf4b` (2026-10-06) **[EXPLICIT]**. It sits after capture and replaces the manual chain *crop tool → (rename) → fuse_scans* (06, 07); its outputs feed the evaluation tools.

## 2. Inputs and outputs

### 2.1 CLI (`parse_args`, `bulk_process.py:328-364`)

Everything after a literal `--` is split off before argparse and passed verbatim to `fuse_scans` (329-333).

| Argument | Default | Effect |
|---|---|---|
| `source` | required | Directory searched recursively for sessions, or a single session |
| `output` | required | Root for `fused/`, `cropped/`, `logs/`, `summary.csv`, `run_config.json`; must not be the source or inside it (372-373) |
| `--config <toml>` | none | `[crop]`, `[fuse]`, `[fuse.params]` |
| `--sessions 'glob,glob'` | all | `fnmatch` filter on session folder names (393-395) |
| `--cropped-dir <dir>` | `<output>/cropped` | Shared crop cache root |
| `--crop-margin`, `--frame-window`, `--tag-order 6,5,2,4` | from config | Crop overrides (= crop tool's `--margin`, `--frame-window`, `--tag-order`) |
| `--on-crop-fail skip|keep|manual` | `skip` | Auto-crop failure policy |
| `--review-crops` | off | Show the crop preview per scan |
| `--recrop` | off | Ignore cached crops |
| `--fuse-bin <path>` | `../Fuse/build/fuse_scans` | Binary |
| `--fuse-gui` | off | Run `fuse_scans` with its viewer (omit `--no-gui`) |
| `--crop-only`, `--skip-fused`, `--dry-run`, `-v` | off | Control flags |

### 2.2 Config file (`load_config`, 171-197; example TOML)

| Section / key | Example value (current) | Meaning |
|---|---|---|
| `[crop] margin` | `10.0` | px the crop quad is pulled in (code default 0.0, line 127) |
| `[crop] frame_window` | `15` | ± frames for missing tags |
| `[crop] tag_order` | commented | learned per session |
| `[crop] on_fail` | `"skip"` | skip / keep / manual |
| `[crop] review` | `false` | preview per scan |
| `[fuse] binary` | commented | relative to the config file |
| `[fuse] gui` | `false` | |
| `[fuse.params]` | `tag-align = true`, `scan-voxel = 0.0005`, `fusion-cell = 0.0005` (uncommented by `1e31b72`); `rig-config` and all others commented | each key → `--key value`; `true` → bare flag; `false` → omitted; list → repeated flag (`FuseSettings.args`, 149-168); `rig-config` resolved relative to the config's folder (155-156) |

Unknown sections/keys abort with an error (178-185, 194-196); `[crop]` keys may use `-` or `_` (183).

### 2.3 Source layout and discovery

`<source>/.../<Session>/<Session>_r<deg>/<Session>_r<deg>_a<deg>/*.ply` (docstring 5). A **session** is a directory with at least one child matching `_r(-?\d+)(?:_|$)` that contains a subdirectory holding a `.ply` (`is_session`, 80-87). `find_sessions` (90-98) recurses and stops descending at a session. `session_scans` (101-118) picks one PLY per tilt folder: `<tilt>.ply` if present, else the only `.ply`, else the folder is skipped with a warning (consistent with `fuse_scans`' own rule, `scan_discovery.cpp:21-53`).

### 2.4 Outputs (README 55-66; docstring 15-24)

| Path | Content |
|---|---|
| `<out>/cropped/<rel>/<Session>/<rot>/<tilt>/<tilt>_cropped.ply` + `imu_a*.csv`, `<tilt>_tags.csv`, `*_detections.csv`, `scan_info.txt` | crop tool output layout |
| `<out>/cropped/<rel>/<Session>/crop_settings.json` | cache marker: `{settings: cache_key, complete: true, source, scans: [{scan,status,note,points}]}` (284-285) |
| `<out>/fused/<rel>/<Session>.ply` | `fuse_scans` output |
| `<out>/logs/<rel>/<Session>_fuse.log` | the command line and the full combined stdout/stderr of `fuse_scans` |
| `<out>/summary.csv` | one row per session: `session, scans, cropped_auto, cropped_manual, uncropped, skipped, fused, status, seconds, skipped_scans` (423-424), rewritten after every session |
| `<out>/run_config.json` | source, start time, crop cache key, cropped dir, fuse binary, exact fuse args, config path (409-414) |

`<rel>` = the session's parent relative to the source, so sessions of different objects/distances do not collide (README 65-66) **[EXPLICIT]**.

## 3. Walkthrough (`main`, 367-477)

1. Parse args; validate source/output (368-373).
2. `load_config`; apply CLI overrides to `CropSettings` (376-380); `review` is OR-ed (380); validate `on_fail` and `tag_order` length (381-384); fuse binary override, GUI flag, extra args (385-388); require the binary unless `--crop-only` (389-390).
3. `find_sessions`, optional glob filter (392-397); choose crop root (399); print the fuse arguments (401); `--dry-run` lists sessions with scan counts and target paths, then returns (402-406).
4. Write `run_config.json` (408-414).
5. For each session (418-470):
   1. `rel_parent`, label, timer, summary row (419-424).
   2. `crop_session(session, cropped_dir, crop, args.recrop)` (426-428):
      * **Cache hit** (242-250): if not forced and `crop_settings.json` exists with identical `settings` and `complete: true`, reuse the stored per-scan results.
      * **Folder guard** (251-256): if the folder exists but has **no marker** and is non-empty → `RuntimeError` ("wasn't written by this pipeline"); otherwise delete and recreate it.
      * `learn_tag_layout(session, tag_order)` (259-263) — corner-tag order/aspect learned from the session's detections (06 §3.4).
      * For each scan from `session_scans` (265-282): `crop_one` → write cropped cloud as float32 PLY, copy IMU, write tag sidecar from the **original** cloud, copy detections/`scan_info.txt` (272-280).
      * Write the marker (284-285).
      * `QuitAll` (Esc in a window) stops the whole run (429-431); `RuntimeError` marks the session "not cropped" (432-435).
   3. Count statuses into the row (436-440); `usable = scans − skipped` (441).
   4. Status: crop error → keep; `--crop-only` → "cropped"; `usable < 2` → "not fused" (README: *"`fuse_scans` won't fuse fewer"* **[EXPLICIT: README 83]**, matching `main.cpp:426-430`); else fuse (unless `--skip-fused` and the PLY exists) (443-462).
   5. Rewrite `summary.csv` (464-470) — *"so a long run that dies part way still has one"* **[EXPLICIT: 466]**.
6. Final summary print (472-477).

### `crop_one` (204-236)

Read the PLY with Open3D; empty → skipped. Build a `SimpleNamespace(frame_window, margin)` that impersonates the crop tool's argparse result (209) and call `crop_tool.compute_auto_crop(pcd, ply, session, layout, args)`. On `AutoCropError`: `keep` → uncropped original; `manual` → `crop_tool.manual_crop`; else skipped (212-218). On success: print notes, `select_by_index(mask)`; if `review`, show `preview_auto_crop` with the same Enter/M/N/Esc semantics as the crop tool (227-236).

### `fuse_session` (293-321)

Command: `[fuse_bin, -o, out_ply, --sessions, cropped_dir] + fuse.args()` (295), with `--no-gui` appended unless GUI is requested and the `--` extras appended last so they override (166-168; README 51-53). `subprocess.Popen` with stdout+stderr merged, line-buffered text; each line goes to the log, the last 15 lines are kept for the console (301-311). Success = exit code 0 **and** the PLY exists (313-314). On failure the tail is printed and a PLY left by a non-zero exit is deleted *"don't leave a result that looks valid"* **[EXPLICIT: 319-320]**.

## 4. Algorithms and methods

The script itself contains no geometric algorithms; it delegates to:

* **AprilTag-bounded auto crop** (06 §4): reprojection with librealsense Brown–Conrady, pixel-grid intrinsics check, tag-quad completion (parallelogram / rectangle extrapolation), RANSAC planes (Fischler & Bolles 1981), prism crop along the bed normal; tag sidecar via SVD planes and ray–plane intersection.
* **Fusion** (07): IMU-measured tilt seeds, tilt pivot search, rotation-direction search, AprilTag Umeyama / axis-constrained Procrustes poses, FPFH + prerejective RANSAC, point-to-plane ICP, weighted trimmed-mean reconciliation, MLS.

Pipeline-level techniques:

* **Content-addressed crop cache**: the cache key is the dict `{version: CROP_VERSION, margin, frame_window, tag_order, on_fail, review, suffix}` (134-138); equality of this dict plus a `complete` flag decides reuse. It does **not** include the crop tool's source version, the input files' timestamps, or the session path.
* **Config-to-CLI mapping** (149-168): a generic translation of TOML key/value pairs into flags so that new `fuse_scans` flags need no pipeline change **[INFERRED from design]**.

## 5. Why this approach

| Decision | Rationale | Label |
|---|---|---|
| Import the crop module and call the binary instead of re-implementing | single source of truth for both tools | **[EXPLICIT: README 11-13]** |
| Cache crops keyed on settings | *"re-run with different fusion parameters without cropping again"*; `--cropped-dir` to share one cache between output folders | **[EXPLICIT: README 77-82]** |
| Refuse to clear unmarked folders | *"Only clear folders this pipeline made: --cropped-dir may point at hand-made crops"* | **[EXPLICIT: 252]** |
| Default `on_fail = skip` | an un-cropped scan would bring the whole rig/background into fusion | **[INFERRED]** |
| Delete PLY on non-zero exit | avoid half-valid outputs | **[EXPLICIT: 320]** |
| Rewrite summary every session | partial runs still summarised | **[EXPLICIT: 466]** |
| Headless fusion by default | batch use | **[EXPLICIT: README 51-53]** |

## 6. Parameters and tuning

| Name | Code default | Example config | Location | Effect |
|---|---|---|---|---|
| `CropSettings.margin` | 0.0 px | 10.0 | 127; toml 8 | inward shrink of the tag quad (see 06 Obs. 3: applied along the diagonal) |
| `frame_window` | 15 | 15 | 128 | missing-tag window |
| `tag_order` | None (learned) | commented | 129 | fixes slot order |
| `on_fail` | `skip` | `skip` | 130 | failure policy |
| `review` | False | false | 131 | preview windows |
| `suffix` | `_cropped` | — | 132 | output name |
| `CROP_VERSION` | 1 | — | 69 | manual cache invalidation knob (*"Bump when the crop step changes"*) |
| `FuseSettings.binary` | `../Fuse/build/fuse_scans` | commented | 60, 143 | |
| `fuse.params.tag-align` | (fuse default off) | **true** | toml 25 | AprilTag hypothesis (needs the sidecars this pipeline writes) |
| `fuse.params.scan-voxel` | (fuse default 0.003) | **0.0005** | toml 28 | load-time thinning; ceiling on detail |
| `fuse.params.fusion-cell` | (fuse default 0.003) | **0.0005** | toml 30 | height-field cell; output point count ∝ 1/cell² |
| `fuse.params.rig-config` | (none) | **commented out** | toml 24 | without it `fuse_scans` has **no rotation or tilt axis** |
| others (`tag-min-shared`, `tag-max-rmse`, `fusion-mode`, `fusion-voxel`, `fusion-min-scans`, `fusion-trim`, `mls-radius`, `no-*`, `tilt-pivot-range`) | fuse defaults | commented | toml 26-41 | see 07 §6 |

## 7. Assumptions, limitations, failure modes

* Requires **Python ≥ 3.11** (`tomllib`, line 49) and the Crop venv (open3d, numpy) (README 17-21).
* Imports `crop_pointclouds` by inserting `../Crop` into `sys.path` (62-64); any change to that module's function signatures breaks the pipeline silently at run time.
* `crop_one` catches only `AutoCropError`; any other exception (corrupt PLY, malformed IMU CSV → `KeyError`, Open3D errors) propagates out of `crop_session` — it is not a `RuntimeError`, so it is **not** caught at 432 and aborts the whole run, leaving a partially written crop folder without a marker.
* Crop cache trusts the marker blindly: deleted/edited cropped files, a changed crop tool, or changed raw data are not detected; only `CROP_VERSION` or `--recrop` invalidates.
* "Success" of fusion = exit 0 and a file exists; alignment quality is not checked (the log holds the tables).
* No timeout on `fuse_scans`; a pathological parameter set can run for hours **[INFERRED]**.
* `on_fail = manual` and `review` open Open3D windows from a "bulk" run; Esc aborts everything.
* Session detection is purely name-based (`_r<digits>` children); stray folders matching that pattern could be mistaken for sessions **[INFERRED]**.

## 8. Dependencies

| Dependency | Use |
|---|---|
| `crop_pointclouds` (internal) | `learn_tag_layout`, `compute_auto_crop`, `AutoCropError`, `QuitAll`, `manual_crop`, `preview_auto_crop`, `write_ply_like_original`, `copy_imu_file`, `write_tag_sidecar`, `copy_scan_metadata` |
| `fuse_scans` (internal, C++) | invoked via `subprocess.Popen` |
| Open3D | `io.read_point_cloud`, `select_by_index`, `get_axis_aligned_bounding_box` |
| NumPy | `np.flatnonzero(mask)` |
| stdlib | `tomllib` (TOML parse), `argparse`, `fnmatch` (session filter), `json` (marker, run config), `csv` (summary), `shlex` (log/console quoting), `shutil.rmtree`, `subprocess`, `types.SimpleNamespace`, `dataclasses`, `re`, `time` |

## 9. Why outputs may fail, and why "small wave 641 mm" crops may be wrong

Context: commit `713bf4b` — *"Added bul process code to automatically crop and fuse bulk scans. outputs fail, need to debug"*; commit `1e31b72` — *"changed bulk process config file. Cropped scans for small wave 641mm seem to be wrong, causing problems"*, whose only diff uncomments `tag-align = true`, `scan-voxel = 0.0005`, `fusion-cell = 0.0005` in the example TOML **[EXPLICIT: git show 1e31b72]**. The config actually used (e.g. `my_pipeline.toml`), the run logs and the 641 mm data are not in the repository, so the following **cannot be confirmed**; each item is a plausible mechanism visible in the code. All are **[INFERRED]**.

### 9.1 Pipeline / fusion failures

1. **Stale crop folder blocks re-runs (deterministic bug).** If a crop run for a session is interrupted after the folder is created — any non-`AutoCropError` exception in `crop_one`, a crash, Ctrl-C, or Esc (`QuitAll`) — the folder exists without `crop_settings.json`. Every later run then raises `RuntimeError("… exists and wasn't written by this pipeline")` at line 254 and marks the session "not cropped", **including with `--recrop`** (force only bypasses the cache check, 243, not the guard, 253). The user must delete the folder by hand. This would show up exactly as "outputs fail" for sessions that previously crashed.
2. **No rotation geometry in the example config.** `rig-config` is commented out (toml 24) and no `--rotation-axis`/`--tilt-axis` is passed, so in `fuse_scans` `rotation_axis = None` (07 §4.2): no rotation seeds, the rotation-direction stage and the tag-measured group angles are skipped (`fusion_session.cpp:1574-1578`), and scans of the r45/r90/r135 groups get only an IMU tilt seed (if tilt calibration succeeds) or no seed at all. A tilt-only seed for a group really turned by 45–135° can still pass the 15 % nearest-neighbour coverage test on a near-flat cropped square and be **accepted as "seed" without trying FPFH** (`fusion_session.cpp:720-730`); only a better-scoring tag pose can replace it. Groups that the tag chain cannot reach would be fused at the wrong orientation — the README itself warns that such a group *"ends up fused in two orientations"* (README 471-473). Fuse README recommends the rig config (README 133-145, 147-157).
3. **0.5 mm scan voxel is below the sensor's point spacing** (*"a single scan's point spacing (about 1 mm here)"*, Fuse README 750-751), so `VoxelGrid` barely thins the clouds; all neighbourhood operations whose radii are fixed in metres then see far more neighbours — normals (r = 15 mm), FPFH (r = 30 mm, only when a seed fails), the off-surface filter, and MLS (r = 10 mm) — and the per-point fusion samples and hash-map cells grow ~36× relative to the 3 mm default (Fuse README: *"Costs memory and time roughly as 1/m²"*, 736-737; commit 82468b8: *"fusese take along time"*). Plausible outcomes: very long runs, memory exhaustion / process kill (non-zero exit → bulk deletes the PLY and reports failure). In addition, PCL's `VoxelGrid` passes the input through unfiltered when (extent/leaf)³ overflows a 32-bit index (PCL library behaviour, not in this repo); with a 0.5 mm leaf this is reached by any cloud whose bounding box is about 1.1 m per side, e.g. an `on_fail = keep` uncropped scan or a crop column containing far outliers (the prism crop has no height limit, 06 §7).
4. **0.5 mm fusion cells finer than point spacing** make most height-field cells hold ≤ 1 sample per scan, so corroboration and the trimmed mean have little to work with and the output density becomes uneven — the hole/sparsity issue the Fuse README and commits `e26c38a`/`82468b8` describe. This degrades quality rather than causing a non-zero exit.
5. **Skipped scans weaken the session.** With `on_fail = skip`, scans whose crop fails are omitted; if the 0° scan of a group (or all scans sharing tags with the reference) is skipped, the reference changes or tag chains break, making 9.1-2 more likely. If fewer than 2 remain, the session is "not fused" by design.
6. **Mixed crop semantics.** When the surface plane cannot be fitted, `compute_auto_crop` silently falls back to the camera-frustum crop (06 §3.3 step 11). Bulk never shows the preview by default, so frustum-cropped and prism-cropped scans of the same session are fused together, with boundaries that differ by view angle.

### 9.2 Why the 641 mm small-wave crops could be wrong

1. **Wave relief inside the plane band.** The bed plane is fitted to a band only 25 px (≈18 px perpendicular, 06 Obs. 3) inside the inner tag corners (`PLANE_BAND_PX`, `crop_pointclouds.py:112, 727`). If the wave sample extends to the edge of the tag square — more likely at 641 mm where the square fills more of the image — the RANSAC plane (5 mm inlier distance) can lock onto crests or a tilted compromise through the waves. Its normal sets the direction of the crop column (`prism_crop`, 676-683), so the kept region is sheared by roughly $h\tan\theta$ at height $h$, cutting one side of the wave and including background on the other.
2. **Tags at or beyond the image/depth edge at the closer standoff.** At 641 mm a ~600 mm tag square (`fusion_pipeline.h:144` mentions "~600 mm between tags") occupies most of the colour frame; with ±15° tilt a tag can leave the image or fall outside the region where aligned depth exists (after align-to-colour the depth FOV does not cover the full colour frame on a D456 — sensor-specific, not verifiable here). Consequences: (a) corners extrapolated by `complete_square` using image-space aspect ratios learned from other poses (`learn_tag_layout`, 456-469), which are biased under tilt perspective; (b) no depth under a tag → tag plane falls back to the bed plane, misplacing the column corners if the tags are raised; (c) fewer tags in the sidecar for fusion.
3. **Too few decoded tags → geometry guessing.** If no frame of the 641 mm session has exactly four decoded tags, there is no learned order (`learn_tag_layout` returns empty) and slots are guessed from positions; with two tags the "same side vs diagonal" decision uses `side_ratio`, which is also unknown then, so it defaults to "same side, square towards image centre" (578-583) — a wrong guess produces a crop of the wrong half of the scene.
4. **Undecoded false squares.** The wave texture/shadows may produce undecoded square candidates; they are only used when decoded IDs are insufficient, but then `_drop_interior` and size matching are the only safeguards (549-561, 614-616).
5. **Minimum range / hole filling.** At 641 mm, tilted views put wave crests closer to the camera; regions near the D4xx minimum depth produce holes, and `rs_marker`'s hole-filling filter (`rs_marker/main.cpp:747-752`) fills them with neighbour depth. Filled or missing depth near the tags and band can bias both RANSAC planes.
6. **Margin 10 px** in the example config is applied in pixels; at a closer standoff each pixel is smaller on the object, but the effect is a few millimetres and unlikely to look "wrong" on its own.
7. **Cache masking fixes.** If the crop code was changed after a 641 mm session had been cropped (the crop module was modified in `00b2c4e` on the same day), the old crops are reused because the cache key does not include the code version and `CROP_VERSION` stayed 1.

Suggested checks (diagnostic only): run with `--review-crops` on the 641 mm session; inspect the printed notes per scan ("slots …", "extrapolated N missing corner(s)", "crop is a column standing on the surface (tags X mm above it …)" vs "couldn't fit the tag surface"); check `edge_m` in `_tags.csv` against the printed tag size; and read `logs/…_fuse.log` for `[rotation] no rotation axis configured`, `via seed/fpfh/tags`, and `[!] tags put … deg away` warnings.

## 10. Observations

1. **Unrecoverable "not written by this pipeline" state** after any interrupted crop, not cleared by `--recrop` (lines 243-256). See 9.1-1.
2. **Narrow exception handling**: only `AutoCropError`, `QuitAll` and `RuntimeError` are handled; other crop errors abort the whole multi-session run.
3. **Cache key omits code version and inputs** (134-138); `CROP_VERSION` must be bumped by hand.
4. **Example config contradicts the README**: the README's example enables `rig-config` (README 45), the TOML ships with it commented out, and the header says *"Anything not set uses fuse_scans' own default (shown here)"* (toml 20-21) although `scan-voxel` and `fusion-cell` are now set to non-default 0.0005 values, not "shown defaults".
5. **Example values fight the Fuse tool's own guidance**: 0.5 mm scan voxel and cell are below the ~1 mm point spacing the Fuse README cites, and `mls-radius` stays at 10 mm (README advises lowering it when going denser, 752-754).
6. **No timeout / resource guard** on the subprocess; `bufsize=1` line buffering relies on `fuse_scans` flushing (it uses `std::cout`/`std::cerr`, flushed on newline only when attached to a TTY — through a pipe the log may appear in large bursts) **[INFERRED]**.
7. **Exit code 0 with poor registration counts as success**; no parse of `fuse_scans`' quality tables into `summary.csv`.
8. `usable` counts `uncropped` scans as usable (441), so `on_fail = keep` mixes uncropped and cropped clouds in one fusion without warning.
9. The pipeline writes **tag sidecars from the original cloud** but `fuse_scans` will also accept `*_detections.csv` + `scan_info.txt` when the sidecar is missing; if the sidecar writer skipped (e.g. intrinsics mismatch), fuse would try to locate tags in the *cropped* cloud, where they were cut away, and find none.
10. `session_scans` is duplicated logic from `scan_discovery.cpp` (one PLY per tilt folder); the two can drift.
11. `.pyc` for the crop module was committed alongside this pipeline (`713bf4b` stat shows `crop_pointclouds.cpython-311.pyc`).
