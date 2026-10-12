# 13 — Miscellaneous scripts, ignore rules and repository hygiene

**Summary.** This file covers the small items that do not belong to a pipeline component:
* the shell utility `Processing/Height_Analysis/Copy_Trial_Files.sh`;
* the `.gitignore` files (root and per-folder) and the gaps between them and what is actually committed or left untracked;
* committed build artefacts: `.pyc` files, the `depth_quality` ELF binary and a GDB crash log;
* the untracked compiled Capture programs;
* editor/agent folders.

Everything comes from reading files and `git log`/`git ls-files`/`file` output. Nothing was executed.

### Files covered

| Path | Lines / type | Role |
|---|---|---|
| `Processing/Height_Analysis/Copy_Trial_Files.sh` | 98, bash | Collect `*.csv` and `trial_info.txt` from a trial tree |
| `.gitignore` | 11 | Root ignore rules |
| `Processing/Evaluate/.gitignore` | 8 | venv, caches, outputs |
| `Processing/Filter_Sweep/.gitignore` | 6 | venv, cache, results, build |
| `Processing/Fuse/.gitignore` | 6 | CMake build dirs |
| `Processing/Parameter_Sweep/.gitignore` | 3 | venv, cache, results |
| `Processing/Rugosity_Metrics/.gitignore` | 5 (untracked) | caches, `roughness_out/` |
| `Processing/Crop/__pycache__/crop_pointclouds.cpython-31{1,2,3}.pyc` | committed bytecode | Build by-product |
| `Capture/tools/Depth_Quality/depth_quality` | committed ELF x86-64, 281 KB | Build output of `depth_quality.cpp` |
| `Capture/gdb_crash.log` | committed, 2017 lines | Debug session log |
| `Capture/{rec_viewer,rgb_viewer,rs_clean,rs_marker,rs_record,test_res}` | untracked ELF x86-64 | `Capture/Makefile` outputs |
| `.vscode/`, `.claude/` | untracked | Editor / agent settings |

### Contents
1. Copy_Trial_Files.sh
2. Ignore rules
3. Committed artefacts
4. Untracked binaries and folders
5. Observations

---

## 1. `Processing/Height_Analysis/Copy_Trial_Files.sh`

**1. Purpose.** The script copies (or moves) the lightweight metadata of a characterisation dataset, every `*.csv` and `trial_info.txt`, out of a trial directory tree on the external drive. It never touches the large `.db3` recordings (header, lines 3-13). The examples use `/run/media/joab/Seagate/roughness/Joab_Characterisation/Flat_Error` (lines 19-21), so it supports the flat-plate depth-error trials whose `trial_info.txt` is written by the Depth_Quality tool. The folder contains only this script. `Height_Analysis/.venv/` and `Height_Analysis/Plots/` are ignored by the root `.gitignore` (lines 6-7), but no Python analysis code was ever committed there. `git log` shows one commit, `c9f142d`. So the analysis that consumed these files is not in the repository **[INFERRED]**.

**2. Inputs and outputs.**

| Argument | Meaning |
|---|---|
| `SRC_DIR` (positional 1) | Root to search (must exist, line 59) |
| `DEST_DIR` (positional 2) | Destination, created with `mkdir -p` (line 67) |
| `--flatten` | Write `DEST/<parentdir>__<file>` instead of mirroring the tree |
| `--dry-run` | Print the actions only |
| `--move` | `mv` instead of `cp` |
| `-h/--help` | Print the header comment (`grep '^#' "$0"`) |

**3. Walkthrough.**
1. `set -euo pipefail` (line 23).
2. Parse the arguments in any order (lines 32-52). A third positional argument is an error.
3. Strip the trailing slash from `SRC_DIR` (line 65).
4. Run `find SRC -type f \( -iname "*.csv" -o -iname "trial_info.txt" \) -print0` and read the results NUL-safely (`read -r -d ''`, lines 77, 96).
5. For each file, compute the relative path (`${file#"$SRC_DIR"/}`) and the target (flattened or mirrored, lines 78-87). Then echo the action, or perform it, and count it (lines 89-95).

**4. Methods.** Plain `find`/`cp`/`mv`. Matching is case-insensitive, and NUL delimiters make it robust to spaces in names.

**5. Why.** Mirroring the tree avoids collisions, because every trial has a file called `trial_info.txt`. `--flatten` prefixes the parent folder instead **[EXPLICIT: lines 9-13]**. `.db3` files are excluded by construction **[EXPLICIT: lines 7, 74-76]**.

**6. Parameters.** None beyond the flags.

**7. Limitations.**
* `--dry-run` still creates `DEST_DIR` (line 67), and in mirror mode every target sub-directory too (line 86), because the `mkdir -p` calls run before the dry-run check.
* `--flatten` can still collide when two trials in different branches share a parent folder name. `cp`/`mv` then silently overwrite.
* If `DEST_DIR` lies inside `SRC_DIR`, `find` may pick up files already copied.
* `*.csv` matches every CSV, including detections and IMU logs, not just trial results.
* `--help` also prints the shebang line.
* The header calls the script `copy_trial_files.sh`, while the file is `Copy_Trial_Files.sh`.

**8. Dependencies.** bash, GNU findutils, coreutils.

## 2. Ignore rules

Root `.gitignore` (history: 5855c99 → 538a460 → c9f142d → 03de144 → 67d82cc → 00b2c4e):

```
Capture/Recordings/                     Processing/Height_Analysis/Plots/
Capture/tools/Depth_Quality/dq_runs/    Processing/Viewer/
Processing/Fused_Scans/                 Processing/Evaluate/.venv/
Processing/Cropped_Scans/               Processing/Evaluate/References/
Processing/Crop/cropper/                Processing/Filter_Sweep
Processing/Height_Analysis/.venv/
```

The per-folder files are consistent and well commented. Evaluate ignores `.venv/`, `__pycache__/`, `*.pyc`, `.pytest_cache/`, `evaluation_out/` and `exports/`. Fuse ignores `build/`, `build-*/`, `CMakeCache.txt` and `CMakeFiles/`, with the comment that build output "contains absolute paths". Parameter_Sweep ignores `.venv/`, `__pycache__/` and `results/`. Filter_Sweep ignores `.venv/`, `cache/`, `results/`, `results_*/`, `decoder/build/` and `__pycache__/`. Rugosity_Metrics ignores `__pycache__/`, `*.pyc`, `.pytest_cache/` and `roughness_out/`.

Gaps:
* There is **no repository-wide** `__pycache__/` / `*.pyc` rule. Crop has no folder `.gitignore`, which is how the Crop `.pyc` files got committed.
* There is **no rule for `Capture/build/` or the Capture program binaries**, which therefore show as untracked.
* `Processing/Filter_Sweep` (added in 00b2c4e, no trailing slash) ignores the **entire** folder even though its sources are tracked. New files there would be invisible to `git status`. See 11_filter_sweep.md, Observation 1.
* `Processing/Evaluate/References/` (00b2c4e) excludes the STL ground-truth meshes that Filter_Sweep (`sweep_config.yaml:31-33`) and Evaluate need. A fresh clone cannot reproduce the accuracy results without them.

## 3. Committed artefacts

* **`Processing/Crop/__pycache__/crop_pointclouds.cpython-311/312/313.pyc`**. These are bytecode for three Python versions. They were added in e26c38a and 473583a, then updated in 00b2c4e and 713bf4b. They change whenever the cropper is imported (e.g. by Filter_Sweep or Parameter_Sweep), which creates noisy diffs. They should be removed and ignored.
* **`Capture/tools/Depth_Quality/depth_quality`**. This is an ELF 64-bit x86-64 dynamically linked executable, not stripped, 281 376 bytes. It is committed alongside `depth_quality.cpp` and updated in bf56d08, c9f142d, 93c5bd9 and 5354dca. As a build output it is machine- and library-version-specific (librealsense), and each rebuild adds a ~280 KB blob to the history.
* **`Capture/gdb_crash.log`** (2017 lines, added in 00b2c4e). This is a GDB session transcript, presumably from the librealsense recorder segfault investigation **[INFERRED]**. It is a debugging artefact rather than source.

## 4. Untracked binaries and folders

* `Capture/rec_viewer`, `rgb_viewer`, `rs_clean`, `rs_marker`, `rs_record`, `test_res`. All are ELF 64-bit x86-64 dynamically linked and not stripped. They were built 2026-10-08 09:31 and range from 97 KB (`rs_record`) to 261 KB (`rs_marker`). These are exactly the `PROGRAMS` of `Capture/Makefile:6`, linked into the Capture root by the `$(1):` rule (`-o $$@`, Makefile:46-47). `Capture/build/` holds their object files. They are build outputs and should be ignored, not committed.
* `Processing/Filter_Sweep/decoder/build/rs_decode` (ELF, with debug_info) is ignored by its folder `.gitignore`.
* `.vscode/settings.json` holds one key, `cmake.sourceDirectory`, set to an absolute path for `Processing/Fuse`. It is machine-specific.
* `.claude/` holds Claude Code agent settings (`settings.local.json`) and a lock file. It is local tooling state, not project content. Its contents are not reproduced here.
* `Processing/Rugosity_Metrics/` is untracked (see 12_rugosity_metrics.md).

## 5. Observations

1. Add root rules for `__pycache__/`, `*.pyc`, `Capture/build/` and the six Capture binaries (or have the Makefile write them to `build/bin/`). Remove the committed `.pyc` files and the `depth_quality` binary with `git rm --cached`.
2. Replace the root `Processing/Filter_Sweep` line with nothing. The folder's own `.gitignore` already covers its outputs.
3. Decide how the reference STLs are distributed: commit them (if licensing and size allow), use Git LFS, or document where to get them. At present the accuracy pipeline is not reproducible from the repository alone.
4. The `Height_Analysis` ignore entries refer to a `.venv` and `Plots` for code that is not in the repository, so either the analysis is local-only or it was abandoned. The thesis should say which.
5. `Copy_Trial_Files.sh`: make `--dry-run` side-effect-free, and guard against `DEST_DIR` lying inside `SRC_DIR`.
6. `gdb_crash.log` belongs in notes or an issue, not in the source tree.

*Derived from reading files and git metadata only.*
