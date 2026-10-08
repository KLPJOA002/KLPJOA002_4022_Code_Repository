# Rotation x tilt combination sweep

Which subsets of a full rotation/tilt sweep are worth fusing? For every wave,
height and combination of scans, this fuses the subset with `../Fuse` and scores
the result against the reference with `../Evaluate`, then plots quality against
height, scan angle, scan count and runtime.

```
full-sweep session (44 scans) ─> AprilTag crop (../Crop, once, cached)
   ─> per combination: staging session of symlinks ─> fuse_scans --no-gui (../Fuse)
   ─> seed pose + part trim ─> evaluate_scan (../Evaluate) ─> row in results.csv
```

None of the existing tools are modified. They are wrapped as follows:

- `fuse_scans` runs as a subprocess.
- `crop_pointclouds`, `scan_eval` and Parameter_Sweep's `seed_transform` are imported.

## Setup

```bash
cd Processing/Combination_Sweep
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
```

The `fuse_scans` binary is set in `sweep_config.yaml` (`fuse.binary`, default
`../Fuse/build/fuse_scans`). Each run records its path, modification time and
SHA-1 in `run_config.json`.

## Commands

```bash
.venv/bin/python run_sweep.py plan                  # sessions, job count, disk, time estimate
.venv/bin/python run_sweep.py plan --from-run ../Experiments/pilot   # ...using measured timings
.venv/bin/python run_sweep.py run --dry-run         # list every planned job, run nothing
.venv/bin/python run_sweep.py run --pilot --out ../Experiments/pilot # one wave x height, end to end
.venv/bin/python run_sweep.py run                   # the full grid -> ../Experiments/<timestamp>/
.venv/bin/python run_sweep.py run --out ../Experiments/<timestamp>   # resume a stopped run
.venv/bin/python run_sweep.py analyze ../Experiments/<timestamp>     # figures, tables, summary.md
.venv/bin/python run_sweep.py voxel                 # scan-voxel / fusion-cell sensitivity (separate run dir)
.venv/bin/python run_sweep.py analyze ../Experiments/voxel_<timestamp>
.venv/bin/python run_sweep.py crop                  # optional: crop all 12 sessions up front
```

Common flags:

| flag | what it does |
|---|---|
| `--waves Big_Wave` | only these waves |
| `--heights 641mm,863mm` | only these heights |
| `--combos rot-all_tilt-all,single_r0_a0` | only these combination ids |
| `--workers N` | worker processes (overrides the config) |
| `--retry-failed` | re-run jobs whose row says failed |

For an overnight run, start it under `nohup` (or in `tmux`) so it survives closing the terminal:

```bash
nohup .venv/bin/python run_sweep.py run --out ../Experiments/full > /dev/null 2>&1 &
tail -f ../Experiments/full/batch.log
```

## Data and session choice

Each `<wave>/<height>/` folder holds two sessions. The **full-sweep** session is
the one with the most scans (44, against 4 in the other session). It is found
by scan count, not by name, and `sessions.csv` in each run records which session
was used and which was ignored.

The scans are `<session>_r<rot>/<session>_r<rot>_a<tilt>/`:

- rotations 0, 45, 90 and 135;
- tilts −25 to +25 in 5° steps, with the sign kept;
- 4 × 11 = 44 scans.

## Combinations

These are all defined in `sweep_config.yaml`:

- `rotation_families` and `tilt_families`: a name → a list of angles, or `all`.
  `pm10` in a name means ±10.
- `grid`: which rotation families are crossed with which tilt families. Each
  cell becomes the id `rot-<name>_tilt-<name>`, with `n_scans` = rotations ×
  tilts.
  - The tilt=`all` row holds the rotation subsets, and the rot=`all` row holds
    the tilt subsets.
  - `rot-all_tilt-all` is the all-44 upper bound.
  - `family` in the table is `rotation`, `tilt`, `joint`, `all` or `single`.
- `single_scans: true` adds `single_r<rot>_a<tilt>` for every scan. These are
  evaluated **unfused**, because `fuse_scans` needs at least 2 scans. The one
  1-scan grid cell (`rot-0_tilt-0`) is therefore covered by `single_r0_a0`.

The default config gives 47 fused combinations and 44 single scans per (wave,
height). Over 12 heights that is 1092 jobs. Check any edit with `plan`.

## What one job does

1. **Crop.** Each full-sweep session is cropped once into the shared
   `crop_cache`, the same way Bulk_Process_Pipeline crops:
   - writes the cropped PLY, IMU file, tag sidecar, detections and `scan_info.txt`;
   - learns the tag layout per height, falling back to the whole wave when no
     frame decodes all four tags, which is Filter_Sweep's fix for 641 mm;
   - is cached by crop settings. A scan that fails to crop is left out of every
     combination, and listed in `scans_missing`.
2. **Stage.** `jobs/.../staging/<session>/<session>_r*/` holds symlinks to just
   this combination's cropped tilt folders. `fuse_scans` discovers it like a
   real session, so tilt from the IMU, rotation and tag alignment all behave as
   in a normal run. Staging is deleted afterwards.
3. **Fuse.** Runs `fuse_scans -o fused.ply --sessions <staging> <fuse.params> --no-gui`.
   - The full output goes to `fuse.log`.
   - Wall time and the process's own peak RSS are recorded.
   - `timeout_s` kills a hung fusion.
4. **Evaluate.** Same chain as Filter_Sweep:
   - Parameter_Sweep's `seed_transform` (table plane to z = 0, part over the reference);
   - drop points more than 30 mm outside the reference footprint;
   - `scan_eval.evaluate_scan` with full alignment;
   - every Evaluate metric goes into the row.
   - Rows covering less than 85% of the reference are marked `suspect`, as in
     Parameter_Sweep.
5. **Row.** `row.json` in the job folder holds the combination columns, the
   timings, the metrics, `status` (ok / suspect / failed), `failure_stage` and
   `failure_reason`. Any error in any step becomes a failed row. The batch
   never stops for one job.

References, as in Filter_Sweep:

| wave | reference |
|---|---|
| Small_Wave | `Egg carton-smaller v1 (1).stl` |
| Big_Wave | `EGG_1PK.stl` |

## Caching and resuming

- **A job is done when its `row.json` exists.** The job folder is
  `jobs/<wave>/<height>/<combo id>__sv<scan-voxel>_fc<fusion-cell>_<hash>`. The
  hash covers every fusion flag, so changing any of them re-fuses. Single scans
  aren't fused, so their folder has no parameter part.
- **Resuming:** re-run with `--out <run dir>` to skip finished jobs.
  `--retry-failed` re-runs the failed ones.
- **The source data is only read.** Everything is written under
  `experiments_root`, plus the shared `crop_cache`.

## Run directory

| path | what it holds |
|---|---|
| `results.csv` | long format, one row per job, rebuilt from the `row.json` files |
| `batch.log` | progress, one line per finished job with fuse/eval time, RSS, RMSE, failures |
| `sessions.csv` | the session used per (wave, height) and the one ignored |
| `run_config.json` | config snapshot, fuse binary path, mtime and SHA-1 |
| `jobs/.../` | `row.json`, `fuse.log`, `job.log` (Evaluate output), `fused.ply`, `seed_transform.txt`, `eval/metrics.json` |
| `analysis/` | `summary.md`, `ranking_top10.csv`, `smallest_within_<X>pct.csv`, `figures/*.png\|pdf` |

Set `keep_fused_ply: false` to delete each fused cloud once it has been
evaluated. A fused cloud is about 7 MB.

## Analysis

`analyze` uses `analysis.primary_metric` (default `acc_rmse_mm`) for rankings,
and plots every metric listed under `analysis.metrics`.

Figures:

- `height_rotation_*`, `height_tilt_*` and `height_overview_*`: metric vs height,
  one line per combination, a panel per wave. The overview adds the single-scan
  median and IQR.
- `angle_single_vs_tilt_*` and `angle_single_vs_rotation_*`: unfused quality per
  angle.
- `angle_fused_tilt_*` and `angle_fused_rotation_*`: fused quality vs which
  tilts or rotations are included.
- `nscans_*`: diminishing returns.
- `runtime_*`: cost, and quality vs cost. These are separate panels, with no
  dual axis.
- `heatmap_*`: the rotation × tilt grid, median over heights.
- `voxel_*`: from a `voxel` run.

Tables:

- `ranking_top10.csv`: the best combinations per height.
- `smallest_within_<X>pct.csv`: the fewest scans within X% of all-44. Set X
  with `analysis.within_percent` or `--within`.
- `summary.md`: key findings and the metric caveats.

## Parallelism and memory

`workers` jobs run at once, and each `fuse_scans` gets `threads_per_worker`
OpenMP threads.

Measured in the pilot (Big_Wave 863 mm, 0.5 mm voxel/cell): peak fusion RSS is
well under the 54 GB available (see `batch.log` and the `fuse_peak_rss_mb`
column), so RAM is not the limit at 0.5 mm; CPU is. `plan --from-run <dir>`
prints the measured peak and the RAM `workers` of them would need. Re-check
with it before running the `voxel` sweep, because RAM grows about 4× per
halving of `scan-voxel`.
