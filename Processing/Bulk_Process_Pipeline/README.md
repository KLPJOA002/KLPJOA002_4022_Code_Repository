# Bulk crop + fuse pipeline

Crops every scan of every session under a source directory, then fuses each
session into one point cloud.

```
<source>/.../<Session>/<Session>_r*/<Session>_r*_a*/*.ply
   └─> auto AprilTag crop (../Crop)  ─> fuse_scans --no-gui (../Fuse)  ─> <output>/fused/.../<Session>.ply
```

Neither tool is reimplemented or modified. The crop step imports
`../Crop/crop_pointclouds.py` and calls its automatic crop without the preview
window. The fuse step runs `../Fuse/build/fuse_scans`.

## Usage

Run it with the Crop tool's venv, which has open3d and numpy:

```bash
cd Processing/Bulk_Process_Pipeline
../Crop/cropper/bin/python bulk_process.py <source_dir> <output_dir> [options] [-- fuse_scans args]
```

```bash
# every session under Small_Pyramid, fusion settings from a config file
../Crop/cropper/bin/python bulk_process.py /data/3D_Error/Small_Pyramid out/ --config my_pipeline.toml

# fusion flags straight on the command line (everything after -- goes to fuse_scans)
../Crop/cropper/bin/python bulk_process.py /data/3D_Error out/ -- --tag-align --fusion-cell 0.002

# check what would run first
../Crop/cropper/bin/python bulk_process.py /data/3D_Error out/ --config my_pipeline.toml --dry-run
```

The source can be a single session folder or any folder above sessions. It is
searched recursively, and the source is only read, never written to.

## Fusion parameters

Copy `pipeline_config.example.toml` and edit `[fuse.params]`. Each key is a
`fuse_scans` flag without the `--`:

```toml
[fuse.params]
rig-config = "../Fuse/src/rig_config.example.txt"   # relative to the config file
tag-align = true                                     # true -> bare flag
fusion-cell = 0.002                                  # -> --fusion-cell 0.002
fusion-min-scans = 2
```

Arguments after `--` on the command line are added after the config file's
settings, so they override them. `--no-gui` is always added unless
`gui = true` or `--fuse-gui` is set.

## Output

| path | |
|---|---|
| `fused/<rel>/<Session>.ply` | the fused scan for each session |
| `cropped/<rel>/<Session>/` | cropped scans, laid out like the Crop tool's output, with IMU files, tag sidecars, detections and `scan_info.txt` |
| `logs/<rel>/<Session>_fuse.log` | the full `fuse_scans` output (alignment tables and so on) |
| `summary.csv` | one row per session: scan counts, crop outcomes, which scans were left out and why, fuse status, time |
| `run_config.json` | the crop settings and the exact fuse arguments used |

`<rel>` is the session's parent folder relative to the source (for example
`863mm`), so sessions from different objects don't overwrite each other.

## Crop behaviour

- If a scan can't be auto-cropped (missing tags, detections or intrinsics),
  `on_fail` / `--on-crop-fail` decides what happens. `skip` (the default)
  leaves the scan out of the fusion. `keep` fuses it uncropped. `manual`
  opens the Crop tool's manual crop window.
- `--review-crops` shows the Crop tool's preview window for every scan. The
  keys are the same as in the Crop tool: Enter, M, N, and Esc to stop the
  run.
- Crops are cached. A session that was already cropped with the same crop
  settings is reused, so you can re-run with different fusion parameters
  without cropping again. If you change a crop setting, that session is
  cropped again, and `--recrop` forces a re-crop. Use `--cropped-dir` to
  share one crop cache between several output folders, for example one output
  folder per fusion setting.
- A session needs at least 2 usable scans; `fuse_scans` won't fuse fewer.

## Other options

| flag | |
|---|---|
| `--sessions '20261005_16*,2026100[67]_*'` | only process sessions whose name matches these globs |
| `--crop-margin`, `--frame-window`, `--tag-order` | same as the Crop tool's `--margin`, `--frame-window` and `--tag-order` |
| `--crop-only` | crop only, don't fuse |
| `--skip-fused` | don't fuse a session again if its fused PLY already exists |
| `--fuse-bin <path>` | use a different `fuse_scans` executable |
| `-v` | print `fuse_scans` output to the console as well as the log |
