# Point Cloud Fusion (RealSense multi-angle rig)

Combines several PLY scans of the same scene, taken from different tilt/rotation
angles on a rig, into a single fused point cloud. The idea: register every scan
into a common frame, then merge — surfaces hidden from one angle get filled in
by another.

Includes a live viewer so you can watch scans get aligned and merged step by
step, and toggle individual scans on/off to inspect how they overlap.

## Pipeline

For each scan (after the first, which seeds the reference frame):

1. **Preprocess** — voxel downsample + statistical outlier removal (cleans up
   RealSense depth noise before it pollutes registration).
2. **Coarse alignment** — one of two paths:
   - If you supply a known/approximate rig transform for that frame (translation
     + quaternion, or an auto-generated tilt-angle rotation — see `--tilt-axis`
     below), it's used directly as the ICP seed.
   - Otherwise, FPFH feature matching + RANSAC (`SampleConsensusPrerejective`)
     estimates a coarse alignment against the running merged cloud.
3. **Fine alignment** — point-to-plane ICP refines the coarse transform.
4. **Merge** — the aligned scan is added to the running merged cloud (each new
   scan registers against the *accumulated* map, not just the previous scan,
   which reduces drift as more views are added).
5. **Final cleanup** — voxel resample to a uniform density, another outlier pass,
   and normal estimation on the fused result.

## Build

Requires PCL (1.10+) and its dependencies (Eigen, FLANN, VTK — needed for the
`visualization` component the live viewer uses).

```bash
sudo apt install libpcl-dev   # or build PCL from source if you need a newer version
mkdir build && cd build
cmake ..
make -j$(nproc)
```

## Input layout — session folders (recommended for your rig)

```
scan_sessions/
  20260909_143000/                              <- session (date_time)
    20260909_143000_r0/                         <- rotation subfolder
      20260909_143000_r0_a-5/20260909_143000_r0_a-5.ply
      20260909_143000_r0_a0/20260909_143000_r0_a0.ply
      20260909_143000_r0_a5/20260909_143000_r0_a5.ply
    20260909_143000_r45/                        <- next rotation
      20260909_143000_r45_a-5/20260909_143000_r45_a-5.ply
      20260909_143000_r45_a0/20260909_143000_r45_a0.ply
      ...
    20260909_143000_r90/
      ...
```

Three levels: a **session folder** (named by date/time), containing one
**rotation subfolder** per rig rotation setting (`..._r<deg>`), each
containing one **tilt subfolder** per tilt angle (`..._r<deg>_a<deg>`),
holding a single PLY (the "best frame" for that rotation+tilt). Pass each
session folder on the command line — if you only ever scan in one session,
that's just one path:

```bash
./fuse_scans -o fused.ply --sessions scan_sessions/20260909_143000
```

Multiple date_time session folders (e.g. separate scanning runs of the same
object) can be passed together too — everything under every session folder
gets discovered and fused into one cloud.

The discoverer locates the PLY inside each tilt subfolder (it prefers a file
named exactly `<subfolder>.ply`, and falls back to whatever single `.ply` it
finds otherwise — it'll warn you if a subfolder has zero or more than one
candidate). Rotation angle is parsed from each rotation subfolder's name via
`_r<number>`, tilt from each tilt subfolder's name via `_a<number>` (both
handle negative angles like `_a-5`); if a name doesn't match, that scan is
still included, just without that metadata.

Scans are then reordered so a 0° tilt scan seeds the reference frame and
everything else comes in ascending `|tilt|` — this registers far more
reliably than fusing in arbitrary directory order, since each new scan is
closer in angle (and therefore overlap) to what's already merged.

Sanity-check the directory layout before running the full fusion:

```bash
./fuse_scans --list-only --sessions scan_sessions/20260909_143000
```

**If you know which axis your rig tilts/rotates about** (recommended — much
more robust than blind feature matching, especially on scenes without
strong geometric structure), pass them and the parsed angles become direct
ICP seeds instead of relying on FPFH+RANSAC:

```bash
./fuse_scans -o fused.ply --sessions scan_sessions/20260909_143000 --tilt-axis x --rotation-axis z
```

Flip `--tilt-sign -1` / `--rotation-sign -1` if angles come out rotated the
wrong way. See the next section for the config-file version of this, and
for what to do when a rotation subfolder's name *doesn't* carry an explicit
angle and you just know the approximate step size instead.

## Known rig geometry (e.g. ~5° tilt steps, ~45° rotation steps)

If you know your rig's approximate step sizes, seed both the tilt *and*
rotation components directly instead of relying on feature matching for
either. The way to do this so it's easy to change later (without touching
code or CLI flags each time) is a small config file:

```bash
cp rig_config.example.txt my_rig.txt   # edit tilt/rotation axis, sign, step
./fuse_scans -o fused.ply --rig-config my_rig.txt --sessions scan_sessions/20260909_143000
```

```
# my_rig.txt
tilt_axis = x
tilt_sign = 1
rotation_axis = z
rotation_step_deg = 45
rotation_sign = 1
compose_tilt_first = true
```

- **Tilt** and **rotation** both still come from the actual subfolder names
  when present (`..._a<deg>`, `..._r<deg>`) — the config just says which
  axis each rotation is about.
- If a *rotation* subfolder's name doesn't carry a parseable `_r<deg>`
  token, it falls back to `rotation_index * rotation_step_deg`, numbering
  rotation folders in the order they're found *within each session*
  (resetting per session) — this is the "I know it's roughly 45 degrees per
  step" case, no folder renaming required. Tilt has no equivalent
  fallback — it's assumed each tilt subfolder is always explicitly named.
- Edit the file and rerun any time your rig calibration changes — nothing
  to recompile. Individual values can still be overridden per-run with
  `--tilt-axis`, `--tilt-sign`, `--rotation-axis`, `--rotation-step-deg`,
  `--rotation-sign` on top of whatever the config file sets, useful for
  quickly testing a different step size without editing the file.
- `compose_tilt_first` controls how the two rotations combine (tilt applied
  in the frame that then gets rotated, vs. the reverse) — the default
  matches a tilt head mounted on a turntable. If seeded fitness scores are
  consistently *worse* than running without seeding at all, flip this
  before assuming your angles are wrong.
- These are still just ICP seeds, not exact poses — rough rig geometry
  (a few degrees off) is normally fine, since ICP only needs to land within
  `icp_max_correspondence_distance` of the true alignment.

## Other input modes

Flat list, no folder structure, no known poses:

```bash
./fuse_scans -o fused.ply scan_tilt0.ply scan_tilt15.ply scan_tilt-15.ply
```

Full manual control over per-scan poses via a manifest (translation + quaternion):

```
# manifest.txt
# path                    tx    ty    tz    qx    qy    qz    qw
scan_tilt0.ply
scan_tilt15.ply           0.0   0.0   0.0   0.0   0.13  0.0   0.99
scan_tilt-15.ply          0.0   0.0   0.0   0.0  -0.13  0.0   0.99
```

```bash
./fuse_scans -o fused.ply --manifest manifest.txt
```

`--sessions` and `--manifest` are mutually exclusive; the flat-list form is
always available as a fallback.

## Live viewer

Opens by default; pass `--no-gui` to run straight through without it (e.g. on
a headless machine or for a final batch run once you trust the pipeline).

Controls (also printed to the console on startup):

| Key | Action |
|---|---|
| `space` | advance to the next step |
| `p` | toggle auto-play (advances automatically) |
| `1`–`9`, `0` | toggle visibility of scan 1–10 (repeats for more than 10) |
| `a` | show all scans |
| `r` | reset camera |
| `q` / `Esc` | stop viewing, finish the run headless |

What you'll see, in order:
1. The reference scan appears in its own color.
2. Each subsequent scan previews in **gray** at its coarse-aligned pose (so
   you can see how far ICP still has to pull it).
3. Press space and it snaps to its refined pose, committed in a distinct
   color — if its fitness exceeded the quality threshold, that color is
   tinted red so it stands out.
4. Once every scan is merged, the final cleaned-up cloud is shown colored by
   height. Overlapping regions between differently-colored scans are exactly
   where you can visually check the fusion is registering surfaces
   consistently rather than drifting.

Since colors are assigned per scan, toggling individual scans off with the
number keys is the fastest way to check whether a specific tilt or session is
pulling its weight or contributing noise.

The program also prints per-scan fitness scores to the console as it runs,
plus a full summary at the end — cross-reference flagged scans there with
what you saw in the viewer.

## Tuning notes (`PipelineParams` in `fusion_pipeline.h`)

- **`voxel_leaf_size`** — set relative to your D456's actual noise floor at your
  scan range. A few millimetres at close range is typical; too fine and you're
  registering noise, too coarse and you lose the detail you're trying to fuse in.
- **`icp_max_correspondence_distance`** — should be a few times your coarse
  alignment's expected error. Too tight and ICP can't find correspondences to
  even start converging; too loose and it can lock onto the wrong surface.
- **`icp_fitness_reject_threshold`** — flags (not silently drops) scans whose
  fine alignment fitness is suspiciously high; worth inspecting those manually
  in the viewer.
- **Textureless / planar scenes** will make the no-pose FPFH path unreliable —
  lean on `--tilt-axis` or a manifest whenever you can, since geometric
  feature matching struggles without distinctive local structure.
- If your PLY files don't carry RGB, switch `PointT` in `fusion_pipeline.h` to
  `pcl::PointXYZ` and drop the color-dependent bits (there aren't any beyond the
  typedef — geometry-only registration works the same way).

## Output

A single fused, uniformly-resampled PLY with estimated normals
(`pcl::PointXYZRGBNormal`), ready for meshing (e.g. Poisson reconstruction) or
further analysis.
