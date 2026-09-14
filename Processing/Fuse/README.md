# Point Cloud Fusion (RealSense multi-angle rig)

Combines several PLY scans of the same scene, taken from different tilt/rotation
angles on a rig, into a single fused point cloud. The idea: register every scan
into a common frame, then merge — surfaces hidden from one angle get filled in
by another.

Includes a live viewer so you can watch scans get aligned and merged step by
step, and toggle individual scans on/off to inspect how they overlap.

## Pipeline

The run is organised as explicit stages, not one scan-at-a-time loop:

```
load ─> tilt calibration ─> rotation direction ─> coarse-align (all) ─> review ─> refine (all) ─> review ─> fuse
                                                       ^        │              │
                                                       └────────┴──────────────┘
                                            re-run, exclude, nudge or flip, then look again
```

1. **Load** — every PLY is read, optionally cropped to a region of interest,
   voxel downsampled and outlier filtered. Raw clouds are dropped after this
   (a re-crop reloads from disk) so a large session doesn't sit in RAM.
2. **Tilt calibration** — measures where the tilt head's axis actually is
   relative to the camera (see below). Skipped when no tilt axis is
   configured.
3. **Rotation direction** — works out which way the rig was actually turned
   at each rotation step (see below). Skipped when no rotation axis is
   configured or there is only one rotation folder.
4. **Coarse alignment (batch)** — every scan gets a coarse pose, each aligned
   against the map accumulated from the scans before it. Two hypotheses
   compete: the rig-geometry seed (from the folder's tilt/rotation angles) and
   an FPFH+RANSAC feature match. Whichever actually sits better on the map
   wins — a rig seed is scored, not trusted, so a wrong axis or sign shows up
   as a low overlap score instead of silently poisoning the result.
5. **Review** — per-scan alignment error (rmse and overlap fraction) for the
   whole batch at once. Select problem scans, exclude them, nudge them by
   hand, or re-run just those, then move on.
6. **Fine registration (batch)** — point-to-plane ICP refines every scan
   against the accumulated map, with progress shown as it goes.
7. **Review again** — same table, now with ICP fitness. Loop back to re-run
   coarse or fine for any scan that still looks wrong.
8. **Fuse** — overlapping scans are *reconciled* into a single surface
   rather than concatenated (see below), then outlier filtered and given
   normals.

Because each new scan registers against the *accumulated* map rather than
only its predecessor, drift stays lower as more views are added.

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

## Tilt: measured from the IMU, not from the commanded angle

**Rig geometry.** The tilt stage is at the base; the rotating mount sits on
it; the camera sits on that. So the camera-to-world rotation is
`R_tilt(a) · R_rot(θ)` — the tilt is applied *outside* the rotation, and its
axis is fixed in the **world**. In the camera's own frame the tilt axis
therefore turns with each rotation step, which is exactly what the IMU shows:

| group | tilt axis in camera frame |
|---|---|
| r0 | (0.707, 0.707, 0) |
| r45 | (1.000, 0.000, 0) |
| r90 | (0.707, −0.705, 0) |
| r135 | (0.000, −1.000, 0) |

45° apart, one per rotation step. **A single `--tilt-axis` cannot describe
this** — it is correct for at most one group. (This corresponds to
`compose_tilt_first = false`; the old default of `true` models the opposite
arrangement, a tilt head mounted on a turntable.)

The mount also doesn't reach the angle it is told: commanded 15° came out as
13.6°–17.4° depending on the group, and individual steps ranged 0.6×–1.24×.

**So the tilt is measured, not assumed.** The capture writes an
`imu_a<angle>.csv` beside every scan holding that frame's accelerometer
reading. While the rig is still that is purely gravity, and tilting the
camera swings gravity across the frame *by exactly the tilt angle, about
exactly the tilt axis*. The tilt seed is the minimal rotation carrying a
scan's gravity onto its rotation group's reference gravity — no assumption
about which camera axis the hinge lies on, and none that the servo arrived
where it was sent. The minimal rotation is the physically correct choice
here, not merely the convenient one: a horizontal hinge axis is perpendicular
to gravity, which is precisely that rotation.

Gravity fixes two of the three rotation degrees of freedom. The third —
rotation *about* gravity — is the turntable, which leaves gravity unchanged
and is resolved separately.

### The IMU is not necessarily in the point cloud's frame

The accelerometer reports in the camera's **depth** frame, but a PLY need not
be written in that frame. The RealSense **Viewer** exports an OpenGL-style
frame with y and z negated. Applying a correct rotation in the wrong basis
produces an error that is invisible at small tilts and ruinous at large ones
— which looks exactly like a miscalibrated rig, and was the real cause of
high-tilt scans failing to register here.

You can see it in the data without any registration at all: the surface sits
at *negative* z in the PLY, while a stationary accelerometer reads specific
force pointing up at −z, meaning the camera's +z points down, toward the
surface. Both cannot be true in one frame.

So the relationship is **detected**, not assumed — the four rotations a depth
export realistically uses are each tried, and whichever makes the tilted scans
actually line up is kept:

```
[tilt] testing how the IMU frame relates to the point clouds:
         identity (depth frame)    score 0.0614
         Rx180 - y,z negated       score 0.0143
         Ry180 - x,z negated       score 0.0770
         Rz180 - x,y negated       score 0.0627
[tilt] using Rx180 - y,z negated
```

Measured directly on one real session, residual against the reference scan:

| IMU → cloud frame | a−15 | a−10 | a+10 | a+15 |
|---|---|---|---|---|
| identity | 61.3 | 42.0 | 47.7 | 75.6 mm |
| **Rx180 (y,z negated)** | **14.3** | **7.4** | **12.1** | **18.0 mm** |
| Ry180 | 77.0 | 56.4 | 63.3 | 86.9 mm |
| Rz180 | 62.7 | 45.6 | 48.0 | 71.4 mm |

Only rotations are considered; a mirrored frame would flip handedness and
break far more than the tilt seed.

### Lever arm

If the camera sits off the tilt axis, tilting swings the optical centre
through an arc rather than spinning it in place, and a seed that rotates
about the camera origin is wrong by `|(I-R)p|`, where `p` is the offset to
the axis — an error growing with `sin(tilt)`.

The IMU gives the rotation but says nothing about `p`, so the pivot is found
by **search**: coarse-to-fine over the plane perpendicular to the hinge,
minimising the alignment score of the tilted calibration scans. Only two of
its three components are searched — sliding the pivot *along* the hinge
changes nothing, so that direction is a flat valley.

**Keep `--tilt-pivot-range` (default 0.12 m) near the physical truth.** When
the real lever arm is short the score surface is nearly flat in the pivot, so
a wide search does not merely waste time — it wanders off and returns a large,
meaningless offset. On a rig whose tilt axis sits close to the camera face the
search correctly reports a lever arm of about a centimetre; given a half-metre
range to play with, the same data produced a spurious 0.7 m. Set
`--tilt-pivot x,y,z` (or `tilt_pivot` in the rig config) to skip the search
entirely if you know the value.

The frame and the pivot are resolved **together**, not one after the other.
Each frame candidate gets its own pivot search before the candidates are
compared, because with a long lever arm the pivot error dominates the residual
and comparing frames at a zero pivot compares nothing but noise.

Where no IMU reading is available the older commanded-angle path applies
instead, estimating `p` by solving `(I-R)p = t` across the lowest-tilt scans.
**That fit is only adopted if it actually explains the motion** — one that
leaves most of the error unaccounted for is not a measurement, and adopting it
anyway replaces a mildly wrong seed with a confidently wrong one.

### Reading the calibration report

```
[tilt] resolving the IMU-to-cloud frame and the tilt pivot together
         identity (depth frame)  score 0.0650   pivot (-0.084, 0.085, -0.002)
         Rx180 - y,z negated     score 0.0038   pivot (-0.008, -0.009, 0.000)
         Ry180 - x,z negated     score 0.0663   pivot (0.118, 0.121, -0.076)
         Rz180 - x,y negated     score 0.0407   pivot (0.072, -0.076, 0.167)
[tilt] using Rx180 - y,z negated with pivot (-0.008, -0.008, 0.004) m
```

A large gap between the winner and the rest means the frame is settled. The
reported **lever arm** should match your rig: if you know the camera sits
close to the tilt axis and the report claims half a metre, the search range is
too wide, not the rig surprising you.

Two conventions are sometimes reported as **indistinguishable**, and that is
honest rather than a failure: conjugating a rotation by a flip about its own
axis leaves it unchanged, so when the hinge lines up with a camera axis those
candidates describe the rig equally well and either may be used.

## Flat scenes and why alignment scores can lie

A sand surface viewed from above is a height field, not a general 3D scene,
and that changes what "well aligned" means. The nearest point to a sheet of
points is close **no matter how you slide or spin that sheet within its own
plane**, so nearest-neighbour scoring is nearly blind to in-plane error: a
badly rotated scan still reports a small rmse and a high overlap fraction.
This is worth knowing independently of any one feature — it means a good-looking
rmse on a flat scene is not on its own evidence that a scan is placed correctly.

So the pipeline fits the scene's dominant plane once, and if the
out-of-plane spread is under `planarity_threshold` of the in-plane spread it
switches to a height-field metric: both clouds are binned into cells on the
plane and their mean heights compared cell by cell. Getting the rotation
wrong now means the relief no longer lines up, which this sees immediately.
The console says which metric is in use:

```
[scene] planarity 0.065  - flat scene: scoring alignments as a height field, since
                           nearest-neighbour distance cannot see in-plane error here
```

## Rotation direction

The capture rig names every rotation folder with a positive angle
(`_r0`, `_r45`, `_r90`, ...) regardless of which way you physically turned
the mount, so the folder names alone can't tell a clockwise sweep from an
anticlockwise one. This used to need a manual flag you had to remember after
the fact. It is now inferred automatically.

**Why not from the IMU?** Because nothing captured can answer it:

- Rotation is about the **gravity axis**, and the accelerometer only ever
  measures gravity — which is unchanged by a rotation about itself. The
  stored `accel` is therefore invariant to exactly the thing we want to know.
  (It *does* constrain tilt, which is about a horizontal axis.)
- The gyro could answer it if integrated *through* the turn, but the
  recorder is stopped while you turn the mount by hand, so the one stored
  gyro sample is taken stationary and reads ~zero.

**What is decisive is the geometry.** Guessing the wrong direction for a
45° step puts the scan 90° away from the truth. Every combination of
per-step directions is scored as a whole and the best-fitting arrangement
kept — not each step settled greedily before looking at the next, because on
a flat scene an individual step is often close to a coin toss while the whole
sequence is not: a wrong early step makes every later one fit worse, and only
a joint search can see that.

Candidates are scored as **raw seeds**, with no ICP settling, which is
deliberate. On a near-planar scene ICP has three barely-constrained degrees
of freedom, so letting a *wrong* direction settle lets it slide into a
plausible-looking fit and beat the right one — this was observed on real
data, where settling picked the wrong direction for two of three steps.

```
[rotation] 20260101_000000_r45: -45 deg (cumulative -45)
           chosen rmse=0.98mm/100%  rejected rmse=5.11mm/67%  margin=87%
```

The **margin** is per step, and asks the question that actually matters for
that step: how much worse does the whole arrangement get if just this one is
flipped? (Comparing against the runner-up arrangement instead would make even
a well-determined early step look like a coin toss, since two arrangements
differing only in the last step share every earlier placement and score almost
identically.)

On scenes with real 3D structure the margin lands in the 85–99% range. On flat
sand it can be as low as a few percent even when the answer is right — the
evidence genuinely is thin there. Anything below `direction_ambiguous_margin`
(default 15%) is flagged `[AMBIGUOUS]`, which means *verify this visually*,
not *this is wrong*.

Direction is resolved **per rotation step**, not per session, and the angles
are cumulative: `_r90` only means 90° if every step before it went the same
way. Since you turn the mount by hand at each step, you can reverse partway
through a session, and that is handled — a session recorded as `r0, r45, r90`
where you turned forward and then back resolves to `0°, +45°, 0°`.

If the automatic result is wrong or ambiguous, select any scan from that
rotation group in the viewer and press **`t`** to flip it. Because the angles
are cumulative, flipping one step shifts that group *and every later one*,
and all of them are re-aligned.

Pass `--no-rotation-direction` to skip inference and trust the recorded
angles as-is. `--rotation-sign` and `--reverse-rotation` still work and set
the starting assumption, but they are no longer the way to handle this —
inference overrides them per group.

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

## Viewer

Opens by default; pass `--no-gui` to run every stage straight through with no
review (headless batch, e.g. on a remote machine or once you trust the
settings). Pass `--no-roi` to keep the viewer but skip the per-scan crop-box
prompt.

Every scan is drawn in its own color, with a **legend panel** down the left
listing each scan by name alongside its status and alignment error. The
legend is the selection mechanism — there are no number keys, and it scales
to any number of scans (the list scrolls to keep the selection in view).

**Selecting a scan** — any of:

| Input | Action |
|---|---|
| click a legend row | select that scan |
| shift + click a point in the 3D view | select whichever scan owns that point |
| `Up` / `Down` | move through the list |

The selected scan is drawn with larger points and highlighted in the list.

**Acting on the selected scan:**

| Key | Action |
|---|---|
| `v` | show/hide it |
| `i` | isolate it (hide all others; press again to restore) |
| `a` | show all scans |
| `d` | exclude it from the fusion / put it back |
| `n` | toggle nudge mode |

**Re-running alignment:**

| Key | Action |
|---|---|
| `Enter` | re-run the current stage for the selected scan |
| `Shift`+`Enter` | re-run the current stage for every scan |
| `z` | re-run *coarse* alignment for the selected scan |
| `Z` | re-run *coarse* alignment for every scan |

`Enter` does whatever the stage you are reviewing is about, so there is one
key to learn rather than a coarse/fine pair. `z` reaches back to coarse from
the refine review, for a scan ICP cannot rescue and that needs a fresh coarse
pose instead of another refinement.

**Nudge mode** (`n`) lets you correct a pose by hand: arrow keys translate in
x/y, `PageUp`/`PageDown` in z, `,` and `.` rotate about z, and `+`/`-` change
the step size (shown in the status line). Rotation is about the scan's own
centroid, so it turns in place rather than swinging around the world origin.
A nudged pose becomes the seed for any subsequent ICP run on that scan.

**Stage control:**

| Key | Action |
|---|---|
| `space` | accept these results, continue to the next stage |
| `b` | back to the previous review stage |
| `m` | show/hide the on-screen key reference |
| `q` / `Esc` | stop reviewing; the run finishes with the current results |

The current key bindings are always drawn in the top-right of the window, so
you never have to remember them; `m` hides the panel if it gets in the way.

During the ROI crop (unless `--no-roi`): drag the box handles, `space` to
confirm the crop, `d` to use the full cloud.

> **Why these keys?** PCLVisualizer reserves `p w s j c f e q g u o r l x h`,
> the digits and `+`/`-` for its own camera and rendering controls, and a
> registered keyboard callback runs *in addition* to those rather than
> replacing them. Every binding above avoids that set. Notably `e` exits the
> interactor outright, which is why exclude is `d` and not `e`.

Legend columns are `#` (index), `st` (status: `R`eference, `c`oarse,
re`F`ined, `X` excluded, `!` failed), scan name, rmse, and overlap fraction.
A trailing `!` marks a scan flagged for high ICP fitness error. The same
per-scan table is printed to the console at every review stage, so a headless
run gives you the same numbers.

## Fusion: reconciling overlaps, not concatenating them

Concatenating registered scans does not fuse them. Residual registration
error (a few mm) is larger than the output voxel, so where scans overlap
their surfaces land in *different* voxels and survive as two or three offset
sheets. Voxel centroids cannot merge sheets that never share a cell, and
statistical outlier removal will not delete one either — a duplicate sheet is
locally dense and looks like perfectly good data. That is why a merged cloud
looks noisiest exactly where it has the most information.

Reconciliation has three parts:

**1. Weight every point by how much it deserves to be believed.**

- *Registration quality* — each scan's overlap fraction and rmse become a
  confidence in [0,1] that scales all of its points. A scan the pipeline is
  unsure about still contributes without being able to drag the result around.
- *Incidence angle* — depth error grows sharply at grazing incidence, so a
  surface seen face-on beats the same surface seen edge-on.
- *Range* — stereo depth error grows with the square of distance, so weight
  falls off the same way.

`--no-fusion-weights` turns all three off.

**2. Require corroboration.** `--fusion-min-scans n` drops surface that fewer
than *n* distinct scans saw. The default of 1 keeps everything; 2 is a strong
noise filter in the overlap regions, at the cost of trimming the outer edges
only one scan covered.

**3. Combine robustly.** Within each cell the contributing points are
combined with a **trimmed weighted mean** — `--fusion-trim` sets the fraction
discarded from each end (0 = plain mean, 0.5 = weighted median, default 0.25).
This is the part that makes fusion robust rather than merely averaged. With
one misregistered scan among four, a plain mean lands *between* the two
surfaces, belonging to neither:

| trim | error with one 8 mm-misregistered scan in four |
|---|---|
| 0 (mean) | 2.00 mm — i.e. 8 mm ÷ 4, dragged by the bad sheet |
| 0.25 (default) | 0.35 mm — bad sheet rejected |
| 0.5 (median) | 0.34 mm |

Trimming is by *weight*, not by count, so one high-confidence scan is not
outvoted by several poor ones.

### Two modes

**Height field** (`--fusion-mode heightfield`) — for a surface scanned from
above, which is what this rig does. Cells are laid out on the scene's
dominant plane and each yields a single reconciled height. Disagreement
between scans on such a scene is almost entirely *in height*, so collapsing
it per cell removes the multi-sheet problem outright. `--fusion-cell` sets the
lateral resolution.

**Volumetric** (`--fusion-mode volumetric`) — for scenes with real 3D
structure. Points are binned into voxels (`--fusion-voxel`) and reconciled
*along the local surface normal*, since that is the direction registration
error actually displaces a surface in. Lateral placement is left alone.

`--fusion-mode auto` (the default) picks height field on planar scenes and
volumetric otherwise, using the same planarity measure described above.

### Surface fit

A moving-least-squares pass (`--mls-radius`, default 10 mm; `--no-mls` to
skip) fits a local polynomial and projects points onto it. This is what
collapses any *residual* offset between sheets, because it averages along the
surface normal over a radius larger than the misregistration while keeping
lateral detail. **Set the radius to a few times your residual rmse** — too
small and it does nothing, too large and it flattens the relief you are
trying to measure.

### Reading the report

```
Fusion (height field):
  input points            163011
  cells                   7759  (16.64 scans per cell on average)
  after blending          7759
  after outlier removal   6762
  output points           6762
  surface thickness       0.31 mm  ->  0.11 mm
```

**Surface thickness** is the headline number: how far apart the overlapping
scans were within a cell, before and after. The first figure is the
multi-sheet noise you were seeing; the second is what is left of it. Measured
against a known synthetic surface, reconciliation halved the error of the old
concatenate-and-voxel approach (0.96 mm rms → 0.45 mm, 5–95% spread 3.20 mm
→ 1.47 mm).

## Output density

Two settings control it, and they do different jobs:

| Flag | Default | What it does |
|---|---|---|
| `--fusion-cell <m>` | 0.003 | Lateral cell size in height-field mode. **This sets the point count** — one output point per occupied cell, so the count scales roughly as 1/cell². |
| `--fusion-voxel <m>` | 0.002 | The same role in volumetric mode. |
| `--scan-voxel <m>` | 0.003 | Spacing each scan is thinned to *when loaded*. This is the **ceiling on real detail** — a finer fusion cell cannot recover what was discarded here. |

Measured on a 6-scan test set:

| settings | output points |
|---|---|
| default (scan 3 mm, cell 3 mm) | 10,548 |
| `--fusion-cell 0.0015` | 29,360 |
| `--scan-voxel 0.0015 --fusion-cell 0.0015` | 28,623 |
| `--fusion-cell 0.006` | 2,614 |

Note the third row: adding `--scan-voxel` barely changed the *count*, because
the count is set by the cell size alone. What it changes is whether those
points carry genuine detail or just subdivide the same thinned data. So:

- **More points, same information** — lower `--fusion-cell` only.
- **More points and more real detail** — lower both. Costs memory and time
  roughly as 1/m², so halving the voxel is about 4× of each.

The program warns if you ask for a fusion cell finer than `--scan-voxel`,
since that combination cannot add detail.

Two things also remove points after blending: `--fusion-min-scans` (drops
surface too few scans agree on) and the outlier-removal pass
(`--no-fusion-sor` to skip it).

The outlier pass judges each point by its distance from a plane fitted to its
30 nearest neighbours, dropping those more than 3 standard deviations above the
mean. It deliberately does not use statistical outlier removal (mean distance
to neighbours): that measures density, and a fused cloud's density is uneven by
design. With a fusion cell finer than a single scan's point spacing (about
1 mm here), areas seen by many scans come out denser than areas seen by a few,
and SOR deleted the sparser ones wholesale, leaving holes in genuine overlap. And if you go much denser, consider lowering
`--mls-radius` too — but keep it above your residual registration rmse, or the
surface fit will smooth across genuine relief rather than across noise.

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
- **`eval_correspondence_distance`** — the radius used to *score* an alignment
  (rmse + overlap fraction), separate from ICP's own correspondence distance
  so scores stay comparable between the coarse and fine stages. Set it near
  your expected residual error; too large and every hypothesis scores as a
  perfect 100% overlap, too small and a correct alignment looks like a miss.
- **`coarse_min_inlier_fraction`** — how much of a scan must land on the map
  for a coarse pose to be believed. A rig seed below this is rejected in
  favour of feature matching; if both fall short the scan is marked
  coarse-failed rather than handed to ICP, which cannot recover a seed that
  has essentially no overlap.
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
