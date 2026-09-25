# Scan Evaluation (geometric error against a ground-truth reference)

Measures how far a 3D scan is from the truth. Each experimental `.ply` (a
single capture, or Fuse's fused output) is aligned rigidly to a known
reference object and scored with a fixed set of metrics. Every run adds one
row to a shared `results.csv`, so hardware configurations and fusing
algorithms can be compared side by side.

The sibling of `../Fuse`. Fuse aligns scans *to each other* and merges them.
This tool aligns each scan *to the reference* and fuses nothing.

```
load ─> units check ─> align (multi-start, scored, exact refine) ─> metrics ─> write
```

## Why Python when Fuse is C++

The reference has to be definable by an exact equation, parsed safely from a
config file, differentiated symbolically, and extensible with a user-written
class. That is sympy, lambdify and a Python plugin, none of which exist in
C++/PCL. The other Processing tools (Crop, Viewer) are already Python +
Open3D, so this one is too. Fuse's registration was **ported, not rewritten**:

- the same stages, in the same order;
- the same preprocessing (voxel, then SOR, then normals toward the sensor);
- FPFH+RANSAC for the coarse pose and point-to-plane ICP to refine it;
- the same "score hypotheses, don't trust them" design, with the same 15%
  ambiguity margin;
- the same CLI shape: flat flags, a `--list-only` style mode, and a config
  file as the baseline that flags override;
- the same console style (`=== Stage ===`, `[tag]`, `[!]`).

Where Fuse's approach had to change, the reasons are under
[Alignment](#alignment-and-how-it-avoids-look-alike-poses).

## Setup

```bash
cd Processing/Evaluate
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python -m pytest tests          # ~40 s
```

Like Fuse's `build/`, `.venv/` is per machine and not in git. (The old
`Crop/cropper` and `Viewer` venvs point at a python3.11 that is no longer
installed, so they need recreating the same way.)

## Usage

### A mesh as truth

```bash
.venv/bin/python evaluate_scans.py --reference part.stl --reference-units mm \
    --label d435_close_single scan.ply
```

Mesh files carry no units. **mm is assumed** for a bare mesh (what CAD and
slicers use), and the assumption is printed. For anything more than a quick
check, write a small definition file instead, so the metadata travels with
the part:

```toml
# surfaces/egg_carton_3cm_mesh.toml
name         = "egg-carton-3cm-mesh"
type         = "mesh"
units        = "mm"
path         = "../../Fused_Scans/Egg-carton-smaller3cm.ply"
height_field = true     # z = h(x,y): Z-error stats, outline-from-above region, +z visible
```

```bash
.venv/bin/python evaluate_scans.py --reference-def surfaces/egg_carton_3cm_mesh.toml \
    --label d455_fuse_heightfield ../Fused_Scans/Out4.ply
```

### An exact analytic surface as truth

```toml
# surfaces/egg_carton.toml
name  = "egg-carton-3cm"
type  = "height_field"
units = "mm"

[params]
A = 15.0     # amplitude
L = 50.0     # wavelength
c = 25.0     # mean height

[surface]
z = "c - A*sin(2*pi*x/L)*cos(2*pi*y/L)"

[domain]
x = [0.0, 100.0]
y = [0.0, 100.0]
# mask = "x**2 + y**2 <= 40**2"     # optional: non-rectangular outline
```

```bash
.venv/bin/python evaluate_scans.py --reference-def surfaces/egg_carton.toml \
    --label d455_fuse_volumetric ../Fused_Scans/        # a whole directory
```

The three analytic forms, with one example of each in `surfaces/`:

| `type` | `[surface]` | `[domain]` | example |
|---|---|---|---|
| `height_field` | `z = "f(x, y)"` | `x`, `y` ranges, optional `mask` | `egg_carton.toml`, `paraboloid.toml`, `sphere_cap.toml` |
| `parametric` | `x`, `y`, `z` as functions of `u`, `v` | `u`, `v` ranges, optional `mask` | `cylinder_section.toml` |
| `implicit` | `F = "F(x, y, z)"` (surface is F = 0) | `x`, `y`, `z` bounding box | `torus_implicit.toml` |

Optional keys for every type:
- `visible_from`: `"+z"`, `"-x"` or `"x,y,z"`. Only surface facing that way
  counts toward completeness. Height fields default to `+z`.
- `flip_normals`: flip which side counts as positive signed error.
- `description`
- `[export]`: `resolution` and `base_z` for `--export`.

**What an expression may contain:**
- numbers, the variables, the names in `[params]`, and `pi` and `e`;
- `+ - * / ** %`;
- the functions `sin cos tan asin acos atan atan2 sinh cosh tanh exp log ln
  sqrt abs sign min max floor ceil hypot`;
- in a `mask` only, comparisons with `and`/`or`/`not`.

The string is parsed with Python's `ast` module and checked node by node
against that whitelist. The sympy expression is built directly from the
checked tree, so **nothing in the file is ever passed to `eval`**.
`__import__('os')`, attribute access, lambdas and strings are all rejected,
with the offending column. `^` is rejected with a hint to use `**`.

When a definition loads, it is evaluated over a grid covering its whole
domain. **An expression that is undefined anywhere in the domain is an
error**, and the message says where. For example,
`sqrt(R**2 - x**2 - y**2)` on a square larger than the circle fails with
the coordinates of the first bad point. Fix it with a `mask`, as
`sphere_cap.toml` does.

### Writing a plugin surface

For shapes that don't fit an expression (piecewise parts, lookup tables,
closed-form geometry), subclass `ReferenceSurface` and implement three
methods, in the units your `.toml` declares:

```python
# surfaces/plugins/my_part.py
import numpy as np
from scan_eval import ReferenceSurface

class MyPart(ReferenceSurface):
    exact = True
    def __init__(self, size):               # [params] arrive as keyword arguments
        self.size = size
    def sample(self, n, seed=0):             # -> (n,3) points, (n,3) unit normals, area-uniform
        ...
    def closest_points(self, points):        # -> closest surface point and unit normal there
        ...
    def bounds(self):                         # -> (lo, hi)
        ...
    # optional: height(x, y) with is_height_field = True, region_mask(), surface_area()
```

```toml
# surfaces/my_part.toml
name  = "my-part"
type  = "plugin"
units = "mm"
path  = "plugins/my_part.py"
class = "MyPart"
[params]
size = 40.0
```

Distances, signed error, region, metadata and hashing are all derived from
those three methods. A full worked example, a spherical cap with an exact
rim, is in `surfaces/plugins/sphere_cap_plugin.py`. On load, a plugin is
smoke-tested: wrong array shapes, or `closest_points` not landing on its own
samples, fail immediately with a message.

### Exporting an analytic reference (to print it, or to look at it)

```bash
.venv/bin/python evaluate_scans.py --reference-def surfaces/egg_carton.toml --export egg.stl --export-base 0
.venv/bin/python evaluate_scans.py --reference-def surfaces/torus_implicit.toml --export torus.ply --export-resolution 0.3
```

The mesh comes from the same definition the evaluator scores against, in the
definition's units (mm for a slicer). `--export-base Z` closes a height
field into a watertight, outward-facing solid down to z = Z.
Implicit surfaces go through marching cubes, with the vertices then snapped
exactly onto F = 0.

### Other flags

```bash
--init-transform prev/transform.txt   # seed the alignment (4x4, scan -> reference, metres)
--align refine | none                 # ICP only from the seed / use the seed as-is
--units mm   --scale 1.002            # scan units; an explicit correction factor
--allow-scale                         # similarity alignment; reports the recovered scale
--thresholds 0.25,0.5,1,2             # F-score / completeness thresholds, mm
--region-margin -2                    # inset the outline 2 mm (see "region" below)
--no-overlay  --overlay-samples 50000 # skip overlay.ply, or draw fewer reference points
--config eval_config.example.toml     # baseline settings; flags override
--list-only                           # show which scans would be evaluated
```

`--help` lists everything. All distances on the command line are in mm.

## Units and scale

Before aligning, both bounding boxes are printed in mm. If their sizes differ
by a unit-sized factor (mm vs m, cm vs m, inches), a **loud** warning names the
likely mix-up and the row is marked `units_warning` in `results.csv`.

**Nothing is rescaled automatically.** Scale error is one of the things being
measured, so the scan is scored at the scale it was delivered at:
- `--units` and `--reference-units` say what the files are in.
- `--scale` applies a factor you choose.
- `--allow-scale` solves for a uniform scale as part of a similarity
  alignment, and reports it as `recovered_scale`. The metrics are then
  computed on the rescaled scan.

A scan that also contains the table, or several parts, is normal: a factor of
a few between the two sizes does not trigger the warning.

## Alignment, and how it avoids look-alike poses

The egg-carton, and the reference shapes in general (sinusoids, spheres,
planes, cylinders), are periodic, symmetric or nearly featureless. That means
there are wrong poses that fit almost as well as the right one. So the
alignment doesn't trust any single method:

1. **Candidates** are gathered from several sources:
   - the `--init-transform` seed, if given, and the identity;
   - a moments start: centroids and principal axes matched, the scan's
     normal toward its sensor. This is the start that works on featureless
     shapes (sphere, plane) where FPFH finds nothing;
   - five RANSAC runs over FPFH features, each with a different seed;
   - systematic perturbations of the best pose so far: quarter turns about
     the surface normal, upside-down, and slides of ¼ and ½ of the part's
     extent. On a 2-period egg-carton those slides are exactly half a period
     and one period.
2. **Each candidate is refined** with point-to-plane ICP, coarse to fine,
   against dense reference samples carrying exact normals. Candidates that
   converge to the same pose are merged.
3. **Each candidate is scored** by one measure that does not depend on how
   the pose was found. It is a truncated least-squares cost over both
   directions:
   - how well the scan points over the part lie on the surface;
   - how much of the reference has scan near it.

   Every point costs at most τ² (τ = evaluation distance), and a point that
   matches nothing costs the full τ². So sliding the scan off the part's
   edges and outline is penalised even where the relief repeats. Scan
   points outside the part's outline (table, other objects) are left out
   entirely.
4. **The winner and the runner-up** are reported: fitness (overlap scan/ref),
   inlier RMSE, and the margin between them. Below 15% the run is flagged
   **`[AMBIGUOUS]`**: check it visually, or seed it with `--init-transform`.
5. **The final pose is polished against the exact surface.** Each iteration
   uses true closest points and normals from the reference, not samples,
   with MAD trimming of stragglers. Scale is also solved for when
   `--allow-scale` is set.
6. **Degeneracy check.** The final normal-equation matrix is analysed. Its
   rotation columns are scaled by the scan's radius so rotations and
   translations are comparable. Directions whose eigenvalue is tiny compared
   with the largest are named, e.g. *"alignment is UNCONSTRAINED in:
   rotation about Z"* for a paraboloid, or *"rotation about Z; translation
   along X; translation along Y"* for a plane. Error along an unconstrained
   direction is invisible to every metric.

Three things differ from Fuse, each for a measured reason:

- **The score.** Fuse ranks by `rmse / overlap`. On the real Out4 egg-carton
  scan, that chose a pose a quarter period off (84% of the reference
  covered, versus 99.8% for the right pose), because RMSE over inliers drops
  when a bad pose simply has fewer of them. The truncated cost cannot be
  improved by discarding points.
- **Which side the surface is seen from.** An open height field fits
  equally well from above and upside down. Scan normals are oriented toward
  the sensor, as Fuse's `estimateSensorNormals` does, and a match only counts
  if the scan normal and the reference's outward normal agree.
  `--sensor-origin` says where the camera was if it isn't at the scan
  frame's origin, which it is for RealSense and Fuse output.
- **The final refinement runs on original points.** Voxel centroids sit
  inside convex relief, which would bias the pose by a fraction of a voxel.

The chosen transform goes into `transform.txt`. Passing it back with
`--init-transform ... --align none` reproduces the run exactly. Every random
choice (surface sampling, RANSAC) comes from `--seed` (default 0), so a
repeated run gives the same result. Open3D's ICP sums in parallel threads,
so the last bits of the transform can differ (below 1e-9 m); no metric can
see that.

## Metrics

All distances are in **mm**. Error is measured in the reference frame after
alignment.

**Which scan points count.** Only points *over the part* are scored: those
inside the reference's region. For a height field that's its outline seen
from above; otherwise it's the bounding box. Points outside (table,
surroundings, other objects) are counted in `points_excluded` and otherwise
ignored. `--region-margin` grows the region, or insets it when negative.
Insetting is useful with an open analytic surface, which has no side walls,
while the printed part does. On Out4, RMSE is 2.0 mm against the analytic
egg-carton and 1.13 mm with the worst 5% trimmed. That gap is almost
entirely points on the part's walls.

| metric (`results.csv` column) | meaning |
|---|---|
| `acc_rmse_mm`, `acc_mae_mm`, `acc_median_mm`, `acc_std_mm`, `acc_p90/95/99_mm`, `acc_max_mm` | **Accuracy**: unsigned distance from each scored scan point to the *true surface*: exact for analytic references, to the triangles (not the vertices) for meshes. `mae` is the mean unsigned distance. |
| `signed_bias_mm`, `signed_std_mm`, `signed_median_mm`, `signed_p05/p95_mm` | **Signed error** along the surface normal. Positive means above/outside the surface. The bias is systematic offset; the std around it is random noise. |
| `acc_rmse_trim1_mm`, `acc_rmse_trim5_mm`, `acc_mae_trim*`, `signed_bias_trim*`, `signed_std_trim*` | **Robust** versions with the worst 1% / 5% of points (by \|error\|) removed, so a few strays don't dominate. |
| `z_bias_mm`, `z_std_mm`, `z_rmse_mm`, `z_mae_mm`, `z_p95_abs_mm`, `z_max_abs_mm` | **Vertical error** z − h(x,y), height fields only. This is not the distance: on a 45° slope it is √2 × the distance. |
| `covered_fraction` | Fraction of the visible reference surface with a scan point within `--coverage-radius` (default 10 mm). This is how much of the part the scan saw. |
| `completeness@τ` | Of the **covered** reference, the fraction within τ of any scan point (area-weighted: reference samples are area-uniform). A scan of half the part is judged on the half it saw. |
| `completeness_full@τ` | The same over the whole visible reference (uncovered area counts as missed). |
| `precision@τ` | Fraction of scored scan points within τ of the surface. |
| `fscore@τ` | Harmonic mean of `precision@τ` and `completeness@τ`. Thresholds: `--thresholds`, default 0.5, 1, 2, 5 mm. |
| `chamfer_mm`, `chamfer_rms_mm` | Mean of (mean scan→reference distance, mean covered-reference→scan distance), and the RMS version. |
| `hausdorff_mm`, `hausdorff95_mm` | Max of the two directions' maxima, and of their 95th percentiles. |
| `outlier_fraction` | Scored points farther than `--outlier-distance` (default 5 mm). |
| `points_total`, `points_scored`, `points_excluded`, `density_pts_per_cm2`, `spacing_median_mm` | Point counts; scored points per cm² of covered area; median nearest-neighbour spacing. |
| `recovered_scale`, `align_margin`, `align_ambiguous`, `align_unconstrained`, `align_weak`, `units_warning` | How much to trust the row. |
| `reference`, `reference_type`, `reference_exact`, `reference_hash` | Which truth was used. The hash comes from the canonical definition: expression tree, parameters, domain and units for analytic references, file contents and units for meshes, source and parameters for plugins. Whitespace or renaming doesn't change it; any real change does. |

Completeness is measured from reference samples to the nearest scan point,
so it is limited by scan density as well as accuracy. Accuracy is measured
from each scan point to the continuous surface, so it is not.

## Outputs

```
evaluation_out/
  results.csv                  one row per evaluated scan, appended across runs
  batch_summary.csv            the latest batch only, sorted by RMSE
  <label>__<scan>/
    metrics.json               every metric, all parameters, the reference definition + hash,
                               every alignment candidate with its score, degeneracy report
    transform.txt              4x4 scan -> reference (metres); reusable as --init-transform
    aligned_error.ply          aligned scan coloured by signed error (blue below, grey ~0,
                               red above; dark grey = not over the part), plus the error
                               as a scalar field `error_mm` for CloudCompare/MeshLab
    overlay.ply                the reference and the aligned scan in one cloud: orange is
                               the reference surface, blue the scan over it, grey the scan
                               elsewhere (table, other parts). Open it to check the
                               alignment and that this was the right reference - well
                               aligned, the blue sits on the orange and hides it; a shifted
                               pose or the wrong part shows orange peaks beside blue ones.
                               The scalar field `source` (0 reference, 1 scan, 2 scan
                               outside the part) separates them in a viewer
    error_histogram.png        signed error with bias and ±1 std
    error_map.png              top-down for height fields (plus a Z-error panel), otherwise
                               projected onto the best-fit plane
    run.log                    this scan's console output
```

If a later run uses different thresholds, `results.csv` gains the new columns
and keeps the old rows. The console ends with the batch sorted by RMSE, with
flags for ambiguous alignments, degenerate alignments and units warnings.

## Reference meshes in this repository

- `Egg-carton-smaller3cm.ply` is a clean closed solid. Its top is
  `25 − 15·sin(2πx/50)·cos(2πy/50)` mm, which is what `surfaces/egg_carton.toml`
  encodes.
- `Egg-carton-bigger-4cm.ply` is two shells stacked without being merged: a
  base box and the egg surface, sharing an internal double face at z = 10.
  - On load, the 8 faces on its non-manifold edges are removed.
  - It is then not watertight, so the tool warns that signed error comes
    from oriented face normals.
  - The internal face is excluded from completeness, because a
    self-occlusion ray test finds it hidden from above.
  - Its relief is not a pure sin·cos (at y = 6.25 mm the amplitude is
    ~17 mm, where sin·cos gives 14.1).

  It is used as-is, since the mesh *is* the truth.

## Tests

`.venv/bin/python -m pytest tests -s` (`-s` prints the discretisation table):

- **Synthetic recovery, on both analytic and mesh references.** Samples plus
  0.3 mm Gaussian noise are moved into a random camera frame. Checks:
  - the pose is recovered to < 0.2 mm at the part's corners;
  - RMSE is within 8% of the injected noise and the bias is ≈ 0.
- **One-period shift.** The scan is moved 50 mm and the alignment is seeded
  at the look-alike pose. The true pose must still be recovered, with the
  look-alike as runner-up.
- **Genuine ambiguity.** A scan covering one period-wide strip must be
  flagged `[AMBIGUOUS]`, with the two contenders exactly one period apart.
- **Parametric, implicit and plugin references.** The plugin is a
  featureless sphere cap, where FPFH finds nothing and the moments start
  does the work; its free rotation is named.
- **Degeneracy.** A plane reports rotation about Z and translation in X and
  Y; a paraboloid reports exactly rotation about Z.
- **Scale.** `--allow-scale` recovers a 2% scale error to 0.05%.
- **Determinism.** Repeated runs agree to 1e-9 m.
- **Mesh vs analytic.** The egg-carton is exported at 2, 1 and 0.5 mm.
  - The mesh's distance error falls ~4× per halving (O(h²) chord error) and
    stays under the curvature bound κh²/8.
  - Measured max error is 51, 12.8 and 3.2 µm.
  - All metrics agree with the analytic ones to under 3 µm at 0.5 mm; RMSE
    agrees to 0.1 µm.
- **Errors.** Unsafe and invalid expressions, undefined domains, missing,
  empty and faceless files, units mismatches, the watertight solid export,
  hash stability, and the CLI end to end.
