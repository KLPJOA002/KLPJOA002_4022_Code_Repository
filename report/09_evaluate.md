# 09 — Evaluate: geometric error of scans against a ground-truth reference (`Processing/Evaluate`)

**Summary.** `Processing/Evaluate` is a Python/Open3D/SymPy tool that measures how far a 3D scan (`.ply` point cloud, either one RealSense capture or a fused cloud from `../Fuse`) is from a known ground truth. The ground truth can be a triangle mesh (CAD STL/PLY), an exact analytic surface written as an expression in a TOML file (height field $z=f(x,y)$, parametric $S(u,v)$, or implicit $F(x,y,z)=0$), or a user-written Python plugin class. Each scan is aligned rigidly (optionally with a uniform scale) to the reference. The alignment uses a multi-start search: identity/seed, a principal-axis "moments" start, five FPFH+RANSAC starts, and quarter-turn/flip/slide perturbations. Every candidate is refined with point-to-plane ICP and scored with a symmetric truncated least-squares cost. The winning pose is then polished against the *exact* surface, and the normal equations are checked for unconstrained degrees of freedom. Metrics are then computed in mm: accuracy (RMSE/MAE/percentiles of the exact point-to-surface distance), signed bias/std, vertical Z error, coverage, completeness/precision/F-score at thresholds, Chamfer/Hausdorff, outliers and density. They go to a per-scan folder (`metrics.json`, `transform.txt`, coloured PLYs, plots, log) and one row per scan is appended to a shared `results.csv`. The package `scan_eval` is also imported as a library by `Processing/Parameter_Sweep`, `Processing/Filter_Sweep` and the untracked `Processing/Rugosity_Metrics`. Everything below comes from reading the source. Nothing was run, including the tests.

## Files covered

| Path (repo-relative) | Lines | Role |
|---|---|---|
| `Processing/Evaluate/README.md` | 408 | User documentation, design rationale, metric definitions, test summary |
| `Processing/Evaluate/eval_config.example.toml` | 38 | Example `--config` baseline |
| `Processing/Evaluate/requirements.txt` | 11 | Python deps (open3d, numpy, scipy, sympy, matplotlib, scikit-image, pytest) |
| `Processing/Evaluate/evaluate_scans.py` | 310 | CLI entry point: argument parsing, config merge, scan discovery, batch loop, summary |
| `Processing/Evaluate/scan_eval/__init__.py` | 8 | Public API (`ReferenceSurface`, `SurfaceError`, `UnitScaled`) |
| `Processing/Evaluate/scan_eval/log.py` | 69 | Fuse-style console logging, tee to per-scan `run.log` |
| `Processing/Evaluate/scan_eval/params.py` | 138 | `EvalParams` dataclass (all defaults), TOML config loader |
| `Processing/Evaluate/scan_eval/expression.py` | 217 | Safe AST-whitelist expression parser to SymPy, lambdify wrapper |
| `Processing/Evaluate/scan_eval/reference.py` | 224 | `ReferenceSurface` ABC, derived signed distance/region/hash, `UnitScaled` wrapper |
| `Processing/Evaluate/scan_eval/analytic.py` | 477 | Parametric / height-field / implicit references; Newton closest-point projection |
| `Processing/Evaluate/scan_eval/mesh_reference.py` | 262 | Mesh reference via Open3D `RaycastingScene`; cleaning and orientation |
| `Processing/Evaluate/scan_eval/surface_def.py` | 210 | Loads mesh files and TOML definitions (incl. plugin import and smoke test) |
| `Processing/Evaluate/scan_eval/registration.py` | 546 | Multi-start alignment, scorer, exact-surface refinement, degeneracy analysis |
| `Processing/Evaluate/scan_eval/metrics.py` | 153 | All error metrics |
| `Processing/Evaluate/scan_eval/pipeline.py` | 226 | Per-scan stage runner (load → units check → align → metrics → write) |
| `Processing/Evaluate/scan_eval/report.py` | 257 (working tree) | PLY writers, plots, JSON, `results.csv` appender (**uncommitted change**) |
| `Processing/Evaluate/scan_eval/export.py` | 144 | Export analytic references to STL/PLY/OBJ (grid triangulation, solid, marching cubes) |
| `Processing/Evaluate/surfaces/*.toml` | 5–28 each | Example reference definitions (egg-carton analytic + 2 meshes, paraboloid, sphere cap, cylinder section, torus, plugin) |
| `Processing/Evaluate/surfaces/plugins/sphere_cap_plugin.py` | 71 | Example plugin with closed-form closest point |
| `Processing/Evaluate/tests/conftest.py` | 88 | Fixtures, synthetic scanner |
| `Processing/Evaluate/tests/test_alignment.py` | 132 | Pose/noise recovery, periodic ambiguity, degeneracy, scale, determinism |
| `Processing/Evaluate/tests/test_mesh_vs_analytic.py` | 65 | Mesh discretisation error O(h²) and metric agreement |
| `Processing/Evaluate/tests/test_errors_and_cli.py` | 166 | Expression safety, definition errors, units check, export, hash, CLI end to end |

Git history: created in `0f33464` (2026-09-21, message "Added python code to evaluate experimental scan against ground truth. Untested"); overlay PLY added in `59b3365` (2026-09-25); unchanged since apart from the uncommitted `report.py` edit.

## Contents

1. Purpose
2. Inputs and outputs
3. Step-by-step walkthrough
4. Algorithms and methods
   - 4.1 Safe expression parser
   - 4.2 Analytic references and closest-point projection
   - 4.3 Mesh references
   - 4.4 Plugin references
   - 4.5 Registration
   - 4.6 Metrics
   - 4.7 Export
5. Why this approach
6. Parameters and tuning
7. Assumptions, limitations, failure modes
8. Dependencies
9. Tests: what they verify
10. The uncommitted `report.py` change
11. Observations

---

## 1. Purpose

The project needs a quantitative answer to "how accurate is this scan?". The answer has to hold across hardware settings (resolution, depth units, preset), across filter chains, and across fusion algorithms. Evaluate solves that by comparing each scan to an independently known shape: the printed test parts (egg-carton "wave", pyramids) via their CAD meshes or exact equations. It sits downstream of Capture (`rs_marker` PLYs, metres), Crop and Fuse, and upstream of the sweep analyses, which call `scan_eval.pipeline.evaluate_scan` directly. **[EXPLICIT: README.md:1-10; evaluate_scans.py:1-23]**

The README calls it the sibling of `../Fuse`: Fuse aligns scans *to each other* and merges them, while Evaluate aligns each scan *to the reference* and fuses nothing **[EXPLICIT: README.md:9-10]**.

## 2. Inputs and outputs

### 2.1 CLI (`evaluate_scans.py:52-150`)

| Flag | Dest / default | Meaning |
|---|---|---|
| `scans` (positional) | — | `.ply` files and/or directories (recursive `*.ply`, `find_scans` `evaluate_scans.py:181-198`, de-duplicated by resolved path) |
| `--reference PATH` | — | Mesh (`.stl/.ply/.obj/.off/.gltf/.glb`) or `.toml`; kind chosen by extension |
| `--reference-def PATH` | — | TOML definition (mutually exclusive with `--reference`, `evaluate_scans.py:204`) |
| `--reference-units` | bare mesh → `mm` | `m, cm, mm, um, in`; overrides a definition's units |
| `--height-field` | off | Treat a bare mesh as $z=h(x,y)$ |
| `--visible-from DIR` | from definition (height fields `+z`) | `+z`, `-x`, `x,y,z` or `none` |
| `-o/--output` | `evaluation_out` | Output root |
| `--label` | none | Row label; prefix of run folder names |
| `--config TOML` | — | Baseline params; flags override (`resolve_params`, `evaluate_scans.py:153-178`) |
| `--seed` | 0 | All random choices |
| `--list-only` | — | Print scans and exit |
| `--units` | `m` | Units of scan PLYs |
| `--scale` | 1.0 | Extra multiplicative factor on scans |
| `--allow-scale` | off | Similarity (7-DOF) alignment |
| `--align full/refine/none` | `full` | Search mode |
| `--init-transform FILE` | — | 4×4 seed, scan m → reference m |
| `--align-voxel MM` | 0 → diag/60 | Voxel for the alignment copy |
| `--eval-distance MM` | 0 → 3×voxel | Correspondence distance τ for scoring |
| `--starts` | 5 | RANSAC starts |
| `--no-perturb` | perturb on | Skip turn/flip/slide multi-start |
| `--sensor-origin X,Y,Z` | `0,0,0` mm | Camera position in scan frame (normal orientation), or `none` |
| `--ambiguous-margin` | 0.15 | Margin below which the winner is flagged ambiguous |
| `--thresholds MM,...` | 0.5,1,2,5 | Precision/completeness/F thresholds |
| `--outlier-distance` | 5 mm | Outlier threshold |
| `--coverage-radius` | 10 mm | "Covered" radius for reference samples |
| `--region-margin` | 0 mm | Grow (+) / inset (−) the scored region |
| `--trim PCT,...` | 1,5 | Robust-stat trim percentages |
| `--reference-samples` | 200000 | Reference samples for completeness |
| `--no-ply / --no-plots / --no-overlay` | all written | Output toggles |
| `--overlay-samples` | 150000 | Reference points in `overlay.ply` |
| `--export PATH`, `--export-resolution R`, `--export-base Z` | — | Export the reference as a mesh and exit |

Exit code: 0 on success, 1 if any scan failed or on a load/parse error (`evaluate_scans.py:298-306`).

### 2.2 Reference definition TOML (`surface_def.py:1-142`)

| Key | Types allowed | Notes |
|---|---|---|
| `name`, `type`, `units`, `description`, `visible_from`, `flip_normals`, `params`, `export` | all | `type` ∈ `height_field, parametric, implicit, mesh, plugin`; `units` is required (`surface_def.py:87-90`); unknown keys are an error (`surface_def.py:82-84`) |
| `[surface] z` / `x,y,z` / `F` | analytic | expression strings |
| `[domain] x,y` (+`mask`) / `u,v` (+`mask`) / `x,y,z` | analytic | `[min,max]` ranges |
| `path`, `height_field` | mesh | path relative to the TOML file |
| `path`, `class` | plugin | Python file relative to the TOML; `[params]` → constructor kwargs |

### 2.3 Units and frames

- Scans are read as `--units` (default metres: "RealSense and Fuse output are metres", `scan_eval/params.py:24-25`) times `--scale` (`scan_eval/pipeline.py:47`).
- References are evaluated in their own units, but the pipeline only sees them through `UnitScaled`, which presents them in metres (`reference.py:159-215`, `evaluate_scans.py:262`).
- Internally everything is metres. All user-facing distances (flags, config, CSV/JSON keys `*_mm`) are millimetres (`scan_eval/params.py:5-8`; `metrics.py:3-5`).
- The transform $T$ maps scan frame → reference frame, in metres (`registration.py:94`).
- The sensor is assumed at the scan-frame origin unless `--sensor-origin` says otherwise (`scan_eval/params.py:68-73`).

### 2.4 Outputs (`report.py:1-15`, `scan_eval/pipeline.py:131-196`)

| File | Format | Content |
|---|---|---|
| `<out>/<run>/metrics.json` | JSON (NaN allowed) | timestamp, label, scan, `reference.describe()`, `reference_definition`, all parameters, units warning, alignment (transform, scale, chosen/runner-up/all candidates, margin, degeneracy, refinement info), all metrics, elapsed time |
| `<out>/<run>/transform.txt` | `np.savetxt` 4×4, `%.12f`, `#` header | scan→reference, metres; reusable as `--init-transform` |
| `<out>/<run>/aligned_error.ply` | binary little-endian PLY, float32 xyz, uchar RGB, float `error_mm`, uchar `scored` | aligned scan coloured blue/grey/red by signed error (±98th percentile), grey (110,110,110) off-part |
| `<out>/<run>/overlay.ply` | same writer, extra uchar `source` (0 ref, 1 scan on part, 2 scan off part) | reference samples (orange) + aligned scan (blue/grey) |
| `<out>/<run>/error_histogram.png` | PNG 130 dpi | signed error histogram with bias and ±1σ band |
| `<out>/<run>/error_map.png` | PNG | top-down map (height field: plus Z-error panel) or best-fit-plane projection |
| `<out>/<run>/run.log` | text | tee of console output for that scan (debug lines always included) |
| `<out>/results.csv` | CSV | one row per scan, appended across runs; schema is widened when new columns appear |
| `<out>/batch_summary.csv` | CSV | this batch only, sorted by RMSE (only when >1 scan succeeded, `evaluate_scans.py:290-296`) |

Run folder name: `safe_name("<label>__<stem>")`. If two scans share a stem, the parent folder name is prefixed (`scan_eval/pipeline.py:94-100`).

## 3. Step-by-step walkthrough

1. **Parse and merge parameters.** `main` (`evaluate_scans.py:238`) builds the parser and sets verbosity, then calls `resolve_params` (`:153`). The merge order is `EvalParams()` defaults → `load_config` (TOML, sections flattened, `scan_eval/params.py:117-134`) → non-`None` CLI values. Unknown config keys raise (`:160-162`). The config may also carry `reference`, `reference_def`, `height_field` and `scans` (`:159`).
2. **Find scans** (`:244`, `find_scans` `:181`). With `--list-only` it prints them and exits (`:245-249`).
3. **Load the reference** (`:251-252` → `load_reference_from_args` `:201` → `surface_def.load_reference` `surface_def.py:50`):
   - mesh extension → `MeshReference` (default units mm; `units_source` recorded, `:58-61`);
   - `.toml` → `load_definition` (`:71`), which validates keys and units, validates `[params]` as finite numbers (`expression.validate_params`, `expression.py:117-127`), then builds `HeightFieldReference` / `ParametricReference` / `ImplicitReference` / `MeshReference` / plugin.
4. **Export mode.** With `--export`, `export.export_reference` runs and the program exits (`evaluate_scans.py:253-256`).
5. **Wrap in metres** (`UnitScaled`, `:262`). It logs kind, exactness, definition hash and visible surface area (`:263-271`).
6. **Optional seed.** `pipeline.read_transform` (`scan_eval/pipeline.py:50-59`) checks the shape is 4×4, the values are finite and the last row is `0 0 0 1`.
7. **Per-scan loop** (`evaluate_scans.py:283-288`). It calls `pipeline.evaluate_scan` (`scan_eval/pipeline.py:103`). `ValueError`/`FileNotFoundError`/`SurfaceError` are caught per scan and recorded as failures.
   1. **Tee the log.** `run.log` is truncated and attached as a tee (`:108-110`).
   2. **Load** (`load_scan` `:27-47`). The file must be `.ply`. Open3D reads it as a point cloud, then again as a triangle mesh to warn when the "scan" is actually a mesh (`:35-38`). Non-finite points are dropped, at least 100 points are required, and coordinates are multiplied by `unit_factor(units)*scale`.
   3. **Units check** (`check_units` `:62-91`). It prints both bounding boxes in mm. If the largest-extent ratio lies outside (1/60, 60), it picks the closest unit factor from {1000, 100, 10, 25.4} and prints a loud banner. Nothing is rescaled.
   4. **Align** (`registration.align` `registration.py:386`, detailed in §4.5), then print the candidate table (`print_alignment_report` `:508`).
   5. **Apply** $T$ to every original point (`scan_eval/pipeline.py:125`).
   6. **Metrics** (`metrics.compute_metrics` `metrics.py:56`, §4.6), printed by `print_metrics` (`scan_eval/pipeline.py:201`).
   7. **Write** `transform.txt` (with a reuse hint in the header), `aligned_error.ply`, `overlay.ply` (the reference is re-sampled with seed+2 so the overlay never shows the metric samples, `:146-149`), plots, `metrics.json`, and the CSV row (`:131-196`).
8. **Summary and exit.** `print_summary` sorts the batch by RMSE and flags AMBIGUOUS/DEGENERATE/UNITS? (`evaluate_scans.py:213-235`). `batch_summary.csv` is written, failures are listed, and the exit code is set.

## 4. Algorithms and methods

### 4.1 Safe expression parser (`scan_eval/expression.py`)

**Problem.** SymPy's `sympify`/`parse_expr` run Python `eval`, so a malicious TOML could execute code **[EXPLICIT: expression.py:3-8]**.

**Method.** The parser is a whitelist AST walker:

- The text is limited to 4000 chars (`:40,71`) and parsed with `ast.parse(mode="eval")` (`:87`).
- `_Builder.visit` dispatches on node type. Any node type without a `visit_*` method is rejected with its column (`:141-145`).
- Allowed nodes:
  - `Constant`: int/float only; bool and str rejected (`:147-150`).
  - `Name`: declared variables (as `sp.Symbol(real=True)`), parameters (kept symbolic so the hash doesn't depend on number formatting, `:65-67`), and `pi`, `e` (`:152-159`).
  - `BinOp`: `+ - * / ** %`. `^` gets a hint to use `**` (`:161-163`). A constant exponent above 1000, or a base above $10^6$ with an exponent above 10, is rejected to stop SymPy from evaluating things like `9**9**9` exactly (`:41-43,168-172`).
  - `UnaryOp`: `-`, `+`, and `not` (mask only).
  - `Call`: only a plain `Name` in `FUNCTIONS` (sin…hypot, `:28-36`), with no kwargs or `*args`.
  - `Compare` and `BoolOp`: only when `allow_logic=True`, i.e. for domain masks (`:201-217`).
- Parameters may not shadow variables, constants or functions (`:78-83`).

**Evaluation.** After `substitute_params`, `lambdify` produces a NumPy function. The wrapper broadcasts constant results to the input shape, because a constant derivative (e.g. of a plane) would otherwise come back as a scalar (`:99-114`). It also suppresses floating-point warnings (`np.errstate(all="ignore")`), so NaN is detected afterwards rather than raised.

### 4.2 Analytic references and closest-point projection (`scan_eval/analytic.py`)

#### Parametric and height-field surfaces

A parametric surface is $S(u,v) = (x(u,v), y(u,v), z(u,v))$ on a rectangle, optionally masked (`ParametricReference`, `:52`). A height field is the special case $S(x,y) = (x, y, f(x,y))$, so both share one solver (`HeightFieldReference`, `:283-293`) **[EXPLICIT: analytic.py:21-22]**.

**Symbolic derivatives.** SymPy computes $S_u, S_v, S_{uu}, S_{uv}, S_{vv}$ exactly and lambdifies them (`_build`, `:81-102`).

**Normals.** $\mathbf n = \pm (S_u\times S_v)/\lVert S_u\times S_v\rVert$, negated when `flip_normals` (`_normal`, `:134-136`). For a height field this gives $(-f_x,-f_y,1)/\sqrt{1+f_x^2+f_y^2}$, which points up (+z).

**Domain validation** (`_check_domain`, `:109-131`). $S$, $S_u$ and $S_v$ are evaluated on a 257×257 grid inside the mask. Any NaN/inf raises `SurfaceError` with the first bad coordinate. An area element $\lVert S_u\times S_v\rVert < 10^{-12}$ on more than 0.1% of the domain gives a warning.

**Closest point: damped Newton in $(u,v)$** (`_project`, `:152-210`). It minimises $f(u,v) = \lVert S(u,v)-\mathbf p\rVert^2$ (the code's objective, `:161-163`). With residual $\mathbf r = S-\mathbf p$:

$$
\mathbf g = \begin{pmatrix} S_u\cdot\mathbf r\\ S_v\cdot\mathbf r\end{pmatrix},\qquad
H = \begin{pmatrix} S_u\cdot S_u + S_{uu}\cdot\mathbf r & S_u\cdot S_v + S_{uv}\cdot\mathbf r\\ \cdot & S_v\cdot S_v + S_{vv}\cdot\mathbf r\end{pmatrix}
$$

- $\mathbf g$ and $H$ are the gradient and Hessian of $\tfrac12 f$ (`:172-178`).
- **Initial guess.** Nearest of a 600×600 grid of surface points (cKDTree, `:138-145,155-157`). That is about 0.17 mm spacing on a 100 mm part, "well inside Newton's basin" **[EXPLICIT: analytic.py:36-39]**.
- **Full Newton where $H$ is positive definite.** Otherwise it switches to Gauss–Newton ($H \approx J^\top J$, always PSD), e.g. on the concave side past a focal point (`:179-186`) **[EXPLICIT: comment :179-180]**.
- **Regularisation.** A tiny Tikhonov term, $10^{-12}(H_{ii}+1)$, is added (`:187-188`).
- **Line search.** Steps are box-clamped to the domain, with backtracking that halves $t$ up to 12 times and accepts any non-increase (`:191-206`).
- **Stopping.** At most 60 iterations, or when a point moves less than $10^{-12}\times$span (`:159,167,207-209`).

**Masked domains** (`closest_points`, `:213-226`). If the unconstrained optimum lands outside the mask, the nearest *in-mask* seed is used instead. That is accurate only to the seed spacing, and only at the rim **[EXPLICIT: :219-222]**.

**Area-uniform sampling** (`sample`, `:228-244`). This is rejection sampling: draw $(u,v)$ uniformly and accept with probability $\lVert S_u\times S_v\rVert/(1.1\max)$. The accepted points have density proportional to surface area.

**Surface area** (`:250-262`). The trapezoidal rule on a 600² grid of $\lVert S_u\times S_v\rVert$, restricted to the mask and, if `visible_from` is set, to samples whose normal faces it with $\cos > 0.05$.

**Height-field extras.**
- `height(x,y)` returns NaN outside the domain/mask (`:296-303`).
- `region_mask` is the $(x,y)$ rectangle grown by `margin`. With a mask, a point counts if the mask holds at the clipped coordinates, or (for positive margin only) if it lies within `margin` of the rectangle (`:305-314`).

#### Implicit surfaces $F(x,y,z)=0$ (`ImplicitReference`, `:317-477`)

- **Gradient and normal.** The gradient $\nabla F$ is symbolic. The normal is $\pm\nabla F/\lVert\nabla F\rVert$ (`:339-341,386-388`).
- **Domain check** (`:345-373`). $F$ must be finite on a 65³ grid. $\nabla F$ must be finite only in cells where $F$ changes sign, so a torus written with $\sqrt{x^2+y^2}$ (no gradient on its axis) is allowed **[EXPLICIT: :350-353]**. $F$ must change sign somewhere in the box.
- **Newton-to-surface** (`_newton_to_surface`, `:375-384`). The first-order projection along the gradient:
  $$\mathbf q \leftarrow \mathbf q - \frac{F(\mathbf q)}{\lVert\nabla F\rVert^2}\nabla F$$
  It stops when $|F|/\lVert\nabla F\rVert < 10^{-12}\times$box span, or after 40 iterations.
- **Seed pool** (`:390-415`). 3 × 400 000 uniform random box points (fixed RNG seed 12345) are projected onto $F=0$. Points that converged and lie inside the box are kept, then thinned to one per voxel of size diag/500 to approximate area-uniformity.
- **Closest point: foot-point iteration** (`closest_points`, `:417-436`). Start from the nearest pool point $\mathbf x$, then repeat up to 100 times:
  1. Project $\mathbf p$ onto the tangent plane at $\mathbf x$: $\mathbf q = \mathbf p - ((\mathbf p-\mathbf x)\cdot\mathbf n)\mathbf n$.
  2. Pull $\mathbf q$ back onto $F=0$ with up to 8 Newton steps.
  3. Stop when the move is below $10^{-10}\times$span.

  At convergence $\mathbf p-\mathbf x$ is parallel to $\mathbf n$, which is the orthogonality condition for a closest point. **No box constraint** is applied (see Observations).
- **Sampling.** Draws from the thinned pool (with replacement if $n$ exceeds the pool size) (`:438-443`).
- **Region.** The bounding box (`:449-451`).
- **Area.** From a marching-cubes mesh, which needs scikit-image (`:453-465`).

#### Signed and unsigned distance (`reference.py:98-110`)

With $\mathbf c(\mathbf p)$ the closest surface point and $\mathbf n$ the unit normal there:

$$ d(\mathbf p) = \lVert\mathbf p-\mathbf c\rVert, \qquad d_s(\mathbf p) = d\cdot\operatorname{sign}\big((\mathbf p-\mathbf c)\cdot\mathbf n\big) $$

The sign is set to $+1$ when the dot product is exactly 0. Positive means the point is on the normal side ("above/outside").

#### Definition hash (`reference.py:133-142`, `analytic.py:264-273`)

SHA-256 of a canonical JSON with sorted keys. Analytic references hash `sp.srepr` of the symbolic expressions (parameters kept as symbols), the mask, `repr` of the parameter values, the domain, units and flip flag. Whitespace and renaming therefore don't change the hash, but any real change does **[EXPLICIT: README.md:326]**.

### 4.3 Mesh references (`scan_eval/mesh_reference.py`)

**Loading** (`:28-74`):
- The file is read with `o3d.io.read_triangle_mesh`. A file with no vertices is an error, and so is one with vertices but no faces: "distances to sampled points would measure the sampling, not the part" **[EXPLICIT: :40-44]**.
- The file's SHA-256 feeds the definition hash.

**Cleaning and orientation** (`_clean_and_orient`, `:204-262`):
1. Remove duplicated vertices/triangles, degenerate triangles and unreferenced vertices.
2. If the mesh is not edge-manifold, call `remove_non_manifold_edges` and warn. The rationale is "an internal face left over from modelling" **[EXPLICIT: :216-218]**.
3. Closedness: `is_closed` checks that every undirected edge is shared by exactly 2 faces. A NumPy check is used instead of Open3D's `is_watertight`, which takes 20 s on 200k faces **[EXPLICIT: :174-176]**.
4. Consistent winding: `consistently_oriented` checks that every directed edge is unique. If that fails, Open3D `is_orientable`/`orient_triangles` are tried.
5. Outward direction:
   - closed mesh: the divergence-theorem signed volume $V=\tfrac16\sum \mathbf v_0\cdot(\mathbf v_1\times\mathbf v_2)$ must be > 0;
   - open mesh: an area-weighted vote of $\mathbf n_f\cdot(\mathbf c_f-\bar{\mathbf v})$.

   If the result is inward, all triangles are flipped.
6. Warn if the mesh is not watertight: the sign then comes from face normals.

**Distance queries.**
- Open3D's `t.geometry.RaycastingScene.compute_closest_points` gives the exact closest point on the triangles and the `primitive_ids`. The normal is that face's normal (`:87-95`).
- The scene is float32, so vertices and queries are centred on the vertex mean to keep rounding below a micrometre **[EXPLICIT: :64-66]**.

**Height field** (`:100-114`). $h(x,y)$ comes from casting a ray straight down from above the mesh. A miss gives NaN.

**Region for a height field** (`:116-136`). The footprint seen from above: a vertical ray must hit the mesh. A positive margin accepts a hit at any of 4 offset positions $(\pm m,0),(0,\pm m)$. A negative margin requires all of them to hit (inset).

**Visibility** (`:138-152`). Samples must face `visible_from` ($\cos>0.05$, `reference.py:218-224`) and must not be self-occluded. Occlusion is tested by casting a ray from each sample toward the scanner and checking for any hit. This is how the internal double face of `Egg-carton-bigger-4cm.ply` is excluded **[EXPLICIT: README.md:368-375]**.

**Sampling** (`:77-85`). Faces are chosen with probability proportional to area, then a point is placed in the triangle with the standard uniform barycentric formula $\mathbf p=(1-\sqrt{r_1})\mathbf a+\sqrt{r_1}(1-r_2)\mathbf b+\sqrt{r_1}r_2\mathbf c$ (Osada et al. 2002).

### 4.4 Plugin references (`surface_def.py:145-206`, `surfaces/plugins/sphere_cap_plugin.py`)

**Loading** (`surface_def.py:153-186`):
- The plugin file is imported with `importlib.util.spec_from_file_location` under a hashed module name. The class must subclass `ReferenceSurface` and is instantiated with `**params`.
- `definition()` is wrapped to add the plugin source SHA-256, class name and params, so editing the plugin changes the hash.

**Smoke test** (`_smoke_test_plugin`, `:189-206`):
- Sample 64 points and offset each by 1e-3 along its normal.
- `closest_points` must return finite (64,3) arrays, and they must land within 1% of the bounding-box diagonal of the original samples.
- `bounds()` must return two length-3 arrays.

**Example `SphereCap`** (`sphere_cap_plugin.py`):
- Geometry: the sphere centre is at $(0,0,-d)$ with $d=\sqrt{R^2-a^2}$.
- Sampling: uniform $\cos\theta\in[d/R,1]$ (Archimedes' hat-box theorem makes a uniform-$z$ zone area-uniform) (`:33-40`).
- Closest point: radial projection on the cap. Outside the cap's cone it snaps to the rim circle (`:42-56`).
- Region: a disc of radius $a$+margin. Area: $2\pi R(R-d)$.

### 4.5 Registration (`scan_eval/registration.py`)

**Overview** (`align`, `:386-505`). All lengths are in metres.

1. **Scales.** `diag` is the reference bounding-box diagonal. `voxel` = `align_voxel_mm` or diag/60. τ = `eval_distance` = `eval_distance_mm` or 3·voxel (`:388-392`).
2. **Reference samples.** `max(reference_samples//2, 20000)` samples are drawn and only visible ones kept, with exact normals (`:398-400`). They are voxel-downsampled at voxel/2 for ICP (`:402-403`).
3. **Scan preparation** (`prepare_scan`, `:115-127`). A copy of the scan, used for alignment only:
   - voxel downsample;
   - statistical outlier removal (30 neighbours, 1σ) when there are more than 60 points;
   - normals from Open3D `estimate_normals` (hybrid radius 4·voxel, max 30 NN);
   - normals oriented toward the sensor origin (`orient_normals_towards_camera_location`).

   This mirrors Fuse's `preprocessCloud` + `estimateSensorNormals` **[EXPLICIT: :117-120]**.
4. **Scorer** (`Scorer`, `:174-247`), described below.
5. **Candidates** (`add`, `:417-428`). Each start is refined by ICP (unless `none` mode) and scored. Candidates are merged when all 8 bounding-box corners land within τ of each other (`same_pose`, `:250-251`). The better-scoring version is kept and the start names are accumulated.
6. **Starts, by mode:**
   - `none`: score the given transform (or identity) and return immediately. No refinement and no degeneracy analysis (`:431-436`).
   - `refine` and `full`: `init-transform` (if given) and `identity` (`:438-440`).
   - `full` additionally:
     - **Moments start** (`:443-457`). Centroid-and-PCA alignment. `reference_frame` (`:254-264`) gives centre, major axis $u$, $v=n\times u$ and the least-variance normal $n$, oriented toward `visible_from`. The same is computed for the downsampled scan, with its normal flipped toward the sensor; $s_v$ is flipped too, so the frame stays right-handed. Then $R = [u\,v\,n][s_u\,s_v\,s_n]^\top$ and $\mathbf t = \mathbf c_{ref} - R\mathbf c_{scan}$.
     - **FPFH + RANSAC ×5** (`:459-467`). FPFH (Rusu et al., 2009) on both clouds with radius 10·voxel and 100 NN (`fpfh`, `:142-144`). Open3D `registration_ransac_based_on_feature_matching` with mutual filter on, max correspondence 2·voxel, point-to-point (no scale) estimation, `ransac_n=3`, edge-length checker 0.9 plus distance checker, 100 000 iterations, confidence 0.999. Open3D's global RNG is seeded with `seed+k` (`ransac`, `:147-158`). RANSAC is Fischler & Bolles (1981).
     - **Perturbations** of the current leader (`:470-481`): rotations of 90/180/270° about the reference normal through its centroid, a 180° "upside-down" flip about $u$, and slides of ±¼ and ±½ of the reference extent along $u$ and $v$. That makes 3 + 1 + 8 = 12 moves, each refined and scored.
7. **ICP** (`icp`, `:161-171`). Open3D `registration_icp` with `TransformationEstimationPointToPlane`, i.e. point-to-plane ICP (Chen & Medioni 1991; ICP framework Besl & McKay 1992). It runs as a coarse-to-fine schedule with max correspondence distances 6, 3, 1.5 × voxel, each stage up to 60 iterations and relative fitness/RMSE tolerance 1e-7. Target normals are the exact reference normals.
8. **Selection** (`:483-491`). Candidates are sorted by score. The run is ambiguous if
   $$\text{margin} = \frac{s_{runner}-s_{best}}{s_{runner}} < \texttt{ambiguous\_margin}\ (0.15).$$
   The run is "failed" if the best candidate has zero scan overlap or an infinite RMSE; the report then prints "ALIGNMENT FAILED" (`:509-511`).
9. **Exact refinement** on at most 100 000 original points (stride-subsampled), not voxel centroids. The stated reason is that "voxel centroids sit inside convex relief … would bias the pose" **[EXPLICIT: :493-495]** (`refine_exact`, `:269-335`).
10. **Degeneracy analysis** of the final normal equations (`degeneracy`, `:342-382`).

#### Scorer: symmetric truncated least squares (`:174-247`)

**Setup.** The scan (≤20 000 points by stride) and reference subsets (≤20 000) are KD-trees. τ is the evaluation distance. For a pose $T$ the score has two halves.

**Scan → reference half.**
- Only moved scan points inside `region_mask` are used ("over" the part).
- For each, find the nearest reference *sample* and take the point-to-plane residual $|(\mathbf q-\mathbf r)\cdot\mathbf n_r|$.
- A point is an inlier if its Euclidean offset is below τ **and** its sensor-oriented scan normal, rotated by $T$, agrees with the reference normal (dot > 0). This "facing" test distinguishes a thin surface right-way-up from upside-down **[EXPLICIT: :219-221]**.
- Cost per point: $\min(\text{plane}^2,\tau^2)$ for inliers, $\tau^2$ otherwise.

**Reference → scan half.**
- Reference subsamples are transformed into the scan frame with $T^{-1}$.
- Find the nearest scan point within τ. If found, the cost is $\min(((\mathbf s-\mathbf q)\cdot\mathbf n)^2,\tau^2)$, else $\tau^2$.

**Score.**
$$ s(T) = \sqrt{\tfrac12\left(\overline{c_{scan}} + \overline{c_{ref}}\right)} $$
If no scan point is over the part, the scan term is $\tau^2$. The scorer also reports `overlap_scan` (inlier fraction), `overlap_ref` (fraction of reference samples matched) and `rmse` (RMS plane residual of inliers).

This is a truncated least-squares (Geman–McClure-like capped quadratic) robust cost. Every point is capped at $\tau^2$, so a pose cannot lower its cost by discarding points **[EXPLICIT: :187-190]**.

#### Exact-surface point-to-plane refinement (`refine_exact`, `:269-335`)

Each iteration (max 50):

1. $Q = T\,P$. True closest points $C$ and normals $N$ come from the reference (exact for analytic, triangle-exact for meshes). Residual $r_i = (\mathbf q_i-\mathbf c_i)\cdot\mathbf n_i$.
2. **Keep set.** Points within τ of the surface and inside the region. Then MAD trimming: keep $|r_i-\tilde r| < \max(3\hat\sigma, 10^{-6})$ with $\hat\sigma = 1.4826\,\mathrm{median}|r_i-\tilde r|$ (the consistent MAD estimator of σ under Gaussian noise). Abort with a warning if fewer than 20 points remain.
3. **Linearise** about the kept centroid, with $\mathbf X_i = \mathbf q_i-\bar{\mathbf q}$. For a small rotation $\boldsymbol\omega$, translation $\mathbf t$ and (if `allow_scale`) scale increment $\sigma$:
   $$ r_i + (\mathbf X_i\times\mathbf n_i)\cdot\boldsymbol\omega + \mathbf n_i\cdot\mathbf t + (\mathbf X_i\cdot\mathbf n_i)\,\sigma \approx 0 $$
   The Jacobian rows are $J_i=[\mathbf X_i\times\mathbf n_i,\ \mathbf n_i,\ (\mathbf X_i\cdot\mathbf n_i)]$ (`:291-296`).
4. **Column balancing.** The rotation and scale columns are scaled by $1/L$, where $L$ is the RMS radius. The system $(J_s^\top J_s)\,x' = -J_s^\top r$ is solved with `np.linalg.lstsq(..., rcond=1e-6)`. That is a truncated-SVD pseudo-inverse, so null directions (sphere, plane) give zero step rather than a huge one **[EXPLICIT: :303-306]**. The solution is un-scaled afterwards (`:307-313`).
5. **Update.** Rodrigues rotation $R=\exp([\boldsymbol\omega]_\times)$ (`rotation_about`, `:49-56`). The incremental transform about centroid $\mathbf c_0$ is $M:\ \mathbf x\mapsto s R(\mathbf x-\mathbf c_0)+\mathbf c_0+\mathbf t$, then $T\leftarrow MT$ (`:315-323`).
6. **Safeguard.** If the RMS residual rises by more than 50% compared with the previous iteration, revert and stop (`:298-302`).
7. **Convergence.** Stop when $\lVert\mathbf t\rVert + |\boldsymbol\omega|L + |s-1|L < 2\,\mu$m. Tighter is pointless because mesh queries "dither at about a micron" **[EXPLICIT: :325-328]**.
8. **Returned scale.** $\det(R_T)^{1/3}$ (`:334`).

This is a Gauss–Newton point-to-plane ICP against the continuous surface, i.e. Chen & Medioni with exact foot points instead of nearest samples.

#### Degeneracy (observability) analysis (`degeneracy`, `:342-382`)

- **Normalise.** The final $J$ is copied with its rotation (and scale) columns divided by the RMS radius of the aligned on-surface points. $H = J_s^\top J_s/N$ is formed and eigen-decomposed.
- **Classify.** Eigenvalue ratios $\lambda_i/\lambda_{max}$ below 1e-4 are "unconstrained" and below 1e-2 "weak".
- **Name.** For each canonical DOF (rotation about X/Y/Z, translation along X/Y/Z, scale), the squared projection onto the weak subspace is computed. If it exceeds 0.8 the DOF is named; otherwise "coupled motion" vectors are printed.

This is the same idea as the covariance-based ICP stability analysis of Gelfand et al. (2003), though the code does not cite it. The output is surfaced as `align_unconstrained` / `align_weak` columns and a warning.

### 4.6 Metrics (`scan_eval/metrics.py`)

Inputs: the aligned scan (metres, reference frame) and `EvalParams`. Let $\mathcal S$ be the scan points inside `region_mask(margin)` (the "scored" points). Points outside are only counted (`:66-73`). Each reported quantity is in mm.

**Accuracy (scan → surface, unsigned)** (`_stats`, `:26-39`). With $d_i=|d_s(\mathbf p_i)|$ the exact distance to the true surface:

- MAE $=\frac1N\sum d_i$
- RMSE $=\sqrt{\frac1N\sum d_i^2}$
- median, std
- p90/p95/p99 (`np.percentile`, linear interpolation)
- max

**Signed** (`_signed_stats`, `:42-53`):

- bias $=\frac1N\sum d_{s,i}$
- std = population std (ddof=0)
- median, RMSE, p05, p95

**Outliers** (`:80`): fraction with $d_i > 5$ mm.

**Trimmed** (`:84-91`): sort by $d_i$ and keep the best $\mathrm{round}(N(1-p/100))$ (at least 1). Report RMSE, MAE, signed bias and signed std for each $p$ in `trim_percent` (1 %, 5 %).

**Vertical error** (height fields only, `:94-105`): $e_z = z_i - h(x_i,y_i)$ where $h$ is defined. Bias, std, RMSE, MAE, p95(|·|), max(|·|). On a slope of angle $\theta$, $|e_z| = d/\cos\theta$ **[EXPLICIT: README.md:315]**.

**Coverage and completeness (reference → scan)** (`:108-121`):
- Draw `reference_samples` area-uniform samples (seed+1) and keep the visible ones.
- Build a KD-tree on **all** aligned scan points and query with upper bound = coverage radius (10 mm). A sample is "covered" if a scan point lies within the radius.
- `covered_fraction` = mean(covered).
- `reference_area_cm2` and `covered_area_cm2` = area × covered_fraction.

**Threshold metrics** (`:124-134`), for each τ:

- $P_\tau = \frac1N\#\{d_i\le\tau\}$ (precision)
- $C_\tau = $ fraction of *covered* samples with nearest scan distance ≤ τ (completeness)
- `completeness_full@τ`: the same over all visible samples, with uncovered ones counted as misses
- $F_\tau = 2P_\tau C_\tau/(P_\tau+C_\tau)$

These are the precision/recall/F-score definitions popularised by the Tanks and Temples benchmark (Knapitsch et al. 2017). That attribution is **[INFERRED]**; the code does not cite it. If the largest τ ≥ coverage radius, a warning is stored because completeness is then 100% by construction (`:132-134`).

**Symmetric summaries** (`:137-143`), only when some reference is covered. Here $d^{ref}_j$ are the covered-sample distances:

- chamfer $=\tfrac12(\overline{d}+\overline{d^{ref}})$
- chamfer_rms $=\sqrt{\tfrac12(\overline{d^2}+\overline{(d^{ref})^2})}$
- hausdorff $=\max(\max d,\max d^{ref})$
- hausdorff95 = max of the two 95th percentiles

Also reported: `completeness_mean_mm` and `completeness_p95_mm`.

**Density** (`:146-150`): median nearest-neighbour spacing of scored points, and points per cm² of covered area.

The function returns `(metrics, per_point)` with `region`, `signed` and `z_error` arrays (`:152-153`).

### 4.7 Export (`scan_eval/export.py`)

- **Parametric / height field** (`parametric_mesh`, `:34-56`):
  - The grid size is chosen so that the spacing on the surface ≈ `resolution`, using the max $\lVert S_u\rVert$, $\lVert S_v\rVert$ over a 200² grid. The cap is 40 M vertices.
  - Two triangles per cell (`_grid_triangles`, `:22-31`); triangles with any vertex outside the mask are dropped.
  - Each face's winding is flipped to agree with the analytic normal at its centroid.
- **Height-field solid** (`solid_height_field`, `:59-83`):
  - Unmasked domains only, with base below the surface minimum.
  - Top grid, a reversed-winding bottom grid at `base_z`, and side walls built from the boundary ring. The ring is stated to be counter-clockwise from above, so the walls face outward **[EXPLICIT: :80]**. A test checks closure and positive volume (§9).
- **Implicit** (`implicit_mesh`, `:86-106`):
  - Marching cubes (Lorensen & Cline 1987) via `skimage.measure.marching_cubes` on $F\cdot$flip, with a 300 M voxel cap.
  - Vertices are then snapped onto $F=0$ with 10 Newton-to-surface steps, and the winding is reversed so face normals follow $+\nabla F$. That rests on the comment's claim about skimage's orientation, which this audit has not verified.
- **Writing** (`export_reference`, `:109-144`): `.stl/.ply/.obj` in the definition's own units via Open3D (binary). Default resolution = bbox diagonal/500. `[export]` keys in the TOML supply defaults.

## 5. Why this approach

| Decision | Rationale | Label |
|---|---|---|
| Python rather than C++ (unlike Fuse) | References must be exact equations, parsed safely, differentiated symbolically and extensible by plugin: "sympy, lambdify and a Python plugin, none of which exist in C++/PCL". Crop and Viewer are already Python+Open3D | **[EXPLICIT: README.md:16-22]** |
| Port Fuse's registration rather than rewrite | Same stages, preprocessing, FPFH+RANSAC, point-to-plane ICP, the "score hypotheses, don't trust them" design and the 15 % margin | **[EXPLICIT: README.md:22-31; registration.py:3-15]** |
| Truncated LS score instead of Fuse's `rmse/overlap` | On the real Out4 egg-carton scan, `rmse/overlap` picked a pose a quarter period off (84 % vs 99.8 % reference covered), because RMSE over inliers drops when a bad pose has fewer inliers | **[EXPLICIT: README.md:274-278; registration.py:187-190]** |
| Normal-facing test in score | Open height fields fit equally well upside down | **[EXPLICIT: README.md:279-284]** |
| Multi-start with turns/slides | Egg-carton, spheres, planes and cylinders are periodic/symmetric/featureless, so look-alike poses exist; slides of ¼ and ½ extent equal half a period and one period on the 2-period egg-carton | **[EXPLICIT: README.md:226-242]** |
| Moments start | Works on featureless shapes where FPFH finds nothing | **[EXPLICIT: README.md:235-237; registration.py:444-449]** |
| Final refinement on original points against the exact surface | Removes sampling error and voxel-centroid bias | **[EXPLICIT: README.md:285-286; registration.py:270-272,493-495]** |
| Degeneracy report | Error along an unconstrained direction is invisible to every metric | **[EXPLICIT: README.md:264-270]** |
| No automatic rescaling | Scale error is one of the things being measured | **[EXPLICIT: README.md:215-221; scan_eval/pipeline.py:63-64]** |
| Safe AST parser instead of `sympify` | `sympify` uses `eval` | **[EXPLICIT: expression.py:3-8]** |
| Distances to triangles, not vertices | Result independent of tessellation beyond the mesh's own discretisation | **[EXPLICIT: mesh_reference.py:3-7]** |
| Completeness only over the covered region | "A scan of half the part is judged on the half it saw"; `completeness_full` is also given | **[EXPLICIT: metrics.py:11-15; README.md:317-318]** |
| Own PLY writer | Scalar fields (`error_mm`, `scored`, `source`) usable in CloudCompare/MeshLab | **[EXPLICIT: report.py:82-85]** |
| Blue/orange overlay colours | Distinguishable for all colour-vision types and on both backgrounds | **[EXPLICIT: report.py:40-41]** |
| Unit-mismatch tolerance factor 60 | A scan legitimately contains the table or only part of the reference | **[EXPLICIT: scan_eval/pipeline.py:76-78; README.md:223-224]** |

**Alternatives not taken [INFERRED]:**
- **Signed-distance-field or voxel comparison.** It would avoid closest-point solves but adds discretisation error. The exact Newton projection keeps reference error at zero for analytic surfaces, at higher CPU cost.
- **Global registration via Go-ICP or TEASER++.** These would give certifiable global optima, but the scored multi-start plus explicit ambiguity flagging is simpler and makes the failure visible.
- **Fixed-pose (marker-based) registration.** Using the AprilTag frame would remove the alignment from the error budget entirely. It is not used here, so alignment can absorb systematic errors such as a tilt bias. This is a methodological point for the thesis: Evaluate's numbers describe *shape* error after the best rigid fit, not absolute positional error.

## 6. Parameters and tuning (`scan_eval/params.py:17-111`)

| Name | Default | Location | Effect |
|---|---|---|---|
| `seed` | 0 | scan_eval/params.py:21 | Seeds reference sampling (seed, seed+1, seed+2) and RANSAC (seed+k) |
| `units` | `"m"` | :25 | Scan units |
| `scale` | 1.0 | :29 | Multiplies scans after unit conversion |
| `reference_units` | None | :32 | Override reference units (bare mesh → mm) |
| `allow_scale` | False | :35 | Adds scale column to exact refinement (RANSAC/ICP remain rigid) |
| `align` | `"full"` | :41 | full / refine / none |
| `init_transform` | None | :42 | Seed transform path |
| `align_voxel_mm` | 0 → diag/60 | :46 | Alignment downsampling; scales FPFH radii, ICP distances and τ (2.4 mm for a 100×100×30 part per comment) |
| `eval_distance_mm` | 0 → 3×voxel | :51 | τ for scoring, refinement corr. distance and same-pose merge tolerance |
| `ransac_starts` | 5 | :54 | Number of FPFH-RANSAC starts |
| `ransac_max_iterations` | 100000 | :55 | RANSAC iterations (not on CLI) |
| `ransac_confidence` | 0.999 | :56 | RANSAC confidence (not on CLI) |
| `perturb` | True | :59 | Turn/flip/slide multi-start |
| `icp_max_iterations` | 60 | :60 | Per ICP stage |
| `ambiguous_margin` | 0.15 | :63 | Ambiguity flag threshold |
| `degeneracy_weak` / `_unconstrained` | 1e-2 / 1e-4 | :66-67 | Eigenvalue-ratio thresholds |
| `sensor_origin` | `"0,0,0"` (mm) | :73 | Normal orientation; `none` disables facing test |
| `sor_neighbours` / `sor_std` | 30 / 1.0 | :76-77 | SOR on alignment copy (0 disables) |
| `thresholds_mm` | [0.5, 1, 2, 5] | :81 | P/C/F thresholds |
| `outlier_distance_mm` | 5.0 | :83 | Outlier fraction |
| `coverage_radius_mm` | 10.0 | :88 | Covered definition; must exceed max threshold |
| `region_margin_mm` | 0.0 | :92 | Scored region grow/inset |
| `trim_percent` | [1, 5] | :94 | Robust stats |
| `reference_samples` | 200000 | :96 | Completeness samples (alignment uses half, min 20 000) |
| `visible_from` | None | :99 | Visible side override |
| `output`, `label`, `write_ply`, `write_plots`, `write_overlay`, `overlay_samples` | `evaluation_out`, None, True, True, True, 150000 | :102-111 | Output control |

**Hard-coded constants:**

| Constant | Value | Location |
|---|---|---|
| Analytic seed/area grid `_GRID` | 600 | analytic.py:39 |
| Domain check grid | 257² | :111 |
| Newton iterations | 60, 12 backtracks | :167, :195 |
| Implicit pool | 3×400 000 random points, thinning voxel diag/500, RNG 12345 | :396-401 |
| Foot-point iterations | 100 | :426 |
| Scorer subsample | 20 000 | registration.py:196 |
| ICP schedule | 6/3/1.5×voxel | :165 |
| FPFH radius | 10·voxel; normals radius 4·voxel | :144, :124 |
| Exact refinement | 50 iterations, 2 µm stop, 1.5× RMS rollback, 3·MAD | :269, :331, :298, :290 |
| Final-refine point budget | 100 000 | :496 |
| Visibility cosine | 0.05 | reference.py:220 |
| Units-check window | 1/60–60 | scan_eval/pipeline.py:79 |
| Error colour range | 98th percentile of \|error\| | report.py:70-75 |

**Example config** (`eval_config.example.toml`). It sets `reference_def = "surfaces/egg_carton_3cm_mesh.toml"`, metres, `full` alignment, and the default metric settings. Its comment suggests `region_margin_mm = -2` to ignore the rim where the scan sees the side walls (`:34-36`).

## 7. Assumptions, limitations, failure modes, edge cases

- **The best rigid fit is the reference frame.** All metrics are after least-squares-like alignment, so a global systematic distortion is partly absorbed by the pose (e.g. a small tilt bias or rigidly fittable warping). Error along DOFs reported unconstrained is invisible (`README.md:269-270`).
- **The sensor is at the scan origin.** True for RealSense and Fuse output **[EXPLICIT: scan_eval/params.py:72-73]**. For cropped or pre-transformed clouds whose frame is no longer the camera frame, the facing test is wrong unless `--sensor-origin` is set.
- **The part dominates the scan.** The moments start uses the whole downsampled scan, so clutter such as a table pulls it off **[EXPLICIT: registration.py:447]**. Parameter_Sweep had to add its own seed pose for this reason (see report 10).
- **Periodic or symmetric parts.**
  - Genuinely ambiguous poses are flagged but still produce numbers. The pyramid is always flagged ambiguous because of its 4-fold symmetry; that is benign (Parameter_Sweep README:97-99).
  - The ambiguity margin is computed on candidate scores *before* exact refinement.
- **Masked analytic domains.** Rim points use the nearest seed, so the error there is up to about half the seed spacing (≈0.1 mm for 100 mm parts).
- **Box-clamped projection.** Points beyond the domain edge project onto the boundary curve. The offset is then nearly tangent, so the sign is ill-defined. The tests acknowledge this (`test_mesh_vs_analytic.py:30-32`).
- **Implicit references.**
  - The closest point is not restricted to the domain box. A point near the cut can project onto the part of the surface outside the box (e.g. the lower half of the torus).
  - Sampling is only approximately area-uniform.
  - The seed pool uses a fixed RNG (12345), independent of `--seed`.
- **Mesh references.**
  - Signed error is only meaningful for orientable meshes.
  - For open meshes, "outward" is an area-weighted heuristic.
  - Non-manifold faces are deleted.
  - float32 queries are accurate to about 1 µm.
- **Units check blind spots.** Ratios between 1/60 and 60 are never flagged, so a mm-vs-cm (10×) or inch-vs-mm (25.4×) mix-up passes silently, even though those factors are in the guess list (`scan_eval/pipeline.py:79-83`).
- **Completeness depends on density.** It is measured from reference samples to the nearest scan point, so it is limited by point spacing as well as accuracy **[EXPLICIT: README.md:328-330]**.
- **Empty or tiny scans.**
  - Fewer than 100 points: `ScanError`.
  - Fewer than 10 after voxel: `ValueError`.
  - No points over the part after alignment: `ValueError` from `compute_metrics`.

  All of these are caught per scan by the CLI.
- **Determinism.** Every random choice is seeded, but Open3D's parallel ICP can change the last bits (< 1e-9 m) **[EXPLICIT: README.md:289-293]**.
- **Python version.** `scan_eval/pipeline.py:68,88` nest same-type quotes inside f-strings (`f"...{getattr(ref, "units_label", ref.units)}..."`). That syntax is legal only from Python 3.12 (PEP 701). Requirements say "Tested with Python 3.13" but nothing enforces it.

## 8. Dependencies

**Internal:**
- `scan_eval` is self-contained.
- It is *consumed* by `Processing/Parameter_Sweep/sweep/batch.py` and `Processing/Filter_Sweep/filter_sweep/evaluate.py`: `EvalParams`, `UnitScaled`, `load_reference`, `evaluate_scan`.
- `Processing/Rugosity_Metrics` (untracked) uses `log`, `load_config`, `SurfaceError`, `load_scan`, `unique_run_names`, `append_results(..., lead_columns=...)`, `write_json`, plot palette, `UnitScaled`, `load_reference`, and imports `UNIT_CHOICES`, `find_scans` from `evaluate_scans.py`.
- Reference data: `surfaces/egg_carton_*_mesh.toml` point to `Processing/Fused_Scans/*.ply`, and the sweep uses `Evaluate/References/*.stl`. **Both directories are git-ignored** (root `.gitignore:3,10`).

**Third-party (`requirements.txt`):**

| Library | Calls used and what they do |
|---|---|
| **open3d ≥ 0.19** | `io.read_point_cloud` / `read_triangle_mesh` / `write_triangle_mesh` (file I/O); `PointCloud.voxel_down_sample` (centroid per voxel); `remove_statistical_outlier` (drops points whose mean k-NN distance exceeds mean + k·σ); `estimate_normals` (PCA normal per neighbourhood); `orient_normals_towards_camera_location`; `pipelines.registration.compute_fpfh_feature`, `registration_ransac_based_on_feature_matching`, `registration_icp` with `TransformationEstimationPointToPlane`; `t.geometry.RaycastingScene` (`add_triangles`, `compute_closest_points`, `cast_rays`, BVH-accelerated exact queries); `TriangleMesh` cleaning (`remove_duplicated_*`, `remove_degenerate_triangles`, `remove_non_manifold_edges`, `is_edge_manifold`, `is_orientable`, `orient_triangles`); `utility.random.seed` |
| **numpy ≥ 1.26** | All array math, `linalg.lstsq` / `eigh` / `solve`, percentiles |
| **scipy ≥ 1.11** | `spatial.cKDTree` for nearest neighbours (seed lookup, scorer, completeness, spacing); `spatial.transform.Rotation` in tests |
| **sympy ≥ 1.12** | Symbolic expressions, `diff`, `lambdify` (to NumPy), `srepr` (hash canonical form) |
| **matplotlib ≥ 3.8** | Agg backend plots |
| **scikit-image ≥ 0.22** | `measure.marching_cubes` for implicit export and implicit surface area |
| **pytest ≥ 8** | Tests |
| stdlib | `ast`, `tomllib` (3.11+), `hashlib`, `importlib.util`, `csv`, `json` |

## 9. Tests: what they verify (`Processing/Evaluate/tests/`, not run)

**`conftest.py`** provides the shared fixtures:
- `fast_params`: 60 000 reference samples, no plots or PLY.
- `synthetic_scan` (`:29-61`): samples the visible reference, adds isotropic Gaussian noise σ (default 0.3 mm), optionally shifts or selects a sub-region or scales, then expresses the points in a camera frame 0.6 m above the part with random tilt (±25°) and spin. It returns the scan and the true scan→reference $T$.
- `pose_error`: max point displacement and rotation angle.
- Session fixtures `egg` (analytic egg-carton in metres) and `egg_mesh` (the egg-carton exported at 0.5 mm, loaded as a mesh height field).

**`test_alignment.py`:**

| Test | Verifies |
|---|---|
| `test_egg_carton_recovers_transform_and_noise[analytic, mesh]` (`:40`) | Pose error < 0.2 mm at the bbox corners and < 0.1°; RMSE within 8 % of σ; \|bias\| < 0.05σ + 0.01 mm; covered > 97 %; zero outliers; not ambiguous |
| `test_one_period_shift_is_undone` (`:48`) | Scan shifted 50 mm (one period) and seeded at the look-alike pose; the multi-start must find the true pose (< 0.3 mm), with overlap_ref > 0.95, not ambiguous, RMSE ≈ σ |
| `test_partial_periodic_coverage_is_flagged_ambiguous` (`:67`) | A one-period strip must be flagged ambiguous; both contenders have rmse < 1.5σ and overlap > 0.95, and they place the strip > 20 mm apart |
| `test_plugin_reference` (`:82`) | Sphere-cap plugin: RMSE ≈ σ; "rotation about Z" reported unconstrained or weak |
| `test_parametric_and_implicit_references` (`:92`) | Cylinder section and half torus: RMSE within 10 % of σ |
| `test_degeneracy_plane_and_paraboloid` (`:100`) | Plane (refine mode): rot Z, trans X, trans Y unconstrained, trans Z constrained; paraboloid: exactly `["rotation about Z"]` |
| `test_allow_scale_recovers_scale` (`:118`) | 2 % scaled scan gives `scale ≈ 1/1.02` ± 5e-4; rigid alignment gives scale 1 ± 1e-9 |
| `test_deterministic` (`:126`) | Two runs agree to 1e-9 m |

**`test_mesh_vs_analytic.py`:**
- Probe points are placed within a few mm of the analytic egg-carton (normal offsets with σ = 2 mm).
- The mesh-vs-exact distance error must fall 3–5× per halving of $h$ (2 → 1 → 0.5 mm), as expected for $O(h^2)$ chord error.
- That error must stay below $1.5\,\kappa h^2/8$ with $\kappa_{max}=A(2\pi/L)^2$.
- RMSE, MAE, bias, std, Z-RMSE, completeness@1mm and F@1mm on a noisy aligned scan must agree between mesh(0.5 mm) and analytic to 0.003 mm.
- The README reports measured max errors of 51, 12.8 and 3.2 µm (`README.md:400-405`); this audit has not reproduced them.

**`test_errors_and_cli.py`:**
- Unsafe or invalid expressions are rejected with specific messages. Covered cases: `__import__`, attribute access, lambda, strings, `^`, lists, `open`, unknown functions and names, `9**9**9`, wrong arity, syntax errors, comparison outside a mask (`:19-36`).
- A valid expression keeps its parameters symbolic.
- An undefined domain fails, and adding a mask fixes it.
- Definition errors: missing file, unknown key, missing units, YAML, non-numeric parameter (`A="import os"`), point cloud given as a mesh.
- Non-watertight mesh warning.
- Scan errors: missing file, empty PLY.
- Loud units mismatch for mm read as m (1000×).
- Watertight solid export, with signed volume ≈ 100·100·25 mm³ ± 0.2 %.
- Hash stable under whitespace and changed by a parameter edit.
- CLI end to end (`:130-166`):
  - two synthetic scans produce all 7 per-run files;
  - the overlay contains both colours;
  - RMSE ≈ 0.2 mm;
  - an `--align none --init-transform` re-run reproduces RMSE to 1e-4 mm and appends a third CSV row;
  - a missing scan returns exit code 1.

**Coverage gaps [INFERRED]:**
- No test covers the `refine` mode with clutter, the `--config` path, `--region-margin` behaviour, implicit `surface_area` without scikit-image, or the units check for 10× and 25.4× mismatches.
- Validation is on synthetic data with isotropic Gaussian noise only. Real RealSense noise is anisotropic (mostly along the viewing ray and growing with $z^2$), so recovery on real scans is not covered by the tests.

## 10. The uncommitted `report.py` change

`git diff Processing/Evaluate/scan_eval/report.py` (working tree vs HEAD `1e31b72`):

```diff
-def append_results(csv_path, row: dict):
+def append_results(csv_path, row: dict, lead_columns=None):
     """...table stays rectangular. lead_columns (default LEAD_COLUMNS) come first."""
+    lead_columns = LEAD_COLUMNS if lead_columns is None else lead_columns
 ...
-        ordered = [c for c in LEAD_COLUMNS if c in row or c in columns]
+        ordered = [c for c in lead_columns if c in row or c in columns]
```

- **Effect.** Callers can choose which columns lead a CSV. The default keeps the previous behaviour exactly, so `evaluate_scans.py` and `pipeline.py` are unaffected.
- **Who uses it.** Only the untracked `Processing/Rugosity_Metrics`: `roughness_scans.py:273` and `roughness/pipeline.py:479` pass `lead_columns=LEAD_COLUMNS` (their own list). The change exists to support that new component, which itself depends on the uncommitted edit **[INFERRED from grep]**.
- **Risk.** Committing Rugosity_Metrics without this file, or the reverse, would break it with a `TypeError`.

How `append_results` works (`report.py:230-257`):
1. Rows with `None` become `""`.
2. If the CSV exists, it is read fully.
3. If the new row has columns the file lacks (or the file is empty), the whole file is rewritten to `<name>.csv.tmp` with the union of columns (lead columns first, the rest sorted) and atomically `os.replace`d.
4. Otherwise the row is appended.

Not concurrency-safe: two processes appending at once can lose rows **[INFERRED]**.

## 11. Observations (bugs, bad practice, inconsistencies; not fixed)

1. **Implicit surface area crashes without scikit-image.**
   - `ImplicitReference.surface_area` (`analytic.py:453-458`) only guards `ImportError` on `from .export import implicit_mesh`. That import always succeeds.
   - `implicit_mesh` itself converts the missing-skimage `ImportError` into a `SurfaceError` (`export.py:87-91`), which is not caught.
   - `evaluate_scans.py:268` calls `surface_area()` before any scan, so evaluating against an implicit reference fails without scikit-image.
   - This contradicts `Processing/Evaluate/requirements.txt:8`: "Only needed to export *implicit* references".
2. **Completeness and coverage count off-part points.** The KD-tree for reference→scan distances is built from *all* aligned points, including those outside `region_mask` (`metrics.py:111`). Table or wall points next to the part's rim can therefore "cover" reference samples, while accuracy uses only scored points. The two directions use different point sets.
3. **`hausdorff_mm` is not the Hausdorff distance.** The reference-side maximum is taken over covered samples only, and those are ≤ coverage radius (10 mm) by construction (`metrics.py:121,142-143`). Uncovered reference never contributes.
4. **Units check cannot detect 10× or 25.4× mix-ups.** Only ratios outside (1/60, 60) are flagged (`scan_eval/pipeline.py:79`), yet the guess list includes "mm vs cm (10x)" and "in vs mm (25.4x)". Those can only be named for implausible ratios such as 254.
5. **Ambiguity is decided before refinement.**
   - The margin and `ambiguous` flag come from the ICP-refined candidate scores. The final `refine_exact` result is never re-scored (`registration.py:483-499`).
   - With `--align none` there is no runner-up, so `ambiguous=False` and `degeneracy={"available": False}`.
6. **Negative `region_margin` with an analytic mask does not inset the masked outline.** `HeightFieldReference.region_mask` (`analytic.py:305-314`) insets only the rectangle. `MeshReference` insets its outline (`mesh_reference.py:129-135`). The README's description, "insets the outline", holds only for meshes and unmasked analytic domains.
7. **`ImplicitReference.closest_points` has no domain constraint** (`analytic.py:417-436`). Its sampling and region are box-limited, so the distance and sample sets are inconsistent near the box faces.
8. **`refine_exact` rollback is lenient** (`registration.py:298`). A step that raises the RMS by up to 50 % is accepted. After a rollback, `J` (used for degeneracy) belongs to the reverted iteration's linearisation point, not the returned pose, though the difference is small.
9. **Dead code.** `rotation_angle_deg` (`registration.py:65-68`) is never used.
10. **Plugin execution is unrestricted.** `type = "plugin"` imports and runs arbitrary Python from the path in the TOML (`surface_def.py:154-159`). The README presents the expression path as "nothing … is ever passed to eval"; the plugin path is a deliberate escape hatch but defeats that guarantee for any definition directory that isn't trusted.
11. **Config paths resolve against the CWD.** `reference_def`/`reference`/`scans` from `--config` are relative to the working directory, not the config file (`evaluate_scans.py:203,244`). Mesh/plugin paths *inside* a definition TOML are resolved relative to that TOML (`surface_def.py:121,146`). The behaviour is inconsistent **[INFERRED]**.
12. **Reference data is outside version control.**
    - `surfaces/egg_carton_3cm_mesh.toml` and `_4cm_mesh.toml` reference `Processing/Fused_Scans/*.ply`.
    - The sweep references `Evaluate/References/*.stl`.
    - Both directories are git-ignored, so results are not reproducible from the repository alone.
    - README.md:68 shows the 3cm mesh path; the files in `References/` have different, space-containing names (`Egg carton-smaller3cm.ply`).
13. **`results.csv` grows forever.** It is never de-duplicated: re-runs append duplicate rows. Parameter_Sweep's `results/eval_runs/results.csv` has 180 data rows for 90 scans.
14. **Non-standard JSON.** `write_json(..., allow_nan=True)` writes bare `NaN` tokens, which strict JSON parsers reject (`report.py:227`).
15. **Python ≥ 3.12 required.** Nested same-quote f-strings at `scan_eval/pipeline.py:68,88` need it, and this is not documented in requirements.
16. **The scan is read twice.** `load_scan` reads the PLY once as a point cloud and once as a mesh only to count faces (`scan_eval/pipeline.py:33-35`). That doubles I/O on large clouds.
17. **`--allow-scale` affects only the final polish.** RANSAC and ICP stay rigid, so a large scale error (say > 5 %) may prevent the coarse stages from converging **[INFERRED]**. The reported `recovered_scale` is scan→reference, i.e. the *inverse* of the scan's scale error (the test asserts `1/1.02`). Thesis text should state that.
18. **Implicit sampling quirks.**
    - It draws with replacement when $n$ exceeds the pool (`analytic.py:441`). Pool size isn't checked against the default 200 000 completeness samples, so duplicates can enter area-weighted completeness.
    - The pool RNG is fixed at 12345, not tied to `--seed`.
19. **Initial commit marked untested.** The creating commit message says "Untested" (`0f33464`). The test suite was committed in the same commit and no later commit records running it. Whether the tests currently pass cannot be determined without running them.
20. **Undocumented dual role.** `scan_eval` doubles as a library for three other components, but has no packaging (`pyproject`). Consumers do `sys.path.insert` of the Evaluate directory (e.g. `Parameter_Sweep/sweep/batch.py:27-28`), so module-level side effects apply to every importer: `o3d.utility.set_verbosity_level(Error)` at `scan_eval/pipeline.py:20` and `matplotlib.use("Agg")` at `report.py:27`.
