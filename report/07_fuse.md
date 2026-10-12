# 07 — Multi-scan fusion (`Processing/Fuse`, executable `fuse_scans`)

**Summary.** `fuse_scans` is a C++17 / PCL program that takes the per-angle point clouds of a capture session (rig rotations `r<deg>` × tilts `a<deg>`), registers them into one common frame and **reconciles** the overlaps into a single surface instead of concatenating them. It is organised as an explicit stage machine — *load → tilt calibration → rotation-direction inference → coarse alignment (batch) → review → fine registration (batch) → review → fuse* — where each stage operates on a `FusionSession` of per-scan `ScanState`s and can be re-run from an interactive PCL/VTK viewer (scan legend, per-scan exclude, manual nudging, rotation-group flip). Key ideas implemented: (1) initial poses from rig kinematics, with the **tilt measured from the IMU gravity vector** and the IMU→cloud basis and tilt-axis pivot found by search; (2) **joint search over rotation directions** because folder names do not encode the turn direction; (3) **AprilTag coarse alignment** (Umeyama, or a 1-DOF Procrustes about gravity when the IMU fixes the tilt); (4) competing coarse hypotheses (rig seed, FPFH+RANSAC, tags) scored by a scene-aware metric — a **height-field metric on near-planar scenes** because nearest-neighbour scores are blind to in-plane slide/spin; (5) point-to-plane ICP against an incrementally built **voxel map** carrying averaged normals; (6) **weighted, trimmed-mean reconciliation** in height-field or volumetric cells with confidence/incidence/range weights, corroboration filtering, off-surface outlier removal and MLS smoothing. Everything below is derived from reading source, README and git history; nothing was compiled or run.

## Files covered

| Path | Lines | Role |
|---|---|---|
| `Processing/Fuse/CMakeLists.txt` | 48 | Build: PCL ≥1.10 (common io filters features registration surface visualization), Threads, Release default |
| `Processing/Fuse/README.md` | 790 | User/design documentation (main rationale source) |
| `Processing/Fuse/src/main.cpp` | 604 | CLI parsing, option precedence, stage machine, output |
| `Processing/Fuse/src/fusion_pipeline.h` / `.cpp` | 397 / 587 | Types (`ScanInput`, `PipelineParams`), I/O, VoxelGrid, SOR, normals, FPFH+RANSAC, point-to-plane ICP, `VoxelMap`, alignment metrics, plane frame, height grid |
| `Processing/Fuse/src/fusion_session.h` / `.cpp` | 417 / 2091 | `FusionSession`: rotation groups, IMU tilt seeds, tilt/pivot calibration, rotation-direction search, tag poses, coarse/refine batches, contributions, reports |
| `Processing/Fuse/src/fusion_reconcile.h` / `.cpp` | 162 / 587 | Overlap reconciliation (weights, cells, trimmed mean, outlier removal, MLS, thickness report) |
| `Processing/Fuse/src/rig_geometry.h` / `.cpp` | 74 / 67 | `RigGeometry`, axis rotations, rotation about a pivot, minimal rotation between vectors, `rigGuess` |
| `Processing/Fuse/src/rig_config.example.txt` | 38 | Example rig config (key = value) |
| `Processing/Fuse/src/scan_discovery.h` / `.cpp` | 77 / 415 | Session folder walk, angle parsing, IMU gravity read, rig-config parser, scan ordering |
| `Processing/Fuse/src/scan_state.h` / `.cpp` | 133 / 63 | `ScanStatus`, `AlignmentQuality`, `ScanState` |
| `Processing/Fuse/src/tag_alignment.h` / `.cpp` | 108 / 523 | Tag files, intrinsics, 3D tag location, sidecar reader, `fitTags` (Umeyama), `fitTagsAboutAxis` |
| `Processing/Fuse/src/visualizer.h` / `.cpp` | 150 / 750 | PCLVisualizer front-end: ROI box, review loop, keys, nudging |
| `Processing/Fuse/src/legend_panel.h` / `.cpp` | 82 / 193 | Scrollable, clickable scan list overlay |
| `Processing/Fuse/src/scan_colors.h` | 47 | Golden-angle per-scan colours |

Note: the brief listed `Processing/Fuse/rig_config.example.txt`; it actually lives at `Processing/Fuse/src/rig_config.example.txt` (README line 155 tells users to `cp rig_config.example.txt`, which only works from `src/`).

## Contents

0. Overview: data flow and frames
1. Purpose
2. Inputs and outputs
3. Walkthrough (stage machine)
4. Module sections (each with methods, rationale, parameters, limitations)
   4.1 `rig_geometry` · 4.2 `scan_discovery` · 4.3 `fusion_pipeline` · 4.4 `scan_state` · 4.5 `tag_alignment` · 4.6 `fusion_session` (tilt calibration, rotation direction, tag poses, coarse, refine, contributions) · 4.7 `fusion_reconcile` · 4.8 `visualizer`, `legend_panel`, `scan_colors` · 4.9 `main` · 4.10 build
5. Why this approach (consolidated)
6. Parameters and tuning (consolidated table)
7. Assumptions, limitations, failure modes
8. Dependencies
9. Design history (git)
10. Observations

---

## 0. Overview

```
session/ ─ discover (rotation groups, tilt, IMU gravity, rig seeds)
         ─ load: PLY → [tags located / sidecar read] → [ROI box] → VoxelGrid → SOR → normals (viewpoint 0)
         ─ tilt calibration: IMU→cloud basis (4 candidates) × pivot grid search   (or legacy ICP + (I−R)p=t)
         ─ rotation direction: [tags measure group angles] → joint 2^n sign search on representatives
         ─ coarse batch: per scan vs. running VoxelMap: seed | FPFH+RANSAC | tags  → scored, best kept
         ─ review (viewer)  ─ refine batch: point-to-plane ICP vs. running VoxelMap  ─ review
         ─ fuse: contributions (cloud in common frame, rotated normals, sensor origin, confidence)
                 → weights → height-field / volumetric cells → trimmed weighted mean → off-surface outliers → MLS
         ─ PLY (PointXYZRGBNormal, binary)
```

**Frames.** Every PLY is in its camera's librealsense *export* frame $(x,-y,-z)$ of the optical frame (`tag_alignment.h:23-24`). The **common frame** is the reference scan's frame — the first 0° tilt scan after discovery reordering — with transform = its own initial guess (identity in the usual case) (`fusion_session.cpp:440-447`). `ScanState::transform` maps a scan's own (downsampled) cloud into the common frame. The IMU accelerometer is in the depth sensor frame; the basis change to the cloud frame (`imu_to_cloud`) is *detected* (§4.6.2). Units: metres throughout; reports print mm.

## 1. Purpose

Single RealSense views of a surface suffer from occlusion, grazing-angle noise and holes. The rig captures the same scene from several tilts and rotations; this tool registers them and merges them so *"surfaces hidden from one angle get filled in by another"* (`README.md:3-6`) **[EXPLICIT]**. Commit `6de8e04` (2026-09-10) moved the tool from `Capture/More_Tools/Fuse` to `Processing/Fuse` and *"overhaulded fusion code to have an interactive list of scans. allows you to remove problematic scans, and can nudge alignments if needed. Produces a single fused point cloud with (atm low density) and no overlapping coagulated clouds"* **[EXPLICIT: commit 6de8e04]**. It consumes the crop tool's output (06) and produces the fused cloud evaluated downstream (Evaluate, Rugosity). It is called headless by the bulk pipeline (08).

## 2. Inputs and outputs

### 2.1 Input modes (`main.cpp:4-7, 403-421`)

| Mode | Invocation | Metadata |
|---|---|---|
| Sessions (recommended) | `fuse_scans -o out.ply --sessions <dir> [<dir>…]` | rotation/tilt parsed from folder names, IMU gravity, rig seeds, tags |
| Flat list | `fuse_scans -o out.ply a.ply b.ply …` | none: FPFH + ICP only |
| Manifest | `fuse_scans -o out.ply --manifest m.txt` | per-line `path [tx ty tz qx qy qz qw]` initial guess (`parseManifest`, 150-175) |

### 2.2 Per-scan files (session mode)

| File | Required | Reader | Content |
|---|---|---|---|
| `<tilt>/<tilt>.ply` or the single `.ply` in the tilt folder | yes | `findPlyInFolder` (`scan_discovery.cpp:21-53`), `loadPLY` (`fusion_pipeline.cpp:53-65`) | `pcl::PointXYZRGB` (PCL's PLY reader; float32 expected — cf. 06 §2.3) |
| `imu_a<tilt>.csv` (or single `imu_*.csv`) | optional | `readGravityFromImuCsv` (`scan_discovery.cpp:58-102`) | second line, columns 3-5 = accel (m/s²); rejected if $\lVert a\rVert\notin[9.0,10.6]$ |
| `<tilt>_tags.csv` sidecar | optional (`--tag-align`) | `findTagSidecar` / `readTagSidecar` (`tag_alignment.cpp:194-259`) | 3D tag corners, PLY frame |
| `*_detections.csv` + `scan_info.txt` | optional (`--tag-align`, no sidecar) | `findTagFiles`, `readColorIntrinsics`, `locateTags` (`tag_alignment.cpp:161-191, 262-430`) | pixel corners; `stream.Color.intrinsics: fx= fy= ppx= ppy= model= coeffs=` |

### 2.3 CLI options (`main.cpp:63-137, 216-369`)

| Flag | Default | Effect |
|---|---|---|
| `-o <path>` | required | Output PLY |
| `--sessions <dirs…>` | — | Consumes following args until one starts with `-` (229-230) |
| `--manifest <path>` | — | Mutually exclusive with `--sessions` (382-386) |
| `--rig-config <path>` | none | Baseline rig geometry (`loadRigConfigFile`) |
| `--tilt-axis x|y|z`, `--tilt-sign ±1` | None, 1 | Commanded-angle tilt seed (overrides config) |
| `--rotation-axis x|y|z`, `--rotation-sign ±1`, `--rotation-step-deg n` | None, 1, 0 | Rotation seed / fallback step |
| `--reverse-rotation <session>` | — | Per-session rotation-sign flip (legacy; inference overrides) |
| `--tilt-pivot x,y,z` | 0 | Known tilt-axis offset; disables tilt calibration (436) |
| `--tilt-pivot-range <m>` | 0.12 | Pivot search half-range |
| `--no-imu-tilt`, `--no-tilt-calibration`, `--no-rotation-direction` | off | Disable the corresponding stage/feature |
| `--tag-align`, `--tag-min-shared n`, `--tag-max-rmse m` | off, 1, 0.010 | AprilTag hypothesis |
| `--scan-voxel <m>` | 0.003 | `voxel_leaf_size` (load-time thinning and map leaf) |
| `--fusion-mode auto|heightfield|volumetric` | auto | Reconciliation mode |
| `--fusion-cell <m>` / `--fusion-voxel <m>` | 0.003 / 0.002 | Height-field cell / volumetric voxel |
| `--fusion-min-scans n`, `--fusion-trim f`, `--mls-radius m`, `--no-mls`, `--no-fusion-weights`, `--no-fusion-sor` | 1, 0.25, 0.010, –, –, – | Reconciliation controls |
| `--no-gui`, `--no-roi`, `--list-only`, `-h` | — | Headless / no ROI prompt / discovery only / help |
| anything else | — | Treated as a PLY path (`ply_args`, 365-368); **ignored** in sessions/manifest mode |

Option precedence: built-in `DiscoveryOptions` defaults → rig-config file → CLI overrides (371-380) **[EXPLICIT: comment 201-205]**.

### 2.4 Output

One binary PLY of `pcl::PointXYZRGBNormal` (`savePLY`, `fusion_pipeline.cpp:584-587`) in the common (reference scan) frame: one point per occupied cell (height-field) or voxel (volumetric), with MLS-computed (or estimated) normals; plus console reports (per-scan tables, tilt/rotation reports, fusion report). Exit code 1 on fewer than 2 scans, no loadable scans, or any exception (`main.cpp:426-430, 507-509, 597-601`).

## 3. Walkthrough: the stage machine (`main.cpp:498-592`)

1. **Discovery** (`discoverScansFromSessions`, 406) and `printDiscoveredScans`.
2. `PipelineParams` assembled from flags (432-441); a warning if the fusion cell is finer than the scan voxel (446-453).
3. Optional `FusionVisualizer` (457-459); `FusionSession` constructed (461) → `buildRotationGroups`; `setRigGeometry` (always called, so `rig_known_` is always true, 464); `setUseImuTilt`.
4. **Load** → `loadAll` (507).
5. **CalibrateTilt** → `runTiltCalibrationStage`, then IMU-tilt and calibration reports (515-517).
6. **ResolveRotation** → `runRotationDirectionStage`, `printRotationReport` (523-524).
7. **CoarseBatch** → `runCoarseBatch` (530); **CoarseReview** → `printReport` + `viewer->review` (536-544). Actions: Continue → refine; RerunCoarse/Back → coarse again; Quit → fuse directly.
8. **RefineBatch** → `runRefineBatch` (550); **RefineReview** (556-564): Continue → fuse; RerunCoarse → coarse; RerunRefine → refine; Back → coarse review.
9. **Fuse**: `buildContributions` → `reconcileScans(..., session.scenePlane(), &report)` → `report.print()` → `savePLY` (568-586).
10. Viewer shows the final cloud coloured by z (594-595).

Headless runs (`--no-gui`) make `reviewAt` always return Continue (489-492), so the run is linear.

## 4. Modules

### 4.1 `rig_geometry` — rig kinematics primitives

**Purpose.** Turn folder angles into rotations and encode the rig calibration (`rig_geometry.h:8-13`).

**`RigGeometry`** (`rig_geometry.h:14-47`): `tilt_axis`, `tilt_sign`, `tilt_pivot`; `rotation_axis`, `rotation_sign`, `rotation_pivot`; `compose_tilt_first` (true: $T=R_{rot}R_{tilt}$, "tilt head on a turntable"; false: $T=R_{tilt}R_{rot}$, "tilt axis fixed in the world"). Axes are camera-frame axes X/Y/Z or None.

**Functions** (`rig_geometry.cpp`):
* `rotationAboutAxis(axis, deg)` (5-20): Eigen `AngleAxisf` about a unit axis.
* `rotationAboutPoint(axis, deg, p)` (22-29): $T=\begin{bmatrix}R & \mathbf p-R\mathbf p\\0&1\end{bmatrix}$ — rotation leaving $\mathbf p$ fixed.
* `rotationBetweenVectors(a,b)` (31-45): `Quaternionf::FromTwoVectors` — the *minimal (geodesic)* rotation taking $\hat a$ to $\hat b$, about $\hat a\times\hat b$ by $\arccos(\hat a\cdot\hat b)$. Header rationale: *"For two gravity readings this is exactly the tilt the hinge applied, because a horizontal hinge axis is perpendicular to gravity"* (`rig_geometry.h:62-67`) **[EXPLICIT]**.
* `rigGuess(rig, have_rot, rot_deg, have_tilt, tilt_deg, has_guess)` (47-67): commanded-angle seed $R_{rot}(\sigma_r\theta)\,R_{tilt}(\sigma_t a)$ or the reverse, each about its pivot.

**Kinematic model (README:194-228).** The README states the physical rig has the tilt stage at the base and the rotating mount on top of it, so camera-to-world is $R_{tilt}(a)\,R_{rot}(\theta)$ and the tilt axis appears to turn in the camera frame with each rotation group (table README:200-205 shows the measured axis rotating by 45° per group). Therefore *"A single `--tilt-axis` cannot describe this"* and *"the tilt is measured, not assumed"* from the IMU **[EXPLICIT: README 207-215]**. The README also reports that the mount does not reach the commanded angle: *"commanded 15° came out as 13.6°–17.4° … individual steps ranged 0.6×–1.24×"* **[EXPLICIT: README 212-213]**.

### 4.2 `scan_discovery` — folder walk and seeds

**Purpose.** Map the capture folder tree to `ScanInput`s with metadata and rig seeds.

**Walkthrough** (`discoverScansFromSessions`, `scan_discovery.cpp:195-402`):
1. For each session dir: name = basename (213-214), optional per-session rotation-sign override (216-229).
2. List rotation subfolders and **sort by parsed angle** (`_r(-?[0-9]+)(?:_|$)`), not by name, because *"lexicographically "_r135" sorts before "_r45""* **[EXPLICIT: 244-248]**.
3. If `rotation_axis` is set: rotation from the name, else `rotation_index × rotation_step_deg` fallback (270-293). **If no rotation axis is configured, `has_rotation` stays false and `rotation_degrees` stays 0.**
4. For each tilt subfolder (unsorted `directory_iterator`, 297): find the PLY; set `session_name`, `subfolder_name`, `rotation_group = session/rotfolder`, `rotation_index`, `rotation_degrees`; read IMU gravity using the tilt parsed from `_a(-?[0-9]+)$` (323-333); parse tilt (335-350) — `have_tilt` for the *seed* requires a configured tilt axis (343); seed = `rigGuess(...)` (355-361).
5. Reorder (381-399): swap the first 0°-tilt scan to the front (reference); stable-sort the rest by $|a|$. README rationale: *"this registers far more reliably than fusing in arbitrary directory order, since each new scan is closer in angle (and therefore overlap) to what's already merged"* **[EXPLICIT: README 122-125]**.

**Rig config parser** (`loadRigConfigFile`, 148-193): case-insensitive keys `tilt_axis, tilt_sign, tilt_pattern, tilt_pivot, rotation_pivot, rotation_axis, rotation_step_deg, rotation_sign, rotation_pattern, compose_tilt_first, reorder_by_tilt`; unknown keys warned. `rig_config.example.txt` sets `tilt_axis = x`, `tilt_sign = 1`, `rotation_axis = z`, `rotation_step_deg = 45`, `rotation_sign = 1`, `compose_tilt_first = true` (example lines 9-38).

**Limitations.** Tilt subfolders are visited in filesystem order, so the tie order between e.g. `a5` and `a-5` after the stable sort is filesystem-dependent. A missing IMU file or an implausible magnitude silently drops gravity for that scan (warning only for the magnitude).

### 4.3 `fusion_pipeline` — building blocks

**Types** (`fusion_pipeline.h`): `PointT = pcl::PointXYZRGB` (15), `PointNT = PointXYZRGBNormal`; `ScanInput` (25-72, includes `gravity`/`has_gravity`); `RegistrationResult` (74-88, with `hit_max_iterations` because *"PCL counts running out of iterations as converging"* **[EXPLICIT: 79-81]**); `PipelineParams` (90-236, see §6).

**Preprocessing** (`preprocessCloud`, `fusion_pipeline.cpp:215-229`): `pcl::VoxelGrid` (centroid of points per cubic voxel of edge `voxel_leaf_size`) then `pcl::StatisticalOutlierRemoval` (for each point, mean distance $\bar d_i$ to its $k=30$ neighbours; remove if $\bar d_i > \mu_{\bar d} + 1.0\,\sigma_{\bar d}$). Order rationale: *"Voxel downsample first - cheaper, and gives SOR a saner neighbourhood density."* **[EXPLICIT: 218]**.

**Normals** (`estimateSensorNormals`, 245-260): `NormalEstimationOMP` (PCA of the radius-0.015 m neighbourhood covariance; normal = smallest-eigenvalue eigenvector) with viewpoint (0,0,0) so all normals face the sensor. Rationale: *"the map averages normals from several scans, and oppositely-signed estimates of one surface would cancel"* **[EXPLICIT: 252-256]**.

**FPFH + RANSAC coarse alignment** (`coarseAlignFPFH`, 278-310). Fast Point Feature Histograms (Rusu, Blodow & Beetz, ICRA 2009) at radius 0.03 m on normals at 0.015 m; `pcl::SampleConsensusPrerejective` — the prerejective RANSAC of Buch et al. (ICRA 2013): sample 3 source points, pick each one's correspondence randomly among the 5 most similar target features, reject the sample early if the polygon edge-length similarity is below 0.9, estimate the rigid transform, count inliers within 0.05 m, accept if ≥25 % inliers; 50 000 iterations. Fitness = $1-\frac{|\text{inliers}|}{|\text{source}|}$.

**Point-to-plane ICP** (`refineICP`, 313-351): `pcl::IterativeClosestPointWithNormals<PointNT,PointNT>`, which by default minimises the point-to-plane error (Chen & Medioni 1991/1992; ICP framework of Besl & McKay 1992) using a linearised least-squares step (PCL's `TransformationEstimationPointToPlaneLLS`, the small-angle linearisation of Low 2004 — library default, not overridden here):

$$E(R,\mathbf t)=\sum_{(i,j)\in\mathcal C}\big((R\mathbf p_i+\mathbf t-\mathbf q_j)\cdot\mathbf n_j\big)^2,\quad \lVert R\mathbf p_i+\mathbf t-\mathbf q_j\rVert<d_{max}.$$

Settings: $d_{max}=0.02$ m, 60 iterations, transformation ε $10^{-8}$, Euclidean fitness ε $10^{-6}$. Both clouds carry pre-computed normals; *"Only the target's normals are read: PCL's non-symmetric point-to-plane objective ignores the source's"*, and re-estimating normals per call was *"85% of the whole refine stage"* on a 28-scan session **[EXPLICIT: 321-326]**. Point-to-plane chosen because it *"converges faster and more accurately than point-to-point on the roughly-continuous surfaces this rig scans"* **[EXPLICIT: 318-319]**. Fitness reported with `getFitnessScore(d_max²)` so that parts outside the map's footprint do not dominate **[EXPLICIT: 345-347]**.

**`VoxelMap`** (`fusion_pipeline.h:280-313`, `.cpp:115-212`): the registration target. Each voxel (21-bit-per-axis packed key, 102-112) keeps running sums of position, colour, and *rotated* normals with counts; output = true centroid, mean colour, normalised mean normal (NaN if none, so ICP skips it). Rationale: incremental cost and **equal weight per scan** — re-voxelising a map of centroids *"hands the newest scan about half the weight wherever it overlaps - so the map chased the latest scan instead of holding the consensus"* **[EXPLICIT: fusion_pipeline.h:269-279]**.

**Alignment metrics.**
* `evaluateAlignment` (355-406): stride-sampled (≤20 000) source points after transform; nearest target neighbour (kd-tree); inlier if distance ≤ `eval_correspondence_distance` (0.01 m). Returns $\mathrm{rmse}=\sqrt{\frac1{n_{in}}\sum d^2}$ and inlier fraction $f=n_{in}/n_{sampled}$. Stride rather than prefix because *"PLY point order follows the depth image raster"* **[EXPLICIT: 370-372]**.
* `AlignmentQuality::score()` = $\mathrm{rmse}/f$ (lower better; ∞ if invalid) (`scan_state.h:56-61`) — rewards coverage over a tight fit on a sliver **[EXPLICIT: scan_state.h:53-55]**.
* `estimatePlaneFrame` (410-440): PCA of the whole cloud; normal = smallest-eigenvalue eigenvector, $u$ = largest; planarity $=\sqrt{\lambda_{min}/\lambda_{max}}$.
* `buildHeightGrid` / `evaluateAgainstHeightGrid` / `evaluateAlignmentOnPlane` (454-546): express points in the plane frame $(u,v,h)$, bin $(u,v)$ into `plane_grid_cell` (0.01 m) cells, mean height per cell (cells with < `min_points_per_cell` dropped); compare source vs target cell heights where both exist; inlier if $|\Delta h|\le 0.01$ m; rmse over inliers; fraction = inliers / overlapping cells. README rationale: on a flat sheet *"nearest-neighbour scoring is nearly blind to in-plane error"*; with height comparison *"a 5 mm in-plane slide raised the height-field score by 36%, the nearest-neighbour score by only 16%"* **[EXPLICIT: README 323-352]**.

**Unused in the current flow:** `postProcessMerged` (549-576), `voxelDownsample` is used, but `final_voxel_leaf_size`, `run_final_outlier_removal`, `estimate_output_normals`, `output_normal_radius` only feed `postProcessMerged`, which nothing calls.

### 4.4 `scan_state` — per-scan state

`ScanStatus` = Pending, Reference, CoarseAligned, CoarseFailed, Refined, RefineFailed, Excluded (`scan_state.h:15-24`); `contributesToFusion` = Reference | CoarseAligned | Refined (`scan_state.cpp:23-30`) — coarse-only scans are fused *"they are positioned well enough to be worth showing and fusing"* **[EXPLICIT: scan_state.cpp:25-26]**. `ScanState` (`scan_state.h:66-133`) holds `prepared` (downsampled, filtered cloud in camera frame; raw cloud deliberately dropped to save RAM **[EXPLICIT: 70-74]**), `normals`, ROI, `coarse_transform`, `transform`, `coarse_quality`, `fine_quality`, `coarse_source` ("seed"/"fpfh"/"tags"), `tags`, `tag_fit`, `icp_result`, `status`, `flagged`. `cloudWithNormals` (`scan_state.cpp:40-63`) builds the ICP source with NaN normals if missing.

### 4.5 `tag_alignment` — AprilTag 3D location and pose fits

**Purpose.** Use tags fixed in the world as exact correspondences that pin down *"the in-plane slide and spin that a flat sand surface leaves geometry-only methods nearly blind to"* **[EXPLICIT: tag_alignment.h:12-18]**.

**3D location** (`locateTags`, `tag_alignment.cpp:315-430`), the C++ twin of the crop tool's sidecar (06 §3.5):
1. `readDetections` (122-156): per-ID median of each corner coordinate over all **decoded** rows (positional CSV parsing of the 15-column format).
2. Inner quad at 0.7 scale; one pass over the raw cloud projecting each point (PLY→camera flip, librealsense distortion `distortNormalised`, 24-33) and collecting points inside each inner quad (bounding-box pre-check then `insideQuad`, 72-85).
3. ≥30 points → PCA plane (`fitPlane`, 96-112), inliers within $\max(3\text{ mm}, 2.5\,\mathrm{rms})$, refit.
4. Corners = ray–plane intersections $\mathbf X_k=\frac{\mathbf n\cdot\bar{\mathbf p}}{\mathbf n\cdot\mathbf r_k}\mathbf r_k$ with `pixelRay` using 30-step fixed-point undistortion (38-49), converted back to the PLY frame. Rationale: *"exact in the image, and averaged over the whole tag in depth, rather than trusting the single depth sample at a corner"* **[EXPLICIT: 400-403]**.
5. Report edge length and plane rms per tag.

Intrinsics: `readColorIntrinsics` (262-312) parses `stream.Color:` (resolution) and `stream.Color.intrinsics:` (fx, fy, ppx, ppy, model, coeffs) from `scan_info.txt` (format written by `rs_marker/main.cpp:723-731`); distortion is applied unless `model=None` or all coefficients are zero.

**Sidecar** (`readTagSidecar`, 221-259): rows `marker_id,corner,x,y,z,frames,plane_rms_m,edge_m`; tags missing any of the four corners are skipped. `findTagSidecar` prefers `<folder>_tags.csv` (194-218). `locateScanTags` (`fusion_session.cpp:585-627`) uses the sidecar first (*"for a cropped scan it is the only source"* **[EXPLICIT: 590-591]**), else detections + `scan_info.txt`.

**`fitTags` — Umeyama / Kabsch** (434-468): stacks the 4 corners of every shared tag ID and calls `Eigen::umeyama(src, dst, false)` (rigid, no scale; *"both sides are metric, so a scale term could only absorb error"* **[EXPLICIT: 454-455]**). Umeyama (1991): with centred point sets, $\Sigma = \frac1n\sum (\mathbf y_i-\bar{\mathbf y})(\mathbf x_i-\bar{\mathbf x})^\top = UDV^\top$, $R = U\,\mathrm{diag}(1,1,\det(UV^\top))\,V^\top$, $\mathbf t=\bar{\mathbf y}-R\bar{\mathbf x}$. Residual = corner RMS.

**`fitTagsAboutAxis` — 1-DOF Procrustes about gravity** (471-523). Given a fixed base rotation $B$ (the scan's IMU-derived tilt seed) and the vertical axis $\hat{\mathbf u}$, solve only the turn $\phi$ about $\hat{\mathbf u}$ and the translation. With $\mathbf a_i = $ centred, axis-projected $B\mathbf x_i$ and $\mathbf b_i$ = centred, axis-projected targets:

$$\phi=\operatorname{atan2}\Big(\sum_i \hat{\mathbf u}\cdot(\mathbf a_i\times\mathbf b_i),\;\sum_i \mathbf a_i\cdot\mathbf b_i\Big),\quad R = R_{\hat{\mathbf u}}(\phi)\,B,\quad \mathbf t = \bar{\mathbf y}-R_{\hat{\mathbf u}}(\phi)\,\overline{B\mathbf x}.$$

This is the closed-form 2D orthogonal Procrustes solution. Rationale: the tags a scan sees *"usually lie nearly in a line, so tilt about that line rests on each 80 mm tag's own plane, which depth noise sets only to a degree or two - while the IMU measures tilt well"* **[EXPLICIT: tag_alignment.h:100-106]**; README reports this took tag poses on the Small_Wave session from 1–2.5° to 0.2–0.6° off the final ICP poses **[EXPLICIT: README 452-459]**.

### 4.6 `fusion_session` — the pipeline proper (2091 lines)

#### 4.6.1 Construction, rotation groups, seeds

* `buildRotationGroups` (69-115): groups scans by `rotation_group`, sorts groups by session then rotation index, and sets each group's `nominal_step_deg = |θ_k − θ_{k−1}|` (a magnitude only, because *"They are always recorded positive, so this is a magnitude only - the sign is exactly what has to be inferred"* **[EXPLICIT: 104-106]**).
* `resolveGroupGravityReferences` (118-144): each group's lowest-|tilt| scan is its *within-group reference*; its gravity is `reference_gravity`.
* `imuRotationBetween` (947-960): $R_{tilt} = F\,R_{\min}(\mathbf g_{scan}\to\mathbf g_{ref})\,F^\top$ — minimal rotation in the IMU frame **conjugated** into the cloud frame by $F=$ `imu_to_cloud` (*"Conjugation, not multiplication: the same physical rotation expressed in a different basis"* **[EXPLICIT: 955-957]**).
* `tiltComponentFor` (167-207): IMU path (preferred) $T=\begin{bmatrix}R_{tilt}&\mathbf p-R_{tilt}\mathbf p\\0&1\end{bmatrix}$ with pivot $\mathbf p$ = calibrated pivot, else configured `tilt_pivot`; fallback commanded-angle rotation about the configured axis relative to the group reference's tilt.
* `applyGuessToScan` (210-239): seed $= R_{rot}(\theta_{cum})\cdot T_{tilt}$ ("Tilt first, into the group's reference frame; then the group's own rotation, into the common frame", 235-236). `rotation_sign` is not reapplied because the cumulative angle carries the sign (228-229). Note: `compose_tilt_first` is not consulted here (see Observations).
* `applyRotationAngles(k0)` (305-321): $\theta_k = \theta_{k-1} + s_k\,\Delta_k$ (or $s_k\Delta_k$ at a new session), unless the group's angle came from tags (absolute); then reseeds every scan of each group.

The physical meaning: a scan's pose in the common frame is "rotate within the group to undo the tilt relative to the group's flattest scan (measured by gravity), then turn the group by its cumulative turntable angle about `rotation_axis` through `rotation_pivot`". The rotation pivot is *"Not auto-estimated yet - the tilt sweep gives many small steps to fit against, whereas rotation gives only a handful of large ones"* **[EXPLICIT: rig_geometry.h:33-35]**.

#### 4.6.2 Load (`loadAll`, 365-449; `reloadScan`, 452-474)

Per scan: load PLY; **locate tags before the ROI crop** (*"the tags sit around the edge of the scene, which is exactly what an ROI tends to cut away"* **[EXPLICIT: 377-378]**); optional ROI from the viewer; `preprocessCloud`; `estimateSensorNormals`; raw cloud dropped. Load failures mark the scan Excluded instead of aborting (403-409). The reference = first loaded scan (413-421). Then the **scene plane** is fitted once from the reference (429-438) and decides between height-field and nearest-neighbour scoring for the whole run (`scoreAlignment`, 1510-1521).

#### 4.6.3 Tilt calibration (`runTiltCalibrationStage`, 1192-1443)

**IMU path** (1206-1229), taken when IMU tilt is enabled and >1 scan has gravity:
1. `selectTiltCalibrationScans` (912-944): scans in the reference's rotation group with tilt within 15° of the reference, largest tilt first, ≤6.
2. `prepareCalibrationCaches` (963-984): downsample reference and candidates to `max(0.005, voxel_leaf_size)`; build the reference height grid once with `min_points_per_cell = 1` (rationale *"these clouds are already voxel-averaged"* **[EXPLICIT: 972-975]**; caching because rebuilding per evaluation *"made the calibration searches hundreds of times more expensive"* **[EXPLICIT: fusion_pipeline.h:339-342]**).
3. `calibrateImuTiltGeometry` (1101-1189): for each of four basis candidates $F\in\{\mathrm{diag}(1,1,1),\mathrm{diag}(1,-1,-1),\mathrm{diag}(-1,1,-1),\mathrm{diag}(-1,-1,1)\}$ (identity, Rx180, Ry180, Rz180 — rotations only, since *"a mirrored frame would flip handedness"* **[EXPLICIT: 1104-1106]**) run a 2-level pivot search and record the score; keep the best; report ties within ±1 % (*"Conjugating a rotation by a flip about its own axis leaves it unchanged"* **[EXPLICIT: 1138-1140]**); refine the winner's pivot with 4 levels.
4. `searchTiltPivot` (1050-1098): hinge axis $\hat{\mathbf h}$ = hemisphere-aligned mean of the IMU rotation axes (`meanHingeAxis`, 1027-1047); search the plane spanned by $\mathbf u=\hat{\mathbf h}\times\mathbf s$, $\mathbf v=\hat{\mathbf h}\times\mathbf u$ on a 9×9 grid with step = range/4, recentring on the best and shrinking the step ×0.4 per level. Only two components are searched because *"sliding the pivot along the hinge changes nothing"* **[EXPLICIT: fusion_session.h:364-367]**.
5. Objective `calibrationResidual` (987-1024): mean over calibration scans of $\min(\text{score},10)$ for the height-grid score of the scan under $T(\mathbf p)$ vs the reference grid.
6. `applyRotationAngles(0)` re-seeds every scan.

README evidence for detecting $F$: measured residuals per frame (identity 42–76 mm vs **Rx180 7–18 mm**) and the explanation that the RealSense export negates y and z **[EXPLICIT: README 230-267]**; and for keeping the pivot range small: a half-metre range produced *"a spurious 0.7 m"* lever arm **[EXPLICIT: README 282-289]**. Frame and pivot are resolved *together* because *"with a long lever arm the pivot error dominates the residual"* **[EXPLICIT: README 291-294]**.

**Legacy commanded-angle path** (1230-1442), used without IMU readings and with a configured tilt axis: ICP-align up to 6 low-tilt scans (smallest first) against a growing VoxelMap; accept those that converge with ≥50 % overlap; record observed relative motion $(R_{obs}, \mathbf t_{obs})$ and model rotation $R_k$ about the configured axis; solve the pivot by least squares on $(I-R_k)\mathbf p=\mathbf t_k$ (`solvePivot`, 877-909: stacked $3N\times3$ system, `JacobiSVD` with threshold $10^{-4}$, minimum-norm solution because $(I-R)$ is singular along the axis **[EXPLICIT: 899-902]**). Cross-checks: observed axis vs configured (axis error), signed angle ratio observed/commanded; a negative ratio flips `tilt_sign` and re-runs once (1404-1415). The fit is **adopted only if** residual < 0.5 × seed error before, axis error < 10°, and $||\text{scale}|-1|<0.15$ (1421-1434) — *"adopting it anyway replaces a mildly wrong seed with a confidently wrong one"* **[EXPLICIT: 1417-1420]**.

#### 4.6.4 Rotation direction (`runRotationDirectionStage`, 1567-1727)

Problem: *"The capture rig names every rotation folder with a positive angle … regardless of which way you physically turned the mount"*; the IMU cannot help because rotation is about gravity and the gyro is sampled stationary **[EXPLICIT: README 356-370; fusion_pipeline.h:182-192]**.

Algorithm:
1. Skip unless inference is on, a rotation axis is configured, and ≥2 groups exist (1569-1585).
2. If `--tag-align`: `resolveRotationFromTags` (242-302) — solve tag poses, then for each group take the per-scan correction $\Delta=R_{tag}R_{seed}^\top$, its signed angle about the rotation axis, the **median** over the group's scans, and set $\theta_k=\operatorname{remainder}(\theta_k^{folder}+\Delta,360°)$; mark `from_tags` (excluded from the search). README: on Small_Wave the surface search put r90 at 0° while tags measured −90°, and found r135 was a turn back to −45° **[EXPLICIT: README 486-488]**.
3. Representatives = lowest-tilt scan per group (1596-1614).
4. **Joint search** (1644-1671) over all $2^{n}$ sign assignments of the free steps ($n\le12$): `scoreDirectionHypothesis` (1527-1564) places each representative at its cumulative angle (with its IMU tilt), scores it against a map of the previously placed representatives, caps each step at 10 and sums, adding the placed representative to the map. Beyond 12 steps a greedy per-step choice is used (1672-1687). Rationale: a wrong early step makes later ones fit worse, *"which only a joint search can see"* **[EXPLICIT: fusion_pipeline.h:204-211]**. Raw seeds are scored without ICP settling (`direction_test_icp_iterations = 0`, declared but never read) because on planar scenes *"letting a *wrong* direction settle lets it slide into a plausible-looking fit"* **[EXPLICIT: fusion_pipeline.h:195-202; README 380-384 "observed on real data"]**.
5. **Per-step margin** (1708-1721): $m_k = (S_{flip k}-S_{best})/S_{flip k}$; ambiguous if $m_k<0.15$.
6. `applyRotationAngles(1)`; `flipRotationGroup` (324-362) supports the viewer's `t` key (reverses one step, recomputes all later cumulative angles, resets affected scans to Pending).

#### 4.6.5 Tag poses (`solveTagPoses`, 491-582)

Greedy chain from the reference: maintain the set of placed tags (running mean of each tag's corners in the common frame, 498-511); each round choose the unplaced scan sharing the most tags (≥ `tag_min_shared`) with the placed set; fit with `fitTagsAboutAxis` if the reference and the scan have gravity and the scan has a seed (up vector $= R_{ref}\,F\,\hat{\mathbf g}_{ref}$, 526-530), else `fitTags`; reject if corner RMS > `tag_max_fit_rmse` (10 mm). Independence from geometric placements is deliberate: *"a scan a wrong seed has misplaced would otherwise drag every tag it sees with it"* **[EXPLICIT: fusion_session.h:339-343]**.

#### 4.6.6 Coarse alignment (`runCoarseBatch`, 1766-1804; `computeCoarseFor`, 630-692; `computeGeometricCoarseFor`, 695-789)

For each non-reference, loaded, non-excluded scan, in order, against the running `VoxelMap` of everything placed so far:

1. **Seed hypothesis**: coverage = `evaluateAlignment` inlier fraction; quality = `scoreAlignment`. If coverage ≥ `coarse_min_inlier_fraction` (0.15) the seed is accepted immediately (720-730).
2. Otherwise **FPFH+RANSAC** against the map; take whichever of seed/FPFH has the lower score (753); mark CoarseFailed if the winner's coverage < 0.15 or no hypothesis exists (770-788). Two different measures are used on purpose — coverage for "does it land at all", scene-aware score for "which is better" **[EXPLICIT: 700-706]**.
3. **Tag hypothesis** (with `--tag-align` and a valid tag fit): scored the same way; wins if usable and better (or if the geometric hypothesis failed) (643-649). Logs the angular/translational disagreement and warns loudly if > 10° (`tag_disagreement_warn_deg`) — *"That is not imprecision — one of them is wrong"* **[EXPLICIT: README 469-473]**.
4. `transform = coarse_transform`; add to the map if contributing (1793-1802).

#### 4.6.7 Fine registration (`runRefineBatch`, 1807-1834; `refineAgainst`, 1843-1873; `updateFlags`, 1876-1905)

Same sequential map building; for each scan not excluded/coarse-failed: point-to-plane ICP from `coarse_transform`; on convergence status Refined and `fine_quality = scoreAlignment(...)`; otherwise RefineFailed and the coarse pose is kept *"it keeps the scan visible for review rather than silently vanishing"* **[EXPLICIT: 1860-1862]**. Flags: RefineFailed, ICP hit its iteration cap, invalid quality, or score > 1.4 × batch median (median only with ≥3 scores) — relative because *"what a good score looks like depends on the voxel size, the sensor's noise … and the scene's relief"* **[EXPLICIT: 1878-1882]**. Single-scan redo (`coarseAlignScan`, `refineScan`, 1908-1945) aligns against **all** other contributing scans (`buildMap(index, size)`), not just earlier ones.

Because each scan registers against the accumulated map, *"drift stays lower as more views are added"* **[EXPLICIT: README 50-51]**. This is a sequential (not globally optimised) multi-view registration — no pose-graph / loop closure step exists.

#### 4.6.8 Contributions (`buildContributions`, 2006-2058)

For each contributing scan: cloud in the common frame, normals rotated by $R$, `sensor_origin` = transform translation (the camera centre, since PLYs are camera-centred), and **confidence**

$$c=\operatorname{clamp}\Big(f\cdot\frac{r_0}{\max(\mathrm{rmse},r_0)},\,0.02,\,1\Big),\quad r_0 = 0.25\times\texttt{eval\_correspondence\_distance}=2.5\text{ mm},$$

using fine quality if valid, else coarse; the reference gets 1.0. *"Overlap carries most of the signal"* **[EXPLICIT: 2036-2039]**.

Other members: `setExcluded` (1948-1963) restores the reached stage when un-excluding; `nudgeScan` (1966-1973) left-multiplies a common-frame delta and makes it the new ICP seed; `printReport` (2061-2091) prints the per-scan table; `printTiltFromImuReport` (793-863) compares commanded vs IMU-measured tilt per group and prints the measured tilt axis in the camera frame; `printTiltCalibrationReport` (1446-1507), `printRotationReport` (1730-1763).

### 4.7 `fusion_reconcile` — overlap reconciliation

**Problem.** *"Residual registration error (a few mm) is larger than the output voxel, so where scans overlap their surfaces land in different voxels and survive as two or three offset sheets"*; SOR cannot remove a duplicate sheet because it is locally dense **[EXPLICIT: fusion_reconcile.h:14-21; README 623-629]**.

**`reconcileScans`** (`fusion_reconcile.cpp:250-587`):
1. **Concatenate** contributions with per-point scan id and confidence (112-140, 259).
2. **Normals**: each scan's own (rotated) normals if all have them, else estimate on the merged cloud (279-284) — the latter took *"220 s, most of fusion"* on a 28-scan, 1 mm session **[EXPLICIT: 271-277]**.
3. **Weights** (287-316): $w_i = c_{s(i)}\cdot\max(0.05,\;|\hat{\mathbf n}_i\cdot\hat{\mathbf v}_i|)\cdot\min\!\big(1,(r_0/r_i)^2\big)$, with $\hat{\mathbf v}_i$ the unit vector to the scan's sensor, $r_i$ the range, $r_0=0.6$ m. Rationale: incidence (*"A depth sensor's error grows sharply at grazing incidence"*) and range (*"Stereo depth error grows with the square of range"*) **[EXPLICIT: fusion_reconcile.h:67-77]** — consistent with the RealSense stereo depth-RMS model $\sigma_z\propto z^2$ **[INFERRED link]**.
4. **Mode** (319-322): height field if `HeightField`, or `Auto` and scene planarity < 0.15; otherwise volumetric.
5. **Binning** (327-371): height field — key = 2D cell of $(u,v)$ in the scene plane frame with size `height_cell`, value = height $h$; volumetric — key = 3D voxel of size `voxel_size`, value filled later.
6. **Per cell** (384-472): volumetric cells get an axis = hemisphere-aligned mean normal and value = projection of each position on it (393-403) (*"a voxel must be reconciled along ONE axis"* **[EXPLICIT: 363-367]**). Drop the cell if fewer than `min_contributing_scans` distinct scans (405-412). Record pre-blend spread (population std of values) for multi-scan cells. Compute weighted mean position, normal (hemisphere-aligned) and colour; reconciled value = **trimmed weighted mean** (443). Output point: height field — $(\bar u,\bar v)$ of the weighted mean with the reconciled height, mapped back to world (446-454); volumetric — weighted-mean position slid along the axis to the reconciled value (455-462).
7. **Off-surface outlier removal** (`removeOffSurfaceOutliers`, 173-225): for each point, PCA plane of its 31 nearest neighbours, residual = distance to that plane; drop residual > mean + 3σ. Deliberately not SOR because SOR *"measures density"* and deleted sparse-but-genuine areas, *"punching holes in genuine overlap"* **[EXPLICIT: fusion_reconcile.h:117-123; commit 82468b8 "Fusion code changed to reduce the effect of holes in fused scan"]**.
8. **MLS** (492-510): `pcl::MovingLeastSquares<PointT,PointNT>`, polynomial order 2, radius `mls_radius` (0.01 m), normals computed, multithreaded. MLS (Levin 1998; Alexa et al. 2003 point-set surfaces) fits a local reference plane and a bivariate polynomial height function in each neighbourhood and projects the point onto it; here it *"averages along the surface normal over a radius larger than the misregistration while keeping lateral detail"* **[EXPLICIT: README 685-691]**. If MLS returns nothing, the blended cloud with estimated normals is used (512-523).
9. **Thickness after** (529-583): same binning on the output; mean per-cell std; `ReconcileReport::print` (230-247).

**Trimmed weighted mean** (`trimmedWeightedMean`, 53-88): sort samples by value; total weight $W$; keep the central band $[\tau W,(1-\tau)W]$ of the cumulative weight axis (τ capped at 0.49), each sample contributing the overlap of its weight interval with the band:

$$\hat h = \frac{\sum_i \lvert [W_{i-1},W_i]\cap[\tau W,(1-\tau)W]\rvert\;h_i}{\sum_i \lvert [W_{i-1},W_i]\cap[\tau W,(1-\tau)W]\rvert}.$$

τ = 0 is the weighted mean, τ → 0.5 the weighted median. Trimming by weight means *"a single high-confidence scan is not outvoted by several poor ones"* **[EXPLICIT: 45-47]**. README table: with one 8 mm-misregistered scan in four, error 2.00 mm (mean) vs 0.35 mm (τ = 0.25) vs 0.34 mm (median) **[EXPLICIT: README 657-661]**; overall *"reconciliation halved the error of the old concatenate-and-voxel approach (0.96 mm rms → 0.45 mm …)"* against a synthetic surface **[EXPLICIT: README 707-710]** (test harness not present in the repository — cannot verify).

### 4.8 Viewer: `visualizer`, `legend_panel`, `scan_colors`

**Technology.** PCL's `PCLVisualizer` (VTK), not Open3D. Keyboard, mouse and point-picking callbacks only set request flags; all session mutation happens on the main thread inside the blocking `review()` loop **[EXPLICIT: visualizer.h:115-118]**.

**ROI selection** (`selectRoi`, `visualizer.cpp:638-725`): shows the raw cloud with a `vtkBoxWidget` placed on its bounds (rotation disabled, axis-aligned); space confirms, `d` uses the full cloud. The box is read from the widget's polydata bounds because `GetTransform()` is relative to the initial placement — the earlier implementation *"produced a 1 m cube near the optical centre … so every crop silently fell back to the full cloud"* **[EXPLICIT: 683-693]**.

**Review loop** (`review`, 452-635): pre-selects the first flagged/failed scan; redraws each contributing scan in its own colour (`ScanColors::forIndex`, golden-angle hue stepping, `scan_colors.h:32-37`; flagged scans tinted red, 40-45; selected scan drawn with point size 4); handles nudges, exclude toggles, `t` flips (followed by `coarseAlignScan` for every scan in the flipped and later groups, 564-598), single-scan reruns, and returns a `ReviewAction`.

**Keys** (`onKeyboard`, 79-212): Up/Down select; `v` show/hide (opacity 0/1); `i` isolate/un-isolate; `a` all; `d` exclude (and "no crop" in ROI phase); `n` nudge mode; `m` help; Enter / Shift+Enter re-run current stage for one / all; `z`/`Z` re-run coarse for one / all; `t` flip rotation group; space continue; `b` back; `q`/Esc stop. Keys avoid PCL's built-ins because *"our callback runs *in addition* to those"* and `e` exits the interactor **[EXPLICIT: visualizer.h:49-53]**.

**Nudging** (`nudgeSelected`, 426-449): arrows ±x/±y, PgUp/PgDn ±z by `nudge_step_m_` (2 mm), `,`/`.` rotate ∓/± `nudge_step_deg_` (0.5°) about the **common-frame z axis through the scan's centroid**: $\Delta=\begin{bmatrix}R_z&\mathbf c-R_z\mathbf c+\mathbf t\\0&1\end{bmatrix}$, applied as $T\leftarrow\Delta T$; `+`/`-` double/halve both steps.

**Legend** (`legend_panel.cpp`): one text row per scan (`>`, visibility `[o]`, index, status glyph `. R c ! F X`, short label, rmse mm, overlap %, `!` if flagged, 73-94); scrolls to keep the selection visible (123-134); click hit-testing on retained row geometry in VTK's bottom-left display coordinates, click width capped at a third of the window (115, 182-193).

### 4.9 `main.cpp` — CLI and control

Already described in §2.3 and §3. Noteworthy: the fusion-cell < scan-voxel warning (446-453) reflects the README's density model (*"`--scan-voxel` … is the **ceiling on real detail**"*, README 712-740; commit `e26c38a`: *"Fusioon code accepts different voxel sizes, although introduces holes in final output"* **[EXPLICIT]**).

### 4.10 Build (`CMakeLists.txt`)

Refuses in-source builds (7-10; *"they scatter CMakeCache.txt … and those break as soon as the folder is copied"* **[EXPLICIT]**, commit `9de4ac1` "fixed fusion pieline build problems"); defaults to Release because PCL registration is header-only and an unoptimised build took *"83 s unoptimised, 15 s Release, identical output"* on a 28-scan session **[EXPLICIT: 12-16]**; warns against `-march=native` (Eigen alignment mismatch with distro PCL) **[EXPLICIT: 18-20]**; C++17; links `${PCL_LIBRARIES}` and Threads. A locally built `Processing/Fuse/build/fuse_scans` exists (untracked; mtime 2026-10-06 11:56, newer than every source file, and its strings include `tag-align`), so the binary matches the audited source **[observed via `ls`/`strings`, read-only]**.

## 5. Why this approach (consolidated)

| Choice | Alternative | Rationale | Label |
|---|---|---|---|
| Staged batch pipeline with reviews | one scan-at-a-time loop | review a whole pass at once, re-run any stage | **[EXPLICIT: README 13-20; main.cpp:29-39]** |
| Register against accumulated map | pairwise chain | lower drift | **[EXPLICIT: README 50-51]** |
| IMU-measured tilt | commanded angle about a fixed axis | tilt axis rotates in camera frame; servo inaccuracy | **[EXPLICIT: README 192-228]** |
| Detect IMU→cloud basis | assume identity | wrong basis invisible at small tilt, ruinous at large | **[EXPLICIT: README 230-267]** |
| Pivot by search (IMU path) | `(I−R)p=t` fit | IMU gives rotation but not lever arm | **[EXPLICIT: README 269-280]** |
| Joint 2^n direction search, raw seeds | greedy, ICP-settled | wrong early step affects later; ICP settles wrong hypotheses on flat scenes | **[EXPLICIT: README 372-384]** |
| Tag hypothesis scored, not trusted | always use tags | tags can be wrong (bad detection/depth) | **[EXPLICIT: README 461-467]** |
| Height-field score on flat scenes | NN rmse/overlap | NN blind to in-plane error | **[EXPLICIT: README 323-352]** |
| Point-to-plane ICP | point-to-point | faster, more accurate on continuous surfaces | **[EXPLICIT: fusion_pipeline.cpp:318-319]** |
| Reconcile with trimmed weighted mean | concatenate + voxel + SOR | multi-sheet artefact; robust to one bad scan | **[EXPLICIT: README 621-664]** |
| Off-surface outlier test | SOR | SOR deletes sparse genuine areas → holes | **[EXPLICIT: commit 82468b8; README 746-752]** |
| Precomputed sensor-oriented normals carried through transforms | re-estimate per ICP/fusion | 85 % / 220 s cost; normals cancel if mis-oriented | **[EXPLICIT: fusion_pipeline.cpp:321-326; fusion_reconcile.cpp:271-277]**; commit `538a460` "Optimised fusion code, much faster" |
| No global pose-graph optimisation | e.g. multiway registration (Choi et al. 2015) | not discussed in source | **[INFERRED: simplicity; sequential map assumed sufficient for ≤~30 scans]** |
| No TSDF / Poisson meshing | volumetric integration | output is a point set for metrics; README suggests Poisson as a later step | **[EXPLICIT: README 788-790 for the suggestion; rest INFERRED]** |

## 6. Parameters and tuning (consolidated)

`PipelineParams` (`fusion_pipeline.h:90-236`) and `ReconcileParams` (`fusion_reconcile.h:54-130`):

| Name | Default | Location | Effect |
|---|---|---|---|
| `voxel_leaf_size` (`--scan-voxel`) | 0.003 m | h:93 | load-time VoxelGrid and map leaf; ceiling on detail; cost ∝ 1/leaf² |
| `sor_mean_k`, `sor_stddev_mult` | 30, 1.0 | h:94-95 | per-scan SOR (aggressive: removes points > μ+1σ) |
| `normal_radius` | 0.015 m | h:99 | sensor normals and FPFH normals |
| `fpfh_radius` | 0.03 m | h:100 | FPFH support |
| `sac_max_correspondence`, `sac_max_iterations`, `sac_nr_samples` | 0.05 m, 50 000, 3 | h:101-103 | prerejective RANSAC |
| (hard-coded) correspondence randomness, similarity, inlier fraction | 5, 0.9, 0.25 | cpp:294-297 | prerejective RANSAC |
| `icp_max_correspondence_distance` | 0.02 m | h:106 | ICP gate & fitness gate |
| `icp_max_iterations`, `icp_transformation_epsilon`, `icp_euclidean_fitness_epsilon` | 60, 1e-8, 1e-6 | h:107-109 | ICP termination |
| `flag_score_ratio` | 1.4 | h:116 | flag vs batch median |
| `eval_correspondence_distance` | 0.01 m | h:121 | scoring gate (NN distance / height diff); sets confidence $r_0$ = 2.5 mm |
| `eval_max_samples` | 20 000 | h:122 | scoring subsample |
| `coarse_min_inlier_fraction` | 0.15 | h:127 | accept seed / mark coarse-failed |
| `use_tag_alignment`, `tag_min_shared`, `tag_max_fit_rmse`, `tag_disagreement_warn_deg` | false, 1, 0.010 m, 10° | h:134-150 | tag hypothesis |
| `tilt_pivot_search_range` | 0.12 m | h:158 | pivot grid half-range (step = range/4, ×0.4 per level) |
| `calibration_voxel_size` | 0.005 m | h:163 | calibration clouds |
| `calibrate_tilt_geometry`, `tilt_calibration_max_deg`, `tilt_calibration_max_scans` | true, 15°, 6 | h:172-180 | calibration set |
| `infer_rotation_direction`, `direction_test_icp_iterations`, `direction_max_joint_steps`, `direction_ambiguous_margin` | true, 0 (unused), 12, 0.15 | h:193-215 | direction search |
| `plane_grid_cell` | 0.01 m | h:221 | height-grid cell for scoring |
| `planarity_threshold` | 0.15 | h:226 | height-field scoring switch (fusion Auto uses a hard-coded 0.15) |
| `downsample_accumulated_map` | true | h:229 | map leaf = scan voxel (else no merging) |
| `final_voxel_leaf_size`, `run_final_outlier_removal`, `estimate_output_normals`, `output_normal_radius` | 0.002, true, true, 0.015 | h:232-235 | **unused** (only `postProcessMerged`) |
| `mode` | Auto | rh:62 | fusion mode |
| `weight_by_registration/incidence/range` | true | rh:65-76 | weights |
| `incidence_min_weight`, `range_reference` | 0.05, 0.6 m | rh:72, 77 | weight floor / reference range |
| `min_contributing_scans` | 1 | rh:86 | corroboration |
| `voxel_size`, `height_cell` | 0.002 m, 0.003 m | rh:89, 92 | fusion cell sizes (output count ∝ 1/cell²) |
| `trim_fraction` | 0.25 | rh:100 | robustness |
| `use_mls`, `mls_radius`, `mls_polynomial_order` | true, 0.010 m, 2 | rh:108-110 | surface fit; "a few times your residual rmse" |
| `outlier_neighbours`, `outlier_stddev_mult` | 30, 3.0 | rh:125-126 | off-surface filter |
| viewer nudge steps | 2 mm, 0.5° | visualizer.h:141-142 | manual nudge |

README tuning advice (README 756-784) **[EXPLICIT]**: set `voxel_leaf_size` to the sensor noise floor; ICP distance a few times coarse error; eval distance near expected residual (too large → everything scores 100 %); lean on rig seeds/tags for textureless planar scenes because FPFH is unreliable there.

## 7. Assumptions, limitations, failure modes

* **Scene model.** Height-field scoring and fusion assume a surface viewed from above (single height per lateral cell); overhangs and vertical walls are flattened/averaged in height-field mode. Auto mode chooses height field when planarity < 0.15 — a moderately wavy test object on a flat bed is very likely "planar" by this measure **[INFERRED]**.
* **Reference = first 0° scan.** If it has no tags (`--tag-align`), no tag chain can start (515-518).
* **Rotation pivot** assumed known (default at the optical centre) — not estimated. If the turntable axis is offset from the camera, rotation seeds are wrong by $|(I-R)\mathbf p|$; only tags/FPFH/ICP can repair this **[INFERRED from rig_geometry.h:33-36]**.
* **Without a rotation axis configured** (no `--rig-config`/`--rotation-axis`), groups get no rotation seed, the rotation-direction stage and tag-based group angles are skipped; scans in other groups then rely on the tilt-only seed, FPFH, or the tag hypothesis in the coarse batch.
* **A wrong seed can be accepted** whenever its NN coverage ≥ 15 % (720-730): on a flat-ish scene a 45°-wrong turn still overlaps heavily, so FPFH is never tried; only the tag hypothesis can override it **[INFERRED]**.
* **Sequential registration** — no global optimisation; errors in early scans propagate into the map.
* **ICP basin** of 20 mm: seeds farther off cannot be recovered (README 188-190).
* **Compute cost** scales roughly with (1/scan-voxel)² for every neighbourhood operation; normals (r = 15 mm), FPFH (r = 30 mm) and MLS (r = 10 mm) neighbourhood sizes grow likewise, so very small voxels (≤1 mm) are extremely expensive **[INFERRED]**; README: halving the voxel costs ~4× **[EXPLICIT: 736-737]**.
* **PCL VoxelGrid overflow guard.** PCL's `VoxelGrid` refuses to filter (copies the input through with a warning) if the bounding box divided by the leaf exceeds 2³¹ voxels **[INFERRED: PCL library behaviour, not visible in this repo]**; with a 0.5 mm leaf this happens for bounding boxes of roughly > 1.1 m on all three axes or any equivalent volume, e.g. uncropped clouds or cropped columns with far outliers.
* **Tags** need detections + intrinsics or a sidecar; tag fits need ≥1 shared tag (one tag fixes 6 DOF but poorly conditioned); constrained fit needs gravity on both scans and a seed.
* **IMU magnitude window** 9.0–10.6 m/s² rejects readings taken while moving; nothing checks the gyro.
* **Memory:** all prepared clouds are resident; fusion holds a `WeightedSample` (~48 B) per input point in hash-map vectors.

## 8. Dependencies

| Library | Used for (actual calls) |
|---|---|
| **PCL ≥1.10** | `io::loadPLYFile/savePLYFileBinary`; `CropBox`; `VoxelGrid` (centroid per voxel); `StatisticalOutlierRemoval`; `NormalEstimationOMP` (PCA normals, OpenMP); `FPFHEstimationOMP`; `SampleConsensusPrerejective` (feature RANSAC); `IterativeClosestPointWithNormals` (point-to-plane ICP; `getFitnessScore`, `getConvergeCriteria()->getConvergenceState()`); `MovingLeastSquares` (order-2 polynomial projection with normals); `search::KdTree` (FLANN nearest neighbours); `compute3DCentroid`, `computeCovarianceMatrixNormalized`, `computeMeanAndCovarianceMatrix`, `transformPointCloud`, `concatenateFields`, `copyPointCloud`, `getMinMax3D`; `visualization::PCLVisualizer` (+ VTK `vtkBoxWidget`, `vtkPolyData`, `vtkRenderWindow`) |
| **Eigen** | matrices; `SelfAdjointEigenSolver` (PCA); `JacobiSVD` (pivot least squares); `umeyama`; `AngleAxisf`; `Quaternionf::FromTwoVectors` |
| C++17 std | `<filesystem>` (discovery), `<regex>` (angle tokens), `std::thread::hardware_concurrency` (MLS threads) |
| Threads | linked (`Threads::Threads`) |
| Internal | inputs from `rs_marker` (PLY, IMU CSV, detections CSV, `scan_info.txt`) and from the crop tool (cropped PLY, `_tags.csv`); invoked by `Bulk_Process_Pipeline/bulk_process.py` |

## 9. Design history (git)

| Commit | Date | Change (from message/diff) |
|---|---|---|
| `6de8e04` | 09-10 | Moved from `Capture/More_Tools/Fuse`; interactive scan list, exclude, nudge; single fused cloud "(atm low density)" |
| `e26c38a` | 09-11 | `--scan-voxel`, density section in README; "introduces holes in final output" |
| `82468b8` | 09-14 | SOR replaced by off-surface outlier removal "to reduce the effect of holes"; "fusese take along time" |
| `538a460` | 09-14 | "Optimised fusion code, much faster and performance metrics are now better suited for scans": precomputed normals, `VoxelMap`, height-grid caching (diff touches `fusion_pipeline.*`, `fusion_session.*`, `scan_state.*`) |
| `9de4ac1` | 09-15 | Build fixes: `.gitignore` for build dirs, in-source build guard, README build notes |
| `00b2c4e` | 10-06 | `tag_alignment.{h,cpp}` (+523/+108), `--tag-align` options, sidecar support, ~350 lines in `fusion_session.cpp` (tag poses, tag-measured rotation) |

Build artefacts and output PLYs (`Out*.ply`, object files, the binary) were committed in earlier commits and later removed (538a460, 9de4ac1).

## 10. Observations

1. **`compose_tilt_first` is ignored after the first stage.** Discovery seeds use `rigGuess`, which honours it (`rig_geometry.cpp:66`), but every later reseed via `applyGuessToScan` hard-codes $R_{rot}\cdot T_{tilt}$ (`fusion_session.cpp:237`). README (209-210) says the physical rig corresponds to `compose_tilt_first = false`, while `rig_config.example.txt` sets `true`. With the IMU path the group-relative construction makes the order correct anyway, but the flag is misleading and inconsistent between stages.
2. **`--no-tilt-calibration` with IMU data leaves `imu_to_cloud` at identity** (calibration is the only place it is detected, `fusion_session.cpp:1160`), so IMU tilt seeds and the tag "up" vector (526-530) are then computed in the wrong basis whenever the PLY is in the export frame (README shows Rx180 is the true relation). Nothing warns about this.
3. **Fusion Auto mode ignores `planarity_threshold`**: `fusion_reconcile.cpp:321` hard-codes `0.15`.
4. **Dead/unused code and parameters**: `postProcessMerged`, `buildMergedCloud`, `final_voxel_leaf_size`, `run_final_outlier_removal`, `estimate_output_normals`, `output_normal_radius`, `direction_test_icp_iterations`.
5. **Stale console help** in the viewer constructor (`visualizer.cpp:68-75`) advertises `e`, `c`, `f`, `C`, `F`, `r` — keys that are either PCL built-ins (e exits!) or no longer handled — contradicting the README and the on-screen help.
6. **Nudge step keys `+`/`-` collide with PCL built-ins** (README 609-611 lists `+/-` as reserved), so changing the nudge step also changes point size.
7. **Duplicate handling** of `rerun_stage_all_requested_` (`visualizer.cpp:502-507` and 557-562); the second block is unreachable.
8. **Height-field rebuild comment vs code**: comment says the output point is rebuilt "from its cell centre" (`fusion_reconcile.cpp:448-449`) but the code uses the weighted mean lateral position (450-451). Similarly the volumetric thickness comment says "Project onto a fixed world axis" (546-549) while the code uses the first point's normal per voxel.
9. **Thickness-after metric** is computed on the MLS output, where most height-field cells hold one point and are skipped (`entry.second.size() < 2`, 573), so "before" and "after" are averaged over different cell populations; the headline "surface thickness X → Y mm" is not like-for-like **[INFERRED]**.
10. **`evaluateAlignmentOnPlane` uses `min_points_per_cell = 3`** on voxelised maps, while the header warns that voxelised clouds need 1 (`fusion_pipeline.h:348-352`); with large scan voxels relative to `plane_grid_cell` the grid can empty and every hypothesis scores invalid **[INFERRED]**.
11. **Seed acceptance short-circuits FPFH and only checks coverage** (720-730); a rotation-wrong seed with ≥15 % NN coverage is accepted without comparison (tags are the only check).
12. **Unknown CLI arguments are silently treated as PLY paths** (365-368) and then ignored in session mode, so a typo in a flag (e.g. `--fusion-cel`) produces no error.
13. **`--sessions` argument parsing** stops at the first token beginning with `-`, so a session path starting with `-` cannot be passed; harmless in practice.
14. **printUsage text is outdated**: `--rotation-axis`/`--rotation-step-deg` are described as "between sessions" (82-86) although rotations are now subfolders; the header comment (29-33) omits the tilt-calibration and rotation stages.
15. **Per-scan SOR with σ-multiplier 1.0** (h:95) removes roughly the top ~15 % of points by mean neighbour distance in every scan; combined with VoxelGrid this may thin edges and contribute to holes noted in commits e26c38a/82468b8 **[INFERRED]**.
16. **README path mismatch**: README tells users to `cp rig_config.example.txt` from the Fuse root, but the file is in `src/`.
17. **`reloadScan`** assigns uninitialised `roi_min/roi_max` when the ROI provider returns false (`fusion_session.cpp:462-465`); harmless because `has_roi` is then false, but sloppy.
18. **Tilt-subfolder iteration is unsorted** (`scan_discovery.cpp:297`), making scan order (and thus map-building order and results) filesystem-dependent for equal |tilt|.
19. **No unit tests** or regression data accompany the claims in the README's measurement tables (synthetic-surface errors, 6-scan density table, Small_Wave tag results); these cannot be reproduced from the repository alone.
