# 06 — Point-cloud cropping (`Processing/Crop`)

**Summary.** `crop_pointclouds.py` is a batch, partly interactive Python/Open3D tool that walks a capture *session* folder, and for every per-angle `.ply` written by `rs_marker` cuts the cloud down to the region of interest: the square on the rig bed bounded by the **inner corners of four AprilTags**. The automatic crop reprojects every 3D point back into the colour image (using colour intrinsics it parses directly out of the `.db3` recording, which is actually a ROS1 bag), builds the tag-bounded quadrilateral in pixel space from `*_detections.csv` (extrapolating missing tags from a learned layout), fits the bed surface and the tag plane with RANSAC, and keeps every point inside a 3D *prism* (column) erected on that quad along the surface normal. Each result is previewed in an Open3D window (accept / manual / keep original / quit); a two-step manual box crop is the fallback. Because cropping deletes the tags, the tool also writes a **tag sidecar** (`<scan>_tags.csv`: 3D tag corners measured on the *uncropped* cloud) that the C++ fusion tool reads for `--tag-align`. It copies the IMU, detections and `scan_info.txt` files beside each cropped cloud so the output tree is a drop-in input for `fuse_scans` and for the bulk pipeline (08). `rename_ply.sh` is a 18-line helper that strips the `_cropped` suffix from output filenames. Everything below was derived from reading source, comments, READMEs and git history only; nothing was executed.

## Files covered

| Path | Lines | Role |
|---|---|---|
| `Processing/Crop/crop_pointclouds.py` | 1231 | Batch crop tool: auto (AprilTag) crop, manual crop UI, tag sidecar writer, metadata copier |
| `Processing/Crop/rename_ply.sh` | 18 | Bash helper: rename `*_*_r*_a*_cropped.ply` → `*.ply` |
| (context) `Processing/Fuse/src/tag_alignment.cpp` | 523 | C++ twin of the sidecar's tag-location method (see 07) |
| (context) `Capture/programs/rs_marker/main.cpp` | — | Producer of the inputs (detections CSV header line 268, IMU CSV line 294, `scan_info.txt` 694–760, `export_to_ply` 1212) |

Ignored per audit rules: `Processing/Crop/cropper/` (a Python venv), `Processing/Crop/__pycache__/`.

## Contents

1. Purpose
2. Inputs and outputs
3. Step-by-step walkthrough
4. Algorithms and methods
5. Why this approach
6. Parameters and tuning
7. Assumptions, limitations, failure modes
8. Dependencies
9. `rename_ply.sh`
10. Observations

---

## 1. Purpose

The raw clouds exported by `rs_marker` contain the whole camera view: the test object, the rig bed, the AprilTags and whatever surrounds the rig. Downstream steps (fusion, accuracy evaluation, roughness metrics) only want the test surface. The tool solves two problems:

* **Region of interest.** Remove everything outside the tagged square, consistently across the ~28 scans of a session (rotations × tilts), without hand-cropping each one. Commit `473583a` (2026-09-28): *"python crop tool now tries to automatically crop the depth image to a region bounded by four april tags shown in the detecitons.csv file present in capture"* **[EXPLICIT: commit 473583a]**. The first version (commit `e26c38a`, 2026-09-11) was manual-box-only: *"Added pyhton scrip to crop ply files to smaller size"* **[EXPLICIT: commit e26c38a]**.
* **Keeping metadata with the data.** Fusion needs each scan's IMU sample (`c1ce006`: *"changed cropping python script to also coppy IMU files"* **[EXPLICIT: commit c1ce006]**) and, for tag-based alignment, the tags' 3D corners, which the crop removes. Commit `00b2c4e` added the sidecar for that reason (`crop_pointclouds.py:42-48`) **[EXPLICIT: docstring]**.

Place in the system: **capture (`rs_marker`) → crop (this) → fuse (07) → evaluate / rugosity**. It is also imported as a library by the bulk pipeline (08), which calls `compute_auto_crop`, `learn_tag_layout`, `write_ply_like_original`, `copy_imu_file`, `write_tag_sidecar`, `copy_scan_metadata`, `preview_auto_crop`, `manual_crop` directly (`Bulk_Process_Pipeline/bulk_process.py:204-286`).

## 2. Inputs and outputs

### 2.1 Command line (`main`, `crop_pointclouds.py:1105-1157`)

| Argument | Type / default | Meaning |
|---|---|---|
| `input_dir` | path (required) | Top-level `<Session>` directory; searched recursively for `*.ply` (`find_ply_files`, line 125) |
| `output_dir` | path (required) | Mirror root for outputs |
| `--suffix` | `_cropped` | Appended to the output filename stem |
| `--skip-existing` | flag | Skip clouds whose output exists (but still copy IMU/metadata and back-fill a missing sidecar, lines 1202-1209) |
| `--tags-only` | flag | No cropping, no windows: only write sidecars + copy detections/`scan_info.txt` (lines 1174-1182) |
| `--no-auto` | flag | Never attempt the tag crop; always manual |
| `--auto-accept` | flag | Save automatic crops without the preview window |
| `--tag-order` | `"6,5,2,4"` style | Clockwise order of the four corner-tag IDs; default learned from the session |
| `--frame-window` | int, 15 | Detections from ± this many frames around the exported frame may fill in missing tags |
| `--margin` | float px, 0 | Pull the auto-crop quad this many pixels inward (negative grows it) |

### 2.2 Input files per scan (layout from docstring lines 6-10)

```
<Session>/<Session>_r<deg>/<Session>_r<deg>_a<deg>/<Session>_r<deg>_a<deg>.ply
                                                  /<Session>_r<deg>_a<deg>_detections.csv
                                                  /<Session>_r<deg>_a<deg>.db3
                                                  /imu_a<deg>.csv
                                                  /scan_info.txt
```

| File | Format | Used for | Reader |
|---|---|---|---|
| `*.ply` | binary PLY from librealsense `export_to_ply` (`rs_marker/main.cpp:1212`): float32 xyz + uint8 rgb, metres, in librealsense's export frame $(x,-y,-z)$ of the camera frame (`_ply_to_camera`, line 307-309). Depth was aligned to colour before export (`rs_marker/main.cpp:1131`), so each point originates from an integer **colour** pixel | the data | `o3d.io.read_point_cloud` (lines 1179, 1212) |
| `*_detections.csv` | CSV header `timestamp_ms,frame,marker_id,cx,cy,depth_m,c0x,c0y,c1x,c1y,c2x,c2y,c3x,c3y,decoded` (`rs_marker/main.cpp:268-269`). Pixel coordinates in the colour image; `marker_id = -1` / `decoded = 0` = undetected-ID square candidate | tag positions | `load_detections` (370-387) via `csv.DictReader` |
| `imu_a<y>.csv` | header `frame,timestamp_ms,accel_x,...,gyro_z` (`rs_marker/main.cpp:294`), one data row: the IMU sample of the exported "best frame" | the **frame index** the PLY was exported from | `read_imu_frame` (398-403) |
| `*.db3` | despite the extension, a **ROS1 bag v2.0** written by the librealsense recorder (checked by magic `#ROSBAG V2.0`, line 244) | colour intrinsics (`.../Color_0/info/camera_info` topic) | `read_color_intrinsics` (240-262) |
| `scan_info.txt` | rs_marker text config dump | only *copied* (the C++ fuser reads intrinsics from it) | `copy_scan_metadata` (142-147) |

### 2.3 Outputs (mirrored under `output_dir`)

| Output | Format | Notes |
|---|---|---|
| `<rel>/<stem><suffix>.ply` | binary PLY, **float32** xyz + **uint8** rgb, same frame as input | `write_ply_like_original` (162-175) deliberately avoids Open3D's legacy float64 writer because *"downstream PCL-based tools (expecting float32) silently fail to parse"* **[EXPLICIT: line 164-166]** |
| `imu_a<y>.csv` | copied verbatim | `copy_imu_file` (150-159) |
| `<tilt folder>_tags.csv` | CSV `marker_id,corner,x,y,z,frames,plane_rms_m,edge_m`, 4 rows per located tag, metres, PLY frame | `write_tag_sidecar` (823-854); consumed by `Fuse/src/tag_alignment.cpp:221-259` (`readTagSidecar`) |
| `*_detections.csv`, `scan_info.txt` | copied verbatim | `copy_scan_metadata` (142-147); needed by `fuse_scans` when no sidecar exists |

Coordinate frames: all 3D outputs stay in the **input PLY's frame** (camera-centred, librealsense export convention). Internally the code switches to the camera frame (x right, y down, z forward) with `_ply_to_camera`, which is its own inverse.

## 3. Step-by-step walkthrough

### 3.1 `main` (1105-1227)

1. Parse args, resolve paths, list all `*.ply` under the input (1159-1169); print the docstring as usage help (1172).
2. `--tags-only` branch (1174-1182): for each PLY, load it, `write_tag_sidecar`, `copy_scan_metadata`, then exit.
3. Otherwise learn the corner-tag layout once for the whole session: `learn_tag_layout(input_dir, args.tag_order)` (1184-1196).
4. For each PLY (1198-1225): compute the mirrored `out_path` (1200); honour `--skip-existing` (1202-1209); load the cloud (1212); `crop_point_cloud` (1214); on `QuitAll` exit immediately (1215-1217); write the cropped cloud as float32 (1220); copy IMU (1222); write the tag sidecar **from the original, uncropped cloud** (1223-1224); copy detections and `scan_info.txt` (1225).

### 3.2 `crop_point_cloud` (1074-1102)

Try `compute_auto_crop` unless `--no-auto`; an `AutoCropError` prints the reason and falls back to `manual_crop` (1078-1081). On success it prints the diagnostic notes and point counts, builds `cropped = pcd.select_by_index(mask)` (1087), and either auto-accepts or shows `preview_auto_crop` (1089-1092). Results: `"crop"` → cropped, `"keep"` → original, `"quit"` → `QuitAll`, `"manual"` → manual crop seeded with the auto crop's axis-aligned bounding box (1099-1100).

### 3.3 `compute_auto_crop` (693-753) — the core

1. Find `imu*.csv` and `*_detections.csv` beside the PLY (694-699); raise `AutoCropError` if missing.
2. `frame = read_imu_frame(imu_path)` — the index of the exported frame (701).
3. `load_detections` → `{frame: [Tag]}`; `collect_tags(by_frame, frame, args.frame_window)` (702-703) returns
   * `decoded`: one tag per ID, taken from the frame **nearest** the exported frame within the window (477-483);
   * `undecoded`: undecoded squares, de-duplicated by position (centre farther than half a side from every known square, 485-492).
   The docstring justifies the window: *"The camera is static during a scan, so nearby frames fill in tags that flickered out of the exported frame"* **[EXPLICIT: 475-476]**.
4. Colour intrinsics: `find_color_intrinsics` tries the scan's own `.db3`, then any `.db3` in the session (cached per bag; 268-281).
5. `match_intrinsics_to_cloud` (337-350): tries three distortion directions (`forward`, `inverse`, `none`) and keeps the one that lands a ≤50 000-point sample of the cloud closest to the integer pixel grid; if the RMS residual exceeds `MAX_GRID_RESIDUAL_PX = 0.15` px the intrinsics are declared not to belong to this cloud (711-714).
6. `compute_crop_quad` (586-631, see §4.4) → pixel quad bounded by the four inner tag corners plus notes and the list of real tags used; `shrink_quad` by `--margin` (717-718).
7. Project all points to pixels `uv` (722); convert to camera frame (721).
8. **Surface plane** (731): RANSAC plane through points in a *band* just inside the quad edge — inside the quad but outside the quad shrunk by `PLANE_BAND_PX = 25` px (727).
9. **Tag plane** (728-732): RANSAC plane through points inside any real tag's pixel quad; falls back to the surface plane.
10. If a surface plane exists (734-742): intersect the four quad-corner pixel rays with the **tag plane** → 3D corners; `prism_crop` keeps points whose projection onto the plane perpendicular to the **surface normal** falls inside those corners (no height limit). The outline (for drawing) is the corners dropped onto the surface plane, extruded from min(height,0) to max height of kept points (738-740).
11. Else (743-749): fall back to the camera frustum through the pixel quad (`points_in_quad(uv, quad)`), outline from the 0.5/99.5 depth percentiles.
12. Fewer than `MIN_AUTO_CROP_POINTS = 1000` kept → `AutoCropError` (751-752).

The comment at 723-726 states the design: *"The surface inside the square (fitted to a band just inside the crop edge, away from the object) sets the column's direction. The tags may sit higher than it, so the column's corners go where the inner-corner rays meet the tags' own plane."* **[EXPLICIT]**

### 3.4 `learn_tag_layout` (428-469)

Scans every `*_detections.csv` in the session; for each frame with **exactly four decoded tags** it records the cyclic ID order of the tag centres sorted by angle about their centroid (`_angle_order`, 406-411; `_canonical_cycle` rotates the cycle to start at the smallest ID, 414-417). The most common cycle is the layout order (unless `--tag-order` given; disagreeing frames are reported, 450-453). For frames matching that order it records (i) `side_ratio` = mean centre-to-centre square side ÷ mean tag side and (ii) `aspect[i]` = mean of the two sides adjacent to side *i* of the inner-corner crop quad ÷ side *i* (460-465). Medians over frames are returned (469).

### 3.5 Tag sidecar: `write_tag_sidecar` (823-854) and `locate_tags_3d` (788-820)

Comment line 760: *"Mirrors locateTags() in Fuse/src/tag_alignment.cpp - keep the two in step."* **[EXPLICIT]**. Steps per decoded tag ID:

1. `median_tag_corners` (769-777): per-ID **median of corner pixels over every decoded frame** of the recording (*"The camera is still for the whole recording, so the median only removes detector jitter"* **[EXPLICIT: 770-771]**).
2. Select cloud points whose pixel lies inside the tag quad shrunk to 70 % about its centre (`TAG_INNER_SCALE = 0.7`, 798-799) — *"border pixels mix tag and background depth"* **[EXPLICIT: 762-763]**. Need ≥ 30 points.
3. Least-squares plane by SVD (`_lsq_plane`, 780-785); keep points within `max(3 mm, 2.5·rms)`; refit (805-810).
4. Each corner = intersection of its pixel ray with the tag plane (812-817), converted back to the PLY frame. Edge length = mean of the four edges (818), printed so the user can compare with the printed tag size.
5. Rows written with 6-decimal metres (843-849).

### 3.6 Interactive windows

* `preview_auto_crop` (872-899): Open3D `VisualizerWithKeyCallback`; removed points are dimmed to `0.25·c + 0.1` (877); the green wireframe from `_crop_outline` (861-869). Keys Enter / M / N / Esc.
* `manual_crop` (1017-1071): step 1 `pick_corners_with_mouse` (920-937) uses `VisualizerWithEditing` (Shift+click picks); the AABB of the picked points (≥2) seeds step 2. Step 2 is `BoxCropper` (940-1014) on an axis-aligned box: arrows / PgUp/PgDn move by `step` (initially 2 % of the largest cloud extent, 954), 1–6 resize per axis symmetrically with a 1 %-of-extent minimum (978-992), 7/8 scale the step by 1/1.5 and 1.5, R resets, Enter crops via `pcd.crop(AxisAlignedBoundingBox)` (1064), N keeps the original, Esc raises `QuitAll`. An empty crop saves the original with a warning (1065-1067).

## 4. Algorithms and methods

### 4.1 Pinhole projection with librealsense's Brown–Conrady variant (`project_to_pixels`, 312-322; `_distort`, 284-295)

A camera-frame point $(X,Y,Z)$ with $Z>0$ maps to normalised coordinates $x=X/Z,\;y=Y/Z$. With $r^2=x^2+y^2$ and radial factor $f=1+k_1r^2+k_2r^4+k_3r^6$, the code first scales $x'=fx,\;y'=fy$ and then adds tangential terms **on the scaled coordinates but with the unscaled $r^2$**:

$$x_d = x' + 2p_1x'y' + p_2(r^2+2x'^2),\qquad y_d = y' + 2p_2x'y' + p_1(r^2+2y'^2)$$

$$u = f_x x_d + c_x,\qquad v = f_y y_d + c_y.$$

The comment (285-288) states this replicates librealsense's `rs2_project_point_to_pixel` and reprojects an exported cloud onto the integer pixel grid to ~$10^{-5}$ px, versus ~0.01 px with textbook Brown–Conrady **[EXPLICIT: comment, and the 00b2c4e diff that introduced it]**. `pixel_rays` (325-334) inverts the mapping; `_undistort` (298-304) is a 20-iteration fixed-point inversion $u_{k+1}=u_k+(x-\mathrm{distort}(u_k))$.

### 4.2 Intrinsics validation by pixel-grid residual (`match_intrinsics_to_cloud`, 337-350)

Because depth was aligned to colour and deprojected per colour pixel, a correct projection must put every point on integer $(u,v)$. The residual

$$\rho = \sqrt{\frac{1}{N}\sum_i \lVert \mathbf u_i - \operatorname{round}(\mathbf u_i)\rVert^2}$$

is computed for three hypotheses (distortion applied forward, inverted, or ignored) and the smallest wins; $\rho > 0.15$ px rejects the intrinsics. This is a self-consistency check rather than a published method.

### 4.3 Reading a ROS1 bag without ROS (`_bag_records`, `_parse_camera_info`, `read_color_intrinsics`, 198-262)

The bag is parsed record by record (`<uint32 header_len><header><uint32 data_len><data>`; header fields are `len,key=value`). Uncompressed chunk records (`op=0x05`) are descended into; connection records (`op=0x07`) map connection IDs to topic names; the first message record (`op=0x02`) on a topic ending `Color_0/info/camera_info` is decoded as `sensor_msgs/CameraInfo` (seq, stamp, frame_id, height, width, distortion_model, D[], K[9]) with $f_x=K_0, f_y=K_4, c_x=K_2, c_y=K_5$ (220-237). Compressed chunks are silently skipped (line 252), so a bz2/lz4 bag yields no intrinsics.

### 4.4 Tag-bounded quad in image space (`compute_crop_quad`, 586-631; `complete_square`, 510-541)

*Slot assignment.* With a learned order and ≥2 decoded layout tags, slot $i$ = tag `order[i]` (592-593). Empty slots are filled with an undecoded square whose centre lies within `0.35·side` of the predicted position (598-609). Otherwise (no layout, or <2 known IDs), all squares are pooled (decoded first, then undecoded sorted by size similarity), squares lying inside the triangle of three others are dropped as false detections (`_drop_interior`, 549-561, using a sign-of-cross-product point-in-triangle test 544-546), and slots are assigned geometrically (`_slots_without_layout`, 564-583): 4 → angular order; 3 → the corner opposite the longest side is the middle slot; 2 → diagonal if their distance (in tag sides) is nearer $\sqrt2\cdot$`side_ratio` than `side_ratio`, else assumed adjacent with the square towards the image centre.

*Completion of missing corners* (`complete_square`):

* 3 known (parallelogram completion): $\mathbf p_m = \mathbf p_{m-1} + \mathbf p_{m+1} - \mathbf p_{m+2}$.
* 2 adjacent ($i,i{+}1$): side $\mathbf e=\mathbf p_{i+1}-\mathbf p_i$, inward normal $\mathbf n = k_i R_{90}\mathbf e$ with $R_{90}(v)=(-v_y,v_x)$ (504-507) and $k_i$ the learned aspect (1 if unknown); $\mathbf p_{i+2}=\mathbf p_{i+1}+\mathbf n$, $\mathbf p_{i+3}=\mathbf p_i+\mathbf n$.
* 2 diagonal ($i,i{+}2$): the diagonal is $\mathbf D=\mathbf e + kR_{90}\mathbf e$; solving the 2×2 linear system gives

$$\mathbf e = \frac{1}{1+k^2}\begin{pmatrix} D_x + kD_y \\ D_y - kD_x\end{pmatrix}$$

(line 534), then proceeds as the adjacent case.

*Inner corners* (628-631): the square's centre is computed from the (completed) tag **centres**; for each real tag the corner nearest that centre is its inner corner; the four inner corners are completed again the same way. Result: a quad whose edges run along the inner tag corners, so the tags themselves are excluded.

*Point-in-convex-polygon* (`points_in_quad`, 634-641): vertices sorted by angle; a pixel is inside iff $(\mathbf q_{k+1}-\mathbf q_k)\times(\mathbf u-\mathbf q_k)\ge 0$ for all edges. `shrink_quad` (644-648) moves each vertex a fixed number of pixels towards the centroid along the vertex–centroid direction.

### 4.5 RANSAC plane fitting (`fit_plane`, 651-666)

Open3D `PointCloud.segment_plane(distance_threshold=0.005, ransac_n=3, num_iterations=500)` — RANSAC (Fischler & Bolles 1981): repeatedly fit a plane to 3 random points and keep the one with the most inliers within 5 mm. The plane $ax+by+cz+d=0$ is normalised and oriented so the camera origin is on the positive side ($d>0$), making $h(\mathbf p)=\mathbf n\cdot\mathbf p+d$ the height above the plane towards the camera. Needs ≥500 sample and inlier points.

### 4.6 Back-projecting the quad onto the tag plane and prism crop (`intersect_rays`, 669-673; `prism_crop`, 676-683)

For pixel ray direction $\mathbf r=(x,y,1)$ and plane $(\mathbf n,d)$: $\mathbf X=\frac{-d}{\mathbf n\cdot\mathbf r}\,\mathbf r$. The four 3D corners and all points are projected onto an orthonormal basis $\{\mathbf e_1,\mathbf n\times\mathbf e_1\}$ of the plane perpendicular to the surface normal ($\mathbf e_1$ = first quad edge with its normal component removed); the 2D point-in-quad test then defines an infinite prism along $\mathbf n$. Effect: the crop boundary is a vertical (surface-normal) wall through the tag inner corners, independent of the camera's viewing angle, so tilted scans are cut along the same world boundary — unlike the frustum fallback, whose walls follow the camera rays.

### 4.7 Least-squares plane by SVD (sidecar, `_lsq_plane`, 780-785)

Centroid $\bar{\mathbf p}$; the plane normal is the right-singular vector of $[\mathbf p_i-\bar{\mathbf p}]$ with the smallest singular value (total least squares / PCA). One round of inlier trimming at $\max(3\text{ mm},2.5\,\mathrm{rms})$ then refit. Corners: $\mathbf X_k = \frac{\mathbf n\cdot\bar{\mathbf p}}{\mathbf n\cdot\mathbf r_k}\mathbf r_k$.

### 4.8 Robust pixel corners

Temporal median of each corner coordinate over all decoded frames (sidecar) — a robust estimator of a static target under detector jitter. The crop quad, in contrast, uses only the nearest frame per tag within ±`frame_window` frames (no averaging).

AprilTag detection itself is not done here; it comes from `rs_marker`'s detector via the CSV (see the capture chapter).

## 5. Why this approach

| Decision | Rationale | Label |
|---|---|---|
| Crop by the four tags rather than a fixed box | Commit message describes auto-cropping to the tag-bounded region; tags are fixed in the world, so the same physical square is kept in every rotated/tilted scan | **[EXPLICIT: 473583a]** for the intent; consistency argument **[INFERRED]** |
| Use the tags' **inner** corners | So *"the tags themselves are cut away"* | **[EXPLICIT: docstring 19-20]** |
| Map 3D → pixel with recorded intrinsics instead of fitting a 2D crop in 3D | Detections are in colour pixels and the cloud is depth-aligned-to-colour, so exact reprojection is available | **[EXPLICIT: docstring 29-31; Fuse/src/tag_alignment.h:20-24]** |
| Grid-residual check & distortion-direction auto-selection | *"Every point of a RealSense cloud comes from an integer pixel, so the right projection puts the points on the pixel grid"* | **[EXPLICIT: 338-340]** |
| Prism along surface normal, corners on tag plane | Tags may be raised above the bed; column direction should follow the bed | **[EXPLICIT: 723-726]** |
| Plane band 25 px inside the edge | Fit the bed *"away from the object"* | **[EXPLICIT: 110-112, 723-724]** |
| Learning tag order / aspect from 4-tag frames | Tells whether two tags share a side or a diagonal and on which side the square lies | **[EXPLICIT: docstring 25-28]** |
| Undecoded squares only fill gaps | *"Decoded tags are preferred. Undecoded squares (marker_id -1) are only used to fill in corners that no decoded tag covers."* | **[EXPLICIT: 21-22]** |
| Float32 PLY writer | PCL tools fail on Open3D's float64 output | **[EXPLICIT: 164-166]** |
| Sidecar measured on original cloud | *"The crop cuts the AprilTags out of the cloud"*; crop keeps the frame so the corners stay valid | **[EXPLICIT: 42-46; Fuse README:496-505]** |
| Preview + manual fallback | Human-in-the-loop safety net when tags fail | **[INFERRED]** |

Alternatives not taken (all **[INFERRED]**): detecting the tags directly in the PLY colours (would need a detector in Python); a 3D oriented bounding box from tag corners only (would need all four tags located in 3D; the pixel-quad approach tolerates missing tags via extrapolation); a frustum-only crop (kept as the fallback, but its walls are view-dependent).

## 6. Parameters and tuning

| Name | Default | Location | Effect |
|---|---|---|---|
| `UNDECODED_MATCH_FRACTION` | 0.35 | line 104 | Max distance (× square side) for an undecoded square to fill a missing slot |
| `MIN_AUTO_CROP_POINTS` | 1000 | 106 | Auto crop rejected below this |
| `MAX_GRID_RESIDUAL_PX` | 0.15 px | 109 | Intrinsics/cloud mismatch threshold |
| `PLANE_BAND_PX` | 25 px | 112 | Width (vertex shift) of the band used to fit the bed |
| `PLANE_INLIER_DIST_M` | 0.005 m | 113 | RANSAC inlier distance |
| `MIN_PLANE_POINTS` | 500 | 114 | Min sample/inliers for a plane |
| RANSAC `ransac_n`, iterations | 3, 500 | 658 | Open3D `segment_plane` |
| `TAG_INNER_SCALE` | 0.7 | 764 | Fraction of each tag used for its plane (sidecar) |
| `TAG_MIN_PLANE_POINTS` | 30 | 765 | Min points per tag plane |
| `TAG_PLANE_INLIER_M` | 0.003 m | 766 | Inlier floor for tag plane refit |
| `_undistort` iterations | 20 | 298 | Fixed-point inversion |
| intrinsics sample | ≤ 50 000 pts | 341 | Grid-residual test size |
| `--frame-window` | 15 frames | 1143-1149 | Temporal window for missing tags |
| `--margin` | 0 px | 1150-1156 | Inward shrink of the quad (bulk config uses 10 px) |
| `BoxCropper.step` | 2 % of max extent | 954 | Manual move/resize step |
| `BoxCropper.min_size` | 1 % of extent per axis | 955 | Minimum box size |
| default manual box | 50 % of cloud AABB, centred | 902-907 | Used when nothing picked and no auto box |

## 7. Assumptions, limitations, failure modes, edge cases

* **Static camera during a recording** (used for both the ±15-frame window and the median corners). Motion would mix tag positions.
* **Exactly four corner tags** around the region; layout learning needs at least one frame with exactly four decoded tags (`len(decoded) == 4`, line 441). A fifth decoded tag in all frames would prevent learning.
* **Square/rectangular, convex target region in the image.** Aspect ratios are learned in *image* space and pooled over all scans of the session, but perspective changes with tilt/rotation, so extrapolation from two tags in a strongly tilted view can be biased **[INFERRED]**.
* **Planar bed in the 25-px band.** If the object's relief reaches the edge of the square, the RANSAC band plane can be tilted or fit to part of the object, tilting the prism walls **[INFERRED]** (see 08 for the Small_Wave case).
* **Depth must exist under the tags** for the tag plane and sidecar; otherwise the tag plane falls back to the surface plane (tags assumed at bed height) and the sidecar skips the tag.
* **Intrinsics only from uncompressed bags**; compressed chunks are skipped (252). Any `.db3` in the session is accepted as a fallback (272), assuming identical stream settings.
* **Frame index alignment**: the IMU CSV's `frame` must use the same numbering as the detections' `frame` column; not verifiable from this code alone.
* **Prism has no height limit** (677-678): flying pixels above/below the bed inside the column are kept.
* Fallback frustum crop (743-749) is view-dependent; mixing frustum- and prism-cropped scans in one session gives inconsistent boundaries **[INFERRED]**.
* `segment_plane` is randomised: repeated runs can give slightly different crops (no seed set) **[INFERRED]**.
* Manual crop boxes are axis-aligned in the PLY frame, which is not aligned with the bed when the camera is tilted.

## 8. Dependencies

| Dependency | Calls and what they do |
|---|---|
| **Open3D** (`o3d`) | `io.read_point_cloud` (legacy reader, float64 internally); `t.geometry.PointCloud` + `t.io.write_point_cloud` (tensor API, preserves float32/uint8 dtypes); `geometry.PointCloud.segment_plane` (RANSAC plane); `select_by_index`, `crop(AxisAlignedBoundingBox)`, `get_axis_aligned_bounding_box`; `LineSet` (wireframe); `visualization.VisualizerWithKeyCallback` (GLFW key callbacks), `VisualizerWithEditing` (Shift+click point picking; `get_picked_points` returns ints or `PickedPoint` objects depending on version, handled at 910-917) |
| **NumPy** | vector maths, SVD (`np.linalg.svd`), medians, masks |
| stdlib | `struct` (binary bag parsing), `csv`, `re` (tilt from folder name `_a(-?\d+)$`, line 133), `shutil.copy2`, `argparse`, `pathlib`, `collections.Counter/defaultdict`, `dataclasses` |
| Internal producers | `rs_marker` (PLY, detections CSV, IMU CSV, `.db3` recording, `scan_info.txt`) |
| Internal consumers | `Fuse` (`fuse_scans` reads `*_tags.csv`, `imu_a*.csv`, `scan_info.txt`, `*_detections.csv`), `Bulk_Process_Pipeline` (imports this module) |

The tool runs in the venv at `Processing/Crop/cropper/` (referenced by `Bulk_Process_Pipeline/README.md:21`); no `requirements.txt` exists in `Processing/Crop` (cannot determine pinned versions).

## 9. `rename_ply.sh`

`find "$TARGET_DIR" -type f -name "*_*_r*_a*_cropped.ply" -print0` piped to a `while read -d $'\0'` loop renames each file to `${file%_cropped.ply}.ply` with `mv` and prints the change (lines 7-16). Purpose (**[INFERRED]**): `fuse_scans` prefers a PLY named exactly `<tilt folder>.ply` (`Fuse/src/scan_discovery.cpp:21-28`); renaming makes cropped trees look like raw capture trees. It is not strictly required, since discovery falls back to the single `.ply` in a folder. No dry-run, no overwrite guard: if `<name>.ply` already exists, `mv` silently overwrites it.

## 10. Observations

1. **Two implementations of the same tag geometry diverge.** The sidecar code says it *"Mirrors locateTags() in Fuse/src/tag_alignment.cpp - keep the two in step"* (760), but Python takes intrinsics from the `.db3` bag and auto-selects distortion direction (`match_intrinsics_to_cloud`), whereas C++ reads `scan_info.txt` and always applies forward distortion (`tag_alignment.cpp:24-33, 262-312`). The geometric method (median corners, 0.7 inner quad, two-pass plane, ray–plane corners) agrees, but the intrinsics source and the "inverse/none" distortion options do not, and the Python path additionally rejects clouds failing the 0.15 px grid test while C++ has no such check. Duplicated maths across languages is a maintenance risk.
2. **Different temporal treatment of corners.** The crop quad uses single-frame corners (nearest frame per tag, 477-483); the sidecar uses the all-frame median (769-777). The crop boundary therefore carries detector jitter that the sidecar does not.
3. **`shrink_quad` moves vertices along the diagonal, not edges perpendicularly** (644-648). For a square, a "25 px" band is about $25/\sqrt2\approx17.7$ px wide; `--margin` likewise moves edges by ≈ margin/√2. Docstring/help text describe it as pixels "in from the tags' inner corners", which is literally true for the corners but not for the edges.
4. **Learned aspect pooled across all poses** (456-469) while perspective varies with tilt and rotation; extrapolated corners for strongly tilted scans may be off **[INFERRED]**.
5. **No exception guard beyond `AutoCropError`.** e.g. `read_imu_frame` raises `KeyError`/`ValueError` if the CSV lacks a `frame` column (402); `find_color_intrinsics` catches parse errors but other unexpected errors abort the whole batch.
6. **`collect_tags` de-duplicates undecoded squares against decoded ones from *all* window frames** — fine — but decoded tags keep only the nearest-frame instance even if that instance was a partial/poor detection.
7. **Fallback frustum crop silently changes semantics** (743-749); the preview note mentions it but `--auto-accept` (and the bulk pipeline, which never shows the preview by default) would save it without a human noticing.
8. **No height limit on the prism** (676-683): outliers directly above/below the square survive, and the fuser's VoxelGrid/height-field steps then see a taller bounding box (relevant to 07/08).
9. **`segment_plane` without a fixed seed** → non-deterministic crops across runs **[INFERRED]**; the bulk pipeline caches crops, partly masking this.
10. `--tags-only` writes sidecars only for clouds found in `input_dir`, and assumes the output root mirrors an earlier crop run (README 513-515); mismatched roots produce sidecars in folders without cropped clouds.
11. `rename_ply.sh` has no collision check; and its glob `*_*_r*_a*_cropped.ply` would also match unrelated names containing `_r`/`_a` fragments.
12. `.pyc` files for three interpreter versions are committed under `Processing/Crop/__pycache__/` (commits 473583a, 00b2c4e, 713bf4b) — build artefacts in version control.
13. Docstring claims the `.db3` files are the recordings; they are ROS1 bags with a non-standard extension. Correctly handled, but surprising for any other tool that assumes ROS2 `.db3` (SQLite) **[INFERRED]**.
