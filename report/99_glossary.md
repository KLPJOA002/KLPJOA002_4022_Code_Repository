# 99 — Glossary of terms and symbols

## A. Hardware, sensors and file formats

| Term | Meaning in this project |
|---|---|
| **D456** | Intel RealSense D456: an active-IR stereo depth camera with an RGB camera and an IMU (BMI085 accelerometer + gyro). It is IP65-rated, with a ~95 mm baseline. |
| **Rig / mount** | A servo-driven tilt head (Arduino over serial, `mountSerial.cpp`) plus a manually stepped rotation (nominal +45° per step) |
| **Tilt (`a<deg>`)** | Servo tilt angle of one scan; folder suffix `_a<deg>` |
| **Rotation (`r<deg>`)** | Nominal rig rotation; folder suffix `_r<deg>`. It is not measured. |
| **Session** | One run of rs_marker: `<YYYYMMDD_HHMMSS>/` containing all rotation and tilt leaves |
| **Leaf / scan** | One `<TS>_r<r>_a<a>/` folder: one capture at one pose |
| **Best frame** | The frame among the `-n` captured frames with the most valid depth pixels (inside the optional `-b` window). It is exported as the PLY. |
| **`.bag`** | ROS1 bag: the librealsense recording format before 2.57 |
| **`.db3`** | Normally a ROS2 SQLite bag. **Here, the files are ROS1 bag content with a `.db3` name** (`Processing/Filter_Sweep/decoder/rs_decode.cpp:24`). |
| **PLY** | Polygon File Format. rs_marker PLYs hold float32 xyz in metres, uint8 rgb, stored as (x, −y, −z). |
| **`scan_info.txt`** | Per-scan `key: value` metadata from rs_marker: options, intrinsics, results |
| **`trial_info.txt`** | Per-trial metadata from depth_quality |
| **`_detections.csv`** | Per-frame AprilTag detections from rs_marker |
| **`_tags.csv`** | Crop sidecar: 3D tag corners on the uncropped cloud, used by `fuse_scans --tag-align` |
| **Depth units** | Size of one raw depth count. Set to 0.0001 m (0.1 mm) here; the librealsense default is 1 mm. |
| **Preset** | RealSense visual preset, e.g. High Density or High Accuracy, that bundles stereo-matching thresholds |
| **Laser / emitter power** | IR dot-projector power (mW). It adds texture for stereo matching. |
| **Thermal compensation** | D4xx option that periodically recalibrates for temperature drift; disabled here (reason unknown) |
| **HDR merge** | librealsense filter merging alternating exposures; present in Filter_Sweep but unused (no HDR recordings) |

## B. Vision and geometry

| Term | Meaning |
|---|---|
| **Optical frame** | Camera frame with x right, y down, z forward (into the scene) |
| **Intrinsics $(f_x, f_y, c_x, c_y)$** | Pinhole focal lengths and principal point in pixels, plus a distortion model and coefficients |
| **Deprojection** | Pixel $(u,v)$ + depth $Z$ → 3D point: $X = (u-c_x)Z/f_x$, $Y = (v-c_y)Z/f_y$ (before distortion correction) |
| **Extrinsics** | Rigid transform $[R\,|\,t]$ between two sensors, e.g. depth → colour |
| **Align (depth → colour)** | Re-projecting depth pixels into the colour camera so each colour pixel gets a depth value |
| **Disparity $d$** | Horizontal pixel shift between the stereo images: $d = fB/Z$ |
| **Baseline $B$** | Distance between the stereo IR imagers |
| **Sub-pixel RMS** | RMS plane-fit residual in disparity pixels, $\sqrt{\overline{(fB/Z - fB/Z_\text{fit})^2}}$; Intel's measure of stereo matching precision |
| **Fill rate** | Fraction of ROI pixels with valid (non-zero) depth |
| **Depth RMS error** | Expected to grow as $\sigma_Z \approx Z^2 \sigma_d /(fB)$ |
| **AprilTag 36h11** | Fiducial family: 6×6 data bits, minimum Hamming distance 11 (Olson 2011; Wang & Olson 2016). It is decoded here with OpenCV's ArUco module. |
| **Candidate (`marker_id = -1`)** | A quad that passes shape and contrast checks but fails ID decoding |
| **CLAHE** | Contrast-Limited Adaptive Histogram Equalisation (Zuiderveld 1994); present in rs_marker but disabled |
| **ROI** | Region of interest |
| **Bed plane** | Plane fitted to the support surface near the tags; Crop uses it to define the crop prism |

## C. Point-cloud processing and registration

| Term | Meaning |
|---|---|
| **Voxel downsampling** | Replace all points in each cubic cell of side $v$ by their centroid (PCL `VoxelGrid`, Open3D `voxel_down_sample`) |
| **SOR** | Statistical Outlier Removal. A point is dropped if its mean $k$-NN distance exceeds $\mu + \alpha\sigma$. |
| **Normals (sensor-facing)** | PCA normal of the $k$-neighbourhood, flipped to point toward the sensor origin |
| **FPFH** | Fast Point Feature Histogram (Rusu et al. 2009): a local descriptor built from normal-angle statistics |
| **RANSAC** | Random Sample Consensus (Fischler & Bolles 1981). It is used on FPFH correspondences to estimate a coarse rigid pose. |
| **ICP** | Iterative Closest Point (Besl & McKay 1992). **Point-to-plane** ICP (Chen & Medioni 1992) minimises $\sum ((R p_i + t - q_i)\cdot n_i)^2$. |
| **Fitness / overlap** | Fraction of source points with a correspondence within the threshold |
| **Inlier RMSE** | RMS distance of the corresponding pairs |
| **Kabsch / Umeyama** | Closed-form least-squares rigid (± scale) transform between paired points, via SVD |
| **Rig seed** | Initial pose computed from the nominal tilt/rotation angles and the rig axis geometry |
| **Tag alignment** | A pose from matching 3D AprilTag corners between scans (`--tag-align`) |
| **Hypothesis scoring** | Competing initial poses are each refined and scored; the best wins. A margin under 15% is flagged *ambiguous*. |
| **Accumulated map** | Union, in voxel form, of scans already aligned; each new scan registers to it |
| **Fusion cell** | Grid cell in which overlapping points from different scans are merged (weighted, trimmed mean) |
| **MLS** | Moving Least Squares smoothing: project each point onto a locally fitted polynomial surface |
| **Tilt pivot calibration** | Grid search for the tilt axis position relative to the camera |
| **Gravity vector** | Accelerometer reading at rest, used to measure tilt; magnitudes outside 9.0–10.6 m/s² are rejected |

## D. Evaluation metrics (Evaluate, sweeps)

| Symbol / name | Meaning |
|---|---|
| $d_i$ | Unsigned distance from scan point $i$ to the reference surface |
| **acc_rmse_mm** | $\sqrt{\tfrac1N\sum d_i^2}$, in mm; the main metric in the sweeps |
| **MAE, median, p90/p95/p99, max** | Statistics of $d_i$ |
| **Signed bias / std** | Mean and standard deviation of the signed distance (positive = outside / above) |
| **Trimmed** | Statistics after dropping the worst fraction of points |
| **Coverage / completeness** | Fraction of reference samples with a scan point within a radius |
| **Precision / recall / F-score @ τ** | Fraction of scan points within τ of the reference / of reference points within τ of the scan / their harmonic mean (Knapitsch et al. 2017, Tanks & Temples style), for τ = 0.5, 1, 2, 5 mm |
| **Chamfer** | Mean of the two directional nearest-neighbour distances |
| **Hausdorff** | Max of the two directional maxima. Here the reference side is capped at 10 mm. |
| **Suspect** | Sweep flag set when covered fraction < 0.85 |
| **Spearman ρ** | Rank correlation of configuration rankings between objects |
| **Height field / parametric / implicit reference** | $z = f(x,y)$; $\mathbf r(u,v)$; $F(x,y,z)=0$ |
| **Newton / foot-point projection** | Iterative closest-point-on-surface solve for exact $d_i$ |

## E. Roughness (Rugosity_Metrics; ISO 25178-2 areal parameters)

| Symbol | Definition |
|---|---|
| $z(x,y)$ | Height after levelling and detrending, on a regular grid |
| **Sa** | Arithmetic mean height: $\frac1A\iint |z|\,dA$ |
| **Sq** | Root-mean-square height: $\sqrt{\frac1A\iint z^2\,dA}$ |
| **Sp, Sv, Sz** | Max peak height, max pit depth (reported positive), $S_z = S_p + S_v$ |
| **Ssk** | Skewness: $\frac{1}{S_q^3}\overline{z^3}$ |
| **Sku** | Kurtosis: $\frac{1}{S_q^4}\overline{z^4}$ (3 for Gaussian) |
| **Sdq** | RMS gradient: $\sqrt{\overline{z_x^2 + z_y^2}}$ |
| **Sdr** | Developed interfacial area ratio: $(A_\text{true}/A_\text{proj} - 1)\times100\%$ |
| **Rugosity** | $A_\text{true}/A_\text{proj}$ (two triangles per grid quad); `Sdr = (rugosity−1)·100` |
| **ACF** | Areal autocorrelation function |
| **Sal** | Autocorrelation length: shortest lag at which the ACF decays to $s = 0.2$ |
| **Str** | Texture aspect ratio: Sal divided by the slowest decay length (1 = isotropic) |
| **Std** | Texture direction (lay); here the angular-spectrum peak + 90° |
| **PSD** | Power spectral density $C(q)$ of $z$; the radially averaged form is fitted as $C(q)\propto q^{-2(1+H)}$ |
| **H, D** | Hurst exponent and fractal dimension, $D = 3 - H$ |
| **Dominant wavelength / direction** | 2D PSD peak; direction is that of the wave vector |
| **Detrend (degree 1–4)** | Least-squares polynomial surface removed with MAD-based sigma clipping |
| **MAD** | Median absolute deviation; robust σ ≈ 1.4826·MAD |
| **Multiscale (`ms_*`)** | Sq and rugosity in windows from 4 cells up to half the footprint |
| **Bootstrap CI95** | 95% confidence interval from resampling windows |

## F. Project-specific abbreviations

| Abbrev. | Meaning |
|---|---|
| **DQT** | Intel's Depth Quality Tool; `depth_quality` follows its style |
| **GT** | Ground truth |
| **TS** | Timestamp `YYYYMMDD_HHMMSS` |
| **SOR / FPFH / ICP / MLS** | See section C |
| **Big_/Small_Wave, Big_Pyramid** | Machined or printed test objects (STL references) |
| **641 mm … 933 mm** | Camera heights in the 3D_Error data set |
