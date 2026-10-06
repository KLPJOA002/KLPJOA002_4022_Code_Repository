#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <unordered_map>

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Switch this typedef to pcl::PointXYZ if your PLY scans don't carry color.
using PointT      = pcl::PointXYZRGB;
using PointCloudT = pcl::PointCloud<PointT>;
using PointNT     = pcl::PointXYZRGBNormal;
using PointCloudNT = pcl::PointCloud<PointNT>;
using NormalCloudT = pcl::PointCloud<pcl::Normal>;

// One input scan: its PLY path, and (optionally) a known/approximate
// rig transform that carries it into a common reference frame.
// If has_initial_guess is false, the pipeline estimates a coarse
// alignment itself via feature matching before refining with ICP.
struct ScanInput
{
    std::string     ply_path;
    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
    bool            has_initial_guess = false;

    // Optional metadata populated by folder-based discovery. Not required
    // for fusion itself, but used for logging, ordering, grouping and
    // visualizer labels. label() gives a human-readable identifier for a
    // scan whichever fields happen to be populated.
    std::string session_name;
    std::string subfolder_name;

    // The rotation subfolder this scan came from, e.g. "20260909_1430_r45".
    // Scans sharing a rotation group were captured at one physical mount
    // position, which is the unit the rotation-direction test works on.
    std::string rotation_group;
    int         rotation_index = 0;     // 0-based ordinal of the group within its session
    double      rotation_degrees = 0.0; // magnitude as recorded by the folder name
    bool        has_rotation = false;

    int         tilt_degrees = 0;
    bool        has_tilt = false;

    // Accelerometer reading captured alongside this scan's best frame, in
    // the camera's own frame (m/s^2). While stationary this is purely
    // gravity, which pins down two of the three rotation DOF directly:
    // tilting the camera swings gravity across the frame by exactly the
    // tilt angle, about exactly the tilt axis. That makes it a far better
    // source for the tilt seed than the commanded angle, which assumes the
    // mount went where it was told and that the hinge sits on a nominated
    // camera axis. (It says nothing about the turntable: rotation is about
    // gravity itself, which leaves gravity unchanged.)
    Eigen::Vector3f gravity = Eigen::Vector3f::Zero();
    bool            has_gravity = false;

    std::string label() const
    {
        if (!session_name.empty() && !subfolder_name.empty())
            return session_name + "/" + subfolder_name;
        return ply_path;
    }

    // Compact identifier for the legend panel, where a full label would
    // not fit: the tilt folder's name with the redundant session-name
    // prefix stripped, e.g. "r45_a-5". Falls back to the PLY's filename.
    std::string shortLabel() const;
};

struct RegistrationResult
{
    Eigen::Matrix4f transform      = Eigen::Matrix4f::Identity();
    bool            converged      = false;

    // PCL counts running out of iterations as converging, so `converged`
    // alone cannot tell a scan that settled from one still moving when ICP
    // gave up. This can.
    bool            hit_max_iterations = false;

    // ICP: mean squared distance (m^2) over the correspondences ICP itself
    // would accept. Feature matching: fraction of the source that is not a
    // RANSAC inlier.
    double          fitness_score  = std::numeric_limits<double>::max();
};

struct PipelineParams
{
    // Preprocessing
    float voxel_leaf_size          = 0.003f;   // metres. D455 noise floor is a few mm at close range.
    int   sor_mean_k               = 30;       // statistical outlier removal neighbourhood
    double sor_stddev_mult         = 1.0;

    // Feature-based coarse (global) alignment - only used when a scan
    // has no initial_guess supplied.
    float normal_radius            = 0.015f;
    float fpfh_radius              = 0.03f;
    float sac_max_correspondence   = 0.05f;
    int   sac_max_iterations       = 50000;
    int   sac_nr_samples           = 3;

    // Fine alignment (point-to-plane ICP)
    float icp_max_correspondence_distance = 0.02f;
    int   icp_max_iterations              = 60;
    double icp_transformation_epsilon     = 1e-8;
    double icp_euclidean_fitness_epsilon  = 1e-6;

    // Review flag. A flagged scan is still fused - this only marks it for a
    // look. A refined scan is flagged when ICP hit its iteration cap, or when
    // its alignment score is more than this many times the batch median.
    // Relative rather than absolute, because what a good score looks like
    // depends on the voxel size, the sensor's noise and the scene's relief.
    double flag_score_ratio               = 1.4;

    // Alignment scoring (evaluateAlignment). Correspondence distance for
    // judging a transform after the fact; kept separate from ICP's own so
    // scoring stays comparable across coarse and fine stages.
    float eval_correspondence_distance = 0.01f;
    int   eval_max_samples             = 20000; // stride-sample larger clouds for speed

    // A coarse alignment covering less of the cloud than this is treated
    // as failed rather than handed to ICP - ICP cannot recover from a seed
    // that has essentially no overlap with the map.
    double coarse_min_inlier_fraction = 0.15;

    // --- AprilTag coarse alignment (see tag_alignment.h) ----------------
    // When on, each scan's tags are located at load time, and coarse
    // alignment adds a third hypothesis: the pose that puts this scan's tags
    // on the same tags as already placed by earlier scans. It is scored like
    // the others and only wins if it actually fits the surface better.
    bool   use_tag_alignment = false;

    // Tags two scans must share before a tag pose is attempted. One tag's
    // four corners already fix all six DOF; two make the rotation far better
    // conditioned (the lever arm becomes the tag spacing, not the tag size).
    int    tag_min_shared = 1;

    // A tag fit whose corners disagree by more than this (rms, metres) after
    // fitting is discarded: something is wrong with a detection or its depth.
    // With the tilt taken from the IMU, a residual of a few mm is normal - it
    // is the IMU's tilt error spread over the ~600 mm between tags.
    double tag_max_fit_rmse = 0.010;

    // Tag and geometric poses further apart than this are not a question of
    // precision: one of them is wrong outright, usually a rotation group the
    // direction stage resolved the wrong way. Logged loudly when it happens.
    double tag_disagreement_warn_deg = 10.0;

    // How far from the camera's optical centre to look for the tilt axis,
    // in metres. Keep this near the physical truth: a rig whose tilt axis
    // sits close to the camera face needs only a small range, and searching
    // far beyond it both wastes time and lets the search wander to a large
    // spurious pivot on a score surface that is nearly flat when the real
    // lever arm is short.
    float tilt_pivot_search_range = 0.12f;

    // Cloud resolution used for the calibration searches. Much coarser than
    // the registration voxel: these searches only need the shape of the
    // surface, and run thousands of evaluations.
    float calibration_voxel_size = 0.005f;

    // --- Tilt geometry calibration ------------------------------------
    // The camera is mounted off the tilt head's pivot, so a seed that
    // rotates it about the origin is wrong by an amount that grows with
    // sin(tilt). Rather than make you measure the lever arm with a ruler,
    // the calibration stage aligns the few lowest-tilt scans (where the
    // error is small enough that the seed still works) and solves for the
    // pivot that explains how they actually moved.
    bool  calibrate_tilt_geometry = true;

    // Only scans at or below this tilt are used as calibration input -
    // beyond it the seed is too poor to align reliably, which is the very
    // problem being fixed.
    double tilt_calibration_max_deg = 15.0;

    // Upper bound on how many scans to spend on calibration.
    int   tilt_calibration_max_scans = 6;

    // --- Rotation direction inference ---------------------------------
    // The capture rig records every rotation as a positive increment
    // regardless of which way it was physically turned, and nothing in the
    // captured data resolves that: rotation is about the gravity axis, so
    // the IMU's accelerometer (which only ever measures gravity) is
    // invariant under it, and the recorder is stopped while the mount is
    // turned by hand, so there is no gyro trace to integrate either. What
    // *is* decisive is the geometry: a 45-degree error in the wrong
    // direction is a 90-degree error overall, which collapses the overlap
    // between the scan and the map. So each rotation step's direction is
    // chosen by trying both and keeping whichever actually fits.
    bool  infer_rotation_direction = true;

    // Short ICP run applied to each candidate before scoring it. Off by
    // default, and that is deliberate: on a near-planar scene (sand) ICP
    // has three barely-constrained degrees of freedom, so letting a *wrong*
    // direction settle lets it slide into a plausible-looking fit and beat
    // the right one. Scoring the raw seeds keeps the comparison honest.
    // Raise it only if your scene has strong 3D structure and the rig
    // geometry is rough.
    int   direction_test_icp_iterations = 0;

    // Search every combination of per-step directions and keep the set that
    // fits best overall, rather than fixing each step greedily before
    // looking at the next. On a planar scene an individual step is often a
    // coin toss, while the whole sequence is not - a wrong early step makes
    // every later one fit worse, which only a joint search can see. Falls
    // back to the greedy per-step choice beyond this many steps, where
    // 2^steps combinations stops being affordable.
    int   direction_max_joint_steps = 12;

    // Relative gap between the two candidates' scores, below which the
    // result is flagged as ambiguous rather than trusted silently.
    double direction_ambiguous_margin = 0.15;

    // --- Height-field scoring (planar scenes) --------------------------
    // Cell size for the height grid, in metres. Wants to be a few times the
    // point spacing so each cell averages several points, but well below
    // the scale of the relief being matched.
    float plane_grid_cell = 0.01f;

    // A scene whose out-of-plane spread is below this fraction of its
    // in-plane spread is treated as a height field, and alignment scoring
    // switches to the plane-aware metric.
    double planarity_threshold = 0.15;

    // Merged-map maintenance
    bool  downsample_accumulated_map = true;   // keep the running merged cloud from growing unbounded

    // Final output
    float final_voxel_leaf_size    = 0.002f;
    bool  run_final_outlier_removal = true;
    bool  estimate_output_normals   = true;
    float output_normal_radius      = 0.015f;
};

// ---- Building blocks -------------------------------------------------

PointCloudT::Ptr loadPLY(const std::string& path);

// Axis-aligned crop in the cloud's own frame. Returns the input unchanged
// (a copy) if the box would leave nothing behind.
PointCloudT::Ptr cropCloud(const PointCloudT::ConstPtr& input,
                            const Eigen::Vector4f& min_pt,
                            const Eigen::Vector4f& max_pt);

PointCloudT::Ptr preprocessCloud(const PointCloudT::ConstPtr& input,
                                  const PipelineParams& params);

RegistrationResult coarseAlignFPFH(const PointCloudT::ConstPtr& source,
                                    const PointCloudT::ConstPtr& target,
                                    const PipelineParams& params);

// Surface normals for one scan in its own camera frame, oriented toward the
// sensor at the origin. Estimated once at load: a rigid transform carries a
// normal along with its point, so a placed scan never needs them re-estimated.
NormalCloudT::Ptr estimateSensorNormals(const PointCloudT::ConstPtr& cloud, float radius);

// Point-to-plane ICP. Both clouds arrive with their normals attached (see
// estimateSensorNormals and VoxelMap), so this estimates none itself.
RegistrationResult refineICP(const PointCloudNT::ConstPtr& source,
                              const PointCloudNT::ConstPtr& target,
                              const Eigen::Matrix4f& initial_guess,
                              const PipelineParams& params);

PointCloudT::Ptr voxelDownsample(const PointCloudT::ConstPtr& input, float leaf_size);

// The map a scan registers against: every scan placed so far, merged into
// voxels and built up incrementally as scans are added.
//
// Each voxel keeps running sums, so it holds the true centroid of every point
// that ever landed in it, plus the mean colour and normal. That buys two
// things over re-voxelising the whole map after each scan. Adding a scan costs
// time proportional to that scan rather than to the map. And every scan gets
// equal weight: re-voxelising a map of centroids counts each earlier centroid
// as a single point, which hands the newest scan about half the weight
// wherever it overlaps - so the map chased the latest scan instead of holding
// the consensus of all of them.
class VoxelMap
{
public:
    // leaf_size <= 0 keeps every point as its own entry (no merging).
    explicit VoxelMap(float leaf_size);

    // Adds `cloud` carried into the map frame by `transform`. `normals`, if
    // given, holds one normal per point in the same frame as `cloud`.
    void add(const PointCloudT& cloud, const NormalCloudT* normals,
             const Eigen::Matrix4f& transform);

    bool   empty() const { return cells_.empty(); }
    size_t size() const  { return cells_.size(); }

    PointCloudT::Ptr cloud() const;

    // Voxels no finite normal reached get NaN normals, which point-to-plane
    // ICP skips.
    PointCloudNT::Ptr cloudWithNormals() const;

private:
    struct Cell
    {
        double   x = 0.0, y = 0.0, z = 0.0;
        float    nx = 0.0f, ny = 0.0f, nz = 0.0f;
        float    r = 0.0f, g = 0.0f, b = 0.0f;
        uint32_t count = 0;
        uint32_t normal_count = 0;
    };

    float                                  leaf_size_;
    std::vector<Cell>                      cells_;  // insertion order, so output order is deterministic
    std::unordered_map<uint64_t, uint32_t> index_;  // voxel key -> index into cells_
};

struct AlignmentQuality; // defined in scan_state.h

// A dominant plane fitted to a cloud, with an orthonormal frame on it.
// A sand surface scanned from above is essentially a height field, and that
// changes what "well aligned" even means - see evaluateAlignmentOnPlane.
struct PlaneFrame
{
    bool            valid = false;
    Eigen::Vector3f origin = Eigen::Vector3f::Zero();

    // Rows are the in-plane axes u, v and the plane normal n. Multiplying a
    // world point (minus origin) by this gives (lateral_u, lateral_v, height).
    Eigen::Matrix3f basis = Eigen::Matrix3f::Identity();

    // Out-of-plane spread divided by in-plane spread. Near zero means a
    // flat sheet, where nearest-neighbour scoring stops discriminating.
    double planarity = 1.0;

    bool isPlanar(double threshold) const { return valid && planarity < threshold; }
};

PlaneFrame estimatePlaneFrame(const PointCloudT::ConstPtr& cloud);

// A cloud binned into cells on a plane, holding each cell's mean height.
// Kept as its own type so a fixed target can be gridded once and reused
// across many candidate transforms - rebuilding it per evaluation made the
// calibration searches hundreds of times more expensive than they needed
// to be.
struct HeightGrid
{
    std::unordered_map<long long, float> cells;
    float cell_size = 0.01f;

    // Cells holding fewer points than this are dropped: one stray point's
    // height is noise, not surface. Must be 1 for an already-voxelised cloud,
    // where each point *is* a local average and cells legitimately hold only
    // one or two - requiring more there silently empties the grid.
    int min_points_per_cell = 3;

    bool empty() const { return cells.empty(); }
};

HeightGrid buildHeightGrid(const PointCloudT& cloud, const PlaneFrame& plane,
                            float cell_size, int min_points_per_cell = 3);

// Scores `source` under `transform` against an already-built target grid.
AlignmentQuality evaluateAgainstHeightGrid(const PointCloudT::ConstPtr& source,
                                             const Eigen::Matrix4f& transform,
                                             const PlaneFrame& plane,
                                             const HeightGrid& target,
                                             const PipelineParams& params);

// Scores how well `source` sits on `target` once `transform` is applied,
// independently of whichever algorithm produced that transform. This is
// what makes competing hypotheses (rig seed vs. mirrored rig seed vs.
// feature match) directly comparable - see AlignmentQuality::score().
AlignmentQuality evaluateAlignment(const PointCloudT::ConstPtr& source,
                                    const PointCloudT::ConstPtr& target,
                                    const Eigen::Matrix4f& transform,
                                    const PipelineParams& params);

// Scores an alignment as a height field rather than a point cloud: both
// clouds are binned into cells on the plane and their mean heights compared
// cell by cell.
//
// This exists because evaluateAlignment cannot see in-plane error on a flat
// scene. The nearest point to a sheet of points is close no matter how you
// slide or spin the sheet within its own plane, so a badly rotated scan
// still scores a small rmse and a high overlap. Comparing heights at the
// *same lateral position* removes that blindness: get the rotation wrong and
// the relief no longer lines up, which is immediately visible here.
AlignmentQuality evaluateAlignmentOnPlane(const PointCloudT::ConstPtr& source,
                                            const PointCloudT::ConstPtr& target,
                                            const Eigen::Matrix4f& transform,
                                            const PlaneFrame& plane,
                                            const PipelineParams& params);

// Final cleanup pass: outlier removal, uniform voxel resample, normals.
PointCloudNT::Ptr postProcessMerged(const PointCloudT::ConstPtr& merged,
                                     const PipelineParams& params);

void savePLY(const std::string& path, const PointCloudT::ConstPtr& cloud);
void savePLY(const std::string& path, const PointCloudNT::ConstPtr& cloud);
