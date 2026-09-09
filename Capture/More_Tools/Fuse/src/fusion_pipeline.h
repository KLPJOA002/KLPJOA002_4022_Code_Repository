#pragma once

#include <functional>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Switch this typedef to pcl::PointXYZ if your PLY scans don't carry color.
using PointT      = pcl::PointXYZRGB;
using PointCloudT = pcl::PointCloud<PointT>;
using PointNT     = pcl::PointXYZRGBNormal;
using PointCloudNT = pcl::PointCloud<PointNT>;

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
    // for fusion itself, but used for logging, ordering, and visualizer
    // labels. label() gives a human-readable identifier for a scan
    // whichever fields happen to be populated.
    std::string session_name;
    std::string subfolder_name;
    int         tilt_degrees = 0;
    bool        has_tilt = false;

    std::string label() const
    {
        if (!session_name.empty() && !subfolder_name.empty())
            return session_name + "/" + subfolder_name;
        return ply_path;
    }
};

struct RegistrationResult
{
    Eigen::Matrix4f transform      = Eigen::Matrix4f::Identity();
    bool            converged      = false;
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
    double icp_fitness_reject_threshold   = 0.0015; // reject a scan's alignment above this

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

PointCloudT::Ptr preprocessCloud(const PointCloudT::ConstPtr& input,
                                  const PipelineParams& params);

RegistrationResult coarseAlignFPFH(const PointCloudT::ConstPtr& source,
                                    const PointCloudT::ConstPtr& target,
                                    const PipelineParams& params);

RegistrationResult refineICP(const PointCloudT::ConstPtr& source,
                              const PointCloudT::ConstPtr& target,
                              const Eigen::Matrix4f& initial_guess,
                              const PipelineParams& params);

PointCloudT::Ptr voxelDownsample(const PointCloudT::ConstPtr& input, float leaf_size);

// ---- Full pipeline -----------------------------------------------------

// What to do with the scan just shown at a FusionStepCallback stage.
// Not every stage honours every value - see the stage list below.
enum class StepDecision
{
    Accept,  // commit this result and move on to the next scan
    Retry,   // discard this attempt and redo the current scan from ROI
             // selection onward (coarse alignment is randomized, so a
             // retry on a scan with no known pose usually gives a
             // different result)
    Skip,    // leave this scan out of the fused cloud entirely, move on
    Undo,    // discard the *previous* scan's committed result and go
             // back to redo it (this scan's in-progress attempt, if
             // any, is discarded too)
};

// Called as fusion progresses so a caller (e.g. the visualizer) can show
// what's happening and decide how to proceed. `stage` is one of:
//   "reference"      - the first scan, seeding the common frame
//                       (only Accept/Retry are meaningful here)
//   "coarse"         - coarse alignment computed, not yet committed
//   "refined"        - ICP-refined result, about to be merged in
//   "flagged"        - refined result exceeded the fitness threshold but
//                       would still be merged (quality warning, not a skip)
//   "coarse_failed"  - coarse alignment did not converge
//                       (only Retry/Skip/Undo are meaningful here)
// `scan_cloud_in_common_frame` is the current scan transformed into the
// shared frame at this stage; `merged_so_far` is the running fused cloud
// *before* this scan is folded in. The returned StepDecision controls
// what fuseScans does next; a null callback behaves as if it always
// returned Accept.
using FusionStepCallback = std::function<StepDecision(
    size_t scan_index,
    const ScanInput& scan_input,
    const std::string& stage,
    const PointCloudT::ConstPtr& scan_cloud_in_common_frame,
    const PointCloudT::ConstPtr& merged_so_far,
    const RegistrationResult& result)>;

// Called once per scan (including the first) right after it's loaded from
// disk, before any preprocessing or alignment - lets a caller (e.g. the
// visualizer, via an interactive box widget) crop the raw cloud down to
// the region that actually matters for this scan. Cropping out background
// clutter both speeds up feature matching/ICP (fewer points) and often
// improves alignment quality by removing geometry that has no counterpart
// in the other scans. Return the cloud to use going forward (the input
// unchanged if no crop is wanted).
using RoiSelectionCallback = std::function<PointCloudT::Ptr(
    size_t scan_index,
    const ScanInput& scan_input,
    const PointCloudT::ConstPtr& raw_cloud)>;

// Registers and fuses every scan in `inputs` into a single point cloud.
// Each scan (after the first) is aligned against the *running merged
// cloud* rather than only its immediate predecessor - this reduces
// drift and gives ICP more surface to match against as fusion proceeds.
// Accepted scans are kept as separate clouds internally (not irreversibly
// blended) specifically so that `on_step` can return StepDecision::Undo
// to reopen and redo a previous scan.
PointCloudT::Ptr fuseScans(const std::vector<ScanInput>& inputs,
                            const PipelineParams& params,
                            std::vector<RegistrationResult>* per_scan_results = nullptr,
                            const FusionStepCallback& on_step = nullptr,
                            const RoiSelectionCallback& select_roi = nullptr);

// Final cleanup pass: outlier removal, uniform voxel resample, normals.
PointCloudNT::Ptr postProcessMerged(const PointCloudT::ConstPtr& merged,
                                     const PipelineParams& params);

void savePLY(const std::string& path, const PointCloudT::ConstPtr& cloud);
void savePLY(const std::string& path, const PointCloudNT::ConstPtr& cloud);
