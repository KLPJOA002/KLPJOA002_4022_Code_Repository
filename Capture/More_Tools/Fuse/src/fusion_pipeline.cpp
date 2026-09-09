#include "fusion_pipeline.h"

#include <iostream>
#include <stdexcept>

#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/icp.h>
#include <pcl/surface/mls.h>
#include <pcl/common/transforms.h>

using FeatureT = pcl::FPFHSignature33;
using FeatureCloudT = pcl::PointCloud<FeatureT>;
using NormalCloudT = pcl::PointCloud<pcl::Normal>;

// -------------------------------------------------------------------------
PointCloudT::Ptr loadPLY(const std::string& path)
{
    auto cloud = std::make_shared<PointCloudT>();
    if (pcl::io::loadPLYFile<PointT>(path, *cloud) < 0)
    {
        throw std::runtime_error("Failed to load PLY file: " + path);
    }
    if (cloud->empty())
    {
        throw std::runtime_error("Loaded PLY has zero points: " + path);
    }
    return cloud;
}

// -------------------------------------------------------------------------
PointCloudT::Ptr voxelDownsample(const PointCloudT::ConstPtr& input, float leaf_size)
{
    auto out = std::make_shared<PointCloudT>();
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(input);
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.filter(*out);
    return out;
}

// -------------------------------------------------------------------------
PointCloudT::Ptr preprocessCloud(const PointCloudT::ConstPtr& input,
                                  const PipelineParams& params)
{
    // Voxel downsample first - cheaper, and gives SOR a saner neighbourhood density.
    auto downsampled = voxelDownsample(input, params.voxel_leaf_size);

    auto cleaned = std::make_shared<PointCloudT>();
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(downsampled);
    sor.setMeanK(params.sor_mean_k);
    sor.setStddevMulThresh(params.sor_stddev_mult);
    sor.filter(*cleaned);

    return cleaned;
}

// -------------------------------------------------------------------------
static NormalCloudT::Ptr estimateNormals(const PointCloudT::ConstPtr& cloud, float radius)
{
    auto normals = std::make_shared<NormalCloudT>();
    pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(radius);
    auto tree = std::make_shared<pcl::search::KdTree<PointT>>();
    ne.setSearchMethod(tree);
    ne.compute(*normals);
    return normals;
}

static FeatureCloudT::Ptr computeFPFH(const PointCloudT::ConstPtr& cloud,
                                       const NormalCloudT::ConstPtr& normals,
                                       float radius)
{
    auto features = std::make_shared<FeatureCloudT>();
    pcl::FPFHEstimationOMP<PointT, pcl::Normal, FeatureT> fpfh;
    fpfh.setInputCloud(cloud);
    fpfh.setInputNormals(normals);
    auto tree = std::make_shared<pcl::search::KdTree<PointT>>();
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(radius);
    fpfh.compute(*features);
    return features;
}

// -------------------------------------------------------------------------
RegistrationResult coarseAlignFPFH(const PointCloudT::ConstPtr& source,
                                    const PointCloudT::ConstPtr& target,
                                    const PipelineParams& params)
{
    auto source_normals = estimateNormals(source, params.normal_radius);
    auto target_normals = estimateNormals(target, params.normal_radius);
    auto source_features = computeFPFH(source, source_normals, params.fpfh_radius);
    auto target_features = computeFPFH(target, target_normals, params.fpfh_radius);

    pcl::SampleConsensusPrerejective<PointT, PointT, FeatureT> sac;
    sac.setInputSource(source);
    sac.setSourceFeatures(source_features);
    sac.setInputTarget(target);
    sac.setTargetFeatures(target_features);
    sac.setMaximumIterations(params.sac_max_iterations);
    sac.setNumberOfSamples(params.sac_nr_samples);
    sac.setCorrespondenceRandomness(5);
    sac.setSimilarityThreshold(0.9f);
    sac.setMaxCorrespondenceDistance(params.sac_max_correspondence);
    sac.setInlierFraction(0.25f);

    PointCloudT aligned;
    sac.align(aligned);

    RegistrationResult result;
    result.converged = sac.hasConverged();
    result.transform = sac.getFinalTransformation();
    result.fitness_score = result.converged
        ? 1.0 - static_cast<double>(sac.getInliers().size()) /
                static_cast<double>(source->size())
        : std::numeric_limits<double>::max();
    return result;
}

// -------------------------------------------------------------------------
RegistrationResult refineICP(const PointCloudT::ConstPtr& source,
                              const PointCloudT::ConstPtr& target,
                              const Eigen::Matrix4f& initial_guess,
                              const PipelineParams& params)
{
    // Point-to-plane ICP converges faster and more accurately than
    // point-to-point on the roughly-continuous surfaces this rig scans,
    // so pull normals in for both clouds first.
    auto source_normals = estimateNormals(source, params.normal_radius);
    auto target_normals = estimateNormals(target, params.normal_radius);

    auto source_with_n = std::make_shared<PointCloudNT>();
    auto target_with_n = std::make_shared<PointCloudNT>();
    pcl::concatenateFields(*source, *source_normals, *source_with_n);
    pcl::concatenateFields(*target, *target_normals, *target_with_n);

    pcl::IterativeClosestPointWithNormals<PointNT, PointNT> icp;
    icp.setInputSource(source_with_n);
    icp.setInputTarget(target_with_n);
    icp.setMaxCorrespondenceDistance(params.icp_max_correspondence_distance);
    icp.setMaximumIterations(params.icp_max_iterations);
    icp.setTransformationEpsilon(params.icp_transformation_epsilon);
    icp.setEuclideanFitnessEpsilon(params.icp_euclidean_fitness_epsilon);

    PointCloudNT aligned;
    icp.align(aligned, initial_guess);

    RegistrationResult result;
    result.converged = icp.hasConverged();
    result.transform = icp.getFinalTransformation();
    result.fitness_score = icp.getFitnessScore();
    return result;
}

// -------------------------------------------------------------------------
PointCloudT::Ptr fuseScans(const std::vector<ScanInput>& inputs,
                            const PipelineParams& params,
                            std::vector<RegistrationResult>* per_scan_results,
                            const FusionStepCallback& on_step,
                            const RoiSelectionCallback& select_roi)
{
    if (inputs.empty())
        throw std::runtime_error("fuseScans called with no input scans");

    const size_t n = inputs.size();

    // Each accepted scan's transformed cloud is kept separately (rather
    // than blended into one running cloud) so that a scan can be undone
    // and redone later without having to un-mix its points back out.
    std::vector<PointCloudT::Ptr> committed(n);
    std::vector<bool> accepted(n, false);
    std::vector<RegistrationResult> results(n);

    auto rebuildMerged = [&](size_t upto_exclusive) {
        auto out = std::make_shared<PointCloudT>();
        for (size_t j = 0; j < upto_exclusive; ++j)
            if (accepted[j] && committed[j])
                *out += *committed[j];
        if (params.downsample_accumulated_map && !out->empty())
            out = voxelDownsample(out, params.voxel_leaf_size);
        return out;
    };

    size_t i = 0;
    while (i < n)
    {
        const auto& scan_input = inputs[i];

        auto raw = loadPLY(scan_input.ply_path);
        if (select_roi) raw = select_roi(i, scan_input, raw);
        auto clean = preprocessCloud(raw, params);

        // --- First scan seeds the reference frame ---
        if (i == 0)
        {
            auto seeded = std::make_shared<PointCloudT>();
            pcl::transformPointCloud(*clean, *seeded, scan_input.initial_guess);

            RegistrationResult identity_result;
            identity_result.transform = scan_input.initial_guess;
            identity_result.converged = true;
            identity_result.fitness_score = 0.0;

            std::cout << "[scan 0] " << scan_input.label()
                      << " seeded reference frame with " << seeded->size() << " points\n";

            StepDecision decision = StepDecision::Accept;
            if (on_step)
            {
                auto empty_merged = std::make_shared<PointCloudT>();
                decision = on_step(0, scan_input, "reference", seeded, empty_merged, identity_result);
            }
            if (decision == StepDecision::Retry)
                continue; // redo ROI selection + seeding for scan 0

            committed[0] = seeded;
            accepted[0] = true;
            results[0] = identity_result;
            ++i;
            continue;
        }

        // --- Every subsequent scan is registered against the running merged map ---
        std::cout << "[scan " << i << "] aligning " << scan_input.label() << " ...\n";
        auto merged_target = rebuildMerged(i);

        Eigen::Matrix4f coarse_transform = Eigen::Matrix4f::Identity();

        if (scan_input.has_initial_guess)
        {
            // Trust the rig's known tilt/rotation for this frame and go
            // straight to fine refinement.
            coarse_transform = scan_input.initial_guess;
        }
        else
        {
            // No known pose: estimate a coarse alignment from geometric
            // features before refining. This is the step that gives ICP
            // a starting point close enough to actually converge.
            auto coarse = coarseAlignFPFH(clean, merged_target, params);
            if (!coarse.converged)
            {
                std::cerr << "  [!] coarse alignment failed to converge for "
                          << scan_input.label() << "\n";
                StepDecision decision = StepDecision::Skip;
                if (on_step)
                {
                    auto attempt = std::make_shared<PointCloudT>();
                    pcl::transformPointCloud(*clean, *attempt, coarse.transform);
                    decision = on_step(i, scan_input, "coarse_failed", attempt, merged_target, coarse);
                }
                if (decision == StepDecision::Retry)
                    continue; // redo ROI selection + coarse alignment for this scan
                if (decision == StepDecision::Undo && i > 0)
                {
                    accepted[i - 1] = false;
                    committed[i - 1].reset();
                    --i;
                    continue;
                }
                accepted[i] = false;
                results[i] = coarse;
                ++i;
                continue;
            }
            coarse_transform = coarse.transform;
        }

        if (on_step)
        {
            auto coarse_cloud = std::make_shared<PointCloudT>();
            pcl::transformPointCloud(*clean, *coarse_cloud, coarse_transform);
            RegistrationResult coarse_result;
            coarse_result.transform = coarse_transform;
            coarse_result.converged = true;
            StepDecision decision = on_step(i, scan_input, "coarse", coarse_cloud, merged_target, coarse_result);

            if (decision == StepDecision::Retry)
                continue; // redo ROI selection + coarse alignment for this scan
            if (decision == StepDecision::Skip)
            {
                accepted[i] = false;
                results[i] = coarse_result;
                ++i;
                continue;
            }
            if (decision == StepDecision::Undo && i > 0)
            {
                accepted[i - 1] = false;
                committed[i - 1].reset();
                --i;
                continue;
            }
        }

        auto fine = refineICP(clean, merged_target, coarse_transform, params);

        bool rejected = !fine.converged || fine.fitness_score > params.icp_fitness_reject_threshold;
        if (rejected)
        {
            std::cerr << "  [!] ICP fitness " << fine.fitness_score
                      << " exceeds rejection threshold ("
                      << params.icp_fitness_reject_threshold
                      << ") for " << scan_input.label()
                      << " - keeping result but flagging it\n";
        }

        PointCloudT::Ptr transformed = std::make_shared<PointCloudT>();
        pcl::transformPointCloud(*clean, *transformed, fine.transform);

        StepDecision decision = StepDecision::Accept;
        if (on_step)
            decision = on_step(i, scan_input, rejected ? "flagged" : "refined", transformed, merged_target, fine);

        if (decision == StepDecision::Retry)
            continue; // redo ROI selection + coarse/fine alignment for this scan

        if (decision == StepDecision::Skip)
        {
            accepted[i] = false;
            results[i] = fine;
            ++i;
            continue;
        }

        if (decision == StepDecision::Undo && i > 0)
        {
            accepted[i - 1] = false;
            committed[i - 1].reset();
            --i;
            continue;
        }

        // Accept
        committed[i] = transformed;
        accepted[i] = true;
        results[i] = fine;

        std::cout << "  fitness=" << fine.fitness_score
                  << " converged=" << fine.converged << "\n";
        ++i;
    }

    if (per_scan_results) *per_scan_results = results;

    return rebuildMerged(n);
}

// -------------------------------------------------------------------------
PointCloudNT::Ptr postProcessMerged(const PointCloudT::ConstPtr& merged,
                                     const PipelineParams& params)
{
    PointCloudT::Ptr working = voxelDownsample(merged, params.final_voxel_leaf_size);

    if (params.run_final_outlier_removal)
    {
        auto cleaned = std::make_shared<PointCloudT>();
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(working);
        sor.setMeanK(params.sor_mean_k);
        sor.setStddevMulThresh(params.sor_stddev_mult);
        sor.filter(*cleaned);
        working = cleaned;
    }

    auto output = std::make_shared<PointCloudNT>();
    if (params.estimate_output_normals)
    {
        auto normals = estimateNormals(working, params.output_normal_radius);
        pcl::concatenateFields(*working, *normals, *output);
    }
    else
    {
        pcl::copyPointCloud(*working, *output);
    }
    return output;
}

// -------------------------------------------------------------------------
void savePLY(const std::string& path, const PointCloudT::ConstPtr& cloud)
{
    pcl::io::savePLYFileBinary(path, *cloud);
}

void savePLY(const std::string& path, const PointCloudNT::ConstPtr& cloud)
{
    pcl::io::savePLYFileBinary(path, *cloud);
}
