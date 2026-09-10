#include "fusion_pipeline.h"

#include "scan_state.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include <pcl/io/ply_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/icp.h>
#include <pcl/surface/mls.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>

#include <unordered_map>
#include <Eigen/Eigenvalues>

using FeatureT = pcl::FPFHSignature33;
using FeatureCloudT = pcl::PointCloud<FeatureT>;
using NormalCloudT = pcl::PointCloud<pcl::Normal>;

// -------------------------------------------------------------------------
std::string ScanInput::shortLabel() const
{
    if (!subfolder_name.empty())
    {
        // subfolder_name is "<rotation folder>/<tilt folder>"; the tilt
        // folder alone identifies the scan within its session.
        const auto slash = subfolder_name.rfind('/');
        std::string leaf = (slash == std::string::npos)
            ? subfolder_name
            : subfolder_name.substr(slash + 1);

        const std::string prefix = session_name + "_";
        if (!session_name.empty() && leaf.rfind(prefix, 0) == 0)
            leaf = leaf.substr(prefix.size());
        return leaf;
    }

    const auto slash = ply_path.find_last_of('/');
    return slash == std::string::npos ? ply_path : ply_path.substr(slash + 1);
}

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
PointCloudT::Ptr cropCloud(const PointCloudT::ConstPtr& input,
                            const Eigen::Vector4f& min_pt,
                            const Eigen::Vector4f& max_pt)
{
    auto cropped = std::make_shared<PointCloudT>();
    pcl::CropBox<PointT> crop;
    crop.setInputCloud(input);
    crop.setMin(min_pt);
    crop.setMax(max_pt);
    crop.filter(*cropped);

    if (cropped->empty())
    {
        std::cerr << "  [!] crop box contains no points - keeping the full cloud\n";
        return std::make_shared<PointCloudT>(*input);
    }
    return cropped;
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
AlignmentQuality evaluateAlignment(const PointCloudT::ConstPtr& source,
                                    const PointCloudT::ConstPtr& target,
                                    const Eigen::Matrix4f& transform,
                                    const PipelineParams& params)
{
    AlignmentQuality quality;
    if (!source || !target || source->empty() || target->empty())
        return quality;

    auto moved = std::make_shared<PointCloudT>();
    pcl::transformPointCloud(*source, *moved, transform);

    pcl::search::KdTree<PointT> tree;
    tree.setInputCloud(target);

    // Stride-sample rather than take a prefix: PLY point order follows the
    // depth image raster, so a prefix would only ever cover the top of the
    // frame and would score a tilted scan on the wrong part of the surface.
    const size_t stride = params.eval_max_samples > 0
        ? std::max<size_t>(1, moved->size() / static_cast<size_t>(params.eval_max_samples))
        : 1;

    const float max_dist = params.eval_correspondence_distance;
    const float max_dist_sq = max_dist * max_dist;

    double sum_sq = 0.0;
    std::vector<int>   indices(1);
    std::vector<float> sq_distances(1);

    for (size_t i = 0; i < moved->size(); i += stride)
    {
        const PointT& p = (*moved)[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            continue;

        ++quality.sampled;
        if (tree.nearestKSearch(p, 1, indices, sq_distances) > 0 &&
            sq_distances[0] <= max_dist_sq)
        {
            ++quality.inliers;
            sum_sq += sq_distances[0];
        }
    }

    if (quality.sampled > 0)
        quality.inlier_fraction = static_cast<double>(quality.inliers) /
                                   static_cast<double>(quality.sampled);
    if (quality.inliers > 0)
        quality.rmse = std::sqrt(sum_sq / static_cast<double>(quality.inliers));

    return quality;
}


// -------------------------------------------------------------------------
PlaneFrame estimatePlaneFrame(const PointCloudT::ConstPtr& cloud)
{
    PlaneFrame plane;
    if (!cloud || cloud->size() < 100) return plane;

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud, centroid, covariance);

    // Self-adjoint solver returns eigenvalues ascending, so the first
    // eigenvector is the direction of least spread: the plane normal.
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    const Eigen::Vector3f eigenvalues = solver.eigenvalues();
    const Eigen::Matrix3f eigenvectors = solver.eigenvectors();

    const Eigen::Vector3f normal = eigenvectors.col(0).normalized();
    const Eigen::Vector3f u = eigenvectors.col(2).normalized();
    const Eigen::Vector3f v = normal.cross(u).normalized();

    plane.origin = centroid.head<3>();
    plane.basis.row(0) = u;
    plane.basis.row(1) = v;
    plane.basis.row(2) = normal;

    const float in_plane = std::sqrt(std::max(0.0f, eigenvalues(2)));
    const float out_of_plane = std::sqrt(std::max(0.0f, eigenvalues(0)));
    plane.planarity = (in_plane > 1e-9f) ? (out_of_plane / in_plane) : 1.0;
    plane.valid = true;
    return plane;
}

// -------------------------------------------------------------------------
namespace
{
// Packs a 2D cell index into one key. The offset keeps negative indices from
// colliding with positive ones.
inline long long heightCellKey(long long i, long long j)
{
    return (i + 1000000LL) * 4000000LL + (j + 1000000LL);
}
} // namespace

// -------------------------------------------------------------------------
HeightGrid buildHeightGrid(const PointCloudT& cloud, const PlaneFrame& plane,
                            float cell_size, int min_points_per_cell)
{
    HeightGrid grid;
    grid.cell_size = cell_size;
    grid.min_points_per_cell = min_points_per_cell;

    std::unordered_map<long long, std::pair<double, int>> accumulator;
    accumulator.reserve(cloud.size() / 4 + 16);

    for (const auto& point : cloud)
    {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            continue;

        const Eigen::Vector3f local = plane.basis * (point.getVector3fMap() - plane.origin);
        const long long i = static_cast<long long>(std::floor(local.x() / cell_size));
        const long long j = static_cast<long long>(std::floor(local.y() / cell_size));

        auto& cell = accumulator[heightCellKey(i, j)];
        cell.first += local.z();
        cell.second += 1;
    }

    grid.cells.reserve(accumulator.size());
    for (const auto& entry : accumulator)
    {
        // A single stray point's height is noise, not surface.
        if (entry.second.second >= min_points_per_cell)
            grid.cells[entry.first] = static_cast<float>(entry.second.first / entry.second.second);
    }
    return grid;
}

// -------------------------------------------------------------------------
AlignmentQuality evaluateAgainstHeightGrid(const PointCloudT::ConstPtr& source,
                                             const Eigen::Matrix4f& transform,
                                             const PlaneFrame& plane,
                                             const HeightGrid& target,
                                             const PipelineParams& params)
{
    AlignmentQuality quality;
    if (!plane.valid || !source || source->empty() || target.empty()) return quality;

    PointCloudT moved;
    pcl::transformPointCloud(*source, moved, transform);
    // Grid the source exactly as the target was gridded, or the two are not
    // comparable cell for cell.
    const HeightGrid source_grid =
        buildHeightGrid(moved, plane, target.cell_size, target.min_points_per_cell);
    if (source_grid.empty()) return quality;

    const double max_height_error = params.eval_correspondence_distance;
    double sum_sq = 0.0;

    for (const auto& cell : source_grid.cells)
    {
        const auto it = target.cells.find(cell.first);
        if (it == target.cells.end()) continue; // no lateral overlap here

        ++quality.sampled;
        const double difference = std::abs(static_cast<double>(cell.second) - it->second);
        if (difference <= max_height_error)
        {
            ++quality.inliers;
            sum_sq += difference * difference;
        }
    }

    if (quality.sampled > 0)
        quality.inlier_fraction = static_cast<double>(quality.inliers) /
                                   static_cast<double>(quality.sampled);
    if (quality.inliers > 0)
        quality.rmse = std::sqrt(sum_sq / static_cast<double>(quality.inliers));
    return quality;
}

// -------------------------------------------------------------------------
AlignmentQuality evaluateAlignmentOnPlane(const PointCloudT::ConstPtr& source,
                                            const PointCloudT::ConstPtr& target,
                                            const Eigen::Matrix4f& transform,
                                            const PlaneFrame& plane,
                                            const PipelineParams& params)
{
    if (!plane.valid || !source || !target || source->empty() || target->empty())
        return AlignmentQuality{};

    // Convenience wrapper: builds the target grid every call. Callers running
    // many transforms against one fixed target should build the grid once and
    // use evaluateAgainstHeightGrid instead.
    const HeightGrid target_grid = buildHeightGrid(*target, plane, params.plane_grid_cell);
    return evaluateAgainstHeightGrid(source, transform, plane, target_grid, params);
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
