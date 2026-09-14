#include "fusion_pipeline.h"

#include "scan_state.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include <pcl/io/ply_io.h>
#include <pcl/common/io.h>
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
namespace
{
// Packs a 3D voxel index into one key, 21 bits per axis. Indices wrap beyond
// +/-2^20 voxels, which even at a 0.5 mm leaf is +/-500 m - far outside any
// scan.
inline uint64_t voxelKey(int64_t i, int64_t j, int64_t k)
{
    const int64_t  offset = int64_t(1) << 20;
    const uint64_t mask   = (uint64_t(1) << 21) - 1;
    return ((static_cast<uint64_t>(i + offset) & mask) << 42) |
           ((static_cast<uint64_t>(j + offset) & mask) << 21) |
            (static_cast<uint64_t>(k + offset) & mask);
}
} // namespace

VoxelMap::VoxelMap(float leaf_size) : leaf_size_(leaf_size) {}

void VoxelMap::add(const PointCloudT& cloud, const NormalCloudT* normals,
                   const Eigen::Matrix4f& transform)
{
    const Eigen::Matrix3f rotation    = transform.topLeftCorner<3, 3>();
    const Eigen::Vector3f translation = transform.topRightCorner<3, 1>();
    const bool have_normals = normals && normals->size() == cloud.size();
    const bool merge = leaf_size_ > 0.0f;
    // Same cell boundaries as pcl::VoxelGrid: floor(coordinate / leaf).
    const float inverse_leaf = merge ? 1.0f / leaf_size_ : 0.0f;

    for (size_t n = 0; n < cloud.size(); ++n)
    {
        const PointT& p = cloud[n];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const Eigen::Vector3f q = rotation * p.getVector3fMap() + translation;

        Cell* cell;
        if (merge)
        {
            const uint64_t key = voxelKey(static_cast<int64_t>(std::floor(q.x() * inverse_leaf)),
                                           static_cast<int64_t>(std::floor(q.y() * inverse_leaf)),
                                           static_cast<int64_t>(std::floor(q.z() * inverse_leaf)));
            const auto slot = index_.try_emplace(key, static_cast<uint32_t>(cells_.size()));
            if (slot.second) cells_.emplace_back();
            cell = &cells_[slot.first->second];
        }
        else
        {
            cells_.emplace_back();
            cell = &cells_.back();
        }

        cell->x += q.x(); cell->y += q.y(); cell->z += q.z();
        cell->r += p.r;   cell->g += p.g;   cell->b += p.b;
        ++cell->count;

        if (have_normals)
        {
            const Eigen::Vector3f normal = (*normals)[n].getNormalVector3fMap();
            if (normal.allFinite())
            {
                const Eigen::Vector3f turned = rotation * normal;
                cell->nx += turned.x(); cell->ny += turned.y(); cell->nz += turned.z();
                ++cell->normal_count;
            }
        }
    }
}

PointCloudNT::Ptr VoxelMap::cloudWithNormals() const
{
    const float nan = std::numeric_limits<float>::quiet_NaN();

    auto out = std::make_shared<PointCloudNT>();
    out->resize(cells_.size());
    for (size_t i = 0; i < cells_.size(); ++i)
    {
        const Cell& c = cells_[i];
        PointNT& p = (*out)[i];

        const double inverse_count = 1.0 / static_cast<double>(c.count);
        p.x = static_cast<float>(c.x * inverse_count);
        p.y = static_cast<float>(c.y * inverse_count);
        p.z = static_cast<float>(c.z * inverse_count);
        p.r = static_cast<uint8_t>(std::lround(c.r * inverse_count));
        p.g = static_cast<uint8_t>(std::lround(c.g * inverse_count));
        p.b = static_cast<uint8_t>(std::lround(c.b * inverse_count));

        // Normals are all oriented toward their own sensor, so they add up
        // rather than cancel; the length of the sum only needs normalising.
        const Eigen::Vector3f normal(c.nx, c.ny, c.nz);
        const float length = normal.norm();
        if (c.normal_count > 0 && length > 1e-6f)
        {
            p.normal_x = c.nx / length;
            p.normal_y = c.ny / length;
            p.normal_z = c.nz / length;
        }
        else
        {
            p.normal_x = p.normal_y = p.normal_z = nan;
        }
        p.curvature = 0.0f;
    }
    out->width = static_cast<uint32_t>(out->size());
    out->height = 1;
    out->is_dense = true;
    return out;
}

PointCloudT::Ptr VoxelMap::cloud() const
{
    auto out = std::make_shared<PointCloudT>();
    pcl::copyPointCloud(*cloudWithNormals(), *out);
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

// -------------------------------------------------------------------------
NormalCloudT::Ptr estimateSensorNormals(const PointCloudT::ConstPtr& cloud, float radius)
{
    auto normals = std::make_shared<NormalCloudT>();
    pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(radius);
    ne.setSearchMethod(std::make_shared<pcl::search::KdTree<PointT>>());
    // The scan is still in its camera frame, so the sensor sits at the
    // origin. Set explicitly rather than relying on the cloud's
    // sensor_origin_, which nothing in this pipeline maintains. Consistent
    // orientation matters: the map averages normals from several scans, and
    // oppositely-signed estimates of one surface would cancel.
    ne.setViewPoint(0.0f, 0.0f, 0.0f);
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
RegistrationResult refineICP(const PointCloudNT::ConstPtr& source,
                              const PointCloudNT::ConstPtr& target,
                              const Eigen::Matrix4f& initial_guess,
                              const PipelineParams& params)
{
    // Point-to-plane ICP converges faster and more accurately than
    // point-to-point on the roughly-continuous surfaces this rig scans.
    //
    // The normals come in with the clouds. This used to estimate them here,
    // on both clouds, on every call - and the target is the accumulated map,
    // so each scan paid for normals over every scan placed before it. On a
    // 28-scan session at 1 mm that was 85% of the whole refine stage, and why
    // each refine took longer than the last. (Only the target's normals are
    // read: PCL's non-symmetric point-to-plane objective ignores the source's.)
    pcl::IterativeClosestPointWithNormals<PointNT, PointNT> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);
    icp.setMaxCorrespondenceDistance(params.icp_max_correspondence_distance);
    icp.setMaximumIterations(params.icp_max_iterations);
    icp.setTransformationEpsilon(params.icp_transformation_epsilon);
    icp.setEuclideanFitnessEpsilon(params.icp_euclidean_fitness_epsilon);

    PointCloudNT aligned;
    icp.align(aligned, initial_guess);

    RegistrationResult result;
    result.converged = icp.hasConverged();
    result.transform = icp.getFinalTransformation();
    result.hit_max_iterations =
        icp.getConvergeCriteria()->getConvergenceState() ==
        pcl::registration::DefaultConvergenceCriteria<float>::CONVERGENCE_CRITERIA_ITERATIONS;

    // Limited to ICP's own correspondence distance. Without a limit PCL
    // averages over every source point, so the part of a scan lying outside
    // the map's footprint dominates the figure.
    const double max_range = params.icp_max_correspondence_distance;
    result.fitness_score = icp.getFitnessScore(max_range * max_range);
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
