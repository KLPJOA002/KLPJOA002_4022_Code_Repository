#include "fusion_reconcile.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <thread>
#include <unordered_map>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/centroid.h>
#include <pcl/common/io.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>

namespace
{

// One point's contribution to whichever cell it lands in.
struct WeightedSample
{
    float    value;       // height along the cell axis being combined
    float    weight;
    uint32_t scan;        // which contribution it came from
    Eigen::Vector3f position;
    Eigen::Vector3f normal;
    float    r, g, b;
};

// Packs a 2D or 3D integer cell index into one key. The offset keeps
// negative indices from colliding with positive ones, and the multipliers
// are large enough that no realistic scan extent wraps around.
inline long long cellKey2D(long long i, long long j)
{
    return (i + 1000000LL) * 4000000LL + (j + 1000000LL);
}

inline long long cellKey3D(long long i, long long j, long long k)
{
    return ((i + 100000LL) * 400000LL + (j + 100000LL)) * 400000LL + (k + 100000LL);
}

// Weighted mean of `samples`' values after discarding `trim` of the total
// weight from each end. Trimming by weight rather than by count means a
// single high-confidence scan is not outvoted by several poor ones.
//
// This is the step that makes fusion robust rather than merely averaged:
// with a misregistered scan present, a plain mean lands between the two
// surfaces (belonging to neither), while a trimmed mean discards the
// outlying sheet and returns the surface the majority agrees on.
float trimmedWeightedMean(std::vector<WeightedSample>& samples, float trim)
{
    if (samples.empty()) return 0.0f;
    if (samples.size() == 1) return samples[0].value;

    std::sort(samples.begin(), samples.end(),
               [](const WeightedSample& a, const WeightedSample& b) { return a.value < b.value; });

    double total_weight = 0.0;
    for (const auto& s : samples) total_weight += s.weight;
    if (total_weight <= 0.0) return samples[samples.size() / 2].value;

    const double cut = std::min(0.49, static_cast<double>(trim)) * total_weight;

    double running = 0.0;
    double kept_weight = 0.0;
    double accumulated = 0.0;
    for (const auto& s : samples)
    {
        const double start = running;
        const double end = running + s.weight;
        running = end;

        // Overlap of this sample's weight interval with the kept middle band.
        const double lo = std::max(start, cut);
        const double hi = std::min(end, total_weight - cut);
        if (hi <= lo) continue;

        const double w = hi - lo;
        accumulated += w * s.value;
        kept_weight += w;
    }

    if (kept_weight <= 0.0) return samples[samples.size() / 2].value;
    return static_cast<float>(accumulated / kept_weight);
}

// Population spread of the values, weight-ignoring - used only to report how
// far apart the scans were within a cell.
double spreadOf(const std::vector<WeightedSample>& samples)
{
    if (samples.size() < 2) return 0.0;
    double mean = 0.0;
    for (const auto& s : samples) mean += s.value;
    mean /= static_cast<double>(samples.size());
    double sum_sq = 0.0;
    for (const auto& s : samples) sum_sq += (s.value - mean) * (s.value - mean);
    return std::sqrt(sum_sq / static_cast<double>(samples.size()));
}

size_t distinctScans(const std::vector<WeightedSample>& samples)
{
    std::vector<uint32_t> ids;
    ids.reserve(samples.size());
    for (const auto& s : samples) ids.push_back(s.scan);
    std::sort(ids.begin(), ids.end());
    return static_cast<size_t>(std::distance(ids.begin(), std::unique(ids.begin(), ids.end())));
}

PointCloudT::Ptr concatenate(const std::vector<ScanContribution>& contributions,
                              std::vector<uint32_t>& out_scan_ids,
                              std::vector<float>& out_confidence)
{
    auto merged = std::make_shared<PointCloudT>();
    size_t total = 0;
    for (const auto& c : contributions)
        if (c.cloud) total += c.cloud->size();

    merged->reserve(total);
    out_scan_ids.reserve(total);
    out_confidence.reserve(total);

    for (size_t i = 0; i < contributions.size(); ++i)
    {
        const auto& c = contributions[i];
        if (!c.cloud) continue;
        for (const auto& p : *c.cloud)
        {
            merged->push_back(p);
            out_scan_ids.push_back(static_cast<uint32_t>(i));
            out_confidence.push_back(static_cast<float>(c.confidence));
        }
    }
    merged->width = merged->size();
    merged->height = 1;
    merged->is_dense = false;
    return merged;
}

// The contributions' own normals, laid out in the same order as concatenate()
// lays out their points - or null if any contribution lacks them.
pcl::PointCloud<pcl::Normal>::Ptr concatenateNormals(const std::vector<ScanContribution>& contributions)
{
    auto merged = std::make_shared<pcl::PointCloud<pcl::Normal>>();
    for (const auto& c : contributions)
    {
        if (!c.cloud) continue;
        if (!c.normals || c.normals->size() != c.cloud->size()) return nullptr;
        *merged += *c.normals;
    }
    return merged;
}

pcl::PointCloud<pcl::Normal>::Ptr estimateNormalsFor(const PointCloudT::ConstPtr& cloud,
                                                      float radius)
{
    auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
    pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(radius);
    ne.setSearchMethod(std::make_shared<pcl::search::KdTree<PointT>>());
    ne.compute(*normals);
    return normals;
}

// Drops points far from the plane through their `neighbours` nearest
// neighbours - see ReconcileParams::run_outlier_removal for why this rather
// than statistical outlier removal. The cut is `stddev_mult` standard
// deviations above the mean residual, so it adapts to however smooth the
// blended surface is.
PointCloudT::Ptr removeOffSurfaceOutliers(const PointCloudT::ConstPtr& cloud,
                                           int neighbours, double stddev_mult)
{
    const size_t n = cloud->size();
    const int k = std::max(3, neighbours);
    if (n <= static_cast<size_t>(k)) return std::make_shared<PointCloudT>(*cloud);

    pcl::search::KdTree<PointT> tree;
    tree.setInputCloud(cloud);

    // NaN marks points with no usable fit; they are kept rather than judged.
    std::vector<float> residual(n, std::numeric_limits<float>::quiet_NaN());
    pcl::Indices indices;
    std::vector<float> sq_distances;
    for (size_t i = 0; i < n; ++i)
    {
        const auto& p = (*cloud)[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        // k + 1 because the point finds itself; it stays in the fit, which
        // only makes the test slightly more lenient.
        if (tree.nearestKSearch(p, k + 1, indices, sq_distances) < 4) continue;

        Eigen::Matrix3f covariance;
        Eigen::Vector4f centroid;
        if (pcl::computeMeanAndCovarianceMatrix(*cloud, indices, covariance, centroid) < 3)
            continue;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
        const Eigen::Vector3f normal = solver.eigenvectors().col(0); // smallest eigenvalue
        residual[i] = std::abs(normal.dot(p.getVector3fMap() - centroid.head<3>()));
    }

    double sum = 0.0, sum_sq = 0.0;
    size_t counted = 0;
    for (float r : residual)
    {
        if (!std::isfinite(r)) continue;
        sum += r; sum_sq += static_cast<double>(r) * r; ++counted;
    }
    if (counted == 0) return std::make_shared<PointCloudT>(*cloud);
    const double mean = sum / static_cast<double>(counted);
    const double stddev = std::sqrt(std::max(0.0, sum_sq / static_cast<double>(counted) - mean * mean));
    const double cut = mean + stddev_mult * stddev;

    auto kept = std::make_shared<PointCloudT>();
    kept->reserve(n);
    for (size_t i = 0; i < n; ++i)
        if (!(residual[i] > cut)) kept->push_back((*cloud)[i]);
    kept->width = kept->size();
    kept->height = 1;
    kept->is_dense = false;
    return kept;
}

} // namespace

// -------------------------------------------------------------------------
void ReconcileReport::print() const
{
    std::cout << "\nFusion (" << mode_used << "):\n";
    std::cout << "  input points            " << input_points << "\n";
    std::cout << "  cells                   " << cells_total
              << "  (" << std::fixed << std::setprecision(2) << mean_scans_per_cell
              << " scans per cell on average)\n";
    if (cells_dropped_no_corroboration > 0)
        std::cout << "  dropped, uncorroborated " << cells_dropped_no_corroboration << "\n";
    std::cout << "  after blending          " << points_after_blend << "\n";
    if (points_after_outlier_removal != points_after_blend)
        std::cout << "  after outlier removal   " << points_after_outlier_removal << "\n";
    std::cout << "  output points           " << output_points << "\n";
    std::cout << "  surface thickness       " << std::setprecision(2)
              << spread_before * 1000.0 << " mm  ->  " << spread_after * 1000.0 << " mm\n";
    std::cout << "  (thickness is how far apart the overlapping scans were within a cell -\n"
                 "   the first figure is the multi-sheet noise, the second what remains)\n";
}

// -------------------------------------------------------------------------
PointCloudNT::Ptr reconcileScans(const std::vector<ScanContribution>& contributions,
                                   const ReconcileParams& params,
                                   const PlaneFrame& plane,
                                   ReconcileReport* report)
{
    ReconcileReport local;

    std::vector<uint32_t> scan_ids;
    std::vector<float> confidence;
    auto merged = concatenate(contributions, scan_ids, confidence);
    local.input_points = merged->size();

    if (merged->empty())
    {
        if (report) *report = local;
        return std::make_shared<PointCloudNT>();
    }

    // Normals are needed for the incidence weight, and are worth having
    // anyway for the output.
    //
    // Each scan's own normals are used when the contributions carry them:
    // they describe the surface as that sensor saw it, which is exactly what
    // the incidence weight asks about, and they are already computed.
    // Estimating on the merged cloud instead means every neighbourhood search
    // wades through every overlapping scan's copy of the surface - on a
    // 28-scan session at 1 mm that single step took 220 s, most of fusion -
    // and it blurs normals across whatever offset remains between the copies.
    pcl::PointCloud<pcl::Normal>::Ptr normals;
    if (params.weight_by_incidence || params.estimate_normals)
    {
        normals = concatenateNormals(contributions);
        if (!normals)
            normals = estimateNormalsFor(merged, params.normal_radius);
    }

    // --- Per-point weights ------------------------------------------------
    std::vector<float> weights(merged->size(), 1.0f);
    for (size_t i = 0; i < merged->size(); ++i)
    {
        float w = 1.0f;
        if (params.weight_by_registration) w *= std::max(0.0f, confidence[i]);

        const Eigen::Vector3f position = (*merged)[i].getVector3fMap();
        const Eigen::Vector3f to_sensor =
            contributions[scan_ids[i]].sensor_origin - position;
        const float range = to_sensor.norm();

        if (params.weight_by_incidence && normals && i < normals->size())
        {
            const Eigen::Vector3f n = (*normals)[i].getNormalVector3fMap();
            if (range > 1e-6f && n.allFinite())
            {
                // |cos| because estimated normals have arbitrary orientation.
                const float cosine = std::abs(n.dot(to_sensor / range));
                w *= std::max(params.incidence_min_weight, cosine);
            }
        }

        if (params.weight_by_range && range > 1e-6f)
        {
            const float ratio = params.range_reference / range;
            w *= std::min(1.0f, ratio * ratio); // never reward being closer than reference
        }

        weights[i] = std::max(w, 1e-6f);
    }

    // --- Bin into cells ---------------------------------------------------
    const bool height_field =
        params.mode == ReconcileParams::Mode::HeightField ||
        (params.mode == ReconcileParams::Mode::Auto && plane.valid && plane.planarity < 0.15);
    local.mode_used = height_field ? "height field" : "volumetric";

    std::unordered_map<long long, std::vector<WeightedSample>> cells;
    cells.reserve(merged->size() / 4 + 16);

    for (size_t i = 0; i < merged->size(); ++i)
    {
        const auto& p = (*merged)[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;

        WeightedSample sample;
        sample.weight = weights[i];
        sample.scan = scan_ids[i];
        sample.position = p.getVector3fMap();
        sample.normal = Eigen::Vector3f::UnitZ();
        if (normals && i < normals->size())
        {
            const Eigen::Vector3f n = (*normals)[i].getNormalVector3fMap();
            if (n.allFinite() && n.norm() > 1e-6f) sample.normal = n.normalized();
        }
        sample.r = p.r; sample.g = p.g; sample.b = p.b;

        long long key;
        if (height_field)
        {
            // Lateral position picks the cell; height along the plane normal
            // is the quantity being reconciled. This is the right split for a
            // surface scanned from above: disagreement between scans is
            // almost entirely in height, and collapsing it per cell removes
            // the multi-sheet problem outright.
            const Eigen::Vector3f local_p = plane.basis * (sample.position - plane.origin);
            key = cellKey2D(static_cast<long long>(std::floor(local_p.x() / params.height_cell)),
                             static_cast<long long>(std::floor(local_p.y() / params.height_cell)));
            sample.value = local_p.z();
        }
        else
        {
            const float v = params.voxel_size;
            key = cellKey3D(static_cast<long long>(std::floor(p.x / v)),
                             static_cast<long long>(std::floor(p.y / v)),
                             static_cast<long long>(std::floor(p.z / v)));
            // Value is filled in once the whole cell is known: a voxel must
            // be reconciled along ONE axis, and per-point normals disagree
            // enough that using each point's own would measure every sample
            // along a slightly different direction - corrupting both the
            // trimmed mean and the thickness figure derived from it.
            sample.value = 0.0f;
        }
        cells[key].push_back(std::move(sample));
    }

    local.cells_total = cells.size();

    // --- Combine each cell -------------------------------------------------
    auto blended = std::make_shared<PointCloudT>();
    auto blended_normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
    blended->reserve(cells.size());

    double spread_before_sum = 0.0;
    double scans_per_cell_sum = 0.0;
    size_t multi_scan_cells = 0;

    for (auto& entry : cells)
    {
        auto& samples = entry.second;

        // Establish the cell's reconciliation axis before anything reads a
        // sample's value. For a height field that axis is the plane normal
        // and the values are already set; for a voxel it is the cell's mean
        // surface normal, which is the direction registration error actually
        // displaces a surface in.
        Eigen::Vector3f cell_axis = Eigen::Vector3f::UnitZ();
        if (!height_field)
        {
            const Eigen::Vector3f reference = samples.front().normal;
            Eigen::Vector3f sum = Eigen::Vector3f::Zero();
            for (const auto& s : samples)
                sum += (s.normal.dot(reference) < 0.0f ? -s.normal : s.normal);
            cell_axis = (sum.norm() > 1e-6f) ? sum.normalized() : reference;
            for (auto& s : samples)
                s.value = cell_axis.dot(s.position);
        }

        const size_t scans_here = distinctScans(samples);
        scans_per_cell_sum += static_cast<double>(scans_here);

        if (static_cast<int>(scans_here) < params.min_contributing_scans)
        {
            ++local.cells_dropped_no_corroboration;
            continue;
        }

        if (scans_here > 1)
        {
            spread_before_sum += spreadOf(samples);
            ++multi_scan_cells;
        }

        // Weighted means for everything that isn't the reconciled axis.
        double total_weight = 0.0;
        Eigen::Vector3f mean_position = Eigen::Vector3f::Zero();
        Eigen::Vector3f mean_normal = Eigen::Vector3f::Zero();
        Eigen::Vector3f mean_colour = Eigen::Vector3f::Zero();
        const Eigen::Vector3f reference_normal = samples.front().normal;

        for (const auto& s : samples)
        {
            total_weight += s.weight;
            mean_position += s.weight * s.position;
            // Flip normals into a common hemisphere before averaging, or
            // oppositely-oriented estimates cancel to nothing.
            mean_normal += s.weight * (s.normal.dot(reference_normal) < 0.0f ? -s.normal : s.normal);
            mean_colour += s.weight * Eigen::Vector3f(s.r, s.g, s.b);
        }
        if (total_weight <= 0.0) continue;

        mean_position /= static_cast<float>(total_weight);
        mean_colour /= static_cast<float>(total_weight);
        if (mean_normal.norm() > 1e-6f) mean_normal.normalize();
        else mean_normal = Eigen::Vector3f::UnitZ();

        const float reconciled = trimmedWeightedMean(samples, params.trim_fraction);

        PointT point;
        if (height_field)
        {
            // Rebuild the point from its cell centre and the reconciled
            // height, so the output is one clean sample per cell.
            const Eigen::Vector3f local_mean = plane.basis * (mean_position - plane.origin);
            const Eigen::Vector3f rebuilt(local_mean.x(), local_mean.y(), reconciled);
            const Eigen::Vector3f world = plane.basis.transpose() * rebuilt + plane.origin;
            point.x = world.x(); point.y = world.y(); point.z = world.z();
        }
        else
        {
            // Slide the weighted-mean position along the cell axis so it sits
            // on the robustly-agreed surface, leaving lateral placement be.
            const float current = cell_axis.dot(mean_position);
            const Eigen::Vector3f corrected = mean_position + (reconciled - current) * cell_axis;
            point.x = corrected.x(); point.y = corrected.y(); point.z = corrected.z();
        }

        point.r = static_cast<uint8_t>(std::clamp(mean_colour.x(), 0.0f, 255.0f));
        point.g = static_cast<uint8_t>(std::clamp(mean_colour.y(), 0.0f, 255.0f));
        point.b = static_cast<uint8_t>(std::clamp(mean_colour.z(), 0.0f, 255.0f));
        blended->push_back(point);

        pcl::Normal n;
        n.normal_x = mean_normal.x(); n.normal_y = mean_normal.y(); n.normal_z = mean_normal.z();
        blended_normals->push_back(n);
    }

    blended->width = blended->size();
    blended->height = 1;
    blended->is_dense = false;
    local.points_after_blend = blended->size();
    local.mean_scans_per_cell = local.cells_total > 0
        ? scans_per_cell_sum / static_cast<double>(local.cells_total) : 0.0;
    local.spread_before = multi_scan_cells > 0
        ? spread_before_sum / static_cast<double>(multi_scan_cells) : 0.0;

    // --- Outlier removal ---------------------------------------------------
    PointCloudT::Ptr working = blended;
    if (params.run_outlier_removal && !working->empty())
        working = removeOffSurfaceOutliers(working, params.outlier_neighbours,
                                            params.outlier_stddev_mult);
    local.points_after_outlier_removal = working->size();

    // --- Surface fit --------------------------------------------------------
    auto output = std::make_shared<PointCloudNT>();
    if (params.use_mls && !working->empty())
    {
        pcl::MovingLeastSquares<PointT, PointNT> mls;
        mls.setInputCloud(working);
        mls.setSearchMethod(std::make_shared<pcl::search::KdTree<PointT>>());
        mls.setSearchRadius(params.mls_radius);
        mls.setPolynomialOrder(params.mls_polynomial_order);
        mls.setComputeNormals(true);
        // Each point's fit is independent, so threads change only the order
        // points come out in. PCL defaults to one.
        mls.setNumberOfThreads(std::max(1u, std::thread::hardware_concurrency()));
        mls.process(*output);

        if (output->empty())
        {
            std::cerr << "  [!] MLS returned nothing (radius " << params.mls_radius
                      << " m may be too small for this point spacing) - keeping the blended cloud\n";
        }
    }

    if (output->empty())
    {
        if (params.estimate_normals)
        {
            auto final_normals = estimateNormalsFor(working, params.output_normal_radius);
            pcl::concatenateFields(*working, *final_normals, *output);
        }
        else
        {
            pcl::copyPointCloud(*working, *output);
        }
    }

    local.output_points = output->size();

    // Re-measure the surface thickness on the result, the same way as before,
    // so the two figures are comparable.
    if (!output->empty() && local.spread_before > 0.0)
    {
        std::unordered_map<long long, std::vector<float>> after;
        std::unordered_map<long long, Eigen::Vector3f> voxel_axis;
        for (const auto& p : *output)
        {
            if (!std::isfinite(p.x)) continue;
            if (height_field)
            {
                const Eigen::Vector3f local_p =
                    plane.basis * (p.getVector3fMap() - plane.origin);
                after[cellKey2D(static_cast<long long>(std::floor(local_p.x() / params.height_cell)),
                                 static_cast<long long>(std::floor(local_p.y() / params.height_cell)))]
                    .push_back(local_p.z());
            }
            else
            {
                // Project onto a fixed world axis rather than each point's own
                // normal: within one small voxel the surface is near-planar,
                // so any consistent axis measures its thickness comparably,
                // whereas per-point normals would measure nothing meaningful.
                const float v = params.voxel_size;
                Eigen::Vector3f axis(p.normal_x, p.normal_y, p.normal_z);
                if (!axis.allFinite() || axis.norm() < 1e-6f) axis = Eigen::Vector3f::UnitZ();
                axis.normalize();
                auto& bucket = after[cellKey3D(
                    static_cast<long long>(std::floor(p.x / v)),
                    static_cast<long long>(std::floor(p.y / v)),
                    static_cast<long long>(std::floor(p.z / v)))];
                // First point in the voxel fixes the axis for the rest.
                if (bucket.empty()) voxel_axis[
                    cellKey3D(static_cast<long long>(std::floor(p.x / v)),
                               static_cast<long long>(std::floor(p.y / v)),
                               static_cast<long long>(std::floor(p.z / v)))] = axis;
                const auto& fixed = voxel_axis[cellKey3D(
                    static_cast<long long>(std::floor(p.x / v)),
                    static_cast<long long>(std::floor(p.y / v)),
                    static_cast<long long>(std::floor(p.z / v)))];
                bucket.push_back(fixed.dot(p.getVector3fMap()));
            }
        }
        double sum = 0.0; size_t counted = 0;
        for (const auto& entry : after)
        {
            if (entry.second.size() < 2) continue;
            double mean = 0.0;
            for (float v : entry.second) mean += v;
            mean /= static_cast<double>(entry.second.size());
            double sq = 0.0;
            for (float v : entry.second) sq += (v - mean) * (v - mean);
            sum += std::sqrt(sq / static_cast<double>(entry.second.size()));
            ++counted;
        }
        local.spread_after = counted > 0 ? sum / static_cast<double>(counted) : 0.0;
    }

    if (report) *report = local;
    return output;
}
