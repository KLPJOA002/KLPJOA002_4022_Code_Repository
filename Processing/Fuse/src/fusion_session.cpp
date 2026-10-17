#include "fusion_session.h"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <map>
#include <sstream>
#include <unordered_map>

#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <pcl/common/transforms.h>

namespace
{

// Formats a metre-scale distance as millimetres, or "-" when the quality
// measurement never got any correspondences to average.
std::string mm(double metres)
{
    if (metres >= 1e6) return "    -";
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2) << metres * 1000.0;
    return ss.str();
}

std::string pct(double fraction)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(0) << fraction * 100.0 << "%";
    return ss.str();
}

} // namespace

const char* toString(Stage stage)
{
    switch (stage)
    {
        case Stage::Load:            return "load";
        case Stage::CalibrateTilt:   return "tilt-calibration";
        case Stage::ResolveRotation: return "rotation-direction";
        case Stage::CoarseBatch:  return "coarse-align";
        case Stage::CoarseReview: return "coarse-review";
        case Stage::RefineBatch:  return "refine";
        case Stage::RefineReview: return "refine-review";
        case Stage::Fuse:         return "fuse";
        case Stage::Done:         return "done";
    }
    return "?";
}

// -------------------------------------------------------------------------
FusionSession::FusionSession(std::vector<ScanInput> inputs, PipelineParams params)
    : params_(params)
{
    scans_.reserve(inputs.size());
    for (auto& input : inputs)
    {
        ScanState state;
        state.input = std::move(input);
        scans_.push_back(std::move(state));
    }
    buildRotationGroups();
}

// -------------------------------------------------------------------------
void FusionSession::setRigGeometry(const RigGeometry& rig)
{
    rig_ = rig;
    rig_known_ = true;
    Eigen::Vector3f diagonal;
    if (parseImuFrame(rig_.imu_frame, diagonal))
    {
        tilt_calibration_.imu_to_cloud = diagonal.asDiagonal();
        tilt_calibration_.frame_name = rig_.imu_frame;
        tilt_calibration_.frame_forced = true;
    }
}

// -------------------------------------------------------------------------
void FusionSession::buildRotationGroups()
{
    rotation_groups_.clear();

    // Scans arrive sorted by |tilt| (which registers better), so the groups
    // are interleaved in scan order and have to be collected by name and
    // then put back into capture order.
    std::unordered_map<std::string, size_t> by_name;
    for (size_t i = 0; i < scans_.size(); ++i)
    {
        const auto& in = scans_[i].input;
        if (in.rotation_group.empty()) continue;

        auto it = by_name.find(in.rotation_group);
        if (it == by_name.end())
        {
            RotationGroup group;
            group.name = in.rotation_group;
            group.session_name = in.session_name;
            group.index = in.rotation_index;
            group.cumulative_deg = in.rotation_degrees;
            by_name[in.rotation_group] = rotation_groups_.size();
            rotation_groups_.push_back(std::move(group));
            it = by_name.find(in.rotation_group);
        }
        rotation_groups_[it->second].scans.push_back(i);
    }

    // Capture order: by session, then by the rotation ordinal within it.
    std::sort(rotation_groups_.begin(), rotation_groups_.end(),
               [](const RotationGroup& a, const RotationGroup& b) {
                   if (a.session_name != b.session_name) return a.session_name < b.session_name;
                   return a.index < b.index;
               });

    // Each step's nominal magnitude is the gap between consecutive folder
    // names. They are always recorded positive, so this is a magnitude only -
    // the sign is exactly what has to be inferred.
    for (size_t k = 1; k < rotation_groups_.size(); ++k)
    {
        const auto& prev = rotation_groups_[k - 1];
        auto& group = rotation_groups_[k];
        group.nominal_step_deg = (group.session_name == prev.session_name)
            ? std::abs(group.cumulative_deg - prev.cumulative_deg)
            : std::abs(group.cumulative_deg); // a new session restarts from its own zero
    }
}

// -------------------------------------------------------------------------
void FusionSession::resolveGroupGravityReferences()
{
    for (auto& group : rotation_groups_)
    {
        group.has_reference_gravity = false;
        if (group.scans.empty()) continue;

        // The flattest scan in the group is the within-group reference: the
        // tilt of every other scan is measured relative to it, so the group's
        // own absolute pose stays entirely the rotation stage's business.
        size_t best = group.scans.front();
        int best_tilt = std::numeric_limits<int>::max();
        for (size_t index : group.scans)
        {
            const auto& scan = scans_[index];
            const int tilt = scan.input.has_tilt ? std::abs(scan.input.tilt_degrees) : 999;
            if (tilt < best_tilt) { best_tilt = tilt; best = index; }
        }
        group.reference_scan = best;

        if (scans_[best].input.has_gravity)
        {
            group.reference_gravity = scans_[best].input.gravity;
            group.has_reference_gravity = true;
        }
    }
}

// -------------------------------------------------------------------------
size_t FusionSession::countWithGravity() const
{
    return static_cast<size_t>(
        std::count_if(scans_.begin(), scans_.end(),
                       [](const ScanState& s) { return s.input.has_gravity; }));
}

// -------------------------------------------------------------------------
int FusionSession::rotationGroupOf(size_t scan_index) const
{
    for (size_t g = 0; g < rotation_groups_.size(); ++g)
    {
        const auto& scans = rotation_groups_[g].scans;
        if (std::find(scans.begin(), scans.end(), scan_index) != scans.end())
            return static_cast<int>(g);
    }
    return -1;
}

// -------------------------------------------------------------------------
Eigen::Matrix4f FusionSession::tiltComponentFor(const ScanState& scan,
                                                  const RotationGroup* group) const
{
    // Preferred: measure the tilt from gravity. Tilting the camera swings
    // gravity across the frame by exactly the tilt angle about exactly the
    // tilt axis, so this needs no assumption about which camera axis the
    // hinge lies on, nor that the mount reached the angle it was told to.
    // It matters most when the rig has been rotated: the hinge is fixed in
    // the world, so in the camera's own frame the tilt axis turns with the
    // turntable and a single configured axis cannot describe every group.
    if (use_imu_tilt_ && group && group->has_reference_gravity && scan.input.has_gravity)
    {
        const Eigen::Matrix3f R = imuRotationBetween(scan, group->reference_gravity);

        // The pivot found by the calibration search takes precedence over any
        // configured one; both fall back to the origin, which is only correct
        // if the camera happens to sit on the tilt axis.
        const Eigen::Vector3f pivot =
            tilt_calibration_.valid ? tilt_calibration_.pivot : rig_.tilt_pivot;

        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        T.block<3, 3>(0, 0) = R;
        T.block<3, 1>(0, 3) = pivot - R * pivot;
        return T;
    }

    // Fallback: the commanded angle about the configured axis.
    if (rig_.seedsTilt() && scan.input.has_tilt)
    {
        const double reference_tilt =
            (group && group->reference_scan < scans_.size() &&
             scans_[group->reference_scan].input.has_tilt)
                ? scans_[group->reference_scan].input.tilt_degrees : 0;
        return rotationAboutPoint(
            rig_.tilt_axis,
            rig_.tilt_sign * static_cast<float>(scan.input.tilt_degrees - reference_tilt),
            rig_.tilt_pivot);
    }

    return Eigen::Matrix4f::Identity();
}

// -------------------------------------------------------------------------
void FusionSession::applyGuessToScan(size_t scan_index, double cumulative_rotation_deg,
                                      const RotationGroup* group)
{
    auto& scan = scans_.at(scan_index);
    if (!rig_known_) return;

    const bool have_rotation = rig_.seedsRotation();
    const bool imu_tilt = use_imu_tilt_ && group && group->has_reference_gravity &&
                           scan.input.has_gravity;
    const bool have_tilt = imu_tilt || (rig_.seedsTilt() && scan.input.has_tilt);

    if (!have_rotation && !have_tilt)
    {
        scan.input.has_initial_guess = false;
        scan.input.initial_guess = Eigen::Matrix4f::Identity();
        return;
    }

    // The cumulative angle already carries its sign, so rotation_sign must
    // not be applied to it a second time.
    const Eigen::Matrix4f R_rot = have_rotation
        ? rotationAboutPoint(rig_.rotation_axis, static_cast<float>(cumulative_rotation_deg),
                              rig_.rotation_pivot)
        : Eigen::Matrix4f::Identity();

    // Tilt first, into the group's reference frame; then the group's own
    // rotation, into the common frame.
    scan.input.initial_guess = R_rot * tiltComponentFor(scan, group);
    scan.input.has_initial_guess = true;
}

// -------------------------------------------------------------------------
void FusionSession::resolveRotationFromTags()
{
    // The seeds sit at the folder angles here. That is fine: the tag fit
    // solves the turn about vertical freely, so it lands on the true angle
    // whatever the folder said.
    solveTagPoses();

    // The axis the rig's rotation turns about, in the convention
    // rotationAboutPoint uses, so a measured angle can replace a seeded one.
    const Eigen::Vector3f axis =
        Eigen::AngleAxisf(Eigen::Matrix3f(rotationAboutAxis(rig_.rotation_axis, 90.0f)
                                               .topLeftCorner<3, 3>())).axis();

    for (size_t k = 1; k < rotation_groups_.size(); ++k)
    {
        auto& group = rotation_groups_[k];

        // Each scan's correction to its seeded angle: the seed is
        // R_rotation(angle) * R_tilt, so tag * seed^-1 is the turn still missing.
        std::vector<double> corrections;
        for (size_t index : group.scans)
        {
            const auto& scan = scans_[index];
            if (!scan.tag_fit.valid || index == reference_index_) continue;
            const Eigen::Matrix3f delta = scan.tag_fit.transform.topLeftCorner<3, 3>() *
                scan.input.initial_guess.topLeftCorner<3, 3>().transpose();
            const Eigen::AngleAxisf aa(delta);
            const double sign = aa.axis().dot(axis) >= 0.0f ? 1.0 : -1.0;
            corrections.push_back(sign * aa.angle() * 180.0 / M_PI);
        }
        if (corrections.empty())
        {
            std::cout << "[rotation] " << group.name
                      << ": no scan placed by tags - left to the surface-based search\n";
            continue;
        }

        std::vector<double> sorted = corrections;
        std::nth_element(sorted.begin(), sorted.begin() + sorted.size() / 2, sorted.end());
        const double correction = sorted[sorted.size() / 2];

        double spread = 0.0;
        for (double c : corrections) spread = std::max(spread, std::abs(c - correction));

        // Wrapped to (-180, 180]: a turn back past the start reads as -90,
        // not as the folder's 90 plus 180.
        const double folder_deg = group.cumulative_deg;
        group.cumulative_deg = std::remainder(folder_deg + correction, 360.0);
        group.from_tags = true;
        group.resolved = true;
        group.ambiguous = false;
        group.tag_spread_deg = spread;

        std::cout << "[rotation] " << group.name << " from tags: " << std::fixed
                  << std::setprecision(1) << group.cumulative_deg << " deg (folder says "
                  << folder_deg << "; " << corrections.size()
                  << " scan(s), spread " << spread << " deg)\n";
    }

    applyRotationAngles(1);
}

// -------------------------------------------------------------------------
void FusionSession::applyRotationAngles(size_t from_group_index)
{
    for (size_t k = from_group_index; k < rotation_groups_.size(); ++k)
    {
        auto& group = rotation_groups_[k];
        // A tag-measured angle is absolute, not a step from the previous group.
        if (k > 0 && !group.from_tags)
        {
            const auto& prev = rotation_groups_[k - 1];
            group.cumulative_deg = (group.session_name == prev.session_name)
                ? prev.cumulative_deg + group.step_sign * group.nominal_step_deg
                : group.step_sign * group.nominal_step_deg;
        }
        for (size_t index : group.scans)
            applyGuessToScan(index, group.cumulative_deg, &group);
    }
}

// -------------------------------------------------------------------------
void FusionSession::flipRotationGroup(size_t group_index)
{
    if (group_index == 0 || group_index >= rotation_groups_.size())
    {
        std::cerr << "  [!] the first rotation group defines the frame - nothing to flip\n";
        return;
    }

    auto& group = rotation_groups_[group_index];
    // A manual flip overrides the tags too: back to the folder step, reversed.
    group.from_tags = false;
    group.step_sign = -group.step_sign;
    group.resolved = true;   // an explicit choice overrides whatever was inferred
    group.ambiguous = false;

    applyRotationAngles(group_index);

    // Every group from here on moves, so re-open all of them for alignment.
    for (size_t k = group_index; k < rotation_groups_.size(); ++k)
    {
        for (size_t index : rotation_groups_[k].scans)
        {
            if (index == reference_index_) continue;
            auto& scan = scans_[index];
            if (scan.status == ScanStatus::Excluded) continue;
            scan.status = ScanStatus::Pending;
            scan.flagged = false;
            scan.fine_quality = AlignmentQuality{};
            scan.icp_result = RegistrationResult{};
        }
    }

    std::cout << "[rotation] " << group.name << " flipped to "
              << group.cumulative_deg << " deg";
    if (group_index + 1 < rotation_groups_.size())
        std::cout << " (and " << (rotation_groups_.size() - group_index - 1)
                  << " later group(s) shifted with it)";
    std::cout << " - re-run alignment to place them\n";
}

// -------------------------------------------------------------------------
void FusionSession::loadAll(const ProgressFn& progress, const RoiProvider& roi)
{
    for (size_t i = 0; i < scans_.size(); ++i)
    {
        auto& scan = scans_[i];
        if (progress) progress(i, scans_.size(), scan.label());

        try
        {
            auto raw = loadPLY(scan.input.ply_path);
            scan.raw_point_count = raw->size();

            // Before the crop: the tags sit around the edge of the scene,
            // which is exactly what an ROI tends to cut away.
            if (params_.use_tag_alignment)
                locateScanTags(scan, *raw);

            if (roi)
            {
                Eigen::Vector4f roi_min, roi_max;
                if (roi(scan.input, raw, roi_min, roi_max))
                {
                    scan.has_roi = true;
                    scan.roi_min = roi_min;
                    scan.roi_max = roi_max;
                }
            }
            if (scan.has_roi)
                raw = cropCloud(raw, scan.roi_min, scan.roi_max);

            scan.prepared = preprocessCloud(raw, params_);
            scan.normals  = estimateSensorNormals(scan.prepared, params_.normal_radius);
            // `raw` goes out of scope here on purpose - see ScanState::prepared.

            std::cout << "[load] " << scan.label() << ": " << scan.raw_point_count
                      << " -> " << scan.prepared->size() << " points"
                      << (scan.has_roi ? " (cropped)" : "") << "\n";
        }
        catch (const std::exception& e)
        {
            std::cerr << "  [!] " << scan.label() << " failed to load: " << e.what()
                      << " - excluding it from this run\n";
            scan.status = ScanStatus::Excluded;
            scan.excluded_by = "auto";
            continue;
        }
    }

    // The reference frame has to be a scan that actually loaded.
    reference_index_ = scans_.size();
    for (size_t i = 0; i < scans_.size(); ++i)
    {
        if (scans_[i].loaded() && scans_[i].status != ScanStatus::Excluded)
        {
            reference_index_ = i;
            break;
        }
    }
    resolveGroupGravityReferences();

    if (reference_index_ < scans_.size())
    {
        // Fit the scene's dominant plane once. A sand surface viewed from
        // above is a height field, and that changes which alignment metric
        // can actually see an error.
        scene_plane_ = estimatePlaneFrame(scans_[reference_index_].prepared);
        if (scene_plane_.valid)
        {
            std::cout << "[scene] planarity " << scene_plane_.planarity
                      << (scene_plane_.isPlanar(params_.planarity_threshold)
                            ? "  - flat scene: scoring alignments as a height field, since"
                              " nearest-neighbour distance cannot see in-plane error here"
                            : "  - 3D scene: scoring alignments by nearest-neighbour distance")
                      << "\n";
        }

        auto& ref = scans_[reference_index_];
        // Discovery orders scans so this is a 0-tilt scan; its own rig
        // guess (identity in the usual case) defines the common frame that
        // every other scan's transform is expressed in.
        ref.coarse_transform = ref.input.initial_guess;
        ref.transform        = ref.input.initial_guess;
        ref.status           = ScanStatus::Reference;
        std::cout << "[load] reference frame: " << ref.label() << "\n";
    }
}

// -------------------------------------------------------------------------
void FusionSession::reloadScan(size_t index, const RoiProvider& roi)
{
    auto& scan = scans_.at(index);
    auto raw = loadPLY(scan.input.ply_path);
    scan.raw_point_count = raw->size();
    if (params_.use_tag_alignment && scan.tags.empty())
        locateScanTags(scan, *raw);

    if (roi)
    {
        Eigen::Vector4f roi_min, roi_max;
        scan.has_roi = roi(scan.input, raw, roi_min, roi_max);
        scan.roi_min = roi_min;
        scan.roi_max = roi_max;
    }
    if (scan.has_roi)
        raw = cropCloud(raw, scan.roi_min, scan.roi_max);

    scan.prepared = preprocessCloud(raw, params_);
    scan.normals  = estimateSensorNormals(scan.prepared, params_.normal_radius);
    scan.status = (index == reference_index_) ? ScanStatus::Reference : ScanStatus::Pending;
    scan.flagged = false;
}

// -------------------------------------------------------------------------
VoxelMap FusionSession::buildMap(size_t exclude_index, size_t max_index) const
{
    VoxelMap map(mapLeafSize());
    for (size_t j = 0; j < scans_.size() && j < max_index; ++j)
    {
        if (j == exclude_index) continue;
        const auto& other = scans_[j];
        if (!other.loaded() || !contributesToFusion(other.status)) continue;
        map.add(*other.prepared, other.normals.get(), other.transform);
    }
    return map;
}

// -------------------------------------------------------------------------
void FusionSession::solveTagPoses()
{
    for (auto& scan : scans_) { scan.tag_fit = TagFit{}; scan.tag_rejected_rms = -1.0; }
    if (reference_index_ >= scans_.size()) return;

    // Running sums of each placed tag's corners in the common frame, so a tag
    // seen by several scans is held at the mean of where they put it.
    std::map<int, std::pair<TagCorners, int>> sums;
    TagSet placed;
    auto addTags = [&](const ScanState& scan, const Eigen::Matrix4f& pose) {
        const Eigen::Matrix3f R = pose.topLeftCorner<3, 3>();
        const Eigen::Vector3f t = pose.topRightCorner<3, 1>();
        for (const auto& [id, corners] : scan.tags)
        {
            auto slot = sums.try_emplace(id);
            if (slot.second) slot.first->second.first.fill(Eigen::Vector3f::Zero());
            for (int k = 0; k < 4; ++k) slot.first->second.first[k] += R * corners[k] + t;
            const int n = ++slot.first->second.second;
            for (int k = 0; k < 4; ++k) placed[id][k] = slot.first->second.first[k] / static_cast<float>(n);
        }
    };

    auto& ref = scans_[reference_index_];
    if (ref.tags.empty())
    {
        std::cout << "[tags] the reference scan has no located tags - nothing to chain from\n";
        return;
    }
    ref.tag_fit.valid = true;
    ref.tag_fit.transform = ref.transform;
    addTags(ref, ref.transform);

    // Vertical in the common frame, from the reference's gravity reading. With
    // it, a scan's tilt comes from its IMU seed and the tags only solve the
    // turn about vertical and the translation - see fitTagsAboutAxis.
    const bool have_up = use_imu_tilt_ && ref.input.has_gravity;
    const Eigen::Vector3f up = have_up
        ? Eigen::Vector3f(ref.transform.topLeftCorner<3, 3>() *
                          tilt_calibration_.imu_to_cloud * ref.input.gravity.normalized())
        : Eigen::Vector3f::UnitZ();

    // Greedy: each round places the scan sharing the most tags with what is
    // already placed, since more shared tags means a better-conditioned fit
    // and less error handed on down the chain.
    std::vector<bool> done(scans_.size(), false);
    done[reference_index_] = true;
    size_t placed_count = 1, candidates = 1;
    for (size_t i = 0; i < scans_.size(); ++i)
        if (i != reference_index_ && !scans_[i].tags.empty() &&
            scans_[i].status != ScanStatus::Excluded) ++candidates;

    while (true)
    {
        size_t best = scans_.size();
        int best_shared = 0;
        for (size_t i = 0; i < scans_.size(); ++i)
        {
            const auto& scan = scans_[i];
            if (done[i] || scan.tags.empty() || scan.status == ScanStatus::Excluded) continue;
            int shared = 0;
            for (const auto& entry : scan.tags) shared += placed.count(entry.first) ? 1 : 0;
            if (shared >= std::max(1, params_.tag_min_shared) && shared > best_shared)
            {
                best = i;
                best_shared = shared;
            }
        }
        if (best == scans_.size()) break;

        auto& scan = scans_[best];
        done[best] = true;
        const bool constrain_tilt = have_up && scan.input.has_gravity && scan.input.has_initial_guess;
        TagFit fit = constrain_tilt
            ? fitTagsAboutAxis(scan.tags, placed, params_.tag_min_shared,
                               scan.input.initial_guess.topLeftCorner<3, 3>(), up)
            : fitTags(scan.tags, placed, params_.tag_min_shared);
        if (!fit.valid || fit.rms > params_.tag_max_fit_rmse)
        {
            if (fit.valid) scan.tag_rejected_rms = fit.rms;
            std::cout << "  [i] tag fit for " << scan.label() << " leaves " << mm(fit.rms)
                      << "mm corner error - not trusted\n";
            continue;
        }
        scan.tag_fit = fit;
        addTags(scan, fit.transform);
        ++placed_count;
    }

    std::cout << "[tags] " << placed_count << " of " << candidates
              << " scan(s) with tags could be placed from the tags"
              << (have_up ? " (tilt from the IMU, turn about vertical and position from the tags)"
                          : " alone") << "\n";
}

// -------------------------------------------------------------------------
void FusionSession::locateScanTags(ScanState& scan, const PointCloudT& raw) const
{
    scan.tags.clear();
    std::vector<TagLocateReport> reports;

    // A sidecar wins: for a cropped scan it is the only source, since the crop
    // removed the tags from the cloud.
    std::string sidecar_path;
    const bool from_sidecar = findTagSidecar(scan.input.ply_path, sidecar_path) &&
                              readTagSidecar(sidecar_path, scan.tags, &reports);
    if (!from_sidecar)
    {
        std::string detections_path, scan_info_path;
        CameraIntrinsics intrinsics;
        if (!findTagFiles(scan.input.ply_path, detections_path, scan_info_path))
        {
            std::cout << "  [i] " << scan.label() << ": no *_tags.csv, and no *_detections.csv /"
                         " scan_info.txt beside the PLY - no tags\n";
            return;
        }
        if (!readColorIntrinsics(scan_info_path, intrinsics))
        {
            std::cerr << "  [!] " << scan.label() << ": no colour intrinsics in " << scan_info_path
                      << " - cannot place tags\n";
            return;
        }
        scan.tags = locateTags(raw, detections_path, intrinsics, &reports);
    }

    std::ostringstream line;
    line << "[tags] " << scan.label() << (from_sidecar ? " (from sidecar)" : "") << ":";
    if (reports.empty()) line << " no decoded tags";
    for (const auto& r : reports)
    {
        line << "  id" << r.id;
        if (r.located)
            line << " (edge " << std::fixed << std::setprecision(1) << r.edge_length * 1000.0
                 << "mm, plane rms " << mm(r.plane_rms) << "mm)";
        else
            line << " (skipped: " << r.reason << ")";
    }
    std::cout << line.str() << "\n";
}

// -------------------------------------------------------------------------
void FusionSession::computeCoarseFor(size_t index, const PointCloudT::ConstPtr& map)
{
    // What the pipeline does without tags, run unchanged...
    computeGeometricCoarseFor(index, map);

    auto& scan = scans_[index];
    scan.has_tag_disagreement = false;
    if (!params_.use_tag_alignment || !scan.tag_fit.valid) return;

    // ...then the tag pose, put to the same test.
    const TagFit& fit = scan.tag_fit;
    std::ostringstream ids;
    for (size_t k = 0; k < fit.ids.size(); ++k) ids << (k ? "," : "") << fit.ids[k];

    const AlignmentQuality tag_coverage = evaluateAlignment(scan.prepared, map, fit.transform, params_);
    const AlignmentQuality tag_quality  = scoreAlignment(scan.prepared, map, fit.transform);
    const bool tag_usable = tag_coverage.inlier_fraction >= params_.coarse_min_inlier_fraction;

    const bool existing_ok = scan.status == ScanStatus::CoarseAligned;
    const bool tags_win = tag_usable &&
        (!existing_ok || tag_quality.score() < scan.coarse_quality.score());

    std::cout << "  [tags] " << scan.label() << ": tags " << ids.str()
              << " fit to " << mm(fit.rms) << "mm, score " << mm(tag_quality.score())
              << " vs " << (scan.coarse_source.empty() ? "none" : scan.coarse_source) << " "
              << (existing_ok ? mm(scan.coarse_quality.score()) : std::string("failed"));
    if (!scan.coarse_source.empty())
    {
        // How far apart the two hypotheses put the scan: a few mm and a
        // degree or so is the tags' own precision; tens of degrees means one
        // of them is simply wrong.
        const Eigen::Matrix4f delta = fit.transform * scan.coarse_transform.inverse();
        const double angle = Eigen::AngleAxisf(Eigen::Matrix3f(delta.topLeftCorner<3, 3>())).angle();
        const double shift = (fit.transform.topRightCorner<3, 1>() -
                              scan.coarse_transform.topRightCorner<3, 1>()).norm();
        // Kept for QC and the review, not just printed.
        scan.has_tag_disagreement = true;
        scan.tag_disagreement_deg = angle * 180.0 / M_PI;
        scan.tag_disagreement_shift = shift;
        std::cout << " (poses differ by " << std::fixed << std::setprecision(1)
                  << angle * 180.0 / M_PI << " deg, camera " << mm(shift) << "mm)";
    }
    std::cout << (tag_usable ? "" : "  [tag pose covers too little of the map]")
              << "  -> " << (tags_win ? std::string("tags") : scan.coarse_source) << "\n";

    if (!scan.coarse_source.empty())
    {
        const Eigen::Matrix4f delta = fit.transform * scan.coarse_transform.inverse();
        const double angle_deg =
            Eigen::AngleAxisf(Eigen::Matrix3f(delta.topLeftCorner<3, 3>())).angle() * 180.0 / M_PI;
        if (angle_deg > params_.tag_disagreement_warn_deg)
        {
            const int group = rotationGroupOf(index);
            std::cerr << "  [!] tags put " << scan.label() << " " << std::fixed << std::setprecision(0)
                      << angle_deg << " deg away from the " << scan.coarse_source << " pose"
                      << (group >= 0 ? " - rotation group " + rotation_groups_[group].name +
                                       " is probably resolved the wrong way"
                                     : std::string()) << ". The surface alone cannot tell them"
                         " apart here; check it in the viewer (t flips a group)\n";
        }
    }

    if (!tags_win) return;
    scan.coarse_transform = fit.transform;
    scan.coarse_quality   = tag_quality;
    scan.coarse_source    = "tags";
    scan.status           = ScanStatus::CoarseAligned;
}

// -------------------------------------------------------------------------
void FusionSession::computeGeometricCoarseFor(size_t index, const PointCloudT::ConstPtr& map)
{
    auto& scan = scans_[index];
    scan.coarse_source.clear();

    // Two questions, two measures. Whether a pose lands on the map at all is
    // a question of coverage - how much of the scan has map surface nearby -
    // which nearest-neighbour distance answers well. Which of two poses fits
    // *better* goes to scoreAlignment, which on a flat scene compares heights
    // at matching positions and so sees the in-plane slide and spin that
    // nearest-neighbour distance is nearly blind to. (The height comparison
    // is no good for coverage: it only looks where the clouds already overlap.)

    // Hypothesis 1: the rig's own geometry, if we have it. Cheap, and on a
    // calibrated rig it beats feature matching outright - but a wrong sign
    // or a mis-set axis makes it confidently wrong, which is why we score
    // it instead of trusting it.
    const bool have_seed = scan.input.has_initial_guess;
    AlignmentQuality seed_coverage, seed_quality;
    if (have_seed)
    {
        seed_coverage = evaluateAlignment(scan.prepared, map, scan.input.initial_guess, params_);
        seed_quality  = scoreAlignment(scan.prepared, map, scan.input.initial_guess);
    }

    const bool seed_is_usable =
        have_seed && seed_coverage.inlier_fraction >= params_.coarse_min_inlier_fraction;

    if (seed_is_usable)
    {
        scan.coarse_transform = scan.input.initial_guess;
        scan.coarse_quality   = seed_quality;
        scan.coarse_source    = "seed";
        scan.status           = ScanStatus::CoarseAligned;
        return;
    }

    if (have_seed)
    {
        std::cout << "  [i] rig seed for " << scan.label() << " covers only "
                  << pct(seed_coverage.inlier_fraction) << " of the cloud (need "
                  << pct(params_.coarse_min_inlier_fraction)
                  << ") - falling back to feature matching\n";
    }

    // Hypothesis 2: FPFH + RANSAC against the map. Randomised, so a retry
    // genuinely produces a different answer.
    auto fpfh = coarseAlignFPFH(scan.prepared, map, params_);
    AlignmentQuality fpfh_coverage, fpfh_quality;
    if (fpfh.converged)
    {
        fpfh_coverage = evaluateAlignment(scan.prepared, map, fpfh.transform, params_);
        fpfh_quality  = scoreAlignment(scan.prepared, map, fpfh.transform);
    }

    // Take whichever hypothesis actually sits better on the map. A weak rig
    // seed still beats a failed feature match, so this is not just "prefer
    // FPFH when the seed is weak".
    const bool fpfh_wins = fpfh.converged && fpfh_quality.score() < seed_quality.score();

    AlignmentQuality chosen_coverage;
    if (fpfh_wins)
    {
        scan.coarse_transform = fpfh.transform;
        scan.coarse_quality   = fpfh_quality;
        scan.coarse_source    = "fpfh";
        chosen_coverage       = fpfh_coverage;
    }
    else if (have_seed)
    {
        scan.coarse_transform = scan.input.initial_guess;
        scan.coarse_quality   = seed_quality;
        scan.coarse_source    = "seed";
        chosen_coverage       = seed_coverage;
    }
    else
    {
        scan.coarse_quality = fpfh_quality;
        scan.status = ScanStatus::CoarseFailed;
        std::cerr << "  [!] coarse alignment failed for " << scan.label()
                  << " (no rig seed, feature matching did not converge)\n";
        return;
    }

    scan.status = chosen_coverage.inlier_fraction >= params_.coarse_min_inlier_fraction
        ? ScanStatus::CoarseAligned
        : ScanStatus::CoarseFailed;

    if (scan.status == ScanStatus::CoarseFailed)
    {
        std::cerr << "  [!] best coarse hypothesis for " << scan.label() << " covers only "
                  << pct(chosen_coverage.inlier_fraction)
                  << " of the cloud - marking it coarse-failed\n";
    }
}


// -------------------------------------------------------------------------
void FusionSession::printTiltFromImuReport() const
{
    const size_t with_gravity = countWithGravity();
    if (with_gravity == 0)
    {
        std::cout << "\nTilt source: commanded angles (no usable IMU readings found)\n";
        return;
    }
    if (!use_imu_tilt_)
    {
        std::cout << "\nTilt source: commanded angles (IMU tilt disabled)\n";
        return;
    }

    std::cout << "\nTilt measured from the IMU (" << with_gravity << "/" << scans_.size()
              << " scans have a gravity reading):\n";
    std::cout << "  group                 commanded  measured  ratio  tilt axis in camera frame\n";
    std::cout << "  --------------------  ---------  --------  -----  -------------------------\n";

    for (const auto& group : rotation_groups_)
    {
        if (!group.has_reference_gravity) continue;

        // Show the largest tilt in the group: that is where a wrong axis or
        // a wrong angle scale does the most damage, so it is the row worth
        // looking at.
        size_t worst = group.scans.front();
        int worst_tilt = -1;
        for (size_t index : group.scans)
        {
            const auto& scan = scans_[index];
            if (!scan.input.has_gravity || !scan.input.has_tilt) continue;
            if (std::abs(scan.input.tilt_degrees) > worst_tilt)
            {
                worst_tilt = std::abs(scan.input.tilt_degrees);
                worst = index;
            }
        }
        if (worst_tilt <= 0) continue;

        const auto& scan = scans_[worst];
        const Eigen::Vector3f a = group.reference_gravity.normalized();
        const Eigen::Vector3f b = scan.input.gravity.normalized();
        const double measured = std::acos(std::max(-1.0f, std::min(1.0f, a.dot(b)))) * 180.0 / M_PI;

        Eigen::Vector3f axis = a.cross(b);
        if (axis.norm() > 1e-6f) axis.normalize();

        const double commanded = std::abs(static_cast<double>(
            scan.input.tilt_degrees - scans_[group.reference_scan].input.tilt_degrees));

        std::ostringstream axis_text;
        axis_text << "(" << std::fixed << std::setprecision(3)
                  << axis.x() << ", " << axis.y() << ", " << axis.z() << ")";

        // Show the tail: every group name starts with the same session
        // stamp, so a left-truncated name is the same for all of them.
        std::string short_name = group.name;
        if (short_name.size() > 20) short_name = short_name.substr(short_name.size() - 20);
        std::cout << "  " << std::left << std::setw(20) << short_name << std::right
                  << "  " << std::setw(9) << commanded
                  << "  " << std::setw(8) << std::fixed << std::setprecision(2) << measured
                  << "  " << std::setw(5) << std::setprecision(2)
                  << (commanded > 0 ? measured / commanded : 0.0)
                  << "  " << axis_text.str() << "\n";
    }

    std::cout << "  The tilt axis differs between rotation groups when the hinge is fixed in\n"
                 "  the world and the camera turns with the turntable - which is exactly the\n"
                 "  case a single --tilt-axis cannot describe, and why gravity is used instead.\n";
}

// -------------------------------------------------------------------------
Eigen::Vector3f FusionSession::configuredTiltAxis() const
{
    switch (rig_.tilt_axis)
    {
        case RigAxis::Y: return Eigen::Vector3f::UnitY();
        case RigAxis::Z: return Eigen::Vector3f::UnitZ();
        default:         return Eigen::Vector3f::UnitX();
    }
}

// -------------------------------------------------------------------------
bool FusionSession::solvePivot(const std::vector<Eigen::Matrix3f>& rotations,
                                const std::vector<Eigen::Vector3f>& translations,
                                Eigen::Vector3f& pivot,
                                double& residual)
{
    if (rotations.size() != translations.size() || rotations.empty())
        return false;

    // Rotating about a point p leaves p fixed, so the translation such a
    // rotation induces is t = (I - R) p. Every observed (R, t) pair is one
    // instance of that, and three unknowns are shared across all of them.
    const int rows = static_cast<int>(rotations.size()) * 3;
    Eigen::MatrixXf A(rows, 3);
    Eigen::VectorXf b(rows);

    for (size_t k = 0; k < rotations.size(); ++k)
    {
        A.block<3, 3>(static_cast<int>(k) * 3, 0) =
            Eigen::Matrix3f::Identity() - rotations[k];
        b.segment<3>(static_cast<int>(k) * 3) = translations[k];
    }

    // (I - R) is singular along the rotation axis - sliding the pivot along
    // its own axis changes nothing, so that component is unobservable. SVD
    // gives the minimum-norm solution, which picks the pivot closest to the
    // optical centre out of the infinitely many that fit equally well.
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    svd.setThreshold(1e-4f);
    pivot = svd.solve(b);

    residual = std::sqrt((A * pivot - b).squaredNorm() / static_cast<double>(rotations.size()));
    return true;
}

// -------------------------------------------------------------------------
std::vector<size_t> FusionSession::selectTiltCalibrationScans() const
{
    if (reference_index_ >= scans_.size()) return {};

    const auto& reference = scans_[reference_index_];
    const int reference_tilt = reference.input.has_tilt ? reference.input.tilt_degrees : 0;
    const int ref_group = rotationGroupOf(reference_index_);

    // Stay within the reference's own rotation group: mixing groups would fold
    // the turntable's geometry into what is meant to be a tilt measurement.
    std::vector<size_t> candidates;
    for (size_t i = 0; i < scans_.size(); ++i)
    {
        if (i == reference_index_) continue;
        const auto& scan = scans_[i];
        if (!scan.loaded() || scan.status == ScanStatus::Excluded) continue;
        if (!scan.input.has_tilt) continue;
        if (rotationGroupOf(i) != ref_group) continue;
        if (std::abs(scan.input.tilt_degrees - reference_tilt) > params_.tilt_calibration_max_deg)
            continue;
        candidates.push_back(i);
    }

    // Largest tilts first: they discriminate hardest between frame conventions
    // and pivots, because that is where a wrong one hurts most.
    std::sort(candidates.begin(), candidates.end(), [this, reference_tilt](size_t a, size_t b) {
        return std::abs(scans_[a].input.tilt_degrees - reference_tilt) >
               std::abs(scans_[b].input.tilt_degrees - reference_tilt);
    });
    if (candidates.size() > static_cast<size_t>(params_.tilt_calibration_max_scans))
        candidates.resize(static_cast<size_t>(params_.tilt_calibration_max_scans));
    return candidates;
}

// -------------------------------------------------------------------------
Eigen::Matrix3f FusionSession::imuRotationBetween(const ScanState& scan,
                                                    const Eigen::Vector3f& reference_gravity) const
{
    // Measured in the IMU's own frame...
    const Eigen::Matrix3f in_imu_frame =
        rotationBetweenVectors(scan.input.gravity, reference_gravity);

    // ...then carried into the frame the point clouds are actually written in.
    // Conjugation, not multiplication: the same physical rotation expressed in
    // a different basis. Getting this wrong is invisible at small tilts and
    // ruinous at large ones, which is why the basis is detected, not assumed.
    const Eigen::Matrix3f& F = tilt_calibration_.imu_to_cloud;
    return F * in_imu_frame * F.transpose();
}

// -------------------------------------------------------------------------
void FusionSession::prepareCalibrationCaches(const std::vector<size_t>& calibration_scans)
{
    calibration_clouds_.clear();
    if (reference_index_ >= scans_.size()) return;

    const float voxel = std::max(params_.calibration_voxel_size, params_.voxel_leaf_size);

    auto reference_cloud = voxelDownsample(scans_[reference_index_].prepared, voxel);

    // min_points_per_cell = 1: these clouds are already voxel-averaged, so each
    // point represents a local mean and a cell legitimately holds only one or
    // two. Demanding three would throw most of the grid away and leave the
    // searches comparing almost nothing.
    calibration_target_ = buildHeightGrid(*reference_cloud, scene_plane_,
                                            params_.plane_grid_cell, /*min_points_per_cell=*/1);

    for (size_t index : calibration_scans)
    {
        if (!scans_[index].loaded()) continue;
        calibration_clouds_[index] = voxelDownsample(scans_[index].prepared, voxel);
    }
}

// -------------------------------------------------------------------------
double FusionSession::calibrationResidual(const std::vector<size_t>& calibration_scans,
                                            const Eigen::Vector3f& pivot) const
{
    if (calibration_scans.empty() || reference_index_ >= scans_.size() ||
        calibration_target_.empty())
        return std::numeric_limits<double>::max();

    const int ref_group = rotationGroupOf(reference_index_);
    const Eigen::Vector3f reference_gravity =
        (ref_group >= 0 && rotation_groups_[ref_group].has_reference_gravity)
            ? rotation_groups_[ref_group].reference_gravity
            : scans_[reference_index_].input.gravity;

    double total = 0.0;
    size_t counted = 0;

    for (size_t index : calibration_scans)
    {
        const auto cloud_it = calibration_clouds_.find(index);
        if (cloud_it == calibration_clouds_.end()) continue;
        const auto& scan = scans_[index];
        if (!scan.input.has_gravity) continue;

        const Eigen::Matrix3f R = imuRotationBetween(scan, reference_gravity);
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        T.block<3, 3>(0, 0) = R;
        T.block<3, 1>(0, 3) = pivot - R * pivot;

        const auto quality = evaluateAgainstHeightGrid(cloud_it->second, T, scene_plane_,
                                                         calibration_target_, params_);
        // Score, not rmse: a transform can achieve a small rmse over a sliver
        // of the cloud, and coverage is what separates right from wrong here.
        total += quality.valid() ? std::min(quality.score(), 10.0) : 10.0;
        ++counted;
    }
    return counted > 0 ? total / static_cast<double>(counted)
                        : std::numeric_limits<double>::max();
}

// -------------------------------------------------------------------------
Eigen::Vector3f FusionSession::meanHingeAxis(const std::vector<size_t>& calibration_scans) const
{
    const int ref_group = rotationGroupOf(reference_index_);
    const Eigen::Vector3f reference_gravity =
        (ref_group >= 0 && rotation_groups_[ref_group].has_reference_gravity)
            ? rotation_groups_[ref_group].reference_gravity
            : scans_[reference_index_].input.gravity;

    Eigen::Vector3f hinge = Eigen::Vector3f::Zero();
    for (size_t index : calibration_scans)
    {
        const auto& scan = scans_[index];
        if (!scan.input.has_gravity) continue;
        const Eigen::AngleAxisf aa(imuRotationBetween(scan, reference_gravity));
        Eigen::Vector3f axis = aa.axis();
        // Hemisphere-align before averaging, or opposite tilts cancel out.
        if (hinge.dot(axis) < 0.0f) axis = -axis;
        hinge += axis;
    }
    return (hinge.norm() > 1e-6f) ? hinge.normalized().eval() : Eigen::Vector3f::UnitX();
}

// -------------------------------------------------------------------------
double FusionSession::searchTiltPivot(const std::vector<size_t>& calibration_scans,
                                        int levels, Eigen::Vector3f& out_pivot) const
{
    out_pivot = Eigen::Vector3f::Zero();
    if (calibration_scans.empty()) return std::numeric_limits<double>::max();

    const Eigen::Vector3f hinge = meanHingeAxis(calibration_scans);

    // Two directions spanning the plane perpendicular to the hinge.
    const Eigen::Vector3f seed = (std::abs(hinge.x()) < 0.9f)
        ? Eigen::Vector3f::UnitX() : Eigen::Vector3f::UnitY();
    const Eigen::Vector3f u = hinge.cross(seed).normalized();
    const Eigen::Vector3f v = hinge.cross(u).normalized();

    Eigen::Vector3f centre = Eigen::Vector3f::Zero();
    double best = calibrationResidual(calibration_scans, centre);

    // Nine samples per axis span the configured range, so `step` follows from
    // it rather than being fixed. A rig whose tilt axis sits near the camera
    // face wants a small range: the score surface is nearly flat in the pivot
    // when the real lever arm is short, and a wide search would happily wander
    // off to a large, meaningless offset.
    float step = params_.tilt_pivot_search_range / 4.0f;

    for (int level = 0; level < levels; ++level)
    {
        Eigen::Vector3f level_best = centre;
        for (int a = -4; a <= 4; ++a)
        {
            for (int b = -4; b <= 4; ++b)
            {
                if (a == 0 && b == 0) continue;
                const Eigen::Vector3f candidate =
                    centre + static_cast<float>(a) * step * u + static_cast<float>(b) * step * v;
                const double residual = calibrationResidual(calibration_scans, candidate);
                if (residual < best)
                {
                    best = residual;
                    level_best = candidate;
                }
            }
        }
        centre = level_best;
        step *= 0.4f;
    }

    out_pivot = centre;
    return best;
}

// -------------------------------------------------------------------------
void FusionSession::calibrateImuTiltGeometry(const std::vector<size_t>& calibration_scans)
{
    // The realistic possibilities for a depth-sensor export: the depth frame
    // itself, or one of its three 180-degree flips. Only rotations are
    // considered - a mirrored frame would flip handedness and break far more
    // than the tilt seed.
    struct Candidate { const char* name; Eigen::Vector3f diagonal; };
    const std::vector<Candidate> candidates = {
        { "identity (depth frame)", Eigen::Vector3f( 1,  1,  1) },
        { "Rx180 - y,z negated",    Eigen::Vector3f( 1, -1, -1) },
        { "Ry180 - x,z negated",    Eigen::Vector3f(-1,  1, -1) },
        { "Rz180 - x,y negated",    Eigen::Vector3f(-1, -1,  1) },
    };

    std::cout << "[tilt] resolving the IMU-to-cloud frame and the tilt pivot together\n";
    std::cout << "       (each frame gets its own pivot search - with the camera off the\n"
                 "        tilt axis, comparing frames at a zero pivot compares nothing)\n";

    double best = std::numeric_limits<double>::max();
    double runner_up = std::numeric_limits<double>::max();
    Eigen::Matrix3f best_frame = Eigen::Matrix3f::Identity();
    Eigen::Vector3f best_pivot = Eigen::Vector3f::Zero();
    std::string best_name = candidates.front().name;
    std::vector<std::string> tied;

    for (const auto& candidate : candidates)
    {
        tilt_calibration_.imu_to_cloud = candidate.diagonal.asDiagonal();

        Eigen::Vector3f pivot;
        const double score = searchTiltPivot(calibration_scans, /*levels=*/2, pivot);

        std::cout << "         " << std::left << std::setw(22) << candidate.name << std::right
                  << "  score " << std::fixed << std::setprecision(4) << score
                  << "   pivot (" << std::setprecision(3) << pivot.x() << ", " << pivot.y()
                  << ", " << pivot.z() << ")\n";

        // Conjugating a rotation by a flip about its own axis leaves it
        // unchanged, so candidates commonly come in indistinguishable pairs.
        // Note that rather than pretending to have chosen between them.
        if (score < best * 1.01 && score > best * 0.99)
            tied.push_back(candidate.name);

        if (score < best)
        {
            runner_up = best;
            best = score;
            best_frame = candidate.diagonal.asDiagonal();
            best_pivot = pivot;
            best_name = candidate.name;
            tied.clear();
            tied.push_back(candidate.name);
        }
        else if (score < runner_up)
        {
            runner_up = score;
        }
    }

    // What the data alone would choose is kept for the record either way...
    tilt_calibration_.measured_frame_name = best_name;
    tilt_calibration_.frame_forced = false;

    // ...but a frame given in the rig geometry wins. It is a hardware fact
    // (the IMU is fixed to the camera), and the measurement above cannot
    // reliably separate identity from Rx180 - see RigGeometry::imu_frame.
    Eigen::Vector3f forced_diagonal;
    if (rig_known_ && parseImuFrame(rig_.imu_frame, forced_diagonal))
    {
        const Eigen::Matrix3f forced = forced_diagonal.asDiagonal();
        std::string forced_name = rig_.imu_frame;
        double forced_score = best;
        for (const auto& candidate : candidates)
            if (candidate.diagonal == forced_diagonal) forced_name = candidate.name;
        if (!forced.isApprox(best_frame))
        {
            // Score the configured frame on its own for the warning.
            tilt_calibration_.imu_to_cloud = forced;
            Eigen::Vector3f unused;
            forced_score = searchTiltPivot(calibration_scans, /*levels=*/2, unused);
            std::cerr << "  [!] tilt calibration would pick " << best_name << " (score "
                      << std::fixed << std::setprecision(4) << best << ") but the rig's IMU frame is "
                      << forced_name << " (score " << forced_score << ") - using the rig's"
                      << (forced_score < best * 1.05 ? "; the two are effectively tied here" : "")
                      << "\n";
        }
        best_frame = forced;
        best_name = forced_name;
        best = forced_score;
        tilt_calibration_.frame_forced = true;
    }

    tilt_calibration_.imu_to_cloud = best_frame;
    tilt_calibration_.frame_name = best_name;
    tilt_calibration_.frame_score = best;
    tilt_calibration_.frame_runner_up = runner_up;
    tilt_calibration_.frame_detected = true;
    tilt_calibration_.residual_before_pivot =
        calibrationResidual(calibration_scans, Eigen::Vector3f::Zero());

    if (tied.size() > 1)
    {
        std::cout << "[tilt] ";
        for (size_t i = 0; i < tied.size(); ++i)
            std::cout << (i ? " and " : "") << tied[i];
        std::cout << " are indistinguishable here (they differ only about the hinge"
                     " axis itself) - either describes this rig equally well\n";
    }

    // Refine the winner's pivot properly now that the frame is settled.
    Eigen::Vector3f refined;
    const double refined_score = searchTiltPivot(calibration_scans, /*levels=*/4, refined);

    tilt_calibration_.pivot = refined;
    tilt_calibration_.residual = refined_score;
    tilt_calibration_.valid = true;
    tilt_calibration_.samples = static_cast<int>(calibration_scans.size());

    std::cout << "[tilt] using " << best_name << " with pivot ("
              << std::setprecision(3) << refined.x() << ", " << refined.y() << ", "
              << refined.z() << ") m   score " << std::setprecision(4) << refined_score << "\n";
}

// -------------------------------------------------------------------------
void FusionSession::runTiltCalibrationStage(const ProgressFn& progress, bool sign_retry)
{
    if (!params_.calibrate_tilt_geometry)
    {
        std::cout << "[tilt] calibration disabled - seeds rotate about the configured pivot\n";
        return;
    }
    if (!rig_known_)
        return;

    // The IMU path is checked *before* the tilt-axis guard below, because it
    // needs no configured axis - it measures the axis from gravity. Gating it
    // on --tilt-axis would silently skip frame detection and the pivot search
    // for anyone who (reasonably) omitted a flag the IMU path never reads.
    if (use_imu_tilt_ && countWithGravity() > 1)
    {
        // The IMU supplies the tilt *rotation*, but two things still have to
        // be established before that rotation can be used: which frame the
        // point clouds are written in relative to the IMU, and where the tilt
        // axis passes through. Both are found by trying candidates and
        // keeping what actually makes the tilted scans line up, rather than
        // by fitting the commanded-angle model this stage used to.
        const auto calibration_scans = selectTiltCalibrationScans();
        if (calibration_scans.size() < 2)
        {
            std::cout << "[tilt] need at least 2 tilted scans with gravity readings to"
                         " calibrate (found " << calibration_scans.size() << ")\n";
            return;
        }

        prepareCalibrationCaches(calibration_scans);

        if (progress) progress(0, 1, "frame and pivot");
        calibrateImuTiltGeometry(calibration_scans);

        applyRotationAngles(0);
        return;
    }
    if (reference_index_ >= scans_.size()) return;

    if (!rig_.seedsTilt())
    {
        std::cout << "[tilt] no IMU readings and no --tilt-axis - tilt is left to"
                     " feature matching and ICP\n";
        return;
    }

    const auto& reference = scans_[reference_index_];
    const int reference_tilt = reference.input.has_tilt ? reference.input.tilt_degrees : 0;

    // Calibrate within the reference's own rotation group: mixing groups
    // would fold the turntable's geometry into the tilt fit.
    const int ref_group = rotationGroupOf(reference_index_);

    // Candidates: low tilts only. Their seed error is small enough that the
    // current (uncalibrated) seed still lands ICP correctly - which is what
    // makes them usable as ground truth for the fit.
    std::vector<size_t> candidates;
    for (size_t i = 0; i < scans_.size(); ++i)
    {
        if (i == reference_index_) continue;
        const auto& scan = scans_[i];
        if (!scan.loaded() || scan.status == ScanStatus::Excluded) continue;
        if (!scan.input.has_tilt) continue;
        if (rotationGroupOf(i) != ref_group) continue;
        if (std::abs(scan.input.tilt_degrees - reference_tilt) > params_.tilt_calibration_max_deg)
            continue;
        candidates.push_back(i);
    }

    std::sort(candidates.begin(), candidates.end(), [this, reference_tilt](size_t a, size_t b) {
        return std::abs(scans_[a].input.tilt_degrees - reference_tilt) <
               std::abs(scans_[b].input.tilt_degrees - reference_tilt);
    });
    if (candidates.size() > static_cast<size_t>(params_.tilt_calibration_max_scans))
        candidates.resize(static_cast<size_t>(params_.tilt_calibration_max_scans));

    if (candidates.size() < 2)
    {
        std::cout << "[tilt] need at least 2 low-tilt scans to calibrate against (found "
                  << candidates.size() << ") - leaving the tilt geometry as configured\n";
        return;
    }

    // Align each candidate properly, smallest tilt first, against a map that
    // grows as we go - same ordering logic as the main batch, for the same
    // reason: each scan gets the most surface possible to match against.
    VoxelMap map(mapLeafSize());
    map.add(*reference.prepared, reference.normals.get(), reference.transform);

    const Eigen::Matrix4f reference_pose = reference.transform;
    const Eigen::Matrix4f reference_pose_inv = reference_pose.inverse();

    std::vector<Eigen::Matrix3f> model_rotations;
    std::vector<Eigen::Vector3f> observed_translations;
    std::vector<double> angle_ratios;
    std::vector<Eigen::Vector3f> observed_axes;

    for (size_t n = 0; n < candidates.size(); ++n)
    {
        const size_t index = candidates[n];
        auto& scan = scans_[index];
        if (progress) progress(n, candidates.size(), scan.label());

        const auto icp = refineICP(scan.cloudWithNormals(), map.cloudWithNormals(),
                                   scan.input.initial_guess, params_);
        if (!icp.converged)
        {
            std::cerr << "  [!] " << scan.label() << " did not converge - not using it\n";
            continue;
        }
        const auto quality = evaluateAlignment(scan.prepared, map.cloud(), icp.transform, params_);
        if (quality.inlier_fraction < 0.5)
        {
            std::cerr << "  [!] " << scan.label() << " only reached "
                      << pct(quality.inlier_fraction) << " overlap - not trusting it\n";
            continue;
        }

        // How this scan actually moved relative to the reference, expressed
        // in the reference camera's frame.
        const Eigen::Matrix4f relative = reference_pose_inv * icp.transform;
        const Eigen::Matrix3f R_observed = relative.block<3, 3>(0, 0);
        const Eigen::Vector3f t_observed = relative.block<3, 1>(0, 3);

        const double delta_deg = static_cast<double>(scan.input.tilt_degrees - reference_tilt);
        const Eigen::Matrix4f model = rotationAboutAxis(
            rig_.tilt_axis, rig_.tilt_sign * static_cast<float>(delta_deg));

        model_rotations.push_back(model.block<3, 3>(0, 0));
        observed_translations.push_back(t_observed);

        // Cross-check the orientation independently of the pivot: what axis
        // did it really turn about, and by how much per commanded degree?
        const Eigen::AngleAxisf observed_aa(R_observed);
        const double observed_deg = observed_aa.angle() * 180.0 / M_PI;
        const Eigen::Vector3f axis = observed_aa.axis();
        if (observed_deg > 1e-3 && std::abs(delta_deg) > 1e-6)
        {
            observed_axes.push_back(axis);

            // AngleAxis always reports a positive angle with the direction
            // folded into the axis, so recover the signed rotation about the
            // axis we configured. Without this a backwards tilt_sign is
            // invisible: the magnitudes match perfectly either way.
            const float orientation = axis.dot(configuredTiltAxis()) < 0.0f ? -1.0f : 1.0f;
            const double observed_signed = observed_deg * orientation;
            const double expected_signed = rig_.tilt_sign * delta_deg;
            angle_ratios.push_back(observed_signed / expected_signed);
        }

        // Fold it in so the next candidate matches against more surface.
        map.add(*scan.prepared, scan.normals.get(), icp.transform);
    }

    if (model_rotations.size() < 2)
    {
        std::cout << "[tilt] not enough usable observations to calibrate\n";
        return;
    }

    Eigen::Vector3f pivot;
    double residual = 0.0;
    if (!solvePivot(model_rotations, observed_translations, pivot, residual))
        return;

    tilt_calibration_.valid = true;
    tilt_calibration_.samples = static_cast<int>(model_rotations.size());
    tilt_calibration_.pivot = pivot;
    tilt_calibration_.residual = residual;

    // Orientation cross-checks, averaged over the observations.
    if (!angle_ratios.empty())
    {
        double sum = 0.0;
        for (double r : angle_ratios) sum += r;
        tilt_calibration_.angle_scale = sum / static_cast<double>(angle_ratios.size());
        tilt_calibration_.sign_looks_flipped = tilt_calibration_.angle_scale < 0.0;

        Eigen::Vector3f axis_sum = Eigen::Vector3f::Zero();
        for (const auto& a : observed_axes)
        {
            // Hemisphere-align before averaging, or opposite-signed tilts
            // would cancel each other out.
            axis_sum += (axis_sum.dot(a) < 0.0f) ? -a : a;
        }
        if (axis_sum.norm() > 1e-6f)
        {
            tilt_calibration_.observed_axis = axis_sum.normalized();
            const float dot = std::abs(tilt_calibration_.observed_axis.dot(configuredTiltAxis()));
            tilt_calibration_.axis_error_deg =
                std::acos(std::min(1.0f, dot)) * 180.0 / M_PI;
        }
    }

    // What this buys, at the largest tilt actually present.
    double worst = 0.0;
    for (const auto& scan : scans_)
        if (scan.input.has_tilt)
            worst = std::max(worst, std::abs(static_cast<double>(scan.input.tilt_degrees - reference_tilt)));
    tilt_calibration_.worst_tilt_deg = worst;

    const Eigen::Matrix4f worst_model =
        rotationAboutAxis(rig_.tilt_axis, rig_.tilt_sign * static_cast<float>(worst));
    const Eigen::Matrix3f Rw = worst_model.block<3, 3>(0, 0);
    tilt_calibration_.seed_error_before = ((Eigen::Matrix3f::Identity() - Rw) * pivot).norm();
    tilt_calibration_.seed_error_after = residual;

    // A backwards tilt_sign is partly absorbable by mirroring the pivot, so
    // the fit alone looks plausible. Catch it on the signed rotation instead,
    // correct it, and measure again with the seeds pointing the right way -
    // the same try-both-and-keep-what-fits approach used for rotation.
    if (tilt_calibration_.sign_looks_flipped && !sign_retry)
    {
        std::cout << "[tilt] observed rotation runs opposite to the commanded angle ("
                  << tilt_calibration_.angle_scale
                  << " deg/deg) - flipping tilt_sign to "
                  << -rig_.tilt_sign << " and re-measuring\n";
        rig_.tilt_sign = -rig_.tilt_sign;
        tilt_calibration_ = TiltCalibration{};
        applyRotationAngles(0);
        runTiltCalibrationStage(progress, /*sign_retry=*/true);
        return;
    }

    // Only adopt a fit that actually explains the motion. A lever arm that
    // leaves most of the error unaccounted for is not a measurement of
    // anything - adopting it anyway (as this used to) replaces a mildly
    // wrong seed with a confidently wrong one, which is worse.
    const bool explains_motion =
        tilt_calibration_.residual < 0.5 * tilt_calibration_.seed_error_before &&
        tilt_calibration_.axis_error_deg < 10.0 &&
        std::abs(std::abs(tilt_calibration_.angle_scale) - 1.0) < 0.15;

    if (!explains_motion)
    {
        tilt_calibration_.rejected = true;
        std::cout << "[tilt] fit rejected (residual " << mm(tilt_calibration_.residual)
                  << "mm, axis off by " << tilt_calibration_.axis_error_deg
                  << " deg, scale " << tilt_calibration_.angle_scale
                  << ") - keeping the configured tilt geometry\n";
        return;
    }

    // Adopt it, and rebuild every scan's seed with the measured geometry.
    rig_.tilt_pivot = pivot;
    applyRotationAngles(0);

    std::cout << "[tilt] pivot = (" << pivot.x() << ", " << pivot.y() << ", " << pivot.z()
              << ") m from the optical centre, residual " << mm(residual) << "mm over "
              << tilt_calibration_.samples << " scan(s)\n";
}

// -------------------------------------------------------------------------
void FusionSession::printTiltCalibrationReport() const
{
    const auto& c = tilt_calibration_;
    if (c.rejected)
        std::cout << "\nTilt geometry: measured but REJECTED - the lever-arm model did not\n"
                     "  explain how the scans actually moved, so seeds are unchanged.\n";
    if (!c.valid)
    {
        std::cout << "\nTilt geometry: not calibrated (seeds rotate about "
                  << (rig_.tilt_pivot.isZero() ? "the optical centre"
                                                : "the configured pivot") << ")\n";
        return;
    }

    if (c.frame_detected)
    {
        std::cout << "\nTilt geometry (measured from " << c.samples << " tilted scan(s)):\n";
        std::cout << "  IMU -> cloud frame  " << c.frame_name
                  << "   (score " << std::fixed << std::setprecision(4) << c.frame_score
                  << " vs " << c.frame_runner_up << " for the next best)\n";
        std::cout << "  tilt pivot          (" << std::setprecision(3) << c.pivot.x() << ", "
                  << c.pivot.y() << ", " << c.pivot.z() << ") m\n";
        std::cout << "  lever arm           " << mm(c.pivot.norm())
                  << " mm from the optical centre\n";
        std::cout << "  alignment score     " << std::setprecision(4)
                  << c.residual_before_pivot << "  ->  " << c.residual
                  << "  (pivot at origin -> pivot found)\n";
        if (c.frame_name.rfind("identity", 0) != 0)
            std::cout << "  [i] the point clouds are not in the IMU's frame - typical of a\n"
                         "      PLY exported by the RealSense Viewer, which negates y and z\n";
        return;
    }

    std::cout << "\nTilt geometry (measured from " << c.samples << " low-tilt scan(s)):\n";
    std::cout << "  pivot offset      (" << c.pivot.x() << ", " << c.pivot.y()
              << ", " << c.pivot.z() << ") m\n";
    std::cout << "  lever arm         " << mm(c.pivot.norm()) << " mm from the optical centre\n";
    std::cout << "  fit residual      " << mm(c.residual) << " mm\n";
    std::cout << "  seed error at " << c.worst_tilt_deg << " deg:  "
              << mm(c.seed_error_before) << " mm  ->  " << mm(c.seed_error_after) << " mm"
              << "   (ICP correspondence distance is "
              << mm(params_.icp_max_correspondence_distance) << " mm)\n";
    std::cout << "  observed axis     (" << c.observed_axis.x() << ", " << c.observed_axis.y()
              << ", " << c.observed_axis.z() << "),  "
              << c.axis_error_deg << " deg from the configured tilt axis\n";
    std::cout << "  angle scale       " << c.angle_scale
              << " deg observed per deg commanded\n";

    // Diagnostics for the cases a pivot cannot explain.
    if (c.axis_error_deg > 10.0)
        std::cout << "  [!] the scans rotate about a noticeably different axis than configured"
                     " - check --tilt-axis\n";
    if (c.sign_looks_flipped)
        std::cout << "  [!] the scans rotate opposite to the commanded tilt"
                     " - tilt_sign is backwards\n";
    else if (std::abs(std::abs(c.angle_scale) - 1.0) > 0.15)
        std::cout << "  [!] the observed rotation doesn't match the commanded angle"
                     " - the recorded tilt angles may not be what the mount actually did\n";
    if (c.residual > 0.5 * c.seed_error_before && c.seed_error_before > 0.005)
        std::cout << "  [!] a lever arm doesn't explain most of the error here"
                     " - suspect the tilt axis or sign rather than the pivot\n";
}

// -------------------------------------------------------------------------
AlignmentQuality FusionSession::scoreAlignment(const PointCloudT::ConstPtr& source,
                                                 const PointCloudT::ConstPtr& target,
                                                 const Eigen::Matrix4f& transform) const
{
    // On a flat scene, nearest-neighbour distance is nearly blind to sliding
    // or spinning within the plane - the nearest point of a sheet is close
    // whatever you do to it laterally. Comparing heights at matching lateral
    // positions sees exactly that error.
    if (scene_plane_.isPlanar(params_.planarity_threshold))
        return evaluateAlignmentOnPlane(source, target, transform, scene_plane_, params_);
    return evaluateAlignment(source, target, transform, params_);
}

// -------------------------------------------------------------------------
// Places each group's representative scan at the cumulative angles implied
// by `signs` and returns how well the whole arrangement fits. Lower is
// better. This is the objective the joint direction search minimises.
double FusionSession::scoreDirectionHypothesis(const std::vector<size_t>& representatives,
                                                 const std::vector<float>& signs,
                                                 std::vector<AlignmentQuality>* per_group)
{
    VoxelMap map(mapLeafSize());
    {
        const auto& ref = scans_[representatives[0]];
        map.add(*ref.prepared, nullptr, ref.transform);  // scoring reads no normals
    }

    double total = 0.0;
    double cumulative = rotation_groups_[0].cumulative_deg;

    for (size_t k = 1; k < representatives.size(); ++k)
    {
        if (rotation_groups_[k].from_tags)
            cumulative = rotation_groups_[k].cumulative_deg;
        else
            cumulative += signs[k - 1] * rotation_groups_[k].nominal_step_deg;

        const auto& scan = scans_[representatives[k]];
        const Eigen::Matrix4f guess =
            rotationAboutPoint(rig_.rotation_axis, static_cast<float>(cumulative),
                                rig_.rotation_pivot) *
            tiltComponentFor(scan, &rotation_groups_[k]);

        const auto quality = scoreAlignment(scan.prepared, map.cloud(), guess);
        if (per_group) per_group->push_back(quality);

        // Cap rather than propagate infinity, so one hopeless step doesn't
        // make every hypothesis containing it indistinguishable.
        const double score = quality.valid() ? std::min(quality.score(), 10.0) : 10.0;
        total += score;

        map.add(*scan.prepared, nullptr, guess);
    }
    return total;
}

// -------------------------------------------------------------------------
void FusionSession::runRotationDirectionStage(const ProgressFn& progress)
{
    if (!params_.infer_rotation_direction)
    {
        std::cout << "[rotation] inference disabled - using the recorded directions as-is\n";
        return;
    }
    if (!rig_known_ || !rig_.seedsRotation())
    {
        std::cout << "[rotation] no rotation axis configured - nothing to infer\n";
        return;
    }
    if (rotation_groups_.size() < 2)
    {
        std::cout << "[rotation] only " << rotation_groups_.size()
                  << " rotation group(s) - nothing to infer\n";
        return;
    }
    if (reference_index_ >= scans_.size()) return;

    rotation_groups_[0].resolved = true;
    for (auto& group : rotation_groups_) group.from_tags = false;
    applyRotationAngles(0);

    if (params_.use_tag_alignment)
        resolveRotationFromTags();

    // One representative per group - its lowest-tilt scan, which shares the
    // most surface with the others and so discriminates most sharply.
    std::vector<size_t> representatives;
    for (auto& group : rotation_groups_)
    {
        size_t rep = scans_.size();
        int best_tilt = std::numeric_limits<int>::max();
        for (size_t index : group.scans)
        {
            const auto& scan = scans_[index];
            if (!scan.loaded() || scan.status == ScanStatus::Excluded) continue;
            const int tilt = scan.input.has_tilt ? std::abs(scan.input.tilt_degrees) : 999;
            if (tilt < best_tilt) { best_tilt = tilt; rep = index; }
        }
        if (rep >= scans_.size())
        {
            std::cerr << "  [!] " << group.name << " has no loadable scan - cannot infer\n";
            return;
        }
        representatives.push_back(rep);
    }

    const size_t steps = rotation_groups_.size() - 1;

    // Only the steps the tags did not settle are left to search.
    std::vector<size_t> free_steps;
    for (size_t k = 0; k < steps; ++k)
        if (!rotation_groups_[k + 1].from_tags) free_steps.push_back(k);
    if (free_steps.empty())
    {
        std::cout << "[rotation] every rotation group was placed by the tags - no search needed\n";
        applyRotationAngles(1);
        return;
    }

    for (size_t k = 1; k < rotation_groups_.size(); ++k)
    {
        if (rotation_groups_[k].from_tags) continue;
        if (rotation_groups_[k].nominal_step_deg == 0.0)
        {
            std::cout << "  [i] " << rotation_groups_[k].name
                      << " records no step from the previous group - nothing to resolve\n";
            rotation_groups_[k].resolved = true;
        }
    }

    std::vector<float> best_signs(steps, 1.0f);
    double best_total = std::numeric_limits<double>::max();
    double runner_up = std::numeric_limits<double>::max();

    if (free_steps.size() <= static_cast<size_t>(params_.direction_max_joint_steps))
    {
        // Joint search: every combination of directions, scored as a whole.
        const size_t combinations = size_t(1) << free_steps.size();
        std::cout << "[rotation] searching " << combinations
                  << " direction combination(s) across " << free_steps.size() << " step(s)\n";

        for (size_t mask = 0; mask < combinations; ++mask)
        {
            if (progress) progress(mask, combinations, "direction search");

            std::vector<float> signs(steps, 1.0f);
            for (size_t b = 0; b < free_steps.size(); ++b)
                signs[free_steps[b]] = (mask & (size_t(1) << b)) ? -1.0f : 1.0f;

            const double total = scoreDirectionHypothesis(representatives, signs, nullptr);
            if (total < best_total)
            {
                runner_up = best_total;
                best_total = total;
                best_signs = signs;
            }
            else if (total < runner_up)
            {
                runner_up = total;
            }
        }
    }
    else
    {
        // Too many steps to enumerate: settle each in turn, as before.
        std::cout << "[rotation] " << free_steps.size() << " steps is beyond the joint-search limit ("
                  << params_.direction_max_joint_steps << ") - resolving greedily\n";
        for (size_t k : free_steps)
        {
            std::vector<float> plus = best_signs, minus = best_signs;
            plus[k] = 1.0f; minus[k] = -1.0f;
            const double sp = scoreDirectionHypothesis(representatives, plus, nullptr);
            const double sm = scoreDirectionHypothesis(representatives, minus, nullptr);
            best_signs[k] = (sp <= sm) ? 1.0f : -1.0f;
            best_total = std::min(sp, sm);
            runner_up = std::max(sp, sm);
        }
    }

    // Record the winner, and per-step quality for the report.
    std::vector<AlignmentQuality> qualities;
    scoreDirectionHypothesis(representatives, best_signs, &qualities);

    for (size_t k = 1; k < rotation_groups_.size(); ++k)
    {
        auto& group = rotation_groups_[k];
        if (group.from_tags) continue;
        group.step_sign = best_signs[k - 1];
        group.resolved = true;
        if (k - 1 < qualities.size()) group.chosen_quality = qualities[k - 1];
    }

    // Confidence has to be measured per step, not against the runner-up
    // arrangement overall: two arrangements differing only in the last step
    // score almost identically because every earlier placement is shared,
    // which would make even a well-determined early step look like a coin
    // toss. So ask the question that actually matters for each step - how
    // much worse does everything get if just this one is flipped?
    for (size_t k : free_steps)
    {
        std::vector<float> flipped = best_signs;
        flipped[k] = -flipped[k];
        const double flipped_total = scoreDirectionHypothesis(representatives, flipped, nullptr);

        double margin = 0.0;
        if (std::isfinite(flipped_total) && flipped_total > 0.0)
            margin = (flipped_total - best_total) / flipped_total;

        auto& group = rotation_groups_[k + 1];
        group.margin = std::max(0.0, margin);
        group.ambiguous = group.margin < params_.direction_ambiguous_margin;
    }

    applyRotationAngles(1);

    std::cout << "[rotation] best arrangement scores " << std::fixed << std::setprecision(5)
              << best_total << ", next best " << runner_up << "\n";
}

// -------------------------------------------------------------------------
void FusionSession::printRotationReport() const
{
    if (rotation_groups_.size() < 2) return;

    std::cout << "\nRotation directions:\n";
    std::cout << "  group                          step      cumulative  margin  scans\n";
    std::cout << "  -----------------------------  --------  ----------  ------  -----\n";
    for (size_t k = 0; k < rotation_groups_.size(); ++k)
    {
        const auto& g = rotation_groups_[k];
        std::ostringstream step;
        if (k == 0)           step << "(frame)";
        else if (g.from_tags) step << "tags";
        else                  step << (g.step_sign > 0 ? "+" : "-") << g.nominal_step_deg;

        std::ostringstream margin;
        if (k == 0)           margin << "-";
        else if (g.from_tags) margin << "+-" << std::fixed << std::setprecision(1) << g.tag_spread_deg;
        else                  margin << pct(g.margin);

        std::string short_name = g.name;
        if (short_name.size() > 29) short_name = short_name.substr(short_name.size() - 29);
        std::cout << "  " << std::left << std::setw(29) << short_name << std::right
                  << "  " << std::setw(8) << step.str()
                  << "  " << std::setw(10) << g.cumulative_deg
                  << "  " << std::setw(6) << margin.str()
                  << "  " << std::setw(5) << g.scans.size();
        if (g.ambiguous) std::cout << "   [AMBIGUOUS]";
        std::cout << "\n";
    }
    std::cout << "  (a low margin means both directions fit about as well - "
                 "check those visually and press t to flip;\n"
                 "   'tags' groups were measured from the AprilTags, margin = spread in degrees)\n";
}

// -------------------------------------------------------------------------
void FusionSession::runCoarseBatch(const ProgressFn& progress)
{
    if (reference_index_ >= scans_.size())
    {
        std::cerr << "  [!] no loadable reference scan - nothing to align against\n";
        return;
    }

    if (params_.use_tag_alignment) solveTagPoses();

    // Running map, added to as scans are placed rather than rebuilt per scan.
    VoxelMap map(mapLeafSize());
    {
        const auto& ref = scans_[reference_index_];
        map.add(*ref.prepared, ref.normals.get(), ref.transform);
    }

    for (size_t i = 0; i < scans_.size(); ++i)
    {
        auto& scan = scans_[i];
        if (i == reference_index_) continue;
        if (!scan.loaded() || scan.status == ScanStatus::Excluded) continue;

        if (progress) progress(i, scans_.size(), scan.label());

        computeCoarseFor(i, map.cloud());
        // Coarse output *is* the current pose estimate until ICP improves it.
        scan.transform = scan.coarse_transform;

        std::cout << "[coarse] " << scan.label()
                  << "  rmse=" << mm(scan.coarse_quality.rmse) << "mm"
                  << "  overlap=" << pct(scan.coarse_quality.inlier_fraction)
                  << "  via " << (scan.coarse_source.empty() ? "-" : scan.coarse_source)
                  << "  (" << toString(scan.status) << ")\n";

        if (!contributesToFusion(scan.status)) continue;
        map.add(*scan.prepared, scan.normals.get(), scan.transform);
    }
}

// -------------------------------------------------------------------------
void FusionSession::runRefineBatch(const ProgressFn& progress)
{
    if (reference_index_ >= scans_.size()) return;

    VoxelMap map(mapLeafSize());
    {
        const auto& ref = scans_[reference_index_];
        map.add(*ref.prepared, ref.normals.get(), ref.transform);
    }

    for (size_t i = 0; i < scans_.size(); ++i)
    {
        auto& scan = scans_[i];
        if (i == reference_index_) continue;
        if (!scan.loaded()) continue;
        if (scan.status == ScanStatus::Excluded || scan.status == ScanStatus::CoarseFailed)
            continue;

        if (progress) progress(i, scans_.size(), scan.label());

        refineAgainst(i, map);

        if (!contributesToFusion(scan.status)) continue;
        map.add(*scan.prepared, scan.normals.get(), scan.transform);
    }

    updateFlags();
}

// -------------------------------------------------------------------------
float FusionSession::mapLeafSize() const
{
    return params_.downsample_accumulated_map ? params_.voxel_leaf_size : 0.0f;
}

// -------------------------------------------------------------------------
void FusionSession::refineAgainst(size_t index, const VoxelMap& map)
{
    auto& scan = scans_[index];
    scan.icp_result = refineICP(scan.cloudWithNormals(), map.cloudWithNormals(),
                                 scan.coarse_transform, params_);

    if (scan.icp_result.converged)
    {
        scan.transform = scan.icp_result.transform;
        // The same scene-aware measure the coarse stage uses, so the two are
        // comparable - and so a flat scene's in-plane error actually counts
        // against the confidence fusion derives from this.
        scan.fine_quality = scoreAlignment(scan.prepared, map.cloud(), scan.transform);
        scan.status       = ScanStatus::Refined;
    }
    else
    {
        // Keep the coarse pose: it's worse, but it's still where the rig
        // says the scan belongs, and it keeps the scan visible for review
        // rather than silently vanishing.
        scan.transform = scan.coarse_transform;
        scan.status    = ScanStatus::RefineFailed;
    }

    std::cout << "[refine] " << scan.label()
              << "  icp_fitness=" << scan.icp_result.fitness_score
              << "  rmse=" << mm(scan.fine_quality.rmse) << "mm"
              << "  overlap=" << pct(scan.fine_quality.inlier_fraction)
              << (scan.icp_result.hit_max_iterations ? "  [hit iteration cap]" : "")
              << (scan.status == ScanStatus::RefineFailed ? "  [FAILED]" : "") << "\n";
}

// -------------------------------------------------------------------------
void FusionSession::updateFlags()
{
    // Judged against the batch rather than a fixed threshold: what a good
    // score looks like depends on the voxel size, the sensor's noise at this
    // range and the scene's relief, none of which a constant can know. The
    // median is what "normal" looks like for this session, and isn't dragged
    // around by the few scans that are actually bad.
    std::vector<double> scores;
    for (const auto& s : scans_)
        if (s.status == ScanStatus::Refined && s.fine_quality.valid())
            scores.push_back(s.fine_quality.score());

    double median = 0.0;
    if (!scores.empty())
    {
        std::sort(scores.begin(), scores.end());
        median = scores[scores.size() / 2];
    }
    const bool compare_to_batch = scores.size() >= 3;  // fewer, and a median says nothing

    for (auto& s : scans_)
    {
        if (s.status == ScanStatus::RefineFailed) { s.flagged = true; continue; }
        if (s.status != ScanStatus::Refined) continue;

        s.flagged = s.icp_result.hit_max_iterations ||
                    !s.fine_quality.valid() ||
                    (compare_to_batch && s.fine_quality.score() > params_.flag_score_ratio * median);
    }
}

// -------------------------------------------------------------------------
void FusionSession::coarseAlignScan(size_t index)
{
    auto& scan = scans_.at(index);
    if (!scan.loaded() || index == reference_index_) return;

    // Aligning against every *other* placed scan, not just earlier ones -
    // by review time the whole map exists, so a redo gets the benefit of it.
    const auto map = buildMap(index, scans_.size());
    if (map.empty())
    {
        std::cerr << "  [!] nothing placed to align " << scan.label() << " against\n";
        return;
    }
    computeCoarseFor(index, map.cloud());
    scan.transform = scan.coarse_transform;

    std::cout << "[coarse] " << scan.label()
              << "  rmse=" << mm(scan.coarse_quality.rmse) << "mm"
              << "  overlap=" << pct(scan.coarse_quality.inlier_fraction)
              << "  via " << (scan.coarse_source.empty() ? "-" : scan.coarse_source)
              << "  (" << toString(scan.status) << ")\n";
}

// -------------------------------------------------------------------------
void FusionSession::refineScan(size_t index)
{
    auto& scan = scans_.at(index);
    if (!scan.loaded() || index == reference_index_) return;

    const auto map = buildMap(index, scans_.size());
    if (map.empty())
    {
        std::cerr << "  [!] nothing placed to refine " << scan.label() << " against\n";
        return;
    }
    refineAgainst(index, map);
    updateFlags();
}

// -------------------------------------------------------------------------
void FusionSession::setExcluded(size_t index, bool excluded)
{
    auto& scan = scans_.at(index);
    if (excluded)
    {
        scan.status = ScanStatus::Excluded;
        return;
    }
    if (scan.status != ScanStatus::Excluded) return;

    // Un-excluding restores whatever stage the scan had actually reached.
    if (index == reference_index_)          scan.status = ScanStatus::Reference;
    else if (scan.icp_result.converged)     scan.status = ScanStatus::Refined;
    else if (scan.coarse_quality.valid())   scan.status = ScanStatus::CoarseAligned;
    else                                     scan.status = ScanStatus::Pending;
}

// -------------------------------------------------------------------------
void FusionSession::nudgeScan(size_t index, const Eigen::Matrix4f& delta_in_common_frame)
{
    auto& scan = scans_.at(index);
    // Left-multiply: the delta is expressed in the common frame, so it acts
    // on the scan's already-placed points rather than in its camera frame.
    scan.transform        = delta_in_common_frame * scan.transform;
    scan.coarse_transform = scan.transform; // a nudge becomes the new ICP seed
}

// -------------------------------------------------------------------------
size_t FusionSession::countWithStatus(ScanStatus status) const
{
    return static_cast<size_t>(
        std::count_if(scans_.begin(), scans_.end(),
                       [status](const ScanState& s) { return s.status == status; }));
}

size_t FusionSession::countContributing() const
{
    return static_cast<size_t>(
        std::count_if(scans_.begin(), scans_.end(), [](const ScanState& s) {
            return s.loaded() && contributesToFusion(s.status);
        }));
}

// -------------------------------------------------------------------------
PointCloudT::Ptr FusionSession::buildMergedCloud() const
{
    auto merged = std::make_shared<PointCloudT>();
    for (const auto& scan : scans_)
    {
        if (!scan.loaded() || !contributesToFusion(scan.status)) continue;
        PointCloudT transformed;
        pcl::transformPointCloud(*scan.prepared, transformed, scan.transform);
        *merged += transformed;
    }
    return merged;
}

// -------------------------------------------------------------------------
std::vector<ScanContribution> FusionSession::buildContributions() const
{
    std::vector<ScanContribution> contributions;
    for (const auto& scan : scans_)
    {
        if (!scan.loaded() || !contributesToFusion(scan.status)) continue;

        ScanContribution contribution;
        contribution.cloud = scan.cloudInCommonFrame();
        contribution.label = scan.label();

        // A rigid transform turns normals by its rotation alone.
        if (scan.normals && scan.normals->size() == scan.prepared->size())
        {
            auto turned = std::make_shared<NormalCloudT>(*scan.normals);
            const Eigen::Matrix3f rotation = scan.transform.topLeftCorner<3, 3>();
            for (auto& n : *turned)
            {
                const Eigen::Vector3f v = rotation * n.getNormalVector3fMap();
                n.getNormalVector3fMap() = v;
            }
            contribution.normals = turned;
        }

        // The scans were captured in camera frames whose origin is the
        // sensor, so the transform's translation is where the camera stood
        // in the common frame. Reconciliation needs that to know which
        // direction each surface was viewed from.
        contribution.sensor_origin = scan.transform.block<3, 1>(0, 3);

        // Confidence from how well the scan registered. Overlap carries
        // most of the signal - a scan that only matched a third of itself is
        // not to be trusted regardless of how tight that third was - and
        // rmse then discounts scans that fit loosely.
        const AlignmentQuality& q =
            scan.fine_quality.valid() ? scan.fine_quality : scan.coarse_quality;
        if (q.valid())
        {
            const double rmse_reference = params_.eval_correspondence_distance * 0.25;
            const double tightness = rmse_reference / std::max(q.rmse, rmse_reference);
            contribution.confidence = std::clamp(q.inlier_fraction * tightness, 0.02, 1.0);
        }
        else
        {
            // The reference scan defines the frame and has nothing to be
            // scored against, so it is trusted fully.
            contribution.confidence = 1.0;
        }

        contributions.push_back(std::move(contribution));
    }
    return contributions;
}

// -------------------------------------------------------------------------
void FusionSession::printReport(const std::string& title) const
{
    std::cout << "\n" << title << "\n";
    std::cout << "  idx  status         rmse(mm)  overlap  coarse  icp_fitness  points  scan\n";
    std::cout << "  ---  -------------  --------  -------  ------  -----------  ------  ----\n";

    for (size_t i = 0; i < scans_.size(); ++i)
    {
        const auto& s = scans_[i];
        // Report the latest measurement we have: fine if ICP ran, else coarse.
        const AlignmentQuality& q = s.fine_quality.valid() ? s.fine_quality : s.coarse_quality;

        std::cout << "  " << std::setw(3) << i << "  "
                  << std::left << std::setw(13) << toString(s.status) << std::right << "  "
                  << std::setw(8) << mm(q.rmse) << "  "
                  << std::setw(7) << (q.valid() ? pct(q.inlier_fraction) : "-") << "  "
                  << std::setw(6) << (s.coarse_source.empty() ? "-" : s.coarse_source) << "  ";

        if (s.icp_result.converged)
            std::cout << std::setw(11) << std::fixed << std::setprecision(6) << s.icp_result.fitness_score;
        else
            std::cout << std::setw(11) << "-";

        std::cout << "  " << std::setw(6) << (s.prepared ? s.prepared->size() : 0)
                  << "  " << s.label()
                  << (s.flagged ? "   [FLAGGED]" : "") << "\n";
    }

    std::cout << "  " << countContributing() << " of " << scans_.size()
              << " scan(s) currently contributing to the fused cloud\n";
}

// -------------------------------------------------------------------------
void FusionSession::computeScanQc()
{
    // The same batch median updateFlags judges against, so "score_ratio"
    // here and `flagged` there always agree.
    std::vector<double> scores;
    for (const auto& s : scans_)
        if (s.status == ScanStatus::Refined && s.fine_quality.valid())
            scores.push_back(s.fine_quality.score());
    double median = 0.0;
    if (!scores.empty())
    {
        std::sort(scores.begin(), scores.end());
        median = scores[scores.size() / 2];
    }
    const bool compare_to_batch = scores.size() >= 3;

    for (size_t i = 0; i < scans_.size(); ++i)
    {
        auto& s = scans_[i];
        s.qc_reasons.clear();
        s.qc_warnings.clear();
        s.needs_review = false;
        if (s.status == ScanStatus::Excluded || i == reference_index_) continue;

        // Reasons: any one of these sends the session to the review queue.
        switch (s.status)
        {
            case ScanStatus::Pending:      s.qc_reasons.push_back("not_aligned"); break;
            case ScanStatus::CoarseFailed: s.qc_reasons.push_back("coarse_failed"); break;
            case ScanStatus::RefineFailed: s.qc_reasons.push_back("refine_failed"); break;
            case ScanStatus::Refined:
                if (!s.fine_quality.valid())
                    s.qc_reasons.push_back("no_fine_quality");
                else if (compare_to_batch && s.fine_quality.score() > params_.flag_score_ratio * median)
                    s.qc_reasons.push_back("score_ratio");
                break;
            default: break;
        }
        if (s.coarse_quality.valid() &&
            s.coarse_quality.inlier_fraction < params_.qc_min_coarse_inlier_fraction)
            s.qc_reasons.push_back("low_coarse_overlap");
        if (s.has_tag_disagreement && s.tag_disagreement_deg > params_.tag_disagreement_warn_deg)
            s.qc_reasons.push_back("tag_disagreement");
        const int group = rotationGroupOf(i);
        if (group >= 0 && rotation_groups_[group].ambiguous)
            s.qc_reasons.push_back("group_ambiguous");

        // Warnings: recorded, but not on their own a reason to review. At a
        // 0.5 mm scan voxel ICP routinely runs to its iteration cap on scans
        // that end up well placed (Big_Wave 641 mm: 5 such, fused fine).
        if (s.icp_result.hit_max_iterations) s.qc_warnings.push_back("iteration_cap");
        if (s.tag_rejected_rms >= 0.0)       s.qc_warnings.push_back("tag_fit_rejected");

        // A review that saved this scan has accepted it as it stands.
        s.needs_review = !s.qc_reasons.empty() && !s.reviewed;
    }
}

// -------------------------------------------------------------------------
void FusionSession::computeSessionQc(const ReconcileReport& report)
{
    session_flags_.clear();
    if (report.spread_before > params_.qc_max_spread_before) session_flags_.push_back("spread_before_high");
    if (report.spread_after  > params_.qc_max_spread_after)  session_flags_.push_back("spread_after_high");
}

// -------------------------------------------------------------------------
void FusionSession::setSessionFlag(const std::string& flag, bool on)
{
    const auto it = std::find(session_flags_.begin(), session_flags_.end(), flag);
    if (on && it == session_flags_.end()) session_flags_.push_back(flag);
    if (!on && it != session_flags_.end()) session_flags_.erase(it);
}

// -------------------------------------------------------------------------
bool FusionSession::needsReview() const
{
    if (!session_flags_.empty() && reviewed_at_.empty()) return true;
    return std::any_of(scans_.begin(), scans_.end(), [](const ScanState& s) {
        return s.needs_review && s.status != ScanStatus::Excluded;
    });
}

std::string FusionSession::reviewState() const
{
    if (!reviewed_at_.empty()) return "reviewed";
    return needsReview() ? "needs_review" : "clean";
}

void FusionSession::markReviewed(const std::string& timestamp)
{
    reviewed_at_ = timestamp;
    for (auto& s : scans_)
    {
        s.reviewed = true;
        s.reviewed_at = timestamp;
        s.needs_review = false;
    }
}

// -------------------------------------------------------------------------
PointCloudT::Ptr FusionSession::coarseMapExcluding(const std::vector<size_t>& excluded) const
{
    VoxelMap map(params_.retry_coarse_voxel);
    for (size_t j = 0; j < scans_.size(); ++j)
    {
        if (std::find(excluded.begin(), excluded.end(), j) != excluded.end()) continue;
        const auto& o = scans_[j];
        if (o.loaded() && contributesToFusion(o.status)) map.add(*o.prepared, nullptr, o.transform);
    }
    return map.cloud();
}

void FusionSession::coarseAlignThinned(size_t index, const PointCloudT::ConstPtr& coarse_map)
{
    auto& scan = scans_[index];
    // computeCoarseFor reads the scan's own cloud, so it is swapped for a
    // thinned copy for the duration and always put back.
    const PointCloudT::Ptr full = scan.prepared;
    struct Restore { ScanState& s; PointCloudT::Ptr cloud; ~Restore() { s.prepared = cloud; } } restore{scan, full};
    scan.prepared = voxelDownsample(full, params_.retry_coarse_voxel);
    computeCoarseFor(index, coarse_map);
    scan.transform = scan.coarse_transform;
}

// -------------------------------------------------------------------------
void FusionSession::runAutoRetry(size_t max_scans, const ProgressFn& progress)
{
    computeScanQc();

    std::vector<size_t> candidates;
    for (size_t i = 0; i < scans_.size(); ++i)
    {
        const auto& s = scans_[i];
        if (i != reference_index_ && s.loaded() && s.status != ScanStatus::Excluded && s.needs_review)
            candidates.push_back(i);
    }
    if (candidates.empty())
    {
        std::cout << "[retry] nothing flagged - no retries needed\n";
        return;
    }

    // Worst first: outright failures, then by how badly the current pose
    // scores, so a capped budget is spent where it matters most.
    auto badness = [this](size_t i) {
        const auto& s = scans_[i];
        if (s.status == ScanStatus::RefineFailed || s.status == ScanStatus::CoarseFailed)
            return std::numeric_limits<double>::max();
        const auto& q = s.fine_quality.valid() ? s.fine_quality : s.coarse_quality;
        return q.score();
    };
    std::stable_sort(candidates.begin(), candidates.end(),
                     [&](size_t a, size_t b) { return badness(a) > badness(b); });
    if (candidates.size() > max_scans)
    {
        std::cout << "[retry] " << candidates.size() << " scans flagged; retrying the worst "
                  << max_scans << " (--auto-retry-max-scans), the rest go to review as they are\n";
        candidates.resize(max_scans);
    }

    // Restores a scan's registration state but keeps its retry log.
    auto restore = [this](size_t i, const ScanState& saved) {
        auto log = std::move(scans_[i].retries);
        scans_[i] = saved;
        scans_[i].retries = std::move(log);
    };

    std::vector<bool> group_tried(rotation_groups_.size(), false);

    for (size_t k = 0; k < candidates.size(); ++k)
    {
        const size_t i = candidates[k];
        auto& scan = scans_[i];
        if (progress) progress(k, candidates.size(), scan.label());
        if (!scan.needs_review) continue;   // cleared by an earlier group flip

        // One map of every other scan, shared by every rung, so the scores
        // they are kept or rejected on are directly comparable.
        const VoxelMap map = buildMap(i, scans_.size());
        if (map.empty()) continue;
        const auto map_cloud = map.cloud();
        const auto coarse_map_cloud = coarseMapExcluding({i});
        auto score = [&]() { return scoreAlignment(scan.prepared, map_cloud, scan.transform).score(); };

        auto placeAgainstMap = [&]() {
            coarseAlignThinned(i, coarse_map_cloud);
            if (scan.status == ScanStatus::CoarseAligned) refineAgainst(i, map);
        };

        auto attempt = [&](const std::string& rung, const std::function<void()>& run) {
            const ScanState saved = scan;
            ScanState::RetryAttempt a;
            a.rung = rung;
            a.score_before = score();
            const auto started = std::chrono::steady_clock::now();
            run();
            const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
            a.score_after = score();
            a.kept = contributesToFusion(scan.status) && a.score_after < a.score_before;
            if (!a.kept) restore(i, saved);
            updateFlags();
            computeScanQc();
            a.cleared = a.kept && scan.qc_reasons.empty();
            scan.retries.push_back(a);
            std::cout << "[retry] " << scan.label() << "  " << rung << ": score "
                      << mm(a.score_before) << " -> " << mm(a.score_after)
                      << (a.kept ? "  kept" : "  rejected")
                      << (scan.qc_reasons.empty() ? "  (QC clear)" : "")
                      << "  [" << std::fixed << std::setprecision(0) << seconds << " s]\n";
            return scan.qc_reasons.empty();
        };

        if (attempt("coarse_full_map", placeAgainstMap)) continue;

        if (params_.use_tag_alignment && scan.tag_fit.valid)
        {
            const bool cleared = attempt("no_tags", [&]() {
                const TagFit tags = scan.tag_fit;
                scan.tag_fit.valid = false;
                placeAgainstMap();
                scan.tag_fit = tags;
            });
            if (cleared) continue;
        }

        const int g = rotationGroupOf(i);
        const bool group_suspect = g > 0 &&
            (rotation_groups_[g].ambiguous ||
             (scan.has_tag_disagreement && scan.tag_disagreement_deg > params_.tag_disagreement_warn_deg));
        if (!group_suspect || group_tried[g]) continue;
        group_tried[g] = true;

        // Flipping group g moves it and every later group, so all of those
        // are re-placed, each against a fixed map of the groups that do not
        // move - the same map before and after, so the sums compare.
        std::vector<size_t> moved;
        for (size_t h = static_cast<size_t>(g); h < rotation_groups_.size(); ++h)
            for (size_t j : rotation_groups_[h].scans)
                if (j != reference_index_ && scans_[j].loaded() && scans_[j].status != ScanStatus::Excluded)
                    moved.push_back(j);
        VoxelMap fixed(mapLeafSize());
        for (size_t j = 0; j < scans_.size(); ++j)
        {
            const auto& o = scans_[j];
            if (std::find(moved.begin(), moved.end(), j) != moved.end()) continue;
            if (o.loaded() && contributesToFusion(o.status))
                fixed.add(*o.prepared, o.normals.get(), o.transform);
        }
        if (fixed.empty()) continue;
        const auto fixed_cloud = fixed.cloud();
        const auto fixed_coarse_cloud = coarseMapExcluding(moved);
        auto groupScore = [&]() {
            double total = 0.0;
            for (size_t j : moved)
            {
                const auto q = scoreAlignment(scans_[j].prepared, fixed_cloud, scans_[j].transform);
                total += q.valid() ? std::min(q.score(), 10.0) : 10.0;  // capped as in the direction test
            }
            return total;
        };

        GroupRetry gr;
        gr.group = rotation_groups_[g].name;
        gr.score_before = groupScore();
        const auto saved_groups = rotation_groups_;
        std::vector<ScanState> saved_scans;
        for (size_t j : moved) saved_scans.push_back(scans_[j]);

        flipRotationGroup(static_cast<size_t>(g));
        for (size_t j : moved)
        {
            coarseAlignThinned(j, fixed_coarse_cloud);
            if (scans_[j].status == ScanStatus::CoarseAligned) refineAgainst(j, fixed);
        }
        gr.score_after = groupScore();
        gr.kept = gr.score_after < gr.score_before;
        if (!gr.kept)
        {
            rotation_groups_ = saved_groups;
            for (size_t m = 0; m < moved.size(); ++m) restore(moved[m], saved_scans[m]);
        }
        updateFlags();
        computeScanQc();

        ScanState::RetryAttempt a;
        a.rung = "flip_group";
        a.score_before = gr.score_before;
        a.score_after = gr.score_after;
        a.kept = gr.kept;
        a.cleared = gr.kept && scan.qc_reasons.empty();
        scan.retries.push_back(a);
        group_retries_.push_back(gr);
        std::cout << "[retry] flip " << gr.group << ": summed score " << std::fixed << std::setprecision(4)
                  << gr.score_before << " -> " << gr.score_after << (gr.kept ? "  kept" : "  rejected") << "\n";
    }

    updateFlags();
    computeScanQc();
    size_t still = 0;
    for (const auto& s : scans_) still += s.needs_review ? 1 : 0;
    std::cout << "[retry] " << still << " scan(s) still need review after retries\n";
}
