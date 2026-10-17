#pragma once

#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "fusion_pipeline.h"
#include "tag_alignment.h"

// Where a scan currently sits in the pipeline. Every scan carries its own
// status rather than the pipeline tracking "which index am I on", so that
// batch stages can re-run individual scans out of order and the review UI
// can address scans by identity.
enum class ScanStatus
{
    Pending,        // loaded, nothing aligned yet
    Reference,      // seeds the common frame; never aligned to anything
    CoarseAligned,  // has a usable coarse transform
    CoarseFailed,   // coarse alignment did not converge / scored hopelessly
    Refined,        // ICP-refined transform available
    RefineFailed,   // ICP did not converge
    Excluded,       // deliberately left out of the fused cloud by the user
};

const char* toString(ScanStatus status);

// Inverse of toString, for statuses read back from a poses file.
bool parseScanStatus(const std::string& text, ScanStatus& out);

// True for statuses whose transform is good enough to contribute points.
bool contributesToFusion(ScanStatus status);

// How well a transformed source cloud sits on a target cloud. Kept
// separate from RegistrationResult (which is "what the registration
// algorithm reported about itself") because we also want to score
// transforms we did not compute - e.g. a rig-geometry seed, or the
// mirrored rotation hypothesis in the direction test - on equal footing.
struct AlignmentQuality
{
    // RMS distance from source points to their nearest target point,
    // counting only pairs within the correspondence distance. Metres.
    double rmse = std::numeric_limits<double>::max();

    // Fraction of sampled source points that found *any* target point
    // within the correspondence distance. This is the overlap measure:
    // rmse alone is misleading because a badly-placed cloud can have a
    // tiny rmse over the three points that happen to touch.
    double inlier_fraction = 0.0;

    size_t inliers = 0;
    size_t sampled = 0;

    bool valid() const { return sampled > 0; }

    // Single comparable score, lower is better. Combines the two above so
    // hypotheses can be ranked directly: an alignment covering more of the
    // cloud beats a tighter one covering almost none of it.
    double score() const
    {
        if (!valid() || inlier_fraction <= 0.0)
            return std::numeric_limits<double>::max();
        return rmse / inlier_fraction;
    }
};

// Everything the pipeline knows about one scan. Owns its prepared cloud so
// that stages can be re-run without touching disk again.
struct ScanState
{
    ScanInput input;

    // Downsampled + outlier-filtered cloud in the scan's own camera frame.
    // The raw cloud is deliberately *not* retained: at ~400k points per
    // RealSense frame, keeping every raw scan resident costs hundreds of MB
    // for a large session. Re-cropping the ROI reloads from disk instead.
    PointCloudT::Ptr prepared;

    // Surface normals of `prepared`, one per point, in the camera frame and
    // oriented toward the sensor. Estimated once at load and carried through
    // `transform` wherever the scan is placed, so neither registration nor
    // fusion has to re-estimate normals on clouds that grow with the session.
    NormalCloudT::Ptr normals;

    size_t raw_point_count = 0;

    // Region of interest in the scan's own frame, applied at load time.
    // Persisted here so a reload re-applies the same crop automatically.
    bool            has_roi = false;
    Eigen::Vector4f roi_min = Eigen::Vector4f::Zero();
    Eigen::Vector4f roi_max = Eigen::Vector4f::Zero();

    // Transform carrying `prepared` into the common (reference) frame.
    // coarse_transform is the seed handed to ICP; transform is the current
    // best estimate and is what fusion actually uses.
    Eigen::Matrix4f coarse_transform = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transform        = Eigen::Matrix4f::Identity();

    // Quality of coarse_transform and of transform respectively, both
    // measured against the accumulated map the scan was aligned to.
    AlignmentQuality coarse_quality;
    AlignmentQuality fine_quality;

    // Which hypothesis coarse_transform came from: "seed", "fpfh" or "tags".
    std::string coarse_source;

    // AprilTag corners located in the raw (uncropped) cloud at load time, in
    // the scan's own frame. Empty unless tag alignment is on.
    TagSet tags;

    // This scan's pose in the common frame from the tags alone, chained
    // through shared tags back to the reference (see solveTagPoses).
    // tag_fit.valid is false when no chain reaches it.
    TagFit tag_fit;

    // What ICP itself reported, for cross-checking against fine_quality.
    RegistrationResult icp_result;

    ScanStatus status = ScanStatus::Pending;

    // Set when fine_quality tripped the rejection threshold. The scan is
    // still usable (and still fused) - this only marks it for attention.
    bool flagged = false;

    // --- Quality control (see FusionSession::computeQc) ------------------
    // How far the tag pose and the geometric coarse pose put this scan apart,
    // when both existed. Tens of degrees means one of them is wrong outright.
    bool   has_tag_disagreement = false;
    double tag_disagreement_deg = 0.0;
    double tag_disagreement_shift = 0.0;   // camera position, metres

    // Corner residual of a tag fit that was *rejected* (tag_fit is then
    // invalid), metres; negative when no fit was rejected. Many of these in a
    // session mean the tilt seeds the fit is constrained by are wrong.
    double tag_rejected_rms = -1.0;

    // Why the scan needs a human to look at it (empty: it doesn't), and
    // softer observations that do not on their own. needs_review is
    // qc_reasons being non-empty, unless a review has since accepted it.
    std::vector<std::string> qc_reasons;
    std::vector<std::string> qc_warnings;
    bool needs_review = false;

    // One attempt of the automatic retry ladder (FusionSession::runAutoRetry).
    // Scores are against the same map of every other scan, lower is better.
    struct RetryAttempt
    {
        std::string rung;
        double score_before = 0.0;
        double score_after = 0.0;
        bool   kept = false;      // the attempt's pose replaced the previous one
        bool   cleared = false;   // and left the scan with no qc_reasons
    };
    std::vector<RetryAttempt> retries;

    // Who left the scan out: "" (nobody), "auto" (the pipeline - it failed
    // to load) or "manual" (excluded in a review). The pipeline never
    // excludes a scan for registering badly; it flags it instead.
    std::string excluded_by;

    // Set by a --review pass that saved this scan's pose.
    bool        reviewed = false;
    std::string reviewed_at;

    std::string label() const { return input.label(); }

    bool loaded() const { return prepared && !prepared->empty(); }

    // Points transformed into the common frame. Allocates - call it once
    // and hold the result rather than per-frame in a render loop.
    PointCloudT::Ptr cloudInCommonFrame() const;

    // `prepared` with its normals attached, still in the camera frame - the
    // form ICP takes a source in. NaN normals if none were estimated.
    PointCloudNT::Ptr cloudWithNormals() const;
};
