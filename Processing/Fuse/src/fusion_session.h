#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "fusion_pipeline.h"
#include "fusion_reconcile.h"
#include "rig_geometry.h"
#include "scan_state.h"

// The pipeline as an explicit sequence of stages rather than one loop.
// Each stage is a separate function on FusionSession with no hidden state
// beyond the ScanStates themselves, so any stage can be re-run at any time
// (which is exactly what the review stages need).
//
//   Load ─> CalibrateTilt ─> ResolveRotation ─> CoarseBatch ─> CoarseReview ─┬─> RefineBatch ─> RefineReview ─┬─> Fuse ─> Done
//                                                ^          │                     │          │
//                                                └──────────┘ (re-run / exclude)  └──────────┘ (back, or refine again)
enum class Stage
{
    Load,
    CalibrateTilt,
    ResolveRotation,
    CoarseBatch,
    CoarseReview,
    RefineBatch,
    RefineReview,
    Fuse,
    Done,
};

const char* toString(Stage stage);

// What a review stage decided to do next. The stage controller in main.cpp
// maps these onto stage transitions; the review UI (console or viewer)
// only has to produce one of these.
enum class ReviewAction
{
    Continue,      // proceed to the next stage
    RerunCoarse,   // re-run coarse alignment (for the whole batch, or just
                   // the scans the UI marked, if any)
    RerunRefine,   // re-run fine registration
    Back,          // return to the previous review stage
    Quit,          // abandon interactive review, finish with what we have
};

// done/total progress with a label for the item currently being worked on.
// Called before each item starts, so `done` is the count already finished.
using ProgressFn = std::function<void(size_t done, size_t total, const std::string& label)>;

// Asked once per scan at load time for an optional crop box in the scan's
// own camera frame. Return false to use the full cloud. The visualizer
// implements this with its interactive box widget; headless runs pass
// nothing and every scan is used whole.
using RoiProvider = std::function<bool(const ScanInput& input,
                                        const PointCloudT::ConstPtr& raw,
                                        Eigen::Vector4f& out_min,
                                        Eigen::Vector4f& out_max)>;

// One physical mount position: every scan captured before the rig was
// turned again. The rotation *direction* is a property of the step from
// the previous group to this one, so the group is the unit that direction
// inference works on - not the session, and not the individual scan.
struct RotationGroup
{
    std::string name;            // "<session>/<rotation folder>"
    std::string session_name;
    int         index = 0;       // ordinal within its session, from discovery
    std::vector<size_t> scans;   // indices into FusionSession::scans()

    // Magnitude of the step from the previous group, as recorded by the
    // folder names (which are always positive - that is the whole problem).
    double nominal_step_deg = 0.0;

    // Gravity as read at this group's lowest-tilt scan, which is the
    // within-group reference every other scan's tilt is measured against.
    Eigen::Vector3f reference_gravity = Eigen::Vector3f::Zero();
    bool            has_reference_gravity = false;
    size_t          reference_scan = 0;

    // Direction chosen for that step, and the resulting cumulative angle
    // from the reference group. Cumulative, because a folder name's
    // "r90" only means 90 degrees if every step before it went the same way.
    float  step_sign = 1.0f;
    double cumulative_deg = 0.0;

    bool   resolved = false;     // direction actually tested, not just assumed
    bool   ambiguous = false;    // tested, but the two candidates scored too close
    double margin = 0.0;         // relative score gap between the candidates

    AlignmentQuality chosen_quality;
    AlignmentQuality rejected_quality;

    // Angle measured from the AprilTags rather than inferred from the surface.
    // The tags are fixed in the world, so they say outright how far the rig
    // turned - including turns the folder names don't describe (e.g. a step
    // back past the start). Such a group is left out of the direction search.
    bool   from_tags = false;
    double tag_spread_deg = 0.0;  // largest per-scan deviation from the group's angle
};

// What the tilt calibration measured. Also a diagnostic: if the residual
// stays high after fitting, the mismatch is not a lever arm and the axis
// or sign fields say which of the other candidates it is instead.
struct TiltCalibration
{
    bool valid = false;

    // --- IMU-to-cloud frame ------------------------------------------------
    // The accelerometer reports in the camera's depth frame, but a PLY does
    // not have to be written in that frame - the RealSense Viewer, for one,
    // exports an OpenGL-style frame with y and z negated. Applying a correct
    // rotation in the wrong frame produces an error that grows with the tilt
    // angle, which looks exactly like a miscalibrated rig. So the relationship
    // is detected from the data rather than assumed.
    bool            frame_detected = false;
    std::string     frame_name = "identity";
    Eigen::Matrix3f imu_to_cloud = Eigen::Matrix3f::Identity();
    double          frame_score = 0.0;      // residual with the chosen frame
    double          frame_runner_up = 0.0;  // residual with the next best
    bool rejected = false;   // fitted, but the fit did not explain the motion
    int  samples = 0;

    // Where the tilt axis passes through, in the camera frame (metres).
    Eigen::Vector3f pivot = Eigen::Vector3f::Zero();

    // RMS of the unexplained part of the observed motion, in metres. Small
    // means the lever-arm model accounts for what actually happened.
    double residual = 0.0;

    // Residual before and after the pivot search, over the calibration scans.
    double residual_before_pivot = 0.0;

    // How much the seed was wrong by before and after, at the largest tilt
    // in the set - the number that decides whether ICP can cope.
    double worst_tilt_deg = 0.0;
    double seed_error_before = 0.0;
    double seed_error_after = 0.0;

    // --- Cross-checks against the other explanations for a tilt mismatch ---

    // Axis the scans were actually observed to rotate about, and how far
    // that is from the configured tilt axis. A large angle here means the
    // rig doesn't tilt about the axis you told it to.
    Eigen::Vector3f observed_axis = Eigen::Vector3f::Zero();
    double axis_error_deg = 0.0;

    // Observed rotation per degree of commanded tilt. Should be ~+1.0; a
    // value near -1.0 means tilt_sign is backwards, and a magnitude well
    // off 1.0 means the recorded angles don't match the real motion.
    double angle_scale = 1.0;
    bool   sign_looks_flipped = false;
};

class FusionSession
{
public:
    FusionSession(std::vector<ScanInput> inputs, PipelineParams params);

    // --- Stages ---------------------------------------------------------

    // Reads every PLY, applies its ROI crop, preprocesses, and drops the
    // raw cloud. Scans that fail to load are marked Excluded rather than
    // aborting the run - one unreadable file shouldn't cost you the batch.
    void loadAll(const ProgressFn& progress = nullptr,
                  const RoiProvider& roi = nullptr);

    // Measures the rig's tilt geometry from the scans that already align,
    // and applies it to every scan's seed. See TiltCalibration.
    // `sign_retry` guards the one-shot re-measure after a backwards
    // tilt_sign is corrected; callers leave it false.
    void runTiltCalibrationStage(const ProgressFn& progress = nullptr,
                                   bool sign_retry = false);

    const TiltCalibration& tiltCalibration() const { return tilt_calibration_; }
    void printTiltCalibrationReport() const;

    // Compares what the IMU says each scan's tilt actually was against what
    // the folder name claims it was commanded to be.
    void printTiltFromImuReport() const;

    const PlaneFrame& scenePlane() const { return scene_plane_; }

    // Scores an alignment with whichever metric suits the scene: the
    // height-field one when the scene is a flat sheet, nearest-neighbour
    // otherwise.
    AlignmentQuality scoreAlignment(const PointCloudT::ConstPtr& source,
                                      const PointCloudT::ConstPtr& target,
                                      const Eigen::Matrix4f& transform) const;

    // How many scans carry a usable gravity reading.
    size_t countWithGravity() const;

    // Works out which way the rig actually turned at each rotation step,
    // by trying both directions and keeping whichever fits the scans
    // already placed. Cheap: it only places one representative scan per
    // group, leaving the full alignment to the coarse batch that follows.
    // A no-op when the rig's rotation axis is unknown, when there is only
    // one rotation group, or when inference is switched off.
    void runRotationDirectionStage(const ProgressFn& progress = nullptr);

    // Coarse-aligns every scan that needs it, in index order, each against
    // the map accumulated from the scans before it.
    void runCoarseBatch(const ProgressFn& progress = nullptr);

    // ICP-refines every coarse-aligned scan against the accumulated map.
    void runRefineBatch(const ProgressFn& progress = nullptr);

    // Single-scan versions, for the review stages' "redo just this one"
    // action. These align against *every other* contributing scan, not
    // just the earlier ones, since by review time the whole map exists.
    void coarseAlignScan(size_t index);
    void refineScan(size_t index);

    // Re-reads one scan from disk with a (possibly new) ROI crop, which
    // resets it to Pending.
    void reloadScan(size_t index, const RoiProvider& roi = nullptr);

    // --- Rotation groups -------------------------------------------------

    const std::vector<RotationGroup>& rotationGroups() const { return rotation_groups_; }

    // Which rotation group a scan belongs to, or -1 if it has no rotation
    // metadata (a flat file list, or a manifest).
    int rotationGroupOf(size_t scan_index) const;

    // Reverses one rotation step's direction and re-derives every angle
    // downstream of it - flipping step k moves group k and everything after
    // it, because the angles are cumulative. Affected scans are reset to
    // Pending so the next alignment pass re-places them.
    void flipRotationGroup(size_t group_index);

    void printRotationReport() const;

    void setRigGeometry(const RigGeometry& rig) { rig_ = rig; rig_known_ = true; }
    void setUseImuTilt(bool use) { use_imu_tilt_ = use; }
    bool usingImuTilt() const { return use_imu_tilt_; }
    const RigGeometry& rigGeometry() const { return rig_; }

    // --- State ----------------------------------------------------------

    const std::vector<ScanState>& scans() const { return scans_; }
    ScanState&       scan(size_t i)       { return scans_.at(i); }
    const ScanState& scan(size_t i) const { return scans_.at(i); }
    size_t size() const { return scans_.size(); }

    const PipelineParams& params() const { return params_; }
    PipelineParams&       params()       { return params_; }

    // Index of the scan seeding the common frame (0 after discovery's
    // reordering, which puts a 0-tilt scan first).
    size_t referenceIndex() const { return reference_index_; }

    void setExcluded(size_t index, bool excluded);

    // Applies a manual pose nudge on top of a scan's current transform,
    // expressed in the common frame.
    void nudgeScan(size_t index, const Eigen::Matrix4f& delta_in_common_frame);

    size_t countWithStatus(ScanStatus status) const;
    size_t countContributing() const;

    // --- Output ---------------------------------------------------------

    // Concatenates every contributing scan at its current transform. The
    // raw union, with overlapping surfaces still stacked on top of each
    // other - useful for inspection, not the final product.
    PointCloudT::Ptr buildMergedCloud() const;

    // The same scans packaged for reconciliation: each with its sensor
    // position in the common frame and a confidence derived from how well
    // it registered, so better-placed scans carry more weight in the blend.
    std::vector<ScanContribution> buildContributions() const;

    // Per-scan metrics table for the review stages.
    void printReport(const std::string& title) const;

private:
    // Union of contributing scans other than `exclude_index`, optionally
    // limited to indices below `max_index` (which is what makes a batch
    // sequential: each scan aligns against what came before it, giving
    // ICP progressively more surface without ever matching against a scan
    // that hasn't been placed yet).
    VoxelMap buildMap(size_t exclude_index, size_t max_index) const;

    // Leaf size the running map merges at: the scan voxel, or 0 (no merging)
    // when downsample_accumulated_map is off.
    float mapLeafSize() const;

    // ICP-refines one scan against `map` and records the result. Shared by
    // the batch and the single-scan redo so the two cannot drift apart.
    void refineAgainst(size_t index, const VoxelMap& map);

    // Re-derives every refined scan's review flag. Run after any refine,
    // since the threshold is relative to the whole batch.
    void updateFlags();

    // Scores one complete assignment of per-step directions.
    double scoreDirectionHypothesis(const std::vector<size_t>& representatives,
                                      const std::vector<float>& signs,
                                      std::vector<AlignmentQuality>* per_group);

    // Groups scans by their rotation folder, ordered as captured.
    void buildRotationGroups();

    // Sets each rotation group's angle from its scans' tag poses, where any
    // exist. See RotationGroup::from_tags.
    void resolveRotationFromTags();

    // Re-derives cumulative angles from the per-step signs, and rewrites
    // every affected scan's initial guess to match.
    void applyRotationAngles(size_t from_group_index);

    // Rebuilds one scan's rig seed for a given cumulative rotation angle.
    // `group` supplies the within-group gravity reference used for the tilt
    // component when the IMU path is available.
    void applyGuessToScan(size_t scan_index, double cumulative_rotation_deg,
                           const RotationGroup* group);

    // The tilt component of a scan's seed: the rotation carrying the scan's
    // camera frame into its rotation group's reference camera frame.
    Eigen::Matrix4f tiltComponentFor(const ScanState& scan, const RotationGroup* group) const;

    // Picks each group's lowest-tilt scan and caches its gravity.
    void resolveGroupGravityReferences();

    // Coarse-aligns one scan: the usual geometric hypotheses first, then -
    // with tag alignment on and a tag pose solved for the scan - the tag
    // pose, keeping whichever scores better on the map.
    void computeCoarseFor(size_t index, const PointCloudT::ConstPtr& map);

    // Chooses between the rig-geometry seed and a feature-matched pose by
    // scoring both, and records the winner on the scan.
    void computeGeometricCoarseFor(size_t index, const PointCloudT::ConstPtr& map);

    // Places every scan it can from the tags alone: starting from the
    // reference's tags, repeatedly fits the unplaced scan sharing the most
    // tags with those already placed, and adds its tags to the set. Kept
    // independent of the geometric placements on purpose - a scan a wrong
    // seed has misplaced would otherwise drag every tag it sees with it.
    void solveTagPoses();

    // Finds and locates one scan's tags from its raw cloud.
    void locateScanTags(ScanState& scan, const PointCloudT& raw) const;

    // Least-squares fit of the pivot that explains a set of observed
    // rotations: each observation contributes (I - R) p = t.
    Eigen::Vector3f configuredTiltAxis() const;

    // Scans used to calibrate the tilt: tilted, in the reference's rotation
    // group, largest tilt first.
    std::vector<size_t> selectTiltCalibrationScans() const;

    // Determines the IMU-to-cloud frame and the tilt pivot *together*.
    // They cannot be settled separately: with the camera far from the tilt
    // axis, the pivot error dominates the residual, so comparing frames at a
    // zero pivot ranks noise rather than the frames. Each candidate frame is
    // therefore given its own pivot search before they are compared.
    void calibrateImuTiltGeometry(const std::vector<size_t>& calibration_scans);

    // Coarse-to-fine search for the pivot under the current frame. Returns the
    // best score found and writes the pivot to `out_pivot`. Only the plane
    // perpendicular to the hinge is searched: sliding the pivot along the
    // hinge changes nothing, so that direction is a flat valley.
    double searchTiltPivot(const std::vector<size_t>& calibration_scans,
                            int levels, Eigen::Vector3f& out_pivot) const;

    // Mean hinge direction over the calibration scans, from the measured
    // rotations themselves.
    Eigen::Vector3f meanHingeAxis(const std::vector<size_t>& calibration_scans) const;

    // Mean alignment residual of the calibration scans against the reference,
    // under the current frame and pivot. The objective both searches minimise.
    // Runs against `calibration_target_` and `calibration_clouds_`, which are
    // coarser copies prepared once - these searches run thousands of
    // evaluations and do not need full resolution.
    double calibrationResidual(const std::vector<size_t>& calibration_scans,
                                 const Eigen::Vector3f& pivot) const;

    // Builds the coarse clouds and the reference height grid the calibration
    // searches run against.
    void prepareCalibrationCaches(const std::vector<size_t>& calibration_scans);

    // Rotation carrying `scan` into `reference`'s frame, from their gravity
    // readings, expressed in the point clouds' frame.
    Eigen::Matrix3f imuRotationBetween(const ScanState& scan, const Eigen::Vector3f& reference_gravity) const;

    static bool solvePivot(const std::vector<Eigen::Matrix3f>& rotations,
                            const std::vector<Eigen::Vector3f>& translations,
                            Eigen::Vector3f& pivot,
                            double& residual);

    // Use the IMU's gravity vector for the tilt seed where available,
    // rather than the commanded angle about a nominated camera axis.
    bool                       use_imu_tilt_ = true;

    // Dominant plane of the scene, fitted once from the reference scan.
    // Drives the switch to height-field scoring on flat scenes.
    PlaneFrame                 scene_plane_;

    // Prepared once per calibration, reused across every candidate. The
    // reference grid in particular was previously rebuilt on every one of
    // thousands of evaluations, which is what made calibration crawl.
    HeightGrid                                  calibration_target_;
    std::unordered_map<size_t, PointCloudT::Ptr> calibration_clouds_;

    TiltCalibration            tilt_calibration_;
    std::vector<ScanState>     scans_;
    std::vector<RotationGroup> rotation_groups_;
    PipelineParams             params_;
    RigGeometry                rig_;
    bool                       rig_known_ = false;
    size_t                     reference_index_ = 0;
};
