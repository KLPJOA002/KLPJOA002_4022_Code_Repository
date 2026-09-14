#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "fusion_pipeline.h"

// Reconciling overlapping scans into one surface, rather than piling them
// up on top of each other.
//
// Concatenating registered scans does not fuse them. Residual registration
// error (a few mm here) is larger than the output voxel, so where scans
// overlap their surfaces land in *different* voxels and survive as two or
// three offset sheets. Voxel centroids cannot merge sheets they never share
// a cell with, and statistical outlier removal will not delete one either -
// a duplicate sheet is locally dense and looks like perfectly good data.
// That is what makes a merged cloud look noisy exactly where it has the
// most information.
//
// The fix has three parts:
//   1. Weight every point by how much it deserves to be believed -
//      its scan's registration quality, the angle the surface was seen at,
//      and how far away it was.
//   2. Require corroboration, so points no other scan saw can be dropped.
//   3. Combine overlapping points *robustly*, so one misregistered scan is
//      rejected rather than averaged into the result.

// One registered scan's contribution to the fused cloud.
struct ScanContribution
{
    PointCloudT::ConstPtr cloud;          // already transformed into the common frame

    // Where the camera was, in the common frame. Used for the incidence and
    // range weights: both need to know which direction the surface was
    // viewed from, which is lost once the points are in a shared frame.
    Eigen::Vector3f sensor_origin = Eigen::Vector3f::Zero();

    // How well this scan registered, in [0, 1]. Scales every one of its
    // points' weights, so a scan the pipeline is unsure about contributes
    // to the result without being allowed to drag it around.
    double confidence = 1.0;

    std::string label;
};

struct ReconcileParams
{
    enum class Mode
    {
        Auto,         // height field on flat scenes, volumetric otherwise
        Volumetric,   // weighted voxel blending + surface fit
        HeightField,  // one height per cell on the scene's dominant plane
    };
    Mode mode = Mode::Auto;

    // --- Per-point weighting -------------------------------------------
    bool weight_by_registration = true;

    // Weight by the cosine of the angle between the surface normal and the
    // direction back to the camera. A depth sensor's error grows sharply at
    // grazing incidence, so a surface seen face-on from one scan should win
    // over the same surface seen edge-on from another.
    bool  weight_by_incidence = true;
    float incidence_min_weight = 0.05f;  // floor, so grazing points are quiet, not silent

    // Stereo depth error grows with the square of range, so weight falls off
    // the same way, normalised so points at range_reference weigh 1.
    bool  weight_by_range = true;
    float range_reference = 0.6f;        // metres - roughly this rig's standoff

    float normal_radius = 0.015f;        // for the incidence weight's normals

    // --- Corroboration --------------------------------------------------
    // Minimum number of *distinct scans* that must see a cell for it to
    // survive. 1 keeps everything; 2 keeps only surface that at least two
    // scans agree exists, which is a very effective noise filter in the
    // overlap regions but will also trim the outer edges each scan alone saw.
    int min_contributing_scans = 1;

    // --- Volumetric mode -------------------------------------------------
    float voxel_size = 0.002f;

    // --- Height-field mode -----------------------------------------------
    float height_cell = 0.003f;

    // --- Robust combination ----------------------------------------------
    // Fraction trimmed from each end before averaging the values in a cell.
    // 0 is a plain weighted mean (one bad scan shifts the result); 0.5 is a
    // weighted median (maximally robust, slightly noisier). The default
    // discards a quarter from each end, which rejects a single misregistered
    // sheet while still averaging away sensor noise.
    float trim_fraction = 0.25f;

    // --- Surface fit ------------------------------------------------------
    // Moving least squares: fits a local polynomial to each neighbourhood
    // and projects points onto it. This is what actually collapses residual
    // offset sheets into one surface, because it averages *along the surface
    // normal* over a radius larger than the misregistration, while keeping
    // lateral detail. Set the radius to a few times your residual rmse.
    bool  use_mls = true;
    float mls_radius = 0.010f;
    int   mls_polynomial_order = 2;

    // --- Final cleanup ----------------------------------------------------
    // Removes points that sit off the local surface: each point is compared
    // with a plane fitted to its nearest neighbours, and dropped if its
    // distance from that plane is far above the cloud's typical figure.
    //
    // This deliberately does NOT use statistical outlier removal (mean
    // distance to the k nearest neighbours). That measures *density*, and a
    // fused cloud's density is uneven by construction - once the fusion cell
    // is finer than a single scan's point spacing, areas seen by many scans
    // come out denser than areas seen by a few. SOR then deletes the sparser
    // areas wholesale, punching holes in genuine overlap. Distance from the
    // local surface is the same however many points describe that surface.
    bool   run_outlier_removal = true;
    int    outlier_neighbours = 30;
    double outlier_stddev_mult = 3.0;

    bool   estimate_normals = true;
    float  output_normal_radius = 0.015f;
};

// What reconciliation did, so the result can be judged rather than assumed.
struct ReconcileReport
{
    std::string mode_used;

    size_t input_points = 0;
    size_t cells_total = 0;
    size_t cells_dropped_no_corroboration = 0;
    size_t points_after_blend = 0;
    size_t points_after_outlier_removal = 0;
    size_t output_points = 0;

    double mean_scans_per_cell = 0.0;

    // How far apart the contributing scans were *within* a cell, before and
    // after combining - i.e. the thickness of the surface being fused. The
    // before figure is the sheet separation you were seeing as noise; the
    // after figure is what is left of it.
    double spread_before = 0.0;
    double spread_after = 0.0;

    void print() const;
};

// Fuses the contributions into a single surface. `plane` is only consulted
// for height-field mode (and for Auto's choice); pass an invalid one to
// force volumetric behaviour.
PointCloudNT::Ptr reconcileScans(const std::vector<ScanContribution>& contributions,
                                   const ReconcileParams& params,
                                   const PlaneFrame& plane,
                                   ReconcileReport* report = nullptr);
