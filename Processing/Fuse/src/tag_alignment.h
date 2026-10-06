#pragma once

#include <array>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "fusion_pipeline.h"

// AprilTag-based coarse alignment.
//
// The rig carries AprilTags at fixed positions in the world, and rs_marker
// logs every detection beside the scan as <scan>_detections.csv. A tag seen
// by two scans is a set of four exact correspondences between them, which
// pins down the relative pose directly - including the in-plane slide and
// spin that a flat sand surface leaves geometry-only methods nearly blind to.
//
// The detections are pixel coordinates in the colour image. rs_marker aligns
// depth to colour before exporting the PLY, so the cloud is in the colour
// camera's frame and the colour intrinsics (from scan_info.txt) carry pixels
// to rays in it. The PLY itself is written by librealsense in an OpenGL-style
// frame: (x, -y, -z) of the camera frame.

// Colour stream intrinsics, as librealsense reports them.
struct CameraIntrinsics
{
    int   width = 0, height = 0;
    float fx = 0.0f, fy = 0.0f, ppx = 0.0f, ppy = 0.0f;
    std::array<float, 5> coeffs{};   // k1, k2, p1, p2, k3
    bool  distorted = false;
};

// One tag's four corners, in detector order. Kept in order because the
// order is what makes them correspondences.
using TagCorners = std::array<Eigen::Vector3f, 4>;
using TagSet     = std::map<int, TagCorners>;   // keyed by tag id

// Per-tag diagnostics from locating it in 3D.
struct TagLocateReport
{
    int    id = -1;
    bool   located = false;
    int    frames = 0;          // decoded detections the corners were taken from
    int    plane_points = 0;    // cloud points the tag's plane was fitted to
    double plane_rms = 0.0;     // metres
    double edge_length = 0.0;   // mean of the four edges, metres - a size sanity check
    std::string reason;         // why it was not located, if it wasn't
};

// Finds "<folder>_detections.csv" (or the single *_detections.csv) and
// scan_info.txt beside a scan's PLY. Returns false if either is missing.
bool findTagFiles(const std::string& ply_path,
                  std::string& detections_path,
                  std::string& scan_info_path);

// Finds the tag sidecar crop_pointclouds.py writes beside a cropped PLY:
// "<folder>_tags.csv", or the single *_tags.csv. The crop cuts the tags out of
// the cloud, so for a cropped scan their positions can only come from here -
// measured on the uncropped cloud, whose frame the crop keeps.
bool findTagSidecar(const std::string& ply_path, std::string& sidecar_path);

// Reads a sidecar written by crop_pointclouds.py:
//   marker_id,corner,x,y,z,frames,plane_rms_m,edge_m
// one row per corner, in the PLY's own frame. Tags missing any corner are
// skipped.
bool readTagSidecar(const std::string& path, TagSet& out,
                    std::vector<TagLocateReport>* reports = nullptr);

// Reads "stream.Color.intrinsics" (and the colour resolution) from an
// rs_marker scan_info.txt.
bool readColorIntrinsics(const std::string& scan_info_path, CameraIntrinsics& out);

// Locates every decoded tag in `raw_cloud` (the uncropped PLY, in its own
// frame). Corner pixels are the per-tag median over all decoded frames: the
// camera is still for the whole recording, so the median only removes
// detector jitter. Tags whose corners cannot be placed reliably are left out
// and explained in `reports`.
TagSet locateTags(const PointCloudT& raw_cloud,
                  const std::string& detections_path,
                  const CameraIntrinsics& intrinsics,
                  std::vector<TagLocateReport>* reports = nullptr);

// Rigid transform carrying `source` tags onto `target` tags, fitted over the
// corners of every tag id the two share. Fails when they share fewer than
// `min_shared` tags.
struct TagFit
{
    bool            valid = false;
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    int             shared_tags = 0;
    double          rms = 0.0;   // corner residual after the fit, metres
    bool            tilt_from_imu = false;   // fitted by fitTagsAboutAxis
    std::vector<int> ids;
};

TagFit fitTags(const TagSet& source, const TagSet& target, int min_shared);

// The same fit with the tilt held fixed: the rotation is `base_rotation`
// followed by a turn about `up`, and only that turn and the translation are
// solved for. The tags are poor at tilt - the ones a scan sees usually lie
// nearly in a line, so tilt about that line rests on each 80 mm tag's own
// plane, which depth noise sets only to a degree or two - while the IMU
// measures tilt well. What the IMU cannot see at all, the turn about gravity,
// is what the tags pin down best.
TagFit fitTagsAboutAxis(const TagSet& source, const TagSet& target, int min_shared,
                        const Eigen::Matrix3f& base_rotation, const Eigen::Vector3f& up);
