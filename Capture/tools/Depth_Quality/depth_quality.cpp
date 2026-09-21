// depth_quality.cpp — RealSense depth characterisation (Depth-Quality-Tool style)
//
// Same overall structure as the capture tool (main.cpp): arg parsing → timestamped
// run folder → outer "trial" loop with a manual prompt between trials → inner
// frame loop → per-trial outputs. Instead of AprilTags / tilt sweeps, every frame
// is evaluated inside an ROI against a flat target at a known height and tilt.
//
// Per frame  : console line + CSV row (fill rate, Z error vs ground truth,
//              plane-fit RMS, sub-pixel RMS, fitted distance/pitch/roll, IR level,
//              exposure/gain metadata, accel).
// Per trial  : bag recording, <trial>_metrics.csv, trial_info.txt,
//              plus one row appended to <run>/summary.csv (incl. temporal noise).
//
// Build:
//   g++ -O2 -std=c++17 depth_quality.cpp -o depth_quality
//       -lrealsense2 $(pkg-config --cflags --libs opencv4)   (one line)
//
// Keys (in any image window): q/Esc end trial, r re-select ROI, s save PNG snapshot

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <vector>
#include <csignal>
#include <atomic>
#include <chrono>
#include <algorithm>
#include <fstream>
#include <ctime>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <cerrno>
#include <cctype>
#include <limits>
#include <thread>
#include <sys/stat.h>

static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

static const double NaN = std::numeric_limits<double>::quiet_NaN();
static const double D2R = CV_PI / 180.0;
static const double R2D = 180.0 / CV_PI;

// ── Options ───────────────────────────────────────────────────────────────────
// Everything that describes the physical setup of one trial. Can be changed at
// the prompt between trials.
struct TrialSetup {
    double      height_mm = 0.0;   // perpendicular distance to plane (0 = unused)
    double      axis_mm   = 0.0;   // distance along optical axis   (0 = unused)
    double      pitch_deg = 0.0;   // + = top of image farther away
    double      roll_deg  = 0.0;   // + = right of image farther away
    double      lux       = NaN;   // logged only
    std::string label;
};

struct Options {
    std::string output_dir   = "./dq_runs";
    std::string bag_ext      = ".db3";
    int         duration_sec = 0;
    long long   frame_limit  = 0;
    int         width = 1280, height = 720, fps = 30;
    std::string preset       = "high_density";
    int         emitter      = -1;      // -1 leave as-is, 0 off, 1 on
    float       laser_power  = -1.0f;   // <0 leave as-is
    float       exposure_us  = -1.0f;   // <0 auto exposure
    bool        use_filters  = false;
    float       min_dist = 0.2f, max_dist = 3.0f;
    bool        record       = true;
    bool        use_color    = true;
    bool        use_imu      = true;
    int         print_every  = 1;
    double      err_range_mm = 10.0;
    double      gt_offset_mm = 0.0;
    // ROI
    bool        roi_given  = false;
    cv::Rect    roi;
    double      roi_pct    = 40.0;
    bool        select_roi = false;
    // Dynamic ROI: a patch_size x patch_size square that lies *in the target
    // plane* is re-projected into the image each frame (0 = off = static ROI).
    double      patch_size    = 0.0;
    bool        patch_outer   = false;   // bounding box of the patch, not the inscribed box
    double      patch_smooth  = 0.3;     // EMA weight of the newest plane estimate
    int         patch_hyst_px = 3;       // ignore ROI moves smaller than this, in px
    int         patch_min_px  = 8;       // never let the ROI shrink below this

    TrialSetup  setup;
};

static void print_usage(const char* prog)
{
    std::cout <<
    "Usage: " << prog << " [options]\n"
    "\nGround truth (can also be changed at the prompt between trials):\n"
    "  -H, --height MM        Perpendicular distance depth-origin -> target plane\n"
    "  -Z, --axis-dist MM     Alternative: distance along the optical axis to the plane\n"
    "  -p, --pitch DEG        Camera pitch w.r.t. plane (+ = top of image farther)\n"
    "  -R, --roll DEG         Camera roll  w.r.t. plane (+ = right of image farther)\n"
    "      --gt-offset MM     Added to measured distances (front glass -> depth origin)\n"
    "  -L, --label TEXT       Trial label, e.g. shade / sun_noon (no spaces)\n"
    "      --lux VALUE        Measured illuminance (logged only)\n"
    "\nROI (depth-image pixels):\n"
    "  -r, --roi X,Y,W,H      Explicit ROI\n"
    "      --roi-pct P        Centred ROI of P% width/height (default 40)\n"
    "      --select-roi       Draw the ROI with the mouse on the first frame\n"
    "      --patch-size MM    Track a MM x MM square patch of the target plane:\n"
    "                         the ROI is re-projected every frame so it always\n"
    "                         covers that much floor (0 = off, static ROI)\n"
    "      --patch-outer      Use the bounding box of the projected patch instead\n"
    "                         of the largest box inscribed in it (default: inscribed)\n"
    "      --patch-smooth A   EMA weight of the newest plane fit, 0..1 (default 0.3)\n"
    "      --patch-hyst PX    Only move the ROI when it shifts by more than PX (default 3)\n"
    "\nCapture:\n"
    "  -D, --dir DIR          Output root (default ./dq_runs)\n"
    "  -d, --duration S       Seconds per trial (default: until q)\n"
    "  -n, --frames N         Frames per trial  (default: until q)\n"
    "      --res WxH          Resolution for depth/IR/colour (default 1280x720)\n"
    "      --fps N            Frame rate (default 30)\n"
    "      --preset NAME      default|high_density|high_accuracy|medium_density|hand\n"
    "                         (default high_density, as in the capture tool)\n"
    "      --emitter on|off   Force projector state\n"
    "      --laser-power MW   Projector power\n"
    "      --exposure US      Manual exposure (disables auto-exposure)\n"
    "      --filters          Evaluate the post-processed depth instead of raw\n"
    "      --min-dist M       Threshold filter min (with --filters, default 0.2)\n"
    "      --max-dist M       Threshold filter max (with --filters, default 3.0)\n"
    "      --ext EXT          Recording extension (default .db3)\n"
    "      -noRecord          Do not record a bag\n"
    "      -noColor           Do not stream colour\n"
    "      -noIMU             Do not stream the accelerometer\n"
    "      --print-every N    Console line every N frames (default 1)\n"
    "      --err-range MM     +/- range of the error-map colours (default 10)\n"
    "  -h, --help\n"
    "\nKeys: q/Esc end trial, r re-select ROI, s save PNG snapshot\n";
}

// ── Depth post-processing (same chain as the capture tool; off by default) ───
struct DepthFilters {
    rs2::threshold_filter    threshold;
    rs2::disparity_transform to_disparity{true};
    rs2::spatial_filter      spatial;
    rs2::temporal_filter     temporal;
    rs2::disparity_transform to_depth{false};
    rs2::hole_filling_filter hole_fill;

    DepthFilters(float min_d, float max_d)
    {
        threshold.set_option(RS2_OPTION_MIN_DISTANCE, min_d);
        threshold.set_option(RS2_OPTION_MAX_DISTANCE, max_d);

        spatial.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.8f);
        spatial.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 5.f);
        spatial.set_option(RS2_OPTION_FILTER_MAGNITUDE, 1.f);

        temporal.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.06f);
        temporal.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 100.f);
        temporal.set_option(RS2_OPTION_HOLES_FILL, 1.f);

        hole_fill.set_option(RS2_OPTION_HOLES_FILL, 1.f);
    }

    rs2::depth_frame process(rs2::depth_frame depth)
    {
        depth = threshold.process(depth);
        depth = to_disparity.process(depth);
        depth = spatial.process(depth);
        depth = temporal.process(depth);
        depth = to_depth.process(depth);
        depth = hole_fill.process(depth);
        return depth.as<rs2::depth_frame>();
    }
};

// ── Pipeline helpers ──────────────────────────────────────────────────────────
struct PipelineCtx {
    rs2::pipeline         pipe;
    rs2::pipeline_profile profile;
};

static int preset_value(const std::string& name)
{
    if (name == "default")        return RS2_RS400_VISUAL_PRESET_DEFAULT;
    if (name == "high_density")   return RS2_RS400_VISUAL_PRESET_HIGH_DENSITY;
    if (name == "high_accuracy")  return RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY;
    if (name == "medium_density") return RS2_RS400_VISUAL_PRESET_MEDIUM_DENSITY;
    if (name == "hand")           return RS2_RS400_VISUAL_PRESET_HAND;
    return -1;
}

static void try_set(rs2::sensor& s, rs2_option opt, float val, const char* name)
{
    try {
        if (s.supports(opt)) s.set_option(opt, val);
        else std::cerr << "  (" << name << " not supported on this device)\n";
    } catch (const rs2::error& e) {
        std::cerr << "  Failed to set " << name << ": " << e.what() << "\n";
    }
}

static PipelineCtx start_pipeline(const std::string& record_file, const Options& o)
{
    PipelineCtx ctx;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH,     0, o.width, o.height, RS2_FORMAT_Z16, o.fps);
    cfg.enable_stream(RS2_STREAM_INFRARED,  1, o.width, o.height, RS2_FORMAT_Y8,  o.fps); // pixel-aligned with depth
    if (o.use_color)
        cfg.enable_stream(RS2_STREAM_COLOR, 0, o.width, o.height, RS2_FORMAT_RGB8, o.fps);
    if (o.use_imu)
        cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    if (!record_file.empty())
        cfg.enable_record_to_file(record_file);

    ctx.profile = ctx.pipe.start(cfg);

    auto ds = ctx.profile.get_device().first<rs2::depth_sensor>();
    int preset = preset_value(o.preset);
    if (preset >= 0) try_set(ds, RS2_OPTION_VISUAL_PRESET, float(preset), "visual preset");
    try_set(ds, RS2_OPTION_DEPTH_UNITS, 0.0001f, "depth units");           // 0.1 mm, as in capture tool
    try_set(ds, RS2_OPTION_THERMAL_COMPENSATION, 0.0f, "thermal compensation");
    if (o.emitter >= 0)       try_set(ds, RS2_OPTION_EMITTER_ENABLED, float(o.emitter), "emitter");
    if (o.laser_power >= 0)   try_set(ds, RS2_OPTION_LASER_POWER, o.laser_power, "laser power");
    if (o.exposure_us >= 0) {
        try_set(ds, RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0.0f, "auto exposure");
        try_set(ds, RS2_OPTION_EXPOSURE, o.exposure_us, "exposure");
    } else {
        try_set(ds, RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.0f, "auto exposure");
    }
    return ctx;
}

static PipelineCtx start_pipeline_retry(const std::string& record_file, const Options& o,
                                        int max_attempts = 5)
{
    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        try {
            return start_pipeline(record_file, o);
        } catch (const rs2::error& e) {
            std::cerr << "start_pipeline attempt " << attempt << " failed: "
                      << e.what() << " — retrying...\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(500 * attempt));
        }
    }
    throw std::runtime_error("Failed to start RealSense pipeline after retries");
}

// Stereo baseline in mm (needed for the sub-pixel RMS metric).
static double stereo_baseline_mm(rs2::depth_sensor& ds, const rs2::pipeline_profile& prof)
{
    try {
        if (ds.supports(RS2_OPTION_STEREO_BASELINE))
            return std::fabs(ds.get_option(RS2_OPTION_STEREO_BASELINE));
    } catch (...) {}
    try {
        auto ir1 = prof.get_stream(RS2_STREAM_INFRARED, 1);
        for (auto& sp : ds.get_stream_profiles())
            if (sp.stream_type() == RS2_STREAM_INFRARED && sp.stream_index() == 2)
                return std::fabs(ir1.get_extrinsics_to(sp).translation[0]) * 1000.0;
    } catch (...) {}
    return 0.0;
}

static double frame_md(const rs2::frame& f, rs2_frame_metadata_value k)
{
    try {
        if (f && f.supports_frame_metadata(k)) return double(f.get_frame_metadata(k));
    } catch (...) {}
    return NaN;
}

// ── Image helpers ─────────────────────────────────────────────────────────────
static cv::Mat rgb_to_bgr(const rs2::video_frame& f)
{
    cv::Mat m(f.get_height(), f.get_width(), CV_8UC3, (void*)f.get_data(), f.get_stride_in_bytes());
    cv::Mat out;
    cv::cvtColor(m, out, cv::COLOR_RGB2BGR);
    return out;
}

static cv::Mat y8_view(const rs2::video_frame& f)
{
    return cv::Mat(f.get_height(), f.get_width(), CV_8UC1, (void*)f.get_data(), f.get_stride_in_bytes());
}

static void draw_text_block(cv::Mat& img, const std::vector<std::string>& lines,
                            cv::Point origin, double scale = 0.55)
{
    int line_h = int(26 * scale / 0.55);
    int max_w = 0;
    for (const auto& l : lines) {
        int base = 0;
        max_w = std::max(max_w, cv::getTextSize(l, cv::FONT_HERSHEY_SIMPLEX, scale, 1, &base).width);
    }
    cv::Rect box(origin.x - 6, origin.y - line_h, max_w + 12, line_h * int(lines.size()) + 8);
    box &= cv::Rect(0, 0, img.cols, img.rows);
    cv::Mat sub = img(box);
    sub *= 0.35;                                         // darken background for legibility
    int y = origin.y;
    for (const auto& l : lines) {
        cv::putText(img, l, cv::Point(origin.x, y), cv::FONT_HERSHEY_SIMPLEX,
                    scale, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
        y += line_h;
    }
}

// Blue = measured closer than reference, red = farther, black = hole.
static cv::Mat render_error_map(const cv::Mat& err_mm, double range_mm)
{
    cv::Mat out(err_mm.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    for (int y = 0; y < err_mm.rows; ++y) {
        const float* e = err_mm.ptr<float>(y);
        cv::Vec3b*   o = out.ptr<cv::Vec3b>(y);
        for (int x = 0; x < err_mm.cols; ++x) {
            if (!std::isfinite(e[x])) continue;
            double t = std::clamp(e[x] / range_mm, -1.0, 1.0);
            uchar  k = uchar(255.0 * (1.0 - std::fabs(t)));
            o[x] = (t < 0) ? cv::Vec3b(255, k, k) : cv::Vec3b(k, k, 255);
        }
    }
    return out;
}

// ── Ground truth ──────────────────────────────────────────────────────────────
// Target plane in the depth camera frame (x right, y down, z forward, mm):
//     n · P = H      with |n| = 1, n_z > 0
// Tilt convention (identical for ground truth and for the fitted plane):
//     pitch = atan2(n_y, n_z)   roll = atan2(-n_x, n_z)
struct GroundTruth {
    bool      valid = false;
    cv::Vec3d n{0, 0, 1};
    double    H_mm = 0, axis_mm = 0;
    double    pitch_deg = 0, roll_deg = 0;
};

static GroundTruth make_gt(const TrialSetup& s, double offset_mm)
{
    GroundTruth g;
    g.pitch_deg = s.pitch_deg;
    g.roll_deg  = s.roll_deg;
    cv::Vec3d n(-std::tan(s.roll_deg * D2R), std::tan(s.pitch_deg * D2R), 1.0);
    g.n = cv::normalize(n);
    // Depth origin sits offset_mm behind the front glass along the optical axis.
    if (s.height_mm > 0)    g.H_mm = s.height_mm + offset_mm * g.n[2];
    else if (s.axis_mm > 0) g.H_mm = (s.axis_mm + offset_mm) * g.n[2];
    else return g;
    g.axis_mm = g.H_mm / g.n[2];
    g.valid   = true;
    return g;
}

// ── Dynamic ROI: a square patch of the target plane ───────────────────────────
// Given the target plane in the depth-camera frame (unit normal n, perpendicular
// distance H_mm, so n · P = H_mm) this builds a patch_mm x patch_mm square that
// *lies in that plane*, centred where anchor_ray pierces it, and projects the
// square back into the depth image. The ROI is then the axis-aligned rectangle
// that matches the projected quad.
//
// The upshot: the ROI grows as the camera drops towards the floor, shrinks as it
// rises, and squashes vertically as the camera pitches — so the number of square
// millimetres of floor being characterised stays constant at patch_mm^2.
struct PatchROI {
    bool        ok = false;
    cv::Rect    rect;
    cv::Point2f quad[4];                 // projected corners, for the overlay
    double      centre_axis_mm = NaN;    // Z of the patch centre, for the overlay
};

// Convex quad (corners in order): p is inside when it is on the same side of
// every edge.
static bool point_in_quad(const cv::Point2f q[4], const cv::Point2f& p)
{
    int pos = 0, neg = 0;
    for (int i = 0; i < 4; ++i) {
        const cv::Point2f& a = q[i];
        const cv::Point2f& b = q[(i + 1) % 4];
        double cr = double(b.x - a.x) * (p.y - a.y) - double(b.y - a.y) * (p.x - a.x);
        if      (cr > 0) ++pos;
        else if (cr < 0) ++neg;
    }
    return pos == 0 || neg == 0;
}

static PatchROI project_patch(const rs2_intrinsics& intr, const cv::Vec3d& n, double H_mm,
                              const cv::Vec3d& anchor_ray, double patch_mm, bool outer)
{
    PatchROI out;
    if (!(patch_mm > 0) || !(H_mm > 0)) return out;

    // 1. Patch centre: intersect the anchor ray with the plane.
    //       P(t) = t * anchor_ray,   n · P = H   =>   t = H / (n · anchor_ray)
    const double den = n.dot(anchor_ray);
    if (den <= 1e-9) return out;              // ray parallel to, or facing away from, the plane
    const cv::Vec3d C = anchor_ray * (H_mm / den);

    // 2. Two orthonormal axes that span the plane. e1 is the camera's +x axis with
    //    its out-of-plane component removed, so at zero roll the square's edges stay
    //    parallel to the image rows; e2 = n x e1 then runs down-range in the plane.
    cv::Vec3d e1 = cv::Vec3d(1, 0, 0) - n * n[0];
    if (cv::norm(e1) < 1e-6) e1 = cv::Vec3d(0, 1, 0) - n * n[1];   // camera rolled ~90 deg
    e1 = cv::normalize(e1);
    const cv::Vec3d e2 = cv::normalize(n.cross(e1));

    // 3. The four corners of the square, projected into the depth image.
    const double h = 0.5 * patch_mm;
    const cv::Vec3d corner[4] = { C - e1 * h - e2 * h, C + e1 * h - e2 * h,
                                  C + e1 * h + e2 * h, C - e1 * h + e2 * h };
    for (int i = 0; i < 4; ++i) {
        if (corner[i][2] <= 1e-6) return out;                      // corner behind the camera
        float p3[3] = {float(corner[i][0]), float(corner[i][1]), float(corner[i][2])};
        float px[2];
        rs2_project_point_to_pixel(px, &intr, p3);
        if (!std::isfinite(px[0]) || !std::isfinite(px[1])) return out;
        out.quad[i] = cv::Point2f(px[0], px[1]);
    }
    out.centre_axis_mm = C[2];

    float min_u = out.quad[0].x, max_u = min_u, min_v = out.quad[0].y, max_v = min_v;
    for (int i = 1; i < 4; ++i) {
        min_u = std::min(min_u, out.quad[i].x); max_u = std::max(max_u, out.quad[i].x);
        min_v = std::min(min_v, out.quad[i].y); max_v = std::max(max_v, out.quad[i].y);
    }

    // 4. Axis-aligned rectangle for that quad.
    if (outer) {
        // Smallest box that covers the whole patch (may include floor outside it).
        out.rect = cv::Rect(cv::Point(int(std::floor(min_u)), int(std::floor(min_v))),
                            cv::Point(int(std::ceil (max_u)), int(std::ceil (max_v))));
    } else {
        // Largest box centred on the patch centre that still fits *inside* the quad,
        // so every ROI pixel is guaranteed to land on the patch. Bisect the scale of
        // the bounding box until all four of its corners are inside.
        float p3[3] = {float(C[0]), float(C[1]), float(C[2])}, cpx[2];
        rs2_project_point_to_pixel(cpx, &intr, p3);
        const float hw = 0.5f * (max_u - min_u), hh = 0.5f * (max_v - min_v);
        float lo = 0.f, hi = 1.f;
        for (int it = 0; it < 24; ++it) {
            const float t = 0.5f * (lo + hi);
            const cv::Point2f c[4] = {{cpx[0] - t * hw, cpx[1] - t * hh}, {cpx[0] + t * hw, cpx[1] - t * hh},
                                      {cpx[0] + t * hw, cpx[1] + t * hh}, {cpx[0] - t * hw, cpx[1] + t * hh}};
            bool in = true;
            for (int k = 0; k < 4 && in; ++k) in = point_in_quad(out.quad, c[k]);
            if (in) lo = t; else hi = t;
        }
        out.rect = cv::Rect(cv::Point(int(std::lround(cpx[0] - lo * hw)), int(std::lround(cpx[1] - lo * hh))),
                            cv::Point(int(std::lround(cpx[0] + lo * hw)), int(std::lround(cpx[1] + lo * hh))));
    }
    out.ok = out.rect.width > 0 && out.rect.height > 0;
    return out;
}

// ── Metrics ───────────────────────────────────────────────────────────────────
struct FrameMetrics {
    long long frame = 0;
    double ts_ms = NaN, device_ts_ms = NaN;
    cv::Rect roi;
    int    roi_px = 0, valid = 0;
    double fill_pct = NaN, mean_z_mm = NaN, std_z_mm = NaN;
    // vs ground truth
    bool   has_gt = false;
    double gt_axis_mm = NaN;
    double z_err_median_mm = NaN, z_err_median_pct = NaN;
    double z_err_mean_mm = NaN, z_err_mae_mm = NaN, z_err_rmse_mm = NaN;
    // plane fit (independent of ground truth)
    bool   plane_ok = false;
    double plane_rms_mm = NaN, plane_rms_pct = NaN, subpixel_rms_px = NaN;
    double plane_dist_perp_mm = NaN, plane_dist_axis_mm = NaN;
    double plane_pitch_deg = NaN, plane_roll_deg = NaN, plane_tilt_deg = NaN;
    // fitted plane vs ground truth
    double dist_err_mm = NaN, pitch_err_deg = NaN, roll_err_deg = NaN;
    // conditions
    double ir_mean = NaN, exposure = NaN, gain = NaN, laser_power = NaN, emitter_mode = NaN;
    rs2_vector accel{NAN, NAN, NAN};
};

static double median_of(std::vector<double>& v)
{
    if (v.empty()) return NaN;
    size_t m = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + m, v.end());
    double hi = v[m];
    if (v.size() % 2) return hi;
    double lo = *std::max_element(v.begin(), v.begin() + m);
    return 0.5 * (lo + hi);
}

struct PixelSample { double x, y, z; int idx; };

struct Plane { double a = 0, b = 0, c = 1, d = 0; bool ok = false; };

// Least-squares plane (smallest-eigenvector of the scatter matrix).
static Plane fit_plane(const std::vector<PixelSample>& pts)
{
    Plane p;
    if (pts.size() < 3) return p;
    double mx = 0, my = 0, mz = 0;
    for (const auto& q : pts) { mx += q.x; my += q.y; mz += q.z; }
    const double inv = 1.0 / pts.size();
    mx *= inv; my *= inv; mz *= inv;

    double xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
    for (const auto& q : pts) {
        double dx = q.x - mx, dy = q.y - my, dz = q.z - mz;
        xx += dx * dx; xy += dx * dy; xz += dx * dz;
        yy += dy * dy; yz += dy * dz; zz += dz * dz;
    }
    cv::Matx33d C(xx, xy, xz,
                  xy, yy, yz,
                  xz, yz, zz);
    cv::Mat evals, evecs;
    if (!cv::eigen(cv::Mat(C), evals, evecs)) return p;     // eigenvalues descending
    cv::Vec3d n(evecs.at<double>(2, 0), evecs.at<double>(2, 1), evecs.at<double>(2, 2));
    if (n[2] < 0) n = -n;
    p.a = n[0]; p.b = n[1]; p.c = n[2];
    p.d = -(n[0] * mx + n[1] * my + n[2] * mz);
    p.ok = p.c > 1e-6;
    return p;
}

class DepthAnalyzer {
public:
    void configure(const rs2_intrinsics& in, double depth_scale_m, double baseline_mm)
    {
        intr_        = in;
        scale_mm_    = depth_scale_m * 1000.0;
        baseline_mm_ = baseline_mm;
        rays_.assign(size_t(in.width) * in.height, cv::Vec3d(0, 0, 1));
        for (int v = 0; v < in.height; ++v)
            for (int u = 0; u < in.width; ++u) {
                float px[2] = {float(u), float(v)};
                float pt[3];
                rs2_deproject_pixel_to_point(pt, &intr_, px, 1.0f);
                rays_[size_t(v) * in.width + u] = cv::Vec3d(pt[0], pt[1], pt[2]);
            }
        tn_sum_.assign(rays_.size(), 0.0);
        tn_sum2_.assign(rays_.size(), 0.0);
        tn_count_.assign(rays_.size(), 0);
        set_roi(roi_);
    }

    // reset_stats = false keeps the per-pixel temporal accumulators alive. That is
    // what the dynamic-ROI path needs: the ROI is nudged by a few pixels as the
    // camera moves and wiping the history every time would leave the temporal-noise
    // figure undefined. Pixels that only enter the ROI late are dropped anyway by
    // the min_count filter in temporal_noise_median_mm().
    void set_roi(const cv::Rect& r, bool reset_stats = true)
    {
        roi_ = r & cv::Rect(0, 0, intr_.width, intr_.height);
        err_mm_.create(std::max(roi_.height, 1), std::max(roi_.width, 1), CV_32F);
        err_mm_.setTo(NaN);
        if (reset_stats) reset_temporal();
    }

    void set_gt(const GroundTruth& g) { gt_ = g; }

    void reset_temporal()
    {
        std::fill(tn_sum_.begin(),   tn_sum_.end(),   0.0);
        std::fill(tn_sum2_.begin(),  tn_sum2_.end(),  0.0);
        std::fill(tn_count_.begin(), tn_count_.end(), 0);
    }

    const cv::Rect& roi()          const { return roi_; }
    const cv::Mat&  error_map()    const { return err_mm_; }
    const Plane&    fitted_plane() const { return plane_; }   // last fit, for the dynamic ROI

    FrameMetrics analyze(const rs2::depth_frame& depth)
    {
        FrameMetrics m;
        plane_   = Plane{};
        m.roi    = roi_;
        m.roi_px = roi_.area();
        const int w = depth.get_width(), h = depth.get_height();
        if (w != intr_.width || h != intr_.height || roi_.area() == 0) return m;

        const auto* data   = static_cast<const uint16_t*>(depth.get_data());
        const int   stride = depth.get_stride_in_bytes() / 2;

        samples_.clear();
        double sum_z = 0, sum_z2 = 0;
        for (int v = roi_.y; v < roi_.y + roi_.height; ++v) {
            const uint16_t* row = data + size_t(v) * stride;
            for (int u = roi_.x; u < roi_.x + roi_.width; ++u) {
                uint16_t raw = row[u];
                if (!raw) continue;
                int    idx = v * w + u;
                double z   = raw * scale_mm_;
                const cv::Vec3d& r = rays_[idx];
                samples_.push_back({r[0] * z, r[1] * z, z, idx});
                sum_z += z; sum_z2 += z * z;
                tn_sum_[idx] += z; tn_sum2_[idx] += z * z; ++tn_count_[idx];
            }
        }

        err_mm_.setTo(NaN);
        m.valid    = int(samples_.size());
        m.fill_pct = 100.0 * m.valid / m.roi_px;
        if (m.valid < 3) return m;
        m.mean_z_mm = sum_z / m.valid;
        m.std_z_mm  = std::sqrt(std::max(0.0, sum_z2 / m.valid - m.mean_z_mm * m.mean_z_mm));

        // ── Plane fit metrics (Depth-Quality-Tool style) ────────────────────
        Plane p = fit_plane(samples_);
        plane_ = p;
        if (p.ok) {
            const double fB = intr_.fx * baseline_mm_;
            double ss = 0, ss_sub = 0; int n_sub = 0;
            for (const auto& s : samples_) {
                double res = p.a * s.x + p.b * s.y + p.c * s.z + p.d;   // orthogonal distance
                ss += res * res;
                if (fB > 0) {
                    const cv::Vec3d& r = rays_[s.idx];
                    double den = p.a * r[0] + p.b * r[1] + p.c * r[2];
                    if (den > 1e-9) {
                        double z_fit = -p.d / den;                       // plane depth on this ray
                        double e = fB * (1.0 / s.z - 1.0 / z_fit);       // disparity error [px]
                        ss_sub += e * e; ++n_sub;
                    }
                }
            }
            m.plane_ok           = true;
            m.plane_rms_mm       = std::sqrt(ss / m.valid);
            m.plane_dist_perp_mm = -p.d;
            m.plane_dist_axis_mm = -p.d / p.c;
            m.plane_rms_pct      = 100.0 * m.plane_rms_mm / m.plane_dist_axis_mm;
            m.subpixel_rms_px    = n_sub ? std::sqrt(ss_sub / n_sub) : NaN;
            m.plane_pitch_deg    = std::atan2(p.b, p.c) * R2D;
            m.plane_roll_deg     = std::atan2(-p.a, p.c) * R2D;
            m.plane_tilt_deg     = std::acos(std::clamp(p.c, -1.0, 1.0)) * R2D;
        }

        // ── Per-pixel error along each ray ──────────────────────────────────
        // Reference = ground-truth plane if given, else the fitted plane.
        errs_.clear(); errs_pct_.clear();
        double se = 0, sae = 0, sum_e = 0;
        for (const auto& s : samples_) {
            const cv::Vec3d& r = rays_[s.idx];
            double z_ref;
            if (gt_.valid) {
                double den = gt_.n.dot(r);
                if (den <= 1e-9) continue;
                z_ref = gt_.H_mm / den;
            } else if (p.ok) {
                double den = p.a * r[0] + p.b * r[1] + p.c * r[2];
                if (den <= 1e-9) continue;
                z_ref = -p.d / den;
            } else {
                continue;
            }
            double e = s.z - z_ref;
            err_mm_.at<float>(s.idx / w - roi_.y, s.idx % w - roi_.x) = float(e);
            if (gt_.valid) {
                errs_.push_back(e);
                errs_pct_.push_back(100.0 * e / z_ref);
                se += e * e; sae += std::fabs(e); sum_e += e;
            }
        }

        if (gt_.valid && !errs_.empty()) {
            const double n = double(errs_.size());
            m.has_gt           = true;
            m.gt_axis_mm       = gt_.axis_mm;
            m.z_err_mean_mm    = sum_e / n;
            m.z_err_mae_mm     = sae / n;
            m.z_err_rmse_mm    = std::sqrt(se / n);
            m.z_err_median_mm  = median_of(errs_);
            m.z_err_median_pct = median_of(errs_pct_);
            if (m.plane_ok) {
                m.dist_err_mm   = m.plane_dist_perp_mm - gt_.H_mm;
                m.pitch_err_deg = m.plane_pitch_deg - gt_.pitch_deg;
                m.roll_err_deg  = m.plane_roll_deg  - gt_.roll_deg;
            }
        }
        return m;
    }

    // Median over ROI pixels of the per-pixel standard deviation through time.
    double temporal_noise_median_mm(int min_count) const
    {
        std::vector<double> sds;
        min_count = std::max(min_count, 2);
        for (int v = roi_.y; v < roi_.y + roi_.height; ++v)
            for (int u = roi_.x; u < roi_.x + roi_.width; ++u) {
                size_t idx = size_t(v) * intr_.width + u;
                int c = tn_count_[idx];
                if (c < min_count) continue;
                double mean = tn_sum_[idx] / c;
                double var  = (tn_sum2_[idx] - c * mean * mean) / (c - 1);
                sds.push_back(std::sqrt(std::max(0.0, var)));
            }
        return median_of(sds);
    }

private:
    rs2_intrinsics           intr_{};
    double                   scale_mm_ = 0.1, baseline_mm_ = 0;
    std::vector<cv::Vec3d>   rays_;
    cv::Rect                 roi_;
    Plane                    plane_;
    GroundTruth              gt_;
    cv::Mat                  err_mm_;
    std::vector<PixelSample> samples_;
    std::vector<double>      errs_, errs_pct_;
    std::vector<double>      tn_sum_, tn_sum2_;
    std::vector<int>         tn_count_;
};

// ── Console / CSV output ──────────────────────────────────────────────────────
static std::string fmt(double v, int prec)
{
    std::ostringstream s;
    s << std::fixed << std::setprecision(prec) << v;
    return s.str();
}

static void csv_num(std::ostream& f, double v, int prec)
{
    if (std::isfinite(v)) f << std::fixed << std::setprecision(prec) << v;
    else                  f << "NaN";
}

static void print_frame_line(const FrameMetrics& m)
{
    std::ostringstream s;
    s << std::fixed << "f" << std::setw(6) << m.frame
      << " | fill " << std::setprecision(1) << std::setw(5) << m.fill_pct << "%";
    if (m.has_gt)
        s << " | Zerr med " << std::showpos << std::setprecision(2) << m.z_err_median_mm
          << "mm (" << m.z_err_median_pct << "%)" << std::noshowpos
          << " rmse " << m.z_err_rmse_mm << "mm";
    if (m.plane_ok)
        s << " | plane rms " << std::setprecision(2) << m.plane_rms_mm << "mm"
          << " sub " << std::setprecision(3) << m.subpixel_rms_px << "px"
          << " | pitch " << std::setprecision(2) << m.plane_pitch_deg
          << " roll " << m.plane_roll_deg;
    if (m.has_gt && m.plane_ok)
        s << " | dist err " << std::showpos << std::setprecision(2) << m.dist_err_mm << "mm";
    std::cout << s.str() << "\n";
}

static std::ofstream open_metrics_csv(const std::string& path)
{
    std::ofstream f(path);
    f << "frame,timestamp_ms,device_ts_ms,roi_x,roi_y,roi_w,roi_h,roi_px,valid_px,fill_pct,"
         "mean_z_mm,std_z_mm,gt_axis_mm,z_err_median_mm,z_err_median_pct,z_err_mean_mm,"
         "z_err_mae_mm,z_err_rmse_mm,plane_rms_mm,plane_rms_pct,subpixel_rms_px,"
         "plane_dist_perp_mm,plane_dist_axis_mm,plane_pitch_deg,plane_roll_deg,plane_tilt_deg,"
         "dist_err_mm,pitch_err_deg,roll_err_deg,ir_mean,exposure,gain,laser_power,emitter_mode,"
         "accel_x,accel_y,accel_z\n";
    return f;
}

static void log_metrics(std::ofstream& f, const FrameMetrics& m)
{
    f << m.frame << ",";
    csv_num(f, m.ts_ms, 3);        f << ",";
    csv_num(f, m.device_ts_ms, 3); f << ",";
    f << m.roi.x << "," << m.roi.y << "," << m.roi.width << "," << m.roi.height << ","
      << m.roi_px << "," << m.valid << ",";
    const double cols[] = {
        m.fill_pct, m.mean_z_mm, m.std_z_mm, m.gt_axis_mm,
        m.z_err_median_mm, m.z_err_median_pct, m.z_err_mean_mm, m.z_err_mae_mm, m.z_err_rmse_mm,
        m.plane_rms_mm, m.plane_rms_pct, m.subpixel_rms_px,
        m.plane_dist_perp_mm, m.plane_dist_axis_mm,
        m.plane_pitch_deg, m.plane_roll_deg, m.plane_tilt_deg,
        m.dist_err_mm, m.pitch_err_deg, m.roll_err_deg,
        m.ir_mean, m.exposure, m.gain, m.laser_power, m.emitter_mode,
        m.accel.x, m.accel.y, m.accel.z};
    const size_t n = sizeof(cols) / sizeof(cols[0]);
    for (size_t i = 0; i < n; ++i) {
        csv_num(f, cols[i], 4);
        f << (i + 1 < n ? "," : "\n");
    }
}

// Mean / std across frames, ignoring NaNs.
struct Stat { double mean = NaN, std = NaN; };
static Stat stat_of(const std::vector<FrameMetrics>& h, double FrameMetrics::* field)
{
    double s = 0, s2 = 0; int n = 0;
    for (const auto& m : h) {
        double v = m.*field;
        if (!std::isfinite(v)) continue;
        s += v; s2 += v * v; ++n;
    }
    Stat r;
    if (n == 0) return r;
    r.mean = s / n;
    r.std  = n > 1 ? std::sqrt(std::max(0.0, (s2 - n * r.mean * r.mean) / (n - 1))) : 0.0;
    return r;
}

struct SummaryField { const char* name; double FrameMetrics::* field; };
static const SummaryField SUMMARY_FIELDS[] = {
    {"fill_pct",         &FrameMetrics::fill_pct},
    {"z_err_median_mm",  &FrameMetrics::z_err_median_mm},
    {"z_err_median_pct", &FrameMetrics::z_err_median_pct},
    {"z_err_rmse_mm",    &FrameMetrics::z_err_rmse_mm},
    {"plane_rms_mm",     &FrameMetrics::plane_rms_mm},
    {"plane_rms_pct",    &FrameMetrics::plane_rms_pct},
    {"subpixel_rms_px",  &FrameMetrics::subpixel_rms_px},
    {"plane_dist_perp_mm", &FrameMetrics::plane_dist_perp_mm},
    {"dist_err_mm",      &FrameMetrics::dist_err_mm},
    {"plane_pitch_deg",  &FrameMetrics::plane_pitch_deg},
    {"plane_roll_deg",   &FrameMetrics::plane_roll_deg},
    {"ir_mean",          &FrameMetrics::ir_mean},
    {"exposure",         &FrameMetrics::exposure},
    {"gain",             &FrameMetrics::gain},
};

// ── Session directory helpers ─────────────────────────────────────────────────
static std::string timestamp_str()
{
    std::time_t t  = std::time(nullptr);
    std::tm     tm = *std::localtime(&t);
    char buf[20];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
    return buf;
}

static bool mkdir_p(const std::string& path)
{
    std::string cur = (!path.empty() && path[0] == '/') ? "/" : "";
    std::stringstream ss(path);
    std::string part;
    while (std::getline(ss, part, '/')) {
        if (part.empty()) continue;
        cur += part;
        if (mkdir(cur.c_str(), 0755) != 0 && errno != EEXIST) return false;
        cur += "/";
    }
    return true;
}

static std::string make_session_dir(const std::string& parent, const std::string& name)
{
    if (!mkdir_p(parent)) {
        std::cerr << "Failed to create directory: " << parent << " (" << std::strerror(errno) << ")\n";
        std::exit(1);
    }
    std::string dir = parent + "/" + name;
    if (mkdir(dir.c_str(), 0755) != 0) {
        std::cerr << "Failed to create session directory: " << dir
                  << " (" << std::strerror(errno) << ")\n";
        std::exit(1);
    }
    return dir;
}

static std::string sanitize(const std::string& s)
{
    std::string out;
    for (char c : s) out += (std::isalnum((unsigned char)c) || c == '-' || c == '_' || c == '.') ? c : '_';
    return out;
}

static std::string trial_name(const std::string& run_ts, int idx, const TrialSetup& s)
{
    std::string n = run_ts + "_t" + std::to_string(idx);
    if (s.height_mm > 0)    n += "_h" + fmt(s.height_mm, 0);
    else if (s.axis_mm > 0) n += "_z" + fmt(s.axis_mm, 0);
    else                    n += "_noGT";
    n += "_p" + fmt(s.pitch_deg, 1) + "_r" + fmt(s.roll_deg, 1);
    if (!s.label.empty()) n += "_" + sanitize(s.label);
    return n;
}

static bool parse_roi(const std::string& s, cv::Rect& r)
{
    int x, y, w, h;
    if (std::sscanf(s.c_str(), "%d,%d,%d,%d", &x, &y, &w, &h) != 4 || w <= 0 || h <= 0) return false;
    r = cv::Rect(x, y, w, h);
    return true;
}

static cv::Rect centred_roi(int w, int h, double pct)
{
    pct = std::clamp(pct, 1.0, 100.0);
    int rw = int(w * pct / 100.0), rh = int(h * pct / 100.0);
    return cv::Rect((w - rw) / 2, (h - rh) / 2, rw, rh);
}

// ── Between-trial prompt ──────────────────────────────────────────────────────
// Returns false when the user wants to finish.
static bool prompt_next_trial(Options& o)
{
    std::cout << "\n" << std::string(60, '-') << "\n"
              << "Adjust the rig / lighting, then enter any changes as key=value pairs:\n"
              << "  h=<mm> z=<mm> pitch=<deg> roll=<deg> lux=<val> label=<text> gt=none\n"
              << "  roi=x,y,w,h | roi=select   patch=<mm>|off\n"
              << "  emitter=on|off   laser=<mW>   exposure=auto|<us>\n"
              << "Enter = repeat with the same settings, q = finish.\n> " << std::flush;
    std::string line;
    if (!std::getline(std::cin, line) || !g_running) return false;
    if (!line.empty() && (line[0] == 'q' || line[0] == 'Q')) return false;

    TrialSetup& s = o.setup;
    std::istringstream ss(line);
    std::string tok;
    while (ss >> tok) {
        auto eq = tok.find('=');
        if (eq == std::string::npos) { std::cerr << "  ignored '" << tok << "' (expected key=value)\n"; continue; }
        std::string k = tok.substr(0, eq), v = tok.substr(eq + 1);
        try {
            if      (k == "h")        { s.height_mm = std::stod(v); s.axis_mm = 0; }
            else if (k == "z")        { s.axis_mm = std::stod(v); s.height_mm = 0; }
            else if (k == "pitch")    s.pitch_deg = std::stod(v);
            else if (k == "roll")     s.roll_deg  = std::stod(v);
            else if (k == "lux")      s.lux       = std::stod(v);
            else if (k == "label")    s.label     = v;
            else if (k == "gt" && v == "none") { s.height_mm = 0; s.axis_mm = 0; }
            else if (k == "roi") {
                if (v == "select") o.select_roi = true;
                else if (parse_roi(v, o.roi)) { o.roi_given = true; o.select_roi = false; }
                else std::cerr << "  bad roi '" << v << "'\n";
            }
            else if (k == "patch")    o.patch_size = (v == "off") ? 0.0 : std::max(0.0, std::stod(v));
            else if (k == "emitter")  o.emitter = (v == "on") ? 1 : 0;
            else if (k == "laser")    o.laser_power = std::stof(v);
            else if (k == "exposure") o.exposure_us = (v == "auto") ? -1.0f : std::stof(v);
            else std::cerr << "  unknown key '" << k << "'\n";
        } catch (...) {
            std::cerr << "  bad value for '" << k << "': " << v << "\n";
        }
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
    Options o;

    try {
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            auto next = [&]() -> std::string {
                if (i + 1 >= argc) throw std::runtime_error("missing value for " + arg);
                return argv[++i];
            };
            if      (arg == "-H" || arg == "--height")    { o.setup.height_mm = std::stod(next()); o.setup.axis_mm = 0; }
            else if (arg == "-Z" || arg == "--axis-dist") { o.setup.axis_mm = std::stod(next()); o.setup.height_mm = 0; }
            else if (arg == "-p" || arg == "--pitch")     o.setup.pitch_deg = std::stod(next());
            else if (arg == "-R" || arg == "--roll")      o.setup.roll_deg  = std::stod(next());
            else if (arg == "--gt-offset")                o.gt_offset_mm    = std::stod(next());
            else if (arg == "-L" || arg == "--label")     o.setup.label     = next();
            else if (arg == "--lux")                      o.setup.lux       = std::stod(next());
            else if (arg == "-r" || arg == "--roi") {
                if (!parse_roi(next(), o.roi)) throw std::runtime_error("bad --roi, expected X,Y,W,H");
                o.roi_given = true;
            }
            else if (arg == "--roi-pct")                  o.roi_pct = std::stod(next());
            else if (arg == "--select-roi")               o.select_roi = true;
            else if (arg == "--patch-size")               o.patch_size = std::max(0.0, std::stod(next()));
            else if (arg == "--patch-outer")              o.patch_outer = true;
            else if (arg == "--patch-smooth")             o.patch_smooth = std::clamp(std::stod(next()), 0.01, 1.0);
            else if (arg == "--patch-hyst")               o.patch_hyst_px = std::max(0, std::stoi(next()));
            else if (arg == "-D" || arg == "--dir")       o.output_dir = next();
            else if (arg == "-d" || arg == "--duration")  o.duration_sec = std::stoi(next());
            else if (arg == "-n" || arg == "--frames")    o.frame_limit  = std::stoll(next());
            else if (arg == "--res") {
                if (std::sscanf(next().c_str(), "%dx%d", &o.width, &o.height) != 2)
                    throw std::runtime_error("bad --res, expected WxH");
            }
            else if (arg == "--fps")                      o.fps = std::stoi(next());
            else if (arg == "--preset") {
                o.preset = next();
                if (preset_value(o.preset) < 0) throw std::runtime_error("unknown preset " + o.preset);
            }
            else if (arg == "--emitter")                  o.emitter = (next() == "on") ? 1 : 0;
            else if (arg == "--laser-power")              o.laser_power = std::stof(next());
            else if (arg == "--exposure")                 o.exposure_us = std::stof(next());
            else if (arg == "--filters")                  o.use_filters = true;
            else if (arg == "--min-dist")                 o.min_dist = std::stof(next());
            else if (arg == "--max-dist")                 o.max_dist = std::stof(next());
            else if (arg == "--ext")                      o.bag_ext = next();
            else if (arg == "-noRecord" || arg == "--noRecord") o.record = false;
            else if (arg == "-noColor"  || arg == "--noColor")  o.use_color = false;
            else if (arg == "-noIMU"    || arg == "--noIMU")    o.use_imu = false;
            else if (arg == "--print-every")              o.print_every = std::max(1, std::stoi(next()));
            else if (arg == "--err-range")                o.err_range_mm = std::max(0.1, std::stod(next()));
            else if (arg == "-h" || arg == "--help")      { print_usage(argv[0]); return 0; }
            else throw std::runtime_error("unknown argument " + arg);
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        print_usage(argv[0]);
        return 1;
    }

    if (std::fabs(o.setup.pitch_deg) > 80 || std::fabs(o.setup.roll_deg) > 80) {
        std::cerr << "Error: |pitch| and |roll| must be below 80 degrees.\n"; return 1;
    }

    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    const std::string run_ts       = timestamp_str();
    const std::string session_root = make_session_dir(o.output_dir, run_ts);
    const std::string summary_file = session_root + "/summary.csv";
    std::cout << "Run folder: " << session_root << "\n";

    const char* W_IR = "IR (left) + ROI", *W_DEPTH = "Depth + ROI",
              * W_ERR = "ROI error map", *W_COLOR = "Colour (context only, not aligned)";
    cv::namedWindow(W_IR,    cv::WINDOW_NORMAL); cv::resizeWindow(W_IR,    960, 540);
    cv::namedWindow(W_DEPTH, cv::WINDOW_NORMAL); cv::resizeWindow(W_DEPTH, 640, 360);
    cv::namedWindow(W_ERR,   cv::WINDOW_NORMAL); cv::resizeWindow(W_ERR,   640, 360);
    if (o.use_color) { cv::namedWindow(W_COLOR, cv::WINDOW_NORMAL); cv::resizeWindow(W_COLOR, 480, 270); }

    PipelineCtx   ctx;
    DepthAnalyzer analyzer;
    int  trial_index  = 0;
    bool keep_running = true;

    while (keep_running && g_running)
    {
        if (std::fabs(o.setup.pitch_deg) > 80 || std::fabs(o.setup.roll_deg) > 80) {
            std::cerr << "|pitch| and |roll| must be below 80 degrees — please re-enter.\n";
            keep_running = prompt_next_trial(o);
            continue;
        }

        // ── Trial folder / file names ─────────────────────────────────────────
        const std::string name        = trial_name(run_ts, trial_index, o.setup);
        const std::string trial_dir   = make_session_dir(session_root, name);
        const std::string record_file = o.record ? trial_dir + "/" + name + o.bag_ext : "";
        const std::string csv_file    = trial_dir + "/" + name + "_metrics.csv";
        const std::string info_file   = trial_dir + "/trial_info.txt";

        ctx = start_pipeline_retry(record_file, o, 5);

        std::ofstream csv = open_metrics_csv(csv_file);
        if (!csv.is_open()) { std::cerr << "Failed to open CSV: " << csv_file << "\n"; return 1; }

        // ── Camera model for this trial ───────────────────────────────────────
        auto dev   = ctx.profile.get_device();
        auto ds    = dev.first<rs2::depth_sensor>();
        auto dprof = ctx.profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
        rs2_intrinsics intr     = dprof.get_intrinsics();
        const double   scale_m  = ds.get_depth_scale();
        const double   baseline = stereo_baseline_mm(ds, ctx.profile);

        analyzer.configure(intr, scale_m, baseline);

        cv::Rect roi = o.roi_given ? o.roi : centred_roi(intr.width, intr.height, o.roi_pct);
        roi &= cv::Rect(0, 0, intr.width, intr.height);
        if (roi.area() == 0) {
            std::cerr << "ROI lies outside the " << intr.width << "x" << intr.height
                      << " image — using centred " << o.roi_pct << "% ROI.\n";
            roi = centred_roi(intr.width, intr.height, o.roi_pct);
        }
        analyzer.set_roi(roi);

        const GroundTruth gt = make_gt(o.setup, o.gt_offset_mm);
        analyzer.set_gt(gt);

        // ── Dynamic-ROI state ─────────────────────────────────────────────────
        // The patch stays centred on the ray through the centre of the ROI this
        // trial started with, so -r / --select-roi still choose *where* to look and
        // --patch-size chooses *how much floor* is looked at.
        auto ray_through = [&intr](const cv::Rect& r) {
            float px[2] = {float(r.x + 0.5 * r.width), float(r.y + 0.5 * r.height)};
            float pt[3];
            rs2_deproject_pixel_to_point(pt, &intr, px, 1.0f);
            return cv::Vec3d(pt[0], pt[1], pt[2]);
        };
        cv::Vec3d anchor_ray = ray_through(roi);
        cv::Vec3d plane_n    = gt.n;                  // running plane estimate (EMA)
        double    plane_H    = gt.valid ? gt.H_mm : 0.0;
        bool      plane_seen = gt.valid;
        PatchROI  patch;

        // Seed the ROI from the ground-truth plane so frame 0 is already the right
        // size; without ground truth it starts static and snaps in after frame 0.
        if (o.patch_size > 0 && plane_seen) {
            PatchROI seed = project_patch(intr, plane_n, plane_H, anchor_ray, o.patch_size, o.patch_outer);
            cv::Rect r    = seed.rect & cv::Rect(0, 0, intr.width, intr.height);
            if (seed.ok && r.width >= o.patch_min_px && r.height >= o.patch_min_px) {
                patch = seed; roi = r; analyzer.set_roi(roi);
            }
        }

        auto info = [&](rs2_camera_info k) -> std::string {
            try { return dev.supports(k) ? dev.get_info(k) : "?"; } catch (...) { return "?"; }
        };

        std::cout << "\n" << std::string(60, '=') << "\n"
                  << "Trial   : " << trial_index << "  (" << name << ")\n"
                  << "Device  : " << info(RS2_CAMERA_INFO_NAME) << "  S/N " << info(RS2_CAMERA_INFO_SERIAL_NUMBER)
                  << "  FW " << info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << "\n"
                  << "Streams : Depth+IR " << intr.width << "x" << intr.height << "@" << o.fps
                  << (o.use_color ? " | Colour" : "") << (o.use_imu ? " | Accel" : "") << "\n"
                  << "Preset  : " << o.preset << (o.use_filters ? "  (post-processing ON)" : "  (raw depth)") << "\n"
                  << "Baseline: " << fmt(baseline, 2) << " mm   fx " << fmt(intr.fx, 2) << " px\n"
                  << "ROI     : x=" << roi.x << " y=" << roi.y << " w=" << roi.width << " h=" << roi.height
                  << (o.select_roi ? "  (will be selected on first frame)" : "") << "\n"
                  << (o.patch_size > 0
                        ? "Patch   : " + fmt(o.patch_size, 0) + " mm square on the target plane, "
                          + (o.patch_outer ? "bounding" : "inscribed") + " ROI, tracked every frame"
                          + (plane_seen ? "" : "  (waiting for the first plane fit)") + "\n"
                        : std::string())
                  << "Truth   : " << (gt.valid
                        ? "perp " + fmt(gt.H_mm, 1) + " mm, axis " + fmt(gt.axis_mm, 1) + " mm, pitch "
                          + fmt(gt.pitch_deg, 2) + ", roll " + fmt(gt.roll_deg, 2)
                        : std::string("none (errors shown relative to fitted plane)")) << "\n"
                  << "Record  : " << (record_file.empty() ? "off" : record_file) << "\n"
                  << "CSV     : " << csv_file << "\n"
                  << (o.duration_sec > 0 ? "Duration: " + std::to_string(o.duration_sec) + "s\n" : "")
                  << (o.frame_limit  > 0 ? "Frames  : " + std::to_string(o.frame_limit) + "\n" : "")
                  << "Keys    : q/Esc end trial | r re-select ROI | s snapshot\n"
                  << std::string(60, '-') << "\n";

        DepthFilters   filters(o.min_dist, o.max_dist);
        rs2::colorizer colorizer;

        auto       t_start      = std::chrono::steady_clock::now();
        long long  frame_count  = 0;
        rs2_vector last_accel{NAN, NAN, NAN};
        bool       pending_select = o.select_roi;
        bool       warned_pitch = false, warned_roll = false, warned_dist = false;
        std::vector<FrameMetrics> history;

        while (g_running) {
            if (o.duration_sec > 0) {
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::steady_clock::now() - t_start).count();
                if (elapsed >= o.duration_sec) break;
            }
            if (o.frame_limit > 0 && frame_count >= o.frame_limit) break;

            rs2::frameset frames;
            try {
                frames = ctx.pipe.wait_for_frames();
            } catch (const rs2::error& e) {
                std::cerr << "wait_for_frames: " << e.what() << "\n";
                continue;
            }

            if (auto a = frames.first_or_default(RS2_STREAM_ACCEL))
                last_accel = a.as<rs2::motion_frame>().get_motion_data();

            rs2::depth_frame raw_depth = frames.get_depth_frame();
            if (!raw_depth) continue;
            rs2::video_frame ir = frames.get_infrared_frame(1);
            rs2::video_frame color{rs2::frame{}};
            if (o.use_color) color = frames.get_color_frame();

            rs2::depth_frame depth = o.use_filters ? filters.process(raw_depth) : raw_depth;

            cv::Mat ir_bgr;
            bool ir_ok = ir && ir.get_width() == intr.width && ir.get_height() == intr.height;
            if (ir_ok) cv::cvtColor(y8_view(ir), ir_bgr, cv::COLOR_GRAY2BGR);
            else       ir_bgr = cv::Mat(intr.height, intr.width, CV_8UC3, cv::Scalar(0, 0, 0));

            // ── Interactive ROI selection ─────────────────────────────────────
            if (pending_select) {
                pending_select = false;
                std::cout << "Drag the ROI on the image, then press Enter/Space (c = cancel).\n";
                cv::Rect sel = cv::selectROI("Select ROI", ir_bgr, false);
                cv::destroyWindow("Select ROI");
                if (sel.area() > 0) {
                    analyzer.set_roi(sel);
                    o.roi = analyzer.roi(); o.roi_given = true;
                    anchor_ray = ray_through(o.roi);   // re-anchor the tracked patch
                    patch = PatchROI{};
                    history.clear();
                    std::cout << "ROI set to x=" << o.roi.x << " y=" << o.roi.y
                              << " w=" << o.roi.width << " h=" << o.roi.height
                              << "  (reuse with --roi " << o.roi.x << "," << o.roi.y << ","
                              << o.roi.width << "," << o.roi.height << ")\n";
                }
                o.select_roi = false;
                continue;   // frames piled up during selection — start fresh
            }

            // ── Metrics ───────────────────────────────────────────────────────
            FrameMetrics m  = analyzer.analyze(depth);
            m.frame         = frame_count;
            m.ts_ms         = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - t_start).count();
            m.device_ts_ms  = raw_depth.get_timestamp();
            m.exposure      = frame_md(raw_depth, RS2_FRAME_METADATA_ACTUAL_EXPOSURE);
            m.gain          = frame_md(raw_depth, RS2_FRAME_METADATA_GAIN_LEVEL);
            m.laser_power   = frame_md(raw_depth, RS2_FRAME_METADATA_FRAME_LASER_POWER);
            m.emitter_mode  = frame_md(raw_depth, RS2_FRAME_METADATA_FRAME_EMITTER_MODE);
            m.accel         = last_accel;
            if (ir_ok) m.ir_mean = cv::mean(y8_view(ir)(analyzer.roi()))[0];

            log_metrics(csv, m);
            history.push_back(m);
            if (frame_count % o.print_every == 0) print_frame_line(m);

            // ── Sanity warnings (once per trial) ──────────────────────────────
            if (gt.valid && m.plane_ok && history.size() == 30) {
                if (!warned_pitch && std::fabs(gt.pitch_deg) > 2 && m.plane_pitch_deg * gt.pitch_deg < 0) {
                    std::cout << "!! Fitted pitch has the opposite sign to the given pitch — flip the sign of pitch?\n";
                    warned_pitch = true;
                }
                if (!warned_roll && std::fabs(gt.roll_deg) > 2 && m.plane_roll_deg * gt.roll_deg < 0) {
                    std::cout << "!! Fitted roll has the opposite sign to the given roll — flip the sign of roll?\n";
                    warned_roll = true;
                }
                if (!warned_dist && std::fabs(m.dist_err_mm) > 0.1 * gt.H_mm) {
                    std::cout << "!! Fitted distance is >10% from ground truth — check height, units and --gt-offset.\n";
                    warned_dist = true;
                }
            }

            // ── Display ───────────────────────────────────────────────────────
            const cv::Rect& r = analyzer.roi();
            cv::rectangle(ir_bgr, r, cv::Scalar(0, 255, 0), 2);
            if (o.patch_size > 0 && patch.ok)           // the square patch itself, in amber
                for (int i = 0; i < 4; ++i)
                    cv::line(ir_bgr, patch.quad[i], patch.quad[(i + 1) % 4],
                             cv::Scalar(0, 200, 255), 1, cv::LINE_AA);
            std::vector<std::string> lines = {
                "Trial " + std::to_string(trial_index) + "  frame " + std::to_string(frame_count)
                    + (o.setup.label.empty() ? "" : "  [" + o.setup.label + "]"),
                "Fill " + fmt(m.fill_pct, 1) + "%   mean Z " + fmt(m.mean_z_mm, 1) + " mm",
            };
            if (m.has_gt) {
                lines.push_back("GT axis " + fmt(gt.axis_mm, 1) + " mm  pitch " + fmt(gt.pitch_deg, 1)
                                + "  roll " + fmt(gt.roll_deg, 1));
                lines.push_back("Z err median " + fmt(m.z_err_median_mm, 2) + " mm ("
                                + fmt(m.z_err_median_pct, 2) + "%)  RMSE " + fmt(m.z_err_rmse_mm, 2) + " mm");
            }
            if (m.plane_ok) {
                lines.push_back("Plane RMS " + fmt(m.plane_rms_mm, 2) + " mm (" + fmt(m.plane_rms_pct, 2)
                                + "%)  subpix " + fmt(m.subpixel_rms_px, 3) + " px");
                lines.push_back("Fit: axis " + fmt(m.plane_dist_axis_mm, 1) + " mm  pitch "
                                + fmt(m.plane_pitch_deg, 2) + "  roll " + fmt(m.plane_roll_deg, 2));
            }
            if (o.patch_size > 0)
                lines.push_back("Patch " + fmt(o.patch_size, 0) + " mm -> ROI "
                                + std::to_string(r.width) + "x" + std::to_string(r.height) + " px"
                                + (patch.ok ? "  @ " + fmt(patch.centre_axis_mm, 0) + " mm"
                                            : std::string("  (no plane yet)")));
            lines.push_back("IR mean " + fmt(m.ir_mean, 1) + "   exposure " + fmt(m.exposure, 0));
            draw_text_block(ir_bgr, lines, cv::Point(15, 30));

            auto dcol = colorizer.colorize(depth).as<rs2::video_frame>();
            cv::Mat depth_bgr = rgb_to_bgr(dcol);
            cv::rectangle(depth_bgr, r, cv::Scalar(255, 255, 255), 2);

            cv::Mat err_bgr = render_error_map(analyzer.error_map(), o.err_range_mm);
            if (err_bgr.cols < 480) {
                double k = 480.0 / err_bgr.cols;
                cv::resize(err_bgr, err_bgr, cv::Size(), k, k, cv::INTER_NEAREST);
            }
            draw_text_block(err_bgr, {
                std::string(gt.valid ? "vs ground truth" : "vs fitted plane") + "  blue=closer red=farther",
                "range +/-" + fmt(o.err_range_mm, 1) + " mm, black = hole"}, cv::Point(10, 25), 0.5);

            cv::imshow(W_IR, ir_bgr);
            cv::imshow(W_DEPTH, depth_bgr);
            cv::imshow(W_ERR, err_bgr);
            if (o.use_color && color) cv::imshow(W_COLOR, rgb_to_bgr(color));

            int key = cv::waitKey(1) & 0xFF;
            if (key == 'q' || key == 27) break;
            if (key == 'r') pending_select = true;
            if (key == 's') {
                std::string base = trial_dir + "/snap_f" + std::to_string(frame_count);
                cv::imwrite(base + "_ir.png", ir_bgr);
                cv::imwrite(base + "_depth.png", depth_bgr);
                cv::imwrite(base + "_err.png", err_bgr);
                std::cout << "Saved snapshot " << base << "_*.png\n";
            }

            // ── Dynamic ROI: re-project the patch for the next frame ─────────
            // Plane estimate comes from this frame's own fit (so it follows the
            // floor as the scan moves); ground truth is only the fallback.
            if (o.patch_size > 0) {
                cv::Vec3d n_meas(0, 0, 1);
                double    H_meas = 0.0;
                bool      have   = false;
                if (m.plane_ok) {
                    const Plane& pf = analyzer.fitted_plane();
                    n_meas = cv::normalize(cv::Vec3d(pf.a, pf.b, pf.c));
                    H_meas = -pf.d;                     // perpendicular distance, mm
                    have   = H_meas > 0;
                } else if (gt.valid) {
                    n_meas = gt.n; H_meas = gt.H_mm; have = true;
                }

                if (have) {
                    if (!plane_seen) { plane_n = n_meas; plane_H = H_meas; plane_seen = true; }
                    else {                              // EMA, so the ROI does not jitter
                        const double a = o.patch_smooth;
                        plane_n = cv::normalize(plane_n * (1.0 - a) + n_meas * a);
                        plane_H = plane_H * (1.0 - a) + H_meas * a;
                    }

                    PatchROI np = project_patch(intr, plane_n, plane_H, anchor_ray,
                                                o.patch_size, o.patch_outer);
                    cv::Rect nr = np.rect & cv::Rect(0, 0, intr.width, intr.height);
                    if (np.ok && nr.width >= o.patch_min_px && nr.height >= o.patch_min_px) {
                        patch = np;
                        const cv::Rect& cur = analyzer.roi();
                        const int moved = std::max(std::max(std::abs(nr.x - cur.x), std::abs(nr.y - cur.y)),
                                                   std::max(std::abs(nr.width  - cur.width),
                                                            std::abs(nr.height - cur.height)));
                        // Hysteresis: a sub-pixel wobble in the plane fit should not
                        // resize the ROI (and reallocate the error map) every frame.
                        if (moved > o.patch_hyst_px)
                            analyzer.set_roi(nr, /*reset_stats=*/false);
                    }
                }
            }

            ++frame_count;
        } // end frame loop

        ctx.pipe.stop();
        csv.close();

        // ── Trial summary ─────────────────────────────────────────────────────
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t_start).count();
        const int    n_frames = int(history.size());
        const double tnoise   = analyzer.temporal_noise_median_mm(std::max(2, n_frames / 2));
        const cv::Rect& r     = analyzer.roi();

        std::ostringstream sum;
        sum << (o.patch_size > 0 ? "Frames: " : "Frames (since last ROI change): ") << n_frames << "\n"
            << "Duration: " << fmt(elapsed, 2) << " s\n";
        if (o.patch_size > 0 && n_frames > 0) {
            int wmin = std::numeric_limits<int>::max(), wmax = 0;
            int hmin = std::numeric_limits<int>::max(), hmax = 0;
            for (const auto& f : history) {
                wmin = std::min(wmin, f.roi.width);  wmax = std::max(wmax, f.roi.width);
                hmin = std::min(hmin, f.roi.height); hmax = std::max(hmax, f.roi.height);
            }
            sum << "Patch: " << fmt(o.patch_size, 0) << " mm square ("
                << (o.patch_outer ? "bounding" : "inscribed") << ") -> ROI width "
                << wmin << ".." << wmax << " px, height " << hmin << ".." << hmax
                << " px over the trial (per-frame ROI is in the metrics CSV)\n";
        }
        for (const auto& f : SUMMARY_FIELDS) {
            Stat s = stat_of(history, f.field);
            sum << "  " << std::left << std::setw(20) << f.name << std::right
                << fmt(s.mean, 4) << "  +/- " << fmt(s.std, 4) << "\n";
        }
        sum << "  " << std::left << std::setw(20) << "temporal_noise_mm" << std::right
            << fmt(tnoise, 4) << "  (median per-pixel std over time)\n";

        std::cout << "\n" << std::string(60, '-') << "\n" << sum.str();

        {
            std::ofstream f(info_file);
            f << "trial: "        << trial_index << "\nname: " << name << "\n"
              << "device: "       << info(RS2_CAMERA_INFO_NAME)
              << "\nserial: "     << info(RS2_CAMERA_INFO_SERIAL_NUMBER)
              << "\nfirmware: "   << info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << "\n"
              << "label: "        << o.setup.label << "\n"
              << "height_mm: "    << o.setup.height_mm << "\naxis_dist_mm: " << o.setup.axis_mm << "\n"
              << "pitch_deg: "    << o.setup.pitch_deg << "\nroll_deg: " << o.setup.roll_deg << "\n"
              << "lux: "          << o.setup.lux << "\ngt_offset_mm: " << o.gt_offset_mm << "\n"
              << "gt_valid: "     << gt.valid << "\ngt_perp_mm: " << gt.H_mm << "\ngt_axis_mm: " << gt.axis_mm << "\n"
              << "resolution: "   << intr.width << "x" << intr.height << "@" << o.fps << "\n"
              << "intrinsics: fx=" << intr.fx << " fy=" << intr.fy << " ppx=" << intr.ppx << " ppy=" << intr.ppy << "\n"
              << "depth_scale_m: " << scale_m << "\nbaseline_mm: " << baseline << "\n"
              << "preset: "       << o.preset << "\nfilters: " << o.use_filters << "\n"
              << "emitter_set: "  << o.emitter << "\nlaser_power_set: " << o.laser_power
              << "\nexposure_set_us: " << o.exposure_us << "\n"
              << "roi: "          << r.x << "," << r.y << "," << r.width << "," << r.height
              << (o.patch_size > 0 ? "   (last frame; ROI is tracked)" : "") << "\n"
              << "patch_size_mm: " << o.patch_size << "   (0 = static ROI)\n"
              << "patch_roi_mode: " << (o.patch_size > 0 ? (o.patch_outer ? "outer" : "inner") : "n/a")
              << "\npatch_smooth: " << o.patch_smooth << "\npatch_hyst_px: " << o.patch_hyst_px << "\n"
              << "record: "       << record_file << "\n\n" << sum.str();
        }

        {
            struct stat st;
            bool new_file = stat(summary_file.c_str(), &st) != 0;
            std::ofstream f(summary_file, std::ios::app);
            if (new_file) {
                f << "trial,name,label,height_mm,axis_dist_mm,pitch_deg,roll_deg,lux,gt_offset_mm,"
                     "gt_perp_mm,gt_axis_mm,preset,filters,emitter_set,laser_power_set,exposure_set_us,"
                     "width,height,fps,roi_x,roi_y,roi_w,roi_h,patch_size_mm,frames,temporal_noise_mm";
                for (const auto& fl : SUMMARY_FIELDS) f << "," << fl.name << "_mean," << fl.name << "_std";
                f << "\n";
            }
            f << trial_index << "," << name << "," << sanitize(o.setup.label) << ",";
            csv_num(f, o.setup.height_mm, 2); f << ",";
            csv_num(f, o.setup.axis_mm, 2);   f << ",";
            csv_num(f, o.setup.pitch_deg, 2); f << ",";
            csv_num(f, o.setup.roll_deg, 2);  f << ",";
            csv_num(f, o.setup.lux, 2);       f << ",";
            csv_num(f, o.gt_offset_mm, 2);    f << ",";
            csv_num(f, gt.valid ? gt.H_mm : NaN, 2);    f << ",";
            csv_num(f, gt.valid ? gt.axis_mm : NaN, 2); f << ",";
            f << o.preset << "," << o.use_filters << "," << o.emitter << ","
              << o.laser_power << "," << o.exposure_us << ","
              << intr.width << "," << intr.height << "," << o.fps << ","
              << r.x << "," << r.y << "," << r.width << "," << r.height << ",";
            csv_num(f, o.patch_size, 2);  f << ",";
            f << n_frames << ",";
            csv_num(f, tnoise, 4);
            for (const auto& fl : SUMMARY_FIELDS) {
                Stat s = stat_of(history, fl.field);
                f << ","; csv_num(f, s.mean, 4);
                f << ","; csv_num(f, s.std, 4);
            }
            f << "\n";
        }

        std::cout << "CSV     : " << csv_file << "\n"
                  << "Info    : " << info_file << "\n"
                  << "Summary : " << summary_file << "\n";
        if (!record_file.empty()) std::cout << "Bag     : " << record_file << "\n";

        if (!g_running) break;
        keep_running = prompt_next_trial(o);
        ++trial_index;
    } // end trial loop

    cv::destroyAllWindows();
    std::cout << "Done. Results in " << session_root << "\n";
    return 0;
}
