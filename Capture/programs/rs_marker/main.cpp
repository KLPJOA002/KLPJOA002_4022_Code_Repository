#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <iomanip>
#include <string>
#include <csignal>
#include <atomic>
#include <chrono>
#include <algorithm>
#include <fstream>
#include <ctime>
#include <set>
#include <sstream>
#include <sys/stat.h>
#include <stdio.h>
#include <thread>
#include "mountSerial.h"

static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

static void print_usage(const char* prog)
{
    std::cout << "Usage: " << prog << " [-o output.bag] [-D output_dir] [-d duration_seconds] [-n frames] [-test] [-id]\n"
              << "          [-r WxH] [--fps N] [-u depth_units_m] [-p preset] [-b START:END]\n"
              << "  -o, --output    Record to .bag file (default: no recording)\n"
              << "                  In -test mode: video output path (default: detections_test.mp4)\n"
              << "  -D, --dir       Save bag + CSV into <output_dir>/<YYYYMMDD_HHMMSS>/\n"
              << "  -d, --duration  Duration in seconds     (default: run until q/Ctrl+C)\n"
              << "  -n, --frames    Stop after N frames      (default: no frame limit)\n"
              << "  -test           RGB-only: stream with marker overlays, save annotated video\n"
              << "  -id             RGB-only: detect ALL 36h11 tags, log unique IDs seen\n"
              << "  -nTags N        Expected tags in view; if fewer are found, rerun detection on\n"
              << "                  an illumination-flattened image (default: 4, 0 = off)\n"
              << "  -debugTags      Overlay rejected tag candidates (red) + counts on RGB window\n"
              << "  -r, --res WxH   Depth + colour resolution     (default: 1280x720)\n"
              << "      --fps N     Depth + colour frame rate      (default: 30)\n"
              << "  -u, --units M   Depth units in metres          (default: 0.0001 = 0.1 mm)\n"
              << "  -p, --preset P  Visual preset, applied BEFORE the other depth options (default: high_density)\n"
              << "                  default|high_density|high_accuracy|medium_density|hand|remove_ir_pattern\n"
              << "  -b, --best-range START:END\n"
              << "                  Choose the best frame (fewest holes) only from frames START..END\n"
              << "                  (inclusive, 0-based; either end may be left open, e.g. 30: or :90)\n"
              << "                  (default: all frames)\n"
              << "  -h, --help      Show this message\n";
}

// ── Depth post-processing ─────────────────────────────────────────────────────
struct DepthFilters {
    rs2::decimation_filter   decimation;
    rs2::threshold_filter    threshold;
    rs2::disparity_transform to_disparity{true};
    rs2::spatial_filter      spatial;
    rs2::temporal_filter     temporal;
    rs2::disparity_transform to_depth{false};
    rs2::hole_filling_filter hole_fill;

    DepthFilters()
    {
        threshold.set_option(RS2_OPTION_MIN_DISTANCE,0.1f);
        threshold.set_option(RS2_OPTION_MAX_DISTANCE,1.0f);

        spatial.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.8f); // Spatial filter over small history
        spatial.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 5.f); //threshold for disable low
        spatial.set_option(RS2_OPTION_FILTER_MAGNITUDE, 1.f); // filter magnitude of 1

        temporal.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.06f); //Set temporal filter to apply long filter windows
        temporal.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 100.f); //set the delta threshold high to avoid disabbling on smal temporal changes
        temporal.set_option(RS2_OPTION_HOLES_FILL, 1.f); //select pixel only if valid in the past 8 frames

        hole_fill.set_option(RS2_OPTION_HOLES_FILL, 1.f); //set hole filling option to :something
    }

    rs2::depth_frame process(rs2::depth_frame depth)
    {
        // depth = decimation.process(depth);
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

// Camera settings chosen on the command line (-r, --fps, -u, -p)
struct CamConfig {
    int         width       = 1280;
    int         height      = 720;
    int         fps         = 30;
    float       depth_units = 0.0001f;        // 0.1 mm (device default is 0.001 = 1 mm)
    std::string preset      = "high_density";
};

static int preset_value(const std::string& name)
{
    if (name == "default")           return RS2_RS400_VISUAL_PRESET_DEFAULT;
    if (name == "high_density")      return RS2_RS400_VISUAL_PRESET_HIGH_DENSITY;
    if (name == "high_accuracy")     return RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY;
    if (name == "medium_density")    return RS2_RS400_VISUAL_PRESET_MEDIUM_DENSITY;
    if (name == "hand")              return RS2_RS400_VISUAL_PRESET_HAND;
    if (name == "remove_ir_pattern") return RS2_RS400_VISUAL_PRESET_REMOVE_IR_PATTERN;
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

// Preset goes FIRST: loading a preset rewrites many depth controls, so anything
// set before it would be overwritten.
static void apply_depth_settings(rs2::depth_sensor& ds, const CamConfig& cam)
{
    try_set(ds, RS2_OPTION_VISUAL_PRESET, float(preset_value(cam.preset)), "visual preset");
    try_set(ds, RS2_OPTION_DEPTH_UNITS, cam.depth_units, "depth units");
    try_set(ds, RS2_OPTION_THERMAL_COMPENSATION, 0.0f, "thermal compensation");
}

static PipelineCtx start_pipeline(const std::string& output_file, bool test_mode, const CamConfig& cam)
{
    PipelineCtx ctx;
    rs2::config cfg;

    // Apply the depth settings before streaming starts so the recording's header
    // carries the right depth scale. Options persist on the device across start().
    rs2::context rs_ctx;
    auto devices = rs_ctx.query_devices();
    if (devices.size() > 0) {
        rs2::device dev = devices[0];
        auto ds = dev.first<rs2::depth_sensor>();
        apply_depth_settings(ds, cam);
        cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    }

    if (test_mode) {
        cfg.enable_stream(RS2_STREAM_COLOR, 0, cam.width, cam.height, RS2_FORMAT_RGB8, cam.fps);
    } else {
        cfg.enable_stream(RS2_STREAM_DEPTH, 0, cam.width, cam.height, RS2_FORMAT_Z16,  cam.fps);
        cfg.enable_stream(RS2_STREAM_COLOR, 0, cam.width, cam.height, RS2_FORMAT_RGB8, cam.fps);
        cfg.enable_stream(RS2_STREAM_ACCEL,             RS2_FORMAT_MOTION_XYZ32F);
        cfg.enable_stream(RS2_STREAM_GYRO,              RS2_FORMAT_MOTION_XYZ32F);
        if (!output_file.empty())
            cfg.enable_record_to_file(output_file);
    }
    ctx.profile = ctx.pipe.start(cfg);//start the pipeline with the cfg configuration

    // Check the settings survived the stream start; re-apply (preset first) if not
    auto ds = ctx.profile.get_device().first<rs2::depth_sensor>();
    if (std::fabs(ds.get_depth_scale() - cam.depth_units) > 1e-9f) {
        std::cerr << "Depth units read back as " << ds.get_depth_scale()
                  << " after start, re-applying " << cam.depth_units << "\n";
        apply_depth_settings(ds, cam);
    }
    return ctx;
}


static PipelineCtx start_pipeline_retry(const std::string& output_file, bool test_mode,
                                         const CamConfig& cam, int max_attempts = 5)
{
    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        try {
            return start_pipeline(output_file, test_mode, cam);
        } catch (const rs2::error& e) {
            std::cerr << "start_pipeline attempt " << attempt << " failed: "
                      << e.what() << " — retrying...\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(500 * attempt));
        }
    }
    throw std::runtime_error("Failed to start RealSense pipeline after retries");
}

// RS2 colour frame → cloned BGR Mat (safe after frame goes out of scope)
static cv::Mat to_bgr(const rs2::video_frame& f)
{
    cv::Mat m(f.get_height(), f.get_width(), CV_8UC3,
              (void*)f.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat out;
    cv::cvtColor(m, out, cv::COLOR_RGB2BGR);
    return out;
}

// Colourised depth frame → cloned BGR Mat
static cv::Mat depth_to_mat(const rs2::frame& colorized)
{
    auto vf = colorized.as<rs2::video_frame>();
    return cv::Mat(vf.get_height(), vf.get_width(), CV_8UC3,
                   (void*)colorized.get_data(), cv::Mat::AUTO_STEP).clone();
}

static void show_frames(const cv::Mat& color, const cv::Mat& depth)
{
    // RGB window setup
    cv::namedWindow("RGB + markers", cv::WINDOW_NORMAL);
    cv::resizeWindow("RGB + markers", 1280, 720);
    cv::imshow("RGB + markers", color);

    // Depth window setup
    cv::namedWindow("Depth (filtered)", cv::WINDOW_NORMAL);
    cv::resizeWindow("Depth (filtered)", 1280, 720);
    cv::imshow("Depth (filtered)", depth);
}

static void print_stats(long long frames, double elapsed,
                        double depth_avg, rs2_vector accel, rs2_vector gyro, long holes)
{
    std::cout << "\r"
              << "frames: "   << std::setw(6) << frames
              << "  fps: "    << std::fixed << std::setprecision(1)
                              << std::setw(5) << frames / elapsed
              << "  depth: "  << std::setprecision(3) << std::setw(6) << depth_avg << "m"
            //   << "  accel: [" << std::setprecision(2)
            //                   << accel.x << "," << accel.y << "," << accel.z << "]"
            //   << "  gyro: ["  << gyro.x  << "," << gyro.y  << "," << gyro.z  << "]"
            << " Holes: " << holes
              << "     " << std::flush;
}

// ── Detection result type ─────────────────────────────────────────────────────
struct Detection {
    int                      id;       // -1 when the square was found but bits didn't decode
    std::vector<cv::Point2f> corners;
    cv::Point2f              centre;
    bool                     decoded;
};

// ── CSV logging ───────────────────────────────────────────────────────────────
// Derives csv path from bag name: "capture.bag" → "capture_detections.csv"
// Falls back to "detections.csv" when not recording.
static std::string csv_path(const std::string& bag)
{
    if (bag.empty()) return "detections.csv";
    auto dot = bag.rfind('.');
    return (dot == std::string::npos ? bag : bag.substr(0, dot)) + "_detections.csv";
}

static std::ofstream open_csv(const std::string& path)
{
    std::ofstream f(path);
    f << "timestamp_ms,frame,marker_id,cx,cy,depth_m,"
      << "c0x,c0y,c1x,c1y,c2x,c2y,c3x,c3y,decoded\n";
    return f;
}

// Derives ply path from bag name: "capture.bag" → "capture.ply".
// Falls back to "best_frame.ply" when not recording.
static std::string ply_path(const std::string& bag, int angle, int side)
{
    if (bag.empty()) return "best_frame.ply";
    auto dot = bag.rfind('.');
    return (dot == std::string::npos ? bag : bag.substr(0, dot))+ ".ply";
}

// Directory portion of a path, or "." if there is none.
static std::string dir_of(const std::string& path)
{
    auto slash = path.rfind('/');
    return slash == std::string::npos ? "." : path.substr(0, slash);
}

// Writes the IMU sample captured alongside the selected best frame.
static void write_imu_csv(const std::string& path, long long frame, double timestamp_ms,
                          rs2_vector accel, rs2_vector gyro)
{
    std::ofstream f(path);
    f << "frame,timestamp_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z\n";
    f << std::fixed << std::setprecision(6)
      << frame << "," << timestamp_ms << ","
      << accel.x << "," << accel.y << "," << accel.z << ","
      << gyro.x  << "," << gyro.y  << "," << gyro.z  << "\n";
}

// "A:B", "A:" or ":B" → inclusive frame range; hi = -1 means "to the end".
static bool parse_range(const std::string& s, long long& lo, long long& hi)
{
    const auto colon = s.find(':');
    if (colon == std::string::npos) return false;
    try {
        const std::string a = s.substr(0, colon), b = s.substr(colon + 1);
        lo = a.empty() ? 0  : std::stoll(a);
        hi = b.empty() ? -1 : std::stoll(b);
    } catch (const std::exception&) { return false; }
    return lo >= 0 && (hi < 0 || hi >= lo);
}

// depth_arg may be an empty frame (test mode) — depth_m written as 0.0 in that case.
static void log_detections(std::ofstream& f, long long frame, double timestamp_ms,
                            const std::vector<Detection>& detections,
                            const rs2::frame& depth_arg)
{
    rs2::depth_frame depth = depth_arg.as<rs2::depth_frame>();
    for (const auto& d : detections) {
        float depth_m = 0.0f;
        if (depth) {
            depth_m = depth.get_distance(
                std::max(0, std::min((int)d.centre.x, depth.get_width()  - 1)),
                std::max(0, std::min((int)d.centre.y, depth.get_height() - 1)));
        }
        f << std::fixed << std::setprecision(3)
          << timestamp_ms    << ","
          << frame           << ","
          << d.id            << ","
          << d.centre.x      << "," << d.centre.y << ","
          << depth_m         << ",";
        for (int i = 0; i < 4; ++i)
            f << d.corners[i].x << "," << d.corners[i].y << ",";
        f << (d.decoded ? 1 : 0) << "\n";
    }
    f.flush();
}

// ═════════════════════════════════════════════════════════════════════════════
//  MARKER DETECTION  — edit this section to experiment with AprilTag logic
// ═════════════════════════════════════════════════════════════════════════════

static const std::vector<int> TARGET_IDS = {0, 1, 2, 3, 4, 5, 6, 7, 12, 13, 14, 15, 16, 93, 94, 95};

// ── Undecoded-square filter (detect_all_markers) — tune these first ──────────
// OpenCV's rejected list also holds sand texture, shells, burrows etc.; a
// rejected square is only kept as an undecoded tag if it passes all of these.
constexpr float  CAND_MAX_SIDE_RATIO   = 2.0f;   // longest/shortest side — rejects slivers
constexpr float  CAND_MIN_SIDE_PX      = 15.0f;  // absolute side limits, used when no tag decoded
constexpr float  CAND_MAX_SIDE_PX      = 400.0f;
constexpr float  CAND_MEDIAN_SIDE_TOL  = 0.5f;   // else: +/- this fraction of median decoded side
constexpr int    CAND_CELL_PX          = 8;      // warped cell size for the border contrast check
constexpr double CAND_MIN_CONTRAST     = 8.0;    // white margin - black border (grey levels)
constexpr double CAND_MAX_BORDER_RATIO = 0.75;   // black border must be < this * white margin
constexpr int    CAND_MIN_SIDES_PASS   = 3;      // of 4 — one side may fail (shadow across it)
constexpr int    CAND_MIN_DATA_CELLS   = 4;      // need >= this many white AND black data cells
                                                 // (rejects solid dark blobs/shells; 0 = off)
constexpr float  CAND_DEDUP_FRAC       = 0.5f;   // duplicate if centres closer than this * side

// ── Second pass on an illumination-flattened image (when < expected tags) ────
constexpr double FLATTEN_SIGMA_PX      = 30.0;   // background blur: >> tag cell size, < shadow scale
constexpr bool   FLATTEN_USE_CLAHE     = false;  // true: CLAHE instead of divide-by-blur
constexpr double FLATTEN_CLAHE_CLIP    = 2.0;

// Returns detections for TARGET_IDS only, ignoring any other tags in frame.
static std::vector<Detection> detect_markers(const cv::Mat& gray)
{
    //static auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11));

    static cv::aruco::DetectorParameters params = []() {
        cv::aruco::DetectorParameters p;

        p.adaptiveThreshWinSizeMin = 3; //change parameters of the aruco decector for better sunlight conditions
        p.adaptiveThreshWinSizeMax = 53;
        p.adaptiveThreshWinSizeStep = 4;

        p.adaptiveThreshConstant = 7;

        return p;
    }();

    std::vector<int>                        all_ids;
    std::vector<std::vector<cv::Point2f>>   all_corners;
    cv::aruco::detectMarkers(gray, dictionary, all_corners, all_ids, cv::makePtr<cv::aruco::DetectorParameters>(params));

    std::vector<Detection> results;
    for (size_t i = 0; i < all_ids.size(); ++i) {
        if (std::find(TARGET_IDS.begin(), TARGET_IDS.end(), all_ids[i])
                == TARGET_IDS.end()) continue;

        cv::Point2f centre(0, 0);
        for (const auto& pt : all_corners[i]) centre += pt;
        centre *= 0.25f;

        results.push_back({all_ids[i], all_corners[i], centre, true});
    }
    return results;
}

// Centre = intersection of the quad's diagonals (perspective-correct, unlike the
// corner average). Falls back to the corner average if the diagonals are near-parallel.
static cv::Point2f quad_centre(const std::vector<cv::Point2f>& c)
{
    const cv::Point2f d1 = c[2] - c[0], d2 = c[3] - c[1];
    const float cross = d1.x * d2.y - d1.y * d2.x;
    if (std::fabs(cross) > 1e-2f * std::sqrt(d1.dot(d1) * d2.dot(d2))) {
        const cv::Point2f r = c[1] - c[0];
        return c[0] + ((r.x * d2.y - r.y * d2.x) / cross) * d1;
    }
    return (c[0] + c[1] + c[2] + c[3]) * 0.25f;
}

static float mean_side(const std::vector<cv::Point2f>& c)
{
    float sum = 0.0f;
    for (int i = 0; i < 4; ++i) sum += (float)cv::norm(c[(i + 1) % 4] - c[i]);
    return sum * 0.25f;
}

// Index of a result whose centre lies within CAND_DEDUP_FRAC * side of `centre`, else -1.
static int find_duplicate(const std::vector<Detection>& results, cv::Point2f centre, float side)
{
    for (size_t i = 0; i < results.size(); ++i)
        if (cv::norm(results[i].centre - centre) < CAND_DEDUP_FRAC * side) return (int)i;
    return -1;
}

// Warps the quad into a 10x10-cell image with the quad on cells 1..9. 36h11 layout is
// white margin (cell 0) | black border (1) | 6 data (2-7) | black border (8) | white margin (9).
// Border: per side, compares the margin band with the border band (central half of the
// cell thickness, corner cells skipped); needs CAND_MIN_SIDES_PASS of 4 sides to pass.
// Data: a real tag has both white and black data cells; a solid dark blob does not.
static bool border_contrast_ok(const cv::Mat& gray, const std::vector<cv::Point2f>& quad)
{
    const int   C  = CAND_CELL_PX;
    const float lo = 1.0f * C, hi = 9.0f * C;
    const std::vector<cv::Point2f> dst = {{lo, lo}, {hi, lo}, {hi, hi}, {lo, hi}};
    cv::Mat warped;
    cv::warpPerspective(gray, warped, cv::getPerspectiveTransform(quad, dst),
                        cv::Size(10 * C, 10 * C), cv::INTER_LINEAR, cv::BORDER_REPLICATE);

    auto band_mean = [&](bool horizontal, int cell) {
        const cv::Rect r = horizontal ? cv::Rect(2 * C, cell * C + C / 4, 6 * C, C / 2)
                                      : cv::Rect(cell * C + C / 4, 2 * C, C / 2, 6 * C);
        return cv::mean(warped(r))[0];
    };

    // {horizontal, margin cell, border cell} for top, bottom, left, right
    const int sides[4][3] = {{1, 0, 1}, {1, 9, 8}, {0, 0, 1}, {0, 9, 8}};
    int    passed = 0;
    double outside_sum = 0.0, border_sum = 0.0;
    for (const auto& s : sides) {
        const double outside = band_mean(s[0], s[1]);
        const double border  = band_mean(s[0], s[2]);
        outside_sum += outside;
        border_sum  += border;
        if (outside - border > CAND_MIN_CONTRAST && border < CAND_MAX_BORDER_RATIO * outside)
            ++passed;
    }
    if (passed < CAND_MIN_SIDES_PASS) return false;

    // Data cells 2..7, central half of each, classified against the margin/border midpoint
    const double mid = 0.125 * (outside_sum + border_sum);
    int n_white = 0, n_black = 0;
    for (int r = 2; r < 8; ++r)
        for (int c = 2; c < 8; ++c) {
            const double v = cv::mean(warped(cv::Rect(c * C + C / 4, r * C + C / 4, C / 2, C / 2)))[0];
            (v > mid ? n_white : n_black)++;
        }
    return n_white >= CAND_MIN_DATA_CELLS && n_black >= CAND_MIN_DATA_CELLS;
}

// Shape + size + border-contrast filter for a rejected candidate.
static bool candidate_ok(const cv::Mat& gray, const std::vector<cv::Point2f>& quad,
                         float min_side, float max_side)
{
    if (!cv::isContourConvex(quad)) return false;

    float shortest = (float)cv::norm(quad[1] - quad[0]), longest = shortest;
    for (int i = 1; i < 4; ++i) {
        const float s = (float)cv::norm(quad[(i + 1) % 4] - quad[i]);
        shortest = std::min(shortest, s);
        longest  = std::max(longest,  s);
    }
    if (longest > CAND_MAX_SIDE_RATIO * shortest)      return false;
    if (shortest < min_side || longest > max_side)     return false;
    return border_contrast_ok(gray, quad);
}

// Evens out sun/shadow: gray / heavy blur of itself, scaled to 128. The blur runs at
// quarter resolution (then upsampled) since a sigma-30 kernel at full res is slow.
static cv::Mat flatten_illumination(const cv::Mat& gray)
{
    cv::Mat out;
    if (FLATTEN_USE_CLAHE) {
        static cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(FLATTEN_CLAHE_CLIP, cv::Size(8, 8));
        clahe->apply(gray, out);
        return out;
    }
    cv::Mat f, small, bg;
    gray.convertTo(f, CV_32F);
    cv::resize(f, small, cv::Size(), 0.25, 0.25, cv::INTER_AREA);
    cv::GaussianBlur(small, small, cv::Size(0, 0), FLATTEN_SIGMA_PX * 0.25);
    cv::resize(small, bg, f.size(), 0, 0, cv::INTER_LINEAR);
    cv::divide(f, bg + 1.0, f, 128.0);   // +1 avoids divide-by-zero on black pixels
    f.convertTo(out, CV_8U);
    return out;
}

// Draws detections onto frame and logs new IDs to stdout.
// Decoded tags: green with ID. Undecoded squares (id -1): orange.
static void draw_detections(cv::Mat& frame, const std::vector<Detection>& detections)
{
    for (const auto& d : detections) {
        if (!d.decoded) {
            const cv::Scalar orange(0, 165, 255);
            std::vector<cv::Point> pts(d.corners.begin(), d.corners.end());
            cv::polylines(frame, pts, true, orange, 2);
            cv::circle(frame, d.centre, 4, orange, -1);
            cv::putText(frame,
                        "? (" + std::to_string(int(d.centre.x))
                        + "," + std::to_string(int(d.centre.y)) + ")",
                        d.centre + cv::Point2f(8, -8),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, orange, 1);
            continue;
        }
        std::vector<std::vector<cv::Point2f>> c = {d.corners};
        std::vector<int>                      id = {d.id};
        cv::aruco::drawDetectedMarkers(frame, c, id);
        cv::circle(frame, d.centre, 4, cv::Scalar(0, 255, 0), -1);
        cv::putText(frame,
                    "ID " + std::to_string(d.id)
                    + " (" + std::to_string(int(d.centre.x))
                    + "," + std::to_string(int(d.centre.y)) + ")",
                    d.centre + cv::Point2f(8, -8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }
}

// Detects every 36h11 tag visible — no ID filter. Used by -id and normal mode.
// Also keeps squares whose bits failed to decode (sun/shadow) as id -1, decoded = false,
// if they pass candidate_ok(). If fewer than expected_tags are found (0 = never), reruns
// on an illumination-flattened image and merges. Rejected candidates that fail the
// filter are appended to *filtered_out when given (debug overlay).
static std::vector<Detection> detect_all_markers(const cv::Mat& gray, int expected_tags = 0,
                                                 std::vector<std::vector<cv::Point2f>>* filtered_out = nullptr)
{
    static const cv::aruco::ArucoDetector detector = []() {
        cv::aruco::DetectorParameters p;

        // Adaptive threshold windows 5,13,...,45: small ones keep small/far tags,
        // large ones bridge strong sun/shadow gradients across a tag.
        p.adaptiveThreshWinSizeMin  = 5;
        p.adaptiveThreshWinSizeMax  = 45;
        p.adaptiveThreshWinSizeStep = 8;
        p.adaptiveThreshConstant    = 5;     // lower offset keeps washed-out (grey) black borders
        p.polygonalApproxAccuracyRate = 0.05; // looser polygon fit for blurred / slightly bent edges
        p.perspectiveRemovePixelPerCell         = 8;   // more samples per bit when decoding
        p.perspectiveRemoveIgnoredMarginPerCell = 0.3; // sample cell centres only, ignore edge bleed
        p.minOtsuStdDev = 3.0;               // still try Otsu on low-contrast tags
        p.maxErroneousBitsInBorderRate = 0.5; // tolerate border bits flipped by a shadow edge
        p.errorCorrectionRate = 1.0;          // use the full 36h11 error-correction capacity
        p.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX; // sub-pixel corners → steadier centres
        // Default 0.125 groups the white-margin outline with the black-border square
        // (1 cell apart) and only one reaches `rejected` — usually the margin, which
        // the border check then fails. Keep them separate so the true border is tested.
        p.minMarkerDistanceRate = 0.02;

        return cv::aruco::ArucoDetector(
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), p);
    }();

    std::vector<Detection> results;
    std::vector<float>     decoded_sides;

    auto run_pass = [&](const cv::Mat& img) {
        std::vector<int>                      ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;
        detector.detectMarkers(img, corners, ids, rejected);

        for (size_t i = 0; i < ids.size(); ++i) {
            const float side = mean_side(corners[i]);
            const Detection d{ids[i], corners[i], quad_centre(corners[i]), true};
            const int dup = find_duplicate(results, d.centre, side);
            if (dup < 0)                   results.push_back(d);
            else if (!results[dup].decoded) results[dup] = d;  // now decodes: upgrade it
            else                           continue;
            decoded_sides.push_back(side);
        }

        // Side limits: absolute, or +/- CAND_MEDIAN_SIDE_TOL of the median decoded side
        float min_side = CAND_MIN_SIDE_PX, max_side = CAND_MAX_SIDE_PX;
        if (!decoded_sides.empty()) {
            std::vector<float> s = decoded_sides;
            std::nth_element(s.begin(), s.begin() + s.size() / 2, s.end());
            const float median = s[s.size() / 2];
            min_side = (1.0f - CAND_MEDIAN_SIDE_TOL) * median;
            max_side = (1.0f + CAND_MEDIAN_SIDE_TOL) * median;
        }

        for (const auto& q : rejected) {
            if (q.size() != 4) { if (filtered_out) filtered_out->push_back(q); continue; }
            const cv::Point2f centre = quad_centre(q);
            if (find_duplicate(results, centre, mean_side(q)) >= 0) continue;
            if (!candidate_ok(img, q, min_side, max_side)) {
                if (filtered_out) filtered_out->push_back(q);
                continue;
            }
            results.push_back({-1, q, centre, false});
        }
    };

    run_pass(gray);
    if (expected_tags > 0 && (int)results.size() < expected_tags)
        run_pass(flatten_illumination(gray));

    return results;
}

// Debug overlay: rejected candidates that failed the filter — thin red, centre dot.
static void draw_rejected(cv::Mat& frame, const std::vector<std::vector<cv::Point2f>>& quads)
{
    const cv::Scalar red(0, 0, 255);
    for (const auto& q : quads) {
        std::vector<cv::Point> pts(q.begin(), q.end());
        cv::polylines(frame, pts, true, red, 1);
        if (q.size() == 4) cv::circle(frame, quad_centre(q), 2, red, -1);
    }
}

// Debug overlay: per-frame counts of decoded / undecoded / filtered-out.
static void draw_detection_counts(cv::Mat& frame, const std::vector<Detection>& detections,
                                  size_t n_filtered)
{
    size_t n_decoded = 0;
    for (const auto& d : detections) n_decoded += d.decoded;
    cv::putText(frame,
                "decoded " + std::to_string(n_decoded)
                + "  undecoded " + std::to_string(detections.size() - n_decoded)
                + "  filtered " + std::to_string(n_filtered),
                cv::Point(10, frame.rows - 12),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
}

// ═════════════════════════════════════════════════════════════════════════════

// ── Session directory helpers ─────────────────────────────────────────────────

static std::string timestamp_str()
{
    std::time_t t  = std::time(nullptr);
    std::tm     tm = *std::localtime(&t);
    char buf[20];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
    return buf;
}

// Creates <parent>/<YYYYMMDD_HHMMSS>/ and returns the session directory path.
// Creates <parent> first if it doesn't exist.
static std::string make_session_dir(const std::string& parent, const std::string& name)
{
    mkdir(parent.c_str(), 0755);  // no-op if already exists
    std::string dir = parent + "/" + name;
    if (mkdir(dir.c_str(), 0755) != 0) {
        std::cerr << "Failed to create session directory: " << dir
                   << " (" << std::strerror(errno) << ")\n";
        std::exit(1);
    }
    return dir;
}


// ─────────────────────────────────────────────────────────────────────────────

// "key: value" for every option a sensor / filter exposes, as read back now.
static void dump_options(std::ostream& f, const std::string& prefix, rs2::options o)
{
    for (rs2_option opt : o.get_supported_options()) {
        try {
            const float v = o.get_option(opt);
            f << prefix << rs2_option_to_string(opt) << ": " << v;
            if (const char* desc = o.get_option_value_description(opt, v))
                f << "  (" << desc << ")";
            f << "\n";
        } catch (const rs2::error&) {}   // some options are unreadable in some states
    }
}

// Full configuration of one scan, for post analysis. Read from the live device,
// so it must be called before pipe.stop(). `results` holds the per-scan outcome
// (mode, angle, best frame, files...) already formatted as "key: value" lines.
static void write_scan_info(const std::string& path, const std::string& cmdline,
                            const CamConfig& cam, const PipelineCtx& ctx,
                            DepthFilters& filters, int expected_tags,
                            const std::string& results)
{
    std::ofstream f(path);
    if (!f.is_open()) { std::cerr << "Failed to open " << path << "\n"; return; }
    f << std::setprecision(9);

    f << "# rs_marker scan configuration\n"
      << "written: " << timestamp_str() << "\n"
      << "command: " << cmdline << "\n"
      << results;

    f << "\n# requested\n"
      << "resolution: " << cam.width << "x" << cam.height << "@" << cam.fps << "\n"
      << "preset: "      << cam.preset << "\n"
      << "depth_units_m: " << cam.depth_units << "\n";

    rs2::device dev = ctx.profile.get_device();
    f << "\n# device\n";
    for (int i = 0; i < RS2_CAMERA_INFO_COUNT; ++i) {
        const auto ci = rs2_camera_info(i);
        if (dev.supports(ci))
            f << "device." << rs2_camera_info_to_string(ci) << ": " << dev.get_info(ci) << "\n";
    }

    f << "\n# streams (actual)\n";
    for (auto& sp : ctx.profile.get_streams()) {
        const std::string name = sp.stream_name();
        f << "stream." << name << ": " << rs2_format_to_string(sp.format()) << " @" << sp.fps() << "fps";
        if (auto vp = sp.as<rs2::video_stream_profile>()) {
            const rs2_intrinsics in = vp.get_intrinsics();
            f << " " << in.width << "x" << in.height << "\n"
              << "stream." << name << ".intrinsics: fx=" << in.fx << " fy=" << in.fy
              << " ppx=" << in.ppx << " ppy=" << in.ppy
              << " model=" << rs2_distortion_to_string(in.model) << " coeffs="
              << in.coeffs[0] << "," << in.coeffs[1] << "," << in.coeffs[2] << ","
              << in.coeffs[3] << "," << in.coeffs[4] << "\n";
        } else {
            f << "\n";
        }
    }

    // Read back from the device: what was actually in effect for this scan
    f << "\n# sensor options (actual)\n";
    auto ds = dev.first<rs2::depth_sensor>();
    f << "depth_scale_m: " << ds.get_depth_scale() << "\n";
    for (auto& s : dev.query_sensors()) {
        const std::string sname = s.supports(RS2_CAMERA_INFO_NAME) ? s.get_info(RS2_CAMERA_INFO_NAME) : "sensor";
        dump_options(f, sname + ".", s);
    }

    f << "\n# depth post-processing (depth aligned to colour before filtering)\n"
      << "filter_chain: threshold > to_disparity > spatial > temporal > to_depth > hole_fill  (decimation off)\n";
    dump_options(f, "filter.threshold.", filters.threshold);
    dump_options(f, "filter.spatial.",   filters.spatial);
    dump_options(f, "filter.temporal.",  filters.temporal);
    dump_options(f, "filter.hole_fill.", filters.hole_fill);

    f << "\n# tag detection\n"
      << "expected_tags: " << expected_tags << "\n"
      << "CAND_MAX_SIDE_RATIO: "   << CAND_MAX_SIDE_RATIO   << "\nCAND_MIN_SIDE_PX: " << CAND_MIN_SIDE_PX
      << "\nCAND_MAX_SIDE_PX: "    << CAND_MAX_SIDE_PX      << "\nCAND_MEDIAN_SIDE_TOL: " << CAND_MEDIAN_SIDE_TOL
      << "\nCAND_CELL_PX: "        << CAND_CELL_PX          << "\nCAND_MIN_CONTRAST: " << CAND_MIN_CONTRAST
      << "\nCAND_MAX_BORDER_RATIO: " << CAND_MAX_BORDER_RATIO << "\nCAND_MIN_SIDES_PASS: " << CAND_MIN_SIDES_PASS
      << "\nCAND_MIN_DATA_CELLS: " << CAND_MIN_DATA_CELLS   << "\nCAND_DEDUP_FRAC: " << CAND_DEDUP_FRAC
      << "\nFLATTEN_SIGMA_PX: "    << FLATTEN_SIGMA_PX      << "\nFLATTEN_USE_CLAHE: " << FLATTEN_USE_CLAHE
      << "\nFLATTEN_CLAHE_CLIP: "  << FLATTEN_CLAHE_CLIP    << "\n";
}

// ─────────────────────────────────────────────────────────────────────────────

// ── Test mode helpers ─────────────────────────────────────────────────────────

// Derive video output path from the -o argument (if given), else use default.
static std::string video_path(const std::string& arg)
{
    if (!arg.empty()) return arg;
    return "detections_test.mp4";
}

static cv::VideoWriter open_video(const std::string& path, int w, int h, double fps)
{
    cv::VideoWriter writer(path,
                           cv::VideoWriter::fourcc('m','p','4','v'),
                           fps, cv::Size(w, h));
    return writer;
}

// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[])
{
    std::string output_file;
    std::string output_dir;
    int         duration_sec  = 0;
    long long   frame_limit   = 0;   // 0 = no limit
    bool        test_mode     = false;
    bool        id_mode       = false;
    int         expected_tags = 4;       // -nTags: triggers the flattened second pass
    bool        debug_tags    = false;   // -debugTags: draw filtered-out candidates
    CamConfig   cam;                     // -r, --fps, -u, -p
    long long   best_lo       = 0;       // -b: best-frame search range (inclusive)
    long long   best_hi       = -1;      //     -1 = up to the last frame

    std::unique_ptr<mountSerial> serial;
    char read_buf[30];
    int angle = 0;
    int angle_limit = 0;
    int step = 10;
    int side = 1;
    bool flipped = false;
    bool Enable_Tilt = true;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "-o" || arg == "--output") && i + 1 < argc)
            output_file = argv[++i];
        else if ((arg == "-D" || arg == "--dir") && i + 1 < argc)
            output_dir = argv[++i];
        else if ((arg == "-d" || arg == "--duration") && i + 1 < argc)
            duration_sec = std::stoi(argv[++i]);
        else if ((arg == "-n" || arg == "--frames") && i + 1 < argc)
            frame_limit = std::stoll(argv[++i]);
        else if ((arg == "-a" || arg == "--angle") && i + 1 < argc)
            angle_limit = std::stoi(argv[++i]);
        else if ((arg == "-s" || arg == "--step") && i + 1 < argc)
            step = std::stoi(argv[++i]);
        else if (arg == "-test" || arg == "--test")
            test_mode = true;
        else if (arg == "-id" || arg == "--id")
            id_mode = true;
        else if (arg == "-noTilt" || arg == "--noTilt")
            Enable_Tilt = false;
        else if ((arg == "-nTags" || arg == "--nTags") && i + 1 < argc)
            expected_tags = std::stoi(argv[++i]);
        else if (arg == "-debugTags" || arg == "--debugTags")
            debug_tags = true;
        else if ((arg == "-r" || arg == "--res") && i + 1 < argc) {
            if (std::sscanf(argv[++i], "%dx%d", &cam.width, &cam.height) != 2) {
                std::cerr << "Bad resolution '" << argv[i] << "', expected WxH (e.g. 848x480)\n"; return 1;
            }
        }
        else if (arg == "--fps" && i + 1 < argc)
            cam.fps = std::stoi(argv[++i]);
        else if ((arg == "-u" || arg == "--units") && i + 1 < argc)
            cam.depth_units = std::stof(argv[++i]);
        else if ((arg == "-p" || arg == "--preset") && i + 1 < argc) {
            cam.preset = argv[++i];
            if (preset_value(cam.preset) < 0) {
                std::cerr << "Unknown preset '" << cam.preset << "'\n"; print_usage(argv[0]); return 1;
            }
        }
        else if ((arg == "-b" || arg == "--best-range") && i + 1 < argc) {
            if (!parse_range(argv[++i], best_lo, best_hi)) {
                std::cerr << "Bad best-frame range '" << argv[i] << "', expected START:END\n"; return 1;
            }
        }
        else if (arg == "-h" || arg == "--help") { print_usage(argv[0]); return 0; }
        else { std::cerr << "Unknown argument: " << arg << "\n"; print_usage(argv[0]); return 1; }
    }

    if (id_mode && test_mode) {
        std::cerr << "Error: -id and -test cannot be used together.\n"; return 1;
    }

    if (frame_limit > 0 && best_lo >= frame_limit) {
        std::cerr << "Error: best-frame range starts at frame " << best_lo
                  << " but only " << frame_limit << " frames are captured (-n).\n"; return 1;
    }

    std::string cmdline = argv[0];
    for (int i = 1; i < argc; ++i) cmdline += std::string(" ") + argv[i];

    if (angle_limit>60)
    {
        printf("Input angle too large, set to 60.\n");
        angle_limit = 60;
    }

    if (Enable_Tilt) {
        serial = std::make_unique<mountSerial>();
    }
    else angle_limit = 0;

    //declare the time of the session
    const std::string run_ts = timestamp_str();

    //make a folder for each session
    output_dir = output_dir + "/" + run_ts;
    const std::string session_root = output_dir;  // <session> — holds one folder per rotation

    //Set step size to 1 if entered value is less than 0
    if (step<=0) step = 1; 

    if (Enable_Tilt){
        //Return the mount to the home position
        serial->homeCamera(read_buf); 
        printf("Received message: %s\n", read_buf);
    }

    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);
    PipelineCtx ctx;

    // std::signal(SIGINT,  signal_handler);
    // std::signal(SIGTERM, signal_handler);
    // rs2::pipeline         pipe;     // created ONCE, reused every iteration via stop()/start()
    // rs2::pipeline_profile profile;


    int  rotation_index = 0;
    bool keep_scanning  = true;

    while (keep_scanning && g_running)
    {
        // Reset the tilt sweep state for this rotation set
        angle   = 0;
        side    = 1;
        flipped = false;

        std::string rotation_name = run_ts + "_r" + std::to_string(rotation_index);
        std::string rotation_dir;
        if (!output_dir.empty()) {
            rotation_dir = make_session_dir(session_root, rotation_name);
            std::cout << "Rotation: " << rotation_dir << "\n";
        }

        //start sweep loop here
        printf("Beginning loop now\n");
        printf("Number of iterations = %d\n",(2*(angle_limit/step)+1));
        for (int repetition = 0; repetition<(2*(angle_limit/step)+1);repetition++)
        {
            printf("Iteration %d\n",repetition);


            // if (!output_dir.empty() && !output_file.empty()) {
            //     std::cerr << "Error: -o and -D cannot be used together.\n"; return 1;
            // }
            if (!output_dir.empty()) {
                std::string session_name = rotation_name + "_a" + std::to_string(angle*side);
                std::string session_dir  = make_session_dir(rotation_dir, session_name);
                output_file = session_dir + "/" + session_name + ".db3";
                std::cout << "Session : " << session_dir << "\n";
            }

        
        // try {
        //     ctx = start_pipeline((test_mode || id_mode) ? "" : output_file,
        //                         test_mode || id_mode);
        // } catch (const rs2::error& e) {
        //     std::cerr << "RealSense error: " << e.what() << "\n"; return 1;
        // }
        ctx = start_pipeline_retry((test_mode || id_mode) ? "" : output_file, test_mode || id_mode, cam, 5);


        std::string   csv_file = id_mode ? "" : csv_path(test_mode ? "" : output_file);
        std::ofstream csv;
        if (!id_mode) {
            csv = open_csv(csv_file);
            if (!csv.is_open()) {
                std::cerr << "Failed to open CSV: " << csv_file << "\n"; return 1;
            }
        }

        // ── Best-frame outputs: only meaningful in normal mode (depth + IMU) ──────
        bool        capture_best = !test_mode && !id_mode;
        std::string ply_file     = capture_best ? ply_path(output_file,angle,side) : "";
        std::string imu_file     = capture_best
            ? (output_file.empty() ? "imu_a"+std::to_string(angle*side)+".csv" : dir_of(output_file) + "/imu_a"+std::to_string(angle*side)+".csv")
            : "";
        std::string info_file = output_file.empty() ? "scan_info.txt" : dir_of(output_file) + "/scan_info.txt";
        const std::string best_range_str = std::to_string(best_lo) + ":"
            + (best_hi < 0 ? std::string("end") : std::to_string(best_hi));

        // ── Test mode: open video writer ──────────────────────────────────────────
        cv::VideoWriter video_writer;
        std::string     vid_file;
        if (test_mode) {
            vid_file     = video_path(output_file);
            video_writer = open_video(vid_file, cam.width, cam.height, 5.0);
            if (!video_writer.isOpened()) {
                std::cerr << "Failed to open video writer: " << vid_file << "\n"; return 1;
            }
        }
        // ─────────────────────────────────────────────────────────────────────────
        //write the text dialog displaying which mode and the details therein.
        if (id_mode) {
            std::cout << "ID MODE — RGB-only, detect ALL AprilTag 36h11 tags\n"
                    << "Wave each tag in front of the camera to confirm it works.\n"
                    << "New unique IDs will be printed as they are detected.\n"
                    << "Stream  : Color " << cam.width << "x" << cam.height << "\n"
                    << (duration_sec > 0
                            ? "Duration: " + std::to_string(duration_sec) + "s\n"
                            : "Press q or Ctrl+C to stop.\n")
                    << std::string(60, '-') << "\n";
        } else if (test_mode) {
            std::cout << "TEST MODE — RGB-only, no depth/IMU\n"
                    << "Detecting AprilTag 36h11 — ALL tags\n"
                    << "Stream  : Color " << cam.width << "x" << cam.height << "\n"
                    << "Video   : " << vid_file  << "\n"
                    << "CSV     : " << csv_file  << "\n"
                    << (duration_sec > 0
                            ? "Duration: " + std::to_string(duration_sec) + "s\n"
                            : "Press q or Ctrl+C to stop.\n")
                    << std::string(60, '-') << "\n";
        } else {
            std::cout << "Detecting AprilTag 36h11 — ALL tags\n"
                    << "Streams : Depth + Color " << cam.width << "x" << cam.height << "@" << cam.fps
                    << " | Accel | Gyro\n"
                    << "Depth   : preset " << cam.preset << ", units " << cam.depth_units << " m\n"
                    << "Record  : " << (output_file.empty() ? "off" : output_file) << "\n"
                    << "CSV     : " << csv_file << "\n"
                    << "PLY     : " << ply_file << "  (best frame — least holes, frames " << best_range_str << ")\n"
                    << "IMU     : " << imu_file << "\n"
                    << "Config  : " << info_file << "\n"
                    << (duration_sec > 0
                            ? "Duration: " + std::to_string(duration_sec) + "s\n"
                            : "")
                    << (frame_limit > 0
                            ? "Frames  : " + std::to_string(frame_limit) + "\n"
                            : "")
                    << (duration_sec == 0 && frame_limit == 0
                            ? "Press q or Ctrl+C to stop.\n"
                            : "")
                    << std::string(60, '-') << "\n";
        }

        DepthFilters   filters;
        rs2::align     align_to_color(RS2_STREAM_COLOR);
        rs2::colorizer colorizer;
        rs2::pointcloud pc;

        auto       t_start     = std::chrono::steady_clock::now();
        long long  frame_count = 0;
        rs2_vector last_accel{}, last_gyro{};
        std::set<int> seen_ids;  // used by -id mode

        // ── Best-frame tracking (normal mode): most valid depth pixels = fewest holes ──
        rs2::depth_frame best_depth{rs2::frame{}};
        rs2::video_frame best_color{rs2::frame{}};
        rs2_vector       best_accel{}, best_gyro{};
        long long        best_frame_idx    = -1;
        double           best_ts_ms        = 0.0;
        int              best_valid_pixels = -1;
        int              best_total_pixels = 0;
        long long        frames_in_range   = 0;


        while (g_running) {
            if (duration_sec > 0) {
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::steady_clock::now() - t_start).count();
                if (elapsed >= duration_sec) break;
            }
            if (frame_limit > 0 && frame_count >= frame_limit) break;

            // rs2::frameset frames;
            // if (!ctx.pipe.poll_for_frames(&frames)) continue;
            rs2::frameset frames = ctx.pipe.wait_for_frames();

            if (id_mode) {
                // ── ID mode loop: detect all 36h11 tags, log unique IDs ───────────
                rs2::video_frame color_frame = frames.get_color_frame(); //get colour frame from realsense 
                if (!color_frame) continue;

                cv::Mat color_bgr = to_bgr(color_frame);
                cv::Mat gray; //convert to grey for detection
                cv::cvtColor(color_bgr, gray, cv::COLOR_BGR2GRAY);
                
                // cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8,8));
                // clahe->apply(gray,gray);

                std::vector<std::vector<cv::Point2f>> filtered_out;
                auto detections = detect_all_markers(gray, expected_tags,
                                                     debug_tags ? &filtered_out : nullptr); //detect apriltags

                for (const auto& d : detections) { //lis detection on console
                    if (d.decoded && seen_ids.insert(d.id).second) {
                        std::cout << "[NEW] ID: " << d.id
                                << "  (total unique: " << seen_ids.size() << ")\n"
                                << std::flush;
                    }
                }

                if (debug_tags) {
                    draw_rejected(color_bgr, filtered_out);
                    draw_detection_counts(color_bgr, detections, filtered_out.size());
                }
                draw_detections(color_bgr, detections); //draw each detection on colour frame

                // Overlay unique-ID list in top-left corner
                cv::putText(color_bgr,
                            "Unique IDs seen: " + std::to_string(seen_ids.size()),
                            cv::Point(10, 28),
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 220, 255), 2);
                int y = 58;
                for (int id : seen_ids) {
                    cv::putText(color_bgr, "  " + std::to_string(id),
                                cv::Point(10, y),
                                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 220, 255), 1);
                    y += 22;
                }

                cv::namedWindow("AprilTag ID scanner (36h11)", cv::WINDOW_NORMAL);
                cv::resizeWindow("AprilTag ID scanner (36h11)", 1280, 720);
                cv::imshow("AprilTag ID scanner (36h11)", color_bgr);
                if (cv::waitKey(1) == 'q') break;

            // ─────────────────────────────────────────────────────────────────
            } else if (test_mode) {
                // ── Test mode loop: RGB only ──────────────────────────────────────
                rs2::video_frame color_frame = frames.get_color_frame(); //get colour frame from realsense
                if (!color_frame) continue;

                cv::Mat color_bgr = to_bgr(color_frame); 
                cv::Mat gray; 
                cv::cvtColor(color_bgr, gray, cv::COLOR_BGR2GRAY);  //convert to gray for april tag detection

                auto detections = detect_markers(gray);
                draw_detections(color_bgr, detections); //detect tags and draw onto colour frame

                video_writer.write(color_bgr); //write the frame to the stored video file

                if (!detections.empty()) { //log each detection and its time?
                    double ts_ms = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - t_start).count();
                    log_detections(csv, frame_count, ts_ms, detections, rs2::frame{});
                }

                cv::imshow("RGB + markers (test)", color_bgr); //displa the frame
                if (cv::waitKey(1) == 'q') break;

            // ─────────────────────────────────────────────────────────────────
            } else {
                // ── Normal mode loop ──────────────────────────────────────────────
                if (auto a = frames.first_or_default(RS2_STREAM_ACCEL))
                    last_accel = a.as<rs2::motion_frame>().get_motion_data(); //get accelerometer date from frame
                if (auto g = frames.first_or_default(RS2_STREAM_GYRO)) 
                    last_gyro  = g.as<rs2::motion_frame>().get_motion_data(); //get gyro data from frame

                frames = align_to_color.process(frames); // align the depth pixels to colour?
                rs2::depth_frame raw_depth = frames.get_depth_frame(); //get depth frame
                if (!raw_depth) continue;

                rs2::depth_frame filtered   = filters.process(raw_depth);  //filter depth
                rs2::video_frame color_frame = frames.get_color_frame();  //get colour frame
                cv::Mat          color_bgr  = to_bgr(color_frame);
                cv::Mat          depth_bgr  = depth_to_mat(colorizer.colorize(filtered)); //convert both colour and depth to bgr format

                double ts_ms = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - t_start).count(); // elapsed time?

                // ── Marker detection ─────────────────────────────────────────────
                cv::Mat gray;
                cv::cvtColor(color_bgr, gray, cv::COLOR_BGR2GRAY); //convert to gray for detection

                //cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8,8));
                //clahe->apply(gray,gray); //Applied CLAHE after gray conversion to reduce dynamic range of image 

                std::vector<std::vector<cv::Point2f>> filtered_out;
                auto detections = detect_all_markers(gray, expected_tags,
                                                     debug_tags ? &filtered_out : nullptr); //detect tags and draw on colour frame
                if (debug_tags) {
                    draw_rejected(color_bgr, filtered_out);
                    draw_detection_counts(color_bgr, detections, filtered_out.size());
                }
                draw_detections(color_bgr, detections);
                draw_detections(gray,detections);

                if (!detections.empty())
                    log_detections(csv, frame_count, ts_ms, detections, filtered);
                // ─────────────────────────────────────────────────────────────────

                show_frames(color_bgr, depth_bgr); //show both colour and depth frames
                if (cv::waitKey(1) == 'q') break;

                // Depth stats
                double sum = 0.0; int valid = 0; int holes = 0;
                const int w = filtered.get_width(), h = filtered.get_height();
                for (int y = 0; y < h; ++y)
                    for (int x = 0; x < w; ++x) {
                        float d = filtered.get_distance(x, y);
                        if (d > 0.0f) { sum += d; ++valid; }
                        else {holes++;}
                    } //get depth stats for which is the best frame

                // Best-frame selection: most valid depth pixels == fewest holes,
                // only among frames inside the -b range
                const bool in_range = frame_count >= best_lo && (best_hi < 0 || frame_count <= best_hi);
                if (in_range) ++frames_in_range;
                if (in_range && valid > best_valid_pixels) {
                    best_valid_pixels = valid;
                    best_total_pixels = w * h;
                    best_depth        = filtered;
                    best_color        = color_frame;
                    best_accel        = last_accel;
                    best_gyro         = last_gyro;
                    best_frame_idx    = frame_count;
                    best_ts_ms        = ts_ms;
                }

                if (frame_count % 5 == 0) { //display stats for every 5th frame
                    double elapsed = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - t_start).count();
                    print_stats(frame_count, elapsed, valid > 0 ? sum / valid : 0.0,
                                last_accel, last_gyro, holes);
                }
                // ─────────────────────────────────────────────────────────────────
                
            }

            ++frame_count;
        } //end while loop

        // ── Export best frame (most valid depth pixels = fewest holes) ────────────
        if (capture_best && best_valid_pixels >= 0) {
            best_color.keep();
            best_depth.keep();

            pc.map_to(best_color);
            rs2::points points = pc.calculate(best_depth);
            points.export_to_ply(ply_file, best_color);
            write_imu_csv(imu_file, best_frame_idx, best_ts_ms, best_accel, best_gyro);
        }
        // ─────────────────────────────────────────────────────────────────────────

        // ── Full scan configuration (read from the device, so before stop) ──────
        {
            const double scan_s = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t_start).count();
            std::ostringstream r;
            r << "mode: " << (id_mode ? "id" : test_mode ? "test" : "normal") << "\n"
              << "session: "        << run_ts << "\n"
              << "rotation_deg: "   << rotation_index << "\n"
              << "tilt_enabled: "   << Enable_Tilt << "\n"
              << "tilt_angle_deg: " << angle * side << "\n"
              << "tilt_limit_deg: " << angle_limit << "\ntilt_step_deg: " << step << "\n"
              << "duration_limit_s: " << duration_sec << "\nframe_limit: " << frame_limit << "\n"
              << "frames_captured: " << frame_count << "\n"
              << "scan_duration_s: " << std::fixed << std::setprecision(3) << scan_s << "\n"
              << std::defaultfloat
              << "record: "         << ((test_mode || id_mode) ? "" : output_file) << "\n"
              << "csv: "            << csv_file << "\n";
            if (capture_best) {
                r << "best_range: "        << best_range_str << "\n"
                  << "best_frames_in_range: " << frames_in_range << "\n";
                if (best_valid_pixels >= 0)
                    r << "best_frame: "        << best_frame_idx << "\n"
                      << "best_timestamp_ms: " << best_ts_ms << "\n"
                      << "best_valid_pixels: " << best_valid_pixels << " / " << best_total_pixels << "\n"
                      << "ply: " << ply_file << "\nimu: " << imu_file << "\n";
                else
                    r << "best_frame: none (no depth frame inside the range)\n";
            }
            if (test_mode) r << "video: " << vid_file << "\n";
            write_scan_info(info_file, cmdline, cam, ctx, filters, expected_tags, r.str());
        }

        ctx.pipe.stop();
        if (test_mode) video_writer.release();

        //std::this_thread::sleep_for(std::chrono::milliseconds(200)); // let the device release before restart

        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t_start).count();
        std::cout << "\n" << std::string(60, '-') << "\n"
                << "Frames  : " << frame_count << "\n"
                << "Duration: " << std::fixed << std::setprecision(2) << elapsed << "s\n";

        if (id_mode) {
            std::cout << "Unique IDs detected (" << seen_ids.size() << "): ";
            for (int id : seen_ids) std::cout << id << " ";
            std::cout << "\n";
        } else {
            std::cout << "CSV     : " << csv_file << "\n";
            if (test_mode) {
                std::cout << "Video   : " << vid_file << "\n";
            } else {
                if (!output_file.empty())
                    std::cout << "Bag     : " << output_file << "\n";
                if (best_valid_pixels >= 0) {
                    std::cout << "PLY     : " << ply_file
                            << "  (frame " << best_frame_idx << ")\n"
                            << "IMU     : " << imu_file << "\n";
                } else {
                    std::cout << "PLY     : (skipped — no depth frame captured in frames "
                              << best_range_str << ")\n";
                }
                std::cout << "Config  : " << info_file << "\n";
            }
        }
        frame_count=0;
        
        if(Enable_Tilt){

            angle += step;
        
            if (repetition >= (angle_limit/step) && !flipped)
            {
                angle = step;
                side = -1;
                flipped = true;
            }
            serial->setCameraAngle(read_buf,angle*side);
            printf("Received message: %s\n", read_buf);
        }
        //std::chrono::milliseconds(1000);

        } //end sweep for loop

        // Angle sweep for this rotation is done — return to home before asking
        // the user to rotate the mount (rotation is changed manually).
        if (Enable_Tilt) {
            serial->homeCamera(read_buf);
            serial->homeCamera(read_buf);
            printf("Received message: %s\n", read_buf);
        }

        if (!g_running) break;

        std::cout << "\n" << std::string(60, '-') << "\n"
                  << "Completed rotation " << rotation_index << ".\n"
                  << "Rotate the mount to the next position, then press Enter to "
                     "continue scanning (or type q + Enter to finish): ";
        std::string user_line;
        std::getline(std::cin, user_line);
        if (!user_line.empty() && (user_line[0] == 'q' || user_line[0] == 'Q')) {
            keep_scanning = false;
        } else {
            rotation_index += 45;
        }
    } //end rotation while loop

    //Return the mount to the home position
    printf("Returning to home position");
    if(Enable_Tilt){
        serial->homeCamera(read_buf);
        serial->homeCamera(read_buf);
        printf("Received message: %s\n", read_buf);
    }
    return 0;
}
