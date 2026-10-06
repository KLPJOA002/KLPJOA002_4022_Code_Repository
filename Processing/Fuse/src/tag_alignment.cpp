#include "tag_alignment.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <unordered_map>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

namespace fs = std::filesystem;

namespace
{

// librealsense's rs2_project_point_to_pixel for the colour stream's model:
// radial scaling first, then the tangential terms on the scaled coordinates.
// Checked against an rs_marker PLY: every exported point reprojects to its
// integer source pixel to within 1e-5 px under this, and is visibly off
// (~0.25 px mean) without the distortion.
Eigen::Vector2f distortNormalised(const Eigen::Vector2f& n, const CameraIntrinsics& in)
{
    if (!in.distorted) return n;
    const auto& c = in.coeffs;
    const float r2 = n.squaredNorm();
    const float f = 1.0f + c[0] * r2 + c[1] * r2 * r2 + c[4] * r2 * r2 * r2;
    const float x = n.x() * f, y = n.y() * f;
    return { x + 2.0f * c[2] * x * y + c[3] * (r2 + 2.0f * x * x),
             y + 2.0f * c[3] * x * y + c[2] * (r2 + 2.0f * y * y) };
}

// Inverse of distortNormalised by fixed-point iteration. The colour lens's
// distortion is mild, so this converges to well under a thousandth of a
// pixel in a handful of steps.
Eigen::Vector2f undistortNormalised(const Eigen::Vector2f& d, const CameraIntrinsics& in)
{
    if (!in.distorted) return d;
    Eigen::Vector2f n = d;
    for (int i = 0; i < 30; ++i)
    {
        const Eigen::Vector2f err = distortNormalised(n, in) - d;
        n -= err;
        if (err.squaredNorm() < 1e-14f) break;
    }
    return n;
}

// Camera frame (z forward) <-> PLY frame (librealsense's OpenGL-style export).
inline Eigen::Vector3f plyToCamera(const Eigen::Vector3f& p) { return { p.x(), -p.y(), -p.z() }; }
inline Eigen::Vector3f cameraToPly(const Eigen::Vector3f& p) { return { p.x(), -p.y(), -p.z() }; }

bool projectToPixel(const Eigen::Vector3f& camera, const CameraIntrinsics& in, Eigen::Vector2f& pixel)
{
    if (camera.z() <= 1e-6f) return false;
    const Eigen::Vector2f d = distortNormalised(
        Eigen::Vector2f(camera.x() / camera.z(), camera.y() / camera.z()), in);
    pixel = { d.x() * in.fx + in.ppx, d.y() * in.fy + in.ppy };
    return true;
}

Eigen::Vector3f pixelRay(const Eigen::Vector2f& pixel, const CameraIntrinsics& in)
{
    const Eigen::Vector2f n = undistortNormalised(
        Eigen::Vector2f((pixel.x() - in.ppx) / in.fx, (pixel.y() - in.ppy) / in.fy), in);
    return { n.x(), n.y(), 1.0f };
}

// Convex quad containment; works for either winding.
bool insideQuad(const std::array<Eigen::Vector2f, 4>& q, const Eigen::Vector2f& p)
{
    int sign = 0;
    for (int i = 0; i < 4; ++i)
    {
        const Eigen::Vector2f a = q[i], b = q[(i + 1) % 4];
        const float cross = (b.x() - a.x()) * (p.y() - a.y()) - (b.y() - a.y()) * (p.x() - a.x());
        const int s = (cross > 0.0f) - (cross < 0.0f);
        if (s == 0) continue;
        if (sign == 0) sign = s;
        else if (s != sign) return false;
    }
    return true;
}

float median(std::vector<float>& v)
{
    const size_t mid = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + mid, v.end());
    return v[mid];
}

// Least-squares plane through `points`: returns normal and centroid, and the
// rms distance of the points from it.
void fitPlane(const std::vector<Eigen::Vector3f>& points,
              Eigen::Vector3f& normal, Eigen::Vector3f& centroid, double& rms)
{
    centroid.setZero();
    for (const auto& p : points) centroid += p;
    centroid /= static_cast<float>(points.size());

    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (const auto& p : points) { const Eigen::Vector3f d = p - centroid; cov += d * d.transpose(); }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    normal = solver.eigenvectors().col(0).normalized();

    double sum_sq = 0.0;
    for (const auto& p : points) { const double d = normal.dot(p - centroid); sum_sq += d * d; }
    rms = std::sqrt(sum_sq / static_cast<double>(points.size()));
}

struct PixelTag
{
    std::array<Eigen::Vector2f, 4> corners;
    int frames = 0;
};

// Per-id median corner pixels over every decoded row of the detections CSV:
//   timestamp_ms,frame,marker_id,cx,cy,depth_m,c0x,c0y,c1x,c1y,c2x,c2y,c3x,c3y,decoded
std::map<int, PixelTag> readDetections(const std::string& path)
{
    std::map<int, std::array<std::vector<float>, 8>> samples;

    std::ifstream file(path);
    std::string line;
    std::getline(file, line); // header
    while (std::getline(file, line))
    {
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream iss(line);
        double timestamp, frame, cx, cy, depth;
        int id, decoded;
        float c[8];
        if (!(iss >> timestamp >> frame >> id >> cx >> cy >> depth
                  >> c[0] >> c[1] >> c[2] >> c[3] >> c[4] >> c[5] >> c[6] >> c[7] >> decoded))
            continue;
        // Undecoded candidates have no id, so nothing to correspond them with.
        if (!decoded || id < 0) continue;

        auto& s = samples[id];
        for (int k = 0; k < 8; ++k) s[k].push_back(c[k]);
    }

    std::map<int, PixelTag> tags;
    for (auto& [id, s] : samples)
    {
        PixelTag tag;
        tag.frames = static_cast<int>(s[0].size());
        for (int k = 0; k < 4; ++k)
            tag.corners[k] = { median(s[2 * k]), median(s[2 * k + 1]) };
        tags[id] = tag;
    }
    return tags;
}

} // namespace

// -------------------------------------------------------------------------
bool findTagFiles(const std::string& ply_path, std::string& detections_path,
                  std::string& scan_info_path)
{
    const fs::path ply(ply_path);
    const fs::path folder = ply.parent_path().empty() ? fs::path(".") : ply.parent_path();

    const fs::path preferred = folder / (ply.stem().string() + "_detections.csv");
    if (fs::exists(preferred))
    {
        detections_path = preferred.string();
    }
    else
    {
        std::vector<fs::path> candidates;
        for (const auto& entry : fs::directory_iterator(folder))
        {
            const std::string name = entry.path().filename().string();
            const std::string suffix = "_detections.csv";
            if (entry.is_regular_file() && name.size() > suffix.size() &&
                name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0)
                candidates.push_back(entry.path());
        }
        if (candidates.size() != 1) return false;
        detections_path = candidates.front().string();
    }

    const fs::path info = folder / "scan_info.txt";
    if (!fs::exists(info)) return false;
    scan_info_path = info.string();
    return true;
}

// -------------------------------------------------------------------------
bool findTagSidecar(const std::string& ply_path, std::string& sidecar_path)
{
    const fs::path ply(ply_path);
    const fs::path folder = ply.parent_path().empty() ? fs::path(".") : ply.parent_path();

    const fs::path preferred = folder / (folder.filename().string() + "_tags.csv");
    if (fs::exists(preferred))
    {
        sidecar_path = preferred.string();
        return true;
    }

    std::vector<fs::path> candidates;
    for (const auto& entry : fs::directory_iterator(folder))
    {
        const std::string name = entry.path().filename().string();
        const std::string suffix = "_tags.csv";
        if (entry.is_regular_file() && name.size() > suffix.size() &&
            name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0)
            candidates.push_back(entry.path());
    }
    if (candidates.size() != 1) return false;
    sidecar_path = candidates.front().string();
    return true;
}

// -------------------------------------------------------------------------
bool readTagSidecar(const std::string& path, TagSet& out, std::vector<TagLocateReport>* reports)
{
    std::ifstream file(path);
    if (!file) return false;

    struct Partial { TagCorners corners; int seen = 0; TagLocateReport report; };
    std::map<int, Partial> partial;

    std::string line;
    std::getline(file, line); // header
    while (std::getline(file, line))
    {
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream iss(line);
        int id, corner, frames;
        double x, y, z, plane_rms, edge;
        if (!(iss >> id >> corner >> x >> y >> z >> frames >> plane_rms >> edge)) continue;
        if (corner < 0 || corner > 3) continue;

        auto& p = partial[id];
        p.corners[corner] = Eigen::Vector3f(static_cast<float>(x), static_cast<float>(y),
                                            static_cast<float>(z));
        p.seen |= 1 << corner;
        p.report.id = id;
        p.report.frames = frames;
        p.report.plane_rms = plane_rms;
        p.report.edge_length = edge;
    }

    out.clear();
    for (auto& [id, p] : partial)
    {
        p.report.located = (p.seen == 0xF);
        if (p.report.located) out[id] = p.corners;
        else p.report.reason = "sidecar is missing a corner";
        if (reports) reports->push_back(p.report);
    }
    return true;
}

// -------------------------------------------------------------------------
bool readColorIntrinsics(const std::string& scan_info_path, CameraIntrinsics& out)
{
    std::ifstream file(scan_info_path);
    if (!file) return false;

    bool have_intrinsics = false;
    std::string line;
    while (std::getline(file, line))
    {
        // "stream.Color: RGB8 @30fps 1280x720"
        if (line.rfind("stream.Color:", 0) == 0)
        {
            const auto x = line.rfind('x');
            const auto space = line.rfind(' ', x);
            if (x != std::string::npos && space != std::string::npos)
            {
                out.width  = std::atoi(line.substr(space + 1, x - space - 1).c_str());
                out.height = std::atoi(line.substr(x + 1).c_str());
            }
        }
        // "stream.Color.intrinsics: fx=.. fy=.. ppx=.. ppy=.. model=<name> coeffs=a,b,c,d,e"
        else if (line.rfind("stream.Color.intrinsics:", 0) == 0)
        {
            auto value = [&line](const std::string& key) -> std::string {
                const auto at = line.find(key + "=");
                if (at == std::string::npos) return "";
                const auto start = at + key.size() + 1;
                return line.substr(start, line.find(' ', start) - start);
            };
            try
            {
                out.fx  = std::stof(value("fx"));
                out.fy  = std::stof(value("fy"));
                out.ppx = std::stof(value("ppx"));
                out.ppy = std::stof(value("ppy"));
            }
            catch (const std::exception&) { return false; }

            std::string coeffs = value("coeffs");
            std::replace(coeffs.begin(), coeffs.end(), ',', ' ');
            std::istringstream iss(coeffs);
            for (auto& c : out.coeffs) if (!(iss >> c)) c = 0.0f;

            const bool no_model = line.find("model=None") != std::string::npos;
            out.distorted = !no_model && std::any_of(out.coeffs.begin(), out.coeffs.end(),
                                                      [](float c) { return c != 0.0f; });
            have_intrinsics = true;
        }
    }
    return have_intrinsics && out.fx > 0.0f && out.fy > 0.0f;
}

// -------------------------------------------------------------------------
TagSet locateTags(const PointCloudT& raw_cloud, const std::string& detections_path,
                  const CameraIntrinsics& intrinsics, std::vector<TagLocateReport>* reports)
{
    // Only the inner part of each tag is sampled for its plane. The border
    // pixels mix tag and background depth, and a tag mounted proud of the
    // surface has a step there that would tilt the fit.
    constexpr float kInnerScale     = 0.7f;
    constexpr int   kMinPlanePoints = 30;
    constexpr float kPlaneInlier    = 0.003f;  // metres; RealSense noise at ~0.6 m is ~1 mm

    const auto pixel_tags = readDetections(detections_path);

    struct Work
    {
        int id;
        std::array<Eigen::Vector2f, 4> inner;
        Eigen::Vector2f lo, hi;
        std::vector<Eigen::Vector3f> points;   // camera frame
    };
    std::vector<Work> work;
    for (const auto& [id, tag] : pixel_tags)
    {
        Work w;
        w.id = id;
        Eigen::Vector2f centre = Eigen::Vector2f::Zero();
        for (const auto& c : tag.corners) centre += c;
        centre /= 4.0f;
        w.lo = w.hi = centre;
        for (int k = 0; k < 4; ++k)
        {
            w.inner[k] = centre + kInnerScale * (tag.corners[k] - centre);
            w.lo = w.lo.cwiseMin(w.inner[k]);
            w.hi = w.hi.cwiseMax(w.inner[k]);
        }
        work.push_back(std::move(w));
    }

    // One pass over the cloud, projecting each point back to the pixel it
    // was deprojected from.
    for (const auto& p : raw_cloud)
    {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const Eigen::Vector3f camera = plyToCamera(p.getVector3fMap());
        Eigen::Vector2f pixel;
        if (!projectToPixel(camera, intrinsics, pixel)) continue;

        for (auto& w : work)
        {
            if (pixel.x() < w.lo.x() || pixel.y() < w.lo.y() ||
                pixel.x() > w.hi.x() || pixel.y() > w.hi.y()) continue;
            if (insideQuad(w.inner, pixel)) w.points.push_back(camera);
        }
    }

    TagSet tags;
    for (auto& w : work)
    {
        TagLocateReport report;
        report.id = w.id;
        report.frames = pixel_tags.at(w.id).frames;

        if (static_cast<int>(w.points.size()) < kMinPlanePoints)
        {
            report.reason = "only " + std::to_string(w.points.size()) + " depth points on the tag";
            if (reports) reports->push_back(report);
            continue;
        }

        // Fit, drop the points well off the plane (edge bleed, flying
        // pixels), and refit on what's left.
        Eigen::Vector3f normal, centroid;
        double rms;
        fitPlane(w.points, normal, centroid, rms);
        std::vector<Eigen::Vector3f> inliers;
        const float tolerance = std::max(kPlaneInlier, static_cast<float>(2.5 * rms));
        for (const auto& p : w.points)
            if (std::abs(normal.dot(p - centroid)) <= tolerance) inliers.push_back(p);
        if (static_cast<int>(inliers.size()) < kMinPlanePoints)
        {
            report.reason = "depth on the tag is not planar";
            if (reports) reports->push_back(report);
            continue;
        }
        fitPlane(inliers, normal, centroid, rms);

        // Each corner is where its pixel's ray meets the tag's plane: exact
        // in the image, and averaged over the whole tag in depth, rather than
        // trusting the single depth sample at a corner (where the depth is
        // least reliable, right on the black/white edge).
        TagCorners corners;
        bool ok = true;
        for (int k = 0; k < 4; ++k)
        {
            const Eigen::Vector3f ray = pixelRay(pixel_tags.at(w.id).corners[k], intrinsics);
            const float denom = normal.dot(ray);
            if (std::abs(denom) < 1e-4f) { ok = false; break; }
            corners[k] = cameraToPly((normal.dot(centroid) / denom) * ray);
        }
        if (!ok)
        {
            report.reason = "tag plane is edge-on to the camera";
            if (reports) reports->push_back(report);
            continue;
        }

        double edge = 0.0;
        for (int k = 0; k < 4; ++k) edge += (corners[(k + 1) % 4] - corners[k]).norm();

        report.located = true;
        report.plane_points = static_cast<int>(inliers.size());
        report.plane_rms = rms;
        report.edge_length = edge / 4.0;
        if (reports) reports->push_back(report);
        tags[w.id] = corners;
    }
    return tags;
}

// -------------------------------------------------------------------------
TagFit fitTags(const TagSet& source, const TagSet& target, int min_shared)
{
    TagFit fit;
    for (const auto& [id, corners] : source)
        if (target.count(id)) fit.ids.push_back(id);
    fit.shared_tags = static_cast<int>(fit.ids.size());
    if (fit.shared_tags < std::max(1, min_shared)) return fit;

    Eigen::Matrix3Xf src(3, 4 * fit.ids.size()), dst(3, 4 * fit.ids.size());
    for (size_t t = 0; t < fit.ids.size(); ++t)
    {
        const auto& s = source.at(fit.ids[t]);
        const auto& d = target.at(fit.ids[t]);
        for (int k = 0; k < 4; ++k)
        {
            src.col(4 * t + k) = s[k];
            dst.col(4 * t + k) = d[k];
        }
    }

    // Rigid only: both sides are metric, so a scale term could only absorb
    // error, never explain it.
    fit.transform = Eigen::umeyama(src, dst, false);

    double sum_sq = 0.0;
    for (int c = 0; c < src.cols(); ++c)
    {
        const Eigen::Vector3f moved = fit.transform.topLeftCorner<3, 3>() * src.col(c) +
                                      fit.transform.topRightCorner<3, 1>();
        sum_sq += (moved - dst.col(c)).squaredNorm();
    }
    fit.rms = std::sqrt(sum_sq / static_cast<double>(src.cols()));
    fit.valid = true;
    return fit;
}

// -------------------------------------------------------------------------
TagFit fitTagsAboutAxis(const TagSet& source, const TagSet& target, int min_shared,
                        const Eigen::Matrix3f& base_rotation, const Eigen::Vector3f& up)
{
    TagFit fit;
    for (const auto& [id, corners] : source)
        if (target.count(id)) fit.ids.push_back(id);
    fit.shared_tags = static_cast<int>(fit.ids.size());
    if (fit.shared_tags < std::max(1, min_shared)) return fit;

    const Eigen::Vector3f axis = up.normalized();
    std::vector<Eigen::Vector3f> src, dst;
    for (int id : fit.ids)
        for (int k = 0; k < 4; ++k)
        {
            src.push_back(base_rotation * source.at(id)[k]);
            dst.push_back(target.at(id)[k]);
        }

    Eigen::Vector3f src_mean = Eigen::Vector3f::Zero(), dst_mean = Eigen::Vector3f::Zero();
    for (size_t i = 0; i < src.size(); ++i) { src_mean += src[i]; dst_mean += dst[i]; }
    src_mean /= static_cast<float>(src.size());
    dst_mean /= static_cast<float>(dst.size());

    // Closed-form 2D Procrustes in the plane perpendicular to the axis.
    double sin_sum = 0.0, cos_sum = 0.0;
    for (size_t i = 0; i < src.size(); ++i)
    {
        Eigen::Vector3f a = src[i] - src_mean, b = dst[i] - dst_mean;
        a -= axis * axis.dot(a);
        b -= axis * axis.dot(b);
        sin_sum += axis.dot(a.cross(b));
        cos_sum += a.dot(b);
    }
    const float angle = static_cast<float>(std::atan2(sin_sum, cos_sum));

    const Eigen::Matrix3f R = Eigen::AngleAxisf(angle, axis).toRotationMatrix() * base_rotation;
    fit.transform = Eigen::Matrix4f::Identity();
    fit.transform.topLeftCorner<3, 3>() = R;
    // src_mean is already base-rotated, so only the turn remains to apply.
    fit.transform.topRightCorner<3, 1>() = dst_mean - Eigen::AngleAxisf(angle, axis) * src_mean;

    double sum_sq = 0.0;
    for (int id : fit.ids)
        for (int k = 0; k < 4; ++k)
        {
            const Eigen::Vector3f moved = R * source.at(id)[k] + fit.transform.topRightCorner<3, 1>();
            sum_sq += (moved - target.at(id)[k]).squaredNorm();
        }
    fit.rms = std::sqrt(sum_sq / static_cast<double>(4 * fit.ids.size()));
    fit.tilt_from_imu = true;
    fit.valid = true;
    return fit;
}
