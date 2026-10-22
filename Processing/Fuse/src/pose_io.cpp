#include "pose_io.h"

#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>

#ifndef FUSE_VERSION
#define FUSE_VERSION "unknown"
#endif

namespace
{

// Non-finite or "never measured" (the max() sentinels) values are written as
// null rather than as 1.8e308, which nothing downstream should have to know.
Json::Value number(double v)
{
    if (!std::isfinite(v) || std::abs(v) >= 1e300) return Json::Value(Json::nullValue);
    return Json::Value(v);
}

double numberOr(const Json::Value& v, double fallback)
{
    return v.isNumeric() ? v.asDouble() : fallback;
}

template <int R, int C>
Json::Value matrix(const Eigen::Matrix<float, R, C>& m)
{
    Json::Value rows(Json::arrayValue);
    for (int r = 0; r < R; ++r)
    {
        Json::Value row(Json::arrayValue);
        for (int c = 0; c < C; ++c) row.append(static_cast<double>(m(r, c)));
        rows.append(row);
    }
    return rows;
}

template <int R, int C>
bool readMatrix(const Json::Value& v, Eigen::Matrix<float, R, C>& out)
{
    if (!v.isArray() || static_cast<int>(v.size()) != R) return false;
    for (int r = 0; r < R; ++r)
    {
        const auto& row = v[r];
        if (!row.isArray() || static_cast<int>(row.size()) != C) return false;
        for (int c = 0; c < C; ++c) out(r, c) = static_cast<float>(row[c].asDouble());
    }
    return true;
}

Json::Value vec3(const Eigen::Vector3f& v)
{
    Json::Value a(Json::arrayValue);
    for (int k = 0; k < 3; ++k) a.append(static_cast<double>(v[k]));
    return a;
}

bool readVec3(const Json::Value& v, Eigen::Vector3f& out)
{
    if (!v.isArray() || v.size() != 3) return false;
    for (int k = 0; k < 3; ++k) out[k] = static_cast<float>(v[k].asDouble());
    return true;
}

Json::Value strings(const std::vector<std::string>& list)
{
    Json::Value a(Json::arrayValue);
    for (const auto& s : list) a.append(s);
    return a;
}

std::vector<std::string> readStrings(const Json::Value& v)
{
    std::vector<std::string> out;
    if (v.isArray()) for (const auto& s : v) out.push_back(s.asString());
    return out;
}

Json::Value quality(const AlignmentQuality& q)
{
    Json::Value o(Json::objectValue);
    if (!q.valid()) return Json::Value(Json::nullValue);
    o["rmse_mm"] = number(q.rmse * 1000.0);
    o["inlier_fraction"] = q.inlier_fraction;
    o["inliers"] = static_cast<Json::UInt64>(q.inliers);
    o["sampled"] = static_cast<Json::UInt64>(q.sampled);
    o["score"] = number(q.score());
    return o;
}

AlignmentQuality readQuality(const Json::Value& v)
{
    AlignmentQuality q;
    if (!v.isObject()) return q;
    q.rmse = numberOr(v["rmse_mm"], q.rmse * 1000.0) / 1000.0;
    q.inlier_fraction = numberOr(v["inlier_fraction"], 0.0);
    q.inliers = static_cast<size_t>(v["inliers"].asUInt64());
    q.sampled = static_cast<size_t>(v["sampled"].asUInt64());
    return q;
}

// "<session>/<session>_r45" -> "r45": the same reduction shortLabel applies
// to a tilt folder, so groups and scans are named alike in the file.
std::string shortGroupName(const RotationGroup& g)
{
    std::string name = g.name;
    const auto slash = name.rfind('/');
    if (slash != std::string::npos) name = name.substr(slash + 1);
    const std::string prefix = g.session_name + "_";
    if (!g.session_name.empty() && name.rfind(prefix, 0) == 0) name = name.substr(prefix.size());
    return name;
}

const char* axisName(RigAxis a)
{
    switch (a)
    {
        case RigAxis::X: return "x";
        case RigAxis::Y: return "y";
        case RigAxis::Z: return "z";
        default:         return "none";
    }
}

Json::Value scanJson(const FusionSession& session, size_t i)
{
    const auto& s = session.scan(i);
    Json::Value o(Json::objectValue);
    o["label"] = s.label();
    o["ply"] = s.input.ply_path;
    const int g = session.rotationGroupOf(i);
    o["rotation_group"] = g >= 0 ? shortGroupName(session.rotationGroups()[g]) : "";
    o["status"] = toString(s.status);
    o["transform"] = matrix(s.transform);
    o["coarse_transform"] = matrix(s.coarse_transform);
    o["seed_transform"] = s.input.has_initial_guess ? matrix(s.input.initial_guess) : Json::Value(Json::nullValue);
    o["coarse_source"] = s.coarse_source;
    o["coarse_quality"] = quality(s.coarse_quality);
    o["fine_quality"] = quality(s.fine_quality);

    Json::Value icp(Json::objectValue);
    icp["converged"] = s.icp_result.converged;
    icp["fitness"] = number(s.icp_result.fitness_score);
    icp["hit_max_iterations"] = s.icp_result.hit_max_iterations;
    o["icp"] = icp;

    Json::Value tags(Json::objectValue);
    tags["located"] = static_cast<Json::UInt64>(s.tags.size());
    tags["fit_valid"] = s.tag_fit.valid;
    tags["fit_rms_mm"] = s.tag_fit.valid ? number(s.tag_fit.rms * 1000.0) : Json::Value(Json::nullValue);
    tags["shared"] = s.tag_fit.shared_tags;
    Json::Value ids(Json::arrayValue);
    for (int id : s.tag_fit.ids) ids.append(id);
    tags["ids"] = ids;
    tags["fit_transform"] = s.tag_fit.valid ? matrix(s.tag_fit.transform) : Json::Value(Json::nullValue);
    tags["rejected_rms_mm"] = s.tag_rejected_rms >= 0.0 ? number(s.tag_rejected_rms * 1000.0)
                                                         : Json::Value(Json::nullValue);
    o["tags"] = tags;

    if (s.has_tag_disagreement)
    {
        Json::Value d(Json::objectValue);
        d["angle_deg"] = s.tag_disagreement_deg;
        d["shift_mm"] = s.tag_disagreement_shift * 1000.0;
        o["tag_vs_coarse"] = d;
    }
    else
    {
        o["tag_vs_coarse"] = Json::Value(Json::nullValue);
    }

    o["group_ambiguous"] = g >= 0 && session.rotationGroups()[g].ambiguous;
    o["group_margin"] = g >= 0 ? session.rotationGroups()[g].margin : 0.0;

    if (s.has_roi)
    {
        Json::Value roi(Json::objectValue);
        roi["min"] = vec3(s.roi_min.head<3>());
        roi["max"] = vec3(s.roi_max.head<3>());
        o["roi"] = roi;
    }
    else
    {
        o["roi"] = Json::Value(Json::nullValue);
    }

    o["flagged"] = s.flagged;
    o["qc_reasons"] = strings(s.qc_reasons);
    o["warnings"] = strings(s.qc_warnings);
    o["needs_review"] = s.needs_review;

    Json::Value retries(Json::arrayValue);
    for (const auto& r : s.retries)
    {
        Json::Value a(Json::objectValue);
        a["rung"] = r.rung;
        a["score_before"] = number(r.score_before);
        a["score_after"] = number(r.score_after);
        a["kept"] = r.kept;
        a["cleared"] = r.cleared;
        retries.append(a);
    }
    o["retry"] = retries;

    o["excluded_by"] = s.excluded_by.empty() ? Json::Value(Json::nullValue) : Json::Value(s.excluded_by);
    o["reviewed"] = s.reviewed;
    o["reviewed_at"] = s.reviewed_at.empty() ? Json::Value(Json::nullValue) : Json::Value(s.reviewed_at);
    return o;
}

Json::Value paramsJson(const FusionSession& session)
{
    const auto& p = session.params();
    const auto& rig = session.rigGeometry();
    Json::Value o(Json::objectValue);
    o["scan_voxel"] = p.voxel_leaf_size;
    o["tag_align"] = p.use_tag_alignment;
    o["tag_min_shared"] = p.tag_min_shared;
    o["tag_max_fit_rmse"] = p.tag_max_fit_rmse;
    o["tag_disagreement_warn_deg"] = p.tag_disagreement_warn_deg;
    o["use_imu_tilt"] = session.usingImuTilt();
    o["calibrate_tilt_geometry"] = p.calibrate_tilt_geometry;
    o["infer_rotation_direction"] = p.infer_rotation_direction;
    o["icp_max_iterations"] = p.icp_max_iterations;
    o["icp_max_correspondence_distance"] = p.icp_max_correspondence_distance;
    o["flag_score_ratio"] = p.flag_score_ratio;

    Json::Value r(Json::objectValue);
    r["tilt_axis"] = axisName(rig.tilt_axis);
    r["tilt_sign"] = rig.tilt_sign;
    r["tilt_pivot"] = vec3(rig.tilt_pivot);
    r["rotation_axis"] = axisName(rig.rotation_axis);
    r["rotation_sign"] = rig.rotation_sign;
    r["imu_frame"] = rig.imu_frame;
    o["rig"] = r;

    Json::Value qc(Json::objectValue);
    qc["min_coarse_inlier_fraction"] = p.qc_min_coarse_inlier_fraction;
    qc["max_spread_before_mm"] = p.qc_max_spread_before * 1000.0;
    qc["max_spread_after_mm"] = p.qc_max_spread_after * 1000.0;
    qc["flag_score_ratio"] = p.flag_score_ratio;
    qc["tag_disagreement_warn_deg"] = p.tag_disagreement_warn_deg;
    qc["direction_ambiguous_margin"] = p.direction_ambiguous_margin;
    o["qc"] = qc;
    return o;
}

Json::Value header(const char* schema, const RunInfo& run)
{
    Json::Value doc(Json::objectValue);
    doc["schema"] = schema;
    Json::Value tool(Json::objectValue);
    tool["name"] = "fuse_scans";
    tool["version"] = toolVersion();
    tool["built"] = std::string(__DATE__) + " " + __TIME__;
    doc["tool"] = tool;
    doc["created"] = run.created;
    doc["command"] = run.command;
    return doc;
}

} // namespace

// -------------------------------------------------------------------------
std::string isoTimestamp()
{
    const std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm local{};
    localtime_r(&now, &local);
    std::ostringstream ss;
    ss << std::put_time(&local, "%Y-%m-%dT%H:%M:%S");
    return ss.str();
}

std::string toolVersion() { return FUSE_VERSION; }

bool readJsonFile(const std::string& path, Json::Value& out, std::string& error)
{
    std::ifstream in(path);
    if (!in) { error = "cannot open " + path; return false; }
    Json::CharReaderBuilder builder;
    std::string errs;
    if (!Json::parseFromStream(builder, in, &out, &errs))
    {
        error = path + ": " + errs;
        return false;
    }
    return true;
}

bool writeJsonFile(const std::string& path, const Json::Value& value, std::string& error)
{
    // Written beside the target and renamed over it, so a run killed half way
    // (the sweep's timeout) never leaves a truncated poses file behind.
    const std::string tmp = path + ".tmp";
    {
        std::ofstream out(tmp);
        if (!out) { error = "cannot write " + tmp; return false; }
        Json::StreamWriterBuilder builder;
        builder["indentation"] = " ";
        builder["precision"] = 9;   // float transforms round-trip exactly at 9 significant digits
        std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
        writer->write(value, &out);
        out << "\n";
        if (!out) { error = "write failed: " + tmp; return false; }
    }
    if (std::rename(tmp.c_str(), path.c_str()) != 0) { error = "cannot rename " + tmp; return false; }
    return true;
}

// -------------------------------------------------------------------------
Json::Value PoseIO::posesDocument(const FusionSession& session, const RunInfo& run)
{
    Json::Value doc = header(kPosesSchema, run);
    doc["params"] = paramsJson(session);

    Json::Value s(Json::objectValue);
    s["name"] = session.size() ? session.scan(0).input.session_name : "";
    s["reference"] = session.referenceIndex() < session.size()
        ? session.scan(session.referenceIndex()).input.shortLabel() : "";

    const auto& plane = session.scene_plane_;
    Json::Value p(Json::objectValue);
    p["valid"] = plane.valid;
    p["origin"] = vec3(plane.origin);
    p["basis"] = matrix(plane.basis);
    p["planarity"] = plane.planarity;
    s["scene_plane"] = p;

    const auto& c = session.tilt_calibration_;
    Json::Value t(Json::objectValue);
    t["valid"] = c.valid;
    t["frame"] = c.frame_name;
    t["frame_measured"] = c.measured_frame_name;
    t["frame_forced"] = c.frame_forced;
    t["imu_to_cloud"] = matrix(c.imu_to_cloud);
    t["pivot"] = vec3(c.pivot);
    t["residual"] = c.residual;
    t["frame_score"] = c.frame_score;
    t["frame_runner_up"] = number(c.frame_runner_up);
    s["tilt_calibration"] = t;

    Json::Value groups(Json::arrayValue);
    for (const auto& g : session.rotationGroups())
    {
        Json::Value o(Json::objectValue);
        o["name"] = shortGroupName(g);
        // As the folder name records it (always positive); cumulative_deg is what was used.
        o["folder_deg"] = g.scans.empty() ? 0.0 : session.scan(g.scans.front()).input.rotation_degrees;
        o["nominal_step_deg"] = g.nominal_step_deg;
        o["cumulative_deg"] = g.cumulative_deg;
        o["step_sign"] = g.step_sign;
        o["from_tags"] = g.from_tags;
        o["tag_spread_deg"] = g.tag_spread_deg;
        o["resolved"] = g.resolved;
        o["ambiguous"] = g.ambiguous;
        o["margin"] = g.margin;
        Json::Value scans(Json::arrayValue);
        for (size_t i : g.scans) scans.append(session.scan(i).input.shortLabel());
        o["scans"] = scans;
        groups.append(o);
    }
    s["rotation_groups"] = groups;
    s["flags"] = strings(session.sessionFlags());
    s["needs_review"] = session.needsReview();

    Json::Value review(Json::objectValue);
    review["state"] = session.reviewState();
    review["reviewed_at"] = session.reviewed_at_.empty() ? Json::Value(Json::nullValue)
                                                         : Json::Value(session.reviewed_at_);
    s["review"] = review;

    Json::Value retries(Json::arrayValue);
    for (const auto& r : session.groupRetries())
    {
        Json::Value a(Json::objectValue);
        a["rung"] = "flip_group";
        a["group"] = r.group;
        a["score_before"] = r.score_before;
        a["score_after"] = r.score_after;
        a["kept"] = r.kept;
        retries.append(a);
    }
    s["auto_retry"] = retries;
    doc["session"] = s;

    Json::Value scans(Json::objectValue);
    for (size_t i = 0; i < session.size(); ++i)
        scans[session.scan(i).input.shortLabel()] = scanJson(session, i);
    doc["scans"] = scans;
    return doc;
}

// -------------------------------------------------------------------------
Json::Value PoseIO::qcDocument(const FusionSession& session, const RunInfo& run,
                               const std::string& mode, const ReconcileReport* report,
                               const ReconcileParams& reconcile, int exit_code)
{
    Json::Value doc = header(kQcSchema, run);
    doc["mode"] = mode;
    doc["params"] = paramsJson(session);
    doc["params"]["fusion_cell"] = reconcile.height_cell;
    doc["params"]["fusion_voxel"] = reconcile.voxel_size;
    doc["thresholds"] = doc["params"]["qc"];

    Json::Value scans(Json::objectValue);
    std::vector<std::string> flagged;
    for (size_t i = 0; i < session.size(); ++i)
    {
        const auto& s = session.scan(i);
        scans[s.input.shortLabel()] = scanJson(session, i);
        if (s.needs_review && s.status != ScanStatus::Excluded) flagged.push_back(s.input.shortLabel());
    }
    doc["scans"] = scans;
    doc["flagged_scans"] = strings(flagged);

    if (report)
    {
        Json::Value r(Json::objectValue);
        r["mode_used"] = report->mode_used;
        r["spread_before_mm"] = report->spread_before * 1000.0;
        r["spread_after_mm"] = report->spread_after * 1000.0;
        r["input_points"] = static_cast<Json::UInt64>(report->input_points);
        r["cells_total"] = static_cast<Json::UInt64>(report->cells_total);
        r["cells_dropped_no_corroboration"] = static_cast<Json::UInt64>(report->cells_dropped_no_corroboration);
        r["mean_scans_per_cell"] = report->mean_scans_per_cell;
        r["points_after_blend"] = static_cast<Json::UInt64>(report->points_after_blend);
        r["points_after_outlier_removal"] = static_cast<Json::UInt64>(report->points_after_outlier_removal);
        r["output_points"] = static_cast<Json::UInt64>(report->output_points);
        doc["reconcile"] = r;
    }
    else
    {
        doc["reconcile"] = Json::Value(Json::nullValue);
    }
    doc["session_flags"] = strings(session.sessionFlags());
    doc["review_state"] = session.reviewState();
    doc["needs_review"] = session.needsReview();
    doc["exit_code"] = exit_code;
    return doc;
}

// -------------------------------------------------------------------------
std::vector<std::pair<std::string, std::string>> PoseIO::scanLabels(const Json::Value& doc)
{
    std::vector<std::pair<std::string, std::string>> out;
    const auto& scans = doc["scans"];
    for (const auto& label : scans.getMemberNames())
    {
        const auto& s = scans[label];
        std::string excluded = s["excluded_by"].isString() ? s["excluded_by"].asString() : "";
        if (excluded.empty() && s["status"].asString() == toString(ScanStatus::Excluded)) excluded = "auto";
        out.emplace_back(label, excluded);
    }
    return out;
}

// -------------------------------------------------------------------------
void PoseIO::applyPreLoad(FusionSession& session, const Json::Value& doc)
{
    const auto& scans = doc["scans"];
    for (size_t i = 0; i < session.size(); ++i)
    {
        auto& s = session.scan(i);
        const auto& roi = scans[s.input.shortLabel()]["roi"];
        Eigen::Vector3f lo, hi;
        if (roi.isObject() && readVec3(roi["min"], lo) && readVec3(roi["max"], hi))
        {
            s.has_roi = true;
            s.roi_min << lo, 1.0f;
            s.roi_max << hi, 1.0f;
        }
    }
}

// -------------------------------------------------------------------------
bool PoseIO::applyPoses(FusionSession& session, const Json::Value& doc, std::string& error)
{
    if (doc["schema"].asString() != kPosesSchema)
    {
        error = "not a poses file (schema '" + doc["schema"].asString() + "', expected " + kPosesSchema + ")";
        return false;
    }
    const auto& scans = doc["scans"];
    const auto& sdoc = doc["session"];

    // --- Session state ------------------------------------------------------
    const auto& p = sdoc["scene_plane"];
    PlaneFrame plane;
    plane.valid = p["valid"].asBool();
    readVec3(p["origin"], plane.origin);
    readMatrix(p["basis"], plane.basis);
    plane.planarity = numberOr(p["planarity"], 1.0);
    session.scene_plane_ = plane;

    const auto& t = sdoc["tilt_calibration"];
    auto& c = session.tilt_calibration_;
    c.valid = t["valid"].asBool();
    c.frame_name = t["frame"].asString();
    c.measured_frame_name = t["frame_measured"].asString();
    c.frame_forced = t["frame_forced"].asBool();
    readMatrix(t["imu_to_cloud"], c.imu_to_cloud);
    readVec3(t["pivot"], c.pivot);
    c.residual = numberOr(t["residual"], 0.0);
    c.frame_score = numberOr(t["frame_score"], 0.0);
    c.frame_runner_up = numberOr(t["frame_runner_up"], 0.0);

    std::map<std::string, const Json::Value*> groups;
    for (const auto& g : sdoc["rotation_groups"]) groups[g["name"].asString()] = &g;
    for (auto& g : session.rotation_groups_)
    {
        const auto it = groups.find(shortGroupName(g));
        if (it == groups.end()) continue;
        const auto& o = *it->second;
        g.cumulative_deg = numberOr(o["cumulative_deg"], g.cumulative_deg);
        g.step_sign = static_cast<float>(numberOr(o["step_sign"], g.step_sign));
        g.from_tags = o["from_tags"].asBool();
        g.tag_spread_deg = numberOr(o["tag_spread_deg"], 0.0);
        g.resolved = o["resolved"].asBool();
        g.ambiguous = o["ambiguous"].asBool();
        g.margin = numberOr(o["margin"], 0.0);
    }

    session.session_flags_ = readStrings(sdoc["flags"]);
    session.reviewed_at_ = sdoc["review"]["reviewed_at"].isString() ? sdoc["review"]["reviewed_at"].asString() : "";
    session.group_retries_.clear();
    for (const auto& r : sdoc["auto_retry"])
    {
        FusionSession::GroupRetry gr;
        gr.group = r["group"].asString();
        gr.score_before = numberOr(r["score_before"], 0.0);
        gr.score_after = numberOr(r["score_after"], 0.0);
        gr.kept = r["kept"].asBool();
        session.group_retries_.push_back(gr);
    }

    // --- Scans ----------------------------------------------------------------
    const std::string reference = sdoc["reference"].asString();
    size_t reference_index = session.size();
    for (size_t i = 0; i < session.size(); ++i)
    {
        auto& s = session.scan(i);
        const std::string key = s.input.shortLabel();
        if (!scans.isMember(key))
        {
            error = "scan " + key + " is not in the poses file";
            return false;
        }
        const auto& o = scans[key];

        // A scan that failed to load *now* stays excluded whatever the file says.
        const bool load_failed = s.status == ScanStatus::Excluded && !s.loaded();

        ScanStatus status;
        if (!parseScanStatus(o["status"].asString(), status))
        {
            error = "scan " + key + " has unknown status '" + o["status"].asString() + "'";
            return false;
        }
        s.status = load_failed ? ScanStatus::Excluded : status;
        if (!readMatrix(o["transform"], s.transform))
        {
            error = "scan " + key + " has no valid 4x4 transform";
            return false;
        }
        readMatrix(o["coarse_transform"], s.coarse_transform);
        if (o["seed_transform"].isArray())
        {
            readMatrix(o["seed_transform"], s.input.initial_guess);
            s.input.has_initial_guess = true;
        }
        s.coarse_source = o["coarse_source"].asString();
        s.coarse_quality = readQuality(o["coarse_quality"]);
        s.fine_quality = readQuality(o["fine_quality"]);

        const auto& icp = o["icp"];
        s.icp_result.transform = s.transform;
        s.icp_result.converged = icp["converged"].asBool();
        s.icp_result.hit_max_iterations = icp["hit_max_iterations"].asBool();
        s.icp_result.fitness_score = numberOr(icp["fitness"], std::numeric_limits<double>::max());

        const auto& tags = o["tags"];
        s.tag_fit = TagFit{};
        if (tags["fit_valid"].asBool())
        {
            s.tag_fit.valid = true;
            s.tag_fit.rms = numberOr(tags["fit_rms_mm"], 0.0) / 1000.0;
            s.tag_fit.shared_tags = tags["shared"].asInt();
            for (const auto& id : tags["ids"]) s.tag_fit.ids.push_back(id.asInt());
            readMatrix(tags["fit_transform"], s.tag_fit.transform);
        }
        s.tag_rejected_rms = tags["rejected_rms_mm"].isNumeric() ? tags["rejected_rms_mm"].asDouble() / 1000.0 : -1.0;

        const auto& d = o["tag_vs_coarse"];
        s.has_tag_disagreement = d.isObject();
        s.tag_disagreement_deg = d.isObject() ? numberOr(d["angle_deg"], 0.0) : 0.0;
        s.tag_disagreement_shift = d.isObject() ? numberOr(d["shift_mm"], 0.0) / 1000.0 : 0.0;

        s.flagged = o["flagged"].asBool();
        s.qc_reasons = readStrings(o["qc_reasons"]);
        s.qc_warnings = readStrings(o["warnings"]);
        s.needs_review = o["needs_review"].asBool();
        s.retries.clear();
        for (const auto& r : o["retry"])
        {
            ScanState::RetryAttempt a;
            a.rung = r["rung"].asString();
            a.score_before = numberOr(r["score_before"], 0.0);
            a.score_after = numberOr(r["score_after"], 0.0);
            a.kept = r["kept"].asBool();
            a.cleared = r["cleared"].asBool();
            s.retries.push_back(a);
        }
        s.excluded_by = load_failed ? "auto" : (o["excluded_by"].isString() ? o["excluded_by"].asString() : "");
        s.reviewed = o["reviewed"].asBool();
        s.reviewed_at = o["reviewed_at"].isString() ? o["reviewed_at"].asString() : "";

        if (key == reference) reference_index = i;
    }

    // The registration's reference, when it is part of this run; otherwise
    // (a combination without r0_a0) the first scan that contributes. Either
    // way every transform is already in the registration's common frame.
    if (reference_index == session.size())
    {
        for (size_t i = 0; i < session.size(); ++i)
            if (session.scan(i).loaded() && contributesToFusion(session.scan(i).status)) { reference_index = i; break; }
    }
    session.reference_index_ = reference_index;
    return true;
}
