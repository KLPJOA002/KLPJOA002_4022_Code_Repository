#pragma once

#include <string>
#include <vector>

#include <json/json.h>

#include "fusion_reconcile.h"
#include "fusion_session.h"

// The poses file (--save-poses / --load-poses) and the QC file (--qc).
//
// The poses file is what lets a session be registered once and fused many
// times: it holds every scan's final transform and status, the QC that
// decides whether a human should look at it, and the session state a review
// needs to carry on where registration stopped (tilt calibration, rotation
// group angles, the scene plane). Scans are keyed by ScanInput::shortLabel(),
// e.g. "r45_a-5" - the rotation/tilt identity the folder names give, which
// is the same in every staging of the session.
//
// See the README ("Registering once per session") for the schema.

constexpr const char* kPosesSchema = "fuse_scans.poses/1";
constexpr const char* kQcSchema    = "fuse_scans.qc/1";

// What produced a file, for its header.
struct RunInfo
{
    std::string command;   // the full command line
    std::string created;   // ISO-8601 local time
};

std::string isoTimestamp();
std::string toolVersion();

bool readJsonFile(const std::string& path, Json::Value& out, std::string& error);
bool writeJsonFile(const std::string& path, const Json::Value& value, std::string& error);

struct PoseIO
{
    // --- Writing ------------------------------------------------------------
    static Json::Value posesDocument(const FusionSession& session, const RunInfo& run);

    // `mode` is "register" or "load_poses". With a report, the fused result's
    // spreads and the thresholds they were judged by are included.
    static Json::Value qcDocument(const FusionSession& session, const RunInfo& run,
                                  const std::string& mode, const ReconcileReport* report,
                                  const ReconcileParams& reconcile, int exit_code);

    // --- Reading ------------------------------------------------------------
    // Every scan label in a poses file, with how (if at all) it was excluded.
    static std::vector<std::pair<std::string, std::string>> scanLabels(const Json::Value& doc);

    // Before loadAll: re-applies each scan's stored crop box, so the cloud
    // is prepared exactly as it was when it was registered.
    static void applyPreLoad(FusionSession& session, const Json::Value& doc);

    // After loadAll: restores every scan's pose, status and QC, and the
    // session's calibration, rotation groups and scene plane. No
    // registration runs. False (with `error`) if a scan in the session is
    // missing from the file.
    static bool applyPoses(FusionSession& session, const Json::Value& doc, std::string& error);
};
