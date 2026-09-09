#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "fusion_pipeline.h"

enum class RigAxis { None, X, Y, Z };

struct DiscoveryOptions
{
    // --- Tilt (within a session) ---
    //
    // Matches the trailing tilt-angle suffix on a tilt subfolder's name,
    // e.g. "session_a0", "session_a60", "session_a-15". Must contain
    // exactly one capture group holding the (possibly signed) integer
    // degree value.
    std::string tilt_pattern = R"(_a(-?[0-9]+)$)";

    // If set, auto-generate the tilt component of the initial-guess
    // transform for every scan whose tilt angle was parsed from its
    // folder name: a pure rotation of `tilt_degrees` about this axis.
    // Leave as None to rely purely on feature-based coarse alignment for
    // tilt too (or supply transforms yourself via --manifest).
    RigAxis tilt_axis = RigAxis::None;
    float   tilt_sign = 1.0f; // flip to -1 if your rig's tilt convention is reversed

    // --- Rotation (its own subfolder level between the session and the
    //     tilt subfolders: <session>/<session>_r45/<session>_r45_a15/...) ---
    //
    // Matches a rotation-angle token in the *rotation subfolder's* name,
    // e.g. "session_r45", "rig_r-90". Optional — if a rotation folder's
    // name doesn't match, its rotation falls back to
    // `rotation_index * rotation_step_deg` (rotation folders numbered in
    // the order found within each session, resetting per session), which
    // is exactly the "I know it's roughly 45 degrees per step" case. Set
    // rotation_step_deg to 0 (default) to disable the fallback and only
    // use rotation angles that were explicitly parsed.
    std::string rotation_pattern = R"(_r(-?[0-9]+)(?:_|$))";
    RigAxis     rotation_axis = RigAxis::None; // None disables rotation seeding entirely
    float       rotation_step_deg = 0.0f;
    float       rotation_sign = 1.0f;

    // How the tilt and rotation components combine into one transform:
    // true  -> world = R_rotation * R_tilt   (tilt happens in the frame
    //                                          that then gets rotated -
    //                                          typical for a tilt head
    //                                          mounted on a turntable)
    // false -> world = R_tilt * R_rotation   (rotation happens first,
    //                                          tilt axis fixed in world)
    // If alignment quality is consistently poor with rotation seeding on,
    // try flipping this before assuming the angles themselves are wrong.
    bool compose_tilt_first = true;

    // Reorder discovered scans so the (first found) 0-degree tilt scan
    // becomes the reference frame, and the rest proceed outward in
    // ascending |tilt| — this tends to be much easier for registration
    // than fusing in arbitrary directory-listing order.
    bool reorder_by_tilt = true;
};

// Loads key=value pairs from a simple text config file into `opts`,
// overwriting only the fields present in the file (so a partial config is
// fine). Recognised keys: tilt_axis, tilt_sign, tilt_pattern,
// rotation_axis, rotation_step_deg, rotation_sign, rotation_pattern,
// compose_tilt_first, reorder_by_tilt. Lines starting with '#' and blank
// lines are ignored. This is the intended way to tune rig geometry
// (tilt/rotation step, axis, sign) without recompiling - edit the file and
// rerun. Returns false (and logs why) if the file couldn't be read.
bool loadRigConfigFile(const std::string& path, DiscoveryOptions& opts);

// Walks each session directory two levels deep - rotation subfolders, then
// tilt subfolders inside each of those - and locates the single PLY file
// inside each tilt subfolder (preferring "<tilt_folder_name>.ply", falling
// back to whichever .ply file is present if there's exactly one). Expects:
//   <session>/<session>_r<rot>/<session>_r<rot>_a<tilt>/<...>.ply
//
// `rotation_sign_overrides` lets a single session (keyed by its directory's
// basename, i.e. what ends up in ScanInput::session_name) use a different
// rotation_sign than the rest - for when you physically turned the rig the
// opposite way for that capture but its rotation subfolders are still
// (necessarily) named with plain unsigned angles, so opts.rotation_sign
// alone can't tell the two apart.
std::vector<ScanInput> discoverScansFromSessions(const std::vector<std::string>& session_dirs,
                                                   const DiscoveryOptions& opts,
                                                   const std::unordered_map<std::string, float>&
                                                       rotation_sign_overrides = {});

// Prints what was discovered (session, subfolder, tilt angle, ply path)
// without running the fusion - useful for sanity-checking directory
// layout or hand-writing a manifest override.
void printDiscoveredScans(const std::vector<ScanInput>& scans);
