// fuse_scans - combine multiple per-angle RealSense PLY scans of the same
// scene into a single fused point cloud.
//
// Usage:
//   fuse_scans -o out.ply --sessions session_dir/
//   fuse_scans -o out.ply scan1.ply scan2.ply scan3.ply ...       (flat list)
//   fuse_scans -o out.ply --manifest manifest.txt                 (known poses)
//
// Session mode expects, per session folder:
//   <session>/<session>_r<rot_deg>/<session>_r<rot_deg>_a<tilt_deg>/<...>.ply
// e.g. 20260909_143000/20260909_143000_r0/20260909_143000_r0_a0/...ply,
//      20260909_143000/20260909_143000_r0/20260909_143000_r0_a5/...ply,
//      20260909_143000/20260909_143000_r45/20260909_143000_r45_a0/...ply, ...
// The session folder (named by date/time) contains one rotation subfolder
// per rig rotation setting, each containing one tilt subfolder per tilt
// angle within that rotation. Pass each session folder on the command line
// - the discoverer walks it for its rotation and tilt subfolders and fuses
// everything into a single cloud (ordering scans so a 0-tilt scan seeds the
// reference frame and others come in ascending |tilt|, which registers far
// more reliably than arbitrary order).
//
// Add --tilt-axis x|y|z and/or --rotation-axis x|y|z if you know which axes
// your rig tilts/rotates about; this turns each subfolder's parsed angle
// into an initial-guess rotation fed to ICP directly, instead of relying on
// feature-based coarse alignment. Omit them (default) to align purely from
// the geometry. See --rig-config / rig_config.example.txt for the way to
// set these from an editable file instead of CLI flags each run.
//
// By default a live viewer opens and steps through each scan as it's
// aligned (space to advance, see visualizer.h for full controls). Pass
// --no-gui to run straight through without it.

#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>

#include "fusion_pipeline.h"
#include "scan_discovery.h"
#include "visualizer.h"

namespace
{

void printUsage(const char* prog)
{
    std::cerr <<
        "Usage:\n"
        "  " << prog << " -o out.ply --sessions <dir> [<dir> ...]\n"
        "  " << prog << " -o out.ply scan1.ply scan2.ply ...\n"
        "  " << prog << " -o out.ply --manifest manifest.txt\n"
        "\n"
        "Options:\n"
        "  -o <path>            output fused PLY (required)\n"
        "  --sessions <dirs...> session folders; each contains tilt subfolders,\n"
        "                       each holding one PLY (see header comment in main.cpp)\n"
        "  --manifest <path>    flat list with optional known tx ty tz qx qy qz qw poses\n"
        "  --rig-config <path>  load tilt/rotation axis, sign, and step from a text\n"
        "                       file (key = value per line) - the way to tune rig\n"
        "                       geometry without recompiling; see rig_config.example.txt\n"
        "  --tilt-axis x|y|z    seed ICP with a rotation about this axis using each\n"
        "                       subfolder's parsed tilt angle (overrides config)\n"
        "  --tilt-sign 1|-1     flip tilt angle sign if your rig's convention is reversed\n"
        "  --rotation-axis x|y|z    seed ICP with a rotation about this axis between\n"
        "                           sessions (overrides config)\n"
        "  --rotation-step-deg <n>  assumed degrees between consecutive sessions\n"
        "                           (in the order you pass them) when a session's\n"
        "                           name doesn't itself encode a rotation angle\n"
        "  --rotation-sign 1|-1     flip rotation sign if your rig's convention is reversed\n"
        "  --reverse-rotation <session_name>  flip rotation_sign for just this one session\n"
        "                           (its directory's basename) - for a session where you\n"
        "                           physically turned the rig the other way but its\n"
        "                           rotation subfolders are still named with plain\n"
        "                           unsigned angles. Repeatable.\n"
        "  --list-only          print what --sessions discovers and exit (no fusion)\n"
        "  --no-gui             run headless, skip the live viewer\n";
}

Eigen::Matrix4f transformFromTranslationQuaternion(float tx, float ty, float tz,
                                                     float qx, float qy, float qz, float qw)
{
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    Eigen::Quaternionf q(qw, qx, qy, qz);
    q.normalize();
    T.block<3, 3>(0, 0) = q.toRotationMatrix();
    T.block<3, 1>(0, 3) = Eigen::Vector3f(tx, ty, tz);
    return T;
}

std::vector<ScanInput> parseManifest(const std::string& path)
{
    std::ifstream file(path);
    if (!file)
        throw std::runtime_error("Could not open manifest: " + path);

    std::vector<ScanInput> scans;
    std::string line;
    while (std::getline(file, line))
    {
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        ScanInput scan;
        iss >> scan.ply_path;

        float tx, ty, tz, qx, qy, qz, qw;
        if (iss >> tx >> ty >> tz >> qx >> qy >> qz >> qw)
        {
            scan.initial_guess = transformFromTranslationQuaternion(tx, ty, tz, qx, qy, qz, qw);
            scan.has_initial_guess = true;
        }
        scans.push_back(scan);
    }
    return scans;
}

} // namespace

int main(int argc, char** argv)
{
    std::string output_path;
    std::string manifest_path;
    std::vector<std::string> ply_args;
    std::vector<std::string> session_dirs;
    bool no_gui = false;
    bool list_only = false;
    std::string rig_config_path;
    std::vector<std::string> reverse_rotation_sessions;

    // Collected as overrides rather than applied directly, so a
    // --rig-config file always acts as the baseline and these always win
    // on top of it - regardless of which order they appear on the command
    // line. This is what lets --rig-config be your persistent, editable
    // rig calibration while these stay one-off overrides for a single run.
    std::optional<RigAxis> tilt_axis_override, rotation_axis_override;
    std::optional<float> tilt_sign_override, rotation_step_override, rotation_sign_override;

    auto parseAxisArg = [](const std::string& axis, RigAxis& out) {
        if (axis == "x" || axis == "X") { out = RigAxis::X; return true; }
        if (axis == "y" || axis == "Y") { out = RigAxis::Y; return true; }
        if (axis == "z" || axis == "Z") { out = RigAxis::Z; return true; }
        return false;
    };

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "-o" && i + 1 < argc)
        {
            output_path = argv[++i];
        }
        else if (arg == "--manifest" && i + 1 < argc)
        {
            manifest_path = argv[++i];
        }
        else if (arg == "--sessions")
        {
            while (i + 1 < argc && std::string(argv[i + 1]).rfind("-", 0) != 0)
                session_dirs.push_back(argv[++i]);
        }
        else if (arg == "--rig-config" && i + 1 < argc)
        {
            rig_config_path = argv[++i];
        }
        else if (arg == "--tilt-axis" && i + 1 < argc)
        {
            RigAxis axis;
            if (!parseAxisArg(argv[++i], axis))
            { std::cerr << "Unknown --tilt-axis value: " << argv[i] << " (use x, y, or z)\n"; return 1; }
            tilt_axis_override = axis;
        }
        else if (arg == "--tilt-sign" && i + 1 < argc)
        {
            tilt_sign_override = std::stof(argv[++i]);
        }
        else if (arg == "--rotation-axis" && i + 1 < argc)
        {
            RigAxis axis;
            if (!parseAxisArg(argv[++i], axis))
            { std::cerr << "Unknown --rotation-axis value: " << argv[i] << " (use x, y, or z)\n"; return 1; }
            rotation_axis_override = axis;
        }
        else if (arg == "--rotation-step-deg" && i + 1 < argc)
        {
            rotation_step_override = std::stof(argv[++i]);
        }
        else if (arg == "--rotation-sign" && i + 1 < argc)
        {
            rotation_sign_override = std::stof(argv[++i]);
        }
        else if (arg == "--reverse-rotation" && i + 1 < argc)
        {
            reverse_rotation_sessions.push_back(argv[++i]);
        }
        else if (arg == "--list-only")
        {
            list_only = true;
        }
        else if (arg == "--no-gui")
        {
            no_gui = true;
        }
        else if (arg == "-h" || arg == "--help")
        {
            printUsage(argv[0]);
            return 0;
        }
        else
        {
            ply_args.push_back(arg);
        }
    }

    DiscoveryOptions discovery_opts; // built-in defaults are the baseline...
    if (!rig_config_path.empty() && !loadRigConfigFile(rig_config_path, discovery_opts))
        return 1; // ...overwritten by the config file, if given...
    if (tilt_axis_override) discovery_opts.tilt_axis = *tilt_axis_override;
    if (tilt_sign_override) discovery_opts.tilt_sign = *tilt_sign_override;
    if (rotation_axis_override) discovery_opts.rotation_axis = *rotation_axis_override;
    if (rotation_step_override) discovery_opts.rotation_step_deg = *rotation_step_override;
    if (rotation_sign_override) discovery_opts.rotation_sign = *rotation_sign_override;
    // ...overwritten last by any explicit CLI flags above.

    if (!session_dirs.empty() && !manifest_path.empty())
    {
        std::cerr << "--sessions and --manifest are mutually exclusive.\n";
        return 1;
    }

    if (output_path.empty() && !list_only)
    {
        printUsage(argv[0]);
        return 1;
    }
    if (session_dirs.empty() && manifest_path.empty() && ply_args.empty())
    {
        printUsage(argv[0]);
        return 1;
    }

    std::unordered_map<std::string, float> rotation_sign_overrides;
    for (const auto& session_name : reverse_rotation_sessions)
        rotation_sign_overrides[session_name] = -discovery_opts.rotation_sign;

    std::vector<ScanInput> scans;
    if (!session_dirs.empty())
    {
        scans = discoverScansFromSessions(session_dirs, discovery_opts, rotation_sign_overrides);
        printDiscoveredScans(scans);
    }
    else if (!manifest_path.empty())
    {
        scans = parseManifest(manifest_path);
    }
    else
    {
        for (const auto& p : ply_args)
        {
            ScanInput s;
            s.ply_path = p;
            scans.push_back(s);
        }
    }

    if (list_only)
        return 0;

    if (scans.size() < 2)
    {
        std::cerr << "Need at least 2 scans to fuse (found " << scans.size() << ").\n";
        return 1;
    }

    PipelineParams params; // defaults tuned for close-range D455/D456 scans in metres

    try
    {
        std::unique_ptr<FusionVisualizer> viewer;
        FusionStepCallback step_cb = nullptr;
        RoiSelectionCallback roi_cb = nullptr;
        if (!no_gui)
        {
            viewer = std::make_unique<FusionVisualizer>();
            step_cb = [&viewer](size_t idx, const ScanInput& s, const std::string& stage,
                                  const PointCloudT::ConstPtr& cloud,
                                  const PointCloudT::ConstPtr& merged,
                                  const RegistrationResult& r) {
                return viewer->onStep(idx, s, stage, cloud, merged, r);
            };
            roi_cb = [&viewer](size_t idx, const ScanInput& s, const PointCloudT::ConstPtr& raw) {
                return viewer->selectRoi(idx, s, raw);
            };
        }

        std::vector<RegistrationResult> results;
        auto merged = fuseScans(scans, params, &results, step_cb, roi_cb);

        std::cout << "\nRegistration summary:\n";
        for (size_t i = 0; i < results.size(); ++i)
        {
            std::cout << "  scan " << i << " (" << scans[i].label() << "): "
                      << "converged=" << results[i].converged
                      << " fitness=" << results[i].fitness_score << "\n";
        }

        std::cout << "\nMerged raw point count: " << merged->size() << "\n";
        std::cout << "Running final cleanup pass...\n";

        auto final_cloud = postProcessMerged(merged, params);
        std::cout << "Final fused point count: " << final_cloud->size() << "\n";

        savePLY(output_path, final_cloud);
        std::cout << "Saved fused scan to " << output_path << "\n";

        if (viewer && !viewer->stopped())
            viewer->showFinal(final_cloud);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
