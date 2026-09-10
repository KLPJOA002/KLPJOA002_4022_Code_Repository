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
// The run is organised as explicit pipeline stages rather than one loop:
//
//   load -> coarse-align (all scans) -> review
//        -> refine (all scans)       -> review -> fuse
//
// Each batch stage processes every scan and reports per-scan alignment
// error, so you review a whole pass at once instead of deciding scan by
// scan. From either review stage you can re-run coarse alignment or fine
// registration and look again. Pass --no-gui to run all stages straight
// through with no review (headless batch); see visualizer.h for the
// review controls.

#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <cstdio>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>

#include "fusion_pipeline.h"
#include "fusion_reconcile.h"
#include "fusion_session.h"
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
        "  --no-imu-tilt            don't take the tilt from the IMU's gravity vector;\n"
        "                           use the commanded angles about --tilt-axis instead\n"
        "  --no-tilt-calibration    don't measure the tilt head's pivot offset from\n"
        "                           the low-tilt scans; rotate seeds about the origin\n"
        "                           (or about --tilt-pivot, if given)\n"
        "  --tilt-pivot x,y,z       tilt axis offset from the optical centre, metres\n"
        "  --tilt-pivot-range <m>   how far to search for the tilt axis (default 0.12).\n"
        "                           Keep it near the truth: if the tilt axis is close to\n"
        "                           the camera face, a wide search only wastes time and\n"
        "                           invites a large spurious result\n"
        "  --no-rotation-direction  don't infer which way the rig turned at each\n"
        "                           rotation step; trust the recorded angles instead\n"
        "  --no-roi             skip the per-scan crop box prompt in the viewer\n"
        "\n"
        "Fusion (how overlapping scans are combined):\n"
        "  --fusion-mode <m>    auto (default), heightfield, or volumetric\n"
        "  --fusion-voxel <m>   voxel size for volumetric fusion (default 0.002)\n"
        "  --fusion-cell <m>    lateral cell size for height-field fusion (default 0.003)\n"
        "  --fusion-min-scans <n>  drop surface fewer than n scans agree on (default 1;\n"
        "                       2 is a strong noise filter but trims each scan's edges)\n"
        "  --fusion-trim <f>    fraction trimmed from each end before averaging a cell,\n"
        "                       0=mean, 0.5=median (default 0.25 - rejects one bad scan)\n"
        "  --mls-radius <m>     surface-fit radius; set a few times your residual rmse\n"
        "                       (default 0.010). --no-mls disables the surface fit\n"
        "  --no-fusion-weights  weight every point equally instead of by registration\n"
        "                       quality, viewing angle and range\n"
        "  --no-fusion-sor      skip outlier removal on the fused cloud\n"
        "\n"
        "  --no-gui             run every stage straight through, no viewer or review\n";
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
    bool no_roi = false;
    bool infer_rotation_direction = true;
    bool calibrate_tilt = true;
    bool use_imu_tilt = true;
    ReconcileParams reconcile_params;
    std::optional<Eigen::Vector3f> tilt_pivot_override;
    std::optional<float> tilt_pivot_range;
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
        else if (arg == "--no-roi")
        {
            no_roi = true;
        }
        else if (arg == "--no-rotation-direction")
        {
            infer_rotation_direction = false;
        }
        else if (arg == "--no-tilt-calibration")
        {
            calibrate_tilt = false;
        }
        else if (arg == "--no-imu-tilt")
        {
            use_imu_tilt = false;
        }
        else if (arg == "--fusion-mode" && i + 1 < argc)
        {
            const std::string mode = argv[++i];
            if      (mode == "auto")        reconcile_params.mode = ReconcileParams::Mode::Auto;
            else if (mode == "heightfield") reconcile_params.mode = ReconcileParams::Mode::HeightField;
            else if (mode == "volumetric")  reconcile_params.mode = ReconcileParams::Mode::Volumetric;
            else { std::cerr << "Unknown --fusion-mode: " << mode
                             << " (use auto, heightfield, or volumetric)\n"; return 1; }
        }
        else if (arg == "--fusion-voxel" && i + 1 < argc)
        {
            reconcile_params.voxel_size = std::stof(argv[++i]);
        }
        else if (arg == "--fusion-cell" && i + 1 < argc)
        {
            reconcile_params.height_cell = std::stof(argv[++i]);
        }
        else if (arg == "--fusion-min-scans" && i + 1 < argc)
        {
            reconcile_params.min_contributing_scans = std::stoi(argv[++i]);
        }
        else if (arg == "--fusion-trim" && i + 1 < argc)
        {
            reconcile_params.trim_fraction = std::stof(argv[++i]);
        }
        else if (arg == "--mls-radius" && i + 1 < argc)
        {
            reconcile_params.mls_radius = std::stof(argv[++i]);
        }
        else if (arg == "--no-mls")
        {
            reconcile_params.use_mls = false;
        }
        else if (arg == "--no-fusion-weights")
        {
            reconcile_params.weight_by_registration = false;
            reconcile_params.weight_by_incidence = false;
            reconcile_params.weight_by_range = false;
        }
        else if (arg == "--no-fusion-sor")
        {
            reconcile_params.run_outlier_removal = false;
        }
        else if (arg == "--tilt-pivot-range" && i + 1 < argc)
        {
            tilt_pivot_range = std::stof(argv[++i]);
        }
        else if (arg == "--tilt-pivot" && i + 1 < argc)
        {
            float x = 0, y = 0, z = 0;
            if (std::sscanf(argv[++i], "%f,%f,%f", &x, &y, &z) != 3)
            { std::cerr << "--tilt-pivot expects x,y,z in metres, e.g. 0,-0.04,-0.12\n"; return 1; }
            tilt_pivot_override = Eigen::Vector3f(x, y, z);
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
    if (tilt_axis_override) discovery_opts.rig.tilt_axis = *tilt_axis_override;
    if (tilt_sign_override) discovery_opts.rig.tilt_sign = *tilt_sign_override;
    if (rotation_axis_override) discovery_opts.rig.rotation_axis = *rotation_axis_override;
    if (rotation_step_override) discovery_opts.rotation_step_deg = *rotation_step_override;
    if (rotation_sign_override) discovery_opts.rig.rotation_sign = *rotation_sign_override;
    if (tilt_pivot_override) discovery_opts.rig.tilt_pivot = *tilt_pivot_override;
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
        rotation_sign_overrides[session_name] = -discovery_opts.rig.rotation_sign;

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
    params.infer_rotation_direction = infer_rotation_direction;
    // An explicitly given pivot is a measurement you have already made -
    // don't overwrite it by measuring again.
    params.calibrate_tilt_geometry = calibrate_tilt && !tilt_pivot_override.has_value();
    if (tilt_pivot_range) params.tilt_pivot_search_range = *tilt_pivot_range;

    try
    {
        std::unique_ptr<FusionVisualizer> viewer;
        if (!no_gui)
            viewer = std::make_unique<FusionVisualizer>();

        FusionSession session(std::move(scans), params);
        // The session needs the rig calibration to rebuild initial guesses
        // when it tests a rotation step in the opposite direction.
        session.setRigGeometry(discovery_opts.rig);
        session.setUseImuTilt(use_imu_tilt);

        // Bridges from the session's stage callbacks to the viewer. Both
        // are no-ops when running headless, which is what keeps the stage
        // machine below identical in GUI and --no-gui runs.
        RoiProvider roi_provider = nullptr;
        if (viewer && !no_roi)
        {
            roi_provider = [&viewer](const ScanInput& input, const PointCloudT::ConstPtr& raw,
                                       Eigen::Vector4f& min_pt, Eigen::Vector4f& max_pt) {
                return viewer->selectRoi(input, raw, min_pt, max_pt);
            };
        }

        auto progressFor = [&viewer](Stage stage) -> ProgressFn {
            if (!viewer) return nullptr;
            return [&viewer, stage](size_t done, size_t total, const std::string& label) {
                viewer->showProgress(stage, done, total, label);
            };
        };

        // Asks the viewer what to do next with the results just produced.
        // Headless runs (and a closed window) always continue, so the
        // pipeline runs to completion unattended.
        auto reviewAt = [&viewer, &session](Stage stage) {
            if (!viewer || viewer->stopped()) return ReviewAction::Continue;
            return viewer->review(session, stage);
        };

        // --- Stage machine ------------------------------------------------
        // Every transition is explicit here and nowhere else; the stages
        // themselves are pure operations on the session, so a stage can be
        // re-entered any number of times without special-casing.
        Stage stage = Stage::Load;
        PointCloudNT::Ptr final_cloud;

        while (stage != Stage::Done)
        {
            switch (stage)
            {
                case Stage::Load:
                    std::cout << "\n=== Stage: load ===\n";
                    session.loadAll(progressFor(Stage::Load), roi_provider);
                    if (session.countContributing() == 0)
                        throw std::runtime_error("No scans could be loaded.");
                    stage = Stage::CalibrateTilt;
                    break;

                case Stage::CalibrateTilt:
                    std::cout << "\n=== Stage: tilt calibration ===\n";
                    session.runTiltCalibrationStage(progressFor(Stage::CalibrateTilt));
                    session.printTiltFromImuReport();
                    session.printTiltCalibrationReport();
                    stage = Stage::ResolveRotation;
                    break;

                case Stage::ResolveRotation:
                    std::cout << "\n=== Stage: rotation direction ===\n";
                    session.runRotationDirectionStage(progressFor(Stage::ResolveRotation));
                    session.printRotationReport();
                    stage = Stage::CoarseBatch;
                    break;

                case Stage::CoarseBatch:
                    std::cout << "\n=== Stage: coarse alignment (" << session.size() << " scans) ===\n";
                    session.runCoarseBatch(progressFor(Stage::CoarseBatch));
                    stage = Stage::CoarseReview;
                    break;

                case Stage::CoarseReview:
                {
                    session.printReport("Coarse alignment results:");
                    switch (reviewAt(Stage::CoarseReview))
                    {
                        case ReviewAction::Continue:    stage = Stage::RefineBatch; break;
                        case ReviewAction::RerunCoarse:
                        case ReviewAction::Back:        stage = Stage::CoarseBatch; break;
                        case ReviewAction::RerunRefine: stage = Stage::RefineBatch; break;
                        case ReviewAction::Quit:        stage = Stage::Fuse;        break;
                    }
                    break;
                }

                case Stage::RefineBatch:
                    std::cout << "\n=== Stage: fine registration ===\n";
                    session.runRefineBatch(progressFor(Stage::RefineBatch));
                    stage = Stage::RefineReview;
                    break;

                case Stage::RefineReview:
                {
                    session.printReport("Refined alignment results:");
                    switch (reviewAt(Stage::RefineReview))
                    {
                        case ReviewAction::Continue:    stage = Stage::Fuse;         break;
                        case ReviewAction::RerunCoarse: stage = Stage::CoarseBatch;  break;
                        case ReviewAction::RerunRefine: stage = Stage::RefineBatch;  break;
                        case ReviewAction::Back:        stage = Stage::CoarseReview; break;
                        case ReviewAction::Quit:        stage = Stage::Fuse;         break;
                    }
                    break;
                }

                case Stage::Fuse:
                {
                    std::cout << "\n=== Stage: fuse ===\n";
                    const auto contributions = session.buildContributions();

                    std::cout << "Reconciling " << contributions.size() << " scan(s):\n";
                    for (const auto& c : contributions)
                        std::cout << "    " << std::fixed << std::setprecision(2)
                                  << c.confidence << "  " << c.label << "\n";

                    ReconcileReport fusion_report;
                    final_cloud = reconcileScans(contributions, reconcile_params,
                                                   session.scenePlane(), &fusion_report);
                    fusion_report.print();

                    savePLY(output_path, final_cloud);
                    std::cout << "\nSaved fused scan to " << output_path << "\n";
                    stage = Stage::Done;
                    break;
                }

                case Stage::Done:
                    break;
            }
        }

        if (viewer && !viewer->stopped() && final_cloud)
            viewer->showFinal(final_cloud);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
