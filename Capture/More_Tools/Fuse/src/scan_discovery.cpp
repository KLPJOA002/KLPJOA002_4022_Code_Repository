#include "scan_discovery.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>

namespace fs = std::filesystem;

namespace
{

Eigen::Matrix4f rotationForAxis(RigAxis axis, float degrees_signed)
{
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    float rad = degrees_signed * static_cast<float>(M_PI) / 180.0f;
    Eigen::Matrix3f R;
    switch (axis)
    {
        case RigAxis::X:
            R = Eigen::AngleAxisf(rad, Eigen::Vector3f::UnitX()).toRotationMatrix();
            break;
        case RigAxis::Y:
            R = Eigen::AngleAxisf(rad, Eigen::Vector3f::UnitY()).toRotationMatrix();
            break;
        case RigAxis::Z:
            R = Eigen::AngleAxisf(rad, Eigen::Vector3f::UnitZ()).toRotationMatrix();
            break;
        default:
            R = Eigen::Matrix3f::Identity();
    }
    T.block<3, 3>(0, 0) = R;
    return T;
}

// Finds the single "best frame" PLY inside a tilt subfolder. Prefers a
// file named exactly "<subfolder_name>.ply"; otherwise, if exactly one
// .ply file is present, uses that.
bool findPlyInFolder(const fs::path& folder, const std::string& folder_name, fs::path& out_path)
{
    fs::path preferred = folder / (folder_name + ".ply");
    if (fs::exists(preferred))
    {
        out_path = preferred;
        return true;
    }

    std::vector<fs::path> candidates;
    for (const auto& entry : fs::directory_iterator(folder))
    {
        if (entry.is_regular_file() && entry.path().extension() == ".ply")
            candidates.push_back(entry.path());
    }

    if (candidates.size() == 1)
    {
        out_path = candidates.front();
        return true;
    }
    if (candidates.size() > 1)
    {
        std::sort(candidates.begin(), candidates.end());
        std::cerr << "  [!] " << folder
                  << " contains multiple .ply files with no exact '"
                  << folder_name << ".ply' match - using " << candidates.front()
                  << " (rename the intended file to disambiguate)\n";
        out_path = candidates.front();
        return true;
    }
    return false;
}

RigAxis parseAxisToken(const std::string& raw)
{
    std::string v = raw;
    std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return std::tolower(c); });
    if (v == "x") return RigAxis::X;
    if (v == "y") return RigAxis::Y;
    if (v == "z") return RigAxis::Z;
    if (v == "none" || v.empty()) return RigAxis::None;
    std::cerr << "  [!] unrecognised axis '" << raw << "' in rig config, treating as none\n";
    return RigAxis::None;
}

std::string trim(const std::string& s)
{
    size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

bool parseBool(const std::string& raw, bool fallback)
{
    std::string v = raw;
    std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return std::tolower(c); });
    if (v == "true" || v == "1" || v == "yes" || v == "on") return true;
    if (v == "false" || v == "0" || v == "no" || v == "off") return false;
    return fallback;
}

// Builds the combined tilt+rotation initial guess for one scan. Either
// component may be "unknown" (not seeded), in which case it contributes
// identity and, if *both* are unknown, has_guess comes back false so the
// caller falls through to feature-based coarse alignment as before.
Eigen::Matrix4f buildInitialGuess(const DiscoveryOptions& opts,
                                    float rotation_sign,
                                    bool have_rotation, double rotation_deg,
                                    bool have_tilt, double tilt_deg,
                                    bool& has_guess)
{
    has_guess = have_rotation || have_tilt;
    if (!has_guess) return Eigen::Matrix4f::Identity();

    Eigen::Matrix4f R_rot = have_rotation
        ? rotationForAxis(opts.rotation_axis, rotation_sign * static_cast<float>(rotation_deg))
        : Eigen::Matrix4f::Identity();
    Eigen::Matrix4f R_tilt = have_tilt
        ? rotationForAxis(opts.tilt_axis, opts.tilt_sign * static_cast<float>(tilt_deg))
        : Eigen::Matrix4f::Identity();

    return opts.compose_tilt_first ? (R_rot * R_tilt) : (R_tilt * R_rot);
}

} // namespace

bool loadRigConfigFile(const std::string& path, DiscoveryOptions& opts)
{
    std::ifstream file(path);
    if (!file)
    {
        std::cerr << "  [!] could not open rig config: " << path << "\n";
        return false;
    }

    std::string line;
    int line_no = 0;
    while (std::getline(file, line))
    {
        ++line_no;
        std::string trimmed = trim(line);
        if (trimmed.empty() || trimmed[0] == '#') continue;

        auto eq = trimmed.find('=');
        if (eq == std::string::npos)
        {
            std::cerr << "  [!] " << path << ":" << line_no
                      << " - expected 'key = value', ignoring line: " << trimmed << "\n";
            continue;
        }

        std::string key = trim(trimmed.substr(0, eq));
        std::string value = trim(trimmed.substr(eq + 1));
        std::transform(key.begin(), key.end(), key.begin(), [](unsigned char c) { return std::tolower(c); });

        if (key == "tilt_axis") opts.tilt_axis = parseAxisToken(value);
        else if (key == "tilt_sign") opts.tilt_sign = std::stof(value);
        else if (key == "tilt_pattern") opts.tilt_pattern = value;
        else if (key == "rotation_axis") opts.rotation_axis = parseAxisToken(value);
        else if (key == "rotation_step_deg") opts.rotation_step_deg = std::stof(value);
        else if (key == "rotation_sign") opts.rotation_sign = std::stof(value);
        else if (key == "rotation_pattern") opts.rotation_pattern = value;
        else if (key == "compose_tilt_first") opts.compose_tilt_first = parseBool(value, opts.compose_tilt_first);
        else if (key == "reorder_by_tilt") opts.reorder_by_tilt = parseBool(value, opts.reorder_by_tilt);
        else std::cerr << "  [!] " << path << ":" << line_no << " - unknown key '" << key << "', ignoring\n";
    }

    std::cout << "[config] loaded rig geometry from " << path << "\n";
    return true;
}

std::vector<ScanInput> discoverScansFromSessions(const std::vector<std::string>& session_dirs,
                                                   const DiscoveryOptions& opts,
                                                   const std::unordered_map<std::string, float>&
                                                       rotation_sign_overrides)
{
    std::regex tilt_regex(opts.tilt_pattern, std::regex::icase);
    std::regex rotation_regex(opts.rotation_pattern, std::regex::icase);
    std::vector<ScanInput> scans;

    for (const auto& session_dir_str : session_dirs)
    {
        fs::path session_dir(session_dir_str);
        if (!fs::is_directory(session_dir))
        {
            std::cerr << "  [!] Not a directory, skipping: " << session_dir_str << "\n";
            continue;
        }
        std::string session_name = session_dir.filename().string();
        if (session_name.empty()) // handles a trailing slash
            session_name = session_dir.parent_path().filename().string();

        float rotation_sign = opts.rotation_sign;
        // Match against either the plain session name (its directory's
        // basename) or whatever full path/string was actually passed on
        // the command line - --reverse-rotation is naturally given the
        // same argument as --sessions, so it should just work either way.
        auto override_it = rotation_sign_overrides.find(session_name);
        if (override_it == rotation_sign_overrides.end())
            override_it = rotation_sign_overrides.find(session_dir_str);
        if (override_it != rotation_sign_overrides.end())
        {
            rotation_sign = override_it->second;
            std::cout << "  [i] session '" << session_name << "' uses rotation_sign override "
                      << rotation_sign << "\n";
        }

        int found_in_session = 0;
        int rotation_ordinal = 0; // resets per session: 1st rotation folder found = 0*step, 2nd = 1*step, ...

        // --- Level 1 under a session: rotation subfolders, e.g. "<session>_r45" ---
        // Sorted by name first - fs::directory_iterator order is filesystem-
        // defined (not necessarily creation or alphabetical order), and the
        // rotation_step_deg fallback below depends on visiting these in the
        // rig's actual rotation sequence.
        std::vector<fs::path> rotation_entries;
        for (const auto& rot_entry : fs::directory_iterator(session_dir))
            if (rot_entry.is_directory())
                rotation_entries.push_back(rot_entry.path());
        std::sort(rotation_entries.begin(), rotation_entries.end());

        for (const auto& rotation_path : rotation_entries)
        {
            std::string rotation_folder_name = rotation_path.filename().string();

            bool have_rotation = false;
            double rotation_deg = 0.0;
            if (opts.rotation_axis != RigAxis::None)
            {
                std::smatch rmatch;
                if (std::regex_search(rotation_folder_name, rmatch, rotation_regex) && rmatch.size() > 1)
                {
                    rotation_deg = std::stod(rmatch[1].str());
                    have_rotation = true;
                }
                else if (opts.rotation_step_deg != 0.0f)
                {
                    rotation_deg = static_cast<double>(rotation_ordinal) * opts.rotation_step_deg;
                    have_rotation = true;
                    std::cout << "  [i] '" << rotation_folder_name
                              << "' has no rotation token (pattern: " << opts.rotation_pattern
                              << ") - assuming rotation = " << rotation_deg << " deg (rotation #"
                              << rotation_ordinal << " x rotation-step " << opts.rotation_step_deg << " deg)\n";
                }
                else
                {
                    std::cerr << "  [i] '" << rotation_folder_name
                              << "' has no parseable rotation token and rotation_step_deg is 0 - "
                              << "no rotation seed for scans under it\n";
                }
            }

            // --- Level 2 under a rotation folder: tilt subfolders, e.g. "..._r45_a15" ---
            int tilt_subfolders_seen = 0;
            for (const auto& tilt_entry : fs::directory_iterator(rotation_path))
            {
                if (!tilt_entry.is_directory()) continue;
                ++tilt_subfolders_seen;
                std::string tilt_folder_name = tilt_entry.path().filename().string();

                fs::path ply_path;
                if (!findPlyInFolder(tilt_entry.path(), tilt_folder_name, ply_path))
                {
                    std::cerr << "  [!] No .ply found in " << tilt_entry.path() << " - skipping\n";
                    continue;
                }

                ScanInput scan;
                scan.ply_path = ply_path.string();
                scan.session_name = session_name;
                scan.subfolder_name = rotation_folder_name + "/" + tilt_folder_name;

                bool have_tilt = false;
                double tilt_deg = 0.0;
                std::smatch tmatch;
                if (std::regex_search(tilt_folder_name, tmatch, tilt_regex) && tmatch.size() > 1)
                {
                    tilt_deg = std::stod(tmatch[1].str());
                    scan.tilt_degrees = static_cast<int>(std::lround(tilt_deg));
                    scan.has_tilt = true;
                    have_tilt = (opts.tilt_axis != RigAxis::None);
                }
                else
                {
                    std::cerr << "  [i] Could not parse a tilt angle from folder name '"
                              << tilt_folder_name << "' (pattern: " << opts.tilt_pattern
                              << ") - including scan without tilt metadata\n";
                }

                bool has_guess = false;
                scan.initial_guess = buildInitialGuess(opts, rotation_sign, have_rotation, rotation_deg,
                                                         have_tilt, tilt_deg, has_guess);
                scan.has_initial_guess = has_guess;

                scans.push_back(std::move(scan));
                ++found_in_session;
            }

            if (tilt_subfolders_seen == 0)
            {
                std::cerr << "  [!] '" << rotation_folder_name
                          << "' has no tilt subfolders inside it - skipping\n";
            }

            ++rotation_ordinal;
        }

        std::cout << "[discover] " << session_name << ": found " << found_in_session
                  << " tilt scan(s) across " << rotation_ordinal << " rotation folder(s) in "
                  << session_dir_str << "\n";
    }

    if (opts.reorder_by_tilt && !scans.empty())
    {
        auto zero_it = std::find_if(scans.begin(), scans.end(), [](const ScanInput& s) {
            return s.has_tilt && s.tilt_degrees == 0;
        });
        if (zero_it != scans.end() && zero_it != scans.begin())
        {
            std::iter_swap(scans.begin(), zero_it);
        }

        std::stable_sort(scans.begin() + 1, scans.end(),
                          [](const ScanInput& a, const ScanInput& b) {
                              if (a.has_tilt != b.has_tilt)
                                  return a.has_tilt; // tilt-tagged scans first
                              if (a.has_tilt && b.has_tilt)
                                  return std::abs(a.tilt_degrees) < std::abs(b.tilt_degrees);
                              return false; // stable: preserve discovery order otherwise
                          });
    }

    return scans;
}

void printDiscoveredScans(const std::vector<ScanInput>& scans)
{
    std::cout << "\nDiscovered " << scans.size() << " scan(s):\n";
    for (size_t i = 0; i < scans.size(); ++i)
    {
        const auto& s = scans[i];
        std::cout << "  [" << i << "] " << s.label() << "  ->  " << s.ply_path;
        if (s.has_tilt) std::cout << "   (tilt=" << s.tilt_degrees << " deg)";
        if (s.has_initial_guess) std::cout << "   [auto initial guess]";
        std::cout << "\n";
    }
}
