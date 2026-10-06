// rs_decode - replay an rs_marker recording and stream its depth, aligned to
// colour, to stdout as raw uint16 frames. Used by filter_sweep to fill its
// frame cache with exactly the depth rs_marker saw.
//
// Why C++: rs_marker links the system librealsense. Its align filter (generic
// build) differs from the pip pyrealsense2 wheel's SSE align by ~1.5% of the
// pixels, so only the system library reproduces rs_marker's best frame
// bit for bit (checked: same valid-pixel count as scan_info.txt).
//
// Usage:
//   rs_decode <recording.db3|.bag> <meta.jsonl> [--max-frames N] [--filters SPEC]
//
// stdout   : for every depth frame, W*H uint16 little-endian (row-major)
// meta     : line 1   {"stream": {...}}  - aligned depth size and intrinsics,
//                                          depth units, stereo baseline, HDR state
//            line 2.. {"i":..,"frame_number":..,"timestamp_ms":..,"valid":..,
//                      "sequence_id":..,"sequence_size":..,"sequence_name":..}
//
// --filters applies a post-processing chain after the align, in filter_sweep's
// order, so its output can be compared with the Python implementation:
//   dec=M,spatial=MAG:ALPHA:DELTA:HOLES,temporal=ALPHA:DELTA:PERSIST,hole=MODE
// (any subset; disparity transforms are added around spatial/temporal).
//
// The .db3 files are ROS1 bags: librealsense 2.57 reads them by content.

#include <librealsense2/rs.hpp>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

struct Chain {
    std::unique_ptr<rs2::decimation_filter>   dec;
    std::unique_ptr<rs2::spatial_filter>      spatial;
    std::unique_ptr<rs2::temporal_filter>     temporal;
    std::unique_ptr<rs2::hole_filling_filter> hole;
    rs2::disparity_transform to_disp{true}, to_depth{false};

    rs2::frame process(rs2::frame f)
    {
        if (dec) f = dec->process(f);
        const bool disp = spatial || temporal;
        if (disp) f = to_disp.process(f);
        if (spatial) f = spatial->process(f);
        if (temporal) f = temporal->process(f);
        if (disp) f = to_depth.process(f);
        if (hole) f = hole->process(f);
        return f;
    }
};

static std::vector<float> split_floats(const std::string& s, char sep)
{
    std::vector<float> v;
    std::stringstream ss(s);
    std::string tok;
    while (std::getline(ss, tok, sep)) v.push_back(std::stof(tok));
    return v;
}

static Chain parse_chain(const std::string& spec)
{
    Chain c;
    std::stringstream ss(spec);
    std::string item;
    while (std::getline(ss, item, ',')) {
        const auto eq = item.find('=');
        if (eq == std::string::npos) throw std::runtime_error("bad filter item: " + item);
        const std::string key = item.substr(0, eq);
        const auto v = split_floats(item.substr(eq + 1), ':');
        if (key == "dec" && v.size() == 1) {
            c.dec = std::make_unique<rs2::decimation_filter>(v[0]);
        } else if (key == "spatial" && v.size() == 4) {
            c.spatial = std::make_unique<rs2::spatial_filter>(v[1], v[2], v[0], v[3]);
        } else if (key == "temporal" && v.size() == 3) {
            c.temporal = std::make_unique<rs2::temporal_filter>(v[0], v[1], int(v[2]));
        } else if (key == "hole" && v.size() == 1) {
            c.hole = std::make_unique<rs2::hole_filling_filter>(int(v[0]));
        } else {
            throw std::runtime_error("bad filter item: " + item);
        }
    }
    return c;
}

static long long meta(const rs2::frame& f, rs2_frame_metadata_value v)
{
    return f.supports_frame_metadata(v) ? (long long)f.get_frame_metadata(v) : -1;
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "usage: " << argv[0] << " <recording> <meta.jsonl> [--max-frames N] [--filters SPEC]\n";
        return 2;
    }
    const std::string bag = argv[1], meta_path = argv[2];
    long long max_frames = -1;
    std::string filter_spec;
    for (int i = 3; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--max-frames" && i + 1 < argc) max_frames = std::stoll(argv[++i]);
        else if (a == "--filters" && i + 1 < argc) filter_spec = argv[++i];
        else { std::cerr << "unknown argument " << a << "\n"; return 2; }
    }

    try {
        Chain chain = parse_chain(filter_spec);
        rs2::config cfg;
        cfg.enable_device_from_file(bag, false);
        cfg.enable_stream(RS2_STREAM_DEPTH);
        cfg.enable_stream(RS2_STREAM_COLOR);
        rs2::pipeline pipe;
        auto profile = pipe.start(cfg);
        auto dev = profile.get_device();
        dev.as<rs2::playback>().set_real_time(false);   // every recorded frame, none dropped

        auto ds = dev.first<rs2::depth_sensor>();
        auto opt = [&](rs2_option o) { return ds.supports(o) ? ds.get_option(o) : -1.0f; };

        std::ofstream out_meta(meta_path);
        out_meta << std::setprecision(10);
        std::freopen(nullptr, "wb", stdout);

        rs2::align align(RS2_STREAM_COLOR);
        rs2::frameset fs;
        long long i = 0;
        bool header = false;
        while ((max_frames < 0 || i < max_frames) && pipe.try_wait_for_frames(&fs, 3000)) {
            auto aligned = align.process(fs);
            rs2::depth_frame raw = aligned.get_depth_frame();
            if (!raw) continue;
            rs2::depth_frame d = chain.process(raw).as<rs2::depth_frame>();

            if (!header) {
                const auto in = d.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
                out_meta << "{\"stream\": {\"width\": " << in.width << ", \"height\": " << in.height
                         << ", \"fx\": " << in.fx << ", \"fy\": " << in.fy << ", \"ppx\": " << in.ppx
                         << ", \"ppy\": " << in.ppy << ", \"model\": \"" << rs2_distortion_to_string(in.model)
                         << "\", \"coeffs\": [" << in.coeffs[0] << ", " << in.coeffs[1] << ", " << in.coeffs[2]
                         << ", " << in.coeffs[3] << ", " << in.coeffs[4] << "], \"depth_units\": "
                         << ds.get_depth_scale() << ", \"stereo_baseline_mm\": " << opt(RS2_OPTION_STEREO_BASELINE)
                         << ", \"hdr_enabled\": " << opt(RS2_OPTION_HDR_ENABLED)
                         << ", \"decoder\": \"rs_decode " << RS2_API_VERSION_STR << "\""
                         << ", \"filters\": \"" << filter_spec << "\"}}\n";
                header = true;
            }

            const auto* p = static_cast<const uint16_t*>(d.get_data());
            const size_t n = size_t(d.get_width()) * d.get_height();
            long long valid = 0;
            for (size_t k = 0; k < n; ++k) valid += p[k] > 0;
            if (std::fwrite(p, sizeof(uint16_t), n, stdout) != n) throw std::runtime_error("stdout closed");

            out_meta << "{\"i\": " << i << ", \"frame_number\": " << raw.get_frame_number()
                     << ", \"timestamp_ms\": " << std::fixed << std::setprecision(3) << raw.get_timestamp()
                     << std::defaultfloat << std::setprecision(10)
                     << ", \"valid\": " << valid
                     << ", \"sequence_id\": " << meta(raw, RS2_FRAME_METADATA_SEQUENCE_ID)
                     << ", \"sequence_size\": " << meta(raw, RS2_FRAME_METADATA_SEQUENCE_SIZE)
                     << ", \"sequence_name\": " << meta(raw, RS2_FRAME_METADATA_SEQUENCE_NAME) << "}\n";
            ++i;
        }
        pipe.stop();
        std::fflush(stdout);
        if (!header) { std::cerr << "no depth frames in " << bag << "\n"; return 1; }
    } catch (const std::exception& e) {
        std::cerr << "rs_decode: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
