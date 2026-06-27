#include <stb/stb_image_write.h>

#include "commands/project.hpp"

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>

#include "bag/mcap_reader.hpp"
#include "commands/args.hpp"
#include "msgs/deserialize.hpp"
#include "projection/project.hpp"
#include "tf/tf_buffer.hpp"

namespace fs = std::filesystem;

static void print_help() {
    std::cout <<
        "usage: cal_viz project --input <file> [options]\n\n"
        "Project LiDAR point clouds onto camera images.\n\n"
        "options:\n"
        "  --input       <file>       bag file to read (required)\n"
        "  --output      <dir>        output directory (default: out)\n"
        "  --colorize    <mode>       depth | intensity  (default: depth)\n"
        "  --point-size  <px>         dot radius in pixels (default: 2)\n"
        "  --lidar       <topic>      pin to a specific LiDAR topic\n"
        "  --camera      <topic>      pin to a specific camera topic\n"
        "  --help                     show this message\n";
}

// Derives the camera_info topic from an image topic by replacing the last
// path segment with "camera_info".
static std::string image_to_info_topic(const std::string& topic) {
    auto pos = topic.rfind('/');
    return pos == std::string::npos ? topic : topic.substr(0, pos + 1) + "camera_info";
}

namespace commands {

int project(int argc, char** argv) {
    Args args;
    try {
        args = Args::parse(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        print_help();
        return 1;
    }

    if (args.has("help")) { print_help(); return 0; }

    std::string input, output, lidar_pin, camera_pin;
    int         dot_radius;
    projection::ColorizeMode colorize_mode;
    try {
        input       = args.require("input");
        output      = args.get("output", "out");
        lidar_pin   = args.get("lidar");
        camera_pin  = args.get("camera");
        dot_radius  = args.get_int("point-size", 2);

        std::string colorize_str = args.get("colorize", "depth");
        if (colorize_str == "depth")
            colorize_mode = projection::ColorizeMode::Depth;
        else if (colorize_str == "intensity")
            colorize_mode = projection::ColorizeMode::Intensity;
        else {
            std::cerr << "error: unknown colorize mode '" << colorize_str
                      << "' (choose depth or intensity)\n";
            return 1;
        }
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        print_help();
        return 1;
    }

    McapReader reader(input);

    // ── topic summary ─────────────────────────────────────────────────────────
    auto topics = reader.topics();
    std::sort(topics.begin(), topics.end(),
              [](const TopicInfo& a, const TopicInfo& b) {
                  return a.message_count > b.message_count;
              });

    std::cout << "\ntopics in " << input << ":\n\n" << std::left;
    for (const auto& t : topics)
        std::cout << "  " << std::setw(8) << t.message_count
                  << std::setw(50) << t.topic
                  << t.msg_type << "\n";
    std::cout << "\n";

    // ── pass 1: collect TF, CameraInfo, and PointClouds ──────────────────────
    TfBuffer                                tf;
    std::map<std::string, msgs::CameraInfo> cam_infos;   // info_topic → CameraInfo
    std::map<uint64_t, msgs::PointCloud>    clouds;      // log_time_ns → PointCloud

    RawMessage msg;
    while (reader.next(msg)) {
        if (msg.msg_type.ends_with("TFMessage")) {
            bool is_static = msg.topic == "/tf_static";
            for (auto& st : msgs::as_tf_message(msg))
                is_static ? tf.add_static(st) : tf.add_dynamic(st);

        } else if (msg.msg_type.ends_with("CameraInfo")) {
            if (!camera_pin.empty()) {
                // Accept CameraInfo only for the pinned camera topic.
                std::string expected_info = image_to_info_topic(camera_pin);
                if (msg.topic != expected_info) continue;
            }
            if (auto info = msgs::as_camera_info(msg))
                cam_infos[msg.topic] = std::move(*info);

        } else if (msg.msg_type.ends_with("PointCloud2")) {
            if (!lidar_pin.empty() && msg.topic != lidar_pin) continue;
            if (auto cloud = msgs::as_point_cloud(msg))
                clouds[msg.log_time_ns] = std::move(*cloud);
        }
    }

    std::cout << "TF frames: ";
    for (auto& f : tf.frames()) std::cout << f << "  ";
    std::cout << "\n";
    std::cout << "point clouds: " << clouds.size() << "\n\n";

    if (clouds.empty()) {
        std::cerr << "error: no point clouds found";
        if (!lidar_pin.empty()) std::cerr << " on topic " << lidar_pin;
        std::cerr << "\n";
        return 1;
    }

    // ── pass 2: project and save ──────────────────────────────────────────────
    reader.rewind();

    uint64_t images_saved  = 0;
    uint64_t images_failed = 0;
    uint64_t projected     = 0;

    while (reader.next(msg)) {
        bool is_image = msg.msg_type.ends_with("Image") ||
                        msg.msg_type.ends_with("CompressedImage");
        if (!is_image) continue;
        if (!camera_pin.empty() && msg.topic != camera_pin) continue;

        auto img = msgs::as_image(msg);
        if (!img) { ++images_failed; continue; }

        // Nearest cloud by log timestamp.
        auto it = clouds.lower_bound(msg.log_time_ns);
        if (it == clouds.end()) --it;
        else if (it != clouds.begin()) {
            auto prev = std::prev(it);
            if (msg.log_time_ns - prev->first < it->first - msg.log_time_ns)
                it = prev;
        }
        const msgs::PointCloud& cloud = it->second;

        auto tf_result = tf.lookup(cloud.frame_id, img->frame_id, msg.log_time_ns);
        if (tf_result) {
            std::string info_topic = image_to_info_topic(msg.topic);
            if (cam_infos.count(info_topic)) {
                auto pts = projection::project(cloud, *tf_result, cam_infos[info_topic]);
                projection::render(*img, pts, colorize_mode, dot_radius);
                ++projected;
            }
        }

        std::string rel      = msg.topic.front() == '/' ? msg.topic.substr(1) : msg.topic;
        fs::path    out_path = fs::path(output) / rel / (std::to_string(msg.log_time_ns) + ".jpg");
        fs::create_directories(out_path.parent_path());

        int ok = stbi_write_jpg(out_path.string().c_str(),
                                static_cast<int>(img->width),
                                static_cast<int>(img->height),
                                3, img->data.data(), 90);
        ok ? ++images_saved : ++images_failed;
    }

    std::cout << "images saved:     " << images_saved << "\n"
              << "images projected: " << projected    << "\n";
    if (images_failed)
        std::cout << "images failed:    " << images_failed << "\n";
    std::cout << "output: " << output << "\n";

    return 0;
}

}  // namespace commands
