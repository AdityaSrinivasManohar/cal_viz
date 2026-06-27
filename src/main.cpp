#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb/stb_image_write.h>

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>

#include "bag/mcap_reader.hpp"
#include "msgs/deserialize.hpp"
#include "projection/project.hpp"
#include "tf/tf_buffer.hpp"

namespace fs = std::filesystem;

// Derives the camera_info topic from an image topic by replacing the last
// path segment with "camera_info".
// e.g. /kitti/camera_color_left/image_raw → /kitti/camera_color_left/camera_info
static std::string image_to_info_topic(const std::string& topic) {
    auto pos = topic.rfind('/');
    return pos == std::string::npos ? topic : topic.substr(0, pos + 1) + "camera_info";
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: cal_viz <file.mcap> [output_dir]\n";
        return 1;
    }

    fs::path   out_dir = argc >= 3 ? argv[2] : "out";
    McapReader reader(argv[1]);

    // ── topic summary ─────────────────────────────────────────────────────────
    auto topics = reader.topics();
    std::sort(topics.begin(), topics.end(), [](const TopicInfo& a, const TopicInfo& b) {
        return a.message_count > b.message_count;
    });

    std::cout << "\ntopics in " << argv[1] << ":\n\n"
              << std::left << std::setw(8) << "msgs" << std::setw(45) << "topic" << "type\n"
              << std::string(85, '-') << "\n";
    for (const auto& t : topics)
        std::cout << std::left << std::setw(8) << t.message_count << std::setw(45) << t.topic
                  << t.msg_type << "\n";
    std::cout << "\n";

    // ── pass 1: collect TF, CameraInfo, and PointClouds ──────────────────────
    TfBuffer                                tf;
    std::map<std::string, msgs::CameraInfo> cam_infos;  // info_topic → CameraInfo
    std::map<uint64_t, msgs::PointCloud>    clouds;     // log_time_ns → PointCloud

    RawMessage msg;
    while (reader.next(msg)) {
        if (msg.msg_type.ends_with("TFMessage")) {
            bool is_static = msg.topic == "/tf_static";
            if (is_static) {
                std::cout << "tf_static raw bytes[0..19]: ";
                for (int i = 0; i < std::min<int>(20, msg.data.size()); ++i)
                    std::cout << (int)msg.data[i] << " ";
                std::cout << "\n";
            }
            for (auto& st : msgs::as_tf_message(msg)) {
                is_static ? tf.add_static(st) : tf.add_dynamic(st);
                if (is_static)
                    std::cout << "[tf_static] " << st.parent_frame << " -> " << st.child_frame << "\n";
            }
        } else if (msg.msg_type.ends_with("CameraInfo")) {
            if (auto info = msgs::as_camera_info(msg))
                cam_infos[msg.topic] = std::move(*info);
        } else if (msg.msg_type.ends_with("PointCloud2")) {
            if (auto cloud = msgs::as_point_cloud(msg))
                clouds[msg.log_time_ns] = std::move(*cloud);
        }
    }

    std::cout << "TF frames: ";
    for (auto& f : tf.frames()) std::cout << f << "  ";
    std::cout << "\n\n";

    // ── pass 2: project point cloud onto each image and save ─────────────────
    reader.rewind();

    uint64_t images_saved  = 0;
    uint64_t images_failed = 0;
    uint64_t projected     = 0;

    while (reader.next(msg)) {
        if (!msg.msg_type.ends_with("Image")) continue;

        auto img = msgs::as_image(msg);
        if (!img) { ++images_failed; continue; }

        // Find the temporally nearest point cloud.
        if (!clouds.empty()) {
            auto it = clouds.lower_bound(msg.log_time_ns);
            if (it == clouds.end()) --it;
            else if (it != clouds.begin()) {
                auto prev = std::prev(it);
                if (msg.log_time_ns - prev->first < it->first - msg.log_time_ns)
                    it = prev;
            }
            const msgs::PointCloud& cloud = it->second;

            // Look up the extrinsic transform: lidar frame → camera frame.
            auto tf_result = tf.lookup(cloud.frame_id, img->frame_id, msg.log_time_ns);
            if (tf_result) {
                std::string info_topic = image_to_info_topic(msg.topic);
                if (cam_infos.count(info_topic)) {
                    auto pts = projection::project(cloud, *tf_result, cam_infos[info_topic]);
                    projection::render(*img, pts, projection::ColorizeMode::Depth);
                    ++projected;
                }
            }
        }

        // Save as JPEG, mirroring the topic hierarchy under out_dir.
        std::string rel      = msg.topic.front() == '/' ? msg.topic.substr(1) : msg.topic;
        fs::path    out_path = out_dir / rel / (std::to_string(msg.log_time_ns) + ".jpg");
        fs::create_directories(out_path.parent_path());

        int ok = stbi_write_jpg(out_path.string().c_str(),
                                static_cast<int>(img->width),
                                static_cast<int>(img->height),
                                3, img->data.data(), 90);
        ok ? ++images_saved : ++images_failed;
    }

    std::cout << "images saved:     " << images_saved << " → " << out_dir << "\n";
    std::cout << "images projected: " << projected << "\n";
    if (images_failed) std::cout << "images failed:    " << images_failed << "\n";

    return 0;
}
