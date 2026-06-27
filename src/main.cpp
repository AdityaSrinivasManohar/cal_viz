#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb/stb_image_write.h>

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>

#include "bag/mcap_reader.hpp"
#include "msgs/deserialize.hpp"

namespace fs = std::filesystem;

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: cal_viz <file.mcap> [output_dir]\n";
        return 1;
    }

    fs::path   out_dir = argc >= 3 ? argv[2] : "out";
    McapReader reader(argv[1]);

    // Print topic summary.
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

    // Stream messages, write every image topic to disk.
    RawMessage msg;
    uint64_t   saved = 0;
    uint64_t   failed = 0;

    while (reader.next(msg)) {
        if (!msg.msg_type.ends_with("Image")) continue;

        auto img = msgs::as_image(msg);
        if (!img) {
            ++failed;
            continue;
        }

        // Mirror the topic hierarchy under out_dir.
        // e.g. /kitti/camera_color_left/image_raw → out/kitti/camera_color_left/image_raw/
        std::string rel = msg.topic.front() == '/' ? msg.topic.substr(1) : msg.topic;
        fs::path    topic_dir = out_dir / rel;
        fs::create_directories(topic_dir);

        fs::path out_path = topic_dir / (std::to_string(msg.log_time_ns) + ".jpg");

        int ok = stbi_write_jpg(out_path.string().c_str(), static_cast<int>(img->width),
                                static_cast<int>(img->height),
                                3,  // RGB channels
                                img->data.data(),
                                90  // JPEG quality (0-100)
        );

        ok ? ++saved : ++failed;
    }

    std::cout << "saved " << saved << " images to " << out_dir;
    if (failed) std::cout << "  (" << failed << " failed)";
    std::cout << "\n";

    return 0;
}
