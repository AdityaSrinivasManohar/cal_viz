#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb/stb_image_write.h>

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>

#include "bag/mcap_reader.hpp"
#include "msgs/deserialize.hpp"
#include "tf/tf_buffer.hpp"

namespace fs = std::filesystem;

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

    // ── stream messages ───────────────────────────────────────────────────────
    TfBuffer   tf;
    RawMessage msg;
    uint64_t   images_saved = 0;
    uint64_t   images_failed = 0;

    while (reader.next(msg)) {
        // TF — populate buffer from both static and dynamic topics.
        if (msg.msg_type.ends_with("TFMessage")) {
            bool is_static = (msg.topic == "/tf_static");
            for (auto& st : msgs::as_tf_message(msg)) {
                is_static ? tf.add_static(st) : tf.add_dynamic(st);
            }
            continue;
        }

        // Images — write to disk.
        if (msg.msg_type.ends_with("Image")) {
            auto img = msgs::as_image(msg);
            if (!img) {
                ++images_failed;
                continue;
            }

            std::string rel = msg.topic.front() == '/' ? msg.topic.substr(1) : msg.topic;
            fs::path    out_path = out_dir / rel / (std::to_string(msg.log_time_ns) + ".jpg");
            fs::create_directories(out_path.parent_path());

            int ok = stbi_write_jpg(out_path.string().c_str(), static_cast<int>(img->width),
                                    static_cast<int>(img->height), 3, img->data.data(), 90);
            ok ? ++images_saved : ++images_failed;
        }
    }

    // ── TF tree summary ───────────────────────────────────────────────────────
    std::cout << "TF tree:\n";
    for (auto& frame : tf.frames()) std::cout << "  " << frame << "\n";

    std::cout << "\nimages: saved " << images_saved;
    if (images_failed) std::cout << "  failed " << images_failed;
    std::cout << "\n";

    return 0;
}
