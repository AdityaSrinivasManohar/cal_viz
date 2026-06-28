#include "commands/unbag.hpp"

#include <stb/stb_image_write.h>

#include <filesystem>
#include <iomanip>
#include <iostream>

#include "bag/mcap_reader.hpp"
#include "commands/args.hpp"
#include "msgs/deserialize.hpp"
#include "pcd/pcd_writer.hpp"

namespace fs = std::filesystem;

static void print_help() {
    std::cout << "usage: cal_viz unbag --input <file> [--output <dir>]\n\n"
                 "Extract every LiDAR scan as a .pcd and every camera frame as a .jpg.\n\n"
                 "options:\n"
                 "  --input   <file>   bag file to read (required)\n"
                 "  --output  <dir>    output directory (default: out)\n"
                 "  --help             show this message\n";
}

namespace commands {

int unbag(int argc, char** argv) {
    Args args;
    try {
        args = Args::parse(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        print_help();
        return 1;
    }

    if (args.has("help")) {
        print_help();
        return 0;
    }

    std::string input, output;
    try {
        input = args.require("input");
        output = args.get("output", "out");
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        print_help();
        return 1;
    }

    McapReader reader(input);

    // Print topic summary.
    auto topics = reader.topics();
    std::sort(topics.begin(), topics.end(), [](const TopicInfo& a, const TopicInfo& b) {
        return a.message_count > b.message_count;
    });

    std::cout << "\ntopics in " << input << ":\n\n" << std::left;
    for (const auto& t : topics)
        std::cout << "  " << std::setw(8) << t.message_count << std::setw(50) << t.topic
                  << t.msg_type << "\n";
    std::cout << "\n";

    uint64_t clouds_written = 0;
    uint64_t images_written = 0;
    uint64_t failed = 0;

    RawMessage msg;
    while (reader.next(msg)) {
        std::string rel = msg.topic.front() == '/' ? msg.topic.substr(1) : msg.topic;
        fs::path    out_base = fs::path(output) / rel;

        if (msg.msg_type.ends_with("PointCloud2")) {
            auto cloud = msgs::as_point_cloud(msg);
            if (!cloud) {
                ++failed;
                continue;
            }
            fs::path path = out_base / (std::to_string(msg.log_time_ns) + ".pcd");
            try {
                pcd::write(*cloud, path);
                ++clouds_written;
            } catch (...) {
                ++failed;
            }

        } else if (msg.msg_type.ends_with("Image") || msg.msg_type.ends_with("CompressedImage")) {
            auto img = msgs::as_image(msg);
            if (!img) {
                ++failed;
                continue;
            }
            fs::path path = out_base / (std::to_string(msg.log_time_ns) + ".jpg");
            fs::create_directories(path.parent_path());
            int ok = stbi_write_jpg(path.string().c_str(), static_cast<int>(img->width),
                                    static_cast<int>(img->height), 3, img->data.data(), 90);
            ok ? ++images_written : ++failed;
        }
    }

    std::cout << "point clouds written: " << clouds_written << "\n"
              << "images written:       " << images_written << "\n";
    if (failed) std::cout << "failed:               " << failed << "\n";
    std::cout << "output: " << output << "\n";

    return 0;
}

}  // namespace commands
