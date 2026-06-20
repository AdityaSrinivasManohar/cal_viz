#include "bag/mcap_reader.hpp"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: cal_viz <file.mcap>\n";
        return 1;
    }

    McapReader reader(argv[1]);

    auto topics = reader.topics();

    // Sort by message count descending for readability.
    std::sort(topics.begin(), topics.end(),
              [](const TopicInfo& a, const TopicInfo& b) {
                  return a.message_count > b.message_count;
              });

    std::cout << "topics in " << argv[1] << ":\n\n";
    std::cout << std::left
              << std::setw(8)  << "msgs"
              << std::setw(40) << "topic"
              << "type\n";
    std::cout << std::string(80, '-') << "\n";

    uint64_t total = 0;
    for (const auto& t : topics) {
        std::cout << std::left
                  << std::setw(8)  << t.message_count
                  << std::setw(40) << t.topic
                  << t.msg_type << "\n";
        total += t.message_count;
    }

    std::cout << "\n" << total << " messages across " << topics.size() << " topics\n";
    return 0;
}
