#include <iostream>
#include <string_view>

#include "commands/project.hpp"
#include "commands/unbag.hpp"

static void print_usage() {
    std::cout << "usage: cal_viz <command> [options]\n\n"
                 "commands:\n"
                 "  unbag    extract LiDAR scans (.pcd) and camera frames (.jpg) from a bag\n"
                 "  project  project LiDAR point clouds onto camera images\n\n"
                 "run 'cal_viz <command> --help' for command-specific options\n";
}

int main(int argc, char** argv) {
    if (argc < 2) {
        print_usage();
        return 1;
    }

    std::string_view cmd = argv[1];
    if (cmd == "unbag") return commands::unbag(argc - 1, argv + 1);
    if (cmd == "project") return commands::project(argc - 1, argv + 1);

    std::cerr << "unknown command: " << cmd << "\n\n";
    print_usage();
    return 1;
}
