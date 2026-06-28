#pragma once

#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#include "msgs/types.hpp"

namespace pcd {

// Writes `cloud` to `path` in ASCII PCD v0.7 format.
// Points with non-finite x/y/z are skipped; NaN intensity is written as 0.
inline void write(const msgs::PointCloud& cloud, const std::filesystem::path& path) {
    // Count valid points first so the header can be written in one pass.
    uint32_t n = 0;
    for (const auto& p : cloud.points)
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) ++n;

    std::filesystem::create_directories(path.parent_path());
    std::ofstream f(path);
    if (!f) throw std::runtime_error("pcd::write: cannot open " + path.string());

    f << "# .PCD v0.7 - Point Cloud Data file format\n"
      << "VERSION 0.7\n"
      << "FIELDS x y z intensity\n"
      << "SIZE 4 4 4 4\n"
      << "TYPE F F F F\n"
      << "COUNT 1 1 1 1\n"
      << "WIDTH " << n << "\n"
      << "HEIGHT 1\n"
      << "VIEWPOINT 0 0 0 1 0 0 0\n"
      << "POINTS " << n << "\n"
      << "DATA ascii\n";

    f << std::fixed;
    for (const auto& p : cloud.points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        float intensity = std::isfinite(p.intensity) ? p.intensity : 0.f;
        f << p.x << ' ' << p.y << ' ' << p.z << ' ' << intensity << '\n';
    }
}

}  // namespace pcd
