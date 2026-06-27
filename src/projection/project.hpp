#pragma once

#include <array>
#include <vector>

#include "msgs/types.hpp"
#include "tf/tf_buffer.hpp"

namespace projection {

struct ProjectedPoint {
    int   u, v;        // pixel coordinates
    float depth;       // distance from camera origin (metres)
    float intensity;   // raw LiDAR return (NaN if unavailable)
};

enum class ColorizeMode { Depth, Intensity };

// Projects points from `cloud` into the image plane of `cam_info` using
// `lidar_to_cam` as the extrinsic SE3.  Points behind the camera or outside
// the image bounds are dropped.
std::vector<ProjectedPoint> project(const msgs::PointCloud& cloud,
                                    const Transform&        lidar_to_cam,
                                    const msgs::CameraInfo& cam_info);

// Maps a single ProjectedPoint scalar to an RGB triple.
// `range_min` / `range_max` define the normalization window.
std::array<uint8_t, 3> colorize(const ProjectedPoint& pt, ColorizeMode mode,
                                 float range_min, float range_max);

// Overlays `points` onto `image` in-place using `mode` for coloring.
// `dot_radius` controls the size of each rendered dot (pixels).
void render(msgs::Image& image, const std::vector<ProjectedPoint>& points,
            ColorizeMode mode, int dot_radius = 2);

}  // namespace projection
