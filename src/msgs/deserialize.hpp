#pragma once

#include <optional>

#include "bag/bag_reader.hpp"
#include "msgs/types.hpp"

namespace msgs {

// Each function returns nullopt if msg_type doesn't match or the CDR is malformed.

std::optional<PointCloud> as_point_cloud(const RawMessage& msg);

// Handles both sensor_msgs/Image and sensor_msgs/CompressedImage.
// Output is always RGB8 regardless of source encoding.
std::optional<Image> as_image(const RawMessage& msg);

std::optional<CameraInfo> as_camera_info(const RawMessage& msg);

}  // namespace msgs
