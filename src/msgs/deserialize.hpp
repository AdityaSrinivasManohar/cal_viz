#pragma once

#include <optional>
#include <vector>

#include "bag/bag_reader.hpp"
#include "msgs/types.hpp"
#include "tf/tf_buffer.hpp"

namespace msgs {

// Each function returns nullopt if msg_type doesn't match or the CDR is malformed.

std::optional<PointCloud> as_point_cloud(const RawMessage& msg);

// Handles both sensor_msgs/Image and sensor_msgs/CompressedImage.
// Output is always RGB8 regardless of source encoding.
std::optional<Image> as_image(const RawMessage& msg);

std::optional<CameraInfo> as_camera_info(const RawMessage& msg);

// Handles tf2_msgs/TFMessage (used for both /tf and /tf_static).
// Returns one StampedTransform per geometry_msgs/TransformStamped in the message.
std::vector<StampedTransform> as_tf_message(const RawMessage& msg);

}  // namespace msgs
