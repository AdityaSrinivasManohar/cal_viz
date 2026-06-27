#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace msgs {

struct Timestamp {
    int32_t  sec;
    uint32_t nsec;
};

struct BaseType {
    Timestamp   timestamp;
    std::string frame_id;
};

struct Point {
    float x;
    float y;
    float z;
    float intensity;  // NaN if the source cloud has no intensity field
};

struct PointCloud : BaseType {
    std::vector<Point> points;
};

// Decoded pixel buffer — always RGB8 (3 bytes per pixel, row-major).
// The deserializer handles decoding from both raw and compressed formats.
struct Image : BaseType {
    uint32_t             height;
    uint32_t             width;
    std::vector<uint8_t> data;  // height * width * 3 bytes
};

struct CameraInfo : BaseType {
    uint32_t               height;
    uint32_t               width;
    std::string            distortion_model;  // "plumb_bob" | "equidistant"
    std::vector<double>    D;                 // distortion coefficients (length varies)
    std::array<double, 9>  K;                 // 3×3 intrinsic matrix (row-major)
    std::array<double, 9>  R;                 // 3×3 rectification matrix
    std::array<double, 12> P;                 // 3×4 projection matrix
};

}  // namespace msgs
