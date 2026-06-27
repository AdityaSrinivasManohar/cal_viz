#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

struct Vec3 {
    double x, y, z;
};

struct Quat {
    double x, y, z, w;
};

// SE3 transform: p_parent = R * p_child + t
struct Transform {
    Vec3 translation;
    Quat rotation;
};

struct StampedTransform {
    uint64_t    stamp_ns;
    std::string parent_frame;
    std::string child_frame;
    Transform   tf;
};

class TfBuffer {
public:
    void add_dynamic(const StampedTransform& tf);
    void add_static(const StampedTransform& tf);

    // Returns the transform mapping a point from `from_frame` into `to_frame`
    // at the given timestamp. Returns nullopt if no path exists or the timestamp
    // falls outside the buffered range.
    std::optional<Transform> lookup(const std::string& from_frame, const std::string& to_frame,
                                    uint64_t stamp_ns) const;

    bool can_transform(const std::string& from_frame, const std::string& to_frame) const;

    // All unique frame ids known to the buffer.
    std::vector<std::string> frames() const;

private:
    using EdgeKey = std::pair<std::string, std::string>;

    std::map<EdgeKey, StampedTransform>              static_;
    std::map<EdgeKey, std::vector<StampedTransform>> dynamic_;

    // Returns the sequence of frame ids from `from` to `to`, or nullopt.
    std::optional<std::vector<std::string>> bfs_path(const std::string& from,
                                                     const std::string& to) const;

    Transform interpolate(const std::vector<StampedTransform>& seq, uint64_t stamp_ns) const;
};
