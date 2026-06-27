#include "msgs/deserialize.hpp"

#include <stb/stb_image.h>

#include <cassert>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

// ── CDR reader ────────────────────────────────────────────────────────────────
//
// ROS CDR (little-endian) layout rules:
//   - 4-byte encapsulation header at byte 0: always 0x00 0x01 0x00 0x00
//   - Every field is aligned to its own size from the start of the buffer
//   - Strings: uint32 length (incl. null terminator) + bytes + '\0'
//   - Sequences: uint32 count + elements (each individually aligned)
//   - Fixed arrays: elements back-to-back, first element aligned, rest follow naturally

namespace {

class CdrReader {
public:
    explicit CdrReader(const std::vector<uint8_t>& buf)
        : data_(buf.data()), size_(buf.size()), pos_(4) {}  // skip encapsulation header

    bool ok() const { return !bad_ && pos_ <= size_; }

    // Return a raw pointer to the current position (e.g. for zero-copy point data).
    const uint8_t* raw_ptr() const { return data_ + pos_; }
    size_t         pos() const { return pos_; }

    void skip(size_t n) { pos_ += n; }

    uint8_t u8() {
        check(1);
        return data_[pos_++];
    }

    bool boolean() { return u8() != 0; }

    int32_t i32() {
        align(4);
        check(4);
        int32_t v;
        std::memcpy(&v, data_ + pos_, 4);
        pos_ += 4;
        return v;
    }

    uint32_t u32() {
        align(4);
        check(4);
        uint32_t v;
        std::memcpy(&v, data_ + pos_, 4);
        pos_ += 4;
        return v;
    }

    float f32() {
        align(4);
        check(4);
        float v;
        std::memcpy(&v, data_ + pos_, 4);
        pos_ += 4;
        return v;
    }

    double f64() {
        align(8);
        check(8);
        double v;
        std::memcpy(&v, data_ + pos_, 8);
        pos_ += 8;
        return v;
    }

    // uint32 length (incl. null) + bytes + null
    std::string str() {
        uint32_t len = u32();
        if (len == 0) return {};
        check(len);
        std::string s(reinterpret_cast<const char*>(data_ + pos_), len - 1);
        pos_ += len;
        return s;
    }

private:
    const uint8_t* data_;
    size_t         size_;
    size_t         pos_;
    bool           bad_{false};

    // CDR alignment is relative to the start of the data section (byte 4,
    // after the 4-byte encapsulation header), not from the buffer start.
    void align(size_t n) {
        size_t data_pos = pos_ - 4;
        pos_            = ((data_pos + n - 1) & ~(n - 1)) + 4;
    }

    void check(size_t n) {
        if (pos_ + n > size_) {
            bad_ = true;
            throw std::runtime_error("CDR: buffer overrun");
        }
    }
};

// Read the common std_msgs/Header fields into a BaseType.
msgs::BaseType read_header(CdrReader& r) {
    msgs::BaseType base;
    base.timestamp = {r.i32(), r.u32()};
    base.frame_id  = r.str();
    return base;
}

// Extract a float value from a raw point record at the given byte offset.
// Only FLOAT32 (datatype=7) and FLOAT64 (datatype=8) are supported.
float extract_field(const uint8_t* point, uint32_t offset, uint8_t datatype) {
    if (datatype == 7) {  // FLOAT32
        float v;
        std::memcpy(&v, point + offset, 4);
        return v;
    }
    if (datatype == 8) {  // FLOAT64
        double v;
        std::memcpy(&v, point + offset, 8);
        return static_cast<float>(v);
    }
    return std::numeric_limits<float>::quiet_NaN();
}

}  // namespace

// ── public deserializers ──────────────────────────────────────────────────────

namespace msgs {

std::optional<PointCloud> as_point_cloud(const RawMessage& msg) {
    if (!msg.msg_type.ends_with("PointCloud2")) return std::nullopt;

    try {
        CdrReader r(msg.data);

        PointCloud cloud;
        static_cast<msgs::BaseType&>(cloud) = read_header(r);

        uint32_t height = r.u32();
        uint32_t width = r.u32();
        uint32_t num_points = height * width;

        // PointField[]: name, offset, datatype, count
        struct FieldDef {
            std::string name;
            uint32_t    offset;
            uint8_t     datatype;
        };
        uint32_t              nfields = r.u32();
        std::vector<FieldDef> fields(nfields);
        for (auto& f : fields) {
            f.name = r.str();
            f.offset = r.u32();
            f.datatype = r.u8();
            r.u32();  // count — always 1 for sensor data
        }

        r.boolean();  // is_bigendian
        uint32_t point_step = r.u32();
        r.u32();  // row_step

        uint32_t       data_len = r.u32();
        const uint8_t* raw_pts = r.raw_ptr();
        r.skip(data_len);

        // Look up field offsets by name.
        auto find = [&](const std::string& name) -> const FieldDef* {
            for (auto& f : fields)
                if (f.name == name) return &f;
            return nullptr;
        };

        const auto* fx = find("x");
        const auto* fy = find("y");
        const auto* fz = find("z");
        const auto* fi = find("intensity");

        if (!fx || !fy || !fz) return std::nullopt;

        cloud.points.resize(num_points);
        for (uint32_t i = 0; i < num_points; ++i) {
            const uint8_t* p = raw_pts + i * point_step;
            cloud.points[i].x = extract_field(p, fx->offset, fx->datatype);
            cloud.points[i].y = extract_field(p, fy->offset, fy->datatype);
            cloud.points[i].z = extract_field(p, fz->offset, fz->datatype);
            cloud.points[i].intensity = fi ? extract_field(p, fi->offset, fi->datatype)
                                           : std::numeric_limits<float>::quiet_NaN();
        }

        return cloud;
    } catch (...) {
        return std::nullopt;
    }
}

std::optional<Image> as_image(const RawMessage& msg) {
    // ── CompressedImage ───────────────────────────────────────────────────────
    if (msg.msg_type.ends_with("CompressedImage")) {
        try {
            CdrReader r(msg.data);
            Image     img;
            static_cast<msgs::BaseType&>(img) = read_header(r);
            r.str();  // format ("jpeg" / "png") — stb detects automatically

            uint32_t compressed_len = r.u32();

            int      w = 0, h = 0, ch = 0;
            uint8_t* pixels = stbi_load_from_memory(r.raw_ptr(), static_cast<int>(compressed_len),
                                                    &w, &h, &ch, 3);  // force RGB output

            if (!pixels) return std::nullopt;

            img.width = static_cast<uint32_t>(w);
            img.height = static_cast<uint32_t>(h);
            img.data.assign(pixels, pixels + w * h * 3);
            stbi_image_free(pixels);
            return img;
        } catch (...) {
            return std::nullopt;
        }
    }

    // ── Raw Image ─────────────────────────────────────────────────────────────
    if (!msg.msg_type.ends_with("Image")) return std::nullopt;

    try {
        CdrReader r(msg.data);
        Image     img;
        static_cast<msgs::BaseType&>(img) = read_header(r);

        img.height = r.u32();
        img.width = r.u32();
        std::string encoding = r.str();
        r.boolean();  // is_bigendian
        uint32_t       step = r.u32();
        uint32_t       data_len = r.u32();
        const uint8_t* raw = r.raw_ptr();

        img.data.resize(img.height * img.width * 3);
        uint8_t* dst = img.data.data();

        if (encoding == "rgb8") {
            for (uint32_t row = 0; row < img.height; ++row)
                std::memcpy(dst + row * img.width * 3, raw + row * step, img.width * 3);

        } else if (encoding == "bgr8") {
            for (uint32_t row = 0; row < img.height; ++row) {
                const uint8_t* src = raw + row * step;
                uint8_t*       out = dst + row * img.width * 3;
                for (uint32_t col = 0; col < img.width; ++col) {
                    out[col * 3 + 0] = src[col * 3 + 2];
                    out[col * 3 + 1] = src[col * 3 + 1];
                    out[col * 3 + 2] = src[col * 3 + 0];
                }
            }

        } else if (encoding == "mono8") {
            for (uint32_t row = 0; row < img.height; ++row) {
                const uint8_t* src = raw + row * step;
                uint8_t*       out = dst + row * img.width * 3;
                for (uint32_t col = 0; col < img.width; ++col)
                    out[col * 3 + 0] = out[col * 3 + 1] = out[col * 3 + 2] = src[col];
            }

        } else if (encoding == "16UC1") {
            // Depth images: take the high byte of each uint16 as the 8-bit value.
            for (uint32_t row = 0; row < img.height; ++row) {
                const uint8_t* src = raw + row * step;
                uint8_t*       out = dst + row * img.width * 3;
                for (uint32_t col = 0; col < img.width; ++col) {
                    uint16_t val;
                    std::memcpy(&val, src + col * 2, 2);
                    uint8_t v8 = static_cast<uint8_t>(val >> 8);
                    out[col * 3 + 0] = out[col * 3 + 1] = out[col * 3 + 2] = v8;
                }
            }

        } else {
            return std::nullopt;  // unsupported encoding
        }

        return img;
    } catch (...) {
        return std::nullopt;
    }
}

std::optional<CameraInfo> as_camera_info(const RawMessage& msg) {
    if (!msg.msg_type.ends_with("CameraInfo")) return std::nullopt;

    try {
        CdrReader  r(msg.data);
        CameraInfo info;
        static_cast<msgs::BaseType&>(info) = read_header(r);
        info.height = r.u32();
        info.width = r.u32();
        info.distortion_model = r.str();

        // D — variable-length sequence of float64
        uint32_t d_len = r.u32();
        info.D.resize(d_len);
        for (double& v : info.D) v = r.f64();

        // K, R, P — fixed-size arrays of float64
        for (double& v : info.K) v = r.f64();
        for (double& v : info.R) v = r.f64();
        for (double& v : info.P) v = r.f64();

        return info;
    } catch (...) {
        return std::nullopt;
    }
}

std::vector<StampedTransform> as_tf_message(const RawMessage& msg) {
    if (!msg.msg_type.ends_with("TFMessage")) return {};

    // tf2_msgs/TFMessage CDR layout:
    //   uint32 count
    //   for each TransformStamped:
    //     Header:  stamp.sec (int32), stamp.nanosec (uint32), frame_id (string)
    //     string   child_frame_id
    //     Vector3: translation.x/y/z (float64 × 3)
    //     Quaternion: rotation.x/y/z/w (float64 × 4)

    try {
        CdrReader r(msg.data);

        uint32_t                      count = r.u32();
        std::vector<StampedTransform> result;
        result.reserve(count);

        for (uint32_t i = 0; i < count; ++i) {
            int32_t  sec     = r.i32();
            uint32_t nanosec = r.u32();
            auto     parent  = r.str();
            auto     child   = r.str();

            double tx = r.f64(), ty = r.f64(), tz = r.f64();
            double qx = r.f64(), qy = r.f64(), qz = r.f64(), qw = r.f64();

            result.push_back({
                static_cast<uint64_t>(sec) * 1'000'000'000ULL + nanosec,
                std::move(parent),
                std::move(child),
                {{tx, ty, tz}, {qx, qy, qz, qw}},
            });
        }

        return result;
    } catch (...) {
        return {};
    }
}

}  // namespace msgs
