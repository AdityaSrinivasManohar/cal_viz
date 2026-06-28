## Implementation Design

---

### Dependencies

Everything is statically compiled into the binary. There are no dynamic link dependencies and no host packages required beyond a C++20 compiler and CMake.

| Library | Why we need it | How it's included |
|---|---|---|
| **Eigen3** | Projection math requires reliable quaternion slerp, 3×3/3×4 matrix–vector multiply, and vector normalization. Rolling these by hand is error-prone (especially slerp edge cases near antipodal quaternions). Eigen is header-only so it adds zero link weight. | `FetchContent` — headers only, no build step |
| **stb_image / stb_image_write** | `sensor_msgs/CompressedImage` arrives as a raw JPEG byte blob. We need a decoder to get a pixel buffer before we can draw on it, and an encoder to write the projected output JPEGs. stb is two single-header files with no dependencies. | Vendored in `third_party/stb/` — committed directly to the repo |
| **mcap** | The MCAP format has enough edge cases (chunk compression, CRC validation, summary-section indexing) that a hand-rolled parser becomes a maintenance burden. The official Foxglove C++ library is header-only and handles all of these transparently. | `FetchContent` — headers only, `EXCLUDE_FROM_ALL` |

**third_party layout**

```
third_party/
└── stb/
    ├── stb_image.h
    └── stb_image_write.h
```

stb headers are committed directly to the repo. Eigen and mcap are fetched and pinned to specific git tags by CMake at configure time.

**CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 4.3)
project(cal_viz CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)

FetchContent_Declare(eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        5.0.1
    GIT_SHALLOW    TRUE
)
FetchContent_Declare(mcap
    GIT_REPOSITORY https://github.com/foxglove/mcap.git
    GIT_TAG        releases/cpp/v2.1.3
    GIT_SHALLOW    TRUE
    EXCLUDE_FROM_ALL
)
FetchContent_MakeAvailable(eigen mcap)

add_executable(cal_viz
    src/main.cpp
    src/stb_impl.cpp
    src/bag/mcap_reader.cpp
    src/msgs/deserialize.cpp
    src/tf/tf_buffer.cpp
    src/projection/project.cpp
    src/commands/unbag.cpp
    src/commands/project.cpp
)

target_include_directories(cal_viz PRIVATE src third_party ${mcap_SOURCE_DIR}/cpp/mcap/include)
target_compile_definitions(cal_viz PRIVATE MCAP_COMPRESSION_NO_LZ4)
target_link_libraries(cal_viz PRIVATE Eigen3::Eigen zstd)
```

---

### Source Layout

```
src/
├── bag/
│   ├── bag_reader.hpp        # abstract BagReader interface + RawMessage
│   └── mcap_reader.hpp/.cpp  # MCAP format implementation
├── msgs/
│   ├── types.hpp             # all internal message structs
│   └── deserialize.hpp/.cpp  # ROS CDR → internal struct converters
├── tf/
│   └── tf_buffer.hpp/.cpp    # TF tree storage + BFS lookup + slerp interpolation
├── projection/
│   └── project.hpp/.cpp      # SE3 transform + camera projection + colorization
├── pcd/
│   └── pcd_writer.hpp        # ASCII PCD serialization (header-only)
├── commands/
│   ├── args.hpp              # minimal flag parser
│   ├── unbag.hpp/.cpp        # unbag subcommand
│   └── project.hpp/.cpp      # project subcommand
├── stb_impl.cpp              # single TU that defines STB_IMAGE_IMPLEMENTATION
└── main.cpp                  # subcommand dispatcher
```

---

### Bag Reader Abstraction

The reader layer hides format differences behind a single pull-style interface.
Callers never see MCAP chunks or index records — only `RawMessage`.

```cpp
struct RawMessage {
    std::string          topic;
    std::string          msg_type;        // e.g. "sensor_msgs/PointCloud2"
    uint64_t             log_time_ns;
    uint64_t             publish_time_ns;
    std::vector<uint8_t> data;            // serialized ROS CDR bytes
};

struct TopicInfo {
    std::string topic;
    std::string msg_type;
    uint64_t    message_count;
};

class BagReader {
public:
    virtual ~BagReader() = default;
    virtual std::vector<TopicInfo> topics() = 0;
    virtual bool next(RawMessage& out) = 0;
    virtual void rewind() = 0;
};
```

---

### Message Types

All internal types are format-agnostic. Deserializers in `msgs/deserialize.hpp`
convert raw CDR bytes from `RawMessage::data` into these structs.

#### Common header

```cpp
struct BaseType {
    Timestamp   timestamp;  // { int32_t sec, uint32_t nsec }
    std::string frame_id;
};
```

#### Point cloud

```cpp
struct Point {
    float x, y, z;
    float intensity;  // NaN if the source cloud has no intensity field
};

struct PointCloud : BaseType {
    std::vector<Point> points;
};
```

#### Image

Decoded pixel buffer — always RGB8 (3 bytes per pixel, row-major).
The deserializer handles decoding from both raw (`sensor_msgs/Image`) and
compressed (`sensor_msgs/CompressedImage`) formats via stb_image.

```cpp
struct Image : BaseType {
    uint32_t             height;
    uint32_t             width;
    std::vector<uint8_t> data;  // height * width * 3 bytes
};
```

#### Camera intrinsics

```cpp
struct CameraInfo : BaseType {
    uint32_t               height;
    uint32_t               width;
    std::string            distortion_model;  // "plumb_bob" | "equidistant"
    std::vector<double>    D;                 // distortion coefficients
    std::array<double, 9>  K;                 // 3×3 intrinsic matrix (row-major)
    std::array<double, 9>  R;                 // 3×3 rectification matrix
    std::array<double, 12> P;                 // 3×4 projection matrix
};
```

---

### TF Storage

TF data arrives as a stream of `tf2_msgs/TFMessage` messages on `/tf` and
`/tf_static`. Static transforms are stored once; dynamic transforms are
buffered as time-sorted sequences and interpolated on lookup.

#### Primitives

```cpp
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
```

The stored transform follows the ROS TF convention:
`p_parent = R * p_child + t`

#### TfBuffer

```cpp
class TfBuffer {
public:
    void add_dynamic(const StampedTransform& tf);
    void add_static(const StampedTransform& tf);

    // Returns the transform mapping a point FROM from_frame TO to_frame.
    std::optional<Transform> lookup(
        const std::string& from_frame,
        const std::string& to_frame,
        uint64_t           stamp_ns
    ) const;

    bool can_transform(const std::string& from, const std::string& to) const;
    std::vector<std::string> frames() const;

private:
    using EdgeKey = std::pair<std::string, std::string>;
    std::map<EdgeKey, StampedTransform>              static_;
    std::map<EdgeKey, std::vector<StampedTransform>> dynamic_;
};
```

`lookup()` finds the path between two frames by BFS over the undirected frame
graph, then chains the per-edge transforms. Key lookup direction rule:

- `static_[{P, C}]` maps FROM child C TO parent P (`p_P = R * p_C + t`)
- For a BFS edge `a → b`: if we find key `{a, b}` we take its **inverse**;
  if we find key `{b, a}` we use it **directly**.

---

### Projection Pipeline

```cpp
struct ProjectedPoint {
    int   u, v;      // pixel coordinates
    float depth;     // distance from camera origin (metres)
    float intensity; // raw LiDAR return (NaN if unavailable)
};

enum class ColorizeMode { Depth, Intensity };

std::vector<ProjectedPoint> project(
    const msgs::PointCloud& cloud,
    const Transform&        lidar_to_cam,
    const msgs::CameraInfo& cam_info
);

std::array<uint8_t, 3> colorize(const ProjectedPoint& pt, ColorizeMode mode,
                                 float range_min, float range_max);

void render(msgs::Image& image, const std::vector<ProjectedPoint>& points,
            ColorizeMode mode, int dot_radius = 2);
```

Distortion is applied after the linear projection:

- `plumb_bob` — pinhole + radial-tangential (`k1, k2, p1, p2, k3`)
- `equidistant` — fisheye (`k1, k2, k3, k4`)

If all D coefficients are zero, the 3×4 P matrix is used directly (rectified image path).

Colormaps:
- **Depth** — Turbo rainbow (red = near, blue = far)
- **Intensity** — Grayscale

---

### PCD Writer

ASCII PCD v0.7 is written without any external library (`pcd/pcd_writer.hpp`, header-only).

```cpp
void pcd::write(const msgs::PointCloud& cloud, const std::filesystem::path& path);
```

Points with non-finite x/y/z are skipped. NaN intensity is written as 0.
The header is generated in a single pass after counting valid points.

---

### Data Flow

```
BagReader::next()
    │
    ├─ /tf, /tf_static ──────────────────────► TfBuffer::add_*()
    │
    └─ sensor topics
           │
           ├─ [unbag command]
           │      ├─ PointCloud2 ──► pcd::write()
           │      └─ Image / CompressedImage ──► stbi_write_jpg()
           │
           └─ [project command]
                  │
                  ├─ pass 1: collect TF + CameraInfo + PointClouds
                  │
                  └─ pass 2: per image frame
                         TfBuffer::lookup(cloud.frame_id, image.frame_id)
                             │
                         projection::project()
                             │
                         projection::render()
                             │
                         stbi_write_jpg()
```
