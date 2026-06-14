## Implementation Design

---

### Dependencies

Everything lives under `third_party/` and is statically compiled into the binary. There are no dynamic link dependencies and no host packages required beyond a C++20 compiler and CMake.

| Library | Why we need it | How it's included |
|---|---|---|
| **Eigen3** | Projection math requires reliable quaternion slerp, 3×3/3×4 matrix–vector multiply, and vector normalization. Rolling these by hand is error-prone (especially slerp edge cases near antipodal quaternions). Eigen is header-only so it adds zero link weight. | `FetchContent` — headers only, no build step |
| **stb_image / stb_image_write** | `sensor_msgs/CompressedImage` arrives as a raw JPEG byte blob. We need a decoder to get a pixel buffer before we can draw on it, and an encoder to write the projected output JPEGs. A JPEG codec is non-trivial to write from scratch. stb is two single-header files with no dependencies, making it the smallest possible footprint for this. | Vendored directly in `third_party/stb/` — just two `.h` files committed to the repo |
| **yaml-cpp** | We need to read user-supplied `extrinsics.yaml` (arbitrary structure, not a fixed binary format) and write `camera_info.yaml` / `transforms.yaml`. Hand-writing a YAML parser for the read path is fragile. yaml-cpp builds to a static archive with no transitive deps. | `FetchContent` — built from source as a static lib (`BUILD_SHARED_LIBS=OFF`) |

No external library is used for MCAP or ROS1 bag parsing — those remain raw byte parsing as required.

**third_party layout**

```
third_party/
├── stb/
│   ├── stb_image.h          # vendored — commit directly
│   └── stb_image_write.h
├── eigen/                   # populated by FetchContent at configure time
└── yaml-cpp/                # populated by FetchContent at configure time
```

`stb` headers are committed directly to the repo. Eigen and yaml-cpp are fetched and pinned to a specific git tag by CMake at configure time — no internet access needed after the first `cmake -B build`.

**CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.20)
project(cal_viz)

set(CMAKE_CXX_STANDARD 20)
set(BUILD_SHARED_LIBS OFF)   # force static for all FetchContent deps

include(FetchContent)

FetchContent_Declare(eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
)
FetchContent_Declare(yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG        0.8.0
)
FetchContent_MakeAvailable(eigen yaml-cpp)

add_executable(cal_viz
    src/main.cpp
    src/bag/mcap_reader.cpp
    src/bag/ros1_reader.cpp
    src/bag/factory.cpp
    src/msgs/deserialize.cpp
    src/tf/tf_buffer.cpp
    src/discovery.cpp
    src/projection.cpp
    src/colorize.cpp
    src/pcd_writer.cpp
    src/commands/unbag.cpp
    src/commands/project.cpp
)

target_include_directories(cal_viz PRIVATE src third_party)
target_link_libraries(cal_viz PRIVATE Eigen3::Eigen yaml-cpp::yaml-cpp)
```

---

### Source Layout

```
src/
├── bag/
│   ├── bag_reader.hpp        # abstract BagReader interface + RawMessage
│   ├── mcap_reader.hpp/.cpp  # MCAP format implementation
│   ├── ros1_reader.hpp/.cpp  # ROS1 .bag format implementation
│   └── factory.hpp/.cpp      # make_reader() — detects format from magic bytes
├── msgs/
│   ├── types.hpp             # all internal message structs
│   └── deserialize.hpp/.cpp  # ROS CDR → internal struct converters
├── tf/
│   └── tf_buffer.hpp/.cpp    # TF tree storage + interpolation
├── discovery.hpp/.cpp        # topic scan, type classification, pair matching
├── projection.hpp/.cpp       # SE3 transform + camera projection math
├── colorize.hpp/.cpp         # depth/intensity/doppler/height/ring → RGB
├── pcd_writer.hpp/.cpp       # binary PCD serialization (no lib)
├── commands/
│   ├── unbag.hpp/.cpp        # unbag subcommand
│   └── project.hpp/.cpp      # project subcommand
└── main.cpp
```

---

### Bag Reader Abstraction

The reader layer hides format differences behind a single pull-style interface.
Callers never see MCAP chunks or ROS1 index records — only `RawMessage`.

```cpp
struct RawMessage {
    std::string  topic;
    std::string  msg_type;      // e.g. "sensor_msgs/PointCloud2"
    uint64_t     log_time_ns;
    uint64_t     publish_time_ns;
    std::vector<uint8_t> data;  // serialized ROS CDR bytes
};

struct TopicInfo {
    std::string topic;
    std::string msg_type;
    uint64_t    message_count;
};

class BagReader {
public:
    virtual ~BagReader() = default;

    // Returns metadata for all topics without reading message data.
    virtual std::vector<TopicInfo> topics() = 0;

    // Advances to the next message. Returns false when exhausted.
    virtual bool next(RawMessage& out) = 0;

    // Seek back to the start (used during pair validation sampling).
    virtual void rewind() = 0;
};
```

`factory.hpp` reads the first few bytes of the file and returns the right subclass:

| Magic bytes | Format |
|---|---|
| `\x89MCAP0\r\n` (8 bytes) | MCAP |
| `#ROSBAG V2.0\n` (13 bytes) | ROS1 bag |

---

### Message Types

All internal types are format-agnostic. Deserializers in `msgs/deserialize.hpp`
convert raw CDR bytes from `RawMessage::data` into these structs.

#### Common header

```cpp
struct Header {
    uint64_t    stamp_ns;
    std::string frame_id;
};
```

#### Point cloud (LiDAR and radar)

Both LiDAR and radar use `sensor_msgs/PointCloud2`. Radar is distinguished at
the field level — it carries a `doppler` field; LiDAR does not. There is no
separate radar struct.

```cpp
struct PointField {
    std::string name;
    uint32_t    offset;    // byte offset within a single point record
    uint8_t     datatype;  // matches ROS constants: FLOAT32=7, FLOAT64=8, …
    uint32_t    count;
};

struct PointCloud {
    Header                  header;
    uint32_t                height;
    uint32_t                width;
    std::vector<PointField> fields;
    bool                    is_bigendian;
    uint32_t                point_step;  // bytes per point
    uint32_t                row_step;    // bytes per row (= width * point_step)
    std::vector<uint8_t>    data;
    bool                    is_dense;

    // Convenience accessors — look up field offset at construction time.
    size_t num_points() const { return height * width; }
    float  field_f32(size_t point_idx, const std::string& field_name) const;
    bool   has_field(const std::string& name) const;
    bool   is_radar() const { return has_field("doppler"); }
};
```

#### Image (raw)

```cpp
struct Image {
    Header               header;
    uint32_t             height;
    uint32_t             width;
    std::string          encoding;  // "rgb8", "bgr8", "mono8", "16UC1", …
    bool                 is_bigendian;
    uint32_t             step;      // bytes per row
    std::vector<uint8_t> data;
};
```

#### Compressed image

```cpp
struct CompressedImage {
    Header               header;
    std::string          format;  // "jpeg" | "png"
    std::vector<uint8_t> data;
};
```

Projection always works on a decoded pixel buffer. `CompressedImage` is decoded
to `Image` (via libjpeg-turbo / libpng stubs, or a minimal JFIF decoder) before
entering the projection pipeline.

#### Camera intrinsics

```cpp
struct CameraInfo {
    Header               header;
    uint32_t             height;
    uint32_t             width;
    std::string          distortion_model;  // "plumb_bob" | "equidistant"
    std::vector<double>  D;                 // distortion coefficients (length varies)
    std::array<double,9>  K;               // 3×3 intrinsic matrix (row-major)
    std::array<double,9>  R;               // 3×3 rectification matrix
    std::array<double,12> P;              // 3×4 projection matrix
};
```

---

### TF Storage

TF data arrives as a stream of `tf2_msgs/TFMessage` messages on `/tf` and
`/tf_static`. Each message contains one or more `geometry_msgs/TransformStamped`
records. Static transforms are stored once and never expire; dynamic transforms
are buffered as time-sorted sequences and interpolated on lookup.

#### Primitives

```cpp
struct Vec3 {
    double x, y, z;
};

struct Quat {
    double x, y, z, w;
};

// A single SE3 transform: p_parent = R * p_child + t
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

#### TfBuffer

The buffer maintains one `std::map` keyed by `(parent_frame, child_frame)`.
Each entry holds a `std::vector<StampedTransform>` sorted by `stamp_ns`.

`lookup()` finds the path between two arbitrary frames by BFS over the frame
graph, then chains the per-edge transforms together. Edge transforms are
interpolated at the requested timestamp using `lerp` for translation and `slerp`
for rotation.

```cpp
class TfBuffer {
public:
    void add_dynamic(const StampedTransform& tf);
    void add_static(const StampedTransform& tf);  // stored at stamp=0, never expires

    // Returns the transform that maps a point in `from_frame` to `to_frame`
    // at the given timestamp. Returns nullopt if no path exists or the
    // timestamp is out of range.
    std::optional<Transform> lookup(
        const std::string& from_frame,
        const std::string& to_frame,
        uint64_t           stamp_ns
    ) const;

    bool can_transform(const std::string& from_frame, const std::string& to_frame) const;

private:
    // edge key: {parent, child}
    using EdgeKey = std::pair<std::string, std::string>;
    std::map<EdgeKey, std::vector<StampedTransform>> dynamic_;
    std::map<EdgeKey, StampedTransform>               static_;

    std::optional<std::vector<EdgeKey>> bfs_path(
        const std::string& from, const std::string& to) const;

    Transform interpolate(const std::vector<StampedTransform>& seq, uint64_t t) const;
    Transform chain(const std::vector<Transform>& transforms) const;
};
```

---

### Topic Discovery

```cpp
enum class SensorType { LiDAR, Camera, CompressedCamera, CameraInfo, TF, TFStatic, Unknown };

// Classify a msg_type string into a SensorType.
SensorType classify(const std::string& msg_type);

// A camera image topic paired with its CameraInfo topic.
struct CameraSource {
    std::string  image_topic;
    std::string  camera_info_topic;  // matched by naming convention or frame_id
    SensorType   image_type;         // Camera or CompressedCamera
};

// A validated LiDAR × camera pair ready for projection.
struct ProjectionPair {
    std::string  lidar_topic;
    CameraSource camera;
    float        overlap_pct;     // measured during validation sampling
};

struct DiscoveryResult {
    std::vector<std::string>     lidar_topics;
    std::vector<CameraSource>    camera_sources;
    std::vector<ProjectionPair>  valid_pairs;
    std::vector<std::string>     warnings;  // skipped pairs with reasons
};

DiscoveryResult discover(BagReader& reader, const TfBuffer& tf, float overlap_threshold);
```

CameraInfo matching order:
1. Replace `image`/`color` with `camera_info` in the topic name.
2. Strip the last path component and append `camera_info`.
3. Match on `frame_id` from the first message on each candidate topic.

---

### Projection Pipeline

```cpp
// A single LiDAR point projected into image space.
struct ProjectedPoint {
    int     u, v;       // pixel coordinates
    float   depth;      // distance from camera origin (metres)
    float   intensity;  // raw LiDAR return strength (NaN if unavailable)
    float   doppler;    // m/s, positive = receding (NaN if not radar)
    float   height;     // Z in world frame (metres)
    int32_t ring;       // beam/ring ID (-1 if unavailable)
};

// Projects all points in `cloud` into the image plane defined by `cam_info`,
// using `lidar_to_cam` as the extrinsic SE3 transform.
// Points behind the camera or outside [0,W)×[0,H) are dropped.
std::vector<ProjectedPoint> project(
    const PointCloud&  cloud,
    const Transform&   lidar_to_cam,
    const CameraInfo&  cam_info
);
```

Distortion is applied after the linear projection. Two models are supported,
selected via `CameraInfo::distortion_model`:

- `plumb_bob` — pinhole + radial-tangential (`k1,k2,p1,p2,k3`)
- `equidistant` — fisheye (`k1,k2,k3,k4`)

---

### Colorization

```cpp
enum class ColorizeMode { Depth, Intensity, Doppler, Height, Ring };

// Maps a single ProjectedPoint scalar to an RGB triple using a fixed colormap.
// `range` is the [min, max] used for normalisation (computed per-frame).
std::array<uint8_t,3> colorize(const ProjectedPoint& pt, ColorizeMode mode,
                               float range_min, float range_max);
```

Colormaps:
- **Depth / Height / Ring** — Turbo (perceptually uniform, works for both)
- **Intensity** — Grayscale
- **Doppler** — Diverging blue→white→red (zero = white)

---

### PCD Writer

Binary PCD (little-endian) is written without any external library.

```cpp
void write_pcd(const PointCloud& cloud, const std::string& path);
```

The file header is generated from `PointCloud::fields`. Only `FLOAT32` and
`FLOAT64` field types are written; integer fields are cast to `FLOAT32`.

---

### Data Flow Summary

```
BagReader::next()
    │
    ├─ /tf, /tf_static ──────────────────────► TfBuffer::add_*()
    │
    └─ sensor topics
           │
           ├─ [unbag command]
           │      ├─ PointCloud ──► write_pcd()
           │      └─ Image / CompressedImage ──► write jpg
           │
           └─ [project command]
                  │
                  ├─ discover() ──► valid ProjectionPairs
                  │
                  └─ per pair, per frame:
                         TfBuffer::lookup()
                             │
                         project()
                             │
                         colorize()
                             │
                         render overlay ──► write jpg
```
