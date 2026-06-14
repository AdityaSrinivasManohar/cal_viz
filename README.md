# cal_viz

Projects LiDAR and radar point clouds onto camera images from `.mcap` and ROS1 `.bag` files. No external bag parsing libraries — formats are read with raw byte parsing.

## Commands

### `unbag` — extract raw sensor data

Dumps every LiDAR scan as a binary `.pcd` and every camera frame as a `.jpg`, deduplicated per topic. Run this once per bag; the output can be used independently of projection.

```bash
cal_viz unbag \
  --input  drive.bag \
  --output ./out
```

Output layout:

```
out/
├── lidar/<topic>/<timestamp_ns>.pcd
├── cameras/<topic>/<timestamp_ns>.jpg
├── cameras/<topic>/camera_info.yaml
└── timestamps.csv
```

### `project` — project LiDAR onto camera frames

Discovers all valid LiDAR × camera pairs automatically, validates them (TF path, timestamp overlap, projection overlap), and writes one projected `.jpg` per frame per pair.

```bash
cal_viz project \
  --input              drive.bag  \
  --output             ./out      \
  --colorize           depth      \   # depth | intensity | doppler | height | ring
  --point-size         3          \   # pixels, default 3
  --overlap-threshold  0.05       \   # skip pairs below this, default 5%
  --extrinsics         ext.yaml   \   # optional: override TF with a yaml file
  --lidar              /lidar_front \  # optional: pin to a specific topic
  --camera             /cam_front     # optional: pin to a specific topic
```

Output layout:

```
out/
└── <lidar_topic>__<camera_topic>/
    ├── projected/<timestamp_ns>.jpg
    └── transforms.yaml
```

## Build

Requires CMake ≥ 3.20 and a C++20 compiler. All dependencies are fetched and statically linked automatically — no apt installs needed.

```bash
cmake -B build && cmake --build build
```

The binary is at `./build/cal_viz`.

On first configure, CMake fetches:
- [Eigen 3.4.0](https://gitlab.com/libeigen/eigen.git) — matrix/quaternion math for projection and TF interpolation
- [yaml-cpp 0.8.0](https://github.com/jbeder/yaml-cpp) — reading `extrinsics.yaml`, writing `camera_info.yaml` / `transforms.yaml`

[stb_image / stb_image_write](https://github.com/nothings/stb) is vendored directly in `third_party/stb/`.

## Supported formats

| Format | Detection |
|---|---|
| MCAP | Magic bytes `\x89MCAP0\r\n` |
| ROS1 `.bag` | Magic bytes `#ROSBAG V2.0\n` |

Format is detected from file content, not the file extension.

## Supported message types

| Type | Role |
|---|---|
| `sensor_msgs/PointCloud2` | LiDAR and radar (radar identified by presence of `doppler` field) |
| `sensor_msgs/Image` | Raw camera frames |
| `sensor_msgs/CompressedImage` | Compressed camera frames (JPEG/PNG) |
| `sensor_msgs/CameraInfo` | Camera intrinsics and distortion |
| `tf2_msgs/TFMessage` | Dynamic transforms (`/tf`) |
| `tf2_msgs/TFMessage` | Static transforms (`/tf_static`) |

## Distortion models

- `plumb_bob` — pinhole + radial-tangential (`k1, k2, p1, p2, k3`)
- `equidistant` — fisheye (`k1, k2, k3, k4`)

## Out of scope

- ROS2 `.db3` files — convert to MCAP first (`mcap convert`)
- Any bag/MCAP parsing library
- Radar-specific message types beyond `PointCloud2` with a doppler field
