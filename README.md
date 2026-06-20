# cal_viz

Projects LiDAR and radar point clouds onto camera images from `.mcap` and ROS1 `.bag` files.

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

## Development setup

The recommended way to work on this project is via the included devcontainer, which provides the full toolchain with no host setup required.

**Prerequisites:** Docker and the VS Code Dev Containers extension (or any OCI-compatible runtime).

```bash
# In VS Code: "Dev Containers: Reopen in Container"
# Or from the CLI:
docker build -t cal_viz .devcontainer/
docker run --rm -it -v $(pwd):/workspaces/cal_viz cal_viz
```

The container runs natively on arm64 (Apple Silicon). It provides:

| Tool | Version |
|---|---|
| Clang / Clang++ | 22 |
| clang-format | 22 |
| clang-tidy | 22 |
| clangd | 22 |
| CMake | 4.3.3 |
| mcap CLI | latest |

### IDE integration (clangd)

The project includes a `.clangd` config and `.vscode/settings.json` with format-on-save enabled. After opening the devcontainer in VS Code, install the `clangd` extension (`llvm-vs-code-extensions.vscode-clangd`) if it isn't already active, then run **clangd: Restart language server** from the command palette.

clangd reads `build/compile_commands.json` for include paths and compiler flags. Run CMake configure at least once before opening source files (see Build below).

## Build

All dependencies are fetched and statically linked by CMake — no package manager installs needed beyond what the devcontainer provides.

```bash
cmake -B build
cmake --build build
```

The binary is at `./build/cal_viz`.

On first configure, CMake fetches and pins:

| Library | Version | Purpose |
|---|---|---|
| [Eigen](https://gitlab.com/libeigen/eigen.git) | 5.0.1 | Quaternion slerp, matrix–vector math for projection and TF interpolation |
| [yaml-cpp](https://github.com/jbeder/yaml-cpp) | 0.8.0 | Reading `extrinsics.yaml`, writing `camera_info.yaml` / `transforms.yaml` |
| [mcap](https://github.com/foxglove/mcap) | 2.1.3 | MCAP file reading (header-only, single-header compilation model) |

[stb_image / stb_image_write](https://github.com/nothings/stb) is vendored in `third_party/stb/` and requires no fetch step.

Internet access is only needed on the first `cmake -B build`. Subsequent builds and reconfigures are fully offline.

### Parallel builds

```bash
cmake --build build --parallel
```

### Cleaning

```bash
rm -rf build && cmake -B build
```

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
- Radar-specific message types beyond `PointCloud2` with a doppler field
