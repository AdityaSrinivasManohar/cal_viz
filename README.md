# cal_viz

Projects LiDAR point clouds onto camera images from `.mcap` files.

## Commands

### `unbag` — extract raw sensor data

Dumps every LiDAR scan as an ASCII `.pcd` and every camera frame as a `.jpg`, organised by topic. Run this once per bag to extract raw data independently of projection.

```bash
cal_viz unbag --input drive.mcap --output ./out
```

Options:

| Flag | Default | Description |
|---|---|---|
| `--input` | required | bag file to read |
| `--output` | `out` | output directory |

Output layout:

```
out/
├── <lidar_topic>/<timestamp_ns>.pcd
└── <camera_topic>/<timestamp_ns>.jpg
```

---

### `project` — project LiDAR onto camera frames

Discovers all LiDAR × camera pairs from the TF tree and writes one projected `.jpg` per frame per pair. Pin to specific topics with `--lidar` / `--camera`.

```bash
cal_viz project \
  --input      drive.mcap \
  --output     ./out      \
  --colorize   depth      \
  --pointsize 3          \
  --lidar      /lidar_front \
  --camera     /cam_front
```

Options:

| Flag | Default | Description |
|---|---|---|
| `--input` | required | bag file to read |
| `--output` | `out` | output directory |
| `--colorize` | `depth` | `depth` or `intensity` |
| `--pointsize` | `2` | dot radius in pixels |
| `--lidar` | _(all)_ | pin to a specific LiDAR topic |
| `--camera` | _(all)_ | pin to a specific camera topic |

Output layout:

```
out/
└── <lidar>__<camera>/
    └── <timestamp_ns>.jpg
```

---

## Build

All dependencies are fetched and statically linked by CMake — no package manager installs needed beyond a C++20 compiler and CMake.

```bash
cmake -B build
cmake --build build --parallel
```

The binary is at `./build/cal_viz`.

On first configure, CMake fetches and pins:

| Library | Version | Purpose |
|---|---|---|
| [Eigen](https://gitlab.com/libeigen/eigen.git) | 5.0.1 | Quaternion slerp, matrix–vector math for projection and TF interpolation |
| [mcap](https://github.com/foxglove/mcap) | 2.1.3 | MCAP file reading (header-only) |

[stb_image / stb_image_write](https://github.com/nothings/stb) is vendored in `third_party/stb/` and requires no fetch step.

Internet access is only needed on the first `cmake -B build`. Subsequent builds are fully offline.

### Cleaning

```bash
rm -rf build && cmake -B build
```

---

## Development setup

The recommended way to work on this project is via the included devcontainer, which provides the full toolchain with no host setup required.

**Prerequisites:** Docker and the VS Code Dev Containers extension (or any OCI-compatible runtime).

```bash
# In VS Code: Dev Containers: Reopen in Container
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

clangd reads `build/compile_commands.json` for include paths and compiler flags. Run CMake configure at least once before opening source files.

---

## Supported formats

| Format | Notes |
|---|---|
| MCAP | Detected from magic bytes `\x89MCAP0\r\n` |

## Supported message types

| Type | Role |
|---|---|
| `sensor_msgs/PointCloud2` | LiDAR scans |
| `sensor_msgs/Image` | Raw camera frames |
| `sensor_msgs/CompressedImage` | Compressed camera frames (JPEG/PNG) |
| `sensor_msgs/CameraInfo` | Camera intrinsics and distortion |
| `tf2_msgs/TFMessage` | Dynamic transforms (`/tf`) and static transforms (`/tf_static`) |

## Distortion models

- `plumb_bob` — pinhole + radial-tangential (`k1, k2, p1, p2, k3`)
- `equidistant` — fisheye (`k1, k2, k3, k4`)
