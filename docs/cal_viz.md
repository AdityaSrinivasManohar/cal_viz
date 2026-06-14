### **Requirements**

**Input**

- `.mcap` files
- ROS1 `.bag` files
- Auto-detect format from magic bytes, not file extension
- No external libraries for parsing either format — raw byte parsing only

**Topic Discovery (fully automatic)**

- Scan all topics and message schemas from the file
- Identify all LiDAR topics (`sensor_msgs/PointCloud2`)
- Identify all camera topics (`sensor_msgs/Image`, `sensor_msgs/CompressedImage`)
- Auto-match `sensor_msgs/CameraInfo` to each camera topic by naming convention, falling back to `frame_id` matching
- Build TF tree from `/tf` and `/tf_static`
- Fallback to user-supplied `extrinsics.xml` if TF is missing or incomplete

**Pair Validation**

- For every LiDAR × camera combination:
    - Check TF tree for a valid transform path between their frames
    - Check timestamp overlap between topics
    - Project a sample of points and measure image overlap percentage
    - Skip pairs below configurable overlap threshold (default 5%)
    - Warn on any skipped pairs with reason

**Projection**

- Apply extrinsic transform (SE3) from LiDAR frame to camera frame
- Apply camera intrinsics (K matrix + distortion) from `CameraInfo`
- Support pinhole + radial-tangential distortion
- Support fisheye lens model
- Cull points behind camera or outside image bounds
- Interpolate TF transforms at each message timestamp (slerp + lerp)

**Colorization**

- Depth (distance from camera)
- Intensity (LiDAR return strength)
- Doppler velocity (radar, blue=approaching red=receding)
- Height (Z in world frame)
- Ring / beam ID

---
### **Outputs**

**`unbag` — raw data extraction**

Extracts raw sensor data from the bag, deduplicated per topic (no per-pair copies).

- One `.pcd` per LiDAR scan per LiDAR topic (binary format, no library needed)
- One `.jpg` per camera frame per camera topic
- `camera_info.yaml` with intrinsics for each camera topic
- `timestamps.csv` mapping every file to log and publish timestamps

```
output/
├── lidar/
│   └── <lidar_topic>/
│       └── <timestamp_ns>.pcd
├── cameras/
│   └── <camera_topic>/
│       ├── <timestamp_ns>.jpg
│       └── camera_info.yaml
└── timestamps.csv
```

**`project` — projection overlays**

Produces only the projected images (LiDAR points overlaid on camera frames) for each valid LiDAR × camera pair.

- One `.jpg` per projected frame per valid pair
- `transforms.yaml` with all TF data used for each pair

```
output/
└── <lidar_topic>__<camera_topic>/   # one dir per valid pair
    ├── projected/
    │   └── <timestamp_ns>.jpg
    └── transforms.yaml
```

---

### **CLI**

```bash
# Extract raw sensor data (deduplicated per topic)
cal_viz unbag \
  --input   drive.bag   \
  --output  ./out

# Project LiDAR onto camera frames
cal_viz project \
  --input              drive.bag             \
  --output             ./out                 \
  --colorize           depth|intensity|doppler|height|ring  \  # default: depth
  --point-size         3                     \   # default: 3px
  --overlap-threshold  0.05                  \   # default: 5%
  --extrinsics         extrinsics.yaml       \   # optional override
  --lidar              /lidar_front          \   # optional: force specific topic
  --camera             /cam_front                # optional: force specific topic
```

---

### **Non-requirements (explicitly out of scope)**

- ROS2 `.db3` parsing — users should convert to MCAP first
- Any external bag/MCAP parsing library
- SQLite dependency
- Radar-specific message types beyond `PointCloud2` with doppler field