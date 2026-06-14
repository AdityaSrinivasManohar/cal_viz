I plan to make a cpp project to do the following

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
- Fallback to user-supplied `extrinsics.yaml` if TF is missing or incomplete

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

**Mode 1 — MCAP**

- Annotated `CompressedImage` per valid pair per frame
- Projected 2D points with depth metadata
- All original topics passed through unchanged
- Output viewable directly in Foxglove Studio

**Mode 2 — Folder dump**

- One `.pcd` per LiDAR scan (binary format, no library needed)
- One `.jpg` per raw camera frame
- One `.jpg` per projected frame (points overlaid)
- `timestamps.csv` mapping every file to log and publish timestamps
- `camera_info.yaml` with intrinsics for each camera
- `transforms.yaml` with all TF data used for each pair

**Mode 3 — Both**

- Produces both MCAP and folder dump simultaneously

**Folder structure**

```
output/
├── <lidar_topic>__<camera_topic>/   # one dir per valid pair
│   ├── pointclouds/
│   │   └── <timestamp_ns>.pcd
│   ├── images/
│   │   └── <timestamp_ns>.jpg
│   ├── projected/
│   │   └── <timestamp_ns>.jpg
│   ├── camera_info.yaml
│   └── transforms.yaml
├── timestamps.csv
├── discovery_report.txt
└── output.mcap                      # mode 1 or 3 only
```

**discovery_report.txt**

- All LiDAR and camera topics found
- CameraInfo match result per camera topic
- Per-pair: TF status, timestamp overlap, projection overlap %, decision
- All warnings and skipped pairs with reasons

---

### **CLI**

```bash
cal_viz \
  --input     drive.bag             \
  --output    ./out                 \
  --mode      mcap|folder|both      \   # default: both
  --colorize  depth|intensity|doppler|height|ring  \  # default: depth
  --point-size         3            \   # default: 3px
  --overlap-threshold  0.05         \   # default: 5%
  --extrinsics extrinsics.yaml      \   # optional override
  --lidar      /lidar_front         \   # optional force specific topics
  --camera     /cam_front               # optional force specific topics
```

---

### **Non-requirements (explicitly out of scope)**

- ROS2 `.db3` parsing — users should convert to MCAP first
- Any external bag/MCAP parsing library
- SQLite dependency
- Radar-specific message types beyond `PointCloud2` with doppler field