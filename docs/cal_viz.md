### **Requirements**

**Input**

- `.mcap` files
- Format detected from magic bytes, not file extension

**Projection**

- Apply extrinsic transform (SE3) from LiDAR frame to camera frame using TF tree
- Apply camera intrinsics (K matrix + distortion) from `CameraInfo`
- Support pinhole + radial-tangential distortion (`plumb_bob`)
- Support fisheye lens model (`equidistant`)
- Cull points behind camera or outside image bounds
- Interpolate TF transforms at each message timestamp (slerp + lerp)

**Colorization**

- Depth (distance from camera origin)
- Intensity (LiDAR return strength)

---

### **Outputs**

**`unbag` — raw data extraction**

Extracts raw sensor data from the bag, organised per topic.

- One ASCII `.pcd` per LiDAR scan per topic
- One `.jpg` per camera frame per topic

```
output/
├── <lidar_topic>/
│   └── <timestamp_ns>.pcd
└── <camera_topic>/
    └── <timestamp_ns>.jpg
```

**`project` — projection overlays**

Produces projected images (LiDAR points overlaid on camera frames) for each LiDAR × camera pair that has a valid TF path.

- One `.jpg` per projected frame per valid pair

```
output/
└── <lidar>__<camera>/
    └── <timestamp_ns>.jpg
```

---

### **CLI**

```bash
# Extract raw sensor data
cal_viz unbag \
  --input   drive.mcap \
  --output  ./out

# Project LiDAR onto camera frames
cal_viz project \
  --input      drive.mcap  \
  --output     ./out        \
  --colorize   depth        \   # depth | intensity  (default: depth)
  --pointsize  2            \   # dot radius in pixels  (default: 2)
  --lidar      /lidar_front \   # optional: pin to a specific topic
  --camera     /cam_front       # optional: pin to a specific topic
```

---

### **Non-requirements (or TODOs)**

- ROS1 `.bag` parsing — convert to MCAP first (`mcap convert`)
- ROS2 `.db3` parsing — convert to MCAP first (`mcap convert`)
- Radar-specific message types beyond `PointCloud2`
- Extrinsics YAML override (TF tree is the only source of transforms)
