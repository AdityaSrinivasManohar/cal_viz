# How a PointCloud2 Message is Packed in MCAP

This walks through what the file actually contains when a recorder writes a single
`sensor_msgs/PointCloud2` message to an MCAP file. No binary arithmetic — just the
logical structure, drawn out layer by layer.

---

## The Setup

Suppose a LiDAR scanner publishes on the topic `/lidar/points`, and the cloud has
two points with x, y, z, and intensity fields:

```
Point 0:  x=1.0  y=2.0  z=3.0  intensity=100.0
Point 1:  x=4.0  y=5.0  z=6.0  intensity=200.0
```

For this message to land in the MCAP file, three distinct records must be written:

```
┌───────────────────────────────┐
│  Schema record                │   ← "what type is this?"
├───────────────────────────────┤
│  Channel record               │   ← "what topic uses that type?"
├───────────────────────────────┤
│  Message record               │   ← "here is the actual data"
└───────────────────────────────┘
```

Each record has the same outer envelope:

```
[ op : 1 byte ][ body_len : 8 bytes ][ body : body_len bytes ]
```

---

## Record 1 — Schema

The Schema record says: "schema #7 is called `sensor_msgs/PointCloud2`
and its definition looks like this."

```
┌─────────────────────────────────────────────────────────────┐
│  MCAP envelope                                              │
│  op       = 0x03  (Schema)                                  │
│  body_len = (total bytes below)                             │
├─────────────────────────────────────────────────────────────┤
│  BODY                                                       │
│  schema_id  = 7           (uint16, 2 bytes)                 │
│  name       = "sensor_msgs/PointCloud2"  (uint32 len + str) │
│  encoding   = "ros2msg"   (uint32 len + str)                │
│  data       = <the full ROS2 .msg definition text>          │
│               (uint32 len + bytes)                          │
│               "std_msgs/Header header\nuint32 height\n..."  │
└─────────────────────────────────────────────────────────────┘
```

The `data` field is purely for external tools (like schema registries) that want to
know the field layout. We skip it entirely — we only care about `name`.

> **MCAP strings** use `uint32 length + bytes` with **no null terminator**.
> This differs from CDR strings (used inside the message body) which include a null.

---

## Record 2 — Channel

The Channel record says: "channel #3 carries topic `/lidar/points`,
its messages are encoded in CDR, and their type is schema #7."

```
┌─────────────────────────────────────────────────────────────┐
│  MCAP envelope                                              │
│  op       = 0x04  (Channel)                                 │
│  body_len = (total bytes below)                             │
├─────────────────────────────────────────────────────────────┤
│  BODY                                                       │
│  channel_id       = 3               (uint16)                │
│  schema_id        = 7               (uint16)  ← links to ↑ │
│  topic            = "/lidar/points" (uint32 len + str)      │
│  message_encoding = "cdr"           (uint32 len + str)      │
│  metadata         = {}              (uint32 count=0)        │
└─────────────────────────────────────────────────────────────┘
```

Now the reader knows: any Message record with `channel_id = 3` carries a
`sensor_msgs/PointCloud2`, serialized as ROS CDR, on topic `/lidar/points`.

The double indirection (message → channel → schema) means a file with 10,000
messages does not repeat the 23-character type name 10,000 times.

---

## Record 3 — Message

The Message record says: "at timestamp T, channel #3 published this blob of bytes."

```
┌─────────────────────────────────────────────────────────────┐
│  MCAP envelope                                              │
│  op       = 0x05  (Message)                                 │
│  body_len = (total bytes below)                             │
├─────────────────────────────────────────────────────────────┤
│  BODY                                                       │
│  channel_id   = 3           (uint16)  ← links to channel ↑ │
│  sequence     = 42          (uint32)  ← per-channel counter │
│  log_time     = 1000.5 s    (uint64 nanoseconds)            │
│  publish_time = 1000.5 s    (uint64 nanoseconds)            │
│  data         = <CDR bytes> (rest of body)                  │
└─────────────────────────────────────────────────────────────┘
```

The `data` field is the serialized PointCloud2. We copy it verbatim into
`RawMessage::data` and hand it to the deserializer layer.

---

## Inside the CDR Blob

The `data` bytes are a ROS2 CDR-encoded `sensor_msgs/PointCloud2`. CDR is a
binary format where multi-byte fields must be **naturally aligned** — a `uint32`
must start at a byte offset divisible by 4, a `float32` must be at a multiple of 4,
and so on. Gaps between fields are filled with zero padding bytes.

The logical layout, field by field:

```
CDR encapsulation header  (4 bytes — always 0x00 0x01 0x00 0x00 for LE)
│
├─ Header
│   ├─ stamp.sec       : int32   = 1000
│   ├─ stamp.nanosec   : uint32  = 500000000
│   └─ frame_id        : string  = "lidar"
│                                  (CDR string = uint32 len including \0, then bytes+\0)
│                       [padding to next 4-byte boundary]
│
├─ height       : uint32  = 1         (unorganised cloud → single row)
├─ width        : uint32  = 2         (2 points)
│
├─ fields       : sequence of PointField  (uint32 count = 4, then 4 entries)
│   ├─ PointField "x"
│   │   ├─ name      : string  = "x"
│   │   ├─ offset    : uint32  = 0     ← byte offset within one point record
│   │   ├─ datatype  : uint8   = 7     ← FLOAT32
│   │   └─ count     : uint32  = 1
│   ├─ PointField "y"
│   │   ├─ name      : string  = "y"
│   │   ├─ offset    : uint32  = 4
│   │   ├─ datatype  : uint8   = 7
│   │   └─ count     : uint32  = 1
│   ├─ PointField "z"
│   │   ├─ name      : string  = "z"
│   │   ├─ offset    : uint32  = 8
│   │   ├─ datatype  : uint8   = 7
│   │   └─ count     : uint32  = 1
│   └─ PointField "intensity"
│       ├─ name      : string  = "intensity"
│       ├─ offset    : uint32  = 12
│       ├─ datatype  : uint8   = 7
│       └─ count     : uint32  = 1
│
├─ is_bigendian : bool    = false
│               [padding to 4-byte boundary]
├─ point_step   : uint32  = 16    ← bytes per point (4 fields × 4 bytes)
├─ row_step     : uint32  = 32    ← bytes per row   (2 points × 16 bytes)
│
├─ data         : uint8[] = 32 bytes
│   ├─ point 0  [16 bytes]
│   │   ├─  bytes  0– 3 : float32  x         = 1.0
│   │   ├─  bytes  4– 7 : float32  y         = 2.0
│   │   ├─  bytes  8–11 : float32  z         = 3.0
│   │   └─  bytes 12–15 : float32  intensity = 100.0
│   └─ point 1  [16 bytes]
│       ├─  bytes  0– 3 : float32  x         = 4.0
│       ├─  bytes  4– 7 : float32  y         = 5.0
│       ├─  bytes  8–11 : float32  z         = 6.0
│       └─  bytes 12–15 : float32  intensity = 200.0
│
└─ is_dense     : bool    = true
```

A few things worth noting:

- **Point data is flat**. There are no per-point headers — each point is simply
  `point_step` bytes side by side. To read field `x` from point `i`, the deserializer
  does `data[i * point_step + field.offset]` and reinterprets those 4 bytes as a float.

- **The `fields` array is the schema at runtime**. The PointField records describe
  what lives at each byte offset. Our code uses these offsets directly via
  `PointCloud::field_f32(point_idx, field_name)` rather than hard-coding them.

- **Radar is just a PointCloud2 with an extra field**. A radar cloud would have a
  fifth PointField named `"doppler"` at `offset=16`, making `point_step=20`. Nothing
  else about the format changes.

- **CDR alignment padding is invisible to the deserializer**. The padding bytes only
  appear between the high-level struct fields (Header, height, width, etc.),
  not inside the flat point data array, which is always tightly packed.

---

## End-to-end Summary

```
MCAP file
│
├─ Schema  record  (op=0x03)
│   └─ id=7, name="sensor_msgs/PointCloud2"
│
├─ Channel record  (op=0x04)
│   └─ id=3, schema_id=7, topic="/lidar/points"
│
└─ Message record  (op=0x05)
    ├─ channel_id=3, log_time=1000500000000 ns
    └─ data = CDR blob
        ├─ header  (frame_id="lidar", stamp=1000.5 s)
        ├─ height=1, width=2
        ├─ fields  [x@0, y@4, z@8, intensity@12]  FLOAT32
        ├─ point_step=16, row_step=32
        └─ data[]  = [ 1.0 2.0 3.0 100.0 | 4.0 5.0 6.0 200.0 ]
                       ← point 0, 16 B → ← point 1, 16 B →
```

When `McapReader::next()` returns this message, `RawMessage::data` holds exactly the
CDR blob above. The deserializer in `msgs/deserialize` then walks that blob field by
field, applying the CDR alignment rules, to produce a `msgs::PointCloud` with the
two `Point` structs.
