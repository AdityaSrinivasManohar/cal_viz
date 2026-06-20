# MCAP Format and Parsing Strategy

## What is MCAP?

MCAP is a binary container format for timestamped, multi-channel data. It was designed as the successor to ROS1 `.bag` files and is the native format for ROS2 recordings. The format's primary goals are:

- **Efficient streaming**: records are self-describing and can be read front-to-back without an index.
- **Random access**: an optional summary section at the end provides byte-offset indexes for seeking.
- **Compression**: records can be bundled into chunks and compressed (lz4, zstd, or none).
- **Schema-agnostic**: the format stores any serialized message type (ROS CDR, Protobuf, JSON, etc.) as raw bytes alongside the schema that describes it.

---

## File Structure

An MCAP file is divided into three sections, bookended by magic bytes:

```
┌─────────────────────────────────────────────┐
│  Magic bytes  \x89MCAP0\r\n  (8 bytes)      │
├─────────────────────────────────────────────┤
│  Header record                              │
├─────────────────────────────────────────────┤
│                                             │
│  DATA SECTION                               │
│  ─ Schema records                           │
│  ─ Channel records                          │
│  ─ Chunk records  (contain more records)    │
│  ─ Message records                          │
│  ─ DataEnd record  (sentinel)               │
│                                             │
├─────────────────────────────────────────────┤
│                                             │
│  SUMMARY SECTION  (optional)                │
│  ─ Statistics, ChunkIndex, etc.             │
│                                             │
├─────────────────────────────────────────────┤
│  Footer record                              │
├─────────────────────────────────────────────┤
│  Magic bytes  \x89MCAP0\r\n  (8 bytes)      │
└─────────────────────────────────────────────┘
```

We only parse the **data section**. The summary section exists for tools that need fast random-access; we don't need it because we stream front-to-back.

---

## Record Envelope

Every record, regardless of type, has the same three-field envelope:

```
┌──────────┬───────────────────┬───────────────────────┐
│  op      │  body_len         │  body                 │
│  1 byte  │  8 bytes (uint64) │  body_len bytes       │
└──────────┴───────────────────┴───────────────────────┘
```

- **op**: identifies the record type (see table below).
- **body_len**: number of bytes in the body, little-endian.
- **body**: the record-specific payload.

All multi-byte integers in MCAP are **little-endian**. Strings are **length-prefixed**: a `uint32` byte count followed by that many UTF-8 bytes (no null terminator).

| op     | Record type  | Role                                          |
|--------|--------------|-----------------------------------------------|
| `0x01` | Header       | File-level metadata (profile, library)        |
| `0x02` | Footer       | Pointer to summary section                    |
| `0x03` | Schema       | Declares a message type by name and encoding  |
| `0x04` | Channel      | Maps a topic name to a schema                 |
| `0x05` | Message      | One serialized message with timestamps        |
| `0x06` | Chunk        | A bundle of records, optionally compressed    |
| `0x0F` | DataEnd      | Sentinel marking the end of the data section  |
| others | Index/stats  | Only appear in the summary section            |

---

## Key Record Types in Detail

### Schema (op=0x03)

A Schema record introduces a message type into the file. It assigns a numeric ID so that subsequent Channel and Message records don't need to repeat the full type name.

```
schema_id   : uint16
name        : string   ← e.g. "sensor_msgs/PointCloud2"
encoding    : string   ← e.g. "ros2msg", "proto", "jsonschema"
data        : bytes    ← the raw schema definition (uint32 length + bytes)
```

We store `schema_id → name` in `schema_names_`. The `encoding` and `data` fields are skipped; we only need the name to populate `RawMessage::msg_type`.

### Channel (op=0x04)

A Channel record introduces a topic and links it to a schema.

```
channel_id      : uint16
schema_id       : uint16   ← references a previously seen Schema
topic           : string   ← e.g. "/lidar/points"
message_encoding: string   ← e.g. "cdr"
metadata        : map      ← uint32 count, then key/value string pairs
```

We store `channel_id → { topic, schema_id }` in `channels_`. The indirection through schema_id means thousands of messages on the same topic don't repeat the type string.

### Message (op=0x05)

A Message record carries one serialized payload.

```
channel_id  : uint16
sequence    : uint32   ← per-channel counter (unused by us)
log_time    : uint64   ← nanoseconds since epoch, when recorded
publish_time: uint64   ← nanoseconds since epoch, when published
data        : bytes    ← remainder of body; raw serialized message
```

To recover the topic and type for a given message:

```
channel_id  →  channels_   →  { topic, schema_id }
                                       │
                               schema_names_  →  msg_type
```

The `data` bytes are ROS CDR (or whatever encoding the channel declared). We copy them directly into `RawMessage::data` for the deserializer layer to interpret.

### Chunk (op=0x06)

A Chunk bundles an arbitrary number of records together, optionally compressed.

```
message_start_time : uint64   ← earliest log_time in this chunk
message_end_time   : uint64   ← latest log_time in this chunk
uncompressed_size  : uint64   ← byte count after decompression
uncompressed_crc   : uint32   ← CRC32 of uncompressed data (0 = disabled)
compression        : string   ← "none", "lz4", or "zstd"
records            : bytes    ← the (possibly compressed) record stream
```

The `records` field is itself a sequence of MCAP records — Schema, Channel, and Message records can all appear inside a chunk exactly as they do at the top level. Chunks **cannot** be nested.

When `compression = "none"`, `records` is a raw record stream and can be parsed directly. Compressed chunks (`lz4`, `zstd`) require a decompression step before parsing.

---

## Parsing Strategy

Parsing is delegated to the **Foxglove MCAP C++ library** (`foxglove/mcap`, v2.1.3). We do not hand-roll byte parsing.

The library uses a single-header compilation model: including `<mcap/reader.hpp>` provides declarations; defining `MCAP_IMPLEMENTATION` in exactly one translation unit (`mcap_reader.cpp`) compiles the implementations from the accompanying `.inl` files. Compression support is disabled at compile time via `MCAP_COMPRESSION_NO_LZ4` and `MCAP_COMPRESSION_NO_ZSTD` — these macros are set project-wide in `CMakeLists.txt`.

### What the library provides

- **`mcap::McapReader::open(path)`** — opens the file, reads and validates the magic bytes and Header record, and parses the summary section (if present) to populate channel/schema maps and statistics.
- **`reader.channels()` / `reader.schemas()`** — pre-populated maps available immediately after `open()`, used by `topics()` without scanning the data section.
- **`reader.statistics()`** — returns per-channel message counts from the summary section, also without a data scan.
- **`reader.readMessages()`** — returns a lazy `LinearMessageView` range. Iterating it streams messages front-to-back, handling chunk decompression, record framing, and schema/channel resolution internally.

### How McapReader wraps it

```
McapReader::topics()
  → reader_.channels() + reader_.schemas() + reader_.statistics()
  → O(num_channels) — no file scan

McapReader::next()
  → advance LinearMessageView::Iterator
  → copy mv.channel->topic, mv.schema->name, mv.message.logTime/publishTime
  → memcpy mv.message.data into RawMessage::data
  → return true

McapReader::rewind()
  → recreate IterState{ view = reader_.readMessages(), it = view.begin() }
```

The `IterState` struct bundles the `LinearMessageView` and its iterator together so their lifetimes are tied. It is stored in a `unique_ptr` declared after `mcap::McapReader reader_`, ensuring it is destroyed first (C++ destroys members in reverse declaration order).

### What we still do not support

| Feature | Reason |
|---|---|
| Compressed chunks (lz4 / zstd) | `MCAP_COMPRESSION_NO_LZ4/ZSTD` disabled at compile time; add lz4/zstd deps to enable |
| CRC validation | Library supports it but we leave it at default (disabled) |
| Attachment / Metadata records | Not needed for sensor data extraction |
