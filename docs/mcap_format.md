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

When `compression = "none"`, `records` is a raw record stream and can be parsed directly. Compressed chunks require a decompression step before parsing. This implementation only supports `"none"`; encountering a compressed chunk throws a `std::runtime_error`.

---

## Parsing Strategy

We use a **single-pass, pull-based** approach: the caller calls `next()` and gets one `RawMessage` at a time. Nothing is buffered beyond the current chunk.

### The read source

The key abstraction is `read_into(dst, n)`, which reads `n` bytes from whichever source is currently active:

- **Normal mode**: reads directly from `file_`.
- **Chunk mode**: reads from `chunk_buf_`, a `std::vector<uint8_t>` holding the current chunk's uncompressed record stream. `chunk_pos_` tracks how far we've consumed it.

All typed readers (`r_u8`, `r_u16`, `r_u32`, `r_u64`, `r_str`, `skip`) are built on `read_into`, so the same code path handles both in-chunk and out-of-chunk records transparently.

### Main loop (next / topics)

```
loop:
  if chunk_buf_ is exhausted → clear it (fall back to file_)
  if file_ is at EOF → return false / stop

  op, len = read record header

  DataEnd  → stop
  Chunk    → load body into chunk_buf_, continue
  Schema   → parse, store in schema_names_, continue
  Channel  → parse, store in channels_, continue
  Message  → fill RawMessage, return true     (next only)
           → count per channel_id, skip data  (topics only)
  other    → skip(len)
```

Entering a chunk means reading its body from `file_` into `chunk_buf_` in one bulk read. After that, the loop's next iteration draws from `chunk_buf_` instead of `file_` until the buffer is exhausted, at which point normal file reading resumes.

Because Schema and Channel records accumulate into member maps (`schema_names_`, `channels_`), they are registered once and available for all subsequent Message records regardless of whether they appeared at the top level or inside a chunk.

### topics() vs next()

`topics()` rewinds to `data_start_`, runs the same loop but skips message data entirely (reads channel_id, increments a counter, skips the remaining bytes), then returns a `TopicInfo` per channel. It calls `rewind()` again before returning so the reader is ready to stream from the start.

`next()` runs the same loop but copies message bytes into `RawMessage::data` and returns to the caller immediately. Schema and channel state accumulated during `topics()` is reused, so `next()` will not re-parse them on the second pass — it just overwrites with identical values if it encounters them again.

---

## What We Do Not Support

| Feature | Reason omitted |
|---|---|
| Compressed chunks (lz4 / zstd) | No decompression library — design constraint |
| Summary section fast-path for `topics()` | Full scan is sufficient; avoids footer seek complexity |
| CRC validation | Adds overhead; trust the storage layer |
| Attachment / Metadata records | Not needed for sensor data extraction |
