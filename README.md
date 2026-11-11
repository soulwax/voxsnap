# voxsnap

Pure Dart deterministic binary state snapshots, smallest-three quaternion packing, XOR/RLE delta compression, O(1) time-rewind ring buffers, and smooth interpolated ghost replays.

## Features

- **Compact Binary Streams (`BinarySnapshotWriter` / `BinarySnapshotReader`):** LEB128 varints, ZigZag signed ints, range-quantized floats/vectors, and 32-bit smallest-three compressed unit quaternions.
- **XOR + Zero-RLE Delta Compression (`BinaryDeltaCodec`):** Bit-exact delta snapshots between baseline and current byte frames.
- **Time-Rewind Ring Buffer (`RewindBuffer<T>`):** Bounded fixed-capacity circular buffer supporting reverse scrubbing, timeline truncation, and O(log N) binary time sampling.
- **Ghost Replay Tracks (`GhostTrack`, `GhostPlayer`):** Smooth position + shortest-path quaternion NLERP playback at any framerate, with compact binary export/import for racing ghosts and killcams.
