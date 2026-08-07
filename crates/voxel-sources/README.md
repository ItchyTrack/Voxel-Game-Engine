# Voxel Sources

This crate has two halves:

- `reader` coordinates requests from streaming, networking, and other callers.
- `source_manager` selects and combines registered voxel sources.

The reader calls the source manager directly. The source manager exposes completed loads through `get_completed()` and source changes through `get_source_changes()`; it does not know about reader requesters or tile generation.

Use this crate by implementing a `ChunkSource`.
