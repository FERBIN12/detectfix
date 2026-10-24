"""Bridge-inspection detection pipeline.

One-way flow: ingest .mcap -> decode + time/pose index -> detect (camera YOLO,
sonar model) -> retrace recorded data by (timestamp, pose) -> LiDAR depth
estimation -> map/stitch -> correlate + rule-based severity -> Claude report
agent -> Claude dashboard agent -> human sign-off.

Design rule carried from ARCHITECTURE.md: code enforces, models explain.
Everything up to the report is deterministic Python; the LLM agents only draft
narrative and present it, constrained to the structured detection data.
"""

__all__ = ["schema", "config", "indexing", "detection", "lidar", "correlation", "severity"]
