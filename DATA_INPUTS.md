# Platform Data Contract — What the Drone and ROV Must Provide

This document defines the data each platform must deliver to the detection
workflow. "Code enforces, models explain": every field below is consumed by
deterministic code (ingest, correlation, severity) before any model narrates.

## Shared requirements (both platforms)

Every captured frame/scan must carry a synchronized metadata envelope:

| Field | Type | Why it's required |
|---|---|---|
| `mission_id` | string | Groups all captures in one inspection run |
| `platform` | enum(`drone`,`rov`) | Routes to the correct detector |
| `timestamp_utc` | ISO-8601 (ms) | Cross-sensor alignment; must share clock source |
| `frame_id` / `scan_id` | string | Primary key for a capture |
| `pose` | 6-DoF (x,y,z,roll,pitch,yaw) | Projects detections into map/mesh |
| `sensor_extrinsics_ref` | calib ID | Links to camera↔LiDAR↔GPS / camera↔sonar calibration (Known Gap: sync/extrinsics) |
| `coverage_tag` | element/face hint | Seeds structure-relative location |

> All sensors on a platform must be time-synced to one clock. Unsynced
> pose/GPS/geometry will mis-register on the same defect.

## Recording & retrace (why no re-capture is needed)

The pipeline is **one-way**: it never signals a vehicle to return and re-capture
a defect. This works only because both platforms **record every stream
continuously** across the whole mission, not just at defects:

- Drone: full RGB imagery, full LiDAR point clouds, IMU/flight pose, GNSS/GPS
- ROV: full sonar imagery, full camera imagery, DVL/INS pose, depth

On ingest, the flow builds a **timestamp + pose index** over every stream. When
a detector flags a defect, the system **retraces** into the already-recorded
data by (timestamp, pose) to pull the co-located LiDAR cloud (drone) or camera
frame (ROV) it needs — no command goes back to the vehicle.

Hard requirement for retrace to work: all sensors on a platform must share one
clock and be spatially calibrated to each other (the cross-sensor
sync/extrinsics Known Gap). Retrace-by-timestamp-and-pose is only as accurate as
that calibration. See `PIPELINE.md` for the full one-way flow.

## Drone (above water) — Holybro X500 V2

Role: RGB camera is the **primary detector**; LiDAR provides **geometric
measurement**; GNSS provides the **geodetic tag**.

### Must provide
| Stream | Format | Requirement |
|---|---|---|
| RGB imagery | full-res frames or 4K video + keyframes | High enough GSD to resolve target defects; overlap for stitching |
| Camera intrinsics | calibration | Needed to project 2D detections into 3D |
| LiDAR point cloud | per-frame or per-pass cloud + timestamp | For deferred depth/area/deflection measurement at flagged regions |
| GNSS/GPS | lat, lon, elevation + fix quality | Geodetic pin captured **at detection time** |
| IMU / flight pose | 6-DoF per frame | SLAM pose, LiDAR–camera co-registration |
| Standoff distance | meters (est.) | Validates whether GSD supports fine-crack detection |

### Produced for the workflow
- Classified defect (class, confidence, 2D mask/box) from RGB+YOLO
- Geodetic coordinate per defect (captured at detection time)
- Point-cloud region reference for deferred LiDAR geometry enrichment
- Camera pose + extrinsics ref so geometry/GPS align on the same defect

## ROV (underwater)

Role: Sonar is the **primary detector** (structural/geometric defects);
camera is a **clarity-gated confirmer**.

### Must provide
| Stream | Format | Requirement |
|---|---|---|
| Sonar imagery | imaging/forward-look sonar frames + timestamp | Primary underwater detection; works in turbidity |
| Sonar geometry | range/bearing, sonar type, resolution | Needed to localize pier/substructure defect regions |
| Camera imagery | frames + a visibility/turbidity metric | Confirmation/fine-ID only when clarity allows |
| Camera intrinsics | calibration | Projects camera confirmations when used |
| Pose estimate | DVL / INS dead-reckoning 6-DoF | Structure-relative positioning underwater |
| Depth | meters | Populates `elevation_or_depth` in the location schema |
| Fiducial sightings | marker ID + relative pose (if seen) | Ties ROV dead-reckoning to drone GPS frame (Known Gap #1) |
| Tether/acoustic link status | enum | Flags bandwidth-limited segments (edge pre-filter) |

### Produced for the workflow
- Classified structural defect (class, confidence, region) from sonar model
- Optional camera confirmation + visibility metric
- Structure-relative location (element_id, face, station, offset, depth)
- Pose + extrinsics ref for sonar↔camera agreement on the same defect

## Convergence point

Both platforms emit into the **same normalized detection record** before the
Correlation stage:

```json
{
  "detection_id": "string",
  "mission_id": "string",
  "platform": "drone|rov",
  "class": "crack|spall|exposed_rebar|corrosion_stain|scour|section_loss|pier_damage",
  "confidence": 0.0,
  "location": {
    "geodetic": { "lat": 0.0, "lon": 0.0, "elevation": 0.0 },
    "structure": { "element_id": "", "face": "", "station": 0.0, "offset": 0.0, "elevation_or_depth": 0.0 }
  },
  "geometry": { "depth_mm": null, "area_mm2": null, "deflection_mm": null },
  "evidence_frame_ids": ["string"],
  "model_version": "string",
  "change_status": "new|existing|worsening"
}
```

> `geometry` is populated later (deferred LiDAR enrichment on the drone side);
> it is null at detection time and filled before severity computation.
