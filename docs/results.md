# Verified Results

This document captures runtime metrics observed when running the
`vehicle_detection.launch.py` pipeline on a sample point cloud inside the
`osrf/ros:jazzy-desktop` Docker image.

## Environment

- Container image: `osrf/ros:jazzy-desktop`
- ROS 2 distro: Jazzy
- Compiler: GCC (system default in the image)
- PCL: distro-provided `libpcl-all-dev`
- Workspace mounted into the container; built with `colcon build` in
  release configuration

## Pipeline Numbers

A representative single-frame run with the bundled `detector_params.yaml`:

| Stage           | Points / Count |
|-----------------|----------------|
| Input           | 118,784        |
| After voxel+ROI+ground removal | 9,627 |
| Euclidean clusters | 66          |
| Vehicle detections | 7           |
| Per-frame time  | ~160 ms        |

## Sample Detection

One detection from the published `vision_msgs/msg/Detection3DArray` on
`/vehicle_detections/raw`:

- `bbox.center.position`: (20.29, -9.41, -0.95)
- `bbox.size`: (4.35, 2.54, 1.79) — axis-aligned extents (`dx`, `dy`, `dz`)
  matching the identity orientation of the bbox
- `results[0].hypothesis.class_id`: `car`
- `results[0].hypothesis.score`: 0.8 (placeholder fixed confidence)

## Test Results

`colcon test --packages-select vehicle_detection` runs:

- `test_parameter_validation` — GTest unit tests for the parameter
  validation helpers
- `test_point_cloud_processing` — GTest unit tests for the PCL pipeline
  helpers (box dimensions, vehicle filter, AABB, CropBox, clustering)
- `test_launch_description` — pytest smoke tests confirming
  `vehicle_detection.launch.py` imports, declares the expected launch
  arguments, and registers the expected nodes

## Topics Verified

When `use_detector:=true` (default):

- `/input/points` — sensor_msgs/PointCloud2 from `pcd_loader_node`
- `/vehicle_detections/raw` — vision_msgs/Detection3DArray
- `/debug/points_filtered` — sensor_msgs/PointCloud2 (post voxel + ROI +
  ground removal)
- `/debug/clusters` — visualization_msgs/MarkerArray (cluster AABBs)
- `/vehicle_markers` — visualization_msgs/MarkerArray (passenger-vehicle
  AABBs)

When `use_sender:=true`, `detection_sender_node` republishes filtered
detections on `/vehicle_detections` (and optionally POSTs the JSON
payload to the configured `http_endpoint_url`). The HTTP payload schema
is documented in [`payload_schema.md`](payload_schema.md).

When `use_rviz:=true`, RViz is launched with
`rviz/vehicle_detection.rviz`, which subscribes to all of the above
topics with `Fixed Frame: map`.

## Known Limitations

- Detection orientation is identity (axis-aligned). Yaw estimation is
  out of scope for the MVP.
- Confidence is a fixed placeholder until a learned classifier is
  added.
- Ground removal uses a single RANSAC plane constrained to ~15° of the
  z-axis; sloped or multi-level ground may need preprocessing.
