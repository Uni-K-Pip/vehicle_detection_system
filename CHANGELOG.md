# Changelog

All notable changes to this project will be documented in this file.

## Unreleased

### Added

- Initial requirements, design notes, and pre-coding checklist.
- Initial detector, dataset, and transform parameter files.
- Git ignore rules for ROS 2 build outputs and large point cloud data.
- Roadmap for portfolio-oriented development.
- `ament_cmake` package scaffold under `src/vehicle_detection` with
  `package.xml`, `CMakeLists.txt`, and install rules for launch and config.
- `parameter_validation` library with range, enum, non-empty, and
  existing-file checks plus GTest unit tests.
- `pcd_loader_node`: loads a PCD via PCL and publishes
  `sensor_msgs/msg/PointCloud2` on `/input/points`. Supports `publish_once`
  and `publish_rate_hz` modes, validates parameters, resolves relative PCD
  paths against the working directory, and logs PCD path, point count, and
  field names on startup. Missing or unreadable files cause node init to
  fail with an explicit error message.
- `vehicle_detection.launch.py`: starts `pcd_loader_node` and a static
  `target_frame_id` -> `input_frame_id` transform. Launch arguments:
  `pcd_file`, `input_frame_id`, `target_frame_id`, `publish_once`,
  `params_file`.

### Moved

- `config/` is now installed from `src/vehicle_detection/config/` rather
  than the repository root.

### Planned

- PCL-based vehicle candidate detector.
- RViz visualization.
- HTTP detection sender and Web GUI parameter bridge.
