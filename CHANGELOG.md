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

### Moved

- `config/` is now installed from `src/vehicle_detection/config/` rather
  than the repository root.

### Planned

- PCD loader node.
- PCL-based vehicle candidate detector.
- RViz visualization.
- Launch verification.
