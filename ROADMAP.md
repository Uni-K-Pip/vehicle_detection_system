# Roadmap

This roadmap tracks the growth of `vehicle_detection_system` as a portfolio project.

## v0.1.0 - Design Baseline

- [x] Define requirements for a ROS 2 / C++ / PCL based vehicle detection MVP
- [x] Define initial node responsibilities
- [x] Define ROS 2 topics and message types
- [x] Define detector parameters
- [x] Define initial dataset and PCD placement policy
- [x] Prepare Git tracking files

## v0.2.0 - ROS 2 Package Scaffold

- [x] Create `src/vehicle_detection` as an `ament_cmake` package
- [x] Add `package.xml`
- [x] Add `CMakeLists.txt`
- [x] Add launch and config install rules
- [x] Confirm `colcon build` succeeds

## v0.3.0 - PCD Loader

- [x] Implement `pcd_loader_node`
- [x] Load a configured PCD file
- [x] Publish `sensor_msgs/msg/PointCloud2` to `/input/points`
- [x] Support one-shot and periodic publishing
- [x] Add error handling for missing or invalid PCD files
- [x] Add `vehicle_detection.launch.py` with static `target_frame_id` -> `input_frame_id` transform

## v0.4.0 - Vehicle Candidate Detection

- [x] Implement PCL preprocessing
- [x] Add voxel downsampling
- [x] Add ROI cropping
- [x] Add ground removal
- [x] Add Euclidean clustering
- [x] Compute 3D bounding boxes
- [x] Publish `vision_msgs/msg/Detection3DArray`
- [x] Publish RViz markers

## v0.5.0 - Launch and Visualization

- [x] Add `vehicle_detection.launch.py`
- [x] Add static transform setup
- [x] Add RViz config
- [x] Confirm `/vehicle_detections` output
- [ ] Capture demo screenshot or GIF (manual: launch with
      `use_rviz:=true`; recorded numbers and frame contents are
      already in `docs/results.md` and `docs/topic_echo.md`)

## v0.6.0 - Tests and Quality

- [x] Add unit tests for parameter validation
- [x] Add unit tests for point cloud processing helpers
- [x] Add basic launch verification
- [x] Document `colcon test` results

## v0.7.0 - External Output

- [x] Implement `detection_sender_node`
- [x] Add `send_mode` switching
- [x] Add HTTP JSON POST
- [x] Add a small local receiver example
- [x] Document payload schema

## v0.7.x - Browser GUI Parameter Bridge

- [x] Implement `parameter_bridge_node` (HTTP server)
- [x] Serve static `parameter_gui.html` at `/`
- [x] Expose `GET /api/health`
- [x] Expose `GET /api/parameters` for the configured target nodes
- [x] Expose `POST /api/parameters` with type-aware coercion
- [x] Add `use_gui` launch argument
- [x] Add unit tests for `parameter_json` round-trips

## v1.0.0 - Portfolio MVP

- [x] Provide a reproducible Docker-based setup
      (`Dockerfile` based on `osrf/ros:jazzy-desktop`)
- [x] Provide one-command launch instructions
      (`./tools/run_demo.sh`, also documented in README)
- [ ] Provide RViz demo image (manual capture step; pipeline and
      RViz config are reproducible)
- [x] Provide architecture diagram image
      (`docs/images/architecture.svg`)
- [x] Provide topic echo example (`docs/topic_echo.md`)
- [x] Provide tests and known limitations (`docs/results.md`,
      `docs/limitations.md`; 117 tests, 0 failures)
- [x] Confirm no large data files or local notes are tracked
      (`.gitignore` covers PCDs, `data/pcd/PandasetLidarData/`,
      build artifacts, and `*_LOCAL.md` notes)
