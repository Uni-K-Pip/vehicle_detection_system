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
- [ ] Confirm `colcon build` succeeds

## v0.3.0 - PCD Loader

- [ ] Implement `pcd_loader_node`
- [ ] Load a configured PCD file
- [ ] Publish `sensor_msgs/msg/PointCloud2` to `/input/points`
- [ ] Support one-shot and periodic publishing
- [ ] Add error handling for missing or invalid PCD files

## v0.4.0 - Vehicle Candidate Detection

- [ ] Implement PCL preprocessing
- [ ] Add voxel downsampling
- [ ] Add ROI cropping
- [ ] Add ground removal
- [ ] Add Euclidean clustering
- [ ] Compute 3D bounding boxes
- [ ] Publish `vision_msgs/msg/Detection3DArray`
- [ ] Publish RViz markers

## v0.5.0 - Launch and Visualization

- [ ] Add `vehicle_detection.launch.py`
- [ ] Add static transform setup
- [ ] Add RViz config
- [ ] Confirm `/vehicle_detections` output
- [ ] Capture demo screenshot or GIF

## v0.6.0 - Tests and Quality

- [ ] Add unit tests for parameter validation
- [ ] Add unit tests for point cloud processing helpers
- [ ] Add basic launch verification
- [ ] Document `colcon test` results

## v0.7.0 - External Output

- [ ] Implement `detection_sender_node`
- [ ] Add `send_mode` switching
- [ ] Add HTTP JSON POST
- [ ] Add a small local receiver example
- [ ] Document payload schema

## v1.0.0 - Portfolio MVP

- [ ] Provide a reproducible Docker-based setup
- [ ] Provide one-command launch instructions
- [ ] Provide RViz demo image
- [ ] Provide topic echo example
- [ ] Provide tests and known limitations
- [ ] Confirm no large data files or local notes are tracked
