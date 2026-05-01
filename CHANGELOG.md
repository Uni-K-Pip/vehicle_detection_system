# Changelog

All notable changes to this project will be documented in this file.

## Unreleased

### Added

- `Dockerfile` based on `osrf/ros:jazzy-desktop` for a reproducible
  build / run environment. Installs `ros-jazzy-vision-msgs`,
  `ros-jazzy-tf2-sensor-msgs`, and `python3-colcon-common-extensions`.
- `tools/run_demo.sh`: one-command build + launch. Sources ROS 2,
  runs `colcon build --merge-install`, sources the install space,
  and launches `vehicle_detection.launch.py` with the supplied PCD.
- `docs/topic_echo.md`: captured `ros2 topic list / echo / hz / info`
  output from a live run against the PandaSet sample.
- `docs/limitations.md`: explicit list of MVP limitations (single
  frame geometric detector, identity orientation, HTTP-only sender,
  pipeline timing, etc.).
- `.gitignore`: now also excludes `data/pcd/PandasetLidarData/` so
  the dataset's `.mat` and metadata files cannot be republished.
- `rviz/vehicle_detection.rviz`: bundled RViz config showing input,
  filtered, cluster, and vehicle topics with `Fixed Frame: map`.
- `vehicle_detection.launch.py`: `use_sender`, `use_rviz`, and
  `rviz_config` launch arguments. RViz is started conditionally with
  the bundled config; `detection_sender_node` is started conditionally
  with `send_mode` taken from the params file.
- `detection_sender_node`: subscribes to `/vehicle_detections/raw`,
  republishes on `/vehicle_detections` (`vision_msgs/Detection3DArray`)
  when `send_mode` is `ros_topic` or `both`, and POSTs a JSON payload
  to `http_endpoint_url` when `send_mode` is `http` or `both`. HTTP
  sending runs on a background worker thread with a bounded queue
  (max 32 payloads) so it cannot block the subscriber callback.
  Validates parameters at startup and via
  `on_set_parameters_callback`.
- `detection_json` library: serializes `vision_msgs/Detection3DArray`
  contents into the documented JSON payload (timestamp, frame_id,
  detections[id, class, confidence, center, size, yaw]) and formats
  ROS time as ISO 8601 UTC.
- `http_client` library: minimal POSIX-socket HTTP/1.1 POST helper
  with connect / send / recv timeouts and a `parse_http_url` helper.
- `tools/receive_detections.py`: small Python receiver based on
  `http.server` for verifying the HTTP path locally.
- GTest unit tests for JSON serialization and HTTP URL parsing
  (`test_detection_json`).
- Pytest smoke test for `vehicle_detection.launch.py`
  (`test_launch_description`) verifying the launch module imports,
  declares all expected launch arguments, and registers the expected
  nodes.
- `docs/results.md`: documents verified pipeline metrics, sample
  detections, topics, and known limitations.
- `docs/payload_schema.md`: documents the HTTP JSON payload schema.
- `docs/images/architecture.svg`: rendered architecture diagram for
  the PCD loader, detector, RViz, topic relay, and HTTP JSON output
  flow.
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
- `point_cloud_processing` library: PCL-backed pipeline helpers for
  VoxelGrid downsampling, CropBox ROI filtering, RANSAC plane removal,
  Euclidean clustering, axis-aligned bbox extraction, and passenger-car
  size classification. Pure helpers (`compute_box_dimensions`,
  `is_passenger_vehicle`, `compute_aabb`) are unit-tested with GTest.
- `vehicle_detector_node`: subscribes to `/input/points`, transforms to
  `target_frame_id` via tf2 when needed, runs voxel + ROI + ground
  removal + clustering + AABB + size filter, and publishes
  `/vehicle_detections/raw` (`vision_msgs/msg/Detection3DArray`),
  `/debug/points_filtered` (`sensor_msgs/msg/PointCloud2`),
  `/debug/clusters` and `/vehicle_markers`
  (`visualization_msgs/msg/MarkerArray`). Logs input / filtered /
  cluster / detection counts and per-frame processing time. Validates
  parameters at startup and via `on_set_parameters_callback` so invalid
  runtime updates are rejected with a reason. Reads parameters under a
  mutex-guarded snapshot so concurrent updates do not race the
  subscriber callback.
- `vehicle_detection.launch.py`: now also starts `vehicle_detector_node`
  by default. New `use_detector` launch argument toggles it.
- `parameter_bridge_node`: ROS 2 node hosting the browser-based
  parameter GUI on `gui_port` (default 8081). Serves
  `web/parameter_gui.html` at `/`, exposes `GET /api/health`,
  `GET /api/parameters`, and `POST /api/parameters`, and bridges to
  the parameter services of `pcd_loader_node`,
  `vehicle_detector_node`, and `detection_sender_node` via
  `rclcpp::AsyncParametersClient`. Binds to `127.0.0.1` by default;
  override via the `host` parameter (or the `gui_host` launch
  argument) when intentionally exposing the GUI to a network. Uses
  `cpp-httplib` (MIT, fetched via CMake FetchContent) and
  `nlohmann_json` (rosdep `nlohmann-json-dev`).
- `parameter_json` helpers: convert between `rclcpp::Parameter` and
  JSON with type-aware coercion, including a range check on
  floating-point values targeting integer parameters to avoid
  undefined casts. Covered by GTest unit tests.
- `vehicle_detection.launch.py`: new `use_gui` (default `true`) and
  `gui_host` (default `127.0.0.1`) arguments wire the parameter GUI
  into the rest of the pipeline.

### Changed

- `cpp-httplib` resolution now requires a system `httplib >= 0.27.0`
  or falls back to pinned upstream `v0.28.0`; the Docker image now
  installs `git` for the CMake `FetchContent` fallback.
- `parameter_gui.html` now rejects fractional values for integer
  parameters instead of silently truncating them in the browser.
- `test_launch_description.py` now verifies the `use_gui` / `gui_host`
  launch arguments and the `parameter_bridge_node` launch action.
- `Dockerfile` now runs `apt-get dist-upgrade` before installing
  additional ROS 2 packages so the base `osrf/ros:jazzy-desktop`
  libraries stay ABI-compatible with newly installed message packages
  such as `vision_msgs`.
- `vehicle_detector_node` now publishes raw axis-aligned extents
  (`box.dx() / dy() / dz()`) in `Detection3D.bbox.size` to match the
  identity orientation it reports. Semantic length / width / height
  (max(dx, dy) / min(dx, dy) / dz) are still used for the passenger-
  vehicle size filter and for the JSON payload sent by
  `detection_sender_node`.
- `point_cloud_processing::remove_ground_plane` constrains RANSAC to
  `SACMODEL_PERPENDICULAR_PLANE` along the z-axis with a ~15 deg
  tolerance, so vertical walls are no longer accepted as ground.
- `vehicle_detector_node` now rejects parameter sets where any of
  `vehicle_min_length / width / height` is greater than or equal to
  the corresponding max, both at startup and via the
  `on_set_parameters_callback`.
- `detection_sender_node` now starts the HTTP worker on the first
  transition to a `send_mode` that uses HTTP (`http` or `both`),
  instead of only at construction. Switching from `ros_topic` to
  `http`/`both` at runtime no longer leaves payloads stuck in the
  bounded queue.
- `http_client::http_post_json` now applies the configured
  `http_timeout_ms` to `SO_SNDTIMEO` / `SO_RCVTIMEO` rather than the
  remainder of the connect deadline left in `tv` after `select()`.
- `http_client::send_all` now passes `MSG_NOSIGNAL` to `send()` so a
  receiver that closes the connection mid-write surfaces as a failed
  `HttpPostResult` instead of killing `detection_sender_node` with
  `SIGPIPE`.
- `detection_sender_node` HTTP worker now drops any queued payloads
  on shutdown (`stop_flag_`) instead of draining them. Ctrl+C no
  longer blocks for `(retry_count + 1) * http_timeout_ms` per queued
  POST when the receiver is unreachable.

### Moved

- `config/` is now installed from `src/vehicle_detection/config/` rather
  than the repository root.

### Planned

- Demo screenshot or GIF for `docs/`.
