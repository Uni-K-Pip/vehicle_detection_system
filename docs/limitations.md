# Known Limitations

This MVP intentionally trades feature breadth for clarity. The
following limits are known and documented so reviewers can reason
about the scope before reading the code.

## Detection model

- **Single-frame, geometric detector.** No tracking, temporal
  smoothing, or motion estimation. Each frame is processed
  independently.
- **Classifier is a placeholder.** Every cluster that passes the
  passenger-vehicle size filter is published with
  `class=car, score=0.8`. There is no learned classifier; the size
  filter is the only label gate.
- **Identity orientation only.** AABB-based detection is published
  with an identity quaternion, and the JSON payload reports
  `yaw=0.0`. Oriented bounding boxes (yaw estimation) are out of
  scope for the MVP.
- **Ground removal assumes a near-horizontal ground.** RANSAC is
  constrained to `SACMODEL_PERPENDICULAR_PLANE` along z with a
  ~15° tolerance. Sloped or multi-level ground may need
  preprocessing or a different ground model.

## Pipeline / runtime

- **Single-threaded executor.** `rclcpp::spin` on the default
  executor. The node is sized for ~1 Hz LiDAR frames at 100k–200k
  points; high-rate streams have not been tuned.
- **Per-frame processing time ≈ 150–170 ms** on the recorded
  PandaSet sample (118,784 input points → 9,627 filtered →
  66 clusters → 7 vehicle detections). See
  [`results.md`](results.md).
- **No batching, no zero-copy.** PointCloud2 messages are
  serialized through normal DDS publish/subscribe. Suitable for a
  portfolio demo, not for a production AV stack.

## Coordinate frames / transforms

- The launch file ships an identity static transform from
  `target_frame_id` to `input_frame_id` so the demo runs without
  measured extrinsics. Real deployments must replace this with
  measured values via `tf2_ros::static_transform_publisher`.
- TF lookups use the message timestamp with a 0.1 s timeout. Long
  pipeline stalls or out-of-order timestamps drop frames.

## External output

- `detection_sender_node` HTTP path is **HTTP/1.1 plain only**.
  No TLS / HTTPS, no certificate validation, no auth headers
  beyond a placeholder `http_auth_type=none`.
- The bounded queue caps at 32 payloads. When the receiver is
  slow, the **oldest** payload is dropped (FIFO) with a throttled
  warning log; loss is not reported back to the publisher.

## Build / data

- Builds and runs are verified inside the
  `osrf/ros:jazzy-desktop` image only. Native Windows / macOS
  ROS 2 is out of scope.
- PCD files are not committed (see [`.gitignore`](../.gitignore)).
  `data/pcd/README.md` documents how to obtain the PandaSet
  subset used in the recorded results.
