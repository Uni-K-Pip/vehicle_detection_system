# Topic Echo Examples

These outputs were captured from a live run of
`vehicle_detection.launch.py` against
`data/pcd/PandasetLidarData/Lidar/0001.pcd` inside the
`osrf/ros:jazzy-desktop` Docker image.

## ros2 topic list

```
$ ros2 topic list
/debug/clusters
/debug/points_filtered
/input/points
/parameter_events
/rosout
/tf
/tf_static
/vehicle_detections/raw
/vehicle_markers
```

## /input/points

```
$ ros2 topic info /input/points
Type: sensor_msgs/msg/PointCloud2

Publisher count: 1
Subscription count: 1
```

## /vehicle_detections/raw — first detection in a frame

```
$ ros2 topic echo --once /vehicle_detections/raw
header:
  stamp: { sec: 1777596279, nanosec: 683151998 }
  frame_id: map
detections:
- header:
    stamp: { sec: 1777596279, nanosec: 683151998 }
    frame_id: map
  results:
  - hypothesis: { class_id: car, score: 0.8 }
    pose:
      pose:
        position: { x: 20.286, y: -9.406, z: -0.949 }
        orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
  bbox:
    center:
      position: { x: 20.286, y: -9.406, z: -0.949 }
      orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
    size: { x: 4.346, y: 2.544, z: 1.786 }
  id: '1'
- ...  # 6 more detections in this frame
```

`bbox.size` reports raw axis-aligned extents (`dx`, `dy`, `dz`); the
JSON payload sent by `detection_sender_node` reorders these into
semantic length / width / height (see
[`payload_schema.md`](payload_schema.md)).

## /vehicle_detections/raw — message rate

```
$ ros2 topic hz /vehicle_detections/raw
average rate: 1.000
        min: 0.994s max: 1.006s std dev: 0.00413s window: 5
```

This matches `pcd_loader_node`'s default `publish_rate_hz: 1.0`.

## /vehicle_markers

```
$ ros2 topic echo --once /vehicle_markers --no-arr
markers: '<sequence type: visualization_msgs/msg/Marker, length: 8>'
```

The marker array contains 7 cube markers (one per detection) plus a
`DELETEALL` marker so RViz clears stale boxes between frames.

## QoS

`/vehicle_detections/raw` and `/vehicle_detections` use
`KeepLast(10) + RELIABLE + VOLATILE`:

```
$ ros2 topic info /vehicle_detections/raw -v
QoS profile:
  Reliability: RELIABLE
  History (Depth): KEEP_LAST 10
  Durability: VOLATILE
```
