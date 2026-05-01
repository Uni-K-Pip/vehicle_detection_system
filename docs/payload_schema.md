# HTTP Payload Schema

`detection_sender_node` POSTs the following JSON body to
`http_endpoint_url` whenever `send_mode` is `http` or `both`. Each POST
corresponds to a single `vision_msgs/msg/Detection3DArray` message
received on `raw_detections_topic`.

```
POST {http_endpoint_url}
Content-Type: application/json
```

## Body

```jsonc
{
  "timestamp": "2026-04-30T12:34:56.123Z",  // ISO 8601 UTC, ms precision
  "frame_id": "map",                         // header.frame_id
  "detections": [
    {
      "id": "1",                             // Detection3D.id
      "class": "car",                        // first hypothesis.class_id
      "confidence": 0.8,                     // first hypothesis.score
      "center": {                            // bbox.center.position
        "x": 20.29,
        "y": -9.41,
        "z": -0.95
      },
      "size": {                              // semantic length/width/height
        "length": 4.35,                      //   max(dx, dy)
        "width": 2.54,                       //   min(dx, dy)
        "height": 1.79                       //   dz
      },
      "yaw": 0.0                             // identity orientation
    }
  ]
}
```

## Notes

- `detections` may be an empty array if the upstream
  `vehicle_detector_node` produced no passenger-vehicle candidates for
  the current frame.
- Numeric fields use 6-digit fixed precision; non-finite values are
  serialized as `0.0`.
- `class` and `confidence` reflect the first
  `ObjectHypothesisWithPose` in `Detection3D.results`. The MVP
  hard-codes `class=car` and `confidence=0.8`.
- `yaw` is currently always `0.0`; AABB orientation estimation is out
  of scope for the MVP.
- The sender expects any 2xx response. A non-2xx, a network error, or a
  timeout is logged but does not block the ROS callback (sending runs
  on a background worker thread with a bounded queue of 32 payloads).
