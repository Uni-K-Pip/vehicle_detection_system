# トピック echo の実行例

ここに示す出力は、`osrf/ros:jazzy-desktop` Docker イメージ内で
`data/pcd/PandasetLidarData/Lidar/0001.pcd` に対して
`vehicle_detection.launch.py` をライブ実行した際に取得したもの。

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

## /vehicle_detections/raw — フレーム内の先頭検知

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

`bbox.size` は生の軸並行寸法 (`dx`、`dy`、`dz`) を返す。
`detection_sender_node` が送信する JSON ペイロードでは、これらを
意味付きの length / width / height に並べ替える
([`payload_schema.md`](payload_schema.md) 参照)。

## /vehicle_detections/raw — メッセージ周期

```
$ ros2 topic hz /vehicle_detections/raw
average rate: 1.000
        min: 0.994s max: 1.006s std dev: 0.00413s window: 5
```

これは `pcd_loader_node` の既定値 `publish_rate_hz: 1.0` に一致する。

## /vehicle_markers

```
$ ros2 topic echo --once /vehicle_markers --no-arr
markers: '<sequence type: visualization_msgs/msg/Marker, length: 8>'
```

このマーカー配列は、検知 1 件ごとの cube マーカー 7 個と、
RViz がフレーム間で古い box を消すための `DELETEALL` マーカー 1 個で
構成される。

## QoS

`/vehicle_detections/raw` と `/vehicle_detections` は
`KeepLast(10) + RELIABLE + VOLATILE` を使用する:

```
$ ros2 topic info /vehicle_detections/raw -v
QoS profile:
  Reliability: RELIABLE
  History (Depth): KEEP_LAST 10
  Durability: VOLATILE
```
