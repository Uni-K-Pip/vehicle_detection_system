# Vehicle Detection System

ROS 2 Jazzy / C++ / PCL を使った、PCD点群ベースの普通車検知システム。

## Documents

- 要件定義書: [`docs/vehicle_detection_requirements.md`](docs/vehicle_detection_requirements.md)
- 基本設計書: [`docs/vehicle_detection_design.md`](docs/vehicle_detection_design.md)
- HTTP payload schema: [`docs/payload_schema.md`](docs/payload_schema.md)
- Verified pipeline metrics: [`docs/results.md`](docs/results.md)
- Live `ros2 topic echo` examples: [`docs/topic_echo.md`](docs/topic_echo.md)
- Known limitations: [`docs/limitations.md`](docs/limitations.md)
- コーディング前チェックリスト: [`docs/pre_coding_checklist.md`](docs/pre_coding_checklist.md)

## Architecture

![Vehicle Detection System Architecture](docs/images/architecture.svg)

The diagram above summarizes the ROS 2 node flow from PCD loading
through PCL-based vehicle detection, RViz marker output, ROS topic
relay, and optional HTTP JSON publishing. The full design notes are in
[`docs/vehicle_detection_design.md`](docs/vehicle_detection_design.md).

## Layout

```text
vehicle_detection_system/
  README.md
  ROADMAP.md
  CHANGELOG.md
  LICENSE
  Dockerfile                # reproducible build image (osrf/ros:jazzy-desktop)
  data/pcd/                 # PCD files placed at runtime (not committed)
  docs/                     # requirements, design, results, schemas, limits, images
  src/vehicle_detection/    # ament_cmake ROS 2 package
    package.xml
    CMakeLists.txt
    include/vehicle_detection/
    src/
    launch/
    config/                 # detector_params.yaml, dataset_params.yaml, transforms.yaml
    rviz/                   # vehicle_detection.rviz
    test/
  tools/
    run_demo.sh             # one-command build + launch
    receive_detections.py   # local HTTP receiver for the JSON payload
```

## Quick Start (Docker)

The reference environment is `osrf/ros:jazzy-desktop`. The bundled
[`Dockerfile`](Dockerfile) installs the additional ROS 2 packages
needed by this repo (`vision_msgs`, `tf2_sensor_msgs`,
`colcon-common-extensions`).

```bash
# 1) build the image
docker build -t vehicle_detection:dev .

# 2) place a PCD at data/pcd/sample.pcd (see data/pcd/README.md)

# 3) launch the pipeline (one command)
docker run --rm -it \
  -v "$PWD":/workspace/vehicle_detection_system \
  -w /workspace/vehicle_detection_system \
  vehicle_detection:dev \
  ./tools/run_demo.sh data/pcd/sample.pcd
```

`run_demo.sh` sources ROS 2, runs `colcon build --merge-install
--packages-select vehicle_detection`, sources the install space, and
launches `vehicle_detection.launch.py`.

## Quick Start (host ROS 2)

```bash
# inside a ROS 2 Jazzy environment, from this repo root:
colcon build --merge-install --packages-select vehicle_detection
source install/setup.bash

ros2 launch vehicle_detection vehicle_detection.launch.py \
  pcd_file:=data/pcd/sample.pcd
```

Override behaviour with launch args:

```bash
ros2 launch vehicle_detection vehicle_detection.launch.py \
  pcd_file:=/abs/path/to/cloud.pcd \
  input_frame_id:=lidar \
  target_frame_id:=map \
  publish_once:=true \
  use_sender:=true \
  use_rviz:=true \
  use_gui:=true
```

`use_sender:=true` adds `detection_sender_node` (republish on
`/vehicle_detections` and / or POST JSON per
[`docs/payload_schema.md`](docs/payload_schema.md)). `use_rviz:=true`
opens RViz with the bundled config in
`src/vehicle_detection/rviz/vehicle_detection.rviz`. `use_gui:=true`
(default) starts `parameter_bridge_node` for runtime parameter tuning;
see [Browser GUI](#browser-gui).

## Confirming the pipeline

```bash
ros2 topic list
# /input/points  /vehicle_detections/raw  /vehicle_markers  ...

ros2 topic echo --once /vehicle_detections/raw
ros2 topic hz /vehicle_detections/raw
```

A captured run with the PandaSet sample is recorded in
[`docs/topic_echo.md`](docs/topic_echo.md) and
[`docs/results.md`](docs/results.md).

## RViz demo

With `use_rviz:=true`, RViz subscribes to
`/input/points`, `/debug/points_filtered`, `/debug/clusters`, and
`/vehicle_markers` against `Fixed Frame: map`. Recorded outputs and
counts are summarized in [`docs/results.md`](docs/results.md).
A captured screenshot belongs alongside that document; the run is
fully reproducible via `./tools/run_demo.sh data/pcd/sample.pcd`.

## Browser GUI

`parameter_bridge_node` exposes a small Web UI for tuning ROS 2 parameters
at runtime. The HTTP API is unauthenticated, so it binds to loopback
(`127.0.0.1:8081`) by default and is reachable only from the same machine.

```bash
# stop the GUI for a launch run:
ros2 launch vehicle_detection vehicle_detection.launch.py use_gui:=false
```

To expose the GUI from a Docker container to the Windows host, both bind
on `0.0.0.0` and publish the port:

```bash
# inside the container
ros2 launch vehicle_detection vehicle_detection.launch.py gui_host:=0.0.0.0

# when starting the container, publish the port:
docker run -p 8081:8081 ...
```

`gui_port` and `host` are also settable through `detector_params.yaml` or
`ros2 param set /parameter_bridge_node host 0.0.0.0` if preferred. Treat
non-loopback bindings as a deliberate exposure of runtime parameter writes.

HTTP API exposed by the bridge:

| Method | Path              | Body / Response                                                |
| ------ | ----------------- | -------------------------------------------------------------- |
| GET    | `/`               | `parameter_gui.html`                                           |
| GET    | `/api/health`     | `{ ok, node, target_nodes }`                                   |
| GET    | `/api/parameters` | `{ ok, nodes: [{ name, available, parameters }] }`             |
| POST   | `/api/parameters` | `{ node, parameters: { ... } }` -> `{ ok, updated, rejected }` |

Implementation libraries: [cpp-httplib](https://github.com/yhirose/cpp-httplib)
(MIT, fetched via CMake FetchContent) and
[nlohmann/json](https://github.com/nlohmann/json) (MIT, rosdep key
`nlohmann-json-dev`).

## Tests

```bash
colcon test --merge-install --packages-select vehicle_detection
colcon test-result --verbose --test-result-base build/vehicle_detection
```

Latest run: **117 tests, 0 errors, 0 failures, 14 skipped**
(see [`docs/results.md`](docs/results.md)).

## Initial Decisions

- 初期データセット: PandaSet由来のPCDサブセット
- 入力座標系: `lidar`
- 出力座標系: `map`
- `lidar -> map`未指定時: identity transform
- 検知対象: 普通車のみ
- 検知情報送信: ROS 2 topic / HTTP POST / both / disabled を設定で切替
- GUI: Qt/rqtを使わないブラウザベースUI

## Known Limitations

See [`docs/limitations.md`](docs/limitations.md) for the full list
(single-frame geometric detector, identity orientation, HTTP-only
sender, 1 Hz / 150 ms per-frame target, etc.).
