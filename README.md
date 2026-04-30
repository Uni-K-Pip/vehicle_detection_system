# Vehicle Detection System

ROS 2 Jazzy / C++ / PCL を使った、PCD点群ベースの普通車検知システム。

## Documents

- 要件定義書: `docs/vehicle_detection_requirements.md`
- 基本設計書: `docs/vehicle_detection_design.md`
- コーディング前チェックリスト: `docs/pre_coding_checklist.md`

## Layout

```text
vehicle_detection_system/
  README.md
  ROADMAP.md
  CHANGELOG.md
  LICENSE
  data/pcd/                 # PCD files placed at runtime (not committed)
  docs/                     # requirements, design, checklist
  src/vehicle_detection/    # ament_cmake ROS 2 package
    package.xml
    CMakeLists.txt
    include/vehicle_detection/
    src/
    launch/
    config/                 # detector_params.yaml, dataset_params.yaml, transforms.yaml
    test/
```

## Build

```bash
# inside a ROS 2 Jazzy environment, from this repo root:
colcon build --packages-select vehicle_detection
source install/setup.bash
colcon test --packages-select vehicle_detection
colcon test-result --verbose
```

## Initial Decisions

- 初期データセット: PandaSet由来のPCDサブセット
- 入力座標系: `lidar`
- 出力座標系: `map`
- `lidar -> map`未指定時: identity transform
- 検知対象: 普通車のみ
- 検知情報送信: ROS 2 topic / HTTP POST / both / disabled を設定で切替
- GUI: Qt/rqtを使わないブラウザベースUI

## Coding Readiness

- コーディング開始可。
- 残タスクはPCD実データ配置と正式TF値確認のみ。
- PCD配置時は`data/pcd/README.md`へ取得元、取得日、ライセンス、attributionを記録する。
