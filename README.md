# Vehicle Detection System

ROS 2 Jazzy / C++ / PCL を使った、PCD点群ベースの普通車検知システム。

## Documents

- 要件定義書: `docs/vehicle_detection_requirements.md`
- 基本設計書: `docs/vehicle_detection_design.md`
- コーディング前チェックリスト: `docs/pre_coding_checklist.md`

## Initial Project Layout

```text
vehicle_detection_system/
  docs/
  src/
  launch/
  config/
  data/
    pcd/
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
