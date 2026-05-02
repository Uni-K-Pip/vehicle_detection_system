# ロードマップ

本ロードマップは、`vehicle_detection_system` の開発進捗を追跡するためのもの。

## v0.1.0 - 設計ベースライン

- [x] ROS 2 / C++ / PCL ベースの車両検知 MVP の要件を定義する
- [x] 初期ノードの責務を定義する
- [x] ROS 2 トピックとメッセージ型を定義する
- [x] 検知器パラメータを定義する
- [x] 初期データセットと PCD 配置ポリシーを定義する
- [x] Git 追跡用ファイルを整備する

## v0.2.0 - ROS 2 パッケージひな形

- [x] `ament_cmake` パッケージとして `src/vehicle_detection` を作成する
- [x] `package.xml` を追加する
- [x] `CMakeLists.txt` を追加する
- [x] launch と config の install ルールを追加する
- [x] `colcon build` が成功することを確認する

## v0.3.0 - PCD ローダー

- [x] `pcd_loader_node` を実装する
- [x] 設定された PCD ファイルを読み込む
- [x] `sensor_msgs/msg/PointCloud2` を `/input/points` に publish する
- [x] one-shot と周期 publish の両方をサポートする
- [x] PCD ファイルが存在しない／不正な場合のエラー処理を追加する
- [x] `target_frame_id` -> `input_frame_id` の static transform を持つ `vehicle_detection.launch.py` を追加する

## v0.4.0 - 車両候補の検知

- [x] PCL の前処理を実装する
- [x] voxel ダウンサンプリングを追加する
- [x] ROI クロップを追加する
- [x] 地面除去を追加する
- [x] ユークリッドクラスタリングを追加する
- [x] 3D バウンディングボックスを計算する
- [x] `vision_msgs/msg/Detection3DArray` を publish する
- [x] RViz マーカーを publish する

## v0.5.0 - launch と可視化

- [x] `vehicle_detection.launch.py` を追加する
- [x] static transform の設定を追加する
- [x] RViz コンフィグを追加する
- [x] `/vehicle_detections` の出力を確認する
- [x] デモ用スクリーンショットまたは GIF を撮影する
      (`docs/images/rviz_demo_20260501_225537.png` を `README.md` と
      `docs/results.md` に埋め込み)

## v0.6.0 - テストと品質

- [x] パラメータ検証の単体テストを追加する
- [x] 点群処理ヘルパーの単体テストを追加する
- [x] 基本的な launch 確認を追加する
- [x] `colcon test` の結果を記録する

## v0.7.0 - 外部出力

- [x] `detection_sender_node` を実装する
- [x] `send_mode` 切り替えを追加する
- [x] HTTP JSON POST を追加する
- [x] 小さなローカル受信サンプルを追加する
- [x] ペイロードスキーマを文書化する

## v0.7.x - ブラウザ GUI パラメータブリッジ

- [x] `parameter_bridge_node` (HTTP サーバ) を実装する
- [x] `/` で静的 `parameter_gui.html` を配信する
- [x] `GET /api/health` を公開する
- [x] 設定対象ノードに対する `GET /api/parameters` を公開する
- [x] 型変換付き `POST /api/parameters` を公開する
- [x] `use_gui` launch 引数を追加する
- [x] `parameter_json` の往復変換に対する単体テストを追加する

## v1.0.0 - MVP

- [x] Docker ベースで再現可能なセットアップを提供する
      (`osrf/ros:jazzy-desktop` をベースとした `Dockerfile`)
- [x] 1 コマンドで起動できる手順を提供する
      (`./tools/run_demo.sh`、README にも記載)
- [x] RViz デモ画像を提供する
      (`docs/images/rviz_demo_20260501_225537.png`)
- [x] アーキテクチャ図を提供する
      (`docs/images/architecture.svg`)
- [x] トピック echo の実行例を提供する (`docs/topic_echo.md`)
- [x] テストと既知の制約を提供する (`docs/results.md`、
      `docs/limitations.md`。144 tests、0 failures、18 skipped)
- [x] 大きなデータファイルやローカルメモが追跡されていないことを確認する
      (`.gitignore` で PCD、`data/pcd/PandasetLidarData/`、
      ビルド成果物、`*_LOCAL.md` メモを除外済み)

## v1.1.0 - Phase 2 実用性改善 (進行中)

要件定義書 12 章「Phase 2: 実用性改善」項目のうち、本リリースでは
「複数PCD連続再生」のみを対象とする。残りの項目 (rosbag 入力、検知結果
保存、簡易トラッキング、パラメータプリセット、HTTP payload schema の
バージョン管理) は本リリースには含めず、後続リリースで取り扱う。

- [ ] 複数PCD連続再生 (FR-011)
  - [ ] `pcd_loader_node` に再生リスト機構を追加する
        (`pcd_files` / `pcd_directory` / `pcd_glob` / `loop`)
  - [ ] 単一PCD再生 (MVP) の挙動を変えない
  - [ ] 再生リスト解決のユニットテストを追加する
  - [ ] 要件・設計・README・CHANGELOG に追記する
