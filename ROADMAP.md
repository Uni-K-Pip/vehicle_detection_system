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
「複数PCD連続再生」、「検知結果の保存」、「HTTP payload schema の
バージョン管理」、および「パラメータプリセット管理」を対象とする。
残りの項目 (rosbag 入力、簡易トラッキング) は本リリースには含めず、
後続リリースで取り扱う。

- [x] 複数PCD連続再生 (FR-011)
  - [x] `pcd_loader_node` に再生リスト機構を追加する
        (`pcd_files` / `pcd_directory` / `pcd_glob` / `loop`)
  - [x] 単一PCD再生 (MVP) の挙動を変えない
  - [x] 再生リスト解決のユニットテストを追加する
        (`test_pcd_playlist`)
  - [x] 要件・設計・README・CHANGELOG に追記する
- [x] 検知結果の保存 (FR-012)
  - [x] `detection_sender_node` に検知結果保存パラメータを追加する
        (`save_results` / `result_output_path` / `result_output_format`)
  - [x] HTTP payload と同一構造の JSON Lines を 1 フレーム 1 行で
        append する `DetectionResultWriter` ヘルパーを追加する
  - [x] `save_results=false` の場合は既存挙動を変えない
  - [x] 開けない保存先パスでも `detection_sender_node` をクラッシュ
        させず警告ログを出す
  - [x] `DetectionResultWriter` の単体テストを追加する
        (`test_detection_result_writer`)
  - [x] 要件・設計・CHANGELOG に追記する
- [x] HTTP payload schema のバージョン管理 (FR-013)
  - [x] `serialize_detections` の出力ルートに `schema_version`
        フィールドを追加し、初期値 `"1.0"` を実装側定数として
        一元管理する
  - [x] HTTP POST と JSON Lines 保存の両経路で同じ `schema_version`
        が出力されるようにする (シリアライザ共有)
  - [x] 既存ルートフィールド (`timestamp`、`frame_id`、`detections`)
        とサブフィールドを削除・リネーム・意味変更しない
  - [x] 検知 0 件と検知 1 件以上の payload で `schema_version` が
        出力されることをユニットテストで確認する
  - [x] 要件 (FR-013)、設計、`docs/payload_schema.md` の version
        history、CHANGELOG に追記する
- [x] パラメータプリセット管理 (FR-014)
  - [x] `config/presets/` 配下に読み取り専用プリセット YAML を追加
        (`default`、`pandaset_balanced`、`near_range`)
  - [x] launch 引数 `detector_preset` を追加し、未指定 / `default` で
        既存挙動を維持する
  - [x] 不明なプリセット名で利用可能な一覧を含むエラーで launch を
        失敗させる
  - [x] プリセットの上書き対象は `vehicle_detector_node` の検知
        パラメータに限定し、HTTP 送信、検知結果保存、Web GUI、
        PCD 再生は変えない
  - [x] launch スモークテストに `detector_preset` 引数の宣言確認、
        既知プリセットの解決確認、不明プリセットの拒否確認を追加
  - [x] 要件・設計・README・CHANGELOG に追記する
