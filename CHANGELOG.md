# 変更履歴

本プロジェクトの主な変更点はこのファイルに記録する。

## Unreleased

### Added

- 再現可能なビルド／実行環境のための `osrf/ros:jazzy-desktop` ベース
  `Dockerfile`。`ros-jazzy-vision-msgs`、`ros-jazzy-tf2-sensor-msgs`、
  `python3-colcon-common-extensions` をインストールする。
- `tools/run_demo.sh`: ビルドと launch を 1 コマンドで実行する。
  ROS 2 を source し、`colcon build --merge-install` を実行し、
  install スペースを source した上で、指定された PCD で
  `vehicle_detection.launch.py` を起動する。
- `docs/topic_echo.md`: PandaSet サンプルでの実行から取得した
  `ros2 topic list / echo / hz / info` の出力を記録。
- `docs/limitations.md`: MVP の制約 (単一フレームの幾何検知器、
  identity orientation、HTTP 専用送信、パイプライン処理時間など)
  の明示的な一覧。
- `.gitignore`: データセットの `.mat` とメタデータが再公開されない
  よう、`data/pcd/PandasetLidarData/` も除外対象に追加。
- `rviz/vehicle_detection.rviz`: input・filtered・cluster・vehicle の
  各トピックを `Fixed Frame: map` で表示する RViz コンフィグを同梱。
- `vehicle_detection.launch.py`: `use_sender`、`use_rviz`、`rviz_config`
  の launch 引数。RViz は同梱コンフィグで条件付き起動、
  `detection_sender_node` は params ファイルから取得した `send_mode`
  に応じて条件付き起動。
- `detection_sender_node`: `/vehicle_detections/raw` を購読し、
  `send_mode` が `ros_topic` または `both` のときに
  `/vehicle_detections` (`vision_msgs/Detection3DArray`) へ再 publish し、
  `send_mode` が `http` または `both` のときに `http_endpoint_url` へ
  JSON ペイロードを POST する。HTTP 送信は最大 32 件のキューを持つ
  バックグラウンドワーカースレッドで実行されるため、購読側コールバックを
  ブロックしない。起動時および `on_set_parameters_callback` で
  パラメータを検証する。
- `detection_json` ライブラリ: `vision_msgs/Detection3DArray` の内容を
  ドキュメント化された JSON ペイロード
  (timestamp、frame_id、detections[id, class, confidence, center, size, yaw])
  にシリアライズし、ROS 時刻を ISO 8601 UTC でフォーマットする。
- `http_client` ライブラリ: 接続／送信／受信タイムアウトと
  `parse_http_url` ヘルパーを備えた、最小限の POSIX ソケット
  HTTP/1.1 POST ヘルパー。
- `tools/receive_detections.py`: `http.server` ベースの小さな Python
  受信ツール。HTTP 経路をローカルで検証するためのもの。
- JSON シリアライズと HTTP URL パースの GTest 単体テスト
  (`test_detection_json`)。
- `vehicle_detection.launch.py` の Pytest スモークテスト
  (`test_launch_description`)。launch モジュールの import、想定する
  全 launch 引数の宣言、想定するノードの登録を確認する。
- `docs/results.md`: 確認済みのパイプライン実測値、サンプル検知、
  トピック、既知の制約を記載。
- `docs/payload_schema.md`: HTTP JSON ペイロードのスキーマを記載。
- `docs/images/architecture.svg`: PCD ローダー、検知器、RViz、
  トピックリレー、HTTP JSON 出力フローを描画したアーキテクチャ図。
- `docs/images/rviz_demo_20260501_225537.png`: PandaSet サンプルにおいて
  `/debug/points_filtered` の上に `/vehicle_markers` (緑の AABB) を
  重ねた RViz デモのスクリーンショット。`README.md` と
  `docs/results.md` に埋め込み。
- 初期版の要件定義、設計メモ、コーディング前チェックリスト。
- 初期版の検知器、データセット、transform 用パラメータファイル。
- ROS 2 ビルド成果物および大容量点群データに対する Git の無視ルール。
- ポートフォリオ志向の開発のためのロードマップ。
- `src/vehicle_detection` 配下の `ament_cmake` パッケージひな形。
  `package.xml`、`CMakeLists.txt`、launch および config の install ルール
  を含む。
- `parameter_validation` ライブラリ: 範囲、列挙、非空、ファイル存在の
  各チェックと、それらの GTest 単体テスト。
- `pcd_loader_node`: PCL で PCD を読み込み、`/input/points` に
  `sensor_msgs/msg/PointCloud2` を publish する。`publish_once` と
  `publish_rate_hz` の両モードに対応し、パラメータを検証し、相対 PCD
  パスを作業ディレクトリに対して解決し、起動時に PCD パス・点数・
  フィールド名をログ出力する。ファイルが存在しない、もしくは読み込めない
  場合は明示的なエラーメッセージで初期化を失敗させる。
- `vehicle_detection.launch.py`: `pcd_loader_node` と
  `target_frame_id` -> `input_frame_id` の static transform を起動する。
  launch 引数: `pcd_file`、`input_frame_id`、`target_frame_id`、
  `publish_once`、`params_file`。
- `point_cloud_processing` ライブラリ: PCL ベースのパイプラインヘルパー。
  VoxelGrid ダウンサンプリング、CropBox による ROI フィルタ、
  RANSAC による平面除去、ユークリッドクラスタリング、軸並行 bbox 抽出、
  普通車サイズ判定をまとめている。純粋ヘルパー
  (`compute_box_dimensions`、`is_passenger_vehicle`、`compute_aabb`)
  は GTest で単体テスト済み。
- `vehicle_detector_node`: `/input/points` を購読し、必要に応じて tf2 で
  `target_frame_id` へ変換し、voxel + ROI + 地面除去 + クラスタリング
  + AABB + サイズフィルタを実行し、`/vehicle_detections/raw`
  (`vision_msgs/msg/Detection3DArray`)、`/debug/points_filtered`
  (`sensor_msgs/msg/PointCloud2`)、`/debug/clusters` および
  `/vehicle_markers` (`visualization_msgs/msg/MarkerArray`) を publish する。
  入力／フィルタ後／クラスタ／検知件数とフレームごとの処理時間をログ出力する。
  起動時と `on_set_parameters_callback` でパラメータを検証し、不正な
  実行時更新は理由を添えて拒否する。パラメータは mutex で保護した
  スナップショットから読み出すため、購読側コールバックと並行更新が
  競合しない。
- `vehicle_detection.launch.py`: 既定で `vehicle_detector_node` も起動する
  ようになり、新しい launch 引数 `use_detector` で切り替えられる。
- `parameter_bridge_node`: ブラウザ向けパラメータ GUI を `gui_port`
  (既定 8081) でホストする ROS 2 ノード。`/` で
  `web/parameter_gui.html` を配信し、`GET /api/health`、
  `GET /api/parameters`、`POST /api/parameters` を公開し、
  `rclcpp::AsyncParametersClient` 経由で `pcd_loader_node`、
  `vehicle_detector_node`、`detection_sender_node` の parameter
  サービスへブリッジする。既定で `127.0.0.1` にバインドし、
  意図的にネットワーク公開する場合は `host` パラメータ
  (または `gui_host` launch 引数) で上書きする。`cpp-httplib`
  (MIT、CMake FetchContent で取得) と `nlohmann_json` (rosdep
  `nlohmann-json-dev`) を使用。
- `parameter_json` ヘルパー: `rclcpp::Parameter` と JSON を型対応で
  相互変換する。整数パラメータに対する小数値の範囲チェックも含み、
  未定義のキャストを避ける。GTest 単体テストで網羅。
- `vehicle_detection.launch.py`: 新しい `use_gui` (既定 `true`) と
  `gui_host` (既定 `127.0.0.1`) 引数で、パラメータ GUI を残りの
  パイプラインに組み込む。

### Changed

- `cpp-httplib` の解決はシステムの `httplib >= 0.27.0` を要求するか、
  pin した upstream の `v0.28.0` へフォールバックする方式に変更。
  CMake `FetchContent` フォールバックのため Docker イメージで
  `git` を追加インストールするようにした。
- `parameter_gui.html` は、整数パラメータに対する小数値をブラウザ側で
  暗黙に切り捨てるのではなく、明示的に拒否するようにした。
- `test_launch_description.py` で `use_gui` / `gui_host` の launch 引数と
  `parameter_bridge_node` の launch アクションを検証するようにした。
- `Dockerfile` で追加 ROS 2 パッケージのインストール前に
  `apt-get dist-upgrade` を実行するようにし、ベースの
  `osrf/ros:jazzy-desktop` のライブラリと `vision_msgs` などの
  新規メッセージパッケージとの ABI 互換性を保つようにした。
- `vehicle_detector_node` は、報告する identity orientation に整合する
  よう、`Detection3D.bbox.size` に生の軸並行寸法 (`box.dx() / dy() / dz()`)
  を publish するようにした。普通車サイズフィルタおよび
  `detection_sender_node` の JSON ペイロードでは引き続き、意味付き
  length / width / height (max(dx, dy) / min(dx, dy) / dz) を使用する。
- `point_cloud_processing::remove_ground_plane` は RANSAC を z 軸方向の
  `SACMODEL_PERPENDICULAR_PLANE` (約 15 度の許容) に制約するようになり、
  垂直壁が地面として採択されないようにした。
- `vehicle_detector_node` は、`vehicle_min_length / width / height` の
  いずれかが対応する max 以上であるパラメータ集合を、起動時および
  `on_set_parameters_callback` で拒否するようにした。
- `detection_sender_node` は、HTTP を使う `send_mode` (`http` または
  `both`) に最初に遷移したタイミングで HTTP ワーカーを起動する
  ようになった (以前は構築時のみ)。実行時に `ros_topic` から
  `http` / `both` へ切り替えても、ペイロードが固定キューに
  詰まったままにならない。
- `http_client::http_post_json` は、設定された `http_timeout_ms` を
  `SO_SNDTIMEO` / `SO_RCVTIMEO` に適用するようにした (以前は
  `select()` 後の `tv` に残った接続デッドラインの残りを使用していた)。
- `http_client::send_all` は `send()` に `MSG_NOSIGNAL` を渡すようにし、
  受信側が書き込み中に接続を閉じた場合でも `SIGPIPE` で
  `detection_sender_node` を終了させず、`HttpPostResult` の失敗として
  返すようにした。
- `detection_sender_node` の HTTP ワーカーは、シャットダウン時
  (`stop_flag_`) にキューに残ったペイロードを排出せず破棄するように
  なった。受信側が到達不能な場合、Ctrl+C で
  `(retry_count + 1) * http_timeout_ms` × 残ペイロード数の時間ブロック
  されることがなくなった。

### Moved

- `config/` のインストール元をリポジトリルートから
  `src/vehicle_detection/config/` に変更した。

### Planned

- (なし)
