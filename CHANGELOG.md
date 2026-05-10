# 変更履歴

本プロジェクトの主な変更点はこのファイルに記録する。

## [Unreleased]

Phase 2 実用性改善のうち「複数PCD連続再生」(FR-011)、「検知結果の保存」
(FR-012)、「HTTP payload schema のバージョン管理」(FR-013)、
「パラメータプリセット管理」(FR-014)、および「rosbag 入力対応」(FR-015)
を対象とする。残りの Phase 2 項目 (簡易トラッキング) は本範囲外。

### Added

- rosbag 入力対応を追加 (FR-015)。launch 引数 `input_mode` (string、
  初期値 `pcd`) で入力源を `pcd` / `rosbag` から切り替えられる。
  `input_mode:=pcd` (既定) または未指定時は MVP / FR-011 と同じく
  `pcd_loader_node` で PCD を再生する (既存挙動を変更しない)。
  `input_mode:=rosbag` のときは `pcd_loader_node` を起動せず、代わりに
  `ros2 bag play <rosbag_path> --rate <rosbag_rate> [--loop]
  [--remap <rosbag_topic>:=/input/points]` を `ExecuteProcess` として
  起動する。検知パイプライン (`vehicle_detector_node`、
  `detection_sender_node`、`parameter_bridge_node`、static transform、
  RViz) は `input_mode` の影響を受けず、`/input/points` を購読する
  経路はそのまま。
- launch 引数 `rosbag_path` (string、初期値 `""`)、`rosbag_topic`
  (string、初期値 `""`)、`rosbag_loop` (bool、初期値 `false`)、
  `rosbag_rate` (string、初期値 `1.0`) を追加。`rosbag_path` は
  `input_mode:=rosbag` のときに必須で、相対パスは `pcd_file` と同じく
  `Path.cwd()` を基準にして絶対パス化される。`rosbag_topic` が非空の
  ときに `--remap <rosbag_topic>:=/input/points` を組み立てる。
- `vehicle_detection.launch.py` に `_validate_input_mode_inputs()` と
  `_build_rosbag_play_command()` の純粋ヘルパー、および
  `_validate_input_mode` / `_build_rosbag_player` の OpaqueFunction を
  追加。`input_mode` が `pcd` / `rosbag` 以外、`input_mode=rosbag` で
  `rosbag_path` が空または存在しないパスのときに、原因を含む
  `RuntimeError` を投げて launch を失敗させる。
- `pcd_loader_node` を `LaunchConfigurationEquals('input_mode', 'pcd')`
  で gate して、`input_mode:=rosbag` のときに起動しないようにした。
  これにより rosbag 再生中に `/input/points` への二重 publish が
  発生しない。
- `test_launch_description.py` に FR-015 用の単体テストを追加:
  `input_mode` の既定値 `pcd` 確認、`rosbag_path` / `rosbag_topic` /
  `rosbag_loop` / `rosbag_rate` の既定値が inert であること、
  `_validate_input_mode_inputs` の正常系 (`pcd`、有効な rosbag パス)
  と異常系 (未知の `input_mode`、空の `rosbag_path`、存在しない
  `rosbag_path`)、`_build_rosbag_play_command` の最小構成・`--loop` /
  `--rate` / `--remap` 付与パターン・bool / string の `rosbag_loop`
  受理、`pcd_loader_node` の `LaunchConfigurationEquals` condition。
- 検知パラメータプリセット管理を追加 (FR-014)。launch 引数
  `detector_preset` (string、初期値 `default`) で
  `src/vehicle_detection/config/presets/<name>.yaml` を
  `vehicle_detector_node` の `parameters=` に追加で渡す。プリセットは
  `detector_params.yaml` のオーバーレイとして適用されるため、ベース YAML →
  プリセット → 個別 launch 引数の順で「後勝ち」マージされる。
- 同梱プリセット 3 種を追加:
  - `default.yaml`: no-op オーバーレイ。既存の
    `ros2 launch vehicle_detection vehicle_detection.launch.py` の挙動を
    そのまま維持する。
  - `pandaset_balanced.yaml`: 同梱 PandaSet PCD デモ向けに検証済みの検知
    パラメータ (現在の `detector_params.yaml` の値の明示的なスナップ
    ショット)。
  - `near_range.yaml`: 近距離 (約 20 m) で軽量に検知挙動を確認するための
    ROI と clustering 設定。
- `vehicle_detection.launch.py` に `_resolve_preset_path()` ヘルパーと
  `_resolve_detector_preset` OpaqueFunction を追加。指定された
  `detector_preset` を `config/presets/<name>.yaml` の絶対パスに解決し、
  不明なプリセット名のときは指定名と利用可能なプリセット一覧を含む
  エラーで launch を失敗させる。
- `test_launch_description.py` に `detector_preset` launch 引数の宣言、
  既知プリセット (`default` / `pandaset_balanced` / `near_range`) の解決、
  不明プリセットの `ValueError`、プリセットファイルの存在確認テストを
  追加。
- HTTP POST payload と JSON Lines 保存 payload のルートに
  `schema_version` (string) フィールドを追加 (FR-013)。初期値は
  `"1.0"`。値は `detection_json` 実装側の定数
  (`kPayloadSchemaVersion`) として一元管理しており、ROS パラメータ・
  launch 引数・設定ファイルからは変更できない。HTTP 送信と JSONL 保存は
  同じ `serialize_detections()` を共有するため、両経路の payload は
  `schema_version` を含めて完全に一致する。検知 0 件のフレームでも
  `schema_version` は出力される。既存フィールド (`timestamp`、
  `frame_id`、`detections` および `detections[]` のサブフィールド) は
  削除・リネーム・意味変更していない。
- `docs/payload_schema.md` に schema version の運用ポリシーと
  version history セクションを追加。初版 `1.0` を記録。
- `test_detection_json` に `schema_version` フィールドの存在および
  値 (`"1.0"`) を確認する単体テストを追加。検知 0 件 / 1 件 / エスケープ
  必要な frame_id の各 payload で確認する。
- `test_detection_result_writer` に、JSONL 保存された行へ
  `schema_version` が含まれることを確認する単体テストを追加。
- `pcd_loader_node` に再生リスト機能を追加。新規パラメータ `pcd_files`
  (string[]) で明示リストを、`pcd_directory` (string) と `pcd_glob`
  (string、初期値 `*.pcd`) でディレクトリ展開を、`loop` (bool、初期値
  `true`) でリスト末尾の挙動 (ループ／停止) を指定できる。いずれも
  未指定の場合は従来の `pcd_file` 単一PCD再生に等しい (MVP 互換)。
- `vehicle_detection.launch.py` に `pcd_directory`、`pcd_glob`、`loop`
  の launch 引数を追加し、`ros2 launch ... pcd_directory:=<dir>` で
  ディレクトリ再生を 1 コマンドで起動できるようにした。
- `vehicle_detection_core` 共通ライブラリに `pcd_playlist` モジュール
  (`PcdPlaylistInputs`、`resolve_pcd_playlist`、`wildcard_match`) を
  追加。リスト解決を ROS / PCL から切り離し、純粋ヘルパーとして
  単体テスト可能にしている。
- `test_pcd_playlist` GTest スイート: 再生リスト解決の優先順位、
  ディレクトリ glob のソート、欠落ファイルの拒否、空ディレクトリの
  拒否、空文字エントリの拒否、ワイルドカードマッチをカバーする。
- `test_launch_description.py` で新規 launch 引数 (`pcd_directory`、
  `pcd_glob`、`loop`) の存在を検証するようにした。
- `detection_sender_node` に検知結果のローカルファイル保存機能を追加
  (FR-012)。新規パラメータ `save_results` (bool、初期値 `false`)、
  `result_output_path` (string、初期値 `""`)、`result_output_format`
  (string、初期値 `jsonl`)。`save_results=true` のとき、各検知配列を
  HTTP payload と同一構造の JSON Lines (1 フレーム 1 行) として
  保存先ファイルへ追記する。`send_mode` から独立して動作するため、
  `disabled` でも保存できる。複数 PCD 連続再生 (FR-011) 中も publish
  順に append される。
- `vehicle_detection_core` 共通ライブラリに `DetectionResultWriter`
  (`detection_result_writer.{hpp,cpp}`) を追加。スキーマ生成は既存の
  `serialize_detections()` を再利用し、`std::ofstream` の append +
  binary モードで `'\n'` 区切りで書き込む。設定不正・ファイルオープン
  失敗・書き込み失敗のいずれも例外を出さず、`last_error()` 経由で
  原因を返す。
- `test_detection_result_writer` GTest スイート: 既定無効、無効化時の
  no-op、JSONL 1 行 append、複数 append の順序保持、フォーマット名
  解析 (`jsonl` 受理 / それ以外拒否)、空パス拒否、ディレクトリへの
  オープン失敗、再構成での旧ファイルクローズ、フォーマット拒否時の
  保存無効化をカバーする。
- `config/detector_params.yaml` の `detection_sender_node` セクションに
  `save_results`、`result_output_path`、`result_output_format` の
  既定値とコメントを追加。
- `docs/payload_schema.md` に JSONL 保存形式 (1 フレーム 1 行、HTTP
  payload と同一構造) の説明を追加。

### Changed

- `vehicle_detection.launch.py` の `pcd_loader_node` に
  `LaunchConfigurationEquals('input_mode', 'pcd')` の condition を
  付与した (FR-015)。`input_mode` 未指定または `pcd` のときの挙動は
  既存と同一であり、`input_mode:=rosbag` のときのみ起動を抑制する。
- `vehicle_detection.launch.py` の `vehicle_detector_node` の
  `parameters=` リストに、`detector_preset` から解決したプリセット YAML
  (絶対パス) を `params_file` の後ろに追加した (FR-014)。これにより
  ベース YAML → プリセット → 個別 launch 引数の順で「後勝ち」マージが
  行われる。`detector_preset:=default` (既定) は no-op オーバーレイの
  ため、未指定時の挙動は MVP / 既存 Phase 2 と同一に維持される。
  プリセット YAML は `vehicle_detector_node` のみに渡し、他ノード
  (`pcd_loader_node`、`detection_sender_node`、`parameter_bridge_node`)
  には影響しない。
- `serialize_detections()` のルート JSON 出力に `schema_version` を
  先頭フィールドとして追加した (FR-013)。既存フィールドの順序・名前・
  意味は変えていないため、既存受信側は無視するだけで動作する
  (フォワード互換)。`docs/payload_schema.md` の version history に
  初版 `1.0` を記録した。
- `pcd_loader_node` は `pcl::io::loadPCDFile` を起動時の 1 回呼び出し
  ではなく、各 publish ティック直前の lazy-load + 1 スロットキャッシュ
  に変更。同じインデックスを連続 publish するときは再ロードしない。
- `config/detector_params.yaml` の `pcd_loader_node` セクションに
  `pcd_files`、`pcd_directory`、`pcd_glob`、`loop` の既定値とコメント
  を追加。
- `detection_sender_node` は、検知 callback の冒頭で `save_results` が
  有効なら保存ヘルパーへ payload を append するようになった。
  `result_output_format` への不正値は起動時とパラメータ更新時に拒否
  する一方で、保存先パスのオープン失敗は警告ログのみで吸収し、
  ROS / HTTP 送信側の挙動は維持する。

### Fixed

- `config/detector_params.yaml` の `pcd_loader_node` セクションから
  `pcd_files: []` の行を削除した (FR-011 関連)。ROS 2 jazzy では
  YAML の空配列 (`[]`) は要素型が決まらないため、`--params-file` 経由で
  ロードした `pcd_files` パラメータの override が型 `NOT_SET` 扱いとなり、
  `pcd_loader_node` が `declare_parameter<std::vector<std::string>>` の
  段階で `No parameter value set` を投げて起動直後にクラッシュしていた。
  C++ 側の declare は `std::vector<std::string>{}` をデフォルトとして
  持つため、YAML から行を消すだけで FR-011 の「`pcd_files` 未指定時は
  `pcd_directory` または `pcd_file` にフォールバック」挙動はそのまま
  維持される。明示リストを使う場合は別 YAML / `ros2 param set` から
  非空のリストとして渡す。

## v1.0.0 - 2026-05-02

初版リリース (MVP)。`osrf/ros:jazzy-desktop` 上で PCD 点群から
普通車検知を行い、RViz 表示・ROS topic / HTTP JSON 送信・ブラウザ
パラメータ GUI を含む一連のパイプラインを提供する。

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
- 開発進捗追跡用のロードマップ。
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
