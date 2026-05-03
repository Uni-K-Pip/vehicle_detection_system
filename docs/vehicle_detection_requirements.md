# 車両検知システム 要件定義書

作成日: 2026-04-30  
対象: ミニマム実装版  
使用技術: ROS 2 Jazzy, C++, PCL

## 1. 目的

Autowareのような点群ベースの車両検知パイプラインを、学習済みモデルや実車センサを前提にせず、まずは小さく動く形で構築する。

ミニマム実装では、インターネット上で公開されている利用可能なPCD点群ファイルをローカルに配置し、ROS 2上で読み込み、車両候補を検知し、検知情報を送信するところまでを対象とする。

## 2. 前提条件

### 2.1 実行環境

`C:\Users\kohei\work\myproject\ROS2\ROS2_RESTART_NOTES.md`の内容を前提とする。

- ホストOS: Windows
- 作業ディレクトリ: `C:\Users\kohei\work\myproject\ROS2`
- ROS 2実行環境: Docker Desktop Linux engine上のコンテナ
- Dockerイメージ: `osrf/ros:jazzy-desktop`
- ROS 2ディストリビューション: Jazzy
- コンテナ内ワークスペース: `/root/ROS2/get-started-ros2`
- ビルド方式: `colcon build`
- 既存利用可能例: `ros2_practice`にPCLのVoxelGridサンプルあり

### 2.2 開発方針

- 実装言語はC++を基本とする。
- ROS 2ノード間通信を基本とし、外部送信が必要な場合はHTTP POSTへ切り替えられるようにする。
- 点群処理はPCLを利用する。
- GUIはQt/rqtを使用せず、ブラウザベースの簡易GUIとする。
- ミニマム実装ではリアルタイムLiDAR入力、Autoware本体連携、学習モデル推論は対象外とする。
- PCDファイルはリポジトリへ直接コミットせず、`data/pcd/`配下へ手動またはスクリプトで配置する。
- 利用する一般公開データセットとPCDファイルは設定ファイルで指定する。
- 入力PCDの座標系は`map`以外に`lidar`等も想定し、検知結果は`map`座標系で出力する。

## 3. スコープ

### 3.1 対象範囲

- フリーまたは利用許諾が確認できるPCD点群データの利用
- 設定ファイルで指定したPCDデータセット/PCDファイルの利用
- PCDファイルのROS 2 `sensor_msgs/msg/PointCloud2`化
- 入力点群座標系から`map`座標系への変換
- 点群前処理
- 普通車の車両候補検知
- 検知情報のROS 2トピック送信
- 検知情報のHTTP外部送信
- RViz2での点群・検知結果可視化
- 検知パラメータのGUI調整
- launchファイルによる一括起動
- 最低限の動作確認手順とテスト

### 3.2 対象外

- 実車LiDAR、カメラ、GNSS、IMUとの接続
- Autoware Universeへの組み込み
- トラッキング、経路計画、制御
- 深層学習による3D物体検知
- 普通車以外のクラス分類
- トラック、バス、二輪車、歩行者、その他障害物の検知
- ラベル付きデータによる精度評価の本格実施
- 外部サーバ、クラウド、V2X等への送信

## 4. システム概要

### 4.1 MVP構成

```text
PCDファイル
  -> pcd_loader_node
  -> /input/points
  -> tf2 transform to map
  -> vehicle_detector_node
  -> /vehicle_detections
  -> detection_sender_node
  -> ROS 2トピック または HTTP POST

vehicle_detector_node
  -> /debug/points_filtered
  -> /debug/clusters
  -> /vehicle_markers

parameter_gui_node
  -> ROS 2 parameter service
  -> vehicle_detector_node
```

### 4.2 推奨パッケージ構成

```text
src/
  vehicle_detection/
    src/
      pcd_loader_node.cpp
      vehicle_detector_node.cpp
      detection_sender_node.cpp
      parameter_bridge_node.cpp
      parameter_gui_node.cpp
    launch/
      vehicle_detection.launch.py
    config/
      detector_params.yaml
      dataset_params.yaml
      transforms.yaml
      rviz_vehicle_detection.rviz
    data/
      pcd/.gitkeep
    README.md
```

カスタムメッセージが必要になった場合は、別パッケージとして`vehicle_detection_msgs`を追加する。

## 5. 機能要件

### FR-001 PCDデータ取得・配置

- システムは、外部から取得したPCDファイルを入力データとして扱えること。
- 使用する一般公開データセットは設定ファイルで指定できること。
- 設定ファイルには、データセット名、取得元URL、ライセンス、ローカル配置先、使用PCDファイルパスを記載できること。
- PCDファイルは`data/pcd/`配下に配置すること。
- PCDファイルのライセンス、利用条件、取得元URLを`data/pcd/README.md`に記録すること。
- 実行時にインターネット接続を必須にしないこと。

### FR-002 PCD読み込み

- `pcd_loader_node`は指定されたPCDファイルを読み込むこと。
- 読み込んだ点群を`sensor_msgs/msg/PointCloud2`として publish すること。
- publish先の初期値は`/input/points`とすること。
- 入力点群の`frame_id`はパラメータ`input_frame_id`で指定できること。
- `input_frame_id`の初期値は`lidar`とすること。
- 単発publishと周期publishを切り替えられること。

### FR-002A 座標変換

- `vehicle_detector_node`は入力点群を`target_frame_id`へ変換してから検知処理を行うこと。
- `target_frame_id`の初期値は`map`とすること。
- 入力点群の`frame_id`が`target_frame_id`と同じ場合は変換せず処理すること。
- 入力点群の`frame_id`が`lidar`等の場合は、tf2により`map`座標系へ変換すること。
- `lidar`から`map`への静的TFは設定ファイルで指定できること。
- 静的TFは`x`, `y`, `z`, `roll`, `pitch`, `yaw`で指定すること。
- 静的TFが未指定の場合は、同一原点・回転なしのidentity transformとして扱うこと。
- 検知結果、可視化Marker、HTTP payloadに含める座標は`map`座標系とすること。
- launchまたは設定ファイルから、MVP用の静的TFを設定できること。
- 必要なTFが取得できない場合、検知結果を出さず、警告ログに不足している変換元/変換先を出すこと。

### FR-003 点群前処理

- `vehicle_detector_node`は`/input/points`をsubscribeすること。
- VoxelGridにより点群をダウンサンプリングできること。
- ROI範囲で点群を切り出せること。
- 地面除去を行えること。
- 前処理後の点群をデバッグ用トピックへpublishできること。

### FR-004 普通車候補検知

- 前処理後の点群に対してクラスタリングを行うこと。
- 各クラスタの3D bounding boxを算出すること。
- bounding boxの寸法条件により普通車候補を抽出すること。
- MVPの検知対象は普通車のみとし、トラック、バス、二輪車は対象外とすること。
- 少なくとも以下の情報を検知結果として持つこと。
  - 検知ID
  - 中心座標
  - bounding box寸法
  - yaw角、算出できない場合は0
  - confidence、ミニマム実装では固定値またはルールベース値
  - header

### FR-005 検知情報送信

- MVPではROS 2トピック送信とHTTP外部送信を切り替えられること。
- 送信方式はパラメータ`send_mode`で指定できること。
- `send_mode=ros_topic`の場合、送信トピックの初期値は`/vehicle_detections`とする。
- `send_mode=http`の場合、検知結果を指定URLへHTTP POSTできること。
- `send_mode=both`の場合、ROS 2トピック送信とHTTP POSTを同時に行えること。
- `send_mode=disabled`の場合、デバッグ用publish以外の送信を停止できること。
- メッセージ型は、利用可能であれば`vision_msgs/msg/Detection3DArray`を優先する。
- `vision_msgs`を利用しない場合は、`vehicle_detection_msgs/msg/VehicleDetectionArray`を定義する。
- `detection_sender_node`は、後続の送信方式変更に備えて検知結果の中継ノードとして分離できること。
- HTTP送信のpayloadはJSONとし、検知時刻、frame_id、検知ID、中心座標、寸法、yaw、confidenceを含めること。
- HTTP送信先URL、timeout、retry回数はパラメータで指定できること。
- HTTP送信先URLは設定ファイルで変更できること。
- HTTP送信に認証は不要とすること。
- HTTP送信失敗時は検知処理全体を停止させず、警告ログを出すこと。

### FR-006 可視化

- RViz2で入力点群、前処理後点群、検知bounding boxを確認できること。
- bounding boxは`visualization_msgs/msg/MarkerArray`として`/vehicle_markers`へpublishすること。
- 可視化用RViz設定ファイルを用意すること。

### FR-007 GUIによるパラメータ調整

- 車両検知に関わる主要パラメータをGUIで変更できること。
- GUIはQt/rqtを使用しないこと。
- GUIはブラウザで開くローカルWeb UIとすること。
- `parameter_bridge_node`はHTTP APIを提供し、ROS 2 parameter serviceを利用して実行中ノードのパラメータを更新すること。
- MVPでは簡易Web UIでよいが、少なくとも数値入力またはスライダーで変更できること。
- GUIで変更したパラメータはログに出力すること。
- GUIが起動できない場合でも、YAML設定ファイルと`ros2 param set`で同等の調整ができること。
- Web UI実装に外部ライブラリを使う場合は、MIT/BSD/Apache-2.0等の緩いライセンスのものに限定すること。

### FR-008 設定ファイル

- 検知パラメータは`config/detector_params.yaml`にまとめること。
- launchファイルから設定ファイルを読み込めること。
- パラメータ名、単位、初期値、推奨範囲をREADMEまたは設定ファイルコメントに記載すること。

### FR-009 起動

- `ros2 launch vehicle_detection vehicle_detection.launch.py`でMVP全体を起動できること。
- PCDファイルパス、GUI起動有無、RViz起動有無をlaunch引数で切り替えられること。

### FR-010 ログ

- 起動時に使用PCDファイル、点数、主要パラメータをログ出力すること。
- 検知ごとに検知数をログ出力できること。
- 異常時には原因が分かるエラーを出すこと。

### FR-011 複数PCD連続再生 (Phase 2)

`pcd_loader_node`はMVPの単一PCD再生に加えて、複数のPCDファイルを順番にpublishする「連続再生」モードを提供する。データセット内の複数フレームを順次評価したい場合に、launch を再起動せずに切り替えるための機能。

- 複数PCDファイルを「再生リスト」として指定できること。
  - 明示的なファイルパスのリストで指定する方式 (`pcd_files`) を提供すること。
  - ディレクトリと glob パターンによりリストを構築する方式 (`pcd_directory` + `pcd_glob`) を提供すること。
  - どちらも未指定の場合は、MVPと同じく単一の `pcd_file` を 1 要素のリストとして扱うこと。
  - `pcd_files` が非空の場合は、`pcd_directory` の解決に優先すること。
- 再生リストの各要素は、ノード起動時に存在を検証すること。
  - 1 つでも存在しないファイルがあった場合は、起動を失敗扱いとし、欠落したパスをログ出力すること。
- 再生は `publish_rate_hz` 周期で行い、1 周期ごとに次のファイルへ進むこと。
- リストの末尾に達した場合の挙動は `loop` パラメータで切り替えられること。
  - `loop=true` (初期値) の場合、先頭に戻ってループ再生すること。
  - `loop=false` の場合、最後のファイルを publish した後は publish を停止し、ノードは生存を続けること。
- `publish_once=true` の場合は、リストの先頭ファイルを 1 回だけ publish し、それ以降は publish しないこと (MVP の単一ファイル挙動と整合)。
- 各 publish 直前に対象ファイルを読み込めばよいこと。直前に読み込んだファイルと同一であれば、再読み込みを省略してよい。
- 各 publish 時に、現在のインデックスとファイルパスをログ出力すること。
- 単一PCD再生 (`pcd_files` および `pcd_directory` 未指定) の挙動は MVP と完全に同一であること。

### FR-012 検知結果の保存 (Phase 2)

`detection_sender_node` は、ROS 2 トピック送信および HTTP POST に加えて、
検知結果をローカルファイルへ追記保存できる。実行後にオフラインで検知結果を
検査し、複数のパラメータ設定や PCD セットを比較する用途を想定する。

- 保存形式は MVP 時点では JSON Lines (`.jsonl`) のみ対応すること。
- 1 フレーム分の `vision_msgs/msg/Detection3DArray` を、1 行の JSON
  オブジェクトとしてファイル末尾に追記すること。
- 1 行の JSON は、HTTP POST と同一の payload 構造
  (`docs/payload_schema.md` 参照) を使用し、シリアライザ実装を共有すること
  (重複実装を避ける)。
- 以下のパラメータを `detection_sender_node` に追加すること。
  - `save_results` (bool, 初期値 `false`): 検知結果保存を有効化する。
  - `result_output_path` (string, 初期値 `""`): 保存先ファイルパス。
  - `result_output_format` (string, 初期値 `jsonl`): 保存形式。MVP では
    `jsonl` のみ受け付ける。
- `save_results=false` の場合、ファイルへの書き込みは一切発生せず、
  既存挙動 (ROS 2 / HTTP / disabled の各 send_mode) を変えないこと。
- `save_results=true` で保存先ファイルを開けない場合 (パスが空、親
  ディレクトリが存在しない、書き込み権限がないなど) でも、ノードを
  クラッシュさせず、原因を含む警告ログを出して保存だけを停止すること。
  ROS 2 トピック送信、HTTP 送信、デバッグ publish の挙動は影響を受けない
  こと。
- 保存ファイルは追記モードで開くこと。同じパスで再起動した場合、後続の
  実行のフレームが既存ファイルの末尾に追加されること。
- 複数PCD連続再生 (FR-011) 中でも、publish された各検知結果が publish
  順に append されること。
- `result_output_format` に `jsonl` 以外を指定した場合は、起動時およびパラメータ
  変更時に拒否すること。

## 6. パラメータ要件

| パラメータ | 説明 | 初期値案 |
| --- | --- | --- |
| `dataset_name` | 使用データセット名 | `pandaset_lidar_pcd_subset` |
| `dataset_source_url` | データセット取得元URL | `https://www.pandaset.org/` |
| `dataset_download_url` | PCDサブセット取得URL | `https://ssd.mathworks.com/supportfiles/driving/data/PandasetLidarData.zip` |
| `dataset_license` | データセットライセンス | CC-BY-4.0 |
| `dataset_attribution` | データセット出典表記 | PandaSet by Hesai and Scale AI |
| `dataset_extract_dir` | 展開先ディレクトリ | `data/pandaset_lidar_pcd_subset` |
| `dataset_pcd_glob` | 使用PCDファイル検索パターン | `Lidar/*.pcd` |
| `pcd_file` | 入力PCDファイルパス (単一PCD再生) | `data/pcd/sample.pcd` |
| `pcd_files` | 連続再生する PCD ファイルパスのリスト (Phase 2) | `[]` |
| `pcd_directory` | 連続再生する PCD を glob で集めるディレクトリ (Phase 2) | `""` |
| `pcd_glob` | `pcd_directory` 配下で集める glob パターン (Phase 2) | `*.pcd` |
| `loop` | 再生リスト末尾でループするかどうか (Phase 2) | `true` |
| `input_frame_id` | 入力点群座標系 | `lidar` |
| `target_frame_id` | 検知結果出力座標系 | `map` |
| `static_tf_x` | `lidar`から`map`へのXオフセット [m] | `0.0` |
| `static_tf_y` | `lidar`から`map`へのYオフセット [m] | `0.0` |
| `static_tf_z` | `lidar`から`map`へのZオフセット [m] | `0.0` |
| `static_tf_roll` | `lidar`から`map`へのroll [rad] | `0.0` |
| `static_tf_pitch` | `lidar`から`map`へのpitch [rad] | `0.0` |
| `static_tf_yaw` | `lidar`から`map`へのyaw [rad] | `0.0` |
| `static_tf_use_identity_if_missing` | TF未指定時にidentity transformを使う | `true` |
| `publish_rate_hz` | PCD周期publish周波数 | `1.0` |
| `voxel_leaf_size` | VoxelGrid leaf size [m] | `0.2` |
| `roi_min_x` | ROI最小X [m] | `0.0` |
| `roi_max_x` | ROI最大X [m] | `80.0` |
| `roi_min_y` | ROI最小Y [m] | `-30.0` |
| `roi_max_y` | ROI最大Y [m] | `30.0` |
| `roi_min_z` | ROI最小Z [m] | `-3.0` |
| `roi_max_z` | ROI最大Z [m] | `3.0` |
| `ground_distance_threshold` | 地面平面除去しきい値 [m] | `0.2` |
| `cluster_tolerance` | クラスタ距離しきい値 [m] | `0.8` |
| `cluster_min_size` | 最小クラスタ点数 | `20` |
| `cluster_max_size` | 最大クラスタ点数 | `5000` |
| `vehicle_min_length` | 車両候補最小長 [m] | `2.0` |
| `vehicle_max_length` | 車両候補最大長 [m] | `6.0` |
| `vehicle_min_width` | 車両候補最小幅 [m] | `1.2` |
| `vehicle_max_width` | 車両候補最大幅 [m] | `2.8` |
| `vehicle_min_height` | 車両候補最小高 [m] | `1.0` |
| `vehicle_max_height` | 車両候補最大高 [m] | `3.0` |
| `send_mode` | 送信方式: `disabled`, `ros_topic`, `http`, `both` | `ros_topic` |
| `http_endpoint_url` | HTTP POST送信先URL | `http://host.docker.internal:8080/detections` |
| `http_timeout_ms` | HTTP送信timeout [ms] | `1000` |
| `http_retry_count` | HTTP送信retry回数 | `0` |
| `http_auth_type` | HTTP認証方式 | `none` |
| `save_results` | 検知結果のファイル保存を有効化 (Phase 2) | `false` |
| `result_output_path` | 保存先ファイルパス (Phase 2) | `""` |
| `result_output_format` | 保存形式: `jsonl` (Phase 2) | `jsonl` |
| `gui_port` | Web GUIの待受ポート | `8081` |

## 7. 非機能要件

### NFR-001 実行性能

- MVPではリアルタイム性能を必須としない。
- 1枚のPCD点群に対し、数秒以内に検知結果を出せることを目標とする。
- ダウンサンプリング後の処理が安定して完了することを優先する。

### NFR-002 保守性

- 各ノードは責務を分離すること。
- パラメータはコード中に固定せず、ROS 2 parametersで管理すること。
- 点群処理の主要ステップは関数単位で分割し、単体テスト可能な構造にすること。

### NFR-003 再現性

- 使用したPCDファイル、取得元、取得日、ライセンスを記録すること。
- 使用するデータセット/PCDファイルは設定ファイルで再現できること。
- 同じPCDと同じパラメータで同じ検知結果が得られること。
- 初期パラメータ一式をYAMLで保存すること。

### NFR-004 拡張性

- 将来、rosbag入力、実LiDAR入力、外部通信、Autoware連携へ拡張できる構造にすること。
- 検知アルゴリズムを差し替えられるよう、入出力トピックとメッセージを安定させること。

### NFR-005 ライセンス

- GUI実装にQt/rqtは使用しないこと。
- 外部ライブラリを追加する場合は、MIT、BSD、Apache-2.0、Boost Software License等の利用条件が緩いものを優先すること。
- GPL/LGPL等、配布形態やリンク方式に注意が必要なライブラリはMVPでは避けること。
- 採用ライブラリのライセンスはREADMEに記載すること。

## 8. データ要件

- 入力点群形式はPCDとする。
- PCDには少なくともXYZ座標が含まれること。
- intensityがある場合は保持するが、MVP検知では必須としない。
- 車両が含まれる道路環境のPCDを優先する。
- 一般的にフリーで取得可能な公開データセットを使用する。
- データセット名、URL、ライセンス、ローカルPCDパスは`config/dataset_params.yaml`で指定する。
- ラベル付きデータはMVPでは必須としない。
- 商用利用可否は別途確認する。MVPでは研究・個人開発用途の範囲で利用する。

### 8.1 初期採用データセット

MVPで最初に使用する公開PCDデータセットは、PandaSet由来のPCDサブセットとする。

| 項目 | 内容 |
| --- | --- |
| データセット | PandaSet Lidar PCD subset |
| 元データ | PandaSet by Hesai and Scale AI |
| 採用理由 | 自動運転向け実走行データで普通車を含み、PCD形式のサブセットが入手でき、普通車検知MVPに適しているため |
| 形式 | PCD |
| 規模 | 400点群、約394 MB |
| 初期座標系 | `lidar` |
| 出力座標系 | `map` |
| 初期TF | identity transform |
| download URL | `https://ssd.mathworks.com/supportfiles/driving/data/PandasetLidarData.zip` |
| source URL | `https://www.pandaset.org/` |
| attribution | PandaSet by Hesai and Scale AI |

補欠候補として、直接PCD形式で取得できるDynamic Mapping Benchmarkの`05.zip`を保持する。ただし、MVPの普通車検知ではPandaSetを優先する。

### 8.2 データセット設定ファイル例

`config/dataset_params.yaml`には、少なくとも以下を指定できること。

```yaml
dataset:
  name: pandaset_lidar_pcd_subset
  source_url: https://www.pandaset.org/
  download_url: https://ssd.mathworks.com/supportfiles/driving/data/PandasetLidarData.zip
  license: CC-BY-4.0
  attribution: PandaSet by Hesai and Scale AI
  extract_dir: data/pandaset_lidar_pcd_subset
  pcd_glob: Lidar/*.pcd
  default_pcd_file: data/pandaset_lidar_pcd_subset/Lidar/<first_pcd_file>.pcd
  input_frame_id: lidar
  target_frame_id: map
```

### 8.3 TF設定ファイル例

`config/transforms.yaml`には、`lidar`から`map`への静的TFを指定できること。正式な位置・姿勢オフセットが未指定の場合は、以下のidentity transformを使用する。

```yaml
transforms:
  lidar_to_map:
    parent_frame_id: map
    child_frame_id: lidar
    x: 0.0
    y: 0.0
    z: 0.0
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
    use_identity_if_missing: true
```

## 9. インターフェース要件

### 9.1 ROS 2トピック

| トピック | 型 | 方向 | 用途 |
| --- | --- | --- | --- |
| `/input/points` | `sensor_msgs/msg/PointCloud2` | pub/sub | 入力点群 |
| `/debug/points_filtered` | `sensor_msgs/msg/PointCloud2` | pub | 前処理後点群 |
| `/debug/clusters` | `sensor_msgs/msg/PointCloud2`または`MarkerArray` | pub | クラスタ確認 |
| `/vehicle_detections` | `vision_msgs/msg/Detection3DArray`またはcustom | pub | 普通車検知結果。座標系は`map` |
| `/vehicle_markers` | `visualization_msgs/msg/MarkerArray` | pub | RViz可視化。座標系は`map` |

### 9.2 ROS 2パラメータ

- `pcd_loader_node`はPCD読み込み関連パラメータを持つ。
- `vehicle_detector_node`は前処理、クラスタリング、車両判定関連パラメータを持つ。
- parameter callbackで実行中の値更新に対応する。
- 不正値が設定された場合は拒否し、ログに理由を出す。

### 9.3 HTTP外部送信

- HTTP methodはPOSTとする。
- Content-Typeは`application/json`とする。
- 認証は不要とする。
- `frame_id`は検知結果の座標系を示し、MVPでは`map`とする。
- 送信payload例:

```json
{
  "timestamp": "2026-04-30T00:00:00.000Z",
  "frame_id": "map",
  "detections": [
    {
      "id": 1,
      "class": "car",
      "confidence": 0.8,
      "center": {"x": 12.3, "y": -1.2, "z": 0.8},
      "size": {"length": 4.3, "width": 1.8, "height": 1.6},
      "yaw": 0.0
    }
  ]
}
```

### 9.4 Web GUI

- Web GUIはローカルブラウザから`http://localhost:<gui_port>`で開けること。
- Dockerコンテナ内で起動する場合、ホストWindowsからアクセスできるようポート公開手順をREADMEに記載すること。
- Web GUIはパラメータ一覧取得、値変更、現在値再読込ができること。

## 10. 受け入れ基準

- Docker環境内で`colcon build`が成功すること。
- launchファイルでPCD読み込み、検知、結果publishが起動すること。
- 入力PCDの`frame_id`が`lidar`の場合でも、静的TF設定により検知結果が`map`座標系で出力されること。
- RViz2で入力点群と検知bounding boxを確認できること。
- GUIから主要パラメータを変更し、検知結果またはログに反映されること。
- Qt/rqtなしでGUIを起動できること。
- `send_mode=ros_topic`で`/vehicle_detections`に検知結果がpublishされること。
- `send_mode=http`で指定HTTP endpointに検知結果JSONがPOSTされること。
- HTTP endpointが停止していても検知ノードがクラッシュしないこと。
- PCDファイルが存在しない場合、分かりやすいエラーを出して終了すること。
- 1つ以上のPCDサンプルで`/vehicle_detections`に検知結果がpublishされること。
- READMEにセットアップ、データ配置、起動、パラメータ調整方法が記載されていること。
- `pcd_files` または `pcd_directory` で複数PCDを指定したとき、`publish_rate_hz` の周期で順次 publish されること (Phase 2)。
- `loop=true` でリスト末尾の次に先頭に戻り、`loop=false` でリスト末尾以降は publish が止まること (Phase 2)。
- 複数PCD指定時に、`pcd_files` も `pcd_directory` も未指定の MVP 単一PCD再生の挙動が変わらないこと (Phase 2)。
- `save_results=true`、`result_output_path` 指定時に、各検知フレームが JSON Lines ファイルへ 1 行ずつ追記されること (Phase 2)。
- `save_results=false` の場合、保存先ファイルへの書き込みが行われず既存挙動が変わらないこと (Phase 2)。
- `save_results=true` で開けない保存先パスを指定しても、`detection_sender_node` がクラッシュせず警告ログを出して動作を継続すること (Phase 2)。

## 11. テスト要件

- PCDファイルパス不正時の異常系テスト
- 空点群または点数が少ないPCDでクラッシュしないテスト
- VoxelGrid、ROI、クラスタリングの基本処理テスト
- `lidar`から`map`への座標変換テスト
- TF未設定時に検知結果を出さず警告ログを出すテスト
- パラメータ更新時の値反映テスト
- 送信方式切替テスト
- HTTP送信成功テスト
- HTTP送信失敗時の継続動作テスト
- Web GUIからのパラメータ更新テスト
- launch起動確認
- `ros2 topic echo /vehicle_detections`で検知情報が確認できること
- 再生リスト解決 (`pcd_files` 優先、`pcd_directory` 展開、未指定時の単一PCDフォールバック) の単体テスト (Phase 2)
- `loop=true` / `loop=false` でのリスト末尾挙動の単体テスト (Phase 2)
- 検知結果保存ヘルパーの単体テスト: 無効化時の no-op、有効時の JSONL append、不正パス時の no-throw、未対応フォーマット拒否 (Phase 2)

## 12. 実装フェーズ案

### Phase 1: MVP

- PCD読み込みノード
- `lidar`等の入力座標系から`map`へのTF変換
- PCLベースの車両候補検知ノード
- ROS 2トピックでの検知結果送信
- HTTP POSTでの検知結果送信
- RViz2可視化
- YAMLパラメータ
- ブラウザベースGUIパラメータ調整

### Phase 2: 実用性改善

- rosbag入力対応
- 複数PCD連続再生 (Phase 2 の最初の対応項目。詳細は FR-011)
- 検知結果の保存
- 簡易トラッキング
- パラメータプリセット管理
- HTTP payload schemaのバージョン管理

### Phase 3: Autoware連携・高度化

- Autoware互換メッセージ検討
- 実LiDAR入力対応
- 学習ベース検知への差し替え
- 外部プロセスまたはネットワーク送信

## 13. リスク・課題

- フリーPCDのライセンス条件がデータごとに異なる。
- 初期採用データセットはPandaSet由来のPCDサブセットとし、配布元、取得元URL、ライセンス、出典表記を設定ファイルとREADMEに記録する必要がある。
- ラベルなしPCDでは検知精度の定量評価が難しい。
- PCLクラスタリングのみでは、車両以外の物体を誤検知する可能性がある。
- PCDの座標系、地面高さ、点密度により初期パラメータ調整が必要になる。
- 入力PCDの座標系がデータセットごとに異なる可能性があるため、`input_frame_id`とTF設定の管理が必要になる。
- `lidar`から`map`への正式な位置・姿勢オフセットがない場合、identity transformで開始するため、絶対位置としての`map`座標は仮座標になる。
- GUIはQt/rqtを避けるため、Web UI用HTTPサーバ実装とROS 2 parameter bridgeの設計が必要になる。
- HTTP外部送信先が遅い、または停止している場合に、送信処理が検知処理を詰まらせない設計が必要になる。

## 14. 確認事項

以下は実装前に確認したい事項。

1. PandaSetサブセットの具体的な先頭PCDファイル名を、展開後の実ファイル名に合わせて確定する。
2. 正式な`lidar`から`map`への位置・姿勢オフセットが判明した場合、`x`, `y`, `z`, `roll`, `pitch`, `yaw`を設定する。

## 15. 確定事項

- 検知情報送信は、ROS 2トピック送信とHTTP外部送信を切り替え可能にする。
- HTTP送信先URLは設定ファイルで変更可能にし、payloadはJSON、認証なしとする。
- GUIはQt/rqtを使用せず、ブラウザベースのローカルWeb UIとする。
- 検知対象は普通車のみとする。
- 一般的にフリーで取得可能な公開データセットを設定ファイルで指定する。
- 入力PCDの座標系は`map`以外に`lidar`等も想定し、検知結果は`map`座標系で出力する。
- MVPの初期採用データセットはPandaSet由来のPCDサブセットとする。
- `lidar`から`map`への初期静的TF値は、未指定の場合identity transformとする。
- 正式な位置・姿勢オフセットがある場合は、`x`, `y`, `z`, `roll`, `pitch`, `yaw`で設定可能にする。
