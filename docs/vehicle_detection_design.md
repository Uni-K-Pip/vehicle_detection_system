# 車両検知システム 基本設計書

作成日: 2026-04-30  
対象: ミニマム実装版  
根拠資料: `docs/vehicle_detection_requirements.md`

## 1. 設計ステータス

本書は、要件定義書を実装可能な単位へ分解した基本設計書である。
MVPのコーディング前に必要な主要判断は本書で確定済みとする。

未確定のまま残す事項は以下に限定する。

- 実際に展開されたPandaSet由来PCDサブセット内の先頭PCDファイル名
- 正式な`lidar`から`map`への位置・姿勢オフセット

上記は実装を止める未決事項ではない。初期実装では`data/pcd/sample.pcd`またはlaunch引数で指定されたPCDを使い、TFはidentity transformで開始する。

## 2. 設計方針

- 実装言語はC++、ビルドは`ament_cmake`とする。
- ROS 2ノード間通信を主経路にし、HTTP送信とWeb GUIは周辺機能として分離する。
- 点群処理はPCLで実装し、深層学習推論は行わない。
- 検知結果のROSメッセージは`vision_msgs/msg/Detection3DArray`を標準採用する。
- カスタムメッセージパッケージはMVPでは作成しない。`vision_msgs`が導入できない環境だけ、後続対応として`vehicle_detection_msgs`を検討する。
- GUIはQt/rqtを使わず、`parameter_bridge_node`がローカルHTTP APIと静的HTMLを提供する。
- HTTPクライアント/サーバ実装は、MITライセンスのheader-onlyライブラリ`cpp-httplib`をvendor配置して利用する方針とする。取得できない場合はBoost.Asio/Beastへ切り替える。
- 入力点群が`lidar`等の座標系の場合は、検知前にtf2で`map`へ変換する。
- HTTP送信失敗、GUI停止、RViz未起動は検知処理を停止させない。

## 3. パッケージ構成

```text
vehicle_detection_system/
  README.md
  docs/
    vehicle_detection_requirements.md
    vehicle_detection_design.md
    pre_coding_checklist.md
  config/
    detector_params.yaml
    dataset_params.yaml
    transforms.yaml
    rviz_vehicle_detection.rviz
  data/
    pcd/
      README.md
      .gitkeep
  launch/
    vehicle_detection.launch.py
  src/
    vehicle_detection/
      CMakeLists.txt
      package.xml
      include/vehicle_detection/
        detection_types.hpp
        point_cloud_processing.hpp
        parameter_validation.hpp
        http_json.hpp
      src/
        pcd_loader_node.cpp
        vehicle_detector_node.cpp
        detection_sender_node.cpp
        parameter_bridge_node.cpp
        point_cloud_processing.cpp
        parameter_validation.cpp
      web/
        parameter_gui.html
      test/
        test_point_cloud_processing.cpp
        test_parameter_validation.cpp
```

`src/vehicle_detection`はROS 2パッケージルートとして作成する。
現在の`vehicle_detection_system/config`と`vehicle_detection_system/launch`は、実装時にパッケージ配下へコピーまたは移動してinstall対象にする。

## 4. 依存関係

| 分類 | 依存 | 用途 | ライセンス方針 |
| --- | --- | --- | --- |
| ROS 2 | `rclcpp` | ノード実装 | ROS 2標準 |
| ROS 2 | `sensor_msgs` | 点群入出力 | ROS 2標準 |
| ROS 2 | `vision_msgs` | 検知結果 | ROS 2標準 |
| ROS 2 | `visualization_msgs` | RViz marker | ROS 2標準 |
| ROS 2 | `geometry_msgs` | bbox pose | ROS 2標準 |
| ROS 2 | `tf2`, `tf2_ros`, `tf2_sensor_msgs` | 座標変換 | ROS 2標準 |
| ROS 2 | `pcl_conversions` | ROS/PCL変換 | ROS 2標準 |
| PCL | `pcl_common`, `pcl_filters`, `pcl_segmentation`, `pcl_io` | 点群処理 | BSD系 |
| HTTP | `cpp-httplib` | HTTP POST、Web GUI API | MIT |
| JSON | `nlohmann_json`または手組み生成 | HTTP payload | MITまたは依存なし |

`cpp-httplib`と`nlohmann_json`を使う場合は、ライセンス表記をREADMEへ追加する。

## 5. システム構成

```text
PCD file
  -> pcd_loader_node
  -> /input/points [sensor_msgs/msg/PointCloud2, frame_id=input_frame_id]
  -> vehicle_detector_node
       - tf2 transform to target_frame_id
       - voxel downsample
       - ROI crop
       - ground removal
       - Euclidean clustering
       - axis-aligned bbox filtering
  -> /vehicle_detections/raw [vision_msgs/msg/Detection3DArray]
  -> detection_sender_node
       - ROS topic relay
       - HTTP POST
  -> /vehicle_detections [vision_msgs/msg/Detection3DArray]

vehicle_detector_node
  -> /debug/points_filtered [sensor_msgs/msg/PointCloud2]
  -> /debug/clusters [visualization_msgs/msg/MarkerArray]
  -> /vehicle_markers [visualization_msgs/msg/MarkerArray]

parameter_bridge_node
  -> HTTP API and static Web UI
  -> ROS 2 parameter services
```

A rendered architecture diagram is available at
[`docs/images/architecture.svg`](images/architecture.svg).

外部利用者が購読する安定トピックは`/vehicle_detections`とする。
`/vehicle_detections/raw`はノード間中継用の内部トピックであり、後続で送信方式を差し替えるために使う。

## 6. ノード設計

### 6.1 `pcd_loader_node`

責務:

- 起動時に指定PCDファイル (または再生リスト) を解決する。
- PCDを`sensor_msgs/msg/PointCloud2`へ変換し、`/input/points`へpublishする。
- 単発publishまたは周期publishを切り替える。
- Phase 2: 複数PCDの連続再生に対応する。

主要パラメータ:

| パラメータ | 型 | 初期値 | 検証 |
| --- | --- | --- | --- |
| `pcd_file` | string | `data/pcd/sample.pcd` | 再生リストが解決した場合は存在確認、未指定可 |
| `pcd_files` | string[] | `[]` | 各要素は空文字不可、存在確認 (非空時) |
| `pcd_directory` | string | `""` | 非空時はディレクトリの存在確認 |
| `pcd_glob` | string | `*.pcd` | 空文字不可 |
| `loop` | bool | `true` | なし |
| `input_frame_id` | string | `lidar` | 空文字不可 |
| `input_points_topic` | string | `/input/points` | 空文字不可 |
| `publish_once` | bool | `false` | なし |
| `publish_rate_hz` | double | `1.0` | `> 0.0` |

再生リスト解決 (Phase 2):

1. `pcd_files` が非空ならその順序のままリストとして採用する。
2. `pcd_files` が空かつ `pcd_directory` が非空なら、`pcd_directory` 配下で `pcd_glob` (既定 `*.pcd`) にマッチするファイルを `std::filesystem` でソート列挙し、リストとして採用する。
3. それ以外は `pcd_file` を 1 要素のリストとして採用する (MVP 互換)。
4. リスト全要素について存在確認を行い、欠落があれば起動失敗扱い。

処理:

1. 再生リストを解決し、各要素の絶対パス化と存在確認を行う。
2. リスト先頭から `pcl::io::loadPCDFile<pcl::PCLPointCloud2>` で読み込み、`pcl_conversions::fromPCL` でROSメッセージへ変換する。
3. `header.frame_id`へ`input_frame_id`を設定する。
4. 点数、フィールド名、PCDパスをログ出力する。
5. `publish_once=true`なら起動後 1 回 publish し、ノードは生存する。
6. `publish_once=false`なら`publish_rate_hz`周期で次のインデックスのファイルを publish する。
   - 直前にロード済みのインデックスと同一の場合は、再ロードを省略してキャッシュ済みメッセージを再利用する。
   - リストの末尾に達したとき、`loop=true` なら先頭に戻り、`loop=false` なら publish 用タイマーを停止する (ノードは生存)。
7. 各 publish 直前に、現在のインデックスとファイルパスを INFO ログに出力する。

異常時:

- 再生リストが空 (どのパラメータにも値がない) 場合はエラーログを出し、ノード初期化を失敗扱いにする。
- リストの 1 要素でも存在しないファイルがあれば、欠落パスをログ出力して起動失敗扱いにする。
- 点数0の場合は警告を出して空点群としてpublishし、検知側で安全に処理する。
- 読み込み失敗時はPCDパスとPCLエラーの概要をログ出力する。連続再生中に失敗した場合は、その回の publish をスキップして次のティックへ進める。

### 6.2 `vehicle_detector_node`

責務:

- 入力点群を`target_frame_id`へ変換する。
- PCLによる前処理とクラスタリングを行う。
- 普通車候補を`vision_msgs/msg/Detection3DArray`としてpublishする。
- RViz用Markerをpublishする。

入力:

| トピック | 型 |
| --- | --- |
| `/input/points` | `sensor_msgs/msg/PointCloud2` |

出力:

| トピック | 型 | 内容 |
| --- | --- | --- |
| `/vehicle_detections/raw` | `vision_msgs/msg/Detection3DArray` | sender向け内部検知結果 |
| `/debug/points_filtered` | `sensor_msgs/msg/PointCloud2` | ROI、地面除去後点群 |
| `/debug/clusters` | `visualization_msgs/msg/MarkerArray` | クラスタ確認用marker |
| `/vehicle_markers` | `visualization_msgs/msg/MarkerArray` | 車両bbox marker |

処理:

1. `PointCloud2`を受信する。
2. `header.frame_id`が`target_frame_id`と一致する場合は変換を省略する。
3. 一致しない場合はtf2で`target_frame_id`への変換を取得する。
4. TF取得に失敗した場合は警告を出し、検知結果をpublishしない。
5. `pcl::PointCloud<pcl::PointXYZ>`へ変換する。XYZ以外のフィールドはMVP検知では使わない。
6. VoxelGridでダウンサンプリングする。
7. CropBoxでROI範囲へ切り出す。
8. RANSAC平面推定で地面候補を除去する。
9. EuclideanClusterExtractionでクラスタを抽出する。
10. 各クラスタのaxis-aligned bboxを計算する。
11. bbox寸法で普通車候補を抽出する。
12. 検知結果、debug marker、vehicle markerをpublishする。

地面除去:

- `pcl::SACSegmentation<pcl::PointXYZ>`を使う。
- モデルは`SACMODEL_PLANE`、手法は`SAC_RANSAC`とする。
- distance thresholdは`ground_distance_threshold`を使う。
- 入力点数が`cluster_min_size`未満の場合は地面除去をスキップする。
- 平面推定に失敗した場合は警告を出し、地面除去前点群を後続へ渡す。

クラスタリング:

- `pcl::search::KdTree<pcl::PointXYZ>`を作成する。
- `cluster_tolerance`, `cluster_min_size`, `cluster_max_size`を使う。
- 抽出クラスタごとにmin/max座標を計算する。

普通車判定:

- bbox寸法は`dx = max_x - min_x`, `dy = max_y - min_y`, `dz = max_z - min_z`とする。
- `length = max(dx, dy)`, `width = min(dx, dy)`, `height = dz`とする。
- `vehicle_min_*`から`vehicle_max_*`の範囲内なら普通車候補とする。
- `yaw`はMVPでは0.0とする。
- `confidence`はMVPでは0.8固定とする。
- 検知IDは受信フレーム内で1から採番する。

`vision_msgs/msg/Detection3DArray`への対応:

- `array.header.frame_id = target_frame_id`
- `detection.header = array.header`
- `detection.id = <frame-local id>`
- `detection.bbox.center.position = bbox center`
- `detection.bbox.center.orientation = yaw 0.0のquaternion`
- `detection.bbox.size.x = length`
- `detection.bbox.size.y = width`
- `detection.bbox.size.z = height`
- `detection.results[0].hypothesis.class_id = "car"`
- `detection.results[0].hypothesis.score = confidence`

パラメータ更新:

- `on_set_parameters_callback`で範囲検証する。
- 不正値は拒否し、拒否理由を`SetParametersResult.reason`とログに出す。
- 更新対象は次フレームから反映する。
- 処理中に値が変わっても破綻しないよう、受信callback冒頭で設定値のスナップショットを作る。

### 6.3 `detection_sender_node`

責務:

- `/vehicle_detections/raw`を購読する。
- `send_mode`に応じてROS 2トピック、HTTP POST、両方、無効を切り替える。
- HTTP送信失敗時も検知系を停止させない。
- Phase 2: `save_results=true` のとき、検知結果を JSON Lines ファイルへ
  追記する。保存処理は send_mode から独立し、`disabled` でも動作する。

主要パラメータ:

| パラメータ | 型 | 初期値 | 検証 |
| --- | --- | --- | --- |
| `send_mode` | string | `ros_topic` | `disabled`, `ros_topic`, `http`, `both` |
| `raw_detections_topic` | string | `/vehicle_detections/raw` | 空文字不可 |
| `detections_topic` | string | `/vehicle_detections` | 空文字不可 |
| `http_endpoint_url` | string | `http://host.docker.internal:8080/detections` | `http://`または`https://` |
| `http_timeout_ms` | int | `1000` | `>= 1` |
| `http_retry_count` | int | `0` | `>= 0` |
| `http_auth_type` | string | `none` | MVPでは`none`のみ |
| `save_results` | bool | `false` | なし (Phase 2) |
| `result_output_path` | string | `""` | 空かつ `save_results=true` ならファイルオープン失敗扱い (Phase 2) |
| `result_output_format` | string | `jsonl` | `jsonl` のみ。それ以外は拒否 (Phase 2) |

送信モード:

| `send_mode` | ROS topic | HTTP POST |
| --- | --- | --- |
| `disabled` | しない | しない |
| `ros_topic` | する | しない |
| `http` | しない | する |
| `both` | する | する |

HTTP payload:

```json
{
  "timestamp": "2026-04-30T00:00:00.000Z",
  "frame_id": "map",
  "detections": [
    {
      "id": "1",
      "class": "car",
      "confidence": 0.8,
      "center": {"x": 12.3, "y": -1.2, "z": 0.8},
      "size": {"length": 4.3, "width": 1.8, "height": 1.6},
      "yaw": 0.0
    }
  ]
}
```

HTTP処理:

- Content-Typeは`application/json`とする。
- retryは同一callback内で最大`http_retry_count`回だけ行う。
- 送信は別スレッドまたは短いtimeoutで行い、検知callbackを長時間ブロックしない。
- 失敗時はendpoint、HTTP status、例外概要を警告ログに出す。

検知結果保存 (Phase 2):

- 専用ヘルパー `DetectionResultWriter` を `vehicle_detection_core` ライブラリに
  追加し、`detection_sender_node` から利用する。スキーマ生成は HTTP 経路と
  共有する `serialize_detections()` (1 行 JSON) をそのまま再利用する。
  保存形式は MVP では JSON Lines (`.jsonl`) のみ対応する。
- ファイルは `std::ofstream` を `std::ios::out | app | binary` で開き、
  改行は `'\n'` 固定とする。1 receive callback ごとに `serialize_detections()`
  の結果を 1 行追記する。
- `save_results=true` への遷移時、または `result_output_path` /
  `result_output_format` の更新時に、ヘルパーを再構成する。
- `result_output_format` が `jsonl` 以外なら、起動時とパラメータ更新時に
  拒否する (`SetParametersResult.successful=false`)。
- ファイルオープンに失敗した場合 (空パス、親ディレクトリ未存在、権限不足など)
  は、ヘルパーが内部状態を「無効」に戻し、ノードは警告ログを出して
  パラメータ更新自体は受理する。`detection_sender_node` はクラッシュさせない。
- 保存処理は `send_mode` から独立して動作する。`send_mode=disabled` でも
  `save_results=true` なら保存される。
- 受信 callback 内で `writer_.append(payload)` を呼ぶ。失敗時は throttled
  warning を出し、ROS topic / HTTP 送信は通常通り続行する。連続再生
  (FR-011) でも、各 publish された検知配列が publish 順に append される。

### 6.4 `parameter_bridge_node`

責務:

- ローカルWeb UIを提供する。
- HTTP APIからROS 2 parameter serviceを呼び出す。
- GUIで変更したパラメータをログ出力する。

MVPでは`parameter_gui_node`を分離せず、`parameter_bridge_node`が静的HTMLとAPIの両方を提供する。

HTTP endpoint:

| Method | Path | 内容 |
| --- | --- | --- |
| GET | `/` | `parameter_gui.html` |
| GET | `/api/parameters` | 対象ノードの現在値一覧 |
| POST | `/api/parameters` | 単一または複数パラメータ更新 |
| GET | `/api/health` | GUI/API生存確認 |

対象ノード:

- `/vehicle_detector_node`
- `/pcd_loader_node`
- `/detection_sender_node`

POST例:

```json
{
  "node": "/vehicle_detector_node",
  "parameters": {
    "voxel_leaf_size": 0.25,
    "cluster_tolerance": 0.9
  }
}
```

応答例:

```json
{
  "ok": true,
  "updated": ["voxel_leaf_size", "cluster_tolerance"],
  "rejected": []
}
```

GUI要件:

- パラメータ一覧を表示する。
- 数値パラメータはnumber inputで変更する。
- `send_mode`はselectで変更する。
- 更新、再読込、health表示を持つ。
- ブラウザはホストWindows側から`http://localhost:8081`でアクセスする想定とする。

異常時:

- 対象ノードが見つからない場合はHTTP 503相当のJSONを返す。
- パラメータ更新が拒否された場合は拒否理由をJSONに含める。
- GUIサーバが起動できなくても、他ノードのlaunchは継続可能にする。

## 7. Launch設計

起動コマンド:

```bash
ros2 launch vehicle_detection vehicle_detection.launch.py
```

launch引数:

| 引数 | 初期値 | 用途 |
| --- | --- | --- |
| `pcd_file` | `data/pcd/sample.pcd` | 入力PCDパス |
| `input_frame_id` | `lidar` | 入力点群座標系 |
| `target_frame_id` | `map` | 検知結果座標系 |
| `publish_once` | `false` | PCDを1回だけpublish |
| `send_mode` | `ros_topic` | 送信方式 |
| `use_gui` | `true` | Web GUI起動 |
| `use_rviz` | `false` | RViz2起動 |
| `rviz_config` | `config/rviz_vehicle_detection.rviz` | RViz設定 |

launch処理:

1. `detector_params.yaml`を各ノードへ読み込む。
2. launch引数で`pcd_file`, `input_frame_id`, `target_frame_id`, `send_mode`, `publish_once`を上書きできるようにする。
3. `transforms.yaml`を読み、`tf2_ros/static_transform_publisher`を起動する。
4. `pcd_loader_node`, `vehicle_detector_node`, `detection_sender_node`を起動する。
5. `use_gui=true`なら`parameter_bridge_node`を起動する。
6. `use_rviz=true`ならRViz2を起動する。

TF設定:

- `transforms.yaml`の`parent_frame_id`を`map`、`child_frame_id`を`lidar`として扱う。
- `x y z yaw pitch roll parent child`の順序ではなく、使用するROS 2 Jazzyの`static_transform_publisher --x --y --z --roll --pitch --yaw --frame-id --child-frame-id`形式で明示指定する。
- 設定が欠けていて`use_identity_if_missing=true`ならidentity transformを起動する。

## 8. 設定ファイル設計

`config/detector_params.yaml`はROS 2 node parametersとして読み込む。
単位、初期値、推奨範囲はコメントで管理する。

`config/dataset_params.yaml`はデータ取得・配置の再現性を担保するメタデータとして扱う。
実行時のPCDパスは`pcd_file`パラメータまたはlaunch引数で最終決定する。

`config/transforms.yaml`は静的TFの元データとして扱う。
launchファイルがこの内容を読んで`static_transform_publisher`へ渡す。

## 9. データ配置設計

PCDファイルはリポジトリへコミットしない。
取得後は以下へ配置する。

```text
vehicle_detection_system/data/pcd/
  sample.pcd
  README.md
```

`data/pcd/README.md`に以下を記録する。

- データセット名
- 取得元URL
- ダウンロードURL
- ライセンス
- attribution
- 取得日
- 配置したPCDファイル名
- 変換や抽出を行った場合の手順

PandaSet公式サイトは、PandaSetを自動運転向けopen-source datasetとして説明しており、学術・商用利用可能なデータセットとして案内している。
ただし、取得時点の利用条件は必ず公式Termsで再確認し、READMEに記録する。

## 10. エラー処理設計

| 事象 | 挙動 |
| --- | --- |
| PCDファイルなし | `pcd_loader_node`がエラーログを出して起動失敗 |
| PCD読み込み失敗 | PCDパス、PCLエラー概要を出して起動失敗 |
| 空点群 | 警告し、空の`PointCloud2`をpublish可能 |
| TFなし | `vehicle_detector_node`が警告し、そのフレームの検知をskip |
| 前処理後点群なし | 空の検知配列とdebug出力をpublish |
| 地面推定失敗 | 警告し、地面除去をskipして続行 |
| HTTP送信失敗 | 警告し、検知処理は継続 |
| パラメータ不正値 | parameter callbackで拒否 |
| GUI起動失敗 | GUIだけ停止し、検知系は継続 |

## 11. ログ設計

起動時:

- ノード名
- PCDファイルパス
- 点数とPCDフィールド
- input/target frame
- 主要検知パラメータ
- send_modeとHTTP endpoint
- GUI port

フレーム処理時:

- 入力点数
- 前処理後点数
- クラスタ数
- 検知数
- 処理時間

異常時:

- 原因
- 対象パラメータまたはファイル
- 復旧可能かどうか

## 12. テスト設計

単体テスト:

| 対象 | 観点 |
| --- | --- |
| `parameter_validation` | 範囲内受理、不正値拒否 |
| `point_cloud_processing` | ROI切り出し、空点群、少数点群 |
| bbox算出 | center、length、width、height |
| 車両判定 | 境界値、範囲外除外 |
| HTTP JSON変換 | 必須フィールド、空検知配列 |
| 再生リスト解決 (Phase 2) | `pcd_files`優先、`pcd_directory`展開、未指定時の単一PCDフォールバック、欠落ファイル拒否、`loop`末尾挙動 |
| 検知結果保存 (Phase 2) | 無効時 no-op、JSONL 1 行 append、複数 append の順序保持、不正パス時 no-throw、未対応フォーマット拒否 |

ROS統合テスト:

| 対象 | 観点 |
| --- | --- |
| `pcd_loader_node` | PCD読み込み、frame_id設定、単発/周期publish |
| `vehicle_detector_node` | `/input/points`受信、`/vehicle_detections/raw` publish |
| TF変換 | `lidar`入力から`map`出力 |
| TF欠落 | 検知publishなし、警告ログ |
| `detection_sender_node` | `send_mode`切替 |
| HTTP送信 | 成功、失敗時継続 |
| GUI API | GET/POST parameters |
| launch | 全体起動、GUI/RViz切替 |

手動確認:

```bash
colcon build
source install/setup.bash
ros2 launch vehicle_detection vehicle_detection.launch.py pcd_file:=data/pcd/sample.pcd
ros2 topic echo /vehicle_detections
ros2 topic echo /vehicle_markers
ros2 param set /vehicle_detector_node voxel_leaf_size 0.25
```

## 13. 実装順序

1. ROS 2パッケージ雛形を作成する。
2. `package.xml`と`CMakeLists.txt`へ依存を追加する。
3. `point_cloud_processing`と`parameter_validation`をライブラリとして実装する。
4. `pcd_loader_node`を実装し、PCD publishを確認する。
5. `vehicle_detector_node`を実装し、raw検知結果とmarkerを確認する。
6. `detection_sender_node`を実装し、ROS topic relayを確認する。
7. HTTP POSTを追加する。
8. `parameter_bridge_node`とWeb UIを追加する。
9. launchとRViz設定を追加する。
10. READMEへセットアップ、データ配置、起動、パラメータ調整方法を追記する。
11. 単体テストとlaunch確認を実行する。

## 14. 要件対応表

| 要件 | 設計上の対応 |
| --- | --- |
| FR-001 PCDデータ取得・配置 | 9章、`data/pcd/README.md` |
| FR-002 PCD読み込み | 6.1章、7章 |
| FR-002A 座標変換 | 6.2章、7章、`config/transforms.yaml` |
| FR-003 点群前処理 | 6.2章、12章 |
| FR-004 普通車候補検知 | 6.2章 |
| FR-005 検知情報送信 | 6.3章 |
| FR-012 検知結果の保存 (Phase 2) | 6.3章「検知結果保存」、12章 |
| FR-006 可視化 | 5章、6.2章、7章 |
| FR-007 GUIによるパラメータ調整 | 6.4章 |
| FR-008 設定ファイル | 8章、`config/detector_params.yaml` |
| FR-009 起動 | 7章 |
| FR-010 ログ | 11章 |
| NFR-001 実行性能 | 2章、6.2章、12章 |
| NFR-002 保守性 | 3章、6章 |
| NFR-003 再現性 | 8章、9章 |
| NFR-004 拡張性 | 5章、6.3章 |
| NFR-005 ライセンス | 4章、9章、`data/pcd/README.md` |

## 15. 完了条件

コーディング前の準備は以下を満たした時点で完了とする。

- 要件から設計への対応が本書で説明されている。
- ノード責務、トピック、メッセージ、パラメータが確定している。
- エラー時挙動が確定している。
- テスト観点が確定している。
- 実装順序が確定している。
- PCD配置とライセンス記録手順が用意されている。

本日時点で上記は完了している。
