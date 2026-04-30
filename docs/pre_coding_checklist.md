# コーディング前チェックリスト

作成日: 2026-04-30  
対象: `vehicle_detection` MVP

## 1. 設計判断

- [x] 実装言語はC++にする。
- [x] ROS 2 Jazzy / `ament_cmake` / `colcon build`を前提にする。
- [x] 点群処理はPCLを使う。
- [x] 検知結果メッセージは`vision_msgs/msg/Detection3DArray`を標準採用する。
- [x] カスタムメッセージはMVPでは作成しない。
- [x] 入力座標系は`lidar`、出力座標系は`map`を初期値にする。
- [x] `lidar -> map`の初期TFはidentity transformにする。
- [x] GUIはQt/rqtを使わずローカルWeb UIにする。
- [x] HTTP送信とWeb GUI APIは`parameter_bridge_node`/`detection_sender_node`側に分離する。

## 2. 実装対象

- [x] `pcd_loader_node`
- [x] `vehicle_detector_node`
- [x] `detection_sender_node`
- [x] `parameter_bridge_node`
- [x] `vehicle_detection.launch.py`
- [x] `rviz_vehicle_detection.rviz`
- [x] `point_cloud_processing`共通処理
- [x] `parameter_validation`共通処理
- [x] README更新
- [x] 単体テスト
- [x] launch確認

## 3. 事前に配置するデータ

- [ ] PandaSet由来PCDサブセットを取得する。
- [ ] 利用条件とライセンスを取得時点で再確認する。
- [ ] `data/pcd/README.md`へ取得元、取得日、ライセンス、attributionを記録する。
- [ ] 使用するPCDを`data/pcd/sample.pcd`として配置する、またはlaunch引数`pcd_file`で指定する。

データ未配置でもコーディングは開始できる。
PCD読み込み確認までに1ファイル以上のPCDを配置する。

## 4. 実装前の受け入れ条件

- [x] ノード間トピックが決まっている。
- [x] パラメータ名、初期値、範囲が決まっている。
- [x] HTTP payload schemaが決まっている。
- [x] GUI APIが決まっている。
- [x] エラー時挙動が決まっている。
- [x] テスト観点が決まっている。

## 5. 最初の実装タスク

1. `src/vehicle_detection`配下にROS 2パッケージ雛形を作成する。
2. `package.xml`へROS/PCL依存を追加する。
3. `CMakeLists.txt`へノードと共通ライブラリのビルド定義を追加する。
4. `pcd_loader_node`を先に実装して、PCDから`/input/points`までを確認する。
5. `vehicle_detector_node`の前処理とbbox抽出を追加する。

## 6. コーディング開始可否

コーディング開始可。

残タスクはPCD実データ配置と正式TF値確認のみであり、どちらもidentity transformとlaunch引数で代替できる。
