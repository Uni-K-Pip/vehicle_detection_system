# 既知の制約

本 MVP は、機能の幅よりも明瞭さを優先することを意図的に選んでいる。
レビュー担当者がコードを読む前にスコープを把握できるよう、以下の制約を
明示しておく。

## 検知モデル

- **単一フレームの幾何検知器。** トラッキング、時系列平滑化、運動推定は
  いずれも行わない。各フレームは独立に処理される。
- **分類器はプレースホルダ。** 普通車サイズフィルタを通過したすべての
  クラスタを `class=car, score=0.8` で publish する。学習済み分類器は
  存在せず、サイズフィルタが唯一のラベル判定。
- **identity orientation のみ。** AABB ベースの検知は identity quaternion
  で publish され、JSON ペイロードでは `yaw=0.0` を返す。yaw 推定を
  含む oriented bounding box は MVP のスコープ外。
- **地面除去はほぼ水平な地面を仮定。** RANSAC は z 軸方向の
  `SACMODEL_PERPENDICULAR_PLANE` (約 15 度の許容) に制約されている。
  傾斜地や複数階層の地面を扱うには、別途前処理または異なる地面モデルが
  必要となる。

## パイプライン／ランタイム

- **シングルスレッド executor。** デフォルト executor 上で
  `rclcpp::spin` する。本ノードは 100k〜200k 点規模で約 1 Hz の LiDAR
  フレームを想定したサイジングであり、高レート入力向けの調整は
  していない。
- **フレーム当たりの処理時間 ≒ 150〜170 ms** (記録した PandaSet
  サンプル時。入力 118,784 点 → フィルタ後 9,627 点 → 66 クラスタ →
  車両検知 7 件)。詳細は [`results.md`](results.md) を参照。
- **バッチング・ゼロコピーなし。** PointCloud2 メッセージは通常の
  DDS publish/subscribe でシリアライズされる。デモ向け
  であり、量産 AV スタック用ではない。

## 座標系／TF

- launch ファイルは `target_frame_id` から `input_frame_id` への
  identity な static transform を同梱しており、実測の extrinsics が
  なくてもデモが動作するようにしてある。実環境では
  `tf2_ros::static_transform_publisher` 経由で実測値に置き換える必要が
  ある。
- TF lookup はメッセージのタイムスタンプを使い、タイムアウトは 0.1 秒。
  パイプラインの長時間停止やタイムスタンプの順序ずれが起きると
  フレームをドロップする。

## 外部出力

- `detection_sender_node` の HTTP 経路は **HTTP/1.1 平文のみ**。
  TLS / HTTPS、証明書検証、`http_auth_type=none` 以上の認証ヘッダは
  実装していない。
- 固定キューの上限は 32 ペイロード。受信側が遅い場合、**最古** の
  ペイロードを FIFO で破棄し、抑制付きの警告ログを出す。
  ロスは publisher 側へは通知されない。

## ビルド／データ

- ビルドと実行は `osrf/ros:jazzy-desktop` イメージ内でのみ動作確認している。
  ネイティブ Windows / macOS の ROS 2 はスコープ外。
- PCD ファイルはコミットしない ([`.gitignore`](../.gitignore) を参照)。
  記録済みの結果に使った PandaSet サブセットの取得方法は
  `data/pcd/README.md` に記載。
