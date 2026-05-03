# HTTP ペイロードスキーマ

`detection_sender_node` は、`send_mode` が `http` または `both` のとき、
以下の JSON ボディを `http_endpoint_url` に POST する。1 回の POST は、
`raw_detections_topic` で受信した 1 件の
`vision_msgs/msg/Detection3DArray` メッセージに対応する。

```
POST {http_endpoint_url}
Content-Type: application/json
```

## Body

```jsonc
{
  "timestamp": "2026-04-30T12:34:56.123Z",  // ISO 8601 UTC, ms precision
  "frame_id": "map",                         // header.frame_id
  "detections": [
    {
      "id": "1",                             // Detection3D.id
      "class": "car",                        // first hypothesis.class_id
      "confidence": 0.8,                     // first hypothesis.score
      "center": {                            // bbox.center.position
        "x": 20.29,
        "y": -9.41,
        "z": -0.95
      },
      "size": {                              // semantic length/width/height
        "length": 4.35,                      //   max(dx, dy)
        "width": 2.54,                       //   min(dx, dy)
        "height": 1.79                       //   dz
      },
      "yaw": 0.0                             // identity orientation
    }
  ]
}
```

## 補足

- 上流の `vehicle_detector_node` がそのフレームで普通車候補を出さなかった
  場合、`detections` は空配列になりうる。
- 数値フィールドは 6 桁の固定精度。非有限値 (NaN / Inf) は `0.0` として
  シリアライズする。
- `class` と `confidence` は、`Detection3D.results` の先頭
  `ObjectHypothesisWithPose` を反映する。MVP では `class=car`、
  `confidence=0.8` をハードコードしている。
- `yaw` は現状常に `0.0`。AABB の姿勢推定は MVP のスコープ外。
- 送信側は 2xx 応答を期待する。2xx 以外、ネットワークエラー、
  タイムアウトはログに記録するが、ROS コールバックはブロックしない
  (送信は最大 32 ペイロードの固定キューを持つバックグラウンドワーカー
  スレッド上で実行される)。

## ローカル保存 (JSON Lines, Phase 2)

`detection_sender_node` は `save_results=true` のとき、上記と同じ JSON
ボディを 1 フレーム 1 行として、`result_output_path` で指定した
JSON Lines (`.jsonl`) ファイル末尾へ追記する。スキーマは HTTP POST と
同一実装 (`serialize_detections`) を共有するため、HTTP の payload を
そのままファイル化したものと等価である。

- ファイルは追記モードで開くため、同一パスでの再起動はファイル末尾に
  続けて書き込む。
- 改行は `\n` 固定。ファイルは UTF-8 想定。
- 保存処理は `send_mode` と独立しているため、`send_mode=disabled` でも
  保存できる。ファイルが開けない場合 (空パス、親ディレクトリなし、
  権限不足など) は警告ログを出し、保存だけが停止する。ROS 送信と
  HTTP 送信の挙動には影響しない。
- `result_output_format` は `jsonl` のみ受け付ける。それ以外は起動時と
  パラメータ更新時に拒否される。
