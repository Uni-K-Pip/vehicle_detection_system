# PCDデータ配置メモ

このディレクトリには、実行時に使用するPCDファイルを配置する。
PCDファイル本体はサイズとライセンス管理の都合でリポジトリへコミットしない。

## 初期想定データセット

| 項目 | 内容 |
| --- | --- |
| データセット名 | PandaSet Lidar PCD subset |
| 元データ | PandaSet by Hesai and Scale AI |
| source URL | https://www.pandaset.org/ |
| download URL | https://ssd.mathworks.com/supportfiles/driving/data/PandasetLidarData.zip |
| ライセンス | CC BY 4.0想定。取得時点の公式Termsで再確認する |
| attribution | PandaSet by Hesai and Scale AI |

## 配置ルール

- 代表サンプルは`sample.pcd`として配置する。
- 複数ファイルを使う場合はサブディレクトリを作成する。
- launch時に別ファイルを使う場合は`pcd_file:=<path>`で指定する。

## 取得記録テンプレート

```text
取得日:
取得者:
データセット名:
取得元URL:
ダウンロードURL:
ライセンス:
attribution:
配置ファイル:
抽出・変換手順:
備考:
```
