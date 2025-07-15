# ROS2 MIDIコントローラーノード

このパッケージは、USB接続MIDIコントローラー（例：KORG nanoKONTROL Studio等）をROS2ノードとして利用するためのものです。
MIDIメッセージ（スライダー、ノブ、ボタン、システムメッセージ）をROS2トピックに変換・配信します。YAML設定でマッピングやデバイス追加も柔軟に対応。

## 特徴
- USB MIDIコントローラーのプラグ＆プレイ対応
- MIDIメッセージをROS2トピックに変換（スライダー、ノブ、ボタン、システム）
- YAMLによるデバイスマッピング・設定
- モジュール化された拡張性の高いPythonコード
- テストスクリプト・サンプルlaunch付属

## 前提条件
- Ubuntu 22.04.5 LTS (Jammy Jellyfish)
- Python 3.10.12
- ROS2 Humble
- USB MIDIコントローラー
- ALSA MIDIサポート

## インストール
```bash
# このリポジトリをクローン
$ git clone https://github.com/takatronix/midi_controller_ros2.git
$ cd midi_controller_ros2

# 依存パッケージのインストール
$ pip install -r requirements.txt

# （オプション）ROS2パッケージとしてビルド
$ colcon build
```

## 使い方
### ROS2ノードとして起動
```bash
# ROS2環境をsource
$ source /opt/ros/<ros2-distro>/setup.bash

# ノードを起動
$ ros2 launch midi_controller_node midi_controller_launch.py
```

### テストスクリプト（スタンドアロン）
```bash
$ python3 test_midi_ros2.py
```

## サポートデバイス
- KORG nanoKONTROL Studio（デフォルトマッピング）
- 汎用USB MIDIコントローラー（設定で追加可能）

## 設定
`midi_controller_node/config/devices.yaml` を編集してマッピングやデバイス追加が可能です。

## 詳細ドキュメント
- [ユーザーガイド](docs/USER_GUIDE.md) - 包括的な使用方法とサンプル
- [APIリファレンス](docs/API_REFERENCE.md) - 詳細な仕様とトピック情報

## ライセンス
[LICENSE](LICENSE) を参照してください。

## 貢献
Pull Request・Issue歓迎！

## 連絡先
大塚崇

---

## English Documentation
[README in English](docs/README_en.md) is also available.
