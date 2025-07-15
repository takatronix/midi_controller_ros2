# 🎹 MIDI Controller for ROS2

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-green.svg)](https://docs.ros.org/en/humble/)

KORG nanoKONTROL StudioなどのMIDIコントローラーをROS2システムで使用するための包括的なソリューションです。

## ✨ 特徴

- 🎛️ **複数MIDIデバイス対応**: KORG nanoKONTROL Studio、汎用MIDIデバイス
- 🔧 **モジュラー設計**: 再利用可能で拡張しやすいアーキテクチャ
- ⚙️ **設定ファイル**: YAMLファイルでデバイス設定を管理
- 🎯 **ROS2統合**: 標準的なROS2トピックに変換
- 🧪 **包括的テスト**: 単体テストから統合テストまで
- 📚 **詳細ドキュメント**: セットアップから応用例まで

## 🚀 クイックスタート

### 前提条件

- Ubuntu 20.04+ / ROS2 Humble
- Python 3.8+
- MIDIコントローラー（KORG nanoKONTROL Studio推奨）

### インストール

```bash
# リポジトリをクローン
git clone https://github.com/yourusername/midi-controller-ros2.git
cd midi-controller-ros2

# 依存関係をインストール
pip install mido python-rtmidi pyyaml

# ROS2ワークスペースをビルド
colcon build --packages-select midi_controller_node
source install/setup.bash
```

### 基本的な使用

```bash
# MIDI機能テスト
python3 test_midi_node.py

# ROS2ノード実行
ros2 run midi_controller_node midi_controller

# トピック確認
ros2 topic echo /midi/slider_0
```

## 📖 ドキュメント

- [📋 プロジェクト概要](MIDI_CONTROLLER_PROJECT.md)
- [🔄 リファクタリング計画](REFACTORING_PLAN.md)
- [🎛️ デバイス設定](src/midi_controller_node/config/devices.yaml)
- [🧪 テストガイド](docs/testing.md)

## 🎛️ 対応デバイス

### ✅ テスト済み
- **KORG nanoKONTROL Studio**
  - 8個のスライダー (CC 0-7)
  - 8個のノブ (CC 16-23)
  - 32個のボタン (Note 0-31)
  - ピッチベンド機能

### 🔧 理論上対応
- その他の標準MIDIデバイス
- カスタムマッピングで任意のデバイスに対応可能

## 📡 ROS2トピック

### 出力トピック

| トピック名 | メッセージ型 | 説明 |
|-----------|-------------|------|
| `/midi/slider_X` | `std_msgs/Float32` | スライダー値 (0.0-1.0) |
| `/midi/knob_X` | `std_msgs/Float32` | ノブ値 (0.0-1.0) |
| `/midi/button_X` | `std_msgs/Bool` | ボタン状態 (true/false) |
| `/midi/joy` | `sensor_msgs/Joy` | 統合Joyメッセージ |
| `/midi/system` | `std_msgs/Int32` | システムメッセージ |

## 🔧 カスタマイズ

### 新しいデバイス対応

```python
from midi_controller_node.midi.devices import GenericMidiDevice

# カスタム設定
custom_config = {
    'mappings': {
        'sliders': {
            'cc_range': [10, 17],
            'topic_prefix': '/custom/slider'
        }
    }
}

device = GenericMidiDevice(config=custom_config)
```

### 設定ファイルでの管理

```yaml
# config/devices.yaml
devices:
  my_device:
    name: "My MIDI Device"
    mappings:
      sliders:
        cc_range: [0, 7]
        topic_prefix: "/my/slider"
```

## 🧪 テスト

```bash
# 基本機能テスト
python3 test_midi_node.py

# リファクタリング後テスト
python3 test_refactored_modules.py

# 再利用性デモ
python3 reusability_demo.py

# 詳細監視
python3 midi_monitor_detailed.py
```

## 🏗️ プロジェクト構造

```
midi-controller-ros2/
├── src/midi_controller_node/          # ROS2パッケージ
│   ├── midi_controller_node/
│   │   ├── midi/                      # MIDI処理モジュール
│   │   │   ├── midi_device.py         # 基本デバイスクラス
│   │   │   ├── midi_message.py        # メッセージ処理
│   │   │   ├── midi_mapper.py         # マッピング機能
│   │   │   └── devices/               # デバイス固有実装
│   │   │       ├── nanokontrol.py     # KORG nanoKONTROL
│   │   │       └── generic_device.py  # 汎用デバイス
│   │   ├── ros2/                      # ROS2統合
│   │   └── utils/                     # ユーティリティ
│   ├── config/                        # 設定ファイル
│   │   └── devices.yaml
│   ├── launch/                        # 起動ファイル
│   └── test/                          # パッケージテスト
├── test_midi_node.py                  # MIDI機能テスト
├── test_midi_ros2.py                  # ROS2統合テスト
├── test_refactored_modules.py         # リファクタリングテスト
├── reusability_demo.py                # 再利用性デモ
├── midi_monitor_detailed.py           # 詳細監視ツール
├── MIDI_CONTROLLER_PROJECT.md         # プロジェクト詳細
├── REFACTORING_PLAN.md                # リファクタリング計画
└── README.md                          # このファイル
```

## 🤝 貢献

プルリクエストやイシューの報告を歓迎します！

### 開発環境セットアップ

```bash
# 開発用依存関係
pip install -r requirements-dev.txt

# テスト実行
python3 -m pytest tests/

# コードフォーマット
black src/
isort src/
```

### 貢献ガイドライン

1. フォークしてブランチを作成
2. 機能を実装
3. テストを追加
4. プルリクエストを作成

## 📄 ライセンス

このプロジェクトはMITライセンスの下で公開されています。詳細は[LICENSE](LICENSE)ファイルを参照してください。

## 🙏 謝辞

- [mido](https://github.com/mido/mido) - MIDIライブラリ
- [python-rtmidi](https://github.com/SpotlightKid/python-rtmidi) - リアルタイムMIDI
- [ROS2](https://docs.ros.org/) - ロボットオペレーティングシステム

## 📞 サポート

- 🐛 **バグ報告**: [Issues](https://github.com/yourusername/midi-controller-ros2/issues)
- 💡 **機能要望**: [Discussions](https://github.com/yourusername/midi-controller-ros2/discussions)
- 📧 **連絡先**: your.email@example.com

---

⭐ このプロジェクトが役に立ったら、スターを付けてください！ 