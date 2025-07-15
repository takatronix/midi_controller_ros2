# MIDIコントローラーノード

KORG nanoKONTROL StudioなどのMIDIコントローラーからの入力をROS2トピックに変換するノードです。

## 機能

- **スライダー**: 8個のスライダー（CC 0-7）を `/midi/slider_X` トピックに変換
- **ノブ**: 8個のノブ（CC 16-23）を `/midi/knob_X` トピックに変換  
- **ボタン**: 32個のボタン（Note On/Off）を `/midi/button_X` トピックに変換
- **統合Joy**: 全ての入力を統合した `/midi/joy` トピック
- **システム**: システムメッセージを `/midi/system` トピックに変換

## インストール

### 依存関係

```bash
pip install mido python-rtmidi
```

### ビルド

```bash
cd ros2_ws
colcon build --packages-select midi_controller_node
source install/setup.bash
```

## 使用方法

### 1. 基本的な実行

```bash
# ノードを直接実行
ros2 run midi_controller_node midi_controller

# またはlaunchファイルを使用
ros2 launch midi_controller_node midi_controller_launch.py
```

### 2. パラメータ付き実行

```bash
ros2 launch midi_controller_node midi_controller_launch.py \
    midi_port_name:="nanoKONTROL Studio nanoKONTROL" \
    publish_rate:=100.0 \
    enable_debug:=true
```

### 3. トピックの確認

```bash
# 利用可能なトピックを確認
ros2 topic list | grep midi

# スライダーの値を確認
ros2 topic echo /midi/slider_0

# ボタンの状態を確認
ros2 topic echo /midi/button_0

# Joyメッセージを確認
ros2 topic echo /midi/joy
```

## トピック一覧

### 入力トピック
なし（MIDIハードウェアから直接入力）

### 出力トピック

| トピック名 | メッセージ型 | 説明 |
|-----------|-------------|------|
| `/midi/slider_X` | `std_msgs/Float32` | スライダー値 (0.0-1.0) |
| `/midi/knob_X` | `std_msgs/Float32` | ノブ値 (0.0-1.0) |
| `/midi/button_X` | `std_msgs/Bool` | ボタン状態 (true/false) |
| `/midi/joy` | `sensor_msgs/Joy` | 統合Joyメッセージ |
| `/midi/system` | `std_msgs/Int32` | システムメッセージ |

## パラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
|-------------|----|-------------|------|
| `midi_port_name` | string | "nanoKONTROL Studio nanoKONTROL" | MIDIポート名 |
| `publish_rate` | double | 100.0 | パブリッシュレート (Hz) |
| `enable_debug` | bool | true | デバッグモード |

## テスト

### 1. MIDI機能テスト（ROS2環境なし）

```bash
cd ros2_ws
python3 test_midi_node.py
```

### 2. ROS2環境でのテスト

```bash
# ターミナル1: MIDIノードを起動
ros2 run midi_controller_node midi_controller

# ターミナル2: テストノードを起動
python3 test_midi_ros2.py
```

## 対応デバイス

- **KORG nanoKONTROL Studio** (テスト済み)
- その他の標準MIDIデバイス

## トラブルシューティング

### MIDIポートが見つからない場合

```bash
# 利用可能なMIDIポートを確認
aconnect -l
amidi -l

# ポート名を確認してパラメータを調整
ros2 run midi_controller_node midi_controller --ros-args \
    -p midi_port_name:="実際のポート名"
```

### 権限エラーの場合

```bash
# ユーザーをaudioグループに追加
sudo usermod -a -G audio $USER

# 再ログイン後、権限を確認
ls -la /dev/snd/
```

## 開発者向け情報

### ファイル構造

```
midi_controller_node/
├── midi_controller_node/
│   ├── __init__.py
│   └── midi_controller.py      # メインノード
├── launch/
│   └── midi_controller_launch.py
├── test/
├── package.xml
├── setup.py
└── README.md
```

### カスタマイズ

新しいMIDIデバイスに対応する場合は、`midi_controller.py`の`process_midi_message`メソッドを修正してください。

## ライセンス

MIT License 