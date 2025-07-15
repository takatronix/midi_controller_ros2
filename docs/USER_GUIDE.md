# MIDIコントローラー ROS2 ユーザーガイド

## 📋 目次
1. [概要](#概要)
2. [クイックスタート](#クイックスタート)
3. [トピック一覧](#トピック一覧)
4. [使用方法](#使用方法)
5. [設定](#設定)
6. [トラブルシューティング](#トラブルシューティング)
7. [サンプルコード](#サンプルコード)
8. [よくある質問](#よくある質問)

## 📖 概要

このパッケージは、USB接続MIDIコントローラー（KORG nanoKONTROL Studio等）をROS2で使用するためのものです。

### 🎯 主な機能
- **スライダー**: 8個のスライダー（0.0-1.0の値）
- **ノブ**: 8個のノブ（0.0-1.0の値）
- **ボタン**: 32個のボタン（ON/OFF状態）
- **統合Joy**: 全ての入力を統合したJoyメッセージ
- **システム**: システムメッセージ

### 🎹 対応デバイス
- **KORG nanoKONTROL Studio** (テスト済み)
- その他の標準MIDIデバイス

## 🚀 クイックスタート

### 1. インストール
```bash
# リポジトリをクローン
git clone https://github.com/takatronix/midi_controller_ros2.git
cd midi_controller_ros2

# 依存関係をインストール
pip install -r requirements.txt

# ROS2ワークスペースでビルド
cd ros2_ws
colcon build --packages-select midi_controller_node
source install/setup.bash
```

### 2. MIDIデバイスの確認
```bash
# 利用可能なMIDIポートを確認
aconnect -l
amidi -l
```

### 3. ノードの起動
```bash
# 基本的な起動
ros2 launch midi_controller_node midi_controller_launch.py

# または直接実行
ros2 run midi_controller_node midi_controller
```

### 4. 動作確認
```bash
# トピック一覧を確認
ros2 topic list | grep midi

# スライダーの値を確認
ros2 topic echo /midi/slider_0

# ボタンの状態を確認
ros2 topic echo /midi/button_0
```

## 📡 トピック一覧

### 出力トピック

| トピック名 | メッセージ型 | 説明 | 値の範囲 |
|-----------|-------------|------|----------|
| `/midi/slider_0` 〜 `/midi/slider_7` | `std_msgs/Float32` | スライダー値 | 0.0 - 1.0 |
| `/midi/knob_0` 〜 `/midi/knob_7` | `std_msgs/Float32` | ノブ値 | 0.0 - 1.0 |
| `/midi/button_0` 〜 `/midi/button_31` | `std_msgs/Bool` | ボタン状態 | true/false |
| `/midi/joy` | `sensor_msgs/Joy` | 統合Joyメッセージ | - |
| `/midi/system` | `std_msgs/Int32` | システムメッセージ | - |

### Joyメッセージの構造
```yaml
# /midi/joy トピックの内容
header:
  stamp: {sec: 123, nanosec: 456}
  frame_id: "midi_controller"
axes: [slider_0, slider_1, ..., slider_7, knob_0, knob_1, ..., knob_7]  # 16個
buttons: [button_0, button_1, ..., button_31]  # 32個
```

## 🎮 使用方法

### 基本的な使用方法

#### 1. スライダーの値を取得
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class SliderListener(Node):
    def __init__(self):
        super().__init__('slider_listener')
        self.subscription = self.create_subscription(
            Float32,
            '/midi/slider_0',
            self.slider_callback,
            10
        )
    
    def slider_callback(self, msg):
        self.get_logger().info(f'スライダー0の値: {msg.data}')

def main():
    rclpy.init()
    node = SliderListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 2. ボタンの状態を取得
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class ButtonListener(Node):
    def __init__(self):
        super().__init__('button_listener')
        self.subscription = self.create_subscription(
            Bool,
            '/midi/button_0',
            self.button_callback,
            10
        )
    
    def button_callback(self, msg):
        if msg.data:
            self.get_logger().info('ボタン0が押されました！')
        else:
            self.get_logger().info('ボタン0が離されました！')

def main():
    rclpy.init()
    node = ButtonListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 3. Joyメッセージを使用
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyListener(Node):
    def __init__(self):
        super().__init__('joy_listener')
        self.subscription = self.create_subscription(
            Joy,
            '/midi/joy',
            self.joy_callback,
            10
        )
    
    def joy_callback(self, msg):
        # スライダー値（axes[0-7]）
        slider_values = msg.axes[0:8]
        # ノブ値（axes[8-15]）
        knob_values = msg.axes[8:16]
        # ボタン状態（buttons[0-31]）
        button_states = msg.buttons
        
        self.get_logger().info(f'スライダー0: {slider_values[0]:.3f}')
        self.get_logger().info(f'ノブ0: {knob_values[0]:.3f}')
        self.get_logger().info(f'ボタン0: {button_states[0]}')

def main():
    rclpy.init()
    node = JoyListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 高度な使用方法

#### 複数のトピックを同時に監視
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Joy

class MidiController(Node):
    def __init__(self):
        super().__init__('midi_controller')
        
        # 複数のトピックをサブスクライブ
        self.slider_sub = self.create_subscription(
            Float32, '/midi/slider_0', self.slider_callback, 10
        )
        self.button_sub = self.create_subscription(
            Bool, '/midi/button_0', self.button_callback, 10
        )
        self.joy_sub = self.create_subscription(
            Joy, '/midi/joy', self.joy_callback, 10
        )
        
        self.slider_value = 0.0
        self.button_pressed = False
    
    def slider_callback(self, msg):
        self.slider_value = msg.data
        self.process_inputs()
    
    def button_callback(self, msg):
        self.button_pressed = msg.data
        self.process_inputs()
    
    def joy_callback(self, msg):
        # Joyメッセージから全ての値を取得
        pass
    
    def process_inputs(self):
        # スライダーとボタンの組み合わせで処理
        if self.button_pressed and self.slider_value > 0.5:
            self.get_logger().info('条件満足！')

def main():
    rclpy.init()
    node = MidiController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## ⚙️ 設定

### パラメータ設定

#### 起動時のパラメータ指定
```bash
ros2 launch midi_controller_node midi_controller_launch.py \
    midi_port_name:="nanoKONTROL Studio nanoKONTROL" \
    publish_rate:=100.0 \
    enable_debug:=true
```

#### パラメータ一覧
| パラメータ名 | 型 | デフォルト値 | 説明 |
|-------------|----|-------------|------|
| `midi_port_name` | string | "nanoKONTROL Studio nanoKONTROL" | MIDIポート名 |
| `publish_rate` | double | 100.0 | パブリッシュレート (Hz) |
| `enable_debug` | bool | true | デバッグモード |

### デバイス設定ファイル

`midi_controller_node/config/devices.yaml` を編集してデバイス設定を変更できます。

```yaml
devices:
  nanokontrol_studio:
    name: "KORG nanoKONTROL Studio"
    port_pattern: "nanoKONTROL"
    device_type: "nanokontrol_studio"
    
    sliders:
      cc_range: [0, 7]
      topic_prefix: "/midi/slider"
      value_range: [0.0, 1.0]
    
    knobs:
      cc_range: [16, 23]
      topic_prefix: "/midi/knob"
      value_range: [0.0, 1.0]
    
    buttons:
      note_range: [0, 31]
      topic_prefix: "/midi/button"
      value_type: "bool"
```

## 🔧 トラブルシューティング

### よくある問題と解決方法

#### 1. MIDIポートが見つからない
```bash
# 問題の確認
aconnect -l
amidi -l

# 解決方法
# 1. デバイスが正しく接続されているか確認
# 2. ポート名を確認してパラメータを調整
ros2 run midi_controller_node midi_controller --ros-args \
    -p midi_port_name:="実際のポート名"
```

#### 2. 権限エラー
```bash
# 解決方法
sudo usermod -a -G audio $USER
# 再ログイン後
ls -la /dev/snd/
```

#### 3. トピックが受信されない
```bash
# 確認手順
# 1. ノードが起動しているか確認
ros2 node list

# 2. トピックが存在するか確認
ros2 topic list | grep midi

# 3. トピックの情報を確認
ros2 topic info /midi/slider_0

# 4. デバッグモードで起動
ros2 run midi_controller_node midi_controller --ros-args -p enable_debug:=true
```

#### 4. 値が正しく変換されない
```bash
# デバッグ情報を確認
ros2 topic echo /midi/system

# 設定ファイルを確認
cat midi_controller_node/config/devices.yaml
```

### ログの確認
```bash
# ノードのログを確認
ros2 node list
ros2 node info /midi_controller_node

# 詳細なログを有効化
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
ros2 run midi_controller_node midi_controller --ros-args -p enable_debug:=true
```

## 📝 サンプルコード

### 完全なサンプルアプリケーション

#### ロボット制御の例
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import math

class MidiRobotController(Node):
    def __init__(self):
        super().__init__('midi_robot_controller')
        
        # MIDI入力をサブスクライブ
        self.midi_sub = self.create_subscription(
            Joy, '/midi/joy', self.midi_callback, 10
        )
        
        # ロボット制御用のパブリッシャー
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )
        
        self.get_logger().info('MIDIロボットコントローラーが起動しました')
    
    def midi_callback(self, msg):
        # Joyメッセージから値を取得
        axes = msg.axes
        buttons = msg.buttons
        
        # スライダー0: 前進/後退
        # スライダー1: 左右
        # ボタン0: 緊急停止
        
        if buttons[0]:  # 緊急停止ボタン
            self.stop_robot()
        else:
            # 移動速度を計算
            linear_x = (axes[0] - 0.5) * 2.0  # -1.0 〜 1.0
            angular_z = (axes[1] - 0.5) * 2.0  # -1.0 〜 1.0
            
            self.move_robot(linear_x, angular_z)
    
    def move_robot(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f'移動: 直進={linear_x:.2f}, 回転={angular_z:.2f}')
    
    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('緊急停止！')

def main():
    rclpy.init()
    node = MidiRobotController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### データ記録の例
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import csv
import time
from datetime import datetime

class MidiDataLogger(Node):
    def __init__(self):
        super().__init__('midi_data_logger')
        
        self.midi_sub = self.create_subscription(
            Joy, '/midi/joy', self.midi_callback, 10
        )
        
        # CSVファイルの準備
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"midi_data_{timestamp}.csv"
        
        with open(self.filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['timestamp', 'slider_0', 'slider_1', 'knob_0', 'button_0'])
        
        self.get_logger().info(f'データ記録を開始: {self.filename}')
    
    def midi_callback(self, msg):
        timestamp = time.time()
        axes = msg.axes
        buttons = msg.buttons
        
        # データをCSVに記録
        with open(self.filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                timestamp,
                axes[0],  # slider_0
                axes[1],  # slider_1
                axes[8],  # knob_0
                buttons[0]  # button_0
            ])

def main():
    rclpy.init()
    node = MidiDataLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## ❓ よくある質問

### Q1: 他のMIDIデバイスを使用できますか？
A: はい、標準的なMIDIデバイスであれば使用可能です。`devices.yaml`で設定を追加してください。

### Q2: トピックの更新頻度はどのくらいですか？
A: デフォルトで100Hzです。`publish_rate`パラメータで変更可能です。

### Q3: 複数のMIDIデバイスを同時に使用できますか？
A: 現在のバージョンでは1つのデバイスのみ対応しています。

### Q4: 値の範囲を変更できますか？
A: はい、`devices.yaml`の`value_range`で変更可能です。

### Q5: エラーが発生した場合はどうすればよいですか？
A: まず`enable_debug:=true`でデバッグモードを有効にし、ログを確認してください。

## 📞 サポート

問題が解決しない場合は、以下の情報とともにIssueを作成してください：

1. 使用しているMIDIデバイス
2. ROS2のバージョン
3. エラーメッセージ
4. 実行したコマンド
5. ログ出力

---

**最終更新**: 2024年12月
**バージョン**: 1.0.0 