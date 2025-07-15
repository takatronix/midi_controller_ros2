# MIDIコントローラー ROS2 API リファレンス

## 📋 目次
1. [概要](#概要)
2. [トピック仕様](#トピック仕様)
3. [メッセージ型](#メッセージ型)
4. [パラメータ](#パラメータ)
5. [ノード情報](#ノード情報)
6. [設定ファイル](#設定ファイル)
7. [エラーコード](#エラーコード)

## 📖 概要

このドキュメントは、MIDIコントローラーノードの詳細なAPI仕様を説明します。

## 📡 トピック仕様

### 出力トピック

#### スライダートピック
| トピック名 | メッセージ型 | QoS設定 | 説明 |
|-----------|-------------|---------|------|
| `/midi/slider_0` | `std_msgs/Float32` | 10 | スライダー0の値 |
| `/midi/slider_1` | `std_msgs/Float32` | 10 | スライダー1の値 |
| `/midi/slider_2` | `std_msgs/Float32` | 10 | スライダー2の値 |
| `/midi/slider_3` | `std_msgs/Float32` | 10 | スライダー3の値 |
| `/midi/slider_4` | `std_msgs/Float32` | 10 | スライダー4の値 |
| `/midi/slider_5` | `std_msgs/Float32` | 10 | スライダー5の値 |
| `/midi/slider_6` | `std_msgs/Float32` | 10 | スライダー6の値 |
| `/midi/slider_7` | `std_msgs/Float32` | 10 | スライダー7の値 |

**値の範囲**: 0.0 - 1.0
**更新頻度**: パラメータ`publish_rate`で設定（デフォルト: 100Hz）

#### ノブトピック
| トピック名 | メッセージ型 | QoS設定 | 説明 |
|-----------|-------------|---------|------|
| `/midi/knob_0` | `std_msgs/Float32` | 10 | ノブ0の値 |
| `/midi/knob_1` | `std_msgs/Float32` | 10 | ノブ1の値 |
| `/midi/knob_2` | `std_msgs/Float32` | 10 | ノブ2の値 |
| `/midi/knob_3` | `std_msgs/Float32` | 10 | ノブ3の値 |
| `/midi/knob_4` | `std_msgs/Float32` | 10 | ノブ4の値 |
| `/midi/knob_5` | `std_msgs/Float32` | 10 | ノブ5の値 |
| `/midi/knob_6` | `std_msgs/Float32` | 10 | ノブ6の値 |
| `/midi/knob_7` | `std_msgs/Float32` | 10 | ノブ7の値 |

**値の範囲**: 0.0 - 1.0
**更新頻度**: パラメータ`publish_rate`で設定（デフォルト: 100Hz）

#### ボタントピック
| トピック名 | メッセージ型 | QoS設定 | 説明 |
|-----------|-------------|---------|------|
| `/midi/button_0` 〜 `/midi/button_31` | `std_msgs/Bool` | 10 | ボタン0-31の状態 |

**値**: true（押下）/ false（離上）
**更新頻度**: イベント駆動（ボタン操作時のみ）

#### 統合Joyトピック
| トピック名 | メッセージ型 | QoS設定 | 説明 |
|-----------|-------------|---------|------|
| `/midi/joy` | `sensor_msgs/Joy` | 10 | 全ての入力を統合したJoyメッセージ |

**更新頻度**: パラメータ`publish_rate`で設定（デフォルト: 100Hz）

#### システムトピック
| トピック名 | メッセージ型 | QoS設定 | 説明 |
|-----------|-------------|---------|------|
| `/midi/system` | `std_msgs/Int32` | 10 | システムメッセージ |

**値**: MIDIシステムメッセージの値
**更新頻度**: イベント駆動（システムメッセージ受信時のみ）

## 📨 メッセージ型

### std_msgs/Float32
```yaml
# スライダー・ノブ用メッセージ
data: 0.5  # float32: 0.0-1.0の範囲
```

### std_msgs/Bool
```yaml
# ボタン用メッセージ
data: true  # bool: true/false
```

### std_msgs/Int32
```yaml
# システムメッセージ用
data: 123  # int32: MIDIシステムメッセージ値
```

### sensor_msgs/Joy
```yaml
# 統合Joyメッセージ
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: "midi_controller"
axes: [0.5, 0.3, 0.7, 0.2, 0.8, 0.1, 0.9, 0.4, 0.6, 0.0, 0.5, 0.3, 0.7, 0.2, 0.8, 0.1]  # 16個
buttons: [false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true]  # 32個
```

**axes配列の構造**:
- `axes[0-7]`: スライダー0-7の値
- `axes[8-15]`: ノブ0-7の値

**buttons配列の構造**:
- `buttons[0-31]`: ボタン0-31の状態

## ⚙️ パラメータ

### 基本パラメータ

#### midi_port_name
- **型**: string
- **デフォルト値**: "nanoKONTROL Studio nanoKONTROL"
- **説明**: MIDIポート名
- **設定例**:
  ```bash
  ros2 run midi_controller_node midi_controller --ros-args \
      -p midi_port_name:="KORG nanoKONTROL Studio"
  ```

#### publish_rate
- **型**: double
- **デフォルト値**: 100.0
- **範囲**: 1.0 - 1000.0
- **説明**: パブリッシュレート（Hz）
- **設定例**:
  ```bash
  ros2 run midi_controller_node midi_controller --ros-args \
      -p publish_rate:=50.0
  ```

#### enable_debug
- **型**: bool
- **デフォルト値**: true
- **説明**: デバッグモードの有効/無効
- **設定例**:
  ```bash
  ros2 run midi_controller_node midi_controller --ros-args \
      -p enable_debug:=false
  ```

### パラメータの確認・変更

#### パラメータ一覧の確認
```bash
ros2 param list /midi_controller_node
```

#### パラメータ値の確認
```bash
ros2 param get /midi_controller_node midi_port_name
ros2 param get /midi_controller_node publish_rate
ros2 param get /midi_controller_node enable_debug
```

#### パラメータ値の変更（実行中）
```bash
ros2 param set /midi_controller_node publish_rate 50.0
```

#### 全パラメータの表示
```bash
ros2 param dump /midi_controller_node
```

## 🖥️ ノード情報

### ノード名
- **デフォルト**: `/midi_controller_node`
- **変更可能**: launchファイルで指定

### ノードの確認
```bash
# ノード一覧
ros2 node list

# ノード情報
ros2 node info /midi_controller_node
```

### ノードの詳細情報
```bash
# パブリッシャー一覧
ros2 node info /midi_controller_node | grep Publishers

# サブスクライバー一覧
ros2 node info /midi_controller_node | grep Subscribers

# サービス一覧
ros2 node info /midi_controller_node | grep Service
```

## 📄 設定ファイル

### devices.yaml

#### ファイルパス
```
midi_controller_node/config/devices.yaml
```

#### 設定構造
```yaml
devices:
  # デバイス設定
  nanokontrol_studio:
    name: "KORG nanoKONTROL Studio"
    port_pattern: "nanoKONTROL"
    device_type: "nanokontrol_studio"
    
    # スライダー設定
    sliders:
      cc_range: [0, 7]           # MIDI CC番号範囲
      topic_prefix: "/midi/slider"
      value_range: [0.0, 1.0]    # 値の範囲
      description: "8個のスライダー"
    
    # ノブ設定
    knobs:
      cc_range: [16, 23]         # MIDI CC番号範囲
      topic_prefix: "/midi/knob"
      value_range: [0.0, 1.0]    # 値の範囲
      description: "8個のノブ"
    
    # ボタン設定
    buttons:
      note_range: [0, 31]        # MIDI Note番号範囲
      topic_prefix: "/midi/button"
      value_type: "bool"         # 値の型
      description: "32個のボタン"
    
    # ピッチベンド設定
    pitch_wheel:
      topic_prefix: "/midi/pitch"
      value_range: [0.0, 1.0]
      description: "ピッチベンド機能"
    
    # システムメッセージ設定
    system:
      topic_name: "/midi/system"
      value_type: "int32"
      description: "システムメッセージ"

# グローバル設定
global_settings:
  # デフォルトパラメータ
  default_publish_rate: 100.0
  default_enable_debug: true
  
  # エラーハンドリング
  max_connection_retries: 3
  connection_timeout: 5.0
  
  # パフォーマンス設定
  buffer_size: 1000
  poll_interval: 0.001  # 1ms
  
  # ログ設定
  log_level: "INFO"
  enable_statistics: true
```

#### 設定項目の詳細

##### デバイス設定
| 項目 | 型 | 説明 |
|------|----|------|
| `name` | string | デバイス名 |
| `port_pattern` | string | ポート名のパターン（正規表現） |
| `device_type` | string | デバイスタイプ |

##### スライダー設定
| 項目 | 型 | 説明 |
|------|----|------|
| `cc_range` | array | MIDI CC番号の範囲 [min, max] |
| `topic_prefix` | string | トピック名のプレフィックス |
| `value_range` | array | 値の範囲 [min, max] |
| `description` | string | 説明文 |

##### ノブ設定
| 項目 | 型 | 説明 |
|------|----|------|
| `cc_range` | array | MIDI CC番号の範囲 [min, max] |
| `topic_prefix` | string | トピック名のプレフィックス |
| `value_range` | array | 値の範囲 [min, max] |
| `description` | string | 説明文 |

##### ボタン設定
| 項目 | 型 | 説明 |
|------|----|------|
| `note_range` | array | MIDI Note番号の範囲 [min, max] |
| `topic_prefix` | string | トピック名のプレフィックス |
| `value_type` | string | 値の型（"bool"） |
| `description` | string | 説明文 |

##### グローバル設定
| 項目 | 型 | 説明 |
|------|----|------|
| `default_publish_rate` | float | デフォルトのパブリッシュレート |
| `default_enable_debug` | bool | デフォルトのデバッグモード |
| `max_connection_retries` | int | 最大接続リトライ回数 |
| `connection_timeout` | float | 接続タイムアウト（秒） |
| `buffer_size` | int | バッファサイズ |
| `poll_interval` | float | ポーリング間隔（秒） |
| `log_level` | string | ログレベル |
| `enable_statistics` | bool | 統計情報の有効/無効 |

## ⚠️ エラーコード

### MIDI接続エラー
| エラーコード | 説明 | 解決方法 |
|-------------|------|----------|
| `MIDI_PORT_NOT_FOUND` | MIDIポートが見つからない | ポート名を確認、デバイス接続を確認 |
| `MIDI_PORT_ACCESS_DENIED` | MIDIポートへのアクセスが拒否された | 権限を確認、audioグループに追加 |
| `MIDI_PORT_BUSY` | MIDIポートが使用中 | 他のアプリケーションを終了 |

### 設定エラー
| エラーコード | 説明 | 解決方法 |
|-------------|------|----------|
| `CONFIG_FILE_NOT_FOUND` | 設定ファイルが見つからない | ファイルパスを確認 |
| `CONFIG_PARSE_ERROR` | 設定ファイルの解析エラー | YAML構文を確認 |
| `INVALID_DEVICE_CONFIG` | 無効なデバイス設定 | 設定値を確認 |

### 実行時エラー
| エラーコード | 説明 | 解決方法 |
|-------------|------|----------|
| `MIDI_MESSAGE_PARSE_ERROR` | MIDIメッセージの解析エラー | デバッグモードで詳細を確認 |
| `TOPIC_PUBLISH_ERROR` | トピックパブリッシュエラー | ROS2環境を確認 |
| `MEMORY_ALLOCATION_ERROR` | メモリ割り当てエラー | システムリソースを確認 |

### エラーログの確認
```bash
# ログレベルの設定
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"

# デバッグモードで起動
ros2 run midi_controller_node midi_controller --ros-args -p enable_debug:=true
```

### エラー処理の例
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class ErrorHandlingExample(Node):
    def __init__(self):
        super().__init__('error_handling_example')
        
        # エラーハンドリング付きサブスクライバー
        self.subscription = self.create_subscription(
            Float32,
            '/midi/slider_0',
            self.slider_callback,
            10
        )
    
    def slider_callback(self, msg):
        try:
            # 値の範囲チェック
            if not (0.0 <= msg.data <= 1.0):
                self.get_logger().warn(f'無効な値: {msg.data}')
                return
            
            # 正常処理
            self.get_logger().info(f'スライダー値: {msg.data}')
            
        except Exception as e:
            self.get_logger().error(f'エラーが発生しました: {e}')

def main():
    rclpy.init()
    node = ErrorHandlingExample()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'予期しないエラー: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

**最終更新**: 2024年12月
**バージョン**: 1.0.0 