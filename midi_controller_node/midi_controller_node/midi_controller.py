#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Bool
from sensor_msgs.msg import Joy
import mido
import threading
import time
from typing import Dict, List, Optional


class MidiControllerNode(Node):
    """
    ROS2ノード: MIDIコントローラーからの入力をROS2トピックに変換
    KORG nanoKONTROL Studio対応
    """
    
    def __init__(self):
        super().__init__('midi_controller_node')
        
        # パラメータ設定
        self.declare_parameter('midi_port_name', 'nanoKONTROL Studio nanoKONTROL')
        self.declare_parameter('publish_rate', 100.0)  # Hz
        self.declare_parameter('enable_debug', True)
        
        self.midi_port_name = self.get_parameter('midi_port_name').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.enable_debug = self.get_parameter('enable_debug').value
        
        # MIDIポート
        self.midi_port: Optional[mido.ports.BaseInput] = None
        
        # パブリッシャー
        self.publishers: Dict[str, any] = {}
        self.setup_publishers()
        
        # MIDIメッセージバッファ
        self.midi_messages = []
        self.midi_lock = threading.Lock()
        
        # 実行フラグ
        self.running = True
        
        # MIDIポート接続
        self.connect_midi_port()
        
        # タイマー設定
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_midi_data)
        
        # MIDI受信スレッド開始
        self.midi_thread = threading.Thread(target=self.midi_receiver_loop)
        self.midi_thread.daemon = True
        self.midi_thread.start()
        
        self.get_logger().info('MIDIコントローラーノードが起動しました')
        self.get_logger().info(f'MIDIポート: {self.midi_port_name}')
    
    def setup_publishers(self):
        """ROS2パブリッシャーを設定"""
        
        # スライダー用 (CC 0-7)
        for i in range(8):
            self.publishers[f'slider_{i}'] = self.create_publisher(
                Float32, f'/midi/slider_{i}', 10
            )
        
        # ノブ用 (CC 16-23)
        for i in range(8):
            self.publishers[f'knob_{i}'] = self.create_publisher(
                Float32, f'/midi/knob_{i}', 10
            )
        
        # ボタン用 (Note On/Off)
        for i in range(32):
            self.publishers[f'button_{i}'] = self.create_publisher(
                Bool, f'/midi/button_{i}', 10
            )
        
        # 統合Joyメッセージ
        self.publishers['joy'] = self.create_publisher(
            Joy, '/midi/joy', 10
        )
        
        # システムメッセージ
        self.publishers['system'] = self.create_publisher(
            Int32, '/midi/system', 10
        )
    
    def connect_midi_port(self):
        """MIDIポートに接続"""
        try:
            # 利用可能なMIDIポートを確認
            input_ports = mido.get_input_names()
            self.get_logger().info(f'利用可能なMIDIポート: {input_ports}')
            
            # 指定されたポートを探す
            target_port = None
            for port_name in input_ports:
                if self.midi_port_name in port_name:
                    target_port = port_name
                    break
            
            if target_port is None:
                self.get_logger().error(f'MIDIポート "{self.midi_port_name}" が見つかりません')
                self.get_logger().error('利用可能なポートを使用します')
                if input_ports:
                    target_port = input_ports[0]
                else:
                    self.get_logger().error('利用可能なMIDIポートがありません')
                    return
            
            # MIDIポートを開く
            self.midi_port = mido.open_input(target_port)
            self.get_logger().info(f'MIDIポート "{target_port}" に接続しました')
            
        except Exception as e:
            self.get_logger().error(f'MIDIポート接続エラー: {e}')
    
    def midi_receiver_loop(self):
        """MIDIメッセージ受信ループ"""
        if self.midi_port is None:
            self.get_logger().error('MIDIポートが接続されていません')
            return
        
        self.get_logger().info('MIDI受信ループを開始しました')
        
        try:
            for msg in self.midi_port.iter_pending():
                if not self.running:
                    break
                
                with self.midi_lock:
                    self.midi_messages.append(msg)
                
                if self.enable_debug:
                    self.get_logger().debug(f'MIDI受信: {msg}')
        
        except Exception as e:
            self.get_logger().error(f'MIDI受信エラー: {e}')
    
    def process_midi_message(self, msg):
        """MIDIメッセージを処理してROS2メッセージに変換"""
        
        if msg.type == 'control_change':
            # コントロールチェンジ（スライダー・ノブ）
            cc_number = msg.control
            value = msg.value / 127.0  # 0-127を0.0-1.0に正規化
            
            if 0 <= cc_number <= 7:
                # スライダー
                publisher = self.publishers.get(f'slider_{cc_number}')
                if publisher:
                    ros_msg = Float32()
                    ros_msg.data = value
                    publisher.publish(ros_msg)
            
            elif 16 <= cc_number <= 23:
                # ノブ
                knob_index = cc_number - 16
                publisher = self.publishers.get(f'knob_{knob_index}')
                if publisher:
                    ros_msg = Float32()
                    ros_msg.data = value
                    publisher.publish(ros_msg)
        
        elif msg.type in ['note_on', 'note_off']:
            # ノート（ボタン）
            note_number = msg.note
            velocity = msg.velocity
            
            if 0 <= note_number <= 31:
                publisher = self.publishers.get(f'button_{note_number}')
                if publisher:
                    ros_msg = Bool()
                    ros_msg.data = (msg.type == 'note_on' and velocity > 0)
                    publisher.publish(ros_msg)
        
        elif msg.type == 'system_reset':
            # システムリセット
            publisher = self.publishers.get('system')
            if publisher:
                ros_msg = Int32()
                ros_msg.data = 1
                publisher.publish(ros_msg)
    
    def publish_midi_data(self):
        """MIDIデータをROS2トピックにパブリッシュ"""
        
        # バッファされたMIDIメッセージを処理
        with self.midi_lock:
            messages = self.midi_messages.copy()
            self.midi_messages.clear()
        
        for msg in messages:
            self.process_midi_message(msg)
        
        # 統合Joyメッセージをパブリッシュ
        self.publish_joy_message()
    
    def publish_joy_message(self):
        """統合Joyメッセージをパブリッシュ"""
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        
        # 軸（スライダー・ノブ）
        axes = [0.0] * 16  # 8スライダー + 8ノブ
        
        # ボタン
        buttons = [False] * 32
        
        # 現在の値を取得（簡易実装）
        # 実際の実装では、最後の値を保持する必要があります
        
        joy_msg.axes = axes
        joy_msg.buttons = buttons
        
        self.publishers['joy'].publish(joy_msg)
    
    def cleanup(self):
        """リソースのクリーンアップ"""
        self.running = False
        
        if self.midi_port:
            self.midi_port.close()
            self.get_logger().info('MIDIポートを閉じました')


def main(args=None):
    rclpy.init(args=args)
    
    node = MidiControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 