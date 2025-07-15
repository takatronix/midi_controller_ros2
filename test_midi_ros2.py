#!/usr/bin/env python3

"""
ROS2環境でのMIDIコントローラーノードテストスクリプト
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int32
from sensor_msgs.msg import Joy
import time


class MidiTestNode(Node):
    """MIDIノードのテスト用ノード"""
    
    def __init__(self):
        super().__init__('midi_test_node')
        
        # サブスクライバー
        self.slider_subscribers = []
        self.knob_subscribers = []
        self.button_subscribers = []
        
        # スライダー用サブスクライバー
        for i in range(8):
            sub = self.create_subscription(
                Float32,
                f'/midi/slider_{i}',
                lambda msg, idx=i: self.slider_callback(msg, idx),
                10
            )
            self.slider_subscribers.append(sub)
        
        # ノブ用サブスクライバー
        for i in range(8):
            sub = self.create_subscription(
                Float32,
                f'/midi/knob_{i}',
                lambda msg, idx=i: self.knob_callback(msg, idx),
                10
            )
            self.knob_subscribers.append(sub)
        
        # ボタン用サブスクライバー
        for i in range(32):
            sub = self.create_subscription(
                Bool,
                f'/midi/button_{i}',
                lambda msg, idx=i: self.button_callback(msg, idx),
                10
            )
            self.button_subscribers.append(sub)
        
        # Joyメッセージ用サブスクライバー
        self.joy_subscriber = self.create_subscription(
            Joy,
            '/midi/joy',
            self.joy_callback,
            10
        )
        
        # システムメッセージ用サブスクライバー
        self.system_subscriber = self.create_subscription(
            Int32,
            '/midi/system',
            self.system_callback,
            10
        )
        
        self.get_logger().info('MIDIテストノードが起動しました')
        self.get_logger().info('MIDIコントローラーを操作してください')
    
    def slider_callback(self, msg: Float32, index: int):
        """スライダーコールバック"""
        self.get_logger().info(f'スライダー {index}: {msg.data:.3f}')
    
    def knob_callback(self, msg: Float32, index: int):
        """ノブコールバック"""
        self.get_logger().info(f'ノブ {index}: {msg.data:.3f}')
    
    def button_callback(self, msg: Bool, index: int):
        """ボタンコールバック"""
        status = "ON" if msg.data else "OFF"
        self.get_logger().info(f'ボタン {index}: {status}')
    
    def joy_callback(self, msg: Joy):
        """Joyメッセージコールバック"""
        axes_str = ', '.join([f'{x:.3f}' for x in msg.axes])
        buttons_str = ', '.join(['1' if b else '0' for b in msg.buttons])
        self.get_logger().info(f'Joy - 軸: [{axes_str}], ボタン: [{buttons_str}]')
    
    def system_callback(self, msg: Int32):
        """システムメッセージコールバック"""
        self.get_logger().info(f'システムメッセージ: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    
    node = MidiTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 