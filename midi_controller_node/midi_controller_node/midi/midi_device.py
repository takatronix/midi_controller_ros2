"""
基本MIDIデバイスクラス

MIDIデバイスの接続、メッセージ受信、基本的な処理機能を提供します。
"""

import mido
import threading
import time
from typing import Optional, Callable, List, Dict, Any
from abc import ABC, abstractmethod


class MidiDevice(ABC):
    """
    基本MIDIデバイスクラス
    
    全てのMIDIデバイスの基底クラスです。
    """
    
    def __init__(self, port_name: str = None, auto_connect: bool = True):
        """
        MIDIデバイスを初期化
        
        Args:
            port_name: MIDIポート名（Noneの場合は自動検出）
            auto_connect: 自動接続するかどうか
        """
        self.port_name = port_name
        self.midi_port: Optional[mido.ports.BaseInput] = None
        self.is_connected = False
        self.is_running = False
        
        # メッセージ処理
        self.message_callbacks: List[Callable] = []
        self.message_buffer: List[mido.Message] = []
        self.buffer_lock = threading.Lock()
        
        # 統計情報
        self.stats = {
            'messages_received': 0,
            'connection_errors': 0,
            'last_message_time': None
        }
        
        # 自動接続
        if auto_connect:
            self.connect()
    
    def connect(self) -> bool:
        """
        MIDIポートに接続
        
        Returns:
            接続成功時True
        """
        try:
            # ポート名が指定されていない場合は自動検出
            if self.port_name is None:
                self.port_name = self._auto_detect_port()
            
            if self.port_name is None:
                raise RuntimeError("利用可能なMIDIポートが見つかりません")
            
            # ポートを開く
            self.midi_port = mido.open_input(self.port_name)
            self.is_connected = True
            
            # 受信スレッド開始
            self._start_receiver_thread()
            
            return True
            
        except Exception as e:
            self.stats['connection_errors'] += 1
            raise RuntimeError(f"MIDIポート接続エラー: {e}")
    
    def disconnect(self):
        """MIDIポートを切断"""
        self.is_running = False
        self.is_connected = False
        
        if self.midi_port:
            self.midi_port.close()
            self.midi_port = None
    
    def add_message_callback(self, callback: Callable[[mido.Message], None]):
        """
        メッセージコールバックを追加
        
        Args:
            callback: メッセージ受信時に呼ばれる関数
        """
        self.message_callbacks.append(callback)
    
    def get_messages(self) -> List[mido.Message]:
        """
        バッファされたメッセージを取得
        
        Returns:
            メッセージのリスト
        """
        with self.buffer_lock:
            messages = self.message_buffer.copy()
            self.message_buffer.clear()
        return messages
    
    def get_stats(self) -> Dict[str, Any]:
        """
        統計情報を取得
        
        Returns:
            統計情報の辞書
        """
        return self.stats.copy()
    
    def _auto_detect_port(self) -> Optional[str]:
        """
        自動的にMIDIポートを検出
        
        Returns:
            検出されたポート名、見つからない場合はNone
        """
        input_ports = mido.get_input_names()
        
        # デバイス固有の検出ロジック
        for port in input_ports:
            if self._is_compatible_port(port):
                return port
        
        # 互換性のあるポートが見つからない場合は最初のポートを使用
        return input_ports[0] if input_ports else None
    
    def _start_receiver_thread(self):
        """メッセージ受信スレッドを開始"""
        self.is_running = True
        self.receiver_thread = threading.Thread(target=self._receiver_loop)
        self.receiver_thread.daemon = True
        self.receiver_thread.start()
    
    def _receiver_loop(self):
        """メッセージ受信ループ"""
        while self.is_running and self.is_connected:
            try:
                if self.midi_port:
                    msg = self.midi_port.poll()
                    if msg:
                        self._process_message(msg)
                time.sleep(0.001)  # 1ms待機
            except Exception as e:
                # エラーログは後で実装
                pass
    
    def _process_message(self, msg: mido.Message):
        """
        受信したメッセージを処理
        
        Args:
            msg: MIDIメッセージ
        """
        # 統計情報更新
        self.stats['messages_received'] += 1
        self.stats['last_message_time'] = time.time()
        
        # バッファに追加
        with self.buffer_lock:
            self.message_buffer.append(msg)
        
        # コールバック実行
        for callback in self.message_callbacks:
            try:
                callback(msg)
            except Exception as e:
                # コールバックエラーは後でログ出力
                pass
    
    @abstractmethod
    def _is_compatible_port(self, port_name: str) -> bool:
        """
        ポートがこのデバイスと互換性があるかを判定
        
        Args:
            port_name: ポート名
            
        Returns:
            互換性がある場合True
        """
        pass
    
    def __enter__(self):
        """コンテキストマネージャー開始"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """コンテキストマネージャー終了"""
        self.disconnect() 