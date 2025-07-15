"""
KORG nanoKONTROL Studio専用クラス

KORG nanoKONTROL Studioの特性に特化したMIDIデバイスクラスです。
"""

from typing import Dict, Any
from ..midi_device import MidiDevice


class NanoKontrolStudio(MidiDevice):
    """
    KORG nanoKONTROL Studio専用クラス
    
    このデバイスの特性：
    - 8個のスライダー (CC 0-7)
    - 8個のノブ (CC 16-23)
    - 32個のボタン (Note 0-31)
    - ピッチベンド機能
    """
    
    def __init__(self, port_name: str = None, auto_connect: bool = True):
        """
        nanoKONTROL Studioを初期化
        
        Args:
            port_name: MIDIポート名
            auto_connect: 自動接続するかどうか
        """
        super().__init__(port_name, auto_connect)
        
        # デバイス固有の設定
        self.device_name = "KORG nanoKONTROL Studio"
        self.device_type = "nanokontrol_studio"
        
        # マッピング設定
        self.mappings = {
            'sliders': {
                'cc_range': (0, 7),
                'topic_prefix': '/midi/slider',
                'value_range': (0.0, 1.0)
            },
            'knobs': {
                'cc_range': (16, 23),
                'topic_prefix': '/midi/knob',
                'value_range': (0.0, 1.0)
            },
            'buttons': {
                'note_range': (0, 31),
                'topic_prefix': '/midi/button',
                'value_type': 'bool'
            }
        }
    
    def _is_compatible_port(self, port_name: str) -> bool:
        """
        nanoKONTROL Studioと互換性があるポートかどうかを判定
        
        Args:
            port_name: ポート名
            
        Returns:
            互換性がある場合True
        """
        return 'nanoKONTROL' in port_name
    
    def get_device_info(self) -> Dict[str, Any]:
        """
        デバイス情報を取得
        
        Returns:
            デバイス情報の辞書
        """
        return {
            'name': self.device_name,
            'type': self.device_type,
            'port_name': self.port_name,
            'is_connected': self.is_connected,
            'mappings': self.mappings,
            'stats': self.get_stats()
        }
    
    def get_mapping_config(self) -> Dict[str, Any]:
        """
        マッピング設定を取得
        
        Returns:
            マッピング設定の辞書
        """
        return self.mappings.copy() 