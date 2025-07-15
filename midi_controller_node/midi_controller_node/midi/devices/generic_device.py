"""
汎用MIDIデバイスクラス

標準的なMIDIデバイスに対応する汎用クラスです。
"""

from typing import Dict, Any
from ..midi_device import MidiDevice


class GenericMidiDevice(MidiDevice):
    """
    汎用MIDIデバイスクラス
    
    標準的なMIDIデバイスに対応します。
    """
    
    def __init__(self, port_name: str = None, auto_connect: bool = True, config: Dict[str, Any] = None):
        """
        汎用MIDIデバイスを初期化
        
        Args:
            port_name: MIDIポート名
            auto_connect: 自動接続するかどうか
            config: デバイス設定
        """
        super().__init__(port_name, auto_connect)
        
        # デバイス固有の設定
        self.device_name = "Generic MIDI Device"
        self.device_type = "generic"
        
        # デフォルトマッピング設定
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
        
        # カスタム設定で上書き
        if config:
            self.mappings.update(config.get('mappings', {}))
    
    def _is_compatible_port(self, port_name: str) -> bool:
        """
        汎用デバイスは全てのポートと互換性がある
        
        Args:
            port_name: ポート名
            
        Returns:
            常にTrue
        """
        return True
    
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
    
    def set_mapping(self, mapping_type: str, config: Dict[str, Any]):
        """
        マッピング設定を動的に変更
        
        Args:
            mapping_type: マッピングタイプ ('sliders', 'knobs', 'buttons')
            config: 新しい設定
        """
        if mapping_type in self.mappings:
            self.mappings[mapping_type].update(config) 