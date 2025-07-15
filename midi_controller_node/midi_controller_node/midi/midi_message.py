"""
MIDIメッセージ処理クラス

MIDIメッセージの解析、変換、正規化機能を提供します。
"""

import mido
from typing import Dict, Any, Optional, Union
from dataclasses import dataclass


@dataclass
class MidiMessage:
    """MIDIメッセージのデータクラス"""
    
    type: str
    channel: int
    value: Union[int, float, bool]
    timestamp: float
    raw_message: mido.Message
    
    def to_dict(self) -> Dict[str, Any]:
        """辞書形式に変換"""
        return {
            'type': self.type,
            'channel': self.channel,
            'value': self.value,
            'timestamp': self.timestamp
        }


class MidiMessageProcessor:
    """MIDIメッセージ処理クラス"""
    
    @staticmethod
    def normalize_control_change(value: int) -> float:
        """
        Control Change値を0.0-1.0に正規化
        
        Args:
            value: 元の値 (0-127)
            
        Returns:
            正規化された値 (0.0-1.0)
        """
        return max(0.0, min(1.0, value / 127.0))
    
    @staticmethod
    def normalize_pitch_wheel(value: int) -> float:
        """
        ピッチベンド値を0.0-1.0に正規化
        
        Args:
            value: 元の値 (-8192 ～ +8191)
            
        Returns:
            正規化された値 (0.0-1.0)
        """
        return max(0.0, min(1.0, (value + 8192) / 16383.0))
    
    @staticmethod
    def process_message(msg: mido.Message) -> Optional[MidiMessage]:
        """
        MIDIメッセージを処理
        
        Args:
            msg: 生のMIDIメッセージ
            
        Returns:
            処理されたメッセージ、処理できない場合はNone
        """
        import time
        
        timestamp = time.time()
        
        if msg.type == 'control_change':
            normalized_value = MidiMessageProcessor.normalize_control_change(msg.value)
            return MidiMessage(
                type='control_change',
                channel=msg.channel,
                value=normalized_value,
                timestamp=timestamp,
                raw_message=msg
            )
        
        elif msg.type == 'note_on':
            is_pressed = msg.velocity > 0
            return MidiMessage(
                type='note_on',
                channel=msg.channel,
                value=is_pressed,
                timestamp=timestamp,
                raw_message=msg
            )
        
        elif msg.type == 'note_off':
            return MidiMessage(
                type='note_off',
                channel=msg.channel,
                value=False,
                timestamp=timestamp,
                raw_message=msg
            )
        
        elif msg.type == 'pitchwheel':
            normalized_value = MidiMessageProcessor.normalize_pitch_wheel(msg.pitch)
            return MidiMessage(
                type='pitchwheel',
                channel=msg.channel,
                value=normalized_value,
                timestamp=timestamp,
                raw_message=msg
            )
        
        elif msg.type == 'system_reset':
            return MidiMessage(
                type='system_reset',
                channel=0,
                value=1,
                timestamp=timestamp,
                raw_message=msg
            )
        
        return None 