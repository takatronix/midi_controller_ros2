"""
MIDIマッピングクラス

MIDIメッセージをROS2トピックにマッピングする機能を提供します。
"""

from typing import Dict, Any, Optional, List, Callable
from .midi_message import MidiMessage, MidiMessageProcessor


class MidiMapper:
    """MIDIメッセージマッピングクラス"""
    
    def __init__(self, device_config: Dict[str, Any]):
        """
        MIDIマッパーを初期化
        
        Args:
            device_config: デバイス設定
        """
        self.device_config = device_config
        self.mappings = device_config.get('mappings', {})
        self.topic_callbacks: Dict[str, List[Callable]] = {}
    
    def add_topic_callback(self, topic_name: str, callback: Callable):
        """
        トピックコールバックを追加
        
        Args:
            topic_name: トピック名
            callback: コールバック関数
        """
        if topic_name not in self.topic_callbacks:
            self.topic_callbacks[topic_name] = []
        self.topic_callbacks[topic_name].append(callback)
    
    def process_message(self, msg: MidiMessage) -> Optional[Dict[str, Any]]:
        """
        MIDIメッセージを処理してマッピング
        
        Args:
            msg: 処理されたMIDIメッセージ
            
        Returns:
            マッピング結果、マッピングできない場合はNone
        """
        if msg.type == 'control_change':
            return self._process_control_change(msg)
        elif msg.type in ['note_on', 'note_off']:
            return self._process_note(msg)
        elif msg.type == 'pitchwheel':
            return self._process_pitch_wheel(msg)
        elif msg.type == 'system_reset':
            return self._process_system_reset(msg)
        
        return None
    
    def _process_control_change(self, msg: MidiMessage) -> Optional[Dict[str, Any]]:
        """Control Changeメッセージを処理"""
        # スライダー処理
        if 'sliders' in self.mappings:
            slider_config = self.mappings['sliders']
            cc_range = slider_config.get('cc_range', [0, 7])
            
            if cc_range[0] <= msg.raw_message.control <= cc_range[1]:
                slider_index = msg.raw_message.control - cc_range[0]
                topic_name = f"{slider_config['topic_prefix']}_{slider_index}"
                
                return {
                    'topic_name': topic_name,
                    'message_type': 'Float32',
                    'value': msg.value,
                    'source': 'slider',
                    'index': slider_index
                }
        
        # ノブ処理
        if 'knobs' in self.mappings:
            knob_config = self.mappings['knobs']
            cc_range = knob_config.get('cc_range', [16, 23])
            
            if cc_range[0] <= msg.raw_message.control <= cc_range[1]:
                knob_index = msg.raw_message.control - cc_range[0]
                topic_name = f"{knob_config['topic_prefix']}_{knob_index}"
                
                return {
                    'topic_name': topic_name,
                    'message_type': 'Float32',
                    'value': msg.value,
                    'source': 'knob',
                    'index': knob_index
                }
        
        return None
    
    def _process_note(self, msg: MidiMessage) -> Optional[Dict[str, Any]]:
        """Noteメッセージを処理"""
        if 'buttons' in self.mappings:
            button_config = self.mappings['buttons']
            note_range = button_config.get('note_range', [0, 31])
            
            if note_range[0] <= msg.raw_message.note <= note_range[1]:
                button_index = msg.raw_message.note - note_range[0]
                topic_name = f"{button_config['topic_prefix']}_{button_index}"
                
                return {
                    'topic_name': topic_name,
                    'message_type': 'Bool',
                    'value': msg.value,
                    'source': 'button',
                    'index': button_index
                }
        
        return None
    
    def _process_pitch_wheel(self, msg: MidiMessage) -> Optional[Dict[str, Any]]:
        """ピッチベンドメッセージを処理"""
        if 'pitch_wheel' in self.mappings:
            pitch_config = self.mappings['pitch_wheel']
            topic_name = f"{pitch_config['topic_prefix']}_{msg.channel}"
            
            return {
                'topic_name': topic_name,
                'message_type': 'Float32',
                'value': msg.value,
                'source': 'pitch_wheel',
                'channel': msg.channel
            }
        
        return None
    
    def _process_system_reset(self, msg: MidiMessage) -> Optional[Dict[str, Any]]:
        """システムリセットメッセージを処理"""
        if 'system' in self.mappings:
            system_config = self.mappings['system']
            topic_name = system_config.get('topic_name', '/midi/system')
            
            return {
                'topic_name': topic_name,
                'message_type': 'Int32',
                'value': msg.value,
                'source': 'system'
            }
        
        return None
    
    def get_all_topics(self) -> List[str]:
        """
        全てのトピック名を取得
        
        Returns:
            トピック名のリスト
        """
        topics = []
        
        # スライダートピック
        if 'sliders' in self.mappings:
            slider_config = self.mappings['sliders']
            cc_range = slider_config.get('cc_range', [0, 7])
            for i in range(cc_range[0], cc_range[1] + 1):
                topics.append(f"{slider_config['topic_prefix']}_{i}")
        
        # ノブトピック
        if 'knobs' in self.mappings:
            knob_config = self.mappings['knobs']
            cc_range = knob_config.get('cc_range', [16, 23])
            for i in range(cc_range[0], cc_range[1] + 1):
                topics.append(f"{knob_config['topic_prefix']}_{i}")
        
        # ボタントピック
        if 'buttons' in self.mappings:
            button_config = self.mappings['buttons']
            note_range = button_config.get('note_range', [0, 31])
            for i in range(note_range[0], note_range[1] + 1):
                topics.append(f"{button_config['topic_prefix']}_{i}")
        
        # システムトピック
        if 'system' in self.mappings:
            system_config = self.mappings['system']
            topics.append(system_config.get('topic_name', '/midi/system'))
        
        return topics 