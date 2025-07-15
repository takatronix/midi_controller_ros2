"""
MIDIデバイス固有実装モジュール

各MIDIデバイス専用のクラスを提供します。
"""

from .nanokontrol import NanoKontrolStudio
from .generic_device import GenericMidiDevice

__all__ = ['NanoKontrolStudio', 'GenericMidiDevice'] 