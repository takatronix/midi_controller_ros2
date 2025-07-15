#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """MIDIコントローラーノードのlaunchファイル"""
    
    # パラメータ設定
    midi_port_arg = DeclareLaunchArgument(
        'midi_port_name',
        default_value='nanoKONTROL Studio nanoKONTROL',
        description='MIDIポート名'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='100.0',
        description='パブリッシュレート (Hz)'
    )
    
    enable_debug_arg = DeclareLaunchArgument(
        'enable_debug',
        default_value='true',
        description='デバッグモード有効'
    )
    
    # MIDIコントローラーノード
    midi_node = Node(
        package='midi_controller_node',
        executable='midi_controller',
        name='midi_controller_node',
        output='screen',
        parameters=[{
            'midi_port_name': LaunchConfiguration('midi_port_name'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'enable_debug': LaunchConfiguration('enable_debug'),
        }],
        remappings=[
            ('/midi/joy', '/joy'),
        ]
    )
    
    return LaunchDescription([
        midi_port_arg,
        publish_rate_arg,
        enable_debug_arg,
        midi_node,
    ]) 