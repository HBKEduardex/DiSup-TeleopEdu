#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Movimiento 4 motores + stream + UART cmds
        Node(
            package='robot_canbus',
            executable='can_ps4_bridge',
            name='can_ps4_bridge',
            output='screen'
        ),

        # Estado (rpm, vbat) desde CAN
        Node(
            package='robot_status',
            executable='status_node',
            name='robot_status',
            output='screen'
        ),

        # Video TX (GStreamer)
        Node(
            package='robot_video_tx',
            executable='video_tx',
            name='video_streamer',
            output='screen'
        ),

        # UART ↔ ESP32 (S/H + pH)
        Node(
            package='robot_actuators',
            executable='uart_bridge',
            name='uart_bridge',
            output='screen',
            parameters=[
                {'port': '/dev/ttyUSB0'},
                {'baudrate': 115200},
            ]
        ),
    ])
