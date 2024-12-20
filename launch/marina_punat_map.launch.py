#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction, ExecuteProcess, RegisterEventHandler

def generate_launch_description():

    yaml_file = os.path.join(get_package_share_directory('marinero_navigation'),'config/marina_punat_map','marina_punat.yaml')

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': yaml_file}],
    )

    configure_map_server = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/map_server', 'configure'],
        output='screen'
    )

    activate_map_server = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/map_server', 'activate'],
        output='screen'
    )

    shutdown_map_server = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/map_server', 'shutdown'],
        output='screen'
    )

    delayed_configure = TimerAction(
        period=4.0,
        actions=[configure_map_server]
    )

    delayed_activate = TimerAction(
        period=6.0,
        actions=[activate_map_server]
    )

    on_shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=map_server_node,
            on_exit=[shutdown_map_server]
        )
    )

    return LaunchDescription([
        map_server_node,
        delayed_configure,
        delayed_activate,
        on_shutdown_handler
    ])