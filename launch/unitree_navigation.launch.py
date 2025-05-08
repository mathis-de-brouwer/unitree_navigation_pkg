#!/usr/bin/env python3
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='unitree_navigation_pkg',
            executable='navigation_node',
            name='navigation_node'
        ),
        launch_ros.actions.Node(
            package='unitree_navigation_pkg',
            executable='segmentation_node',
            name='segmentation_node'
        ),
        # launch_ros.actions.Node(
        #     package='unitree_navigation_pkg',
        #     executable='websocket_server',
        #     name='websocket_server'
        # ),
    ])
