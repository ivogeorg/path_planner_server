#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'path_planner_server'
    map_file_name = 'turtle_eight_columns.yaml'
    map_file_path = os.path.join(get_package_share_directory(pkg_name), 'config', map_file_name)

    map_server_node = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename':map_file_path}])

    # amcl_config_file_name = 'amcl_config.yaml'
    # For global localizatoin:
    amcl_config_file_name = 'amcl_config_global.yaml'
    # With initial location in the config file:
    # amcl_config_file_name = 'amcl_config_initialized.yaml'
    amcl_config_file_path = os.path.join(get_package_share_directory(pkg_name), 'config', amcl_config_file_name)
    amcl_node = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_config_file_path])

    lifecycle_manager_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]
    )
    
    return LaunchDescription([
        map_server_node,
        amcl_node,
        lifecycle_manager_node
    ])
