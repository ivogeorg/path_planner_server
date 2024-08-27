#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

# from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_name = 'path_planner_server'
    pkg_share_name = FindPackageShare(pkg_name)
    config_dir_name = 'config'

    map_file_name = 'turtle_eight_columns.yaml'
    map_file_path = PathJoinSubstitution([pkg_share_name, config_dir_name, map_file_name])

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
    amcl_config_file_path = PathJoinSubstitution([pkg_share_name, config_dir_name, amcl_config_file_name])
    amcl_node = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_config_file_path])


    # planner node
    planner_config_file_name = 'planner_server.yaml'
    nav2_yaml = PathJoinSubstitution([pkg_share_name, config_dir_name, planner_config_file_name])
    planner_node = Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_yaml])

    # controller node
    controller_config_file_name = 'controller.yaml'
    controller_yaml = PathJoinSubstitution([pkg_share_name, config_dir_name, controller_config_file_name])
    controller_node = Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[controller_yaml])

    # manager of recovery behaviors node
    recovery_config_file_name = 'recovery.yaml'
    recovery_yaml = PathJoinSubstitution([pkg_share_name, config_dir_name, recovery_config_file_name])
    recovery_svr_node = Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            parameters=[recovery_yaml],
            output='screen')

    # behavior tree navigator node
    bt_nav_config_file_name = 'bt_navigator.yaml'
    bt_navigator_yaml = PathJoinSubstitution([pkg_share_name, config_dir_name, bt_nav_config_file_name])
    bt_nav_node = Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml])
            
    # lifecycle manager node (note: unique name)
    lifecycle_manager_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server',
                                        'amcl',
                                        'planner_server', 
                                        'controller_server',
                                        'recoveries_server',
                                        'bt_navigator']}]
    )
    
    return LaunchDescription([
        map_server_node,
        amcl_node,
        planner_node,
        controller_node,
        recovery_svr_node,
        bt_nav_node,
        lifecycle_manager_node
    ])
