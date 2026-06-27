#!/usr/bin/env python3
"""
Launch para a fase de NAVEGAÇÃO com mapa já guardado.
Usa Nav2 (AMCL + planners) em vez de SLAM.

Pré-requisito: ter guardado o mapa com:
  ros2 run nav2_map_server map_saver_cli -f /home/nuno/Documents/ArgusAI/map
"""

import os
import launch
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection

WORLD_PATH = '/home/nuno/Documents/ArgusAI/meu_mapa_tiago.wbt'
MAP_YAML = '/home/nuno/Documents/ArgusAI/map.yaml'


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_tiago')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    webots = WebotsLauncher(
        world=WORLD_PATH,
        mode='realtime',
        ros2_supervisor=True
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': '<robot name=""><link name=""/></robot>'}],
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],   # evita "jump back in time" no TF
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    lidar_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['0', '0', '0.5', '0', '0', '0', 'base_link', 'Hokuyo_URG_04LX_UG01'],
    )

    scan_relay = Node(
        package='topic_tools',
        executable='relay',
        name='scan_relay',
        output='screen',
        arguments=['/Tiago_Lite/Hokuyo_URG_04LX_UG01', '/scan'],
    )

    controller_manager_timeout = ['--controller-manager-timeout', '500']
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )

    robot_description_path = os.path.join(package_dir, 'resource', 'tiago_webots.urdf')
    ros2_control_params = '/home/nuno/Documents/ArgusAI/ros2_control.yml'   # odometria calibrada
    mappings = [('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'),
                ('/diffdrive_controller/odom', '/odom')]

    tiago_driver = WebotsController(
        robot_name='Tiago_Lite',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
            ros2_control_params
        ],
        remappings=mappings,
        respawn=True
    )

    # Config de RViz propria para navegacao (caminho, costmaps, particulas, rasto)
    rviz_config = '/home/nuno/Documents/ArgusAI/nav.rviz'
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config=' + rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Copia local do nav2_params (com use_astar: true no planeador global)
    nav2_params = '/home/nuno/Documents/ArgusAI/nav2_params.yaml'
    navigation_nodes = []
    if 'nav2_bringup' in get_packages_with_prefixes():
        navigation_nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
            launch_arguments=[
                ('map', MAP_YAML),
                ('params_file', nav2_params),
                ('use_sim_time', use_sim_time),
            ]
        ))

    waiting_nodes = WaitForControllerConnection(
        target_driver=tiago_driver,
        nodes_to_start=[
            rviz,
            lidar_tf_publisher,
            scan_relay,
            diffdrive_controller_spawner,
            joint_state_broadcaster_spawner,
        ] + navigation_nodes
    )

    return LaunchDescription([
        webots,
        webots._supervisor,
        robot_state_publisher,
        footprint_publisher,
        tiago_driver,
        waiting_nodes,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())]
            )
        )
    ])
