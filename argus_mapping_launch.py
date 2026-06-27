#!/usr/bin/env python3

import os
import launch
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection

WORLD_PATH = '/home/nuno/Documents/ArgusAI/meu_mapa_tiago.wbt'


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
        # use_sim_time obrigatorio: sem isto o TF e carimbado com relogio de sistema
        # (epoch enorme) vs tempo de simulacao do resto -> tf2 deteta "jump back in
        # time", limpa o buffer e o slam_toolbox crasha (ExtrapolationException).
        parameters=[{'use_sim_time': True}],
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    # FIX: liga o frame do lidar ao base_link para o SLAM funcionar.
    # O driver Webots publica /Tiago_Lite/Hokuyo_URG_04LX_UG01 mas o URDF
    # espera um dispositivo chamado "hokuyo" — como os nomes não coincidem,
    # o TF do lidar não é publicado automaticamente.
    lidar_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],   # ver nota no footprint_publisher
        arguments=['0', '0', '0.5', '0', '0', '0', 'base_link', 'Hokuyo_URG_04LX_UG01'],
    )

    # FIX: relay garante que /scan existe antes do SLAM arrancar.
    # Incluído no launch para evitar race condition com terminal manual.
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
    # Copia local do ros2_control (editavel) para calibrar a odometria das rodas
    ros2_control_params = '/home/nuno/Documents/ArgusAI/ros2_control.yml'
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

    rviz_config = os.path.join(package_dir, 'resource', 'default.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config=' + rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Copia local dos parametros do SLAM (editavel sem mexer no ficheiro do sistema)
    toolbox_params = '/home/nuno/Documents/ArgusAI/slam_toolbox_params.yaml'
    slam_toolbox = Node(
        parameters=[toolbox_params, {'use_sim_time': use_sim_time}],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
    )

    waiting_nodes = WaitForControllerConnection(
        target_driver=tiago_driver,
        nodes_to_start=[
            rviz,
            slam_toolbox,
            lidar_tf_publisher,
            scan_relay,
            diffdrive_controller_spawner,
            joint_state_broadcaster_spawner,
        ]
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
