# Copyright (c) 2024 Intelligent Robotics Lab (URJC)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    rviz = LaunchConfiguration('rviz')
    # slam_toolbox= LaunchConfiguration('slam_toolbox')
    

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Launch rviz'
    )

    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value='/home/unitree/go2_ws1/map/map.yaml', # Valore di default vuoto. Se vuoi localizzare, devi specificarlo.
        description='Full path to map yaml file to load for localization.'
    )
    map_file = LaunchConfiguration('map_file')

    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(
            get_package_share_directory('go2_navigation'),
            'config',
            'costmap_config.yaml'),
        description='Full path to the Nav2 parameters file to use.'
    )
    nav2_params_file = LaunchConfiguration('nav2_params_file')

    declare_slam_enable_cmd = DeclareLaunchArgument(
        'slam_enable',
        default_value='True',
        description='bool value to choose between mapping and navigation'
    )
    slam_enable = LaunchConfiguration('slam_enable')

    robot_description_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('go2_description'),
            'launch/'), 'robot.launch.py'])
    )

    driver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('go2_driver'),
            'launch/'), 'go2_driver.launch.py'])
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('go2_rviz'),
            'launch/'), 'rviz.launch.py']),
        condition=IfCondition(PythonExpression([rviz]))
    )

    livox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('livox_ros_driver2'),
            'launch_ROS2/'), 'rviz_MID360_launch.py'])
        # condition=IfCondition(PythonExpression([rviz]))
    )

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/livox/lidar'),  # Topic della point cloud in ingresso dal Livox
            ('scan', '/laser')              # Topic del laser scan in uscita
        ],
        parameters=[{
            'target_frame': 'livox_frame',       # Frame del sensore laser simulato (deve essere trasformabile rispetto al base_link)
            'transform_tolerance': 0.01,
            'min_height': 0.15,            # Altezza minima dei punti da considerare
            'max_height': 1.0,             # Altezza massima dei punti da considerare
            'angle_min': -3.14,            # Angolo minimo del laser scan (rad) (-pi)
            'angle_max': 3.14,             # Angolo massimo del laser scan (rad) (pi)
            'angle_increment': 0.0087,     # Incremento angolare del laser scan (rad) (~0.5 gradi)
            'range_min': 0.1,              # Distanza minima dei raggi laser
            'range_max': 100.0,            # Distanza massima dei raggi laser
            'use_intensities': False,      # Se usare le intensità della point cloud
            'concurrency_level': 1,        # Livello di concorrenza per l'elaborazione
            'use_sim_time': LaunchConfiguration('use_sim_time', default='false') # Aggiungi questo se hai un use_sim_time globale
        }],
        output='screen'
    )

    # --- Modulo Map Server (Condizionale) ---
    # Questo nodo carica la mappa salvata
    # map_server_node = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     parameters=[
    #         {'yaml_filename': "/home/unitree/go2_ws1/map/map.yaml"}
    #         # {'use_sim_time': False}
    #     ]
    # )

    nav_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('go2_navigation'),
            'launch/'), 'nav2_navigation_launch.py']),
        launch_arguments={
            'params_file': nav2_params_file, # <--- Qui passiamo il tuo file di configurazione
            'use_sim_time': 'false'    # Passa use_sim_time anche a Nav2
            # 'autostart': 'true'              # Generalmente utile per Nav2
        }.items()
        # condition=IfCondition(PythonExpression([rviz]))
    )

    ld = LaunchDescription()
    # ld.add_action(declare_lidar_cmd)
    # ld.add_action(declare_realsense_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(robot_description_cmd)
    ld.add_action(declare_nav2_params_file_cmd)
    ld.add_action(declare_slam_enable_cmd)
    ld.add_action(declare_map_file_cmd)
    # ld.add_action(lidar_cmd)
    # ld.add_action(realsense_cmd)
    ld.add_action(driver_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(livox_cmd)
    ld.add_action(pointcloud_to_laserscan_node)
    ld.add_action(nav_node)
    
    return ld
