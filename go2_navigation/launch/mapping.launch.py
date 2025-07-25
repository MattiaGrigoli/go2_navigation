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

    # for mapping
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/livox/lidar'),  
            ('scan', '/laser')              
        ],
        parameters=[{
            'target_frame': 'livox_frame',       
            'transform_tolerance': 0.01,
            'min_height': 0.10,            
            'max_height': 1.35,             
            'angle_min': -3.14,            
            'angle_max': 3.14,             
            'angle_increment': 0.0087,     
            'range_min': 0.1,              
            'range_max': 100.0,            
            'use_intensities': False,      
            'concurrency_level': 2,        
            'use_sim_time': LaunchConfiguration('use_sim_time', default='false') 
        }],
        output='screen'
    )

    slam_toolbox_config_path = os.path.join(
        get_package_share_directory('go2_navigation'),
        'config',
        'slam_config.yaml' 
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node', 
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_toolbox_config_path,
            {'scan_topic': '/laser'}, 
            {'use_sim_time': 'false'}, 
            {'map_frame': 'map'},       
            {'odom_frame': 'odom'},
            {'base_frame': 'base_link'},
        ],
        # condition=IfCondition(PythonExpression([slam_toolbox]))
        condition=IfCondition(PythonExpression([slam_enable]))
    )

    nav_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'),
            'launch/'), 'navigation_launch.py']),
        launch_arguments={
            'params_file': nav2_params_file, 
            'use_sim_time': 'false'    
            # 'autostart': 'true'              
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
    # ld.add_action(lidar_cmd)
    # ld.add_action(realsense_cmd)
    ld.add_action(driver_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(livox_cmd)
    ld.add_action(pointcloud_to_laserscan_node)
    ld.add_action(slam_toolbox_node)
    ld.add_action(nav_node)
    
    return ld
