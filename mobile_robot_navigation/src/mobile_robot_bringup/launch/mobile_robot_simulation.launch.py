#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 获取包路径
    pkg_gazebo = get_package_share_directory('mobile_robot_gazebo')
    pkg_description = get_package_share_directory('mobile_robot_description')
    
    # 启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 声明启动参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # 包含Gazebo启动文件 - 使用office场景
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_gazebo, 'launch', 'office_gazebo.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # 包含Robot State Publisher启动文件
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_description, 'launch', 'robot_state_publisher.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        gazebo_launch,
        robot_state_publisher_launch
    ])