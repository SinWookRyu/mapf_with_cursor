#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 패키지 디렉토리 가져오기
    pkg_share = get_package_share_directory('mapf_simulator')
    
    # RViz 설정 파일 경로 (새로운 설정 파일 사용)
    rviz_config_file = os.path.join(pkg_share, 'config', 'mapf_simulator_fixed.rviz')
    
    # Launch 인수 선언
    num_agents_arg = DeclareLaunchArgument(
        'num_agents',
        default_value='3',
        description='Number of agents in the simulation'
    )
    
    world_width_arg = DeclareLaunchArgument(
        'world_width',
        default_value='20.0',
        description='Width of the simulation world'
    )
    
    world_height_arg = DeclareLaunchArgument(
        'world_height',
        default_value='20.0',
        description='Height of the simulation world'
    )
    
    collision_distance_arg = DeclareLaunchArgument(
        'collision_distance',
        default_value='1.5',
        description='Distance for collision detection between agents'
    )
    
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='10.0',
        description='Simulation update rate in Hz'
    )
    
    # MAPF 시뮬레이터 노드 (simple_mapf_node 사용)
    mapf_simulator_node = Node(
        package='mapf_simulator',
        executable='simple_mapf_node',
        name='simple_mapf_node',
        output='screen',
        parameters=[{
            'num_agents': LaunchConfiguration('num_agents'),
            'collision_distance': LaunchConfiguration('collision_distance'),
            'update_rate': LaunchConfiguration('update_rate'),
        }]
    )
    
    # GUI 컨트롤 노드
    gui_control_node = Node(
        package='mapf_simulator',
        executable='gui_control_node',
        name='gui_control',
        output='screen'
    )
    
    # RViz 노드 (새로운 설정 파일 사용)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        env={
            'DISPLAY': ':0',
            'LIBGL_ALWAYS_INDIRECT': '1'
        }
    )
    
    # Static transform publisher for world to map frame
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        output='screen'
    )
    
    return LaunchDescription([
        num_agents_arg,
        world_width_arg,
        world_height_arg,
        collision_distance_arg,
        update_rate_arg,
        mapf_simulator_node,
        gui_control_node,
        rviz_node,
        static_transform_publisher,
    ])
