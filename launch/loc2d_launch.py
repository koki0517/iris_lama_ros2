import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode

def generate_launch_description():
    # マップファイルのパス
    map_yaml_file = '/home/koki/ros2_ws/src/abu2026-R2/abu26r2/maps/red_full_5cm.yaml'
    
    # 設定ファイルのパス
    parameters_file_path = os.path.join(
        get_package_share_directory('iris_lama_ros2'),
        'config',
        'localization.yaml'
    )

    # 1. map_server (LifecycleNode)
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_file}]
    )

    # 2. lifecycle_manager (map_serverを構成->活性化まで自動で行う)
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    # 3. iris_lama_ros2 (loc2d_ros)
    loc2d_node = Node(
        package='iris_lama_ros2',
        namespace='iris_lama_ros2',
        executable='loc2d_ros',
        name='loc2d_ros',
        output='screen',
        parameters=[parameters_file_path],
    )

    return LaunchDescription([
        map_server_node,
        lifecycle_manager_node,
        loc2d_node
    ])