import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode

def generate_launch_description():
    # 設定ファイルのパス
    parameters_file_path = os.path.join(
        get_package_share_directory('iris_lama_ros2'),
        'config',
        'localization.yaml'
    )

    # 3. iris_lama_ros2 (loc2d_ros)
    loc2d_node = Node(
        package='iris_lama_ros2',
        namespace='iris_lama_ros2',
        executable='loc2d_ros',
        name='loc2d_ros',
        output='screen',
        parameters=[parameters_file_path],
        remappings=[
            ('/map', '/map_server/map')
        ]
    )

    return LaunchDescription([
        loc2d_node
    ])