import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robomas_controller'),
        'config',
        'robomas_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='robomas_controller',
            executable='robomas_bridge_node', # CMakeLists.txtで指定した実行ファイル名
            name='robomas_node',
            output='screen',
            parameters=[config]
        )
    ])