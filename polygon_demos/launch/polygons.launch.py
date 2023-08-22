from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    polygon_node = Node(
        package='polygon_demos',
        executable='demo',
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', str(get_package_share_path('polygon_demos') / 'config/demo_polygons.rviz')],
        output='screen',
    )
    return LaunchDescription([
        polygon_node,
        rviz_node
    ])
