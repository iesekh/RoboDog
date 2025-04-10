from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package',
            executable='mqtt_input_node',
            name='mqtt_input_node',
            output='screen'
        ),
        Node(
            package='your_package',
            executable='crawl_gait_node',
            name='crawl_gait_node',
            output='screen'
        ),
        Node(
            package='your_package',
            executable='lx16a_driver_node',
            name='lx16a_driver_node',
            output='screen'
        )
    ])
