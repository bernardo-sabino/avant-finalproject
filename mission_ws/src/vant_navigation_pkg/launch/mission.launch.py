from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    vision_node = Node(
        package='vant_vision_pkg',
        executable='detection_node',
        name='drone_vision_node',
        output='screen'
    )
    navigation_node = Node(
        package='vant_navigation_pkg',
        executable='navigation_node',
        name='drone_mission_node',
        output='screen'
    )

    return LaunchDescription([
        vision_node,
        navigation_node,
    ])