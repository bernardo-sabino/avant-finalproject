from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    distance_sensor_yaml = os.path.join(
        os.path.expanduser('~'),
        'ardu_ws/src/mavros_test/launch/distance_sensor.yaml'
    )

    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            output='screen',
            parameters=[
                {'fcu_url': 'udp://127.0.0.1:14550@'},
                distance_sensor_yaml
            ],
        ),
    ])
