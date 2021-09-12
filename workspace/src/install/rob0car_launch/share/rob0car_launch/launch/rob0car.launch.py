from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rob0car_sensors',
            namespace='rob0car_sensors',
            executable='rob0car_sensors',
            name='rob0car_sensors'
        ),
        Node(
            package='rob0car_actors',
            namespace='rob0car_actors',
            executable='rob0car_actors',
            name='rob0car_actors'
        ),
        Node(
            package='rob0car_control',
            namespace='rob0car_control',
            executable='rob0car_control',
            name='rob0car_control'
        ),
        Node(
            package='joy',
            namespace='joy',
            executable='joy_node',
            name='joy_node'
        )
    ])
