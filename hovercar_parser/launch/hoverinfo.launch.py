from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hovercar_parser',
            executable='hoverinfo',
            parameters=[
                {'log_path': "log.yml"},
                {'hover_port': "/dev/ttyUSB0"},
                {'battery_capacity': 4100.0}
            ],
            output='screen',
            emulate_tty=True
        )
    ])
