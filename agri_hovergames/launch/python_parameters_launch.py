from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agri_hovergames',
            executable='videoHGd',
            name='custom_videoHGd',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'verbose':  1},
                {'NavQplus': 1},
                {'width':    640},
                {'height':   480}
            ]
        )
    ])
