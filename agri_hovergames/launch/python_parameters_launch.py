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
                {'verbose':   0},
                {'NavQplus':  1},
                {'file_path': 'None'},        # path to the video, like '/home/user/FilmTest.mp4' or 'None'
                {'width':     640},
                {'height':    480}
            ]
        ),
        
        Node(
            package='agri_hovergames',
            executable='flightHGd',
            name='custom_flightHGd',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'verbose':    0},
                {'depth_cam':  1}
            ]
        ),        
        
        Node(
            package='agri_hovergames',
            executable='healthHGd',
            name='custom_healthHGd',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'verbose':     1},
                {'NavQplus':    1},
                {'file_path':  '/home/user/FilmTest.mp4'},        # path to the video, like '/home/user/FilmTest.mp4' or 'None'
                {'width':       640},
                {'height':      480},
                {'target':     'npu'},                              # target = {'cpu', 'gpu', 'npu'}
                {'no_threads':  1},
                {'model_path': '/home/user/detection-precision-npu-in-uint8_out-uint8_channel_ptq.tflite'},
                {'mean':        127.5},
                {'stdv':        127.5},
                {'prob_thr':    0.9}
            ]
        ),        
        
        Node(
            package='agri_hovergames',
            executable='broadcastHGd',
            name='custom_broadcastHGd',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'verbose':     0},
                {'server_ip':  '192.168.0.101'},
                {'source_sel':  2}                 # 0 - from videp_publisher_node (videoHGd), 1 - from flight_control_node (flightHGd), 2 - from health_plant_node
            ]
        )
    ])
