from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    tf_broadcaster = Node(
        package='local_pgm',
        executable='tf_broadcaster',
        name='tf_broadcaster',
        output='screen',
        emulate_tty=True,
    )

    gps_receiver = Node(
        package='local_pgm',
        executable='gps_receiver',
        name='gps_receiver',
        output='screen',
        emulate_tty=True,
    )

    # firebase_publisher = Node(
    #     package='local_pgm',
    #     executable='firebase_publisher',
    #     name='firebase_publisher',
    #     output='screen',
    #     emulate_tty=True,
    # )
    return LaunchDescription([
        tf_broadcaster,
        gps_receiver,
        # firebase_publisher,
    ])
    # return LaunchDescription([
    #     Node(
    #         package='local_pgm',
    #         executable='tf_broadcaster',
    #         output='screen',
    #     ),
    #     Node(
    #         package='local_pgm',
    #         executable='gps_receiver',
    #         output='screen',
    #     ),
