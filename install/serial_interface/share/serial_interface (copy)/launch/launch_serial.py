from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        # 1. Serial Interface (UART communication + capteurs)
        Node(
            package='serial_interface',
            executable='serial_interface_node',
            name='serial_interface_node',
            output='screen'
        ),

        # 2. Motion Planner (logique centrale)
        TimerAction(
            period=1.0,
            actions=[Node(
                package='serial_interface',
                executable='motion_planner_node',
                name='motion_planner_node',
                output='screen'
            )]
        ),

        # 3. Ultrasonic Sensor Node (publie les requêtes de distance)
        TimerAction(
            period=2.0,
            actions=[Node(
                package='serial_interface',
                executable='ultrasonic_sensor_node',
                name='ultrasonic_sensor_node',
                output='screen'
            )]
        ),

        # 4. Webcam Node (publie webcam_alert)
        TimerAction(
            period=3.0,
            actions=[Node(
                package='serial_interface',
                executable='webcam_node',
                name='webcam_node',
                output='screen'
            )]
        )
    ])
