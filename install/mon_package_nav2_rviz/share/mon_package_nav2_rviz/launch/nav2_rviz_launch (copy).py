from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch'
    )

    return LaunchDescription([
        # Lancement de Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_dir, 'bringup_launch.py')
            ),
            launch_arguments={'use_sim_time': 'true', 'autostart': 'true'}.items()
        ),

        # Lancement de RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(nav2_launch_dir, '../rviz/nav2_default_view.rviz')]
        ),

        # Lancement du nœud du capteur IR
        Node(
            package='mon_package_nav2_rviz',
            executable='ir_sensor_node',
            name='ir_sensor_node',
            output='screen'
        )
    ])

