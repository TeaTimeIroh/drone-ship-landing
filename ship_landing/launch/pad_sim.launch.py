"""Start everything the pad-landing mission needs except the mission itself.

    ros2 launch ship_landing pad_sim.launch.py
    ros2 launch ship_landing pad_sim.launch.py view:=true     also open the camera view

Runs landing_pad.launch.py (Gazebo, ArduPilot SITL, the ROS 2 bridge) and the marker
detector, with simulation time already switched on for the detector.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = Path(get_package_share_directory('ship_landing'))

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(pkg / 'launch' / 'landing_pad.launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )
    detector = Node(
        package='ship_landing',
        executable='marker_detector',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    camera_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=['/landing_pad/debug_image'],
        condition=IfCondition(LaunchConfiguration('view')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Open the Gazebo window.'),
        DeclareLaunchArgument('rviz', default_value='false', description='Open RViz.'),
        DeclareLaunchArgument('view', default_value='false',
                              description='Open the camera view with the detection drawn on it.'),
        simulator,
        detector,
        camera_view,
    ])
