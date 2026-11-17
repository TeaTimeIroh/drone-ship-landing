"""Start everything the pad-landing mission needs except the mission itself.

    ros2 launch ship_landing pad_sim.launch.py                       static pad
    ros2 launch ship_landing pad_sim.launch.py view:=true            also open the camera view
    ros2 launch ship_landing pad_sim.launch.py world:=moving_pad     pad moving back and forth
    ros2 launch ship_landing pad_sim.launch.py world:=moving_pad pattern:=circle pad_speed:=1.0

Runs landing_pad.launch.py (Gazebo, ArduPilot SITL, the ROS 2 bridge) and the marker
detector, with simulation time already switched on for the detector. In the moving_pad
world it also starts the node that drives the pad and the bridge that carries its
commands to Gazebo.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Gazebo topics of the moving_pad model, and the ROS names they get.
PAD_EAST = '/model/moving_pad/joint/slide_east/cmd_vel'
PAD_NORTH = '/model/moving_pad/joint/slide_north/cmd_vel'
PAD_STATE = '/world/moving_pad/model/moving_pad/joint_state'


def generate_launch_description():
    pkg = Path(get_package_share_directory('ship_landing'))
    pad_moves = IfCondition(PythonExpression(["'", LaunchConfiguration('world'), "' == 'moving_pad'"]))

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(pkg / 'launch' / 'landing_pad.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
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

    # "]" sends ROS messages to Gazebo, "[" brings Gazebo messages to ROS.
    pad_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='pad_bridge',
        arguments=[
            PAD_EAST + '@std_msgs/msg/Float64]gz.msgs.Double',
            PAD_NORTH + '@std_msgs/msg/Float64]gz.msgs.Double',
            PAD_STATE + '@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            (PAD_EAST, '/moving_pad/cmd_vel_east'),
            (PAD_NORTH, '/moving_pad/cmd_vel_north'),
            (PAD_STATE, '/moving_pad/joint_states'),
        ],
        output='screen',
        condition=pad_moves,
    )
    pad_mover = Node(
        package='ship_landing',
        executable='pad_mover',
        parameters=[{
            'use_sim_time': True,
            'pattern': LaunchConfiguration('pattern'),
            'speed': ParameterValue(LaunchConfiguration('pad_speed'), value_type=float),
        }],
        output='screen',
        condition=pad_moves,
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='landing_pad',
                              description='World to load: landing_pad or moving_pad.'),
        DeclareLaunchArgument('gui', default_value='true', description='Open the Gazebo window.'),
        DeclareLaunchArgument('rviz', default_value='false', description='Open RViz.'),
        DeclareLaunchArgument('view', default_value='false',
                              description='Open the camera view with the detection drawn on it.'),
        DeclareLaunchArgument('pattern', default_value='line',
                              description='moving_pad world: path of the pad, line or circle.'),
        DeclareLaunchArgument('pad_speed', default_value='0.5',
                              description='moving_pad world: speed of the pad in m/s.'),
        simulator,
        detector,
        camera_view,
        pad_bridge,
        pad_mover,
    ])
