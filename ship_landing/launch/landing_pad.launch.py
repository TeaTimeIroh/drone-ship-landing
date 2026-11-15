"""Launch the iris quadcopter in the landing_pad world: Gazebo, ArduPilot SITL and the ROS 2 bridge.

    ros2 launch ship_landing landing_pad.launch.py
    ros2 launch ship_landing landing_pad.launch.py rviz:=true

Reuses the iris launch file from ardupilot_gz_bringup and changes two things:
the world, and the autopilot defaults (adds config/camera_down.parm).
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORLD_NAME = 'landing_pad'


def generate_launch_description():
    pkg = Path(get_package_share_directory('ship_landing'))
    pkg_bringup = Path(get_package_share_directory('ardupilot_gz_bringup'))
    pkg_sitl = Path(get_package_share_directory('ardupilot_sitl'))
    pkg_ardupilot_gazebo = Path(get_package_share_directory('ardupilot_gazebo'))
    pkg_ros_gz_sim = Path(get_package_share_directory('ros_gz_sim'))

    # Let Gazebo find model://aruco_pad in this package.
    resource_path = os.pathsep.join(
        p for p in (str(pkg / 'models'), os.environ.get('GZ_SIM_RESOURCE_PATH', '')) if p)
    set_resource_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path)

    # Autopilot parameter defaults: the upstream iris set, then ours on top.
    sitl_params = pkg_sitl / 'config' / 'default_params'
    defaults = ','.join(str(p) for p in (
        sitl_params / 'copter.parm',
        pkg_ardupilot_gazebo / 'config' / 'gazebo-iris-gimbal.parm',
        sitl_params / 'dds_udp.parm',
        sitl_params / 'dds_use_ns.parm',
        pkg / 'config' / 'camera_down.parm',
    ))

    gz_launch = str(pkg_ros_gz_sim / 'launch' / 'gz_sim.launch.py')
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={
            'gz_args': f'-v3 -s -r {pkg / "worlds" / "landing_pad.sdf"}',
        }.items(),
    )
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': '-v3 -g'}.items(),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(pkg_bringup / 'launch' / 'robots' / 'iris.launch.py')),
        launch_arguments={
            'world_name': WORLD_NAME,
            'defaults': defaults,
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', str(pkg_bringup / 'rviz' / 'iris.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Open the Gazebo window.'),
        DeclareLaunchArgument('rviz', default_value='false', description='Open RViz.'),
        set_resource_path,
        gz_server,
        gz_gui,
        robot,
        rviz,
    ])
