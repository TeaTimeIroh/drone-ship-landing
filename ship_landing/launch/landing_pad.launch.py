"""Launch the iris quadcopter in one of this package's worlds: Gazebo, ArduPilot SITL and the ROS 2 bridge.

    ros2 launch ship_landing landing_pad.launch.py                      static pad
    ros2 launch ship_landing landing_pad.launch.py world:=moving_pad    pad on a cart
    ros2 launch ship_landing landing_pad.launch.py rviz:=true

`world` is the name of a file in worlds/ without ".sdf"; the world inside the file must
have the same name. Reuses the iris launch file from ardupilot_gz_bringup and changes
two things: the world, and the autopilot defaults (adds config/camera_down.parm).
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


def generate_launch_description():
    pkg = Path(get_package_share_directory('ship_landing'))
    pkg_bringup = Path(get_package_share_directory('ardupilot_gz_bringup'))
    pkg_sitl = Path(get_package_share_directory('ardupilot_sitl'))
    pkg_ardupilot_gazebo = Path(get_package_share_directory('ardupilot_gazebo'))
    pkg_ros_gz_sim = Path(get_package_share_directory('ros_gz_sim'))

    # Let Gazebo find this package's models (model://aruco_pad, model://moving_pad).
    resource_path = os.pathsep.join(
        p for p in (str(pkg / 'models'), os.environ.get('GZ_SIM_RESOURCE_PATH', '')) if p)
    set_resource_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path)

    # Autopilot parameter defaults: the upstream iris set plus our camera file.
    # The order matters. ArduPilot re-reads the defaults later in start-up only if the
    # LAST file in this list had parameters it could not place on the first read. The DDS
    # parameters are in that situation, so a DDS file must stay last, as it is upstream.
    # With camera_down.parm last, DDS_USE_NS was silently left at 0 and every topic
    # moved from /ap/v1/... to /ap/...
    sitl_params = pkg_sitl / 'config' / 'default_params'
    defaults = ','.join(str(p) for p in (
        sitl_params / 'copter.parm',
        pkg_ardupilot_gazebo / 'config' / 'gazebo-iris-gimbal.parm',
        pkg / 'config' / 'camera_down.parm',
        sitl_params / 'dds_udp.parm',
        sitl_params / 'dds_use_ns.parm',
    ))

    gz_launch = str(pkg_ros_gz_sim / 'launch' / 'gz_sim.launch.py')
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={
            'gz_args': [f'-v3 -s -r {pkg / "worlds"}/', LaunchConfiguration('world'), '.sdf'],
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
            'world_name': LaunchConfiguration('world'),
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
        DeclareLaunchArgument('world', default_value='landing_pad',
                              description='World to load: landing_pad or moving_pad.'),
        DeclareLaunchArgument('gui', default_value='true', description='Open the Gazebo window.'),
        DeclareLaunchArgument('rviz', default_value='false', description='Open RViz.'),
        set_resource_path,
        gz_server,
        gz_gui,
        robot,
        rviz,
    ])
