from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetRemap
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    laser_filter_config_path = PathJoinSubstitution(
        [FindPackageShare('astrobot_bringup'), 'config', 'box_laser_filter.yaml']
    )

    laser_launch_path = PathJoinSubstitution(
        [FindPackageShare('astrobot_bringup'), 'launch', 'lasers.launch.py']
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(laser_launch_path),
            launch_arguments={
                'sensor': 'ld06',
                'topic_name': '/base/scan/unfiltered',
                'frame_id': 'base_laser',
                'lidar_serial_port': '/dev/ttyUSB1'
            }.items()   
        ),
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[
                laser_filter_config_path
            ],
            remappings=[
                ('/scan', '/base/scan/unfiltered'),
                ('/scan_filtered', '/base/scan')
            ]
        )
    ])
