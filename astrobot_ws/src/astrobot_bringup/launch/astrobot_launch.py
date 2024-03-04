from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals


def generate_launch_description():
    sensors_launch_path = PathJoinSubstitution(
        [FindPackageShare('astrobot_bringup'), 'launch', 'laser_sensor_launch.py']
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('astrobot_description'), 'launch', 'view_robot_launch.py']
    )
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("astrobot_base"), "config", "ekf.yaml"]
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            name='base_serial_port', 
            default_value='/dev/ttyACM0',
            description='Astrobot Base Serial Port'
        ),

        DeclareLaunchArgument(
            name='micro_ros_transport',
            default_value='serial',
            description='micro-ROS transport'
        ),
        DeclareLaunchArgument(
            name='odom_topic', 
            default_value='/odom',
            description='EKF out odometry topic'
        ),
        Node(
            condition=LaunchConfigurationEquals('micro_ros_transport', 'serial'),
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', LaunchConfiguration("base_serial_port")]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config_path
            ],
            remappings=[("odometry/filtered", LaunchConfiguration("odom_topic"))]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensors_launch_path),
        )
    ])
