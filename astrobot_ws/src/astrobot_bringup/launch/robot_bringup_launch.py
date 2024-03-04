from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    sensors_launch_path = PathJoinSubstitution(
        [FindPackageShare('astrobot_bringup'), 'launch', 'laser_sensor_launch.py']
    )

    joy_launch_path = PathJoinSubstitution(
        [FindPackageShare('astrobot_bringup'), 'launch', 'joy_teleop_launch.py']
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('astrobot_description'), 'launch', 'view_robot_launch.py']
    )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("astrobot_base"), "config", "ekf.yaml"]
    )

    default_robot_launch_path = PathJoinSubstitution(
        [FindPackageShare('astrobot_bringup'), 'launch', 'astrobot_launch.py']
    )
    extra_launch_path = PathJoinSubstitution(
        [FindPackageShare('astrobot_bringup'), 'launch', 'extra_launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='default_robot', 
            default_value='true',
            description='Use default robot'
        ),

        DeclareLaunchArgument(
            name='extra', 
            default_value='false',
            description='Launch extra launch file'
        ),

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
            name='micro_ros_port',
            default_value='8888',
            description='micro-ROS udp/tcp port number'
        ),

        DeclareLaunchArgument(
            name='odom_topic', 
            default_value='/odom',
            description='EKF out odometry topic'
        ),
        
        DeclareLaunchArgument(
            name='joy', 
            default_value='false',
            description='Use Joystick'
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
            PythonLaunchDescriptionSource(default_robot_launch_path),
            condition=UnlessCondition(LaunchConfiguration("default_robot")),
            launch_arguments={
                'base_serial_port': LaunchConfiguration("base_serial_port")
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(extra_launch_path),
            condition=IfCondition(LaunchConfiguration("extra")),
        )
    ])