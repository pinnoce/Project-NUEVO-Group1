from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Which mission module the robot node runs. Default is the canonical
    # robot.main; the segment-test launch files include this file and override
    # it (e.g. robot.main_pre_lapf_test) so the full sensor stack still comes up.
    mission_module = LaunchConfiguration("mission_module")
    return LaunchDescription([
        DeclareLaunchArgument("mission_module", default_value="robot.main"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("rplidar_ros"), "launch", "rplidar_c1.launch.py"]
                )
            ),
            launch_arguments={
                "serial_port": "/dev/rplidar",
                "serial_baudrate": "460800",
                "frame_id": "laser_frame",
                "topic_name": "scan",
                "scan_mode": "Standard",
                "angle_compensate": "true",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("sensors"), "launch", "sensors.launch.py"]
                )
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("vision"), "launch", "vision_debug.launch.py"]
                )
            ),
        ),
        Node(
            package="robot",
            executable="robot",
            name="robot",
            output="screen",
            parameters=[{"mission_module": mission_module}],
        ),
    ])
