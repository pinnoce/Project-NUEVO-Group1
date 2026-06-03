from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Segment 2 test: the LAPF leg only — BTN_1 jumps to LAPF_RUN and stops at
    the LAPF goal. Brings up the full stack (lidar + sensors + vision) via
    everything.launch.py and runs robot.main_lapf_only_test."""
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("robot"), "launch", "everything.launch.py"]
                )
            ),
            launch_arguments={"mission_module": "robot.main_lapf_only_test"}.items(),
        ),
    ])
