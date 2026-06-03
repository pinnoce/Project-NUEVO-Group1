from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Segment 3 test: from the LAPF goal to the end — BTN_1 jumps to
    MOV3_ALIGN_PERP and runs through drop-off / stop sign to DONE. Brings up the
    full stack (lidar + sensors + vision) via everything.launch.py and runs
    robot.main_post_lapf_test."""
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("robot"), "launch", "everything.launch.py"]
                )
            ),
            launch_arguments={"mission_module": "robot.main_post_lapf_test"}.items(),
        ),
    ])
