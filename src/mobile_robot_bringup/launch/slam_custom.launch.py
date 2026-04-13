from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    slam_params = os.path.join(
    get_package_share_directory('mobile_robot_bringup'),
            'config',
            'slam_params.yaml'
        )

    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[slam_params],
        output="screen",
    )

    configure = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "/slam_toolbox", "configure"],
        output="screen",
    )

    activate = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "/slam_toolbox", "activate"],
        output="screen",
    )

    delayed_configure = TimerAction(
        period=2.0,   # was 2
        actions=[configure],
    )

    delayed_activate = TimerAction(
        period=5.0,   # was 3
        actions=[activate],
    )

    return LaunchDescription([
        slam,
        delayed_configure,
        delayed_activate,
    ])