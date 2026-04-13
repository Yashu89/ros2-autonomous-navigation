from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mobile_robot_bringup'),
                'launch',
                'slam_custom.launch.py'
            ])
        )
    )

    detector = Node(
        package='mobile_robot_perception',
        executable='box_detector',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    angle = Node(
        package='mobile_robot_perception',
        executable='angle_estimator',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    localizer = Node(
        package='mobile_robot_perception',
        executable='object_localizer',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    manager = Node(
        package='mobile_robot_perception',
        executable='target_manager',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    navigator = Node(
        package='mobile_robot_navigation',
        executable='navigator',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([

        slam,
        TimerAction(period=10.0, actions=[detector]),
        TimerAction(period=11.0, actions=[angle]),
        TimerAction(period=12.0, actions=[localizer]),
        TimerAction(period=13.0, actions=[manager]),
        TimerAction(period=14.0, actions=[navigator]),
    ])