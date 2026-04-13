from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_description=Command([
        'cat ',
        PathJoinSubstitution([
            FindPackageShare('mobile_robot_description'),
            'urdf',
            'mobile_robot.urdf'
        ])
    ])

    robot_state_publisher=Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
        output='screen'
    )

    Spawn_robot=Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file',
            PathJoinSubstitution([
                FindPackageShare('mobile_robot_description'),
                'sdf',
                'mobile_robot.sdf'
            ])
        ],
        output='screen'
    )

    clock_bridge=Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    vel_bridge=Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    odom_bridge=Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'],
        remappings=[('/tf', '/tf_gz_raw')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    joint_state_bridge=Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    camera_bridge=Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        parameters=[{'use_sim_time': True}],
        remappings=[('/scan', '/scan')],
        output='screen'
    )

    odom_tf_node = Node(
        package='mobile_robot_bringup',
        executable='odom_tf_broadcaster',
        parameters=[{'use_sim_time': True}],
        output = 'screen'
    )

    map_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom_clean"],
        parameters=[{'use_sim_time': True}]
    )

    lidar_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0.18", "0", "0", "0", "base_link", "lidar_link"],
        parameters=[{'use_sim_time': True}]
    )

    camera_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.15", "0", "0.15", "0", "0", "0", "base_link", "camera_link"],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        
        robot_state_publisher,
        Spawn_robot,
        clock_bridge,
        vel_bridge,
        odom_bridge,
        joint_state_bridge,
        camera_bridge,   
        #map_odom,
        lidar_bridge,
        odom_tf_node,
        tf_bridge,
        lidar_static_tf,
        camera_static_tf,
    ])