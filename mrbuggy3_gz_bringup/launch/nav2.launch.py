import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription, SomeSubstitutionsType, Substitution
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('slam', default_value='off',
                          choices=['off', 'sync', 'async'],
                          description='Whether to run a SLAM'),
    DeclareLaunchArgument('localization', default_value='false',
                          choices=['true', 'false'],
                          description='Whether to run localization'),
    DeclareLaunchArgument('nav2', default_value='false',
                          choices=['true', 'false'],
                          description='Run nav2'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('world', default_value='depot',
                          description='GZ World'),
    DeclareLaunchArgument('model', default_value='lidar',
                          choices=['base', 'lidar'],
                          description='MR Buggy3 Model'),
    DeclareLaunchArgument('robot_name', default_value='mrbuggy3',
                          description='Robot name')
]


def generate_launch_description():

    # Directories
    pkg_mrbuggy3_gz_bringup = get_package_share_directory(
        'mrbuggy3_gz_bringup')
    pkg_mrbuggy3_description = get_package_share_directory(
        'mrbuggy3_description')
    pkg_mrbuggy3_nav2 = get_package_share_directory(
        'mrbuggy3_nav2')
    pkg_mrbuggy3_rviz = get_package_share_directory(
        'mrbuggy3_rviz')

    pkg_ros_gz_sim = get_package_share_directory(
        'ros_gz_sim')

    # Paths
    gz_sim_launch = PathJoinSubstitution(
        [pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    mrbuggy3_ros_gz_bridge_launch = PathJoinSubstitution(
        [pkg_mrbuggy3_gz_bringup, 'launch', 'ros_gz_bridge.launch.py'])
    rviz_launch = PathJoinSubstitution(
        [pkg_mrbuggy3_rviz, 'launch', 'view_robot.launch.py'])
    nav_launch = PathJoinSubstitution(
        [pkg_mrbuggy3_nav2, 'launch', 'nav_bringup.launch.py'])
    robot_description_launch = PathJoinSubstitution(
        [pkg_mrbuggy3_description, 'launch', 'robot_description.launch.py'])

    # Parameters
    param_file_cmd = DeclareLaunchArgument(
        'param_file',
        default_value=PathJoinSubstitution(
            [pkg_mrbuggy3_gz_bringup, 'config', 'mrbuggy3_node.yaml']),
        description='MR Buggy3 param file')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution(
            [pkg_mrbuggy3_nav2, 'maps', 'depot.yaml']),
        description='Full path to map yaml file to load')

    # Launch configurations
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    mrbuggy3_node_yaml_file = LaunchConfiguration('param_file')

    # Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[
            ('gz_args', [
                LaunchConfiguration('world'), '.sdf',
                ' -v 4',
                ' --gui-config ', PathJoinSubstitution(
                    [pkg_mrbuggy3_gz_bringup,
                     'gui',
                     LaunchConfiguration('model'),
                     'gui.config'])])
        ]
    )

    # Robot description
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_description_launch]),
        launch_arguments=[('model', LaunchConfiguration('model')),
                          ('use_sim_time', LaunchConfiguration('use_sim_time'))]
    )

    # Spawn MR Buggy3
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', LaunchConfiguration('robot_name'),
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw,
            '-topic', 'robot_description'],
        output='screen')

    # ROS GZ bridge
    mrbuggy3_ros_gz_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([mrbuggy3_ros_gz_bridge_launch]),
        launch_arguments=[('model', LaunchConfiguration('model'))]
    )

    # Rviz2
    rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch]),
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # NAV2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav_launch]),
        launch_arguments=[('slam', LaunchConfiguration('slam')),
                          ('nav2', LaunchConfiguration('nav2')),
                          ('localization', LaunchConfiguration('localization')),
                          ('use_sim_time', LaunchConfiguration('use_sim_time')),
                          ('map', LaunchConfiguration('map'))]
    )

    # RPLIDAR static transforms
    rplidar_stf = Node(
            name='rplidar_stf',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '0', '0', '0', '0', '0.0', '0.0',
                'rplidar_link', [LaunchConfiguration('robot_name'), '/rplidar_link/rplidar']]
        )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(param_file_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(gz_sim)
    ld.add_action(mrbuggy3_ros_gz_bridge)
    ld.add_action(rviz2)
    ld.add_action(robot_description)
    ld.add_action(spawn_robot)
    ld.add_action(nav2)
    ld.add_action(rplidar_stf)
    return ld

