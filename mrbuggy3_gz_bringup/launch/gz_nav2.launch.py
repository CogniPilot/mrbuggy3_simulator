from os import environ
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import ExecuteProcess
from launch.conditions import LaunchConfigurationEquals, IfCondition
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='true',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('sync', default_value='true',
                          choices=['true', 'false'],
                          description='Run async or sync SLAM'),
    DeclareLaunchArgument('localization', default_value='slam',
                          choices=['off', 'localization', 'slam'],
                          description='Whether to run localization or SLAM'),
    DeclareLaunchArgument('nav2', default_value='true',
                          choices=['true', 'false'],
                          description='Run nav2'),
    DeclareLaunchArgument('corti', default_value='true',
                          choices=['true', 'false'],
                          description='Run corti'),
    DeclareLaunchArgument('cerebri', default_value='true',
                          choices=['true', 'false'],
                          description='Run cerebri'),
    DeclareLaunchArgument('world', default_value='depot',
                          description='GZ World'),
    DeclareLaunchArgument('debugger', default_value='false',
                          choices=['true', 'false'],
                          description='Run cerebri with gdb debugger.'),
    DeclareLaunchArgument('uart_shell', default_value='false',
                          choices=['true', 'false'],
                          description='Run cerebri with UART shell.'),
    DeclareLaunchArgument('spawn_model', default_value='true',
                          choices=['true', 'false'],
                          description='Spawn MR Buggy3 Model'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='Use sim time'),
]


def generate_launch_description():


    # Parameters

    declare_x = DeclareLaunchArgument(
        'x',
        default_value=['0'],
        description='x position')

    declare_y = DeclareLaunchArgument(
        'y',
        default_value=['0'],
        description='y position')

    declare_z = DeclareLaunchArgument(
        'z',
        default_value=['0'],
        description='z position')

    declare_yaw = DeclareLaunchArgument(
        'yaw',
        default_value=['0'],
        description='yaw position')

    declare_map_yaml = DeclareLaunchArgument(
        'map_yaml', 
        default_value=[LaunchConfiguration('world'), '.yaml'],
        description='Map yaml'),


    # Launch configurations
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    synapse_ros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [get_package_share_directory('synapse_ros'), 'launch', 'synapse_ros.launch.py'])]),
        launch_arguments=[('host', ['192.0.2.1']),
                          ('port', '4242')]
    )

    synapse_gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [get_package_share_directory('synapse_gz'), 'launch', 'synapse_gz.launch.py'])]),
        launch_arguments=[('host', ['127.0.0.1']),
                          ('port', '4241')],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])]),
        launch_arguments=[('gz_args', [LaunchConfiguration('world'), '.sdf', ' -v 1', ' -r'])]
    )

    cerebri = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [get_package_share_directory('cerebri_bringup'), 'launch', 'cerebri.launch.py'])]),
        launch_arguments=[('debugger', LaunchConfiguration('debugger')),
                          ('vehicle', 'mrbuggy3'),
                          ('uart_shell', LaunchConfiguration('uart_shell'))],
    )

    joy = Node(
        namespace='cerebri/in',
        package='joy',
        executable='joy_node',
        output='screen'
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        #namespace='cerebri',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock' + '@rosgraph_msgs/msg/Clock' + '[gz.msgs.Clock'
        ])

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        #namespace='cerebri',
        name='lidar_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        arguments=[
            ['/world/default/model/mrbuggy3/link/lidar_link/sensor/lidar/scan' +
             '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan']
        ],
        remappings=[
            ('/world/default/model/mrbuggy3/link/lidar_link/sensor/lidar/scan',
             '/scan')
        ])

    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        #namespace='cerebri',
        name='odom_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
        arguments=[
            '/model/mrbuggy3/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'
            ],
        remappings=[
            ('/model/mrbuggy3/odometry', '/odom')
            ])

    odom_base_tf_bridge = Node(
        package='ros_gz_bridge', 
        executable='parameter_bridge',
        #namespace='cerebri',
        name='odom_base_tf_bridge',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
           ['/model/mrbuggy3/pose' +
            '@tf2_msgs/msg/TFMessage' +
            '[gz.msgs.Pose_V']
        ],
        remappings=[
           (['/model/mrbuggy3/pose'], '/tf')
        ])

    pose_bridge = Node(
        package='ros_gz_bridge', 
        executable='parameter_bridge',
        #namespace='cerebri',
        name='pose_bridge',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
           ['/model/mrbuggy3/pose' +
            '@tf2_msgs/msg/TFMessage' +
            '[gz.msgs.Pose_V']
        ],
        remappings=[
           (['/model/mrbuggy3/pose'],
            '/_internal/sim_ground_truth_pose')
        ])


    # Robot description
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
        [get_package_share_directory('mrbuggy3_description'), 'launch', 'robot_description.launch.py'])]),
        launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time'))])

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        arguments=[
            '-world', 'default',
            '-name', 'mrbuggy3',
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw,
            '-file', PathJoinSubstitution([get_package_share_directory(
                'mrbuggy3_gz_resource'),
                'models/mrbuggy3/model.sdf'])
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration("spawn_model")))

    rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
        [get_package_share_directory(
        'mrbuggy3_rviz'), 'launch', 'view_robot.launch.py'])]),
        condition=IfCondition(LaunchConfiguration('rviz')),
        launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time'))])

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
        [get_package_share_directory(
        'mrbuggy3_nav2'), 'launch', 'nav2.launch.py'])]),
        condition=IfCondition(LaunchConfiguration('nav2')),
        launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time'))])

    corti = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
        [get_package_share_directory('corti'), 'launch', 'corti.launch.py'])]),
        condition=IfCondition(LaunchConfiguration('corti')),
        launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time'))])

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
        [get_package_share_directory(
        'mrbuggy3_nav2'), 'launch', 'slam.launch.py'])]),
        condition=LaunchConfigurationEquals('localization', 'slam'),
        launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('sync', LaunchConfiguration('sync'))])

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
        [get_package_share_directory(
        'mrbuggy3_nav2'), 'launch', 'localization.launch.py'])]),
        condition=LaunchConfigurationEquals('localization', 'localization'),
        launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('map', PathJoinSubstitution([get_package_share_directory(
                'mrbuggy3_nav2'), 'maps', LaunchConfiguration('map_yaml')]))])

    tf_to_odom = Node(
        package='corti',
        executable='tf_to_odom',
        output='screen',
        parameters=[{
            'base_frame': 'map',
            'target_frame': 'base_link',
            }],
        remappings=[
            ('/odom', '/cerebri/in/odometry')
            ])

    # Define LaunchDescription variable
    return LaunchDescription(ARGUMENTS + [
        robot_description,
        declare_x,
        declare_y,
        declare_z,
        declare_yaw,
        synapse_ros,
        synapse_gz,
        gz_sim,
        cerebri,
        joy,
        odom_bridge,
        clock_bridge,
        lidar_bridge,
        odom_base_tf_bridge,
        pose_bridge,
        rviz2,
        spawn_robot,
        nav2,
        corti,
        slam,
        localization,
        tf_to_odom
    ])
