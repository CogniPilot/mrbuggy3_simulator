from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('robot_name', default_value='mrbuggy3',
                          description='GZ model name'),
    DeclareLaunchArgument('world', default_value='depot',
                          description='World name')
]


def generate_launch_description():
    namespace = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # clock bridge
    clock_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                        namespace=namespace,
                        name='clock_bridge',
                        output='screen',
                        arguments=[
                            '/clock' + '@rosgraph_msgs/msg/Clock' + '[gz.msgs.Clock'
                        ],
                        condition=IfCondition(use_sim_time))

    # cmd_vel bridge
    cmd_vel_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                          name='cmd_vel_bridge',
                          output='screen',
                          parameters=[{
                              'use_sim_time': use_sim_time
                          }],
                          arguments=[
                              '/cmd_vel' + '@geometry_msgs/msg/Twist' + ']gz.msgs.Twist',
                          ])

    # Pose bridge
    pose_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                       namespace=namespace,
                       name='pose_bridge',
                       output='screen',
                       parameters=[{
                            'use_sim_time': use_sim_time
                       }],
                       arguments=[
                           ['/model/', LaunchConfiguration('robot_name'), '/pose' +
                            '@tf2_msgs/msg/TFMessage' +
                            '[gz.msgs.Pose_V']
                       ],
                       remappings=[
                           (['/model/', LaunchConfiguration('robot_name'), '/pose'],
                            '/_internal/sim_ground_truth_pose')
                       ])

    # Joy bridge
    joy_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                       namespace=namespace,
                       name='joy_bridge',
                       output='screen',
                       parameters=[{
                            'use_sim_time': use_sim_time
                       }],
                       arguments=[
                           '/joy@sensor_msgs/msg/Joy@gz.msgs.Joy',
                       ])


    # odom to base_link transform bridge
    odom_base_tf_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                               namespace=namespace,
                               name='odom_base_tf_bridge',
                               output='screen',
                               parameters=[{
                                   'use_sim_time': use_sim_time
                               }],
                               arguments=[
                                   ['/model/', LaunchConfiguration('robot_name'), '/pose' +
                                    '@tf2_msgs/msg/TFMessage' +
                                    '[gz.msgs.Pose_V']
                               ],
                               remappings=[
                                   (['/model/', LaunchConfiguration('robot_name'), '/pose'], '/tf')
                               ])

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(clock_bridge)
    ld.add_action(cmd_vel_bridge)
    ld.add_action(pose_bridge)
    ld.add_action(joy_bridge)
    ld.add_action(odom_base_tf_bridge)
    return ld
