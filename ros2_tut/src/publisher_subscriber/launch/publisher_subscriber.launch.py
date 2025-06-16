from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # Resolve substitutions at runtime
    pub_frequency = LaunchConfiguration('pub_frequency').perform(context)
    velocity_topic = LaunchConfiguration('velocity_topic').perform(context)
    position_topic = LaunchConfiguration('position_topic').perform(context)
    subscriber_output = LaunchConfiguration('subscriber_output').perform(context)
    publisher_output = LaunchConfiguration('publisher_output').perform(context)

    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen'
    )

    position_subscriber_node = Node(
        package='publisher_subscriber',
        executable='position_subscriber',
        name='position_subscriber',
        output=subscriber_output,
        parameters=[{'topic_name': position_topic}]
    )

    velocity_publisher_node = Node(
        package='publisher_subscriber',
        executable='velocity_publisher',
        name='velocity_publisher',
        output=publisher_output,
        parameters=[{
            'topic_name': velocity_topic,
            'frequency': pub_frequency
        }]
    )

    return [
        turtlesim_node,
        position_subscriber_node,
        velocity_publisher_node
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pub_frequency', default_value='10', description='Publishing frequency in Hz'),
        DeclareLaunchArgument('velocity_topic', default_value='cmd_vel', description='Name of the velocity topic'),
        DeclareLaunchArgument('position_topic', default_value='position', description='Name of the position topic'),
        DeclareLaunchArgument('subscriber_output', default_value='log', description='Output type for subscriber node'),
        DeclareLaunchArgument('publisher_output', default_value='screen', description='Output type for publisher node'),
        OpaqueFunction(function=launch_setup)  # Resolved values go here
    ])
