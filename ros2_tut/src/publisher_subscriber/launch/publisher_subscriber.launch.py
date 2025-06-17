from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction,TimerAction ,ExecuteProcess   
from launch.substitutions import LaunchConfiguration  
from launch_ros.actions import Node
import subprocess


def launch_setup(context, *args, **kwargs):
    # Resolve substitutions at runtime
    pub_frequency = LaunchConfiguration('pub_frequency').perform(context)or '10'  # Default to 10 Hz if not provided
    namespace = LaunchConfiguration('namespace').perform(context)or ''  # Default to empty namespace if not provided
    subscriber_output = LaunchConfiguration('subscriber_output').perform(context)or 'log'  # Default to 'log' if not provided
    publisher_output = LaunchConfiguration('publisher_output').perform(context)or 'screen'  # Default to 'screen' if not provided
    turtlesim_node = None
    agent_init_killer_node = None
    node_list_result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True)
    nodes = node_list_result.stdout.splitlines()
    if '/turtlesim' not in nodes:
        turtlesim_node=Node(
                package='turtlesim',
                executable='turtlesim_node',
                name='turtlesim',
                output='screen'
            )

        # Wait 1 second, then kill turtle1
        agent_init_killer_node=TimerAction(
            period=0.5,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'service', 'call', '/kill', 'turtlesim/srv/Kill', '"{name: turtle1}"'],
                    shell=True
                )
            ]
        )

        # Wait 2 seconds, then spawn turtle_1
    agent_initializer_node=TimerAction(
        period=0.5,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', f'"{{x: 5.0, y: 5.0, theta: 0.0, name: {namespace}}}"'],
                shell=True
            )
        ]
    )


    position_subscriber_node = Node(
        package='publisher_subscriber',
        executable='position_subscriber',
        name='position_subscriber',
        namespace=namespace,
        output=subscriber_output )

    velocity_publisher_node = Node(
        package='publisher_subscriber',
        executable='velocity_publisher',
        name='velocity_publisher',
        output=publisher_output,
        namespace=namespace,
        parameters=[{
            'frequency': pub_frequency
        }]
    )
    nodes_list = [turtlesim_node, agent_init_killer_node, agent_initializer_node, position_subscriber_node, velocity_publisher_node]
    return [node for node in nodes_list if node is not None]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pub_frequency', default_value='10', description='Publishing frequency in Hz'),
        DeclareLaunchArgument('namespace', default_value='turtle_1', description='Namespace for the nodes'),
        DeclareLaunchArgument('subscriber_output', default_value='log', description='Output type for subscriber node'),
        DeclareLaunchArgument('publisher_output', default_value='screen', description='Output type for publisher node'),
        OpaqueFunction(function=launch_setup)  # Resolved values go here
    ])
