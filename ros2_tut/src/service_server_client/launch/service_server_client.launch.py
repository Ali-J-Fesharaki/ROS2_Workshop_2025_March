from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction,TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import subprocess

def conditionally_launch_base_nodes(context, *args, **kwargs):
    server_output=LaunchConfiguration('server_output').perform(context)
    client_output=LaunchConfiguration('client_output').perform(context)
    agent_number=LaunchConfiguration('agent_number').perform(context)
    agent_namespace=LaunchConfiguration('agent_namespace').perform(context)
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
                cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', f'"{{x: 5.0, y: 5.0, theta: 0.0, name: {agent_namespace}}}"'],
                shell=True
            )
        ]
    )
        
    # Launch service server node
    spawn_service_server_node=Node(
        package='service_server_client',
        executable='spawn_service_server',
        name='spawn_service_server',
        output=server_output,
    )

    # Launch service client node
    spawn_service_client_node=Node(
        package='service_server_client',
        executable='spawn_service_client',
        name='spawn_service_client',
        output=client_output,
        parameters=[{
            'agent_number': agent_number,
            'agent_name' : agent_namespace
        }]
    )
    nodes_list=[turtlesim_node,spawn_service_client_node, spawn_service_server_node]
    return  [node for node in nodes_list if node is not None]
def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'server_output',
            default_value='log',
            description='Output type for service server node (screen/log)'
        ),
        DeclareLaunchArgument(
            'client_output',
            default_value='screen',
            description='Output type for service client node (screen/log)'
        ),
        DeclareLaunchArgument(
            'agent_number',
            default_value='10',
            description='Number of agents to spawn'
        ),
        DeclareLaunchArgument(
            'agent_namespace',
            default_value='turtle',
            description='Namespace for the spawned turtle'
        ),
        OpaqueFunction(function=conditionally_launch_base_nodes)
        


    ])
