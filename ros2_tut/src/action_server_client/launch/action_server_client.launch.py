from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import subprocess
from ament_index_python.packages import get_package_share_directory

def conditionally_launch_base_nodes(context, *args, **kwargs):
    agent_name=LaunchConfiguration('agent_name').perform(context)
    goal_x=LaunchConfiguration('goal_x').perform(context)
    goal_y=LaunchConfiguration('goal_y').perform(context)
    server_output=LaunchConfiguration('server_output').perform(context)
    client_output=LaunchConfiguration('client_output').perform(context)
    agent_number=LaunchConfiguration('agent_number').perform(context)

    # Check if turtlesim is already running
    node_list_result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True)
    nodes = node_list_result.stdout.splitlines()
    service_server_client_launch = None
    nav_action_server_node = None
    # Check if service_server_client nodes are already running
    if not any('/spawn_service_server' in n or '/spawn_service_client' in n for n in nodes):
        # Add the include only if needed
        launch_file_path = os.path.join(
            get_package_share_directory('service_server_client'),
            'launch',
            'service_server_client.launch.py'
        )

        service_server_client_launch=IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file_path),
                launch_arguments={
                    'server_output': 'log',
                    'client_output': 'log',
                    'agent_number' : agent_number
                }.items()
            )
    if not any('/nav_action_server' in n for n in nodes):
        # Action Server Node
        nav_action_server_node=Node(
            package='action_server_client',
            executable='nav_action_server',
            name='nav_action_server',
            output=server_output,
        )

    # Action Client Node
    nav_action_client_node=Node(
        package='action_server_client',
        executable='nav_action_client',
        name=agent_name,
        output=client_output,
        namespace=agent_name,
        parameters=[{
            'agent_name': agent_name,
            'goal_x': goal_x,
            'goal_y': goal_y,
        }]
    )
    nodes_list = [ service_server_client_launch, nav_action_server_node, nav_action_client_node]
    return [node for node in nodes_list if node is not None]

# Needed for get_package_share_directory
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('server_output', default_value='screen'),
        DeclareLaunchArgument('client_output', default_value='log'),
        DeclareLaunchArgument('agent_name', default_value='turtle_1'),
        DeclareLaunchArgument('goal_x', default_value='8.0'),
        DeclareLaunchArgument('goal_y', default_value='8.0'),
        DeclareLaunchArgument('agent_number', default_value='10'),

        # Conditionally launch turtlesim and service_server_client.launch.py
        OpaqueFunction(function=conditionally_launch_base_nodes),


    ])
