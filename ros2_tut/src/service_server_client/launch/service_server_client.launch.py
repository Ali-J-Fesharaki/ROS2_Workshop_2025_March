from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import subprocess

def conditionally_launch_base_nodes(context, *args, **kwargs):
    actions = []
    service_name=LaunchConfiguration('service_name').perform(context)
    server_output=LaunchConfiguration('server_output').perform(context)
    client_output=LaunchConfiguration('client_output').perform(context)

    node_list_result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True)
    nodes = node_list_result.stdout.splitlines()
    turtlesim_node=None
    if '/turtlesim' not in nodes:
        turtlesim_node=Node(
                package='turtlesim',
                executable='turtlesim_node',
                name='turtlesim',
                output='screen'
            )
        
    # Launch service server node
    spawn_service_server_node=Node(
        package='service_server_client',
        executable='spawn_service_server',
        name='spawn_service_server',
        output=server_output,
        parameters=[{
            'service_name': service_name
        }]
    )

    # Launch service client node
    spawn_service_client_node=Node(
        package='service_server_client',
        executable='spawn_service_client',
        name='spawn_service_client',
        output=client_output,
        parameters=[{
            'service_name':service_name
        }]
    )
    nodes_list=[turtlesim_node,spawn_service_client_node, spawn_service_server_node]
    return  [node for node in nodes_list if node is not None]
def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'service_name',
            default_value='spawn',
            description='Name of the service'
        ),
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
        OpaqueFunction(function=conditionally_launch_base_nodes)
        


    ])
