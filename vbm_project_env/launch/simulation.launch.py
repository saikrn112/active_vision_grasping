import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import yaml

def generate_launch_description():
    world_path = os.path.join(
        get_package_share_directory('vbm_project_env'),
        'worlds',
        'object_with_table.world')
    
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'),'launch'),'/gazebo.launch.py']),
                launch_arguments={'world':world_path}.items(),
    )

    simulation_description_path = os.path.join(get_package_share_directory('vbm_project_env'))
    simulation_urdf_path = os.path.join(simulation_description_path,'urdf','camera.urdf')

    spawn_entity = Node(package='gazebo_ros', executable="spawn_entity.py",
                        arguments=['-file',simulation_urdf_path,
                                    '-entity','camera',
                                  
                                    '-P','1.57'],
                        output='both' )
    
    nodes = [
        gazebo,
        spawn_entity
    ]

    return LaunchDescription(nodes)
