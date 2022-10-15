import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
import xacro
import yaml

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():

    # Constants for paths to different files and folders
    gazebo_models_path = '/home/.gazebo/models'
    package_name = 'segmentation'
    robot_name_in_model = 'coke_can'
    sdf_model_path = '/home/janie/.gazebo/models/coke_can/model.sdf'
    world_file_path = 'worlds/simulation.world'
     

    # Pose where we want to spawn the robot
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.52'
    spawn_r_val = '0.0'
    spawn_p_val = '0.0'
    spawn_yaw_val = '0.0'

    # Set the path to different files and folders.  
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    world_path = os.path.join(pkg_share, world_file_path)
    gazebo_models_path = os.path.join(pkg_share, gazebo_models_path)
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    sdf_model_path = os.path.join(pkg_share, sdf_model_path)
    
    # Launch configuration variables specific to simulation
    sdf_model = LaunchConfiguration('sdf_model')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')
    
    # Declare the launch arguments  
              
    declare_sdf_model_path_cmd = DeclareLaunchArgument(
      name='sdf_model', 
      default_value=sdf_model_path, 
      description='Absolute path to robot sdf file')
  
  
    declare_use_simulator_cmd = DeclareLaunchArgument(
      name='use_simulator',
      default_value='True',
      description='Whether to start the simulator')
  
    declare_world_cmd = DeclareLaunchArgument(
      name='world',
      default_value=world_path,
      description='Full path to the world model file to load')

    # Start Gazebo server
    use_simulator = LaunchConfiguration('use_simulator')

    start_gazebo_server_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("gazebo_ros"), 'launch', 'gzserver.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world_path}.items())
 
    # Start Gazebo client    
    gazebo_path = f"{pkg_gazebo_ros}/launch/gazebo.launch.py"

    start_gazebo_client_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py")),
      launch_arguments={'world':world_path}.items())


    simulation_description_path = os.path.join(get_package_share_directory('segmentation'))
    simulation_urdf_path = os.path.join(simulation_description_path,'urdf','camera.urdf')
    robot_description_config = open(simulation_urdf_path).read()
    robot_description = {'robot_description' : robot_description_config}
    
    # spawn_entity = Node(package='gazebo_ros', executable="spawn_entity.py",
    #                     arguments=['-file',simulation_urdf_path,
    #                                 '-entity','camera',
                                  
    #                                 '-P','1.57'],
    #                     output='both' )

    spawn_entity = Node(package='gazebo_ros', executable="spawn_entity.py",
                      arguments=['-file', simulation_urdf_path,
                                  '-entity','camera',
                                  '-z','1',
                                  '-P','1.57'],
                      output='both' )
    print("SPAWN ENTITY DONE")
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    robot_controllers = os.path.join(get_package_share_directory("segmentation"),"config","gazebo_controllers.yaml")

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
                {"robot_description": robot_description_config}, robot_controllers],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
        output="screen",
    )

    robot_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "-c", "/controller_manager"],
    )

    delay_robot_position_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_position_controller_spawner],
        )
    )
    nodes = [
        controller_manager,
        declare_use_simulator_cmd,
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        
        node_robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        delay_robot_position_controller_spawner_after_joint_state_broadcaster_spawner
    ]

    return LaunchDescription(nodes)