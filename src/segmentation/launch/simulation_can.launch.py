# Author: Addison Sears-Collins
# Date: September 27, 2021
# Description: Load an SDF and world file into Gazebo.
# https://automaticaddison.com
 
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import xacro
import yaml
 
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
  
  ############ You do not need to change anything below this line #############
   
  # Set the path to different files and folders.  
  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
  pkg_share = FindPackageShare(package=package_name).find(package_name)
  world_path = os.path.join(pkg_share, world_file_path)
  gazebo_models_path = os.path.join(pkg_share, gazebo_models_path)
  os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
  sdf_model_path = os.path.join(pkg_share, sdf_model_path)
   
  # Launch configuration variables specific to simulation
  gui = LaunchConfiguration('gui')
  headless = LaunchConfiguration('headless')
  namespace = LaunchConfiguration('namespace')
  sdf_model = LaunchConfiguration('sdf_model')
  use_namespace = LaunchConfiguration('use_namespace')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_simulator = LaunchConfiguration('use_simulator')
  world = LaunchConfiguration('world')
   
  # Declare the launch arguments  
  declare_namespace_cmd = DeclareLaunchArgument(
    name='namespace',
    default_value='',
    description='Top-level namespace')
 
  declare_use_namespace_cmd = DeclareLaunchArgument(
    name='use_namespace',
    default_value='false',
    description='Whether to apply a namespace to the navigation stack')
             
  declare_sdf_model_path_cmd = DeclareLaunchArgument(
    name='sdf_model', 
    default_value=sdf_model_path, 
    description='Absolute path to robot sdf file')
 
  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient')
     
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='true',
    description='Use simulation (Gazebo) clock if true')
 
  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start the simulator')
 
  declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')
   
  # Start Gazebo client
  print(pkg_gazebo_ros)
  gazebo_path = f"{pkg_gazebo_ros}/launch/gazebo.launch.py"
  print(gazebo_path)
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(gazebo_path),
    launch_arguments={'world':world_path}.items())

  # Launch the robot
  spawn_entity_cmd = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-entity', robot_name_in_model, 
               '-file', sdf_model,
                  '-x', spawn_x_val,
                  '-y', spawn_y_val,
                  '-z', spawn_z_val,
                  '-R', spawn_r_val,
                  '-P', spawn_p_val,
                  '-Y', spawn_yaw_val,
                  
		],
                  output='screen')

  simulation_description_path = os.path.join(get_package_share_directory('segmentation'))
  simulation_urdf_path = os.path.join(simulation_description_path,'urdf','camera.urdf')
  robot_description_config = open(simulation_urdf_path).read()
  robot_description = {'robot_description' : robot_description_config}

  robot_controllers = os.path.join(get_package_share_directory("segmentation"),"config","gazebo_controllers.yaml")
  controller_manager = Node(
      package="controller_manager",
      executable="ros2_control_node",
      parameters=[
              {"robot_description": robot_description_config}, robot_controllers],
      output="both",
  )

  spawn_cam_cmd = Node(package='gazebo_ros', executable="spawn_entity.py",
                      arguments=['-file',simulation_urdf_path,
                                  '-entity','camera',
                                  '-z','1',
                                  '-P','1.57'],
                      output='both' )

  joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
                      
  node_robot_state_publisher_cmd = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      output='screen',
      parameters=[robot_description]
  )
  joint_state_broadcaster_spawner = Node(
      package="controller_manager",
      executable="spawner",
      arguments=["joint_state_broadcaster",
                 "--controller-manager", "/controller_manager"],
      output="both",
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
  
  # Create the launch description and populate
  ld = LaunchDescription()
 
  # Declare the launch options
  ld.add_action(declare_namespace_cmd)
  ld.add_action(declare_use_namespace_cmd)
  ld.add_action(declare_sdf_model_path_cmd)
  ld.add_action(declare_simulator_cmd)
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)
  ld.add_action(declare_world_cmd)
 
  # Add any actions
  ld.add_action(start_gazebo_client_cmd)
  ld.add_action(spawn_entity_cmd)
  ld.add_action(spawn_cam_cmd)
  ld.add_action(controller_manager)
  ld.add_action(node_robot_state_publisher_cmd)
#  ld.add_action(joint_state_broadcaster_spawner)
#  ld.add_action(delay_robot_position_controller_spawner_after_joint_state_broadcaster_spawner)
  
 
  return ld
