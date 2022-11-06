import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
import xacro
import yaml

def generate_launch_description():

    # Constants for paths to different files and folders
    package_name = 'segmentation'
    robot_name_in_model = 'beer'
    sdf_model_path = '/home/janie/.gazebo/models/beer/model.sdf'

    # Pose where we want to spawn the robot
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.52'
    spawn_r_val = '0.0'
    spawn_p_val = '0.0'
    spawn_yaw_val = '0.0'
    
    # Launch configuration variables specific to simulation
    sdf_model = LaunchConfiguration('sdf_model')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')
    
    # Declare Gazebo Node

    arg_world_filename = PathJoinSubstitution(
        [FindPackageShare("segmentation"), "worlds", "simulation.world"]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]
        ),
        launch_arguments={
            "verbose": "false",
            "pause": "false",
            "world": arg_world_filename,
        }.items(),
    )

    print('GAZEBO DONE')

    rrbot_description_path = os.path.join(
        get_package_share_directory('segmentation'))

    xacro_file = os.path.join(rrbot_description_path,
                              'urdf',
                              'rbot.urdf.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}

    print("ROBOT DESCRIPTION DONE")

    # simulation_description_path = os.path.join(get_package_share_directory('segmentation'))
    # simulation_urdf_path = os.path.join(simulation_description_path,'urdf','camera.urdf')
    # robot_description_config = open(simulation_urdf_path).read()
    # robot_description = {'robot_description' : robot_description_config}

    spawn_entity = Node(package='gazebo_ros', executable="spawn_entity.py",
                      arguments=['-topic', robot_description,
                                  '-entity','camera'],
                      output='both' )
    print("SPAWN ENTITY DONE")

    spawn_object = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-entity', robot_name_in_model, 
               '-file', sdf_model_path,
                  '-x', spawn_x_val,
                  '-y', spawn_y_val,
                  '-z', spawn_z_val,
                  '-R', spawn_r_val,
                  '-P', spawn_p_val,
                  '-Y', spawn_yaw_val,
                  
		],
                  output='screen')

    
    # Declare Robot State Publisher
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

    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_state_broadcaster",
    #                "--controller-manager", "/controller_manager"],
    #     output="screen",
    # )

    # robot_position_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["forward_position_controller", "-c", "/controller_manager"],
    # )

    # delay_robot_position_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[robot_position_controller_spawner],
    #     )
    # )

    load_joint_state_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "forward_position_controller",
        ],
        output="screen",
    )

    # load_forward_velocity_controller = ExecuteProcess(
    #     cmd=[
    #         "ros2",
    #         "control",
    #         "load_controller",
    #         "--set-state",
    #         "active",
    #         "forward_velocity_controller",
    #     ],
    #     output="screen",
    # )

    nodes = [
        controller_manager,
        # declare_use_simulator_cmd,
        gazebo,
        
        node_robot_state_publisher,
        spawn_entity,
        spawn_object,
        load_joint_state_controller,
        load_joint_trajectory_controller,
        # load_forward_velocity_controller
        # spawn_entity_can,
        # joint_state_broadcaster_spawner,
        # delay_robot_position_controller_spawner_after_joint_state_broadcaster_spawner
    ]

    return LaunchDescription(nodes)