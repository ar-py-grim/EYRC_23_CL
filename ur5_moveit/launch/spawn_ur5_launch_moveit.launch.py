import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch_param_builder import ParameterBuilder

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def load_file(file_path):
    """Load the contents of a file into a string"""
    try:
        with open(file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(file_path):
    """Load a yaml file into a dictionary"""
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def run_xacro(xacro_file):
    """Run xacro and output a file in the same directory with the same name, w/o a .xacro suffix"""
    urdf_file, ext = os.path.splitext(xacro_file)
    if ext != '.xacro':
        raise RuntimeError(f'Input file to xacro must have a .xacro extension, got {xacro_file}')
    os.system(f'xacro {xacro_file} -o {urdf_file}')
    return urdf_file


def generate_launch_description():
    moveit_config_folder_name = 'ur5_moveit'

    xacro_file = get_package_file(moveit_config_folder_name, 'config/ur5.urdf.xacro')
    urdf_file = run_xacro(xacro_file)
    robot_description_arm = load_file(urdf_file)

    srdf_file = get_package_file(moveit_config_folder_name, 'config/ur5.srdf')
    kinematics_file = get_package_file(moveit_config_folder_name, 'config/kinematics.yaml')
    ompl_config_file = get_package_file(moveit_config_folder_name, 'config/ompl_planning.yaml')
    moveit_controllers_file = get_package_file(moveit_config_folder_name, 'config/moveit_controllers.yaml')
    moveit_servo_file = get_package_file(moveit_config_folder_name, "config/ur_servo.yaml")
    moveit_rviz_file = get_package_file(moveit_config_folder_name,'config/moveit.rviz')
    joint_limits_file = get_package_file(moveit_config_folder_name, 'config/initial_positions.yaml')

    robot_description_semantic = load_file(srdf_file)
    kinematics_config = load_yaml(kinematics_file)
    ompl_config = load_yaml(ompl_config_file)
    joint_limits_config = load_yaml(joint_limits_file)

    moveit_controllers = {
        'moveit_simple_controller_manager' : load_yaml(moveit_controllers_file),
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01
    }

    planning_scene_monitor_config = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True
    }

    # MoveIt node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {"use_sim_time": True},
            {'robot_description': robot_description_arm},
            {'robot_description_semantic': robot_description_semantic},
            {'robot_description_kinematics': kinematics_config},
            {'ompl': ompl_config},
            {'joint_limits': joint_limits_config},
            {'planning_pipelines': ['ompl']},
            moveit_controllers,
            trajectory_execution,
            planning_scene_monitor_config,
        ],
    )

    # Servo node for realtime control
    servo_params = (
        ParameterBuilder("moveit_servo")
        .yaml(
            parameter_namespace="moveit_servo",
            file_path=moveit_servo_file,
        )
        .to_dict()
    )
    # servo_yaml = load_yaml(moveit_servo_file)
    # servo_params = {"moveit_servo": servo_yaml}
    robot_description_s = {"robot_description": robot_description_arm}
    robot_description_semantic_s = {"robot_description_semantic": robot_description_semantic}

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description_s,
            robot_description_semantic_s,
        ],
        output="screen",
    )

    #  Visualization (parameters needed for MoveIt display plugin)
    rviz = Node(
        name='rviz',
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=["-d", moveit_rviz_file],
        parameters=[
            {'robot_description': robot_description_arm},
            {'robot_description_semantic': robot_description_semantic},
            {'robot_description_kinematics': kinematics_config},
        ],
    )

    return LaunchDescription([
        move_group_node,
        servo_node,
        rviz
    ])
