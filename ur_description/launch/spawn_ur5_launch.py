'''
Author: Jaison
Credit: e-Yantra, IIT Bombay
Copyright (c) 2023 e-Yantra IITB
'''

import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import re
from ament_index_python import get_package_share_directory

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

def remove_comments(text):
    """Remove XML comments from a string."""
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)


def generate_launch_description():
    xacro_file = get_package_file('ur_description', 'urdf/ur5_robot.urdf.xacro')
    urdf_file = run_xacro(xacro_file)
    robot_description_raw = load_file(urdf_file)
    # Remove problematic comments
    robot_description_arm = remove_comments(robot_description_raw)


    # Load ROS2 controllers
    robot_controllers = PathJoinSubstitution([
        FindPackageShare("ur_description"), "config", "ros_controllers.yaml"])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]))

    # TF information
    robot_state_publisher_arm = Node(
        name='robot_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_arm}],
        remappings=[('robot_description', 'robot_description_ur5')]
    )

    # Spawn UR5 Arm in Gazebo
    spawn_arm = Node(
        package='gazebo_ros', 
        name='ur5_spawner',
        executable='spawn_entity.py',
        arguments=['-entity', 'ur5', '-topic', 'robot_description_ur5', '-x', '1.6', '-y', '-2.4', '-z', '0.58', '-Y', '3.14'],
        output='screen'
    )

    #  Visualization
    rviz = Node(
        name='rviz',
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        remappings=[("~/robot_description", "/robot_description_ur5")],
        output="screen"
    )

    spawn_controllers_manipulator = Node(
        package="controller_manager", 
        executable="spawner",
        name="spawner_mani",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
        output="screen"
    )
    

    spawn_controllers_state = Node(
        package="controller_manager", 
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='gui', default_value='True', description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        gazebo,
        robot_state_publisher_arm,
        controller_manager_node,
        spawn_controllers_manipulator,
        spawn_controllers_state,
        spawn_arm,
        rviz,
        ])
