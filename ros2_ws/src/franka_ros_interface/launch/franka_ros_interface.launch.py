import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    robot_num_parameter_name = 'robot_num'
    robot_num = LaunchConfiguration(robot_num_parameter_name)

    return LaunchDescription([
        DeclareLaunchArgument(
            robot_num_parameter_name,
            default_value='1',
            description='Robot number.'),
        Node(
            package='franka_ros_interface',
            executable='franka_ros_interface_execute_skill_action_server',
            name=['execute_skill_action_server_node_', robot_num],
        ),
        Node(
            package='franka_ros_interface',
            executable='robot_state_publisher',
            name=['robot_state_publisher_node_', robot_num],
        ),
        Node(
            package='franka_ros_interface',
            executable='franka_interface_status_publisher',
            name=['franka_interface_status_publisher_node_', robot_num],
        ),
        Node(
            package='franka_ros_interface',
            executable='run_loop_process_info_state_publisher',
            name=['run_loop_process_info_state_publisher_node_', robot_num],
        ),
        Node(
            package='franka_ros_interface',
            executable='get_current_robot_state_server',
            name=['get_current_robot_state_server_node_', robot_num],
            parameters=[{'robot_state_topic_name': PythonExpression("/robot_state_publisher_node_"+robot_num+"/robot_state")}],
        ),
        Node(
            package='franka_ros_interface',
            executable='get_current_franka_interface_status_server',
            name=['get_current_franka_interface_status_server_node_', robot_num],
            parameters=[{'franka_interface_status_topic_name': PythonExpression("/franka_interface_status_publisher_node_"+robot_num+"/franka_interface_status")}],
        ),
        Node(
            package='franka_ros_interface',
            executable='sensor_data_subscriber',
            name=['sensor_data_subscriber_node_', robot_num],
            parameters=[{'sensor_data_topic_name': PythonExpression("/sensor_data_"+robot_num+"/sensor_data")}],
        ),
    ])