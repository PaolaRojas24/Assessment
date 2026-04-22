from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():

    robot_description_pkg = get_package_share_path("ackermann_vehicle_description")

    urdf_path = os.path.join(robot_description_pkg,
                             'urdf', 'ackermann_robot.urdf.xacro')

    robot_parameters_file_path = os.path.join(robot_description_pkg,
                                              'config', 'params.yaml')
    
    rviz_config_path = os.path.join(robot_description_pkg,
                                    'rviz', 'config.rviz')
    
    parameter_file_arg = DeclareLaunchArgument(
        'robot_parameters_file',
        default_value=robot_parameters_file_path,
        description='Archivo de parametros para definir a nuestro robot.'
    )

    robot_file = LaunchConfiguration('robot_parameters_file')
    
    robot_description = ParameterValue(
        Command(
            [
             'xacro ', 
             urdf_path,
             ' ',
             'robot_parameters_file:=',
             robot_file

            ]), value_type= str)

    robot_state_publisher = Node(
        name='robot_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_gui = Node(
        name='joint_state_publisher_gui',
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    rviz_node = Node(
        name='rviz2',
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path]    
    )

    return LaunchDescription([
        parameter_file_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])