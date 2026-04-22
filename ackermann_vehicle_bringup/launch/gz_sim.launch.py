from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory, get_package_share_path

def generate_launch_description():

    robot_description_pkg = get_package_share_path('ackermann_vehicle_description')

    gz_launch_path = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch')

    gazebo_world_path = os.path.join(get_package_share_path('qcar_gazebo'),
                                     'worlds', 'test_world.sdf')
    
    # gazebo_world_path = 'empty.sdf'

    urdf_path = os.path.join(robot_description_pkg,
                             'urdf', 'ackermann_robot.urdf.xacro')
    
    default_params_file_path = os.path.join(robot_description_pkg, 
                               'config', 'params.yaml')
    
    rviz_config_path = os.path.join(robot_description_pkg,
                                    'rviz', 'config.rviz')
    
    gz_bridge_config_path = os.path.join(get_package_share_path('ackermann_vehicle_bringup'),
                                        'config', 'gz_bridge.yaml')
    
    models_path = os.path.join(get_package_share_path('qcar_gazebo'), 'models')

    # Declare launch arguments for Xacro parameters
    robot_parameters_file_arg = DeclareLaunchArgument(
        'robot_parameters_file', 
        default_value= default_params_file_path,
        description='Parameter file with all property values of the robot'
    )

    robot_parameters_file = LaunchConfiguration('robot_parameters_file')

    
    robot_description = ParameterValue(Command(
        ['xacro', 
         ' ',
         urdf_path, 
         ' ', 
         'robot_parameters_file:=',
         robot_parameters_file
        ]), value_type=str)
    

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_description}]
    )

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            gz_launch_path,
            "/gz_sim.launch.py"
        ]), launch_arguments={'gz_args':f'{gazebo_world_path} -r -v 4'}.items()
    )

    gz_create_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description']
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path]
    )

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': gz_bridge_config_path}]
    )



    return LaunchDescription([
        # Set the Gazebo resource path to include your custom models
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[models_path, ':', os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
        ),
        robot_parameters_file_arg,
        robot_state_publisher_node,
        gz_sim_launch,
        gz_create_entity_node,
        rviz2_node,
        gz_bridge_node
    ])