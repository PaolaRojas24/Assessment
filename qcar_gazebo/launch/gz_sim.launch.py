from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.event_handlers import OnProcessExit
import os
from ament_index_python.packages import get_package_share_path, get_package_share_directory


def start_vehicle_control():
    """
    Starts the necessary controllers for the vehicle's operation in ROS 2.

    @return: A tuple containing ExecuteProcess actions for the joint state, forward velocity, 
             and forward position controllers.
    """
    joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'joint_state_broadcaster'],
        output='screen')

    forward_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'forward_velocity_controller'],
        output='screen')

    forward_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'forward_position_controller'],
        output='screen')

    return (joint_state_controller,
            forward_velocity_controller,
            forward_position_controller)

def nodes_to_execute(context, *args, **kwargs):
    gazebo_launch_path = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch')

    gazebo_worlds_folder_path = os.path.join(get_package_share_path('qcar_gazebo'),
                                     'worlds')
    # gazebo_world_path = 'empty.sdf'

    # display_launch_path = os.path.join(get_package_share_directory('differential_robot_description'), 'launch')

    urdf_path = os.path.join(get_package_share_path('qcar_description'),
                             'urdf', 'qcar_system.urdf.xacro')

    rviz_config_path = os.path.join(get_package_share_path('qcar_description'),
                             'rviz', 'config.rviz')

    gazebo_config_path = os.path.join(get_package_share_path('qcar_gazebo'),
                                     'config', 'gz_bridge.yaml')
    
    vehicle_params_path = os.path.join(get_package_share_path('qcar_gazebo'),
                                       'config', 'ego_params.yaml')

    is_ign_param = LaunchConfiguration('is_ign')

    is_ign = str(is_ign_param.perform(context))

    world = str(LaunchConfiguration('world').perform(context))

    gazebo_world_path = os.path.join(gazebo_worlds_folder_path, world)

    robot_description = ParameterValue(Command(
        ["xacro",
         " ",
         urdf_path,
         " ",
         "use_sim:=",
         "true",
         " ",
         "prefix:=",
         "",
         " ",
         "is_ign:=",
         is_ign
        ]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description':robot_description}]
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    display_robot_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            gazebo_launch_path,
            '/gz_sim.launch.py'
        ]), launch_arguments={'gz_args':f'{gazebo_world_path} -r -v 4'}.items()
        # Default --physics-engine is: gz-physics-dartsim-plugin'
        # --physics-engine gz-physics-bullet-featherstone-plugin
    )

    spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=['-topic', '/robot_description',
                   '-entity', 'qcar',
                   '-x', '0.6434',#'-x', '-0.1375',#'-x', '-1.2711',
                   '-y', '2.7424',#'-y', '0.32',#'-y', '-1.667',#'-y', '-2.5010',
                   '-z', '0.0025',#'-z', '0.0025',#'-z', '0.0025',
                   '-Y', '-3.141593']#'-Y', '-1.57079633']#'-Y', '-0.7405']
    )

    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{'config_file': gazebo_config_path}]
        # parameters=[{'use_sim_time': True}],
        # arguments=[
        #     '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        #     # '/robot_description@std_msgs/msg/String',
        #     # '/differential_robot/odometry@nav_msgs/msg/Odometry',
        #     '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        #     '/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist'
        #     '/cmd_vel@gz.msgs.Twist[geometry_msgs/msg/Twist'
        # ]
    )

    # Start controllers
    joint_state, forward_velocity, forward_position = start_vehicle_control()

    # Load vehicle controller node
    vehicle_controller_node = Node(package='qcar_gazebo',
                                   executable='vehicle_controller',
                                   parameters=[
                                       {'use_sim_time': True}, 
                                       vehicle_params_path],
                                   output='screen')
    
    return [
        RegisterEventHandler(
            event_handler=OnProcessExit(target_action=spawn_entity_node,
                                        on_exit=[joint_state])),
        RegisterEventHandler(
            event_handler=OnProcessExit(target_action=joint_state,
                                        on_exit=[forward_velocity,
                                                 forward_position])),
        display_robot_gazebo,
        spawn_entity_node,
        robot_state_publisher_node,
        rviz2_node,
        vehicle_controller_node,
        bridge_node
        ]

def generate_launch_description():

    # Declare launch arguments for Xacro parameters
    ign_arg = DeclareLaunchArgument(
        'is_ign', 
        default_value= 'false',
        description='Parameter file with all property values of the robot'
    )

    world_arg = DeclareLaunchArgument(
        'world', 
        default_value= 'test_world.sdf',
        description='World file to be loaded in Gazebo. It should be located in the "worlds" directory of the qcar_gazebo package.'
    )

    models_path = os.path.join(get_package_share_path('qcar_gazebo'), 'models')

    return LaunchDescription([
        # Set the Gazebo resource path to include your custom models
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[models_path, ':', os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
        ),
        ign_arg,
        world_arg
        ] + [OpaqueFunction(function=nodes_to_execute)])