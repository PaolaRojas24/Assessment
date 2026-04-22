from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory, get_package_share_path
import os


def _parse_pose_list(poses_text):
    poses = []
    if not poses_text.strip():
        return poses

    entries = [entry.strip() for entry in poses_text.split(';') if entry.strip()]
    for entry in entries:
        values = [value.strip() for value in entry.split(',')]
        if len(values) != 4:
            raise ValueError(
                "Each pose must have 4 values: x,y,z,yaw (radians). "
                f"Invalid entry: '{entry}'"
            )
        x, y, z, yaw = [float(v) for v in values]
        poses.append((x, y, z, yaw))

    return poses


def _as_bool(text):
    return text.strip().lower() in ('1', 'true', 'yes', 'on')


def _nodes_to_execute(context, *args, **kwargs):
    gazebo_launch_path = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch')
    gazebo_worlds_folder_path = os.path.join(get_package_share_path('qcar_gazebo'), 'worlds')
    urdf_path = os.path.join(get_package_share_path('qcar_description'), 'urdf', 'qcar_system.urdf.xacro')
    gazebo_config_path = os.path.join(get_package_share_path('qcar_gazebo'), 'config', 'gz_bridge.yaml')
    vehicle_params_path = os.path.join(get_package_share_path('qcar_gazebo'), 'config', 'ego_params.yaml')

    is_ign = str(LaunchConfiguration('is_ign').perform(context))
    world = str(LaunchConfiguration('world').perform(context))
    base_name = str(LaunchConfiguration('base_name').perform(context))

    num_qcars = int(LaunchConfiguration('num_qcars').perform(context))
    poses_text = str(LaunchConfiguration('poses').perform(context))

    start_x = float(LaunchConfiguration('start_x').perform(context))
    start_y = float(LaunchConfiguration('start_y').perform(context))
    start_z = float(LaunchConfiguration('start_z').perform(context))
    start_yaw = float(LaunchConfiguration('start_yaw').perform(context))
    x_spacing = float(LaunchConfiguration('x_spacing').perform(context))
    y_spacing = float(LaunchConfiguration('y_spacing').perform(context))
    spawn_bridge = _as_bool(str(LaunchConfiguration('spawn_bridge').perform(context)))
    spawn_vehicle_controller = _as_bool(str(LaunchConfiguration('spawn_vehicle_controller').perform(context)))
    spawn_ros2_controllers = _as_bool(str(LaunchConfiguration('spawn_ros2_controllers').perform(context)))

    gazebo_world_path = os.path.join(gazebo_worlds_folder_path, world)

    display_robot_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            gazebo_launch_path,
            '/gz_sim.launch.py'
        ]),
        launch_arguments={'gz_args': f'{gazebo_world_path} -r -v 4'}.items()
    )

    pose_list = _parse_pose_list(poses_text)
    if not pose_list:
        pose_list = [
            (
                start_x + i * x_spacing,
                start_y + i * y_spacing,
                start_z,
                start_yaw,
            )
            for i in range(num_qcars)
        ]

    launch_nodes = [display_robot_gazebo]

    for i, (x, y, z, yaw) in enumerate(pose_list):
        entity_name = f'{base_name}_{i}'
        prefix = f'{entity_name}_'
        robot_description_topic = f'/{entity_name}/robot_description'

        robot_description = ParameterValue(
            Command([
                'xacro', ' ', urdf_path, ' ',
                'use_sim:=', 'true', ' ',
                'prefix:=', prefix, ' ',
                'is_ign:=', is_ign,
            ]),
            value_type=str
        )

        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=entity_name,
            parameters=[{'robot_description': robot_description}]
        )

        spawn_entity_node = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', robot_description_topic,
                '-entity', entity_name,
                '-x', str(x),
                '-y', str(y),
                '-z', str(z),
                '-Y', str(yaw),
            ]
        )

        per_entity_nodes = [robot_state_publisher_node, spawn_entity_node]

        if spawn_bridge:
            bridge_node = Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                namespace=entity_name,
                parameters=[{'config_file': gazebo_config_path}],
                remappings=[
                    ('joint_states', f'/{entity_name}/joint_states'),
                    ('/qcar_sim/odom', f'/{entity_name}/qcar_sim/odom'),
                    ('/qcar_sim/scan', f'/{entity_name}/qcar_sim/scan'),
                    ('/qcar_sim/rgb/image_raw', f'/{entity_name}/qcar_sim/rgb/image_raw'),
                    ('/qcar_sim/rgb/camera_info', f'/{entity_name}/qcar_sim/rgb/camera_info'),
                    ('/qcar_sim/csi_front/image_raw', f'/{entity_name}/qcar_sim/csi_front/image_raw'),
                    ('/qcar_sim/csi_front/camera_info', f'/{entity_name}/qcar_sim/csi_front/camera_info'),
                    ('/qcar_sim/csi_back/image_raw', f'/{entity_name}/qcar_sim/csi_back/image_raw'),
                    ('/qcar_sim/csi_back/camera_info', f'/{entity_name}/qcar_sim/csi_back/camera_info'),
                    ('/qcar_sim/csi_right/image_raw', f'/{entity_name}/qcar_sim/csi_right/image_raw'),
                    ('/qcar_sim/csi_right/camera_info', f'/{entity_name}/qcar_sim/csi_right/camera_info'),
                    ('/qcar_sim/csi_left/image_raw', f'/{entity_name}/qcar_sim/csi_left/image_raw'),
                    ('/qcar_sim/csi_left/camera_info', f'/{entity_name}/qcar_sim/csi_left/camera_info'),
                ]
            )
            per_entity_nodes.append(bridge_node)

        if spawn_vehicle_controller:
            vehicle_controller_node = Node(
                package='qcar_gazebo',
                executable='vehicle_controller',
                namespace=entity_name,
                parameters=[{'use_sim_time': True}, vehicle_params_path],
                remappings=[
                    ('/qcar_sim/user_command', f'/{entity_name}/qcar_sim/user_command'),
                    ('/forward_position_controller/commands', f'/{entity_name}/forward_position_controller/commands'),
                    ('/forward_velocity_controller/commands', f'/{entity_name}/forward_velocity_controller/commands'),
                ],
                output='screen'
            )
            per_entity_nodes.append(vehicle_controller_node)

        if spawn_ros2_controllers:
            joint_state_controller = ExecuteProcess(
                cmd=[
                    'ros2', 'control', 'load_controller',
                    '--set-state', 'active',
                    '--controller-manager', f'/{entity_name}/controller_manager',
                    'joint_state_broadcaster'
                ],
                output='screen'
            )

            forward_velocity_controller = ExecuteProcess(
                cmd=[
                    'ros2', 'control', 'load_controller',
                    '--set-state', 'active',
                    '--controller-manager', f'/{entity_name}/controller_manager',
                    'forward_velocity_controller'
                ],
                output='screen'
            )

            forward_position_controller = ExecuteProcess(
                cmd=[
                    'ros2', 'control', 'load_controller',
                    '--set-state', 'active',
                    '--controller-manager', f'/{entity_name}/controller_manager',
                    'forward_position_controller'
                ],
                output='screen'
            )

            per_entity_nodes.extend([
                RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=spawn_entity_node,
                        on_exit=[joint_state_controller]
                    )
                ),
                RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=joint_state_controller,
                        on_exit=[forward_velocity_controller, forward_position_controller]
                    )
                ),
            ])

        launch_nodes.extend(per_entity_nodes)

    return launch_nodes


def generate_launch_description():
    models_path = os.path.join(get_package_share_path('qcar_gazebo'), 'models')

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[models_path, ':', os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
        ),
        DeclareLaunchArgument(
            'is_ign',
            default_value='false',
            description='Set true/false for xacro is_ign parameter.'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='test_world.sdf',
            description='World file in qcar_gazebo/worlds.'
        ),
        DeclareLaunchArgument(
            'base_name',
            default_value='qcar',
            description='Base entity name. Final names are <base_name>_0, <base_name>_1, ...'
        ),
        DeclareLaunchArgument(
            'num_qcars',
            default_value='1',
            description='Number of qcars to spawn when poses is empty.'
        ),
        DeclareLaunchArgument(
            'poses',
            default_value='',
            description=(
                "Optional explicit poses list: 'x,y,z,yaw;x,y,z,yaw;...'. "
                'When provided, num_qcars/start/spacings are ignored.'
            )
        ),
        DeclareLaunchArgument(
            'start_x',
            default_value='-0.1375',
            description='X for first qcar when using automatic pose generation.'
        ),
        DeclareLaunchArgument(
            'start_y',
            default_value='0.32',
            description='Y for first qcar when using automatic pose generation.'
        ),
        DeclareLaunchArgument(
            'start_z',
            default_value='0.0025',
            description='Z for all qcars when using automatic pose generation.'
        ),
        DeclareLaunchArgument(
            'start_yaw',
            default_value='-1.57079633',
            description='Yaw (rad) for all qcars when using automatic pose generation.'
        ),
        DeclareLaunchArgument(
            'x_spacing',
            default_value='1.0',
            description='X increment between qcars for automatic pose generation.'
        ),
        DeclareLaunchArgument(
            'y_spacing',
            default_value='0.0',
            description='Y increment between qcars for automatic pose generation.'
        ),
        DeclareLaunchArgument(
            'spawn_bridge',
            default_value='true',
            description='Spawn one ros_gz_bridge node per qcar namespace.'
        ),
        DeclareLaunchArgument(
            'spawn_vehicle_controller',
            default_value='true',
            description='Spawn one qcar vehicle_controller node per qcar namespace.'
        ),
        DeclareLaunchArgument(
            'spawn_ros2_controllers',
            default_value='true',
            description='Load ros2_control controllers per qcar namespace.'
        ),
        OpaqueFunction(function=_nodes_to_execute),
    ])
