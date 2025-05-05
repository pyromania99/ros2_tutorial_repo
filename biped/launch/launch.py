from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution ,PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package path
    pkg_path = get_package_share_directory('biped')

    # Set Gazebo model path
    os.environ['GAZEBO_MODEL_PATH'] = os.path.join(pkg_path, '..') + ":" + os.environ.get('GAZEBO_MODEL_PATH', '')

    # File paths
    urdf_path = os.path.join(pkg_path, 'urdf', 'biped.urdf.xacro')
    rviz_config_path = os.path.join(pkg_path, 'config', 'biped_config.rviz')
    controller_config_path = PathJoinSubstitution([pkg_path, 'config', 'biped.yaml'])
    src_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
    asmc_config_path = os.path.join(pkg_path, 'config', 'asmc.yaml')
    effort_asmc_script = os.path.join(src_dir, 'effort_asmc.py')

    # Launch arguments
    use_with_sim_arg = DeclareLaunchArgument('use_sim_time', default_value='True', description='Use simulation settings')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='False', description='Launch RViz')
    use_gazebo_arg = DeclareLaunchArgument('use_gazebo', default_value='True', description='Launch Gazebo')

    use_sim_time = LaunchConfiguration('use_sim_time')
    # asmc_config_path = os.path.join(pkg_path, 'config', 'asmc_config.yaml')

    # asmc_node = Node(
    #     package='biped',
    #     executable='asmc_effort_publisher',
    #     name='asmc_effort_publisher',
    #     output='screen',
    #     parameters=[asmc_config_path]
    # )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'robot_description': ParameterValue(
                    Command(['xacro ', urdf_path, ' ',
                            'USE_WITH_SIM:=', use_sim_time, ' ',
                            'YAML_PATH:=', controller_config_path]), value_type=str),
            }
        ]
    )

    # Joint State Publisher GUI (for debugging)
    joint_state_publisher_gui = Node(
        condition=IfCondition(PythonExpression([
            LaunchConfiguration('use_rviz'), ' and not ', LaunchConfiguration('use_gazebo')
        ])),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'),
                          'launch', 'gz_sim.launch.py')]
        ),
        launch_arguments=[('gz_args', ['-r empty.sdf'])],
        condition=IfCondition(LaunchConfiguration('use_gazebo'))
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(LaunchConfiguration('use_gazebo')),
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]']
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('use_gazebo')),
        arguments=[
            '-name', 'robot',
            '-x', str(-3.5),
            '-y', str(0.0),
            '-z', str(0.0),
            '-Y', str(0.0),
            '-topic', 'robot_description'
        ]
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            controller_config_path,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('~/robot_description', 'robot_description')
        ],
        output='screen'
    )

    load_effort_controller = Node(
        package='controller_manager',
        executable='spawner',
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
        arguments=['effort_controller']
    )

    load_controllers = Node(
        package='controller_manager',
        executable='spawner',
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
        arguments=['leg_joint_trajectory_controller','--inactive']
    )
    
    load_joint_state_broadcaster = Node(
        condition=IfCondition(LaunchConfiguration('use_gazebo')),
        package='controller_manager',
        executable='spawner',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    dynamic_python_scripts = []
    for script in os.listdir(src_dir):
        if script == 'effort_asmc.py':
            # Skip effort_asmc.py - we'll launch it separately with parameters
            continue
        elif script.endswith('.py'):
            dynamic_python_scripts.append(
                ExecuteProcess(
                    cmd=['python3', os.path.join(src_dir, script)],
                    output='screen'
                )
            )

    effort_asmc_process = ExecuteProcess(
        cmd=[
            'python3', effort_asmc_script,
            '--ros-args', '--params-file', asmc_config_path
        ],
        output='screen'
    )

    return LaunchDescription([
        use_with_sim_arg,
        use_rviz_arg,
        use_gazebo_arg,
        spawn_entity,
        gazebo,
        bridge,
        controller_manager,
        load_effort_controller,
        # asmc_node,  
        robot_state_publisher,
        load_joint_state_broadcaster,
        joint_state_publisher_gui,
        rviz_node,
        load_controllers,
        effort_asmc_process,  # This specific call for effort_asmc.py
        *dynamic_python_scripts  # All other Python scripts
    ])
