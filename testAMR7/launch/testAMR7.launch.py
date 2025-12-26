import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch file for 4-wheel swerve drive robot with hardware switching capability.
    
    Gazebo Simulation (default):
        ros2 launch testAMR7 testAMR7.launch.py
        
    Real Hardware:
        ros2 launch testAMR7 testAMR7.launch.py use_sim:=false
        
    With Gamepad:
        ros2 launch testAMR7 testAMR7.launch.py enable_gamepad:=true
    """
    
    # Get package directory
    pkg_testAMR7 = get_package_share_directory('testAMR7')
    
    # ============================================================================
    # LAUNCH ARGUMENTS
    # ============================================================================
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Use Gazebo simulation if true, real hardware if false'
    )
    
    enable_gamepad_arg = DeclareLaunchArgument(
        'enable_gamepad',
        default_value='true',
        description='Enable Xbox gamepad control'
    )
    
    arduino_port_arg = DeclareLaunchArgument(
        'arduino_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Arduino communication'
    )
    
    # ============================================================================
    # GAZEBO ENVIRONMENT SETUP
    # ============================================================================
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_testAMR7, 'meshes'),
            ':',
            os.path.join(os.path.dirname(pkg_testAMR7), ''),
        ]
    )
    
    # ============================================================================
    # ROBOT DESCRIPTION
    # ============================================================================
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('testAMR7'), 'urdf', 'testAMR7.urdf.xacro']
            ),
            ' ',
            'use_sim:=',
            LaunchConfiguration('use_sim'),
            ' ',
            'arduino_port:=',
            LaunchConfiguration('arduino_port'),
        ]
    )
    
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # ============================================================================
    # CONFIGURATION FILES
    # ============================================================================
    controller_config = PathJoinSubstitution(
        [FindPackageShare('testAMR7'), 'config', 'omni_base_controller_config.yaml']
    )
    
    gamepad_config = PathJoinSubstitution(
        [FindPackageShare('testAMR7'), 'config', 'gamepad_controllers.yaml']
    )
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('testAMR7'), 'rviz', 'testAMR7.rviz']
    )

    # ============================================================================
    # CORE NODES (Always Launch)
    # ============================================================================
    
    # Robot State Publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
    )

    # ============================================================================
    # GAMEPAD CONTROL NODES
    # ============================================================================
    
    # Joy node - reads Xbox controller inputs
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[gamepad_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_gamepad'))
    )
    
    # Swerve gamepad controller - converts joy to cmd_vel
    swerve_gamepad_node = Node(
        package='testAMR7',
        executable='swerve_gamepad_controller.py',
        name='swerve_gamepad_controller',
        parameters=[gamepad_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_gamepad'))
    )

    # ============================================================================
    # SIMULATION NODES (Only when use_sim:=true)
    # ============================================================================
    
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
        condition=IfCondition(LaunchConfiguration('use_sim'))
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'testAMR7',
            '-allow_renaming', 'true',
            '-z', '0.1',
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_sim'))
    )

    # Bridge between ROS2 and Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_sim'))
    )

    # ============================================================================
    # REAL HARDWARE NODE (Only when use_sim:=false)
    # ============================================================================
    
    # Controller Manager for Real Hardware
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            controller_config,
        ],
        output='both',
        condition=UnlessCondition(LaunchConfiguration('use_sim'))
    )

    # ============================================================================
    # CONTROLLER SPAWNERS (Always Launch, but timing differs)
    # ============================================================================
    
    # Load Joint State Broadcaster
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Load Omni Base Controller
    load_omni_base_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['omni_base_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # ============================================================================
    # EVENT HANDLERS - SIMULATION PATH
    # ============================================================================
    
    # For Gazebo: Load controllers after entity spawns
    load_joint_state_broadcaster_sim = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_broadcaster],
        ),
        condition=IfCondition(LaunchConfiguration('use_sim'))
    )

    load_omni_base_controller_sim = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_omni_base_controller],
        ),
        condition=IfCondition(LaunchConfiguration('use_sim'))
    )

    # ============================================================================
    # EVENT HANDLERS - REAL HARDWARE PATH
    # ============================================================================
    
    # For Real Hardware: Use TimerAction instead of OnProcessExit
    load_joint_state_broadcaster_hw = TimerAction(
        period=3.0,
        actions=[load_joint_state_broadcaster],
        condition=UnlessCondition(LaunchConfiguration('use_sim'))
    )

    load_omni_base_controller_hw = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[
                TimerAction(
                    period=1.0,
                    actions=[load_omni_base_controller]
                )
            ],
        ),
        condition=UnlessCondition(LaunchConfiguration('use_sim'))
    )

    # ============================================================================
    # LAUNCH DESCRIPTION
    # ============================================================================
    return LaunchDescription([
        # Launch arguments
        use_sim_arg,
        enable_gamepad_arg,
        arduino_port_arg,
        
        # Environment
        gz_resource_path,
        
        # ========== SIMULATION NODES ==========
        gazebo,
        spawn_entity,
        bridge,
        
        # ========== REAL HARDWARE NODES ==========
        controller_manager_node,
        
        # ========== COMMON NODES ==========
        robot_state_pub_node,
        rviz_node,
        
        # ========== GAMEPAD CONTROL ==========
        joy_node,
        swerve_gamepad_node,
        
        # ========== SIMULATION CONTROLLER LOADING ==========
        load_joint_state_broadcaster_sim,
        load_omni_base_controller_sim,
        
        # ========== REAL HARDWARE CONTROLLER LOADING ==========
        load_joint_state_broadcaster_hw,
        load_omni_base_controller_hw,
    ])