import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_behavior_tree = get_package_share_directory('behavior_tree')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Paths to simulation and config assets
    world_file = os.path.join(pkg_behavior_tree, 'worlds', 'V3_Utrack_UGV26.sdf')
    xacro_file = os.path.join(pkg_behavior_tree, 'urdf', 'ugv_rover.xacro')
    
    # Behavior Tree target configuration setup
    bt_file_arg = DeclareLaunchArgument(
        'bt_xml',
        default_value=os.path.join(pkg_behavior_tree, 'config', 'main_tree.xml'),
        description='Path to the Behavior Tree XML file to execute'
    )

    # 1. Fire up Gazebo Sim with your track world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 2. Publish Robot State / Transforms from URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': os.popen(f'xacro {xacro_file}').read()}]
    )

    # 3. Spawn the Rover into the World
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'ugv_rover', '-z', '0.3'],
        output='screen'
    )

    # 4. ROS 2 Control Spawner: Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    # 5. ROS 2 Control Spawner: Friend's Ackerman Controller
    # NOTE: Change "ackermann_steering_controller" if your friend named it differently in their yaml!
    ackerman_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ackermann_steering_controller"],
        output="screen"
    )

    # 6. Your friend's Core MPPI Node
    # NOTE: Change executable to match your friend's MPPI package node name
    mppi_node = Node(
        package='MPPI_Ackerman', 
        executable='mppi_controller_node', 
        name='mppi_controller',
        output='screen',
        parameters=[os.path.join(pkg_behavior_tree, 'config', 'mppi_params.yaml')]
    )

    # 7. Bridge Gazebo /odom and /cmd_vel topics back to ROS 2 standard messages
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    # 8. Your Main Behavior Tree Node Executor
    bt_node = Node(
        package='behavior_tree',
        executable='bt_main_node',
        name='bt_engine',
        output='screen',
        parameters=[{'bt_xml_filename': LaunchConfiguration('bt_xml')}]
    )

    return LaunchDescription([
        bt_file_arg,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        joint_state_broadcaster_spawner,
        ackerman_controller_spawner,
        mppi_node,
        ros_gz_bridge,
        bt_node
    ])