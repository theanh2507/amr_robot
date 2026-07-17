import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'amr_reality_pkg'
    xacro_file_name = 'robot.urdf.xacro'
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_param = False


    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path,'descriptions', 'urdf', xacro_file_name)
    robot_description_config = xacro.process_file(xacro_file)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}


    node_robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='screen',
    parameters=[params]
    )

    robot_control_config = PathJoinSubstitution(
    [FindPackageShare("amr_reality_pkg"), "config/robot_control", "ros2_controllers.yaml"]
    )

    robot_description_param = {'robot_description': robot_description_config.toxml()}

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        # Truyền robot_description_param (dạng dict) và file cấu hình yaml
        parameters=[robot_description_param, robot_control_config],
        output="screen",
    )


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': use_sim_time_param}],
    )

    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': use_sim_time_param}],
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': use_sim_time_param}],
    )

    trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["trajectory_controller", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': use_sim_time_param}],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': use_sim_time_param}],
    )


    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
    )

    # Spawner entity vào Gazebo
    spawn_entity_gazebo = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'amr_reality_pkg'],
        output='screen'
    )

    node_tf_map =Node(package="tf2_ros",
                            executable="static_transform_publisher",
                            output="screen" ,
                            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time if true'),
        control_node,
        node_robot_state_publisher,
        joint_state_broadcaster_spawner,
        # position_controller_spawner,
        # velocity_controller_spawner,
        diff_drive_controller_spawner,
        # node_tf_map,
        rviz_node,
        # gazebo,
        # spawn_entity_gazebo,
    ])

# ros2 topic pub -r10 /velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0]}"
# ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped