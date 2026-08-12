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
    ekf_name = 'ekf.yaml'
    use_sim_time_param = False
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path,'descriptions', 'urdf', xacro_file_name)
    robot_description_config = xacro.process_file(xacro_file)
    ekf_path = os.path.join(pkg_path, 'config', ekf_name)
    
    lidar_launch_path = os.path.join(get_package_share_directory('sllidar_ros2'),'launch','sllidar_a1_launch.py')
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}


    # phat tf tu urdf 
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
        # respawn=True,
        # respawn_delay=5.0,
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

    # Spawner model robot Gazebo
    spawn_entity_gazebo = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'amr_reality_pkg'],
        output='screen'
    )

    node_tf_map =Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen" ,
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"])

    # Load IMU and Lidar
    imu_node = Node(
        package=package_name,
        executable='imu_tl740D.py',
        name='imu_node',
        output='screen',
    )

    lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_path),
            # launch_arguments={
            #     'serial_port': '/dev/ttyUSB0',
            #     'frame_id': 'laser_frame'
            # }.items()
        )

    # Laser Filter
    laser_filter_path = PathJoinSubstitution(
    [FindPackageShare("amr_reality_pkg"), "config", "laser_filter.yaml"])

    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter',
        parameters=[laser_filter_path],
        remappings=[
                ('scan', '/scan'), 
                ('scan_filtered', '/scan_filtered')
            ]
        )

    # Robot Localization
    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_path, {'use_sim_time': use_sim_time_param}],
    )

    # Load ps4
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': 0,
        }]
    )

    ps4_node = Node(
        package=package_name,
        executable='ps4_teleop_node.py',
        name='ps4',
        output='screen',
    )

    # Run Rviz
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
        diff_drive_controller_spawner,
        imu_node,
        lidar_launch,
        joy_node,
        ps4_node,
        laser_filter_node,
        robot_localization,
        rviz_node,
        # gazebo,
        # spawn_entity_gazebo,
    ])



# ls /dev/input/js*
# ros2 run joy joy_node --ros-args -p device_id:=0
# ros2 run tf2_ros tf2_echo base_link Imu_Link               # debug transform


# ros2 topic pub -r10 /velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0]}"
# ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped

# run slamtoolbox
# ros2 launch slam_toolbox online_async_launch.py params_file:=/home/theanh/Robot_Project/AMR_Robot/amr1_ws/src/amr_reality_pkg/config/mapper_params_online_async.yaml

# save map
# ros2 run nav2_map_server map_saver_cli -f /home/theanh/Robot_Project/AMR_Robot/amr1_ws/src/amr_reality_pkg/maps/map_xuong

# load map
# ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/theanh/Robot_Project/AMR_Robot/amr1_ws/src/amr_reality_pkg/maps/map_xuong.yaml
# ros2 run nav2_util lifecycle_bringup map_server       (vi map_server va amcl la managed lifecycle node, khi run xong dang o trang thai unconfigured)

# load amcl
# ros2 run nav2_amcl amcl (--ros-args -p use_sim_time:=true --param-file /path_to_amcl.yaml)
# ros2 run nav2_util lifecycle_bringup amcl

# load map va amcl trong launch file
# ros2 launch nav2_bringup localization_launch.py map:=/home/theanh/Robot_Project/AMR_Robot/amr1_ws/src/amr_reality_pkg/maps/map_xuong.yaml

# load nav2
# ros2 launch nav2_bringup navigation_launch.py map_subscribe_transient_local:=true


# ros2 lifecycle get /amcl
# ros2 lifecycle get /map_server



# mo file udev quan ly ten thiet bi usb
# udevadm info --query=all --name=/dev/ttyUSB0 | grep -E "ID_VENDOR|ID_MODEL|ID_SERIAL"
# sudo nano /etc/udev/rules.d/99-robot-devices.rules
# sudo udevadm control --reload-rules
# sudo udevadm trigger
# ls -l /dev/ttyIMU                 (output: lrwxrwxrwx 1 root root 7 Aug  3 10:07 /dev/ttyLIDAR -> ttyUSB0)


# ssh orangepi@192.168.0.168        # pass: orangepi

 