import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
import xacro

def generate_launch_description():

    package_name = 'amr_sim_pkg'
    xacro_file_name = 'robot.urdf.xacro'
    laser_filter_name = 'laser_filter.yaml'
    world_name = "maze_2.world"
    ekf_name = "ekf.yaml"
    
    pkg_path = get_package_share_directory(package_name)
    xacro_file_path = os.path.join(pkg_path, 'descriptions','urdf', xacro_file_name)
    laser_filter_path = os.path.join(pkg_path, 'config', laser_filter_name)
    world_path = os.path.join(pkg_path, 'worlds', world_name)
    ekf_path = os.path.join(pkg_path, 'config', ekf_name)

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
    )

    # Sử dụng thư viện xacro của ROS để biên dịch file xacro tổng thành chuỗi URDF XML
    # Thao tác này sẽ tự động đi tìm và gộp các file core, sensors, inertial lại với nhau
    robot_description_raw = xacro.process_file(xacro_file_path).toxml()

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,     # Nạp chuỗi cấu trúc robot vào đây
            'use_sim_time': use_sim_time                    # Đồng bộ thời gian với hệ thống
        }]
    )

    # Load World
    world = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model file to load')

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )]),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    spawn_entity = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-topic', 'robot_description',
        '-entity', 'my_sim_robot',
        '-z', '0.1'
    ],
    output='screen')


    # Run Rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Laser Filter
    laser_filter_node = Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter',
            parameters=[laser_filter_path, {'use_sim_time': use_sim_time}],
            remappings=[
                    ('scan', '/scan'),
                    ('scan_filtered', '/scan_filtered'),
                ]
        )

    # Robot Localization
    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_path, {'use_sim_time': use_sim_time}],
    )


    return LaunchDescription([
        declare_use_sim_time_cmd,
        node_robot_state_publisher,
        world,
        gazebo,
        spawn_entity,
        laser_filter_node,
        robot_localization,
        rviz_node,
    ])


# ros2 launch amr_sim_pkg amr_sim.launch.py 

# run slamtoolbox
# ros2 launch slam_toolbox online_async_launch.py params_file:=/home/theanh/Robot_Project/AMR_Robot/amr1_ws/src/amr_sim_pkg/config/mapper_params_online_async.yaml

# save map
# ros2 run nav2_map_server map_saver_cli -f /home/theanh/Robot_Project/AMR_Robot/amr1_ws/src/amr_sim_pkg/maps/map_sim

# load duoc map sau do set initial pose moi xuat hien fixed frame: map
# load map va amcl trong launch file (reliability policy: reliable, durability policy: transient_local)
# ros2 launch amr_sim_pkg localization_launch.py map:=/home/theanh/Robot_Project/AMR_Robot/amr1_ws/src/amr_sim_pkg/maps/map_sim.yaml use_sim_time:=true

# load nav2
# ros2 launch amr_sim_pkg navigation_launch.py map_subscribe_transient_local:=true use_sim_time:=true

# ros2 lifecycle get /amcl
# ros2 lifecycle get /map_server
