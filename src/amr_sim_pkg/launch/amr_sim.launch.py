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

    # 1. Định nghĩa các hằng số tên package và đường dẫn file xacro tổng
    package_name = 'amr_sim_pkg'
    xacro_file_name = 'robot.urdf.xacro'
    
    # Lấy đường dẫn tuyệt đối đến thư mục cài đặt của package
    pkg_path = get_package_share_directory(package_name)
    xacro_file_path = os.path.join(pkg_path, 'descriptions','urdf', xacro_file_name)

    # 2. Tạo tham số Launch Argument (để bật/tắt sim_time khi chạy mô phỏng)
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
    )

    # 3. Sử dụng thư viện xacro của ROS 2 để biên dịch file xacro tổng thành chuỗi URDF XML
    # Thao tác này sẽ tự động đi tìm và gộp các file core, sensors, inertial lại với nhau
    robot_description_raw = xacro.process_file(xacro_file_path).toxml()

    # 4. Cấu hình Node robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw, # Nạp chuỗi cấu trúc robot vào đây
            'use_sim_time': use_sim_time                 # Đồng bộ thời gian với hệ thống
        }]
    )

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )])
    )

    spawn_entity = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-topic', 'robot_description',
        '-entity', 'my_sim_robot',
        '-z', '0.1'
    ],
    output='screen'
    )

    # Run Rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # Laser Filter
    laser_filter_path = PathJoinSubstitution(
    [FindPackageShare("amr_sim_pkg"), "config", "laser_filter.yaml"])

    laser_filter_node = Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter',
            parameters=[laser_filter_path, {'use_sim_time': use_sim_time}],
            remappings=[
                    ('scan', '/scan'),                  # Đọc dữ liệu thô từ Lidar thật gửi vào
                    ('scan_filtered', '/scan_filtered') # Xuất dữ liệu đã cắt góc ra topic này
                ]
        )


    # 5. Trả về đối tượng LaunchDescription để ROS 2 thực thi
    return LaunchDescription([
        declare_use_sim_time_cmd,
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        laser_filter_node,
        rviz_node,
    ])