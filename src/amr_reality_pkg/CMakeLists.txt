cmake_minimum_required(VERSION 3.8)
project(amr_reality_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(canopen_core REQUIRED)
find_package(canopen_interfaces REQUIRED)
find_package(canopen_base_driver REQUIRED)
find_package(canopen_proxy_driver REQUIRED)
find_package(lely_core_libraries REQUIRED)

find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)        
find_package(geometry_msgs REQUIRED)  
find_package(tf2 REQUIRED)             
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)

cogen_dcf(robot_control)

install(DIRECTORY
  launch 
  descriptions        
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  config/ekf.yaml
  config/twist_mux.yaml
  config/laser_filter.yaml
  config/nav2_params.yaml
  config/mapper_params_online_async.yaml
  config/collision_monitor_params.yaml
  DESTINATION share/${PROJECT_NAME}/config    # khai bao ro de tranh gay confict voi cogen_dcf   
)

install(PROGRAMS
  scripts/ps4_teleop_node.py
  scripts/imu_tl740D.py
  scripts/cov_odom_scan.py
  scripts/publish_test_path.py
  DESTINATION lib/${PROJECT_NAME}
)


# ==================== C++ CONFIGURATION ====================
include_directories(
  src/include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# add_executable bao cmake biet lidar_loc co excutable la lidar_loc.cpp
# add_executable(lidar_loc 
#   src/src/lidar_loc.cpp
# )

add_executable(plicp
  src/src/plicp_localization_node.cpp
  src/src/point_to_line_icp.cpp
  src/src/occupancy_grid_map_model.cpp
  # src/src/csm_point_to_line_icp.cpp
)

target_include_directories(plicp
  PRIVATE
  src/include
  ${EIGEN3_INCLUDE_DIRS}
)


ament_target_dependencies(plicp
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_srvs
  OpenCV
  rcutils
  Eigen3
)

target_link_libraries(plicp
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  # csm
  # ${csm_LIBRARIES}
)

# khai bao de chay ros2 run ...
install(TARGETS
  plicp
  DESTINATION lib/${PROJECT_NAME}
)

# target_link_libraries(lidar_loc
#   ${OpenCV_LIBS}
# )

# install(TARGETS
#   lidar_loc
#   # lidar_filter_node
#   DESTINATION lib/${PROJECT_NAME}
# )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
