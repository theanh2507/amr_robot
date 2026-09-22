cmake_minimum_required(VERSION 3.5)

project(plicp_map_matcher)

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(csm REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(plicp_map_matcher src/plicp_map_matcher.cpp)

target_include_directories(plicp_map_matcher
  PRIVATE
  "include"
)

ament_target_dependencies(plicp_map_matcher
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  rcutils
  Eigen3
  tf2_ros
  tf2
  tf2_geometry_msgs
  csm
)

# Link tuong minh phong truong hop ament_target_dependencies khong dua du thu
# vien vao lenh link (da gap thuc te voi ca tf2 lan csm):
# - tf2::fromMsg(...) nam trong libtf2.so, khong phai header-only hoan toan
# - cac ham cua csm (ld_alloc_new, sm_icp, ld_free) can duoc link dung neu
#   package csm khong export day du qua ament_target_dependencies
target_link_libraries(plicp_map_matcher
  tf2::tf2
  ${csm_LIBRARIES}
)

install(TARGETS plicp_map_matcher DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
