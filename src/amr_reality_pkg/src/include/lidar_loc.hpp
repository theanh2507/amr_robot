#ifndef LIDAR_LOC__LIDAR_LOC_HPP_
#define LIDAR_LOC__LIDAR_LOC_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <std_srvs/srv/empty.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <deque>
#include <tuple>
#include <string>
#include <memory>

class LidarLoc : public rclcpp::Node
{
public:
  LidarLoc();

private:
  // ================== Callback ==================
  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  // ================== Xu ly ban do / thuat toan ==================
  void cropMap();
  void processMap();
  cv::Mat createGradientMask(int size);
  bool check(float x, float y, float yaw);
  void callClearCostmaps();
  void poseTf();

  // ================== Tham so cau hinh ==================
  std::string base_frame_, odom_frame_, laser_frame_, laser_topic_;

  // ================== Trang thai / du lieu ban do ==================
  nav_msgs::msg::OccupancyGrid map_msg_;
  cv::Mat map_cropped_;
  cv::Mat map_temp_;
  sensor_msgs::msg::RegionOfInterest map_roi_info_;
  std::vector<cv::Point2f> scan_points_;

  float lidar_x_ = 250, lidar_y_ = 250, lidar_yaw_ = 0;
  const float deg_to_rad_ = static_cast<float>(M_PI) / 180.0f;
  int clear_countdown_ = -1;
  int scan_count_ = 0;

  std::deque<std::tuple<float, float, float>> data_queue_;
  const size_t max_queue_size_ = 10;

  // ================== ROS2 handles ==================
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_sub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_costmaps_client_;
  rclcpp::TimerBase::SharedPtr pose_tf_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

#endif  // LIDAR_LOC__LIDAR_LOC_HPP_