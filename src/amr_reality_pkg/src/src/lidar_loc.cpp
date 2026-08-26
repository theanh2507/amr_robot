#include "lidar_loc.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>
#include <cmath>

using std::placeholders::_1;

// ============================================================
// Constructor
// ============================================================
LidarLoc::LidarLoc() : Node("lidar_loc")
{
  // ---- Tham so (giong ROS1: base_frame, odom_frame, laser_frame, laser_topic) ----
  this->declare_parameter<std::string>("base_frame", "base_footprint");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("laser_frame", "Lidar_Link");
  this->declare_parameter<std::string>("laser_topic", "scan_filtered");

  base_frame_  = this->get_parameter("base_frame").as_string();
  odom_frame_  = this->get_parameter("odom_frame").as_string();
  laser_frame_ = this->get_parameter("laser_frame").as_string();
  laser_topic_ = this->get_parameter("laser_topic").as_string();

  // ---- TF buffer/listener/broadcaster ----
  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  // ---- Subscribers ----
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&LidarLoc::mapCallback, this, _1));

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    laser_topic_, rclcpp::SensorDataQoS(),
    std::bind(&LidarLoc::scanCallback, this, _1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 1,
    std::bind(&LidarLoc::initialPoseCallback, this, _1));

  // ---- Service client clear costmap (tuong duong move_base/clear_costmaps cua ROS1) ----
  clear_costmaps_client_ = this->create_client<std_srvs::srv::Empty>(
    "global_costmap/clear_entirely_global_costmap");

  // ---- Timer thay cho vong lap while(ros::ok()) cua ROS1, 30Hz ----
  pose_tf_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(33),
    std::bind(&LidarLoc::poseTf, this));

  RCLCPP_INFO(this->get_logger(),
    "lidar_loc started. base_frame=%s odom_frame=%s laser_frame=%s laser_topic=%s",
    base_frame_.c_str(), odom_frame_.c_str(), laser_frame_.c_str(), laser_topic_.c_str());
}

// ============================================================
// initialPoseCallback
// ============================================================
void LidarLoc::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  double map_x = msg->pose.pose.position.x;
  double map_y = msg->pose.pose.position.y;

  tf2::Quaternion q;
  tf2::fromMsg(msg->pose.pose.orientation, q);

  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  if (map_msg_.info.resolution <= 0) {
    RCLCPP_ERROR(this->get_logger(), "Ban do khong hop le hoac chua nhan duoc");
    return;
  }

  lidar_x_ = static_cast<float>(
    (map_x - map_msg_.info.origin.position.x) / map_msg_.info.resolution
    - map_roi_info_.x_offset);
  lidar_y_ = static_cast<float>(
    (map_y - map_msg_.info.origin.position.y) / map_msg_.info.resolution
    - map_roi_info_.y_offset);
  lidar_yaw_ = static_cast<float>(-yaw);

  clear_countdown_ = 30;
}

// ============================================================
// mapCallback
// ============================================================
void LidarLoc::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  map_msg_ = *msg;
  cropMap();
  processMap();
}

// ============================================================
// cropMap
// ============================================================
void LidarLoc::cropMap()
{
  const nav_msgs::msg::MapMetaData & info = map_msg_.info;

  int xMax, xMin, yMax, yMin;
  xMax = xMin = static_cast<int>(info.width) / 2;
  yMax = yMin = static_cast<int>(info.height) / 2;
  bool bFirstPoint = true;

  cv::Mat map_raw(info.height, info.width, CV_8UC1, cv::Scalar(128));

  for (unsigned int y = 0; y < info.height; y++) {
    for (unsigned int x = 0; x < info.width; x++) {
      int index = y * info.width + x;
      map_raw.at<uchar>(y, x) = static_cast<uchar>(map_msg_.data[index]);

      if (map_msg_.data[index] == 100) {
        if (bFirstPoint) {
          xMax = xMin = x;
          yMax = yMin = y;
          bFirstPoint = false;
          continue;
        }
        xMin = std::min(xMin, static_cast<int>(x));
        xMax = std::max(xMax, static_cast<int>(x));
        yMin = std::min(yMin, static_cast<int>(y));
        yMax = std::max(yMax, static_cast<int>(y));
      }
    }
  }

  int cen_x = (xMin + xMax) / 2;
  int cen_y = (yMin + yMax) / 2;

  int new_half_width  = std::abs(xMax - xMin) / 2 + 50;
  int new_half_height = std::abs(yMax - yMin) / 2 + 50;
  int new_origin_x = cen_x - new_half_width;
  int new_origin_y = cen_y - new_half_height;
  int new_width  = new_half_width * 2;
  int new_height = new_half_height * 2;

  if (new_origin_x < 0) new_origin_x = 0;
  if ((new_origin_x + new_width) > static_cast<int>(info.width))
    new_width = info.width - new_origin_x;
  if (new_origin_y < 0) new_origin_y = 0;
  if ((new_origin_y + new_height) > static_cast<int>(info.height))
    new_height = info.height - new_origin_y;

  cv::Rect roi(new_origin_x, new_origin_y, new_width, new_height);
  map_cropped_ = map_raw(roi).clone();

  map_roi_info_.x_offset = new_origin_x;
  map_roi_info_.y_offset = new_origin_y;
  map_roi_info_.width  = new_width;
  map_roi_info_.height = new_height;

  // Reset pose giong ban goc (goi lai initialPoseCallback voi pose (0,0,0))
  auto init_pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  init_pose->pose.pose.position.x = 0.0;
  init_pose->pose.pose.position.y = 0.0;
  init_pose->pose.pose.orientation.w = 1.0;
  initialPoseCallback(init_pose);
}

// ============================================================
// createGradientMask
// ============================================================
cv::Mat LidarLoc::createGradientMask(int size)
{
  cv::Mat mask(size, size, CV_8UC1);
  int center = size / 2;
  for (int y = 0; y < size; y++) {
    for (int x = 0; x < size; x++) {
      double distance = std::hypot(x - center, y - center);
      int value = cv::saturate_cast<uchar>(255 * std::max(0.0, 1.0 - distance / center));
      mask.at<uchar>(y, x) = value;
    }
  }
  return mask;
}

// ============================================================
// processMap
// ============================================================
void LidarLoc::processMap()
{
  if (map_cropped_.empty()) return;

  map_temp_ = cv::Mat::zeros(map_cropped_.size(), CV_8UC1);
  cv::Mat gradient_mask = createGradientMask(101);

  for (int y = 0; y < map_cropped_.rows; y++) {
    for (int x = 0; x < map_cropped_.cols; x++) {
      if (map_cropped_.at<uchar>(y, x) == 100) {
        int left   = std::max(0, x - 50);
        int top    = std::max(0, y - 50);
        int right  = std::min(map_cropped_.cols - 1, x + 50);
        int bottom = std::min(map_cropped_.rows - 1, y + 50);

        cv::Rect roi(left, top, right - left + 1, bottom - top + 1);
        cv::Mat region = map_temp_(roi);

        int mask_left = 50 - (x - left);
        int mask_top  = 50 - (y - top);
        cv::Rect mask_roi(mask_left, mask_top, roi.width, roi.height);
        cv::Mat mask = gradient_mask(mask_roi);

        cv::max(region, mask, region);
      }
    }
  }
}

// ============================================================
// check - dieu kien hoi tu vong lap hill-climbing
// ============================================================
bool LidarLoc::check(float x, float y, float yaw)
{
  if (x == 0 && y == 0 && yaw == 0) {
    data_queue_.clear();
    return true;
  }

  data_queue_.push_back(std::make_tuple(x, y, yaw));
  if (data_queue_.size() > max_queue_size_) {
    data_queue_.pop_front();
  }

  if (data_queue_.size() == max_queue_size_) {
    auto & first = data_queue_.front();
    auto & last  = data_queue_.back();

    float dx = std::abs(std::get<0>(last) - std::get<0>(first));
    float dy = std::abs(std::get<1>(last) - std::get<1>(first));
    float dyaw = std::abs(std::get<2>(last) - std::get<2>(first));

    if (dx < 5 && dy < 5 && dyaw < 5 * deg_to_rad_) {
      data_queue_.clear();
      return true;
    }
  }
  return false;
}

// ============================================================
// scanCallback - thuat toan chinh (hill-climbing matching)
// ============================================================
void LidarLoc::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  scan_points_.clear();
  double angle = msg->angle_min;

  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    transformStamped = tf_buffer_->lookupTransform(
      base_frame_, laser_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    return;
  }

  // Kiem tra lidar co lap nguoc khong
  tf2::Quaternion q_lidar;
  tf2::fromMsg(transformStamped.transform.rotation, q_lidar);

  double roll, pitch, yaw;
  tf2::Matrix3x3(q_lidar).getRPY(roll, pitch, yaw);

  // const double tolerance = 0.1;
  // bool lidar_is_inverted = std::abs(std::abs(roll) - M_PI) < tolerance;
  // lidar_is_inverted = lidar_is_inverted && !(std::abs(std::abs(pitch) - M_PI) < tolerance);

  // bool lidar_is_inverted = (std::cos(roll) * std::cos(pitch)) < 0.0;
  // const double tolerance = 0.1; // ~5.7 độ
  // bool lidar_is_inverted = (std::cos(roll) * std::cos(pitch)) < -std::sin(tolerance);

  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    if (msg->ranges[i] >= msg->range_min && msg->ranges[i] <= msg->range_max) {
      float x_laser = msg->ranges[i] * std::cos(angle);
      float y_laser = -msg->ranges[i] * std::sin(angle);

      geometry_msgs::msg::PointStamped point_laser;
      point_laser.header.frame_id = laser_frame_;
      point_laser.header.stamp = msg->header.stamp;
      point_laser.point.x = x_laser;
      point_laser.point.y = y_laser;
      point_laser.point.z = 0.0;

      geometry_msgs::msg::PointStamped point_base;
      tf2::doTransform(point_laser, point_base, transformStamped);

      float x = static_cast<float>(point_base.point.x / map_msg_.info.resolution);
      float y = static_cast<float>(point_base.point.y / map_msg_.info.resolution);

      // if (lidar_is_inverted) {
      //   x = -x;
      //   y = -y;
      // }

      scan_points_.push_back(cv::Point2f(x, y));
    }
    angle += msg->angle_increment;
  }

  if (scan_count_ == 0) scan_count_++;

  // ---- Vong lap hill-climbing (giu nguyen logic goc) ----
  while (rclcpp::ok()) {
    if (!map_cropped_.empty()) {
      std::vector<cv::Point2f> transform_points, clockwise_points, counter_points;

      int max_sum = 0;
      float best_dx = 0, best_dy = 0, best_dyaw = 0;

      for (const auto & point : scan_points_) {
        float rotated_x = point.x * std::cos(lidar_yaw_) - point.y * std::sin(lidar_yaw_);
        float rotated_y = point.x * std::sin(lidar_yaw_) + point.y * std::cos(lidar_yaw_);
        transform_points.push_back(cv::Point2f(rotated_x + lidar_x_, lidar_y_ - rotated_y));

        float clockwise_yaw = lidar_yaw_ + deg_to_rad_;
        rotated_x = point.x * std::cos(clockwise_yaw) - point.y * std::sin(clockwise_yaw);
        rotated_y = point.x * std::sin(clockwise_yaw) + point.y * std::cos(clockwise_yaw);
        clockwise_points.push_back(cv::Point2f(rotated_x + lidar_x_, lidar_y_ - rotated_y));

        float counter_yaw = lidar_yaw_ - deg_to_rad_;
        rotated_x = point.x * std::cos(counter_yaw) - point.y * std::sin(counter_yaw);
        rotated_y = point.x * std::sin(counter_yaw) + point.y * std::cos(counter_yaw);
        counter_points.push_back(cv::Point2f(rotated_x + lidar_x_, lidar_y_ - rotated_y));
      }

      std::vector<cv::Point2f> offsets = {{0, 0}, {1, 0}, {-1, 0}, {0, 1}, {0, -1}};
      std::vector<std::vector<cv::Point2f>> point_sets =
        {transform_points, clockwise_points, counter_points};
      std::vector<float> yaw_offsets = {0, deg_to_rad_, -deg_to_rad_};

      for (size_t i = 0; i < offsets.size(); ++i) {
        for (size_t j = 0; j < point_sets.size(); ++j) {
          int sum = 0;
          for (const auto & point : point_sets[j]) {
            float px = point.x + offsets[i].x;
            float py = point.y + offsets[i].y;
            if (px >= 0 && px < map_temp_.cols && py >= 0 && py < map_temp_.rows) {
              sum += map_temp_.at<uchar>(static_cast<int>(py), static_cast<int>(px));
            }
          }
          if (sum > max_sum) {
            max_sum = sum;
            best_dx = offsets[i].x;
            best_dy = offsets[i].y;
            best_dyaw = yaw_offsets[j];
          }
        }
      }

      lidar_x_ += best_dx;
      lidar_y_ += best_dy;
      lidar_yaw_ += best_dyaw;

      if (check(lidar_x_, lidar_y_, lidar_yaw_)) {
        break;
      }
    } else {
      break;
    }
  }

  if (clear_countdown_ > -1) clear_countdown_--;
  if (clear_countdown_ == 0) {
    callClearCostmaps();
  }
}

// ============================================================
// callClearCostmaps
// ============================================================
void LidarLoc::callClearCostmaps()
{
  if (!clear_costmaps_client_->service_is_ready()) {
    RCLCPP_WARN(this->get_logger(), "Service clear_costmaps chua san sang, bo qua lan nay");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  clear_costmaps_client_->async_send_request(request);
}

// ============================================================
// poseTf - tinh va publish TF map -> odom
// ============================================================
void LidarLoc::poseTf()
{
  if (scan_count_ == 0) return;
  if (map_cropped_.empty() || map_msg_.data.empty() || map_msg_.info.resolution <= 0) return;

  double full_map_pixel_x = lidar_x_ + map_roi_info_.x_offset;
  double full_map_pixel_y = lidar_y_ + map_roi_info_.y_offset;

  double x_in_map_frame = full_map_pixel_x * map_msg_.info.resolution
    + map_msg_.info.origin.position.x;
  double y_in_map_frame = full_map_pixel_y * map_msg_.info.resolution
    + map_msg_.info.origin.position.y;

  double yaw_in_map_frame = -lidar_yaw_;

  tf2::Transform map_to_base;
  map_to_base.setOrigin(tf2::Vector3(x_in_map_frame, y_in_map_frame, 0.0));
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw_in_map_frame);
  map_to_base.setRotation(q);

  geometry_msgs::msg::TransformStamped odom_to_base_msg;
  try {
    odom_to_base_msg = tf_buffer_->lookupTransform(
      odom_frame_, base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(),
      "Khong the lay transform tu '%s' sang '%s': %s",
      odom_frame_.c_str(), base_frame_.c_str(), ex.what());
    return;
  }

  tf2::Transform odom_to_base_tf2;
  tf2::fromMsg(odom_to_base_msg.transform, odom_to_base_tf2);
  tf2::Transform map_to_odom = map_to_base * odom_to_base_tf2.inverse();

  geometry_msgs::msg::TransformStamped map_to_odom_msg;
  map_to_odom_msg.header.stamp = this->get_clock()->now();
  map_to_odom_msg.header.frame_id = "map";
  map_to_odom_msg.child_frame_id = odom_frame_;
  map_to_odom_msg.transform = tf2::toMsg(map_to_odom);

  tf_broadcaster_->sendTransform(map_to_odom_msg);
}
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarLoc>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}