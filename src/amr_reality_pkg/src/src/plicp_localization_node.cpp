#include <mutex>
#include <memory>
#include <cmath>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "plicp_localization/point_to_line_icp.hpp"
#include "plicp_localization/occupancy_grid_map_model.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace plicp_localization
{

class PlicpLocalizationNode : public rclcpp::Node
{
public:
  PlicpLocalizationNode()
  : Node("plicp_localization_node")
  {
    declareParameters();
    readParameters();

    icp_ = std::make_unique<PointToLineICP>(icp_params_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    

    last_amcl_pose_time_ = now();
    last_feedback_time_ = now();

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&PlicpLocalizationNode::mapCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PlicpLocalizationNode::odomCallback, this, std::placeholders::_1));

    initialpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 1,
      std::bind(&PlicpLocalizationNode::initialPoseCallback, this, std::placeholders::_1));

    amcl_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      amcl_pose_topic_, 10,
      std::bind(&PlicpLocalizationNode::amclPoseCallback, this, std::placeholders::_1));

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PlicpLocalizationNode::scanCallback, this, std::placeholders::_1));

    amcl_feedback_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 1);

    RCLCPP_INFO(get_logger(), "plicp_localization_node da khoi dong, cho map + initial pose...");
  }

private:
  // ----------------------- Tham so -----------------------
  void declareParameters()
  {
    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    declare_parameter<std::string>("map_topic", "map");
    declare_parameter<std::string>("amcl_pose_topic", "amcl_pose");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_footprint");

    declare_parameter<double>("outlier_filter.eps_dist", 0.30);           // nguong loc nhieu theo cong thuc 4 

    declare_parameter<int>("icp.max_iterations", 30);

    // nguong khoang cach chap nhan 1 cap pose tuong ung
    declare_parameter<double>("icp.eps_dist_start", 0.5);
    declare_parameter<double>("icp.eps_dist_end", 0.05);

    // neu luong dieu chinh giua 2 lan hoi tu nho hon 2 nguong nay thi coi nhu da hoi tu
    declare_parameter<double>("icp.translation_eps", 0.01);
    declare_parameter<double>("icp.rotation_eps", 0.01);

    // so diem tuong ung hop le toi thieu de coi la so khop dang tin, neu ko dung duoc thi dung pose tu amcl thay vi plipc
    declare_parameter<int>("icp.min_inliers", 20);

    // timeout cho amcl, neu amcl bi treo hoac du lieu qua cu thi dung pose uoc luong tu odom
    declare_parameter<double>("amcl_pose_timeout", 2.0);

    // moi 3 giay publish pose tu plicp set lam initial pose cho amcl
    declare_parameter<double>("amcl_feedback_period", 3.0);

    // gia tri hiep phuong sai ma plicp gui cho amcl, gia tri cang nho amcl cang tin tuong vao gia tri plipc publish
    declare_parameter<double>("amcl_feedback_stddev_xy", 0.2);
    declare_parameter<double>("amcl_feedback_stddev_yaw", 0.05);
  }

  void readParameters()
  {
    scan_topic_ = get_parameter("scan_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    map_topic_ = get_parameter("map_topic").as_string();
    amcl_pose_topic_ = get_parameter("amcl_pose_topic").as_string();
    map_frame_ = get_parameter("map_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();

    outlier_eps_dist_ = get_parameter("outlier_filter.eps_dist").as_double();

    icp_params_.max_iterations = get_parameter("icp.max_iterations").as_int();
    icp_params_.eps_dist_start = get_parameter("icp.eps_dist_start").as_double();
    icp_params_.eps_dist_end = get_parameter("icp.eps_dist_end").as_double();
    icp_params_.translation_eps = get_parameter("icp.translation_eps").as_double();
    icp_params_.rotation_eps = get_parameter("icp.rotation_eps").as_double();
    icp_params_.min_inliers = get_parameter("icp.min_inliers").as_int();

    amcl_pose_timeout_ = get_parameter("amcl_pose_timeout").as_double();
    amcl_feedback_period_ = get_parameter("amcl_feedback_period").as_double();
    amcl_feedback_stddev_xy_ = get_parameter("amcl_feedback_stddev_xy").as_double();
    amcl_feedback_stddev_yaw_ = get_parameter("amcl_feedback_stddev_yaw").as_double();
  }

  // ----------------------- Callbacks -----------------------
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    map_model_.setMap(*msg);
    RCLCPP_INFO(
      get_logger(), "Da nhan ban do: %ux%u, resolution=%.3f m",
      msg->info.width, msg->info.height, msg->info.resolution);
  }

  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // Bo qua neu day chinh la message ma node nay vua tu phat ra de phan hoi cho AMCL,
    // tranh vong lap tu-nhan-lai-chinh-minh tren cung mot topic "initialpose".
    if (ignore_next_initialpose_) {
      ignore_next_initialpose_ = false;
      return;
    }

    estimated_pose_.x = msg->pose.pose.position.x;
    estimated_pose_.y = msg->pose.pose.position.y;
    estimated_pose_.theta = tf2::getYaw(msg->pose.pose.orientation);
    has_initial_pose_ = true;
    has_amcl_pose_ = false;     // uu tien pose vua duoc nguoi dung dat lai (vi du tren rviz)

    RCLCPP_INFO(
      get_logger(), "Nhan initial pose: x=%.3f y=%.3f yaw=%.3f",
      estimated_pose_.x, estimated_pose_.y, estimated_pose_.theta);
  }

  void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    amcl_pose_.x = msg->pose.pose.position.x;
    amcl_pose_.y = msg->pose.pose.position.y;
    amcl_pose_.theta = tf2::getYaw(msg->pose.pose.orientation);
    last_amcl_pose_time_ = now();
    has_amcl_pose_ = true;

    if (!has_initial_pose_) {
      estimated_pose_ = amcl_pose_;
      has_initial_pose_ = true;
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Pose2D current_odom;
    current_odom.x = msg->pose.pose.position.x;
    current_odom.y = msg->pose.pose.position.y;
    current_odom.theta = tf2::getYaw(msg->pose.pose.orientation);

    // tinh uoc luong delta thay doi doi giua 2 lan doc odom
    if (has_last_odom_) {
      // Lan truyen uoc luong bang chuyen dong odometry (motion update) - dung khi
      // chua co pose moi tu AMCL hoac PLICP giua hai lan cap nhat.
      const Pose2D delta = last_odom_.inverse().compose(current_odom);
      estimated_pose_ = estimated_pose_.compose(delta);
    }
    last_odom_ = current_odom;
    has_last_odom_ = true;
    latest_odom_ = current_odom;
    has_odom_ = true;
  }

  /// Tra cuu pose odom->base tai DUNG thoi diem cua scan (khong dung gia tri "moi
  /// nhat" nhu latest_odom_, vi /scan va /odom khong dong bo tuyet doi ve thoi gian -
  /// dac biet khi robot xoay nhanh, lech thoi gian nho cung gay sai vi tri lon).
  bool lookupOdomPoseAtTime(const rclcpp::Time & stamp, Pose2D & odom_pose_out)
  {
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      tf_stamped = tf_buffer_->lookupTransform(
        odom_frame_, base_frame_, stamp,
        rclcpp::Duration::from_seconds(0.1));  // cho toi da 100ms neu du lieu chua kip toi
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Khong tra cuu duoc tf %s->%s tai thoi diem scan: %s",
        odom_frame_.c_str(), base_frame_.c_str(), ex.what());
      return false;
    }

    odom_pose_out.x = tf_stamped.transform.translation.x;
    odom_pose_out.y = tf_stamped.transform.translation.y;

    tf2::Quaternion q(
      tf_stamped.transform.rotation.x,
      tf_stamped.transform.rotation.y,
      tf_stamped.transform.rotation.z,
      tf_stamped.transform.rotation.w);
    odom_pose_out.theta = tf2::getYaw(q);
    return true;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    Pose2D guess;
    OccupancyGridMapModel map_model_copy;
    Pose2D odom_pose;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!map_model_.hasMap() || !has_initial_pose_ || !has_odom_) {
        return;
      }

      // Uu tien pose tu AMCL (chay song song, cho uoc luong global-robust, chiu duoc
      // truong hop robot bi "bat coc"/mat dinh vi) neu con "fresh"; neu khong, dung
      // uoc luong da duoc lan truyen tu odometry ke tu lan PLICP/AMCL gan nhat.
      const bool amcl_fresh =
        has_amcl_pose_ &&
        (now() - last_amcl_pose_time_).seconds() < amcl_pose_timeout_;
      guess = amcl_fresh ? amcl_pose_ : estimated_pose_;

      map_model_copy = map_model_;  // ban do it thay doi -> copy nong, khong giu lock lau
      odom_pose = latest_odom_;

      // Pose2D odom_pose;
      // if (!lookupOdomPoseAtTime(msg->header.stamp, odom_pose))      // truyen odom_pose_out vao odom_pose neu lookupOdomPoseAtTime tra ve true (truyen tham chieu)
      // {
      //   std::lock_guard<std::mutex> lock(mutex_);
      //   odom_pose = latest_odom_;
      // }
    }

    // Buoc 1 (cong thuc 1-4): chuyen tia laser sang Descartes + loc nhieu bang
    // so sanh khoang cach that (dm) voi khoang cach "ao" raycast tu ban do (dv).
    // Nhung diem lech qua nguong (nguoi di lai, xe khac, vat the dong...) bi loai bo
    // truoc khi dua vao PLICP.
    const auto points = filterAndConvertScan(*msg, map_model_copy, guess, outlier_eps_dist_);

    if (static_cast<int>(points.size()) < icp_params_.min_inliers) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Qua it diem hop le sau khi loc nhieu (%zu diem) - bo qua scan nay, dua vao AMCL.",
        points.size());
      publishTf(guess, odom_pose, msg->header.stamp);
      return;
    }

    // Buoc 2: chay PLICP de tinh chinh pose (dau vao la ket qua AMCL lam initial guess)
    const ICPResult result = icp_->align(points, map_model_copy, guess);

    Pose2D final_pose = guess;
    if (result.converged) {
      final_pose = result.pose;

      {
        std::lock_guard<std::mutex> lock(mutex_);
        estimated_pose_ = final_pose;
      }

      RCLCPP_DEBUG(
        get_logger(),
        "PLICP hoi tu sau %d vong lap, %d diem hop le, sai so TB^2=%.6f",
        result.iterations, result.num_inliers, result.mean_sq_error);

      // Buoc 4: phan hoi ket qua PLICP nguoc lai cho AMCL (giong co che feedback
      // DFT->AMCL trong bai bao) de AMCL hoi tu chinh xac hon theo thoi gian.
      maybeSendFeedbackToAmcl(final_pose, msg->header.stamp);
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "PLICP khong hoi tu (%d diem hop le) - dung uoc luong tu AMCL/odometry.",
        result.num_inliers);
    }

    // Buoc 3: xuat bien doi tf map->odom tu pose cuoi cung
    publishTf(final_pose, odom_pose, msg->header.stamp);
  }

  // ----------------------- Ham xu ly -----------------------

  static std::vector<Eigen::Vector2d> filterAndConvertScan(
    const sensor_msgs::msg::LaserScan & scan,
    const OccupancyGridMapModel & map_model,
    const Pose2D & pose_guess,
    double eps_dist)
  {
    std::vector<Eigen::Vector2d> points;
    points.reserve(scan.ranges.size());                   // reserve: cap phat bo nho

    double angle = scan.angle_min;
    for (size_t i = 0; i < scan.ranges.size(); ++i, angle += scan.angle_increment) {

      // khoang cach den vat can tu lidar tren robot that
      const double dm = scan.ranges[i];

      if (!std::isfinite(dm) || dm < scan.range_min || dm > scan.range_max) {
        continue;
      }

      // Cong thuc (1): toa do cac diem laser trong he toa do cam bien/robot
      const Eigen::Vector2d p_local(dm * std::cos(angle), dm * std::sin(angle));

      // Cong thuc (2): khoang cach "ao" ky vong tu ban do, voi pose uoc luong hien tai
      const double dv = map_model.raycast(pose_guess, pose_guess.theta + angle, scan.range_max);

      // Cong thuc (4): loai bo diem neu lech qua nhieu so voi ban do tinh
      // (nguoi di lai, xe khac, vat the dong khong co trong ban do tinh)
      if (std::abs(dm - dv) > eps_dist) {
        continue;
      }

      points.push_back(p_local);
    }
    return points;            // chua du lieu toa do cac scan dm thoa man dieu kien loc nhieu
  }

  void publishTf(
    const Pose2D & map_to_base,
    const Pose2D & odom_to_base,
    const rclcpp::Time & stamp)
  {
    // map->odom = (map->base) ⊕ (odom->base)^-1
    const Pose2D map_to_odom = map_to_base.compose(odom_to_base.inverse());

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = map_frame_;
    tf_msg.child_frame_id = odom_frame_;
    tf_msg.transform.translation.x = map_to_odom.x;
    tf_msg.transform.translation.y = map_to_odom.y;
    tf_msg.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, map_to_odom.theta);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf_msg);
  }

  // plicp hoi tu thi gui pose nguoc lai cho amcl
  void maybeSendFeedbackToAmcl(const Pose2D & pose, const rclcpp::Time & stamp)
  {
    if ((now() - last_feedback_time_).seconds() < amcl_feedback_period_) {
      return;
    }
    last_feedback_time_ = now();

    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    msg.pose.pose.position.x = pose.x;
    msg.pose.pose.position.y = pose.y;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose.theta);
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    // Do tin cay cao vao ket qua PLICP (covariance nho hon nhieu so voi AMCL mac dinh)
    // nhung khong ep ve 0 tuyet doi, de tranh AMCL bi "sup" toan bo tap particle.
    for (auto & v : msg.pose.covariance) {v = 0.0;}
    const double var_xy = amcl_feedback_stddev_xy_ * amcl_feedback_stddev_xy_;
    const double var_yaw = amcl_feedback_stddev_yaw_ * amcl_feedback_stddev_yaw_;
    msg.pose.covariance[0] = var_xy;    // xx
    msg.pose.covariance[7] = var_xy;    // yy
    msg.pose.covariance[35] = var_yaw;  // yaw-yaw

    ignore_next_initialpose_ = true;
    amcl_feedback_pub_->publish(msg);
  }

  // ----------------------- Thanh vien -----------------------
  std::string scan_topic_, odom_topic_, map_topic_, amcl_pose_topic_;
  std::string map_frame_, odom_frame_, base_frame_;

  double outlier_eps_dist_{0.3};

  PointToLineICP::Params icp_params_;
  std::unique_ptr<PointToLineICP> icp_;

  double amcl_pose_timeout_{2.0};
  double amcl_feedback_period_{1.0};
  double amcl_feedback_stddev_xy_{0.05};
  double amcl_feedback_stddev_yaw_{0.02};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_feedback_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_ ;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex mutex_;
  OccupancyGridMapModel map_model_;
  Pose2D estimated_pose_;
  Pose2D amcl_pose_;
  Pose2D last_odom_;
  Pose2D latest_odom_;
  bool has_initial_pose_{false};
  bool has_amcl_pose_{false};
  bool has_last_odom_{false};
  bool has_odom_{false};
  bool ignore_next_initialpose_{false};
  rclcpp::Time last_amcl_pose_time_;
  rclcpp::Time last_feedback_time_;
};

}  // namespace plicp_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<plicp_localization::PlicpLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
