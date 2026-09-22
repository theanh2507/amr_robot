// plicp_map_matcher.cpp
//
// Dinh vi robot bang cach so khop SCAN LIDAR voi BAN DO TINH (occupancy grid),
// dung thuat toan Point-to-Line ICP (PLICP) cua Andrea Censi thong qua thu vien
// "csm". Phoi hop voi AMCL (pkg co san): AMCL cung cap pose uoc luong ban dau
// (initial guess), node nay tinh chinh lai bang PLICP roi:
//   1) xuat bien doi tf map -> odom
//   2) phan hoi ket qua ve AMCL qua topic /initialpose
//
// QUAN TRONG - day la so khop SCAN-VOI-MAP, KHONG PHAI scan-voi-scan (khac
// han rf2o_laser_odometry / laser_scan_matcher goc, von so scan hien tai voi
// scan NGAY TRUOC DO nen sai so tich luy theo thoi gian). Ky thuat dung o day:
// vi thu vien csm chi biet so khop hai "scan dang cuc, co thu tu goc" voi nhau
// (khong biet so khop truc tiep voi occupancy grid), ta tu dung mot "MAP CLOUD"
// bang cach RAYCAST tren ban do tai pose uoc luong hien tai, theo DUNG cung goc
// voi tung tia lidar that - MAP CLOUD nay duoc SINH LAI HOAN TOAN moi lan xu ly
// mot scan moi (khong luu lai scan cu nao ca), nen ICP luon neo vao ban do tuyet
// doi, khong bi troi theo thoi gian.
//
// Cong thuc (1)-(4) trong bai bao lien quan (Vasiljevic et al., "High-accuracy
// vehicle localization for autonomous warehousing") duoc ap dung o buoc loc
// nhieu (so sanh khoang cach do that va khoang cach raycast tu ban do, loai bo
// tia roi vao nguoi di lai / vat the dong khong co trong ban do tinh).

#include <cmath>
#include <cstring>
#include <mutex>
#include <memory>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Dense>

#include <csm/csm.h>

#undef min
#undef max

namespace plicp_map_matcher
{

// ============================================================================
// Pose2D - tien ich bien doi 2D toi gian (x, y, theta), dung phep hop bien doi
// thuan nhat de tinh map->odom va cong don do lech tu csm vao pose_guess.
// ============================================================================
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};

  Pose2D() = default;
  Pose2D(double x_, double y_, double theta_)
  : x(x_), y(y_), theta(theta_) {}

  Eigen::Matrix3d toMatrix() const
  {
    Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    m(0, 0) = c;  m(0, 1) = -s;  m(0, 2) = x;
    m(1, 0) = s;  m(1, 1) = c;   m(1, 2) = y;
    return m;
  }

  static Pose2D fromMatrix(const Eigen::Matrix3d & m)
  {
    Pose2D p;
    p.x = m(0, 2);
    p.y = m(1, 2);
    p.theta = std::atan2(m(1, 0), m(0, 0));
    return p;
  }

  Pose2D inverse() const {return fromMatrix(toMatrix().inverse());}

  // this ⊕ other (ap dung other truoc, roi den this)
  Pose2D compose(const Pose2D & other) const
  {
    return fromMatrix(toMatrix() * other.toMatrix());
  }
};

// ============================================================================
// Node chinh
// ============================================================================
class PlicpMapMatcherNode : public rclcpp::Node
{
public:
  PlicpMapMatcherNode()
  : Node("plicp_map_matcher")
  {
    declareParameters();
    readParameters();

    last_amcl_pose_time_ = now();
    last_feedback_time_ = now();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&PlicpMapMatcherNode::mapCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PlicpMapMatcherNode::odomCallback, this, std::placeholders::_1));

    initialpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 1,
      std::bind(&PlicpMapMatcherNode::initialPoseCallback, this, std::placeholders::_1));

    amcl_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      amcl_pose_topic_, 10,
      std::bind(&PlicpMapMatcherNode::amclPoseCallback, this, std::placeholders::_1));

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PlicpMapMatcherNode::scanCallback, this, std::placeholders::_1));

    amcl_feedback_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 1);

    RCLCPP_INFO(get_logger(), "plicp_map_matcher (backend csm) da khoi dong.");
  }

private:
  // --------------------------------------------------------------------
  // Tham so
  // --------------------------------------------------------------------

  // khai bao va gan cac gia tri mac dinh cho file yaml (file yaml ghi de value cua param)
  void declareParameters()
  {
    declare_parameter<std::string>("scan_topic", "/scan_filtered");
    declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    declare_parameter<std::string>("map_topic", "map");
    declare_parameter<std::string>("amcl_pose_topic", "amcl_pose");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_footprint");
    declare_parameter<std::string>("laser_frame", "Lidar_Link");

    declare_parameter<double>("outlier_eps_dist", 0.30);
    declare_parameter<int>("min_valid_rays", 20);

    declare_parameter<int>("csm.max_iterations", 20);
    declare_parameter<double>("csm.epsilon_xy", 1e-5);
    declare_parameter<double>("csm.epsilon_theta", 1e-5);
    declare_parameter<double>("csm.max_correspondence_dist", 0.3);
    declare_parameter<double>("csm.max_angular_correction_deg", 45.0);
    declare_parameter<double>("csm.max_linear_correction", 0.5);
    declare_parameter<double>("csm.sigma", 0.01);
    declare_parameter<int>("csm.orientation_neighbourhood", 20);
    declare_parameter<double>("csm.outliers_maxPerc", 0.90);

    declare_parameter<double>("amcl_pose_timeout", 2.0);
    declare_parameter<double>("amcl_feedback_period", 1.0);
    declare_parameter<double>("amcl_feedback_stddev_xy", 0.05);
    declare_parameter<double>("amcl_feedback_stddev_yaw", 0.02);
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
    laser_frame_ = get_parameter("laser_frame").as_string();

    outlier_eps_dist_ = get_parameter("outlier_eps_dist").as_double();
    min_valid_rays_ = get_parameter("min_valid_rays").as_int();

    csm_max_iterations_ = get_parameter("csm.max_iterations").as_int();
    csm_epsilon_xy_ = get_parameter("csm.epsilon_xy").as_double();
    csm_epsilon_theta_ = get_parameter("csm.epsilon_theta").as_double();
    csm_max_correspondence_dist_ = get_parameter("csm.max_correspondence_dist").as_double();
    csm_max_angular_correction_deg_ = get_parameter("csm.max_angular_correction_deg").as_double();
    csm_max_linear_correction_ = get_parameter("csm.max_linear_correction").as_double();
    csm_sigma_ = get_parameter("csm.sigma").as_double();
    csm_orientation_neighbourhood_ = get_parameter("csm.orientation_neighbourhood").as_int();
    csm_outliers_maxPerc_ = get_parameter("csm.outliers_maxPerc").as_double();

    amcl_pose_timeout_ = get_parameter("amcl_pose_timeout").as_double();
    amcl_feedback_period_ = get_parameter("amcl_feedback_period").as_double();
    amcl_feedback_stddev_xy_ = get_parameter("amcl_feedback_stddev_xy").as_double();
    amcl_feedback_stddev_yaw_ = get_parameter("amcl_feedback_stddev_yaw").as_double();
  }

  // --------------------------------------------------------------------
  // Callbacks
  // --------------------------------------------------------------------
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    map_ = *msg;
    has_map_ = true;
    RCLCPP_INFO(
      get_logger(), "Da nhan ban do: %ux%u, resolution=%.3f m",
      msg->info.width, msg->info.height, msg->info.resolution);
  }

  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (ignore_next_initialpose_) {
      ignore_next_initialpose_ = false;
      return;
    }
    estimated_pose_.x = msg->pose.pose.position.x;
    estimated_pose_.y = msg->pose.pose.position.y;
    estimated_pose_.theta = tf2::getYaw(msg->pose.pose.orientation);
    has_initial_pose_ = true;
    has_amcl_pose_ = false;
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

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    Pose2D guess;                           // pose o global
    Pose2D odom_pose;
    nav_msgs::msg::OccupancyGrid map_copy;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_map_ || !has_initial_pose_ || !has_odom_) {
        return;
      }

      if (!ensureLaserTf()) return;

      const bool amcl_fresh =
        has_amcl_pose_ && (now() - last_amcl_pose_time_).seconds() < amcl_pose_timeout_;
      guess = amcl_fresh ? amcl_pose_ : estimated_pose_;                    // he global (map->odom)
      odom_pose = latest_odom_;
      map_copy = map_;  // ban do it doi, copy nong tranh giu lock lau


      RCLCPP_INFO(get_logger(),
      "guess: x=%.3f y=%.3f theta=%.3f | amcl_fresh=%d | amcl_pose: x=%.3f y=%.3f",
      guess.x, guess.y, guess.theta, amcl_fresh, amcl_pose_.x, amcl_pose_.y);
    }

    const int n = static_cast<int>(msg->ranges.size());

    // ====================================================================
    // Buoc 1 (cong thuc 1-4): xay LIDAR CLOUD (dang cuc) + loc nhieu bang
    // so sanh khoang cach that (dm) voi khoang cach "ao" raycast tu ban do
    // (dv) tai pose_guess hien tai - loai tia lech qua nguong (nguoi di lai,
    // vat the dong khong co trong ban do tinh).
    // ====================================================================

    // map_cloud_ranges, map_cloud_valid: tap dich Q
    // lidar_angles, lidar_valid: tap nguon P
    std::vector<double> lidar_angles(n), lidar_ranges(n), map_cloud_ranges(n);

    // khoi tao vecto co n phan tu, moi phan tu trong vec to co gia tri 0 hoac 1 (dung de valid tap du lieu P,Q)
    // (1: du lieu co do tin cay; 0: du lieu ko tin cay, loai bo ko dung de so khop)
    std::vector<uint8_t> lidar_valid(n, 0), map_cloud_valid(n, 1);

    int n_valid = 0;
    double angle = msg->angle_min;

    Pose2D T_base_laser(laser_x_offset_, laser_y_offset_, laser_yaw_offset_);
    Pose2D laser_pose_map = guess.compose(T_base_laser);

    // giu nguyen du lieu tap P o he local, bien doi du lieu tap Q tu global sang local 
    for (int i = 0; i < n; ++i, angle += msg->angle_increment) {
      lidar_angles[i] = angle;
      const double dm = msg->ranges[i];
      lidar_ranges[i] = dm;

      // MAP CLOUD: khoang cach ao tai DUNG goc nay, xuat phat tu pose_guess
      // sinh MOI LAN GOI scanCallback (khong luu lai gi tu lan truoc).
      // const double dv = raycast(map_copy, guess, guess.theta + angle, msg->range_max);
      // const double dv = raycast(map_copy, guess, guess.theta + laser_yaw_offset_ + angle, msg->range_max);

      const double ray_global_angle = laser_pose_map.theta + angle;
      const double dv = raycast(map_copy, laser_pose_map, ray_global_angle, msg->range_max);


      // xet vi tri uoc luong cua lidar trong map
      // const double laser_x_map = guess.x + laser_x_offset_ * std::cos(guess.theta) - laser_y_offset_ * std::sin(guess.theta);
      // const double laser_y_map = guess.y + laser_x_offset_ * std::sin(guess.theta) + laser_y_offset_ * std::cos(guess.theta);
      // Pose2D laser_pose_map(laser_x_map, laser_y_map, guess.theta);
      // const double dv = raycast(map_copy, laser_pose_map, guess.theta + laser_yaw_offset_ + angle, msg->range_max);

      if (i < 3) {
        RCLCPP_INFO(get_logger(), "tia %d: angle=%.3f dm=%.3f dv=%.3f", i, angle, dm, dv);
      }

      // he toa do local
      map_cloud_ranges[i] = dv;           // thu duoc toa do dam may diem map, dua tren pose du doan khi so khop occupancy cua map
      if (dv >= msg->range_max) {
        map_cloud_valid[i] = 0;
      }

      if (!std::isfinite(dm) || dm < msg->range_min || dm > msg->range_max) {
        continue;
      }
      if (std::abs(dm - dv) > outlier_eps_dist_) {
        continue;  // nguoi di lai / vat the dong - khong co trong ban do tinh
      }
      lidar_valid[i] = 1;
      ++n_valid;
    }

    // neu so luong luong tia laser hop le it hon so luong tia laser toi thieu can so khop thi gui lai tf cua pose amcl
    if (n_valid < min_valid_rays_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Qua it tia hop le sau loc nhieu (%d) - dua vao AMCL/odometry.", n_valid);
      publishTf(guess, odom_pose, msg->header.stamp);
      return;
    }

    // ====================================================================
    // Buoc 2: dong goi thanh LDP va chay PLICP that su qua csm::sm_icp()
    // ====================================================================
    LDP map_cloud_ldp = buildLdp(lidar_angles, map_cloud_ranges, map_cloud_valid);
    LDP lidar_cloud_ldp = buildLdp(lidar_angles, lidar_ranges, lidar_valid);

    sm_params input;
    std::memset(&input, 0, sizeof(input));
    sm_result output;
    std::memset(&output, 0, sizeof(output));

    input.laser_ref = map_cloud_ldp;      // THAM CHIEU la BAN DO   (dv)
    input.laser_sens = lidar_cloud_ldp;   // DU LIEU dang xet la LIDAR (dm)

    // do lech uoc luong ban dau giua laser_ref va laser_sens (do map_cloud va lidar_cloud deu duoc xay dung dua tren 1 pose gia dinh guess)
    input.first_guess[0] = 0.0;           // deltaX
    input.first_guess[1] = 0.0;           // deltaY
    input.first_guess[2] = 0.0;           // delta theta

    // input.min_reading = 0.0;
    input.min_reading = msg->range_min;                                           // khoang cach do nho nhat de csm chap nhan lam input
    input.max_reading = msg->range_max;                                           // khoang cach do lon nhat de csm chap nhan lam input
    input.max_angular_correction_deg = csm_max_angular_correction_deg_;           // do xoay toi da csm cho phep tim so khop (!!!)
    input.max_linear_correction = csm_max_linear_correction_;                     // khoang cach di chuyen toi da giua 2 lan so khop (!!!)
    input.max_iterations = csm_max_iterations_;                                   // so vong lap pl-icp 
    input.epsilon_xy = csm_epsilon_xy_;                                           // dieu kien dung thuat toan phuong tinh tien (!!!)
    input.epsilon_theta = csm_epsilon_theta_;                                     // dieu kien dung thuat toan goc xoay (!!!)
    input.max_correspondence_dist = csm_max_correspondence_dist_;                 // nguong xet 2 diem duoc coi la khop nhau (correspondence) (!!!)
    input.sigma = csm_sigma_;                                                     // do nhieu (uncertainly) cua measurament (sigma nho: tin tuong vao gia tri quet cua lidar)
    input.use_corr_tricks = 1;                                                    // use smart tricks for finding correspondences
    input.restart = 0;                                                            // cho phep thuat toan restart trong 1 so truong hop khong hoi tu (1: restart, 0: khong restart)
    input.clustering_threshold = 0.25;                                            // khoang cach toi da de coi la 1 cluster (cac diem scan gan nhau trong khoang 0.25m thi coi la 1 cum)
    input.orientation_neighbourhood = csm_orientation_neighbourhood_;             // so luong cac diem laser lan can dung de xac dinh huong phap tuyen
    input.use_point_to_line_distance = 1;                                         // bat dung PLICP (Censi)
    input.do_alpha_test = 0;  
    input.outliers_maxPerc = csm_outliers_maxPerc_;                               // ty le outlier toi da co the loai bo (!!!)
    input.outliers_adaptive_order = 0.7;                                          
    input.outliers_adaptive_mult = 2.0;
    input.do_visibility_test = 0;
    input.outliers_remove_doubles = 1;                                            // 1: loai bo cac correspondence trung lap
    input.do_compute_covariance = 0;                                              // If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov
    input.debug_verify_tricks = 0;
    input.use_ml_weights = 0;
    input.use_sigma_weights = 0;

    sm_icp(&input, &output);

    Pose2D final_pose = guess;
    if (output.valid)                                                             // kiem tra xem thuat toan co hoi tu ko
    {
      // luong hieu chinh
      // const Pose2D correction(output.x[0], output.x[1], output.x[2]);             // tra ve do lech trong he toa do local (delta x, delta y, delta theta)
      // // thuc hien nhan 2 ma tran 3x3 voi nhau (T_final = T_guess * T_delta)
      // final_pose = guess.compose(correction);                                     
      // {
      //   std::lock_guard<std::mutex> lock(mutex_);
      //   estimated_pose_ = final_pose;
      // }

    const Pose2D laser_correction(output.x[0], output.x[1], output.x[2]);
    
    // Update pose của Laser trong Map
    Pose2D final_laser_pose = laser_pose_map.compose(laser_correction);
    
    // Suy ra ngược lại pose của base_footprint trong Map:
    // T_map_base = T_map_laser ⊕ (T_base_laser)^(-1)
    final_pose = final_laser_pose.compose(T_base_laser.inverse());

    std::lock_guard<std::mutex> lock(mutex_);
    estimated_pose_ = final_pose;

      RCLCPP_DEBUG(
        get_logger(), "PLICP(csm) hoi tu sau %d vong lap, %d tia hop le",
        output.iterations, n_valid);

      // gui nguoc lai pose robot cho amcl
      maybeSendFeedbackToAmcl(final_pose, msg->header.stamp);
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "PLICP(csm) khong hoi tu (%d tia hop le) - dung uoc luong AMCL/odometry.", n_valid);
    }

    ld_free(map_cloud_ldp);
    ld_free(lidar_cloud_ldp);

    publishTf(final_pose, odom_pose, msg->header.stamp);
  }

  // --------------------------------------------------------------------
  // Tien ich ban do: raycast + doi chi so o <-> toa do met
  // --------------------------------------------------------------------
  static bool worldToGrid(
    const nav_msgs::msg::OccupancyGrid & map, double wx, double wy, int & gx, int & gy)
  {
    const double res = map.info.resolution;
    gx = static_cast<int>(std::floor((wx - map.info.origin.position.x) / res));
    gy = static_cast<int>(std::floor((wy - map.info.origin.position.y) / res));
    return gx >= 0 && gy >= 0 &&
           gx < static_cast<int>(map.info.width) &&
           gy < static_cast<int>(map.info.height);
  }

  static bool isOccupied(const nav_msgs::msg::OccupancyGrid & map, int gx, int gy)
  {
    if (gx < 0 || gy < 0 || gx >= static_cast<int>(map.info.width) ||
      gy >= static_cast<int>(map.info.height))
    {
      return false;
    }
    const int idx = gy * static_cast<int>(map.info.width) + gx;
    return map.data[idx] >= 50;
  }

  // Do doc theo tia tu pose, huong global_angle, tra ve khoang cach toi o
  // chiem dung dau tien tren ban do (hoac max_range neu khong gap gi).

  // lay vi tri du guess nam o local -> tinh goc chieu ban do global_angle = guess.theta + angle_local
  // ban raycast trong he global de tim o vat can
  static double raycast(
    const nav_msgs::msg::OccupancyGrid & map,
    const Pose2D & pose, double global_angle, double max_range)
  {
    const double res = map.info.resolution;
    const double step = std::max(res * 0.5, 0.01);
    const double dx = std::cos(global_angle);
    const double dy = std::sin(global_angle);

    for (double t = 0.0; t <= max_range; t += step) {
      int gx, gy;
      if (!worldToGrid(map, pose.x + t * dx, pose.y + t * dy, gx, gy)) {
        return max_range;
      }
      if (isOccupied(map, gx, gy)) {
        return t;
      }
    }
    return max_range;
  }

  // --------------------------------------------------------------------
  // csm: dong goi LDP tu 3 mang song song (angles/ranges/valid)
  // angles: cho biet tia thu i chi huong nao
  // ranges: tia thu i do duoc khoang cach bao xa
  // valid: kiem tra xem tia thu i co tin cay ko 
  // --------------------------------------------------------------------
  static LDP buildLdp(
    const std::vector<double> & angles,
    const std::vector<double> & ranges,
    const std::vector<uint8_t> & valid_mask)
  {
    const int n = static_cast<int>(angles.size());
    LDP ld = ld_alloc_new(n);
    for (int i = 0; i < n; ++i) {
      ld->theta[i] = angles[i];
      const bool ok = valid_mask[i] != 0 && std::isfinite(ranges[i]) && ranges[i] > 0.0;
      ld->valid[i] = ok ? 1 : 0;
      ld->readings[i] = ok ? ranges[i] : -1.0;
      ld->cluster[i] = -1;
    }
    ld->min_theta = angles.front();
    ld->max_theta = angles.back();
    for (int k = 0; k < 3; ++k) {
      ld->odometry[k] = 0.0;
      ld->estimate[k] = 0.0;
      ld->true_pose[k] = 0.0;
    }
    return ld;
  }

  // --------------------------------------------------------------------
  // Xuat tf va phan hoi AMCL
  // --------------------------------------------------------------------
  void publishTf(const Pose2D & map_to_base, const Pose2D & odom_to_base, const rclcpp::Time & stamp)
  {
    const Pose2D map_to_odom = map_to_base.compose(odom_to_base.inverse());

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = map_frame_;
    tf_msg.child_frame_id = odom_frame_;
    tf_msg.transform.translation.x = map_to_odom.x;
    tf_msg.transform.translation.y = map_to_odom.y;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, map_to_odom.theta);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf_msg);
  }

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

    for (auto & v : msg.pose.covariance) {v = 0.0;}
    msg.pose.covariance[0] = amcl_feedback_stddev_xy_ * amcl_feedback_stddev_xy_;
    msg.pose.covariance[7] = amcl_feedback_stddev_xy_ * amcl_feedback_stddev_xy_;
    msg.pose.covariance[35] = amcl_feedback_stddev_yaw_ * amcl_feedback_stddev_yaw_;

    ignore_next_initialpose_ = true;
    amcl_feedback_pub_->publish(msg);
  }

bool ensureLaserTf()
{
  if (has_laser_tf_) return true;
  try {
    auto tf = tf_buffer_->lookupTransform(base_frame_, laser_frame_, tf2::TimePointZero);
    laser_x_offset_ = tf.transform.translation.x;
    laser_y_offset_ = tf.transform.translation.y;
    laser_yaw_offset_ = tf2::getYaw(tf.transform.rotation);

    // RCLCPP_INFO(get_logger(), "laser_yaw_offset_%f", laser_yaw_offset_);
    has_laser_tf_ = true;
    RCLCPP_INFO(get_logger(), "Da lay TF %s->%s: yaw_offset=%.1f deg",
      base_frame_.c_str(), laser_frame_.c_str(), laser_yaw_offset_ * 180.0 / M_PI);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Chua lay duoc TF %s->%s: %s", base_frame_.c_str(), laser_frame_.c_str(), ex.what());
    return false;
  }
}

  // --------------------------------------------------------------------
  // Thanh vien
  // --------------------------------------------------------------------
  std::string scan_topic_, odom_topic_, map_topic_, amcl_pose_topic_;
  std::string map_frame_, odom_frame_, base_frame_;


  // gan gia tri mac dinh 
  double outlier_eps_dist_{0.5};
  int min_valid_rays_{20};

  int csm_max_iterations_{25};
  double csm_epsilon_xy_{1e-2};
  double csm_epsilon_theta_{1e-1};
  double csm_max_correspondence_dist_{1.0};
  double csm_max_angular_correction_deg_{45.0};
  double csm_max_linear_correction_{0.8};
  double csm_sigma_{0.1};
  int csm_orientation_neighbourhood_{20};
  double csm_outliers_maxPerc_{0.70};

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

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  double laser_yaw_offset_{0.0};
  double laser_x_offset_{0.0}, laser_y_offset_{0.0};
  std::string laser_frame_;

  std::mutex mutex_;
  nav_msgs::msg::OccupancyGrid map_;
  bool has_map_{false};

  Pose2D estimated_pose_;
  Pose2D amcl_pose_;
  Pose2D last_odom_;
  Pose2D latest_odom_;
  bool has_initial_pose_{false};
  bool has_amcl_pose_{false};
  bool has_last_odom_{false};
  bool has_odom_{false};
  bool ignore_next_initialpose_{false};
  bool has_laser_tf_{false};
  rclcpp::Time last_amcl_pose_time_;
  rclcpp::Time last_feedback_time_;
};

}  // namespace plicp_map_matcher

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<plicp_map_matcher::PlicpMapMatcherNode>());
  rclcpp::shutdown();
  return 0;
}