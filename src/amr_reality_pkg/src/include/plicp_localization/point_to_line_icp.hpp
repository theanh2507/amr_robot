#pragma once

#include <cmath>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>

namespace plicp_localization
{

/// Pose 2D (x, y, theta) trên mặt phẳng.

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
    return Pose2D(m(0, 2), m(1, 2), std::atan2(m(1, 0), m(0, 0)));
  }

  // TỐI ƯU 1: Tính trực tiếp T^-1 = [R^T | -R^T * t]
  // Không cần tạo Matrix3d và không cần gọi .inverse()
  Pose2D inverse() const
  {
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    const double inv_x = -(c * x + s * y);
    const double inv_y = -(-s * x + c * y);
    return Pose2D(inv_x, inv_y, -theta);
  }

  // TỐI ƯU 2: Tính trực tiếp T1 * T2
  // R_res = R1 * R2 (cộng góc), t_res = t1 + R1 * t2
  Pose2D compose(const Pose2D & other) const
  {
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    
    const double res_x = x + (c * other.x - s * other.y);
    const double res_y = y + (s * other.x + c * other.y);
    
    // Chuẩn hóa góc về [-pi, pi]
    double res_theta = theta + other.theta;
    res_theta = std::atan2(std::sin(res_theta), std::cos(res_theta));

    return Pose2D(res_x, res_y, res_theta);
  }
};

// struct Pose2D
// {
//   double x{0.0};
//   double y{0.0};
//   double theta{0.0};

//   Pose2D() = default;
//   Pose2D(double x_, double y_, double theta_)
//   : x(x_), y(y_), theta(theta_) {}

//   // chuyen vecto trang thai (x, y, theta) thanh ma tran dong nhat 3x3
//   Eigen::Matrix3d toMatrix() const
//   {
//     Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
//     const double c = std::cos(theta);
//     const double s = std::sin(theta);
//     m(0, 0) = c;  m(0, 1) = -s;  m(0, 2) = x;
//     m(1, 0) = s;  m(1, 1) = c;   m(1, 2) = y;
//     return m;
//   }

//   // trich xuat lai pose tu ma tran
//   static Pose2D fromMatrix(const Eigen::Matrix3d & m)
//   {
//     return Pose2D(m(0, 2), m(1, 2), std::atan2(m(1, 0), m(0, 0)));
//   }

//   Pose2D inverse() const
//   {
//     return fromMatrix(toMatrix().inverse());
//   }

//   /// this ⊕ other  (áp dụng other trước, rồi đến this — phép hợp biến đổi thuần nhất 2D)
//   Pose2D compose(const Pose2D & other) const
//   {
//     return fromMatrix(toMatrix() * other.toMatrix());
//   }
// };

inline double normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

struct ICPResult
{
  Pose2D pose;
  bool converged{false};
  int iterations{0};
  int num_inliers{0};
  double mean_sq_error{0.0};
};

struct PointToLineICPParams
{
  int max_iterations{30};         // so vong lap toi da
  double eps_dist_start{0.5};     // ngưỡng khoảng cách tương ứng ban đầu (m)
  double eps_dist_end{0.05};      // ngưỡng cuối cùng, giảm dần mỗi vòng lặp (giống công thức (4))
  double translation_eps{1e-4};   // điều kiện hội tụ - tịnh tiến (m)
  double rotation_eps{1e-5};      // điều kiện hội tụ - góc quay (rad)
  int min_inliers{30};            // số điểm hợp lệ tối thiểu để coi là so khớp thành công
  double damping{1e-9};           // ổn định số học cho ma trận Hessian
};

/// Giao diện mô hình bản đồ tham chiếu mà PLICP sẽ so khớp vào.
/// Tách rời thuật toán ICP thuần túy khỏi kiểu dữ liệu bản đồ cụ thể
/// (occupancy grid, point cloud, ...) để dễ tái sử dụng / kiểm thử.
class MapModel
{
public:
  virtual ~MapModel() = default;

  /// Tìm điểm trên bản đồ gần nhất với query_point (hệ tọa độ map) và ước lượng
  /// phương pháp tuyến cục bộ (normal) tại đó, dùng cho metric point-to-line.
  /// Trả về false nếu không có điểm tương ứng nào trong phạm vi max_dist.
  virtual bool findCorrespondence(
    const Eigen::Vector2d & query_point,
    double max_dist,
    Eigen::Vector2d & corr_point,
    Eigen::Vector2d & corr_normal) const = 0;
};

/// Thuật toán so khớp Point-to-Line ICP (PLICP, Censi 2008), dùng làm bước
/// tinh chỉnh sau AMCL đúng như kiến trúc trong bài báo (AMCL -> ICP -> ...).
class PointToLineICP
{
public:

  using Params = PointToLineICPParams;
  explicit PointToLineICP(const Params & params = Params());

  /// points        : đám mây điểm quét laser trong hệ tọa độ robot (base_link), đã lọc nhiễu.
  /// map_model     : mô hình bản đồ dùng để tìm điểm tương ứng.
  /// initial_guess : ước lượng pose ban đầu của robot trong hệ tọa độ map (thường lấy từ AMCL).
  ICPResult align(
    const std::vector<Eigen::Vector2d> & points,
    const MapModel & map_model,
    const Pose2D & initial_guess) const;

private:
  Params params_;
};

}  // namespace plicp_localization
