#include "plicp_localization/point_to_line_icp.hpp"

namespace plicp_localization
{

PointToLineICP::PointToLineICP(const Params & params)
: params_(params)
{
}

ICPResult PointToLineICP::align(
  const std::vector<Eigen::Vector2d> & points,
  const MapModel & map_model,
  const Pose2D & initial_guess) const
{
  ICPResult result;
  result.pose = initial_guess;

  if (static_cast<int>(points.size()) < params_.min_inliers) {
    result.converged = false;
    return result;
  }

  Pose2D T = initial_guess;

  const double eps_step =
    (params_.eps_dist_start - params_.eps_dist_end) /
    std::max(1, params_.max_iterations - 1);
  double eps_dist = params_.eps_dist_start;

  for (int iter = 0; iter < params_.max_iterations; ++iter) {
    const double c = std::cos(T.theta);
    const double s = std::sin(T.theta);

    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();
    double sq_error_sum = 0.0;
    int inliers = 0;

    for (const auto & p_local : points) {
      // dam may diem laser robot (doi tu hej local sang global)
      // Biến đổi điểm quét (hệ robot) sang hệ map với ước lượng pose hiện tại  (cong thuc 2) (p_map toa do cac diem laser theo he m)
      const Eigen::Vector2d p_map(
        c * p_local.x() - s * p_local.y() + T.x,
        s * p_local.x() + c * p_local.y() + T.y);

      Eigen::Vector2d corr_point;               // diem qi tren map tuong ung gan nhat so voi pi (pi: du lieu do cam bien)
      Eigen::Vector2d normal;                   // vecto phap tuyen don vi normal = [0.0 1.0]
      if (!map_model.findCorrespondence(p_map, eps_dist, corr_point, normal)) {
        continue;
      }

      // Sai số point-to-line: khoảng cách có dấu chiếu lên pháp tuyến cục bộ (tich vo huong giua vecto phap tuyen n )
      const double residual = normal.dot(p_map - corr_point);

      // Jacobian của residual theo [dx, dy, dtheta] (Neu thay doi pose 1 luong nho [dx, dy, theta] thi can biet residual se thay doi 1 luong bao nhieu)
      // dao ham p_map theo theta (p_map = R(theta)*p_local + t)

      const double drot_x = -s * p_local.x() - c * p_local.y();
      const double drot_y = c * p_local.x() - s * p_local.y();

      // residual = n.(p_map - q)
      // J(0), J(1): dao ham residual theo dx, dy
      // J(3): dao ham residual theo dtheta
      Eigen::Vector3d J;
      J(0) = normal.x();
      J(1) = normal.y();
      J(2) = normal.x() * drot_x + normal.y() * drot_y;

      H += J * J.transpose();
      b += J * residual;
      sq_error_sum += residual * residual;
      ++inliers;
    }

    result.iterations = iter + 1;
    result.num_inliers = inliers;

    if (inliers < params_.min_inliers) {
      result.converged = false;
      result.pose = T;
      return result;
    }

    result.mean_sq_error = sq_error_sum / static_cast<double>(inliers);         // trung binh sai so

    // Eigen::Matrix3d::Identity(): ma tran don vi kich thuoc 3x3
    // H+damping de tranh ma tran H bi suy bien, luon kha nghich
    H += Eigen::Matrix3d::Identity() * params_.damping;            
    
    // H.dx = -b
    const Eigen::Vector3d dx = -H.ldlt().solve(b);

    // cap nhat pose
    T.x += dx(0);
    T.y += dx(1);
    T.theta = normalizeAngle(T.theta + dx(2));

    // thu hep dan nguong tuong ung qua tung vong lap (cong thuc 4)
    eps_dist = std::max(params_.eps_dist_end, eps_dist - eps_step);

    if (std::abs(dx(0)) < params_.translation_eps &&
      std::abs(dx(1)) < params_.translation_eps &&
      std::abs(dx(2)) < params_.rotation_eps)
    {
      result.converged = true;
      result.pose = T;
      return result;
    }
  }

  result.converged = (result.num_inliers >= params_.min_inliers);
  result.pose = T;
  return result;
}

}  // namespace plicp_localization
