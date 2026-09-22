#include "plicp_localization/occupancy_grid_map_model.hpp"

namespace plicp_localization
{

bool OccupancyGridMapModel::worldToGrid(double wx, double wy, int & gx, int & gy) const
{
  const double res = grid_.info.resolution;
  const double ox = grid_.info.origin.position.x;
  const double oy = grid_.info.origin.position.y;

  // wx, wy, ox, oy: he met
  gx = static_cast<int>(std::floor((wx - ox) / res));     // dia chi o grid tuong ung voi so met wx, wy quy doi
  gy = static_cast<int>(std::floor((wy - oy) / res));
  return isValidCell(gx, gy);
}

// dam may diem goc (tap Q)
bool OccupancyGridMapModel::gridToWorld(int gx, int gy, double & wx, double & wy) const
{
  if (!isValidCell(gx, gy)) {
    return false;
  }
  const double res = grid_.info.resolution;
  const double ox = grid_.info.origin.position.x;
  const double oy = grid_.info.origin.position.y;
  wx = ox + (gx + 0.5) * res;                             
  wy = oy + (gy + 0.5) * res;
  return true;
}

bool OccupancyGridMapModel::isValidCell(int gx, int gy) const
{
  return gx >= 0 && gy >= 0 &&
         gx < static_cast<int>(grid_.info.width) &&
         gy < static_cast<int>(grid_.info.height);
}

bool OccupancyGridMapModel::isOccupied(int gx, int gy) const
{
  if (!isValidCell(gx, gy)) {
    return false;
  }
  const int idx = gy * static_cast<int>(grid_.info.width) + gx;
  const int8_t v = grid_.data[idx];
  return v >= occ_threshold_;
}

// tim qi (diem tuong ung tren ban do) va uoc luong ni (phap tuyen cuc bo)
// max_dist: ban kinh tim kiem pmap_i tuong ung voi pixel na tren map
// query_point -> chuyen sang grid index -> ring search(tim o gan nhat) -> tim neighbor -> tinh phap tuyen dung pca
bool OccupancyGridMapModel::findCorrespondence(
  const Eigen::Vector2d & query_point,
  double max_dist,
  Eigen::Vector2d & corr_point,
  Eigen::Vector2d & corr_normal) const
{
  if (!has_map_) {
    return false;
  }

  // neu pmap_i nam ngoai ban do -> ko the tim diem tuong ung -> false, (qx, qy: don vi pixel)
  int qx, qy;   
  if (!worldToGrid(query_point.x(), query_point.y(), qx, qy)) {
    return false;
  }

  const double res = grid_.info.resolution;

  // doi max_dist he met sang so o grid
  const int max_ring = std::max(1, static_cast<int>(std::ceil(max_dist / res)));      // ceil: lam tron 1 so len so nguyen lon hon so do

  // Tìm kiếm theo vòng (ring search) mở rộng dần trên lưới bản đồ để tìm ô
  // "chiếm dụng" gần nhất trong phạm vi max_dist.
  bool found = false;
  int best_gx = 0, best_gy = 0;
  double best_dist_sq = max_dist * max_dist;

  for (int ring = 0; ring <= max_ring; ++ring) {
    for (int dx = -ring; dx <= ring; ++dx) {
      for (int dy = -ring; dy <= ring; ++dy) {
        if (std::max(std::abs(dx), std::abs(dy)) != ring) {
          continue;  // chỉ xét viền ngoài của ring hiện tại
        }

        // neu gx, gy co vat can(occupancy) chuyen toa do ve met va tinh khoang cach Euclid
        const int gx = qx + dx;
        const int gy = qy + dy;
        if (!isOccupied(gx, gy)) {
          continue;
        }
        double wx, wy;      // don vi: met
        gridToWorld(gx, gy, wx, wy);          // dam may diem tap Q

        // tinh khoang cach euclid gan nhat de xet o grid nao trong map gan nhat voi pmap_i
        const double ddx = wx - query_point.x();
        const double ddy = wy - query_point.y();
        const double dsq = ddx * ddx + ddy * ddy;
        if (dsq < best_dist_sq) {
          best_dist_sq = dsq;
          best_gx = gx;
          best_gy = gy;
          found = true;
        }
      }
    }
    // Nếu đã có ứng viên và vòng tiếp theo chắc chắn xa hơn khoảng cách tốt
    // nhất hiện có, có thể dừng sớm.
    if (found && static_cast<double>(ring) * res > std::sqrt(best_dist_sq)) {
      break;
    }
  }

  if (!found) {
    return false;
  }


  double best_wx, best_wy;                                      // toa do he met tu pixel trong grid nam gan nhat voi tia laser dang xet
  gridToWorld(best_gx, best_gy, best_wx, best_wy);
  corr_point = Eigen::Vector2d(best_wx, best_wy);               // met

  // Ước lượng pháp tuyến cục bộ bằng PCA trên các ô chiếm dụng lân cận
  // (mô phỏng metric "point-to-line" thay vì chỉ so khớp point-to-point).
  Eigen::Vector2d mean = Eigen::Vector2d::Zero();                 // ma tran 2 hang 1 cot
  std::vector<Eigen::Vector2d> neighbors;
  for (int dx = -normal_window_; dx <= normal_window_; ++dx) {
    for (int dy = -normal_window_; dy <= normal_window_; ++dy) {
      const int gx = best_gx + dx;
      const int gy = best_gy + dy;
      if (!isOccupied(gx, gy)) {
        continue;
      }
      double wx, wy;
      gridToWorld(gx, gy, wx, wy);
      neighbors.emplace_back(wx, wy);           // emplace_back: tao 1 doi tuong (wx, wy) vao cuoi 1 vecto  
      mean += Eigen::Vector2d(wx, wy);          // cong tong cac pixel co chua occupancy de tinh centroid (tam khoi luong) = mean / neighbors.size()
    }
  }

  if (neighbors.size() < 3) {
    // Không đủ điểm lân cận để xác định một đường thẳng cục bộ:
    // dùng tạm metric point-to-point (pháp tuyến = hướng từ điểm tương ứng về query).
    const Eigen::Vector2d dir = query_point - corr_point;
    const double norm = dir.norm();             // tong binh phuong
    corr_normal = (norm > 1e-6) ? (dir / norm) : Eigen::Vector2d(1.0, 0.0);
    return true;
  }

  mean /= static_cast<double>(neighbors.size());
  Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
  for (const auto & pt : neighbors) {
    const Eigen::Vector2d d = pt - mean;        
    cov += d * d.transpose();
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
  // Vector riêng ứng với trị riêng NHỎ NHẤT chính là pháp tuyến của đường biên cục bộ
  corr_normal = solver.eigenvectors().col(0).normalized();

  return true;
}

double OccupancyGridMapModel::raycast(
  const Pose2D & pose,
  double global_angle,
  double max_range) const
{
  if (!has_map_) {
    return max_range;
  }

  const double res = grid_.info.resolution;
  const double step = std::max(res * 0.5, 0.01);
  const double dx = std::cos(global_angle);
  const double dy = std::sin(global_angle);

  for (double t = 0.0; t <= max_range; t += step) {
    const double wx = pose.x + t * dx;
    const double wy = pose.y + t * dy;
    int gx, gy;
    if (!worldToGrid(wx, wy, gx, gy)) {
      return max_range;  // ra khỏi vùng bản đồ -> coi như không có vật cản
    }
    if (isOccupied(gx, gy)) {
      return t;
    }
  }
  return max_range;
}

}  // namespace plicp_localization
