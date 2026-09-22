#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "plicp_localization/point_to_line_icp.hpp"

namespace plicp_localization
{

/// Triển khai MapModel dựa trên nav_msgs::msg::OccupancyGrid.
/// - findCorrespondence(): tìm ô "chiếm dụng" gần nhất + ước lượng pháp tuyến
///   cục bộ bằng PCA trên các ô lân cận (dùng cho bước PLICP).
/// - raycast(): dò tia trên bản đồ để tính khoảng cách "ảo" (virtual measurement),
///   dùng cho bước lọc nhiễu theo công thức (2)-(4) của bài báo.
class OccupancyGridMapModel : public MapModel
{
public:
  OccupancyGridMapModel() = default;

  void setMap(const nav_msgs::msg::OccupancyGrid & grid)
  {
    grid_ = grid;
    has_map_ = true;
  }

  bool hasMap() const {return has_map_;}

  bool findCorrespondence(
    const Eigen::Vector2d & query_point,
    double max_dist,
    Eigen::Vector2d & corr_point,
    Eigen::Vector2d & corr_normal) const override;

  /// Tính khoảng cách "ảo" (virtual measurement) từ pose theo hướng global_angle,
  /// bằng cách dò dọc theo tia cho tới khi gặp ô chiếm dụng hoặc vượt max_range.
  /// Tương ứng việc dựng lại (xoi, yoi) từ dvi trong công thức (2)-(3) của bài báo.
  double raycast(
    const Pose2D & pose,
    double global_angle,
    double max_range) const;

private:
  bool worldToGrid(double wx, double wy, int & gx, int & gy) const;
  bool gridToWorld(int gx, int gy, double & wx, double & wy) const;
  bool isOccupied(int gx, int gy) const;
  bool isValidCell(int gx, int gy) const;

  nav_msgs::msg::OccupancyGrid grid_;
  bool has_map_{false};

  // Giá trị ô >= ngưỡng này được coi là "chiếm dụng" (occupied)
  int8_t occ_threshold_{100};

  // Bán kính (theo ô, chuẩn Chebyshev) dùng để ước lượng pháp tuyến cục bộ qua PCA
  int normal_window_{2};
};

}  // namespace plicp_localization
