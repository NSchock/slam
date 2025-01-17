#ifndef TRIANGULATE_H
#define TRIANGULATE_H

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/src/Core/util/Constants.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/types.hpp>

using Mat34 = Eigen::Matrix<double, 3, 4>;
using Point2 = cv::Point2f;

/**
 * Triangulate image points from two cameras to obtain the corresponding 3d
 * point.
 */
inline std::pair<Eigen::Vector3d, bool>
triangulate_points(const Mat34 &proj_left, const Mat34 &proj_right,
                   const Point2 &point_left, const Point2 &point_right) {
  Eigen::Matrix4d A;
  A.row(0) = point_left.y * proj_left.row(2) - proj_left.row(1);
  A.row(1) = proj_left.row(0) - point_left.x * proj_left.row(2);
  A.row(2) = point_right.y * proj_right.row(2) - proj_right.row(1);
  A.row(3) = proj_right.row(0) - point_right.x * proj_right.row(2);

  auto svd = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);

  auto triangulated_point =
      (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();
  // TODO: why is my triangulated_point the negative of OpenCV's?

  return {triangulated_point,
          triangulated_point.z() > 0 &&
              svd.singularValues()[3] / svd.singularValues()[2] < 1e-2};
  // return {triangulated_point, true};
}

#endif
