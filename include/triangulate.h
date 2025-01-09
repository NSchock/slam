#ifndef TRIANGULATE_H
#define TRIANGULATE_H

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/src/Core/util/Constants.h>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/types.hpp>

using Mat34 = Eigen::Matrix<double, 3, 4>;
using Point2 = cv::Point2f;

/**
 * Triangulate image points from two cameras to obtain the corresponding 3d
 * point.
 *
 * TODO: verify this works correctly---if I pass in full projection matrices
 * (intrinsic * extrinsic), and points calculated on images, does it give
 * correct answer? Or should I convert as in slambook?
 * Also should I check result is valid, e.g., that z coordinate is positive?
 */
inline std::pair<Eigen::Vector4d, bool>
triangulate_points(const Mat34 &proj_left, const Mat34 &proj_right,
                   const Point2 &point_left, const Point2 &point_right) {
  Eigen::Matrix4d A;
  Eigen::Vector4d b = Eigen::Vector4d::Zero();
  A.row(0) = point_left.y * proj_left.row(2) - proj_left.row(1);
  A.row(1) = proj_left.row(0) - point_left.x * proj_left.row(2);
  A.row(2) = point_right.y * proj_right.row(2) - proj_right.row(1);
  A.row(3) = proj_right.row(0) - point_right.x * proj_right.row(2);

  auto svd = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);

  auto triangulated_point =
      (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();
  // std::cout << "triangulated point: " << triangulated_point << "\n";
  // std::cout << "homogenous: " << svd.matrixV().col(3) << "\n";
  // std::cout << "V: " << svd.matrixV() << "\n";
  // std::cout << "singular values: " << svd.singularValues() << "\n";
  //
  // okay, here's where the problem occurs: goes straight, then turns right,
  // then turns left. At the right turn, it messes up but corrects itself. At
  // the left turn, it irreparably messes up.
  // why is my triangulated point the negative of opencv's?
  return {svd.matrixV().col(3),
          triangulated_point.z() > 0 &&
              svd.singularValues()[3] / svd.singularValues()[2] < 1e-2};
}

#endif
