#ifndef CAMERA_H
#define CAMERA_H

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

class Camera {
public:
  Eigen::Matrix3d intrinsic_matrix_;
  Sophus::SE3d extrinsic_matrix_;
  double baseline_; // the distance from the given camera to the left camera

  Camera(const std::array<double, 12> &rectified_camera_data);

  Camera(const Eigen::Matrix3d &intrinsic_matrix,
         const Sophus::SE3d &extrinsic_matrix)
      : intrinsic_matrix_(intrinsic_matrix),
        extrinsic_matrix_(extrinsic_matrix) {}

  Eigen::Matrix<double, 3, 4> projection_matrix() const {
    return intrinsic_matrix_ * extrinsic_matrix_.matrix3x4();
  }
};

#endif
