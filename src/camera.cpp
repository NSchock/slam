#include "camera.h"
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

Camera::Camera(const std::array<double, 12> &rectified_camera_data) {
  // thanks to rectification, rotation is trivial, and rectified_camera_data
  // gives the matrix K[I | t], where I is the identity 3x3 matrix, and t is the
  // 3x1 translation vector translating the camera center (in world coordinates)
  // to the origin (in world coordinates)
  intrinsic_matrix_ << rectified_camera_data[0], rectified_camera_data[1],
      rectified_camera_data[2], rectified_camera_data[4],
      rectified_camera_data[5], rectified_camera_data[6],
      rectified_camera_data[8], rectified_camera_data[9],
      rectified_camera_data[10];

  Eigen::Vector3d translation;
  translation << rectified_camera_data[3], rectified_camera_data[7],
      rectified_camera_data[11];
  // Above gives Kt, where K is the intrinsic matrix, so to get actual
  // translation need to multiply by K^{-1}
  translation = intrinsic_matrix_.inverse() * translation;

  extrinsic_matrix_ = Sophus::SE3d(Sophus::SO3d(), translation);
  baseline_ = translation.norm();

  projection_matrix_ = intrinsic_matrix_ * extrinsic_matrix_.matrix3x4();
}
