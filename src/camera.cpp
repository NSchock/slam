#include "camera.h"
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

Camera::Camera(const std::array<double, 12> &rectified_camera_data) {
  // We could alternatively find the intrinsic and extrinsic matrix and the
  // baseline using OpenCV's cv::decomposeProjectionMatrix. But since the camera
  // data is already rectified, we know that the rotation is trivial, so we
  // already know how to extract the desired data. Also,
  // cv::decomposeProjectionMatrix will return the negative of the desired
  // translation vector, due to rectification
  intrinsic_matrix_ << rectified_camera_data[0], rectified_camera_data[1],
      rectified_camera_data[2], rectified_camera_data[4],
      rectified_camera_data[5], rectified_camera_data[6],
      rectified_camera_data[8], rectified_camera_data[9],
      rectified_camera_data[10];

  Eigen::Vector3d translation;
  translation << rectified_camera_data[3], rectified_camera_data[7],
      rectified_camera_data[11];
  translation = intrinsic_matrix_.inverse() * translation;

  extrinsic_matrix_ = Sophus::SE3d(Sophus::SO3d(), translation);
  baseline_ = translation.norm();
}
