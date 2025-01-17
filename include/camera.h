#ifndef CAMERA_H
#define CAMERA_H

#include <Eigen/Core>
#include <Eigen/src/Core/util/Memory.h>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

using Matrix34 = Eigen::Matrix<double, 3, 4>;

class Camera {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  /**
   * Constructor for initializing Camera using rectified camera projection
   * matrix data, as given, e.g., in the Kitti dataset.
   *
   * rectified_camera_data gives the 3x4 projection matrix in row-major form,
   * from world coordinates (= left camera coordinates, by convention), to image
   * pixel coordinates.
   */
  Camera(const std::array<double, 12> &rectified_camera_data);

  /**
   * Returns the intrinsic matrix, mapping from camera coordinates to image
   * pixel coordinates.
   */
  Eigen::Matrix3d intrinsic_matrix() const { return intrinsic_matrix_; }

  Eigen::Vector4d intrinsic_data() const { return intrinsic_data_; }

  /**
   * Returns the extrinsic matrix, mapping from world coordinates to camera
   * coordinates, expressed as a Sophus SE3d object.
   */
  Sophus::SE3d extrinsic_matrix_SE3() const { return extrinsic_matrix_; }

  /**
   * Returns the extrinsic matrix, mapping from world coordinates to camera
   * coordinates, expressed as a 3x4 matrix.
   */
  Matrix34 extrinsic_matrix() const { return extrinsic_matrix_.matrix3x4(); }

  /**
   * Returns the baseline, i.e., the distance from the given camera to the left
   * camera.
   */
  double baseline() const { return baseline_; }

  /**
   * Returns the projection matrix, mapping from world coordinates to image
   * pixel coordinates. This is just (intrinsic matrix) * (extrinsic matrix).
   */
  Matrix34 projection_matrix() const { return projection_matrix_; }

private:
  Sophus::SE3d extrinsic_matrix_;
  Eigen::Matrix3d intrinsic_matrix_;
  Eigen::Vector4d intrinsic_data_;
  Matrix34 projection_matrix_;

  double baseline_;
};

#endif
